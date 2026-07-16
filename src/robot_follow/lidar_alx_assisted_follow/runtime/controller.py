"""Cloud-driven control loop for LiDAR + ALX Assisted Follow.

Core design:
- Strategy is cloud-driven: each distinct cloud sequence is processed at most once
- 40 Hz timeout wake performs safety checks only, never recomputes strategy from old cloud
- Overload is latest-only: if several clouds arrive during one strategy pass,
  process the newest next and discard intermediate unprocessed frames
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from typing import Deque, List, Optional, Protocol, Union

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import PrototypeConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    CloudSnapshot,
    ConfidenceResult,
    GatedTargetObservation,
    MotionIntent,
    PreprocessedCloud,
    StopKind,
    StopReason,
    StopSnapshot,
    StrategyResult,
)
from robot_follow.lidar_alx_assisted_follow.motion.apf import APFInput, compute_apf
from robot_follow.lidar_alx_assisted_follow.motion.dispatch import (
    G1BridgeProtocol,
    dispatch_strategy_or_stop,
)
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.csv_log import PrototypeCSVLogger
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEvent,
    ControlEventInbox,
    ControlEventKind,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot, ALXSnapshotBuffer
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import (
    ALXPositionBuffer,
    ALXPositionSample,
)
from robot_follow.lidar_alx_assisted_follow.sensors.wristband import (
    WristbandStatus,
    WristbandTopicState,
)
from robot_follow.lidar_alx_assisted_follow.sensors.cloud import SequencedFastLIOPointCloudSource
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)
from robot_follow.lidar_alx_assisted_follow.tracking.preprocessing import preprocess_tracking_cloud
from robot_follow.lidar_alx_assisted_follow.tracking.target import (
    LockState,
    TargetLockManager,
    TargetLockSnapshot,
)
from robot_follow.ao.live.radar_service_manager import RadarServiceManager


class ALXReaderProtocol(Protocol):
    """Protocol for filtered ALX position and wristband ROS2 topics."""

    def latest_snapshot(
        self,
    ) -> Optional[Union[ALXSnapshot, ALXPositionSample]]:
        """Get the latest received ALX position sample."""
        ...

    def is_ok(self) -> bool:
        """Check whether the position topic source is healthy and fresh."""
        ...

    def wristband_status(self) -> WristbandStatus:
        """Get the latest explicit wristband control status."""
        ...

    def is_wristband_status_fresh(self, now: Optional[float] = None) -> bool:
        """Check whether wristband control status has a fresh heartbeat."""
        ...


@dataclass
class RuntimeState:
    """Mutable runtime state for control loop."""

    # Processed sequence tracking
    last_processed_sequence: int = 0

    # Strategy results for logging/status
    last_strategy_result: Optional[StrategyResult] = None

    # Confidence result
    last_confidence: Optional[ConfidenceResult] = None

    # Target observation
    last_observation: Optional[GatedTargetObservation] = None

    # Cloud age
    last_cloud_age_ms: float = 0.0

    # ALX age
    last_alx_age_ms: float = 0.0
    last_alx_body_xy: Optional[np.ndarray] = None
    last_alx_stamp_s: Optional[float] = None
    last_alx_offset_ms: Optional[float] = None
    last_alx_synchronized: bool = False

    # Freshness anchor for watchdog cloud-age checks.
    last_cloud_received_at_s: Optional[float] = None

    # Last explicit ALX control state observed from the reliable wristband
    # topic.  This lets OFF/Fault stop managed services even if the lifecycle
    # was already in WRISTBAND_OFF because of an earlier partial startup.
    last_wristband_control_state: Optional[WristbandTopicState] = None


@dataclass
class LidarALXAssistedFollowRuntime:
    """Cloud-driven control loop runtime.

    This is the main orchestrator for the LiDAR + ALX Assisted Follow prototype.

    Threading:
    - Web thread: owns WebSocket connections, pushes control events
    - Control thread: processes clouds, runs strategy, dispatches to G1

    Safety:
    - StopLatch: any thread can request stop, only control thread can clear
    - HARD stops: user_stop, controller_disconnected, wristband_off, etc.
    - SOFT stops: confidence_low, alx_stale (can auto-recover)
    """

    config: PrototypeConfig
    cloud_source: SequencedFastLIOPointCloudSource
    alx_reader: ALXReaderProtocol
    alx_buffer: Union[ALXSnapshotBuffer, ALXPositionBuffer]
    lifecycle: WristbandLifecycleGuardian
    radar_services: RadarServiceManager
    target_manager: TargetLockManager
    stop_latch: StopLatch
    event_inbox: ControlEventInbox
    latest_status: LatestStatus
    limiter: CommandSlewRateLimiter
    bridge: G1BridgeProtocol
    csv_logger: PrototypeCSVLogger

    # Internal state
    _state: RuntimeState = field(default_factory=RuntimeState)
    _last_strategy_time: float = 0.0
    _last_safety_stop_reason: Optional[str] = None
    _bridge_initialized: bool = False
    _running: bool = False
    _control_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the control loop thread."""
        if self._running:
            return

        self._running = True
        self._control_thread = threading.Thread(
            target=self.run_control_loop,
            name="LidarALXControl",
            daemon=True,
        )
        self._control_thread.start()

    def stop(self) -> None:
        """Stop the control loop thread."""
        self._running = False
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None

    def set_alx_reader(self, reader: ALXReaderProtocol) -> None:
        """Install the live reader before the control loop starts."""
        self.alx_reader = reader

    def run_control_loop(self) -> None:
        """Run the control loop in the caller-owned control thread."""
        self._running = True
        self._control_loop()

    def _control_loop(self) -> None:
        """Main control loop.

        Runs in dedicated control thread.

        Loop:
        1. Wait for cloud event or 25 ms timeout
        2. Drain control events
        3. Sample/deduplicate ALX
        4. Update lifecycle
        5. Process newest unseen cloud (if any)
        6. Run watchdog safety checks
        7. Call bridge.tick(now)
        """
        self.stop_latch.bind_control_thread(threading.get_ident())
        watchdog_interval_s = 1.0 / self.config.runtime.watchdog_hz

        while self._running:
            now = time.monotonic()

            # Step 1: Wait for cloud event or timeout
            cloud_snapshot = self.cloud_source.wait_for_update(
                timeout_s=watchdog_interval_s
            )

            # Step 2: Drain control events
            self._drain_control_events()

            # Step 3: Sample/deduplicate ALX
            alx_snapshot = self._sample_alx(now)

            # Step 4: Update lifecycle
            self._update_lifecycle(alx_snapshot, cloud_snapshot is not None, now)

            # Step 5: Process newest cloud (if new)
            if cloud_snapshot is not None:
                # Only process if sequence > last_processed_sequence
                if cloud_snapshot.sequence > self._state.last_processed_sequence:
                    self._process_cloud(cloud_snapshot, alx_snapshot, now)

            # Step 6: Run watchdog safety checks
            self._run_watchdog_checks(now)

            # Step 7: Call bridge.tick
            self.bridge.tick(now)

    def _drain_control_events(self) -> None:
        """Drain all pending control events.

        Process select_target, start_follow, stop_follow, controller_disconnect.
        """
        while not self.event_inbox.empty:
            event = self.event_inbox.pop()
            if event is None:
                break

            self._process_control_event(event)

    def _process_control_event(self, event: ControlEvent) -> None:
        """Process a single control event.

        Parameters
        ----------
        event : ControlEvent
            Control event from Web.
        """
        if event.kind == ControlEventKind.SELECT_TARGET:
            # ALX is the follow target. Browser clicks never retarget motion.
            return

        elif event.kind == ControlEventKind.START_FOLLOW:
            # Start following
            # Requires: valid candidate, controller ownership, ready lifecycle,
            # fresh ALX, C >= start
            self._handle_start_follow()

        elif event.kind == ControlEventKind.STOP_FOLLOW:
            # Stop following
            # Request HARD stop, preserve target
            self.stop_latch.request("user_stop", StopKind.HARD)
            self.target_manager.stop_follow()

        elif event.kind == ControlEventKind.CONTROLLER_DISCONNECT:
            # Controller disconnected
            # Request HARD stop, unlock
            self.stop_latch.request("controller_disconnected", StopKind.HARD)
            self.target_manager.unlock()

    def _handle_start_follow(self) -> None:
        """Handle start_follow command.

        Requirements:
        - Valid ALX-derived candidate target
        - Controller ownership
        - SENSORS_READY lifecycle
        - Fresh synchronized ALX
        - C >= 0.80
        """
        # Check target state
        target_snapshot = self.target_manager.snapshot()
        if target_snapshot.state != LockState.CANDIDATE:
            return

        # Check lifecycle
        if self.lifecycle.state != LifecycleState.SENSORS_READY:
            return

        # Check ALX fresh
        if self._state.last_confidence is None or not self._state.last_confidence.alx_fresh:
            return

        # Check confidence
        if self._state.last_confidence.total < self.config.confidence.start:
            return

        # Clear latch and start following
        if self._state.last_strategy_result is not None:
            start_after_sequence = self._state.last_strategy_result.sequence
        else:
            start_after_sequence = self._state.last_processed_sequence

        self.stop_latch.clear_for_start(start_after_sequence)
        self.target_manager.start()

    def _sample_alx(
        self,
        now: float,
    ) -> Optional[Union[ALXSnapshot, ALXPositionSample]]:
        """Sample and deduplicate ALX data.

        Parameters
        ----------
        now : float
            Current monotonic time.

        Returns
        -------
        ALXSnapshot or None
            Latest ALX snapshot, or None if unavailable.
        """
        snapshot = self.alx_reader.latest_snapshot()

        if snapshot is not None:
            self.alx_buffer.append(snapshot, now)

        return snapshot

    def _update_lifecycle(
        self,
        alx_snapshot: Optional[Union[ALXSnapshot, ALXPositionSample]],
        cloud_received: bool,
        now: float,
    ) -> None:
        """Update sensor lifecycle.

        Parameters
        ----------
        alx_snapshot : ALXSnapshot or None
            Latest ALX sample.
        cloud_received : bool
            Whether a cloud was received.
        now : float
            Current monotonic time.
        """
        # The dedicated control topic, not position arrival, owns the wristband
        # lifecycle.  PointStamped intentionally contains no tag identifier;
        # tag ID 0 preserves the existing single-tag topic-mode session model.
        wristband_status = self.alx_reader.wristband_status()
        status_fresh = self.alx_reader.is_wristband_status_fresh(now)
        control_state = (
            wristband_status.state
            if status_fresh
            else WristbandTopicState.SERIAL_FAULT
        )
        control_state_changed = (
            control_state != self._state.last_wristband_control_state
        )
        self._state.last_wristband_control_state = control_state
        wristband_on = (
            status_fresh and control_state == WristbandTopicState.ON
        )
        tag_id = 0 if wristband_on else None
        serial_ok = (
            status_fresh
            and control_state != WristbandTopicState.SERIAL_FAULT
        )

        # Tick lifecycle
        previous_state = self.lifecycle.state
        new_state, should_stop_services, transition_reason = self.lifecycle.tick(
            wristband_on=wristband_on,
            tag_id=tag_id,
            serial_ok=serial_ok,
            cloud_received=cloud_received,
            now=now,
        )

        if (
            previous_state == LifecycleState.WRISTBAND_OFF
            and new_state == LifecycleState.STARTING_SENSORS
        ):
            self.radar_services.ensure_running()

        # Handle lifecycle transitions
        # An explicit OFF is safety-critical in its own right.  Do not rely
        # solely on a lifecycle transition: a runtime started while sensors
        # were already running can otherwise remain in WRISTBAND_OFF and miss
        # the command to terminate the processes it owns.
        forced_stop_reason: Optional[str] = None
        if control_state_changed:
            if control_state == WristbandTopicState.OFF:
                forced_stop_reason = StopReason.WRISTBAND_OFF.value
            elif control_state in (
                WristbandTopicState.UNKNOWN,
                WristbandTopicState.SERIAL_FAULT,
            ):
                forced_stop_reason = StopReason.SERIAL_ERROR.value

        if should_stop_services or forced_stop_reason is not None:
            self.radar_services.stop()
            # Request stop when the lifecycle detects a safety transition.
            if transition_reason in ("wristband_off_or_serial_error", "ready_timeout", "tag_changed"):
                if control_state == WristbandTopicState.OFF:
                    stop_reason = StopReason.WRISTBAND_OFF.value
                elif transition_reason == "ready_timeout":
                    stop_reason = StopReason.READY_TIMEOUT.value
                elif transition_reason == "tag_changed":
                    stop_reason = "tag_changed"
                else:
                    stop_reason = StopReason.SERIAL_ERROR.value
                self.stop_latch.request(stop_reason, StopKind.HARD)
                self.target_manager.unlock()
            elif forced_stop_reason is not None:
                self.stop_latch.request(forced_stop_reason, StopKind.HARD)
                self.target_manager.unlock()

        if new_state == LifecycleState.SENSORS_READY and not self._bridge_initialized:
            try:
                self.bridge.init()
                self._bridge_initialized = True
            except Exception:
                self.stop_latch.request("bridge_fault", StopKind.HARD)

    def _match_alx_for_cloud(
        self,
        cloud: CloudSnapshot,
        now: float,
    ) -> tuple[Optional[np.ndarray], bool]:
        """Match one already-filtered ALX sample to this cloud's sensor time."""
        best_snapshot = self.alx_buffer.for_cloud(
            cloud.sensor_timestamp_s,
            self.lifecycle.session_tag_id,
        )

        alx_body_xy: Optional[np.ndarray] = None
        alx_fresh = False
        alx_age_ms = 0.0
        alx_stamp_s: Optional[float] = None
        alx_offset_ms: Optional[float] = None

        if best_snapshot is not None:
            alx_body_xy = self.alx_buffer.transform_to_body(best_snapshot)
            alx_age_s = max(0.0, now - best_snapshot.received_at)
            alx_age_ms = alx_age_s * 1000.0
            # The ROS topic sample has an acquisition/receive stamp mapped to
            # ROS time.  Legacy raw snapshots use received_at as a fallback so
            # existing pure runtime tests remain valid during this migration.
            alx_stamp_s = float(
                getattr(best_snapshot, "timestamp_s", best_snapshot.received_at)
            )
            alx_offset_ms = (cloud.sensor_timestamp_s - alx_stamp_s) * 1000.0
            alx_fresh = (
                alx_body_xy is not None
                and alx_age_s < self.config.alx.stale_timeout_s
            )
            if alx_fresh:
                distance_m = float(np.linalg.norm(alx_body_xy))
                alx_fresh = (
                    self.config.alx.min_distance_m <= distance_m
                    <= self.config.alx.max_distance_m
                )

        self._state.last_alx_age_ms = alx_age_ms
        self._state.last_alx_body_xy = alx_body_xy if alx_fresh else None
        self._state.last_alx_stamp_s = alx_stamp_s
        self._state.last_alx_offset_ms = alx_offset_ms
        self._state.last_alx_synchronized = alx_fresh
        return self._state.last_alx_body_xy, alx_fresh

    def _process_cloud(
        self,
        cloud: CloudSnapshot,
        alx_snapshot: Optional[Union[ALXSnapshot, ALXPositionSample]],
        now: float,
    ) -> None:
        """Process a cloud (run strategy).

        This is the core strategy execution for one cloud sequence.

        Parameters
        ----------
        cloud : CloudSnapshot
            Cloud snapshot.
        alx_snapshot : ALXSnapshot or None
            Latest ALX sample.
        now : float
            Current monotonic time.
        """
        # Mark sequence as processed
        self._state.last_processed_sequence = cloud.sequence
        self._state.last_cloud_received_at_s = cloud.received_at_s

        # Compute cloud age
        cloud_age_s = now - cloud.received_at_s
        self._state.last_cloud_age_ms = cloud_age_s * 1000.0

        # Check cloud staleness
        if cloud_age_s > self.config.sensors.max_cloud_age_s:
            self.stop_latch.request("cloud_stale", StopKind.HARD)
            self._log_strategy_zero(cloud, "cloud_stale", now)
            return

        # Match before any early strategy return so decision diagnostics and
        # Web visualization always describe the same cloud/ALX pair.
        alx_body_xy, alx_fresh = self._match_alx_for_cloud(cloud, now)

        # Check lifecycle
        if self.lifecycle.state != LifecycleState.SENSORS_READY:
            self._log_strategy_zero(cloud, "lifecycle_not_ready", now)
            return

        # Preprocess cloud
        preprocessed = preprocess_tracking_cloud(
            cloud.points_xyz,
            self.config.target,
            self.config.calibration,
        )

        # A hard stop blocks all tracking and motion work, but the Web monitor
        # must retain the current LiDAR view so an operator can diagnose and
        # explicitly restart from an informed state.
        latch_snapshot = self.stop_latch.snapshot()
        if latch_snapshot.requested and latch_snapshot.kind == StopKind.HARD:
            reason = latch_snapshot.reason or "stop"
            self._log_strategy_zero(cloud, reason, now, preprocessed)
            return

        compute_start = time.monotonic()

        # The matched ALX sample is the follow target. LiDAR remains an
        # obstacle source: this pass neither selects nor tracks a person from
        # the cloud, and it runs only because this cloud is new.
        if not alx_fresh or alx_body_xy is None:
            self.stop_latch.request("alx_stale", StopKind.SOFT)
            self._state.last_confidence = ConfidenceResult(
                c_t=0.0, c_g=0.0, c_a=0.0, total=0.0,
                alx_fresh=False, reason="alx_stale",
            )
            self._state.last_observation = None
            self._log_strategy_zero(cloud, "alx_stale", now, preprocessed)
            return

        self.target_manager.set_external_target(
            (float(alx_body_xy[0]), float(alx_body_xy[1])), now,
        )
        target_snapshot = self.target_manager.snapshot()
        follow_center_xy = self.target_manager.follow_center_xy
        self._state.last_observation = None
        confidence = ConfidenceResult(
            c_t=1.0, c_g=1.0, c_a=1.0, total=1.0,
            alx_fresh=True, reason="alx_target_matched",
        )
        self._state.last_confidence = confidence

        latch_snapshot = self.stop_latch.snapshot()
        if (
            latch_snapshot.requested
            and latch_snapshot.kind == StopKind.SOFT
            and latch_snapshot.reason == StopReason.ALX_STALE.value
        ):
            self.stop_latch.clear_soft_recovery()

        if target_snapshot.center_xy is None or follow_center_xy is None:
            self._log_strategy_zero(cloud, "alx_stale", now, preprocessed)
            return

        # A new ALX position creates a candidate, never autonomous motion.
        # The operator must still send Start; until then each cloud pass keeps
        # the bridge at zero velocity.
        if target_snapshot.state != LockState.FOLLOWING:
            self.stop_latch.request("motion_not_allowed", StopKind.SOFT)
            self._log_strategy_zero(cloud, "motion_not_allowed", now, preprocessed)
            return

        target_mask = self._alx_target_mask(preprocessed, alx_body_xy)

        # Run APF
        apf_input = APFInput(
            cloud=preprocessed,
            target_center_xy=follow_center_xy,
            target_mask=target_mask,
            confidence=confidence,
            sequence=cloud.sequence,
            alx_body_xy=alx_body_xy,
        )

        apf_result = compute_apf(apf_input, self.config.apf, self.config.output_motion)

        compute_ms = (time.monotonic() - compute_start) * 1000.0
        strategy_result = StrategyResult(
            sequence=cloud.sequence,
            apf_requested=apf_result.intent,
            dispatched=apf_result.intent,
            safety_complete=True,
            stop_reason=apf_result.stop_reason,
            compute_ms=compute_ms,
        )

        # NumPy work cannot be preempted, but a result that exceeded the
        # configured budget must never become a motion command.
        if compute_ms > self.config.runtime.max_strategy_process_ms:
            timeout_intent = MotionIntent(
                sequence=cloud.sequence,
                vx=0.0,
                vy=0.0,
                omega=0.0,
                reason=StopReason.STRATEGY_TIMEOUT.value,
                generated_at_s=now,
            )
            strategy_result = StrategyResult(
                sequence=cloud.sequence,
                apf_requested=timeout_intent,
                dispatched=timeout_intent,
                safety_complete=True,
                stop_reason=StopReason.STRATEGY_TIMEOUT,
                compute_ms=compute_ms,
            )

        # Dispatch to G1
        dispatch_start = time.monotonic()
        if (
            strategy_result.stop_reason is None
            and dispatch_start - cloud.received_at_s
            > self.config.runtime.max_strategy_age_s
        ):
            stale_intent = MotionIntent(
                sequence=cloud.sequence,
                vx=0.0,
                vy=0.0,
                omega=0.0,
                reason=StopReason.STRATEGY_STALE.value,
                generated_at_s=dispatch_start,
            )
            strategy_result = StrategyResult(
                sequence=cloud.sequence,
                apf_requested=stale_intent,
                dispatched=stale_intent,
                safety_complete=True,
                stop_reason=StopReason.STRATEGY_STALE,
                compute_ms=compute_ms,
            )
        latch_snapshot = self.stop_latch.snapshot()

        dispatch_result = dispatch_strategy_or_stop(
            strategy=strategy_result,
            latch_snapshot=latch_snapshot,
            bridge=self.bridge,
            limiter=self.limiter,
            now_s=now,
            latch=self.stop_latch,
        )

        dispatch_ms = (time.monotonic() - dispatch_start) * 1000.0
        strategy_result = replace(
            strategy_result,
            dispatched=dispatch_result.dispatched,
        )
        self._state.last_strategy_result = strategy_result

        # Log to CSV
        self.csv_logger.log_strategy(
            monotonic_s=now,
            cloud_sequence=cloud.sequence,
            cloud_age_ms=self._state.last_cloud_age_ms,
            alx_age_ms=self._state.last_alx_age_ms,
            lifecycle_state=self.lifecycle.state.value,
            lock_state=target_snapshot.state.value,
            target_center_x=target_snapshot.center_xy[0] if target_snapshot.center_xy else float("nan"),
            target_center_y=target_snapshot.center_xy[1] if target_snapshot.center_xy else float("nan"),
            target_support_count=0,
            confidence=confidence,
            stop_reason=(
                dispatch_result.stop_reason
                or (
                    strategy_result.stop_reason.value
                    if strategy_result.stop_reason is not None
                    else None
                )
            ),
            apf_requested_vx=strategy_result.apf_requested.vx,
            apf_requested_vy=strategy_result.apf_requested.vy,
            apf_requested_omega=strategy_result.apf_requested.omega,
            dispatched_vx=strategy_result.dispatched.vx,
            dispatched_vy=strategy_result.dispatched.vy,
            dispatched_omega=strategy_result.dispatched.omega,
            strategy_compute_ms=strategy_result.compute_ms,
            g1_dispatch_ms=dispatch_ms,
            bridge_state="initialized",
            sdk_error=None,
            calibration_status="provisional",
        )
        self._publish_status(cloud, preprocessed)

    def _alx_target_mask(
        self, cloud: PreprocessedCloud, alx_body_xy: np.ndarray
    ) -> np.ndarray:
        """Exclude LiDAR returns spatially coincident with the ALX tag."""
        mask = np.zeros(cloud.points_xyz.shape[0], dtype=bool)
        if cloud.points_xyz.size == 0:
            return mask
        radius = self.config.target.web_pick_radius_m
        mask[:] = np.hypot(
            cloud.points_xyz[:, 0] - alx_body_xy[0],
            cloud.points_xyz[:, 1] - alx_body_xy[1],
        ) <= radius
        return mask

    def _log_strategy_zero(
        self,
        cloud: CloudSnapshot,
        reason: str,
        now: float,
        preprocessed: Optional[PreprocessedCloud] = None,
    ) -> None:
        """Log zero-motion strategy result.

        Parameters
        ----------
        cloud : CloudSnapshot
            Cloud snapshot.
        reason : str
            Stop reason.
        now : float
            Current monotonic time.
        """
        zero_intent = MotionIntent(
            sequence=cloud.sequence,
            vx=0.0,
            vy=0.0,
            omega=0.0,
            reason=reason,
            generated_at_s=now,
        )

        strategy_result = StrategyResult(
            sequence=cloud.sequence,
            apf_requested=zero_intent,
            dispatched=zero_intent,
            safety_complete=True,
            stop_reason=StopReason(reason) if reason in [e.value for e in StopReason] else None,
            compute_ms=0.0,
        )

        self._state.last_strategy_result = strategy_result
        self._publish_status(cloud, preprocessed)

    def _run_watchdog_checks(self, now: float) -> None:
        """Run watchdog safety checks.

        These checks run at 40 Hz regardless of cloud arrival.

        Checks:
        - Stop latch state
        - Cloud staleness
        - ALX staleness
        - Lifecycle state
        """
        # Check stop latch
        latch_snapshot = self.stop_latch.snapshot()
        if latch_snapshot.requested:
            # Dispatch stop
            self._dispatch_stop_from_latch(latch_snapshot)
            return

        # Check cloud staleness after sensors have reached READY.  The watchdog
        # deliberately performs this safety check without re-running strategy.
        if (
            self.lifecycle.state == LifecycleState.SENSORS_READY
            and self._state.last_cloud_received_at_s is not None
            and now - self._state.last_cloud_received_at_s
            > self.config.sensors.max_cloud_age_s
        ):
            self.stop_latch.request("cloud_stale", StopKind.HARD)
            self._dispatch_stop_from_latch(self.stop_latch.snapshot())
            return

        # Check ALX staleness
        if not self._state.last_confidence or not self._state.last_confidence.alx_fresh:
            self.stop_latch.request("alx_stale", StopKind.SOFT)

    def _dispatch_stop_from_latch(self, latch_snapshot: StopSnapshot) -> None:
        """Dispatch stop command from latch state.

        Parameters
        ----------
        latch_snapshot : StopSnapshot
            Current stop latch state.
        """
        reason = latch_snapshot.reason or "stop"

        # Log safety stop (only when reason changes)
        if self._last_safety_stop_reason != reason:
            self.csv_logger.log_safety_stop(
                monotonic_s=time.monotonic(),
                cloud_sequence=self._state.last_processed_sequence,
                stop_reason=reason,
                lifecycle_state=self.lifecycle.state.value,
                lock_state=self.target_manager.snapshot().state.value,
                calibration_status="provisional",
            )
            self._last_safety_stop_reason = reason

        # Dispatch zero motion
        self.limiter.reset()
        from robot_follow.core.types import MotionCommand
        command = MotionCommand(
            kind="stop",
            vx=0.0,
            vy=0.0,
            omega=0.0,
            reason=reason,
        )
        self.bridge.dispatch(command)

    def _publish_status(
        self,
        cloud: Optional[CloudSnapshot],
        preprocessed: Optional[PreprocessedCloud],
    ) -> None:
        """Replace the bounded Web status snapshot with JSON-safe current state."""
        target_snapshot = self.target_manager.snapshot()
        observation = self._state.last_observation
        strategy = self._state.last_strategy_result
        confidence = self._state.last_confidence
        max_points = self.config.web.max_visualization_points

        points = []
        target_points = []
        if preprocessed is not None:
            raw_points = preprocessed.points_xyz
            stride = max(1, int(np.ceil(len(raw_points) / float(max_points))))
            points = raw_points[::stride].tolist()
            if observation is not None:
                target_points = observation.support_points_xyz.tolist()

        center = target_snapshot.center_xy
        alx_xy = self._state.last_alx_body_xy
        self.latest_status.update(
            {
                "cloud_sequence": cloud.sequence if cloud is not None else None,
                "frame_id": cloud.frame_id if cloud is not None else None,
                "points": points,
                "target_points": target_points,
                "target_gate": (
                    {
                        "kind": "ellipse",
                        "center_xy": list(center),
                        "radial_radius_m": self.config.target.gate_radial_radius_m,
                        "tangential_radius_m": self.config.target.gate_tangential_radius_m,
                    }
                    if center is not None
                    else None
                ),
                "alx": {
                    "body_xy": alx_xy.tolist() if alx_xy is not None else None,
                    "age_s": self._state.last_alx_age_ms / 1000.0,
                    "stamp_s": self._state.last_alx_stamp_s,
                    "offset_ms": self._state.last_alx_offset_ms,
                    "synchronized": self._state.last_alx_synchronized,
                },
                "lifecycle_state": self.lifecycle.state.value,
                "lock_state": target_snapshot.state.value,
                "confidence": (
                    {
                        "c_t": confidence.c_t,
                        "c_g": confidence.c_g,
                        "c_a": confidence.c_a,
                        "total": confidence.total,
                    }
                    if confidence is not None
                    else None
                ),
                "stop": {
                    "requested": self.stop_latch.snapshot().requested,
                    "kind": (
                        self.stop_latch.snapshot().kind.value
                        if self.stop_latch.snapshot().kind is not None
                        else None
                    ),
                    "reason": self.stop_latch.snapshot().reason,
                },
                "dispatched_command": (
                    {
                        "vx": strategy.dispatched.vx,
                        "vy": strategy.dispatched.vy,
                        "omega": strategy.dispatched.omega,
                        "reason": strategy.dispatched.reason,
                    }
                    if strategy is not None
                    else None
                ),
                "calibration_status": "provisional",
            }
        )
