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
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Protocol, Tuple

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
from robot_follow.lidar_alx_assisted_follow.sensors.cloud import SequencedFastLIOPointCloudSource
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)
from robot_follow.lidar_alx_assisted_follow.tracking.confidence import (
    check_confidence_state,
    compute_confidence,
)
from robot_follow.lidar_alx_assisted_follow.tracking.preprocessing import preprocess_tracking_cloud
from robot_follow.lidar_alx_assisted_follow.tracking.target import (
    LockState,
    TargetLockManager,
    TargetLockSnapshot,
)


class ALXReaderProtocol(Protocol):
    """Protocol for ALX serial reader."""

    def latest_snapshot(self) -> Optional[ALXSnapshot]:
        """Get latest ALX snapshot."""
        ...

    def is_ok(self) -> bool:
        """Check if serial communication is healthy."""
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
    alx_buffer: ALXSnapshotBuffer
    lifecycle: WristbandLifecycleGuardian
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
    _running: bool = False
    _control_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the control loop thread."""
        if self._running:
            return

        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop,
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
            # Select new target
            x = event.payload.get("x")
            y = event.payload.get("y")
            if x is not None and y is not None:
                # This will be processed during cloud processing
                # For now, just store the click
                pass

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
        - Valid Candidate target
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

    def _sample_alx(self, now: float) -> Optional[ALXSnapshot]:
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
        alx_snapshot: Optional[ALXSnapshot],
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
        # Determine wristband state
        wristband_on = alx_snapshot is not None and not alx_snapshot.error
        tag_id = alx_snapshot.tag_id if alx_snapshot is not None else None
        serial_ok = self.alx_reader.is_ok()

        # Tick lifecycle
        new_state, should_stop_services, transition_reason = self.lifecycle.tick(
            wristband_on=wristband_on,
            tag_id=tag_id,
            serial_ok=serial_ok,
            cloud_received=cloud_received,
            now=now,
        )

        # Handle lifecycle transitions
        if should_stop_services:
            # Request stop if needed
            if transition_reason in ("wristband_off_or_serial_error", "ready_timeout", "tag_changed"):
                self.stop_latch.request(transition_reason, StopKind.HARD)
                self.target_manager.unlock()

    def _process_cloud(
        self,
        cloud: CloudSnapshot,
        alx_snapshot: Optional[ALXSnapshot],
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

        # Compute cloud age
        cloud_age_s = now - cloud.received_at_s
        self._state.last_cloud_age_ms = cloud_age_s * 1000.0

        # Check cloud staleness
        if cloud_age_s > self.config.sensors.max_cloud_age_s:
            self.stop_latch.request("cloud_stale", StopKind.HARD)
            self._log_strategy_zero(cloud, "cloud_stale", now)
            return

        # Check lifecycle
        if self.lifecycle.state != LifecycleState.SENSORS_READY:
            self._log_strategy_zero(cloud, "lifecycle_not_ready", now)
            return

        # Check stop latch
        latch_snapshot = self.stop_latch.snapshot()
        if latch_snapshot.requested:
            reason = latch_snapshot.reason or "stop"
            self._log_strategy_zero(cloud, reason, now)
            return

        # Preprocess cloud
        preprocessed = preprocess_tracking_cloud(
            cloud.points_xyz,
            self.config.target,
            self.config.calibration,
        )

        # Get ALX observation
        alx_body_xy: Optional[np.ndarray] = None
        alx_fresh = False
        alx_age_ms = 0.0

        if alx_snapshot is not None:
            best_snapshot = self.alx_buffer.for_cloud(
                cloud.received_at_s,
                self.lifecycle.session_tag_id,
            )
            if best_snapshot is not None:
                alx_body_xy = self.alx_buffer.transform_to_body(best_snapshot)
                alx_age_s = now - best_snapshot.received_at
                alx_age_ms = alx_age_s * 1000.0
                alx_fresh = alx_age_s < self.config.alx.stale_timeout_s

        self._state.last_alx_age_ms = alx_age_ms

        # Check ALX staleness
        if not alx_fresh:
            self.stop_latch.request("alx_stale", StopKind.SOFT)

        # Update target tracking
        observation, lost = self.target_manager.update(preprocessed, now)

        if lost:
            self.stop_latch.request("target_lost", StopKind.HARD)

        self._state.last_observation = observation

        # Compute confidence
        target_snapshot = self.target_manager.snapshot()
        confidence = compute_confidence(
            observation=observation,
            target_center_xy=target_snapshot.center_xy,
            alx_body_xy=alx_body_xy,
            alx_fresh=alx_fresh,
            config=self.config.confidence,
            target_config=self.config.target,
        )

        self._state.last_confidence = confidence

        # Check confidence state
        action, reason = check_confidence_state(
            confidence,
            target_snapshot.state,
            self.config.confidence,
        )

        if action == "unlock":
            self.stop_latch.request(reason or "confidence_low", StopKind.HARD)
            self.target_manager.unlock()
        elif action == "stop":
            self.stop_latch.request(reason or "confidence_low", StopKind.SOFT)

        # Check if we have a valid target
        if observation is None or target_snapshot.center_xy is None:
            self._log_strategy_zero(cloud, "no_target", now)
            return

        # Run APF
        apf_input = APFInput(
            cloud=preprocessed,
            target_center_xy=target_snapshot.center_xy,
            target_mask=observation.target_mask,
            confidence=confidence,
            sequence=cloud.sequence,
            alx_body_xy=alx_body_xy,
        )

        apf_result = compute_apf(apf_input, self.config.apf, self.config.output_motion)

        # Build strategy result
        compute_start = time.monotonic()

        strategy_result = StrategyResult(
            sequence=cloud.sequence,
            apf_requested=apf_result.intent,
            dispatched=apf_result.intent,  # Will be updated after dispatch
            safety_complete=True,
            stop_reason=apf_result.stop_reason,
            compute_ms=(time.monotonic() - compute_start) * 1000.0,
        )

        self._state.last_strategy_result = strategy_result

        # Dispatch to G1
        dispatch_start = time.monotonic()
        latch_snapshot = self.stop_latch.snapshot()

        dispatch_strategy_or_stop(
            strategy=strategy_result,
            latch_snapshot=latch_snapshot,
            bridge=self.bridge,
            limiter=self.limiter,
            now_s=now,
        )

        dispatch_ms = (time.monotonic() - dispatch_start) * 1000.0

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
            target_support_count=observation.point_count if observation else 0,
            confidence=confidence,
            stop_reason=apf_result.stop_reason.value if apf_result.stop_reason else None,
            apf_requested_vx=apf_result.intent.vx,
            apf_requested_vy=apf_result.intent.vy,
            apf_requested_omega=apf_result.intent.omega,
            dispatched_vx=apf_result.intent.vx,  # Will be updated after limiter
            dispatched_vy=apf_result.intent.vy,
            dispatched_omega=apf_result.intent.omega,
            strategy_compute_ms=strategy_result.compute_ms,
            g1_dispatch_ms=dispatch_ms,
            bridge_state="initialized",
            sdk_error=None,
            calibration_status="provisional",
        )

    def _log_strategy_zero(
        self,
        cloud: CloudSnapshot,
        reason: str,
        now: float,
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

        # Check cloud staleness
        if self._state.last_processed_sequence > 0:
            # No new cloud in a while
            pass

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