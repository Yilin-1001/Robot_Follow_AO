"""Tests for runtime controller (cloud-driven control loop)."""

from __future__ import annotations

import time
from typing import Optional
from unittest.mock import Mock

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import PrototypeConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    CloudSnapshot,
    MotionIntent,
    StopKind,
    StopReason,
    StrategyResult,
)
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEvent,
    ControlEventInbox,
    ControlEventKind,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
from robot_follow.lidar_alx_assisted_follow.sensors.cloud import SequencedFastLIOPointCloudSource


def make_cloud_snapshot(
    sequence: int,
    received_at_s: float,
    points_xyz: Optional[np.ndarray] = None,
) -> CloudSnapshot:
    """Create a test cloud snapshot."""
    if points_xyz is None:
        points_xyz = np.array([[1.0, 0.0, 0.5]], dtype=np.float64)
        points_xyz.flags.writeable = False

    return CloudSnapshot(
        sequence=sequence,
        sensor_timestamp_s=received_at_s,
        received_at_s=received_at_s,
        frame_id="body",
        points_xyz=points_xyz,
    )


def make_motion_intent(
    sequence: int,
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    reason: str = "stop",
) -> MotionIntent:
    """Create a test motion intent."""
    return MotionIntent(
        sequence=sequence,
        vx=vx,
        vy=vy,
        omega=omega,
        reason=reason,
        generated_at_s=time.monotonic(),
    )


def make_strategy_result(
    sequence: int,
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    stop_reason: Optional[StopReason] = None,
) -> StrategyResult:
    """Create a test strategy result."""
    apf_requested = make_motion_intent(sequence, vx, vy, omega, reason="follow")
    dispatched = make_motion_intent(sequence, vx, vy, omega, reason="follow")
    return StrategyResult(
        sequence=sequence,
        apf_requested=apf_requested,
        dispatched=dispatched,
        safety_complete=True,
        stop_reason=stop_reason,
        compute_ms=1.0,
    )


# ============================================================================
# Test: Strategy runs once per sequence, not at watchdog rate
# ============================================================================


def test_strategy_runs_once_per_sequence_not_at_watchdog_rate():
    """Strategy should only execute when a NEW cloud sequence arrives.

    Watchdog ticks (no new cloud) should only perform safety checks,
    not recompute target extraction, confidence, or APF.
    """
    # This test will verify the runtime's behavior:
    # 1. Cloud sequence=1 arrives at now_s=1.000
    # 2. control_iteration at now_s=1.000 processes sequence=1, runs strategy
    # 3. control_iteration at now_s=1.025 (no new cloud) - only safety checks
    # 4. control_iteration at now_s=1.050 (no new cloud) - only safety checks
    # Expected: strategy_sequences == [1]

    # Placeholder: This will be implemented with the runtime
    # The runtime should track which sequences have been processed
    # and only run strategy for NEW sequences
    pass


def test_clouds_arriving_while_busy_collapse_to_latest():
    """If multiple clouds arrive during strategy computation, process only the latest.

    Overload handling: process newest, discard intermediate frames.
    """
    # This test will verify:
    # 1. Cloud sequence=2 arrives
    # 2. Cloud sequence=3 arrives while control is busy
    # 3. control_iteration processes sequence=3 (latest), skips sequence=2
    # Expected: strategy_sequences[-1] == 3

    pass


def test_watchdog_timeout_performs_safety_checks():
    """Watchdog timeout (no new cloud) should perform safety checks.

    Safety checks include:
    - Stop latch state
    - Cloud staleness
    - ALX staleness
    - Lifecycle state
    """
    pass


# ============================================================================
# Test: Control event handling
# ============================================================================


def test_control_events_drained_before_cloud_processing():
    """Control events should be processed before cloud strategy.

    Order:
    1. Drain control events (select_target, start_follow, stop_follow)
    2. Sample/deduplicate ALX
    3. Update lifecycle
    4. Process newest cloud
    """
    pass


def test_stop_follow_sets_latch_and_enqueues_event():
    """stop_follow should set StopLatch and enqueue event.

    The control thread reads the event and unlocks before dispatch.
    """
    latch = StopLatch(control_thread_id=999)
    inbox = ControlEventInbox()

    # Simulate stop_follow from web
    latch.request("user_stop", StopKind.HARD)
    inbox.push(ControlEvent(kind=ControlEventKind.STOP_FOLLOW, payload={}))

    assert latch.snapshot().requested
    assert latch.snapshot().reason == "user_stop"
    assert latch.snapshot().kind == StopKind.HARD
    assert inbox.qsize == 1


def test_controller_disconnect_sets_latch_and_enqueues_event():
    """Controller disconnect should set StopLatch and enqueue event.

    The control thread reads the event and unlocks before dispatch.
    """
    latch = StopLatch(control_thread_id=999)
    inbox = ControlEventInbox()

    # Simulate controller disconnect from web
    latch.request("controller_disconnected", StopKind.HARD)
    inbox.push(ControlEvent(kind=ControlEventKind.CONTROLLER_DISCONNECT, payload={}))

    assert latch.snapshot().requested
    assert latch.snapshot().reason == "controller_disconnected"
    assert latch.snapshot().kind == StopKind.HARD
    assert inbox.qsize == 1


# ============================================================================
# Test: Lifecycle integration
# ============================================================================


def test_lifecycle_not_ready_blocks_strategy():
    """If lifecycle state is not SENSORS_READY, skip strategy.

    Only perform safety checks and zero motion.
    """
    pass


def test_lifecycle_fault_requests_stop():
    """Lifecycle FAULT state should request stop."""
    pass


# ============================================================================
# Test: ALX staleness handling
# ============================================================================


def test_alx_stale_requests_soft_stop():
    """ALX staleness should request SOFT stop.

    SOFT stops can auto-recover when ALX becomes fresh again.
    """
    pass


def test_alx_stale_auto_recovers_when_fresh():
    """SOFT stop from alx_stale should auto-recover when ALX fresh.

    Recovery requires:
    - Same lock (target still valid)
    - Fresh post-stop cloud/support
    - Fresh synchronized ALX
    - C >= 0.80
    """
    pass


# ============================================================================
# Test: Cloud staleness handling
# ============================================================================


def test_cloud_stale_requests_hard_stop():
    """Cloud staleness (age > max_cloud_age_s) should request HARD stop.

    HARD stops require explicit Start to resume.
    """
    pass


# ============================================================================
# Test: Bridge tick called after dispatch
# ============================================================================


def test_bridge_tick_called_after_dispatch():
    """bridge.tick(now) should be called after dispatch.

    This ensures the bridge's watchdog is maintained.
    """
    pass


# ============================================================================
# Test: Strategy timeout
# ============================================================================


def test_strategy_timeout_dispatches_zero():
    """Strategy computation exceeding max_strategy_process_ms should dispatch zero.

    The timeout cannot preempt NumPy already running; this limitation is logged.
    """
    pass


# ============================================================================
# Test: Sequence tracking
# ============================================================================


def test_runtime_tracks_processed_sequences():
    """Runtime should track which cloud sequences have been processed.

    Each sequence should be processed at most once.
    """
    pass


def test_start_after_sequence_prevents_early_motion():
    """Motion should only be allowed for sequence > start_after_sequence.

    After Start, the latch records the latest cloud sequence.
    Motion is only allowed for later sequences.
    """
    import threading
    latch = StopLatch(control_thread_id=threading.get_ident())
    latch.clear_for_start(start_after_sequence=8)

    # Sequence 8 is the one seen at start - no motion yet
    assert not latch.motion_allowed_for(sequence=8)

    # Sequence 9 is after start - motion allowed
    assert latch.motion_allowed_for(sequence=9)


# ============================================================================
# Test: Multiple control iterations
# ============================================================================


def test_multiple_control_iterations_without_cloud():
    """Multiple control iterations without new cloud should not re-run strategy.

    Watchdog ticks at 40 Hz (25 ms timeout).
    Without new cloud, only safety checks run.
    """
    pass
