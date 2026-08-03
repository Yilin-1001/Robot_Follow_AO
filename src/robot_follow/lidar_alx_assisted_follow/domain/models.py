"""Core domain models - frozen dataclasses for cross-component contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class StopKind(str, Enum):
    """Stop request kind - HARD dominates SOFT."""

    SOFT = "soft"
    HARD = "hard"


class StopReason(str, Enum):
    """Canonical stop reason strings for StrategyResult, StopSnapshot, Web frames, and CSV.

    SOFT stops (can auto-recover on same lock):
    - confidence_low: C < confidence.stop
    - alx_stale: ALX no valid/synchronized/fresh sample

    HARD stops (require explicit Start):
    - user_stop: Web stop_follow
    - controller_disconnected: WebSocket controller disconnect
    - wristband_off: Wristband turned off
    - serial_error: ALX serial port error
    - ready_timeout: Sensor startup timeout
    - cloud_stale: Cloud age > max_cloud_age_s
    - target_lost: No valid support for > lost_timeout_s
    - bridge_fault: G1 bridge/SDK fault

    Zero-motion stops (current frame only, do not set StopLatch):
    - lifecycle_not_ready: Not yet SENSORS_READY
    - target_too_close: Target distance <= stop_distance_m
    - non_target_hard_clearance: Non-target clearance <= hard_stop_clearance_m
    - unsafe_rotation: Rotation envelope unsafe
    - all_directions_unsafe: All candidate directions unsafe
    - strategy_timeout: Strategy computation > max_strategy_process_ms
    - strategy_stale: Cloud-to-dispatch age > max_strategy_age_s
    - invalid_input: Invalid input data
    """

    # SOFT stops (auto-recoverable)
    CONFIDENCE_LOW = "confidence_low"
    ALX_STALE = "alx_stale"

    # HARD stops (require explicit Start)
    USER_STOP = "user_stop"
    CONTROLLER_DISCONNECTED = "controller_disconnected"
    WRISTBAND_OFF = "wristband_off"
    SERIAL_ERROR = "serial_error"
    READY_TIMEOUT = "ready_timeout"
    CLOUD_STALE = "cloud_stale"
    TARGET_LOST = "target_lost"
    BRIDGE_FAULT = "bridge_fault"

    # Zero-motion stops (current frame, no latch)
    LIFECYCLE_NOT_READY = "lifecycle_not_ready"
    TARGET_TOO_CLOSE = "target_too_close"
    NON_TARGET_HARD_CLEARANCE = "non_target_hard_clearance"
    UNSAFE_ROTATION = "unsafe_rotation"
    ALL_DIRECTIONS_UNSAFE = "all_directions_unsafe"
    STRATEGY_TIMEOUT = "strategy_timeout"
    STRATEGY_STALE = "strategy_stale"
    INVALID_INPUT = "invalid_input"


class StartState(str, Enum):
    """Result of the most recent Web follow-enable request."""

    IDLE = "idle"
    ACCEPTED = "accepted"
    REJECTED = "rejected"


class MotionState(str, Enum):
    """Operator-facing derived motion state for the Web monitor."""

    DISABLED = "disabled"
    WAITING_WRISTBAND = "waiting_wristband"
    WAITING_LIDAR = "waiting_lidar"
    WAITING_ALX = "waiting_alx"
    FOLLOWING = "following"
    TARGET_RETREAT = "target_retreat"
    ESCAPING = "escaping"
    CLEARANCE_RECOVERY = "clearance_recovery"
    BLOCKED_OBSTACLE = "blocked_obstacle"
    SAFETY_STOP = "safety_stop"


@dataclass(frozen=True)
class StartResult:
    """Persistent feedback for a Web Start Follow request."""

    state: StartState = StartState.IDLE
    reason: Optional[str] = None
    timestamp_s: Optional[float] = None


@dataclass(frozen=True)
class CloudSnapshot:
    """Immutable LiDAR cloud snapshot with sequencing."""

    sequence: int
    sensor_timestamp_s: float
    received_at_s: float
    frame_id: str
    points_xyz: np.ndarray


@dataclass(frozen=True)
class ALXObservation:
    """ALX observation synchronized to a cloud."""

    snapshot: object  # ALXSnapshot, but avoid circular import
    received_at_s: float
    body_xy: Optional[np.ndarray]
    cloud_skew_s: Optional[float]
    synchronized: bool


@dataclass(frozen=True)
class PreprocessedCloud:
    """Downsampled cloud with source index mapping."""

    points_xyz: np.ndarray
    source_indices: np.ndarray


@dataclass(frozen=True)
class GatedTargetObservation:
    """Target observation that passed gate checks."""

    center_xy: tuple[float, float]
    target_mask: np.ndarray
    support_points_xyz: np.ndarray
    point_count: int
    vertical_extent_m: float
    horizontal_footprint_m: float
    radial_mad_m: float
    observed_at_s: float


@dataclass(frozen=True)
class ConfidenceResult:
    """Legacy confidence record retained only for CSV compatibility.

    The assisted-follow runtime no longer computes or gates motion on this
    value: ALX freshness and time synchronization are the target validity
    contract.
    """

    c_t: float
    c_g: float
    c_a: float
    total: float
    alx_fresh: bool
    reason: str


@dataclass(frozen=True)
class MotionIntent:
    """Motion command intent before limiting."""

    sequence: int
    vx: float
    vy: float
    omega: float
    reason: str
    generated_at_s: float


@dataclass(frozen=True)
class StrategyResult:
    """Complete strategy output for one cloud."""

    sequence: int
    apf_requested: MotionIntent
    dispatched: MotionIntent
    safety_complete: bool
    stop_reason: Optional[StopReason]
    compute_ms: float


@dataclass(frozen=True)
class StopSnapshot:
    """Immutable view of stop latch state."""

    requested: bool
    kind: Optional[StopKind]
    reason: Optional[StopReason]
    start_after_sequence: int
    version: int
