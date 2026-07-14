"""Target tracking and confidence computation for LiDAR + ALX Assisted Follow."""

from __future__ import annotations

from robot_follow.lidar_alx_assisted_follow.tracking.preprocessing import (
    preprocess_tracking_cloud,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import (
    TargetLockManager,
    TargetLockSnapshot,
    LockState,
)
from robot_follow.lidar_alx_assisted_follow.tracking.confidence import (
    compute_confidence,
)

__all__ = [
    "preprocess_tracking_cloud",
    "TargetLockManager",
    "TargetLockSnapshot",
    "LockState",
    "compute_confidence",
]
