"""Shared contracts for local obstacle-avoidance modes.

This module deliberately sits outside the strategy package so the legacy APF
implementation can use the contract without an import cycle.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from robot_follow.lidar_alx_assisted_follow.domain.models import (
    MotionIntent,
    PreprocessedCloud,
    StopReason,
)


@dataclass(frozen=True)
class AvoidanceInput:
    """One cloud-synchronous local-planning request in the body frame."""

    cloud: PreprocessedCloud
    target_center_xy: Tuple[float, float]
    target_mask: np.ndarray
    sequence: int
    alx_body_xy: Optional[np.ndarray] = None
    target_retreat_active: bool = False
    confidence: object | None = None


@dataclass(frozen=True)
class AvoidanceResult:
    """Motion requested by an avoidance mode and any frame-local stop."""

    intent: MotionIntent
    stop_reason: Optional[StopReason]
