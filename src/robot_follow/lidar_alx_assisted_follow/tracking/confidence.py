"""Confidence computation and state transitions."""

from __future__ import annotations

from typing import Optional

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    ConfidenceConfig,
    TargetConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain import (
    ConfidenceResult,
    GatedTargetObservation,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import (
    LockState,
    TargetLockManager,
)


def compute_confidence(
    observation: Optional[GatedTargetObservation],
    target_center_xy: Optional[tuple[float, float]],
    alx_body_xy: Optional[np.ndarray],
    alx_fresh: bool,
    config: ConfidenceConfig,
    target_config: TargetConfig,
    is_selected_seed: bool = False,
    center_displacement: Optional[float] = None,
    frame_interval: Optional[float] = None,
) -> ConfidenceResult:
    """Compute confidence from temporal, geometric, and ALX components.

    Parameters
    ----------
    observation : GatedTargetObservation or None
        Current target observation, or None if no observation.
    target_center_xy : tuple[float, float] or None
        Current target center in body frame.
    alx_body_xy : np.ndarray or None
        ALX position in body frame, or None if unavailable.
    alx_fresh : bool
        Whether ALX data is fresh (not stale).
    config : ConfidenceConfig
        Confidence computation configuration.
    target_config : TargetConfig
        Target tracking configuration.
    is_selected_seed : bool
        True if this is the initial click seed observation.
    center_displacement : float or None
        Displacement from last accepted center (m).
    frame_interval : float or None
        Time since last accepted frame (s).

    Returns
    -------
    ConfidenceResult
        Computed confidence components and total.
    """
    # --- c_t: temporal component ---
    c_t = _compute_temporal(
        observation=observation,
        is_selected_seed=is_selected_seed,
        center_displacement=center_displacement,
        frame_interval=frame_interval,
        config=target_config,
    )

    # --- c_g: geometric component ---
    c_g = _compute_geometric(
        observation=observation,
        config=config,
        target_config=target_config,
    )

    # --- c_a: ALX component ---
    c_a = _compute_alx(
        observation=observation,
        target_center_xy=target_center_xy,
        alx_body_xy=alx_body_xy,
        config=config,
    )

    # --- Total confidence ---
    total = float(np.clip(
        config.weight_temporal * c_t
        + config.weight_geometry * c_g
        + config.weight_alx * c_a,
        0.0,
        1.0,
    ))

    # Determine reason string
    reasons = []
    if observation is None:
        reasons.append("no_observation")
    if not alx_fresh:
        reasons.append("alx_stale")
    if total < config.stop:
        reasons.append(f"low_confidence({total:.3f})")

    reason = ", ".join(reasons) if reasons else "nominal"

    return ConfidenceResult(
        c_t=c_t,
        c_g=c_g,
        c_a=c_a,
        total=total,
        alx_fresh=alx_fresh,
        reason=reason,
    )


def _compute_temporal(
    observation: Optional[GatedTargetObservation],
    is_selected_seed: bool,
    center_displacement: Optional[float],
    frame_interval: Optional[float],
    config: TargetConfig,
) -> float:
    """Compute temporal confidence component c_t.

    c_t = 1.0 if selected seed, or if accepted track with:
      - center_displacement <= max_center_displacement_m
      - frame_interval <= max_frame_interval_s
    Otherwise c_t = 0.0.
    """
    if observation is None:
        return 0.0

    if is_selected_seed:
        return 1.0

    # Check displacement
    if center_displacement is not None and center_displacement > config.max_center_displacement_m:
        return 0.0

    # Check frame interval
    if frame_interval is not None and frame_interval > config.max_frame_interval_s:
        return 0.0

    return 1.0


def _compute_geometric(
    observation: Optional[GatedTargetObservation],
    config: ConfidenceConfig,
    target_config: TargetConfig,
) -> float:
    """Compute geometric confidence component c_g.

    c_g = 0.40 * p + 0.30 * h + 0.30 * q

    where:
      p = clip(point_count / (2 * min_support_points), 0, 1)
      h = clip(vertical_extent / (2 * min_vertical_extent), 0, 1)
      q = 1 - clip(radial_mad / max_radial_mad, 0, 1)
    """
    if observation is None:
        return 0.0

    # p: point count ratio
    p = float(np.clip(
        observation.point_count / (2.0 * target_config.min_support_points),
        0.0,
        1.0,
    ))

    # h: vertical extent ratio
    h = float(np.clip(
        observation.vertical_extent_m / (2.0 * target_config.min_vertical_extent_m),
        0.0,
        1.0,
    ))

    # q: radial MAD inverse ratio
    q = float(np.clip(
        1.0 - observation.radial_mad_m / target_config.max_radial_mad_m,
        0.0,
        1.0,
    ))

    return 0.40 * p + 0.30 * h + 0.30 * q


def _compute_alx(
    observation: Optional[GatedTargetObservation],
    target_center_xy: Optional[tuple[float, float]],
    alx_body_xy: Optional[np.ndarray],
    config: ConfidenceConfig,
) -> float:
    """Compute ALX confidence component c_a.

    c_a = clip(1.0 - norm(alx_body_xy - target_center) / tolerance, 0, 1)

    Returns 0 if no ALX data or no target center.
    """
    if alx_body_xy is None or target_center_xy is None:
        return 0.0

    if observation is None:
        return 0.0

    # Compute distance between ALX position and target center
    alx_xy = np.asarray(alx_body_xy, dtype=np.float64)
    target_xy = np.array(target_center_xy, dtype=np.float64)
    distance = float(np.linalg.norm(alx_xy - target_xy))

    return float(np.clip(
        1.0 - distance / config.alx_body_to_wrist_tolerance_m,
        0.0,
        1.0,
    ))


def check_confidence_state(
    confidence: ConfidenceResult,
    current_state: LockState,
    config: ConfidenceConfig,
) -> tuple[str, Optional[str]]:
    """Check confidence against state thresholds and return actions.

    Parameters
    ----------
    confidence : ConfidenceResult
        Current computed confidence.
    current_state : LockState
        Current lock state.
    config : ConfidenceConfig
        Confidence configuration.

    Returns
    -------
    tuple[str, str or None]
        (action, reason) where action is one of:
        - "ok" : no action needed
        - "stop" : request confidence stop
        - "unlock" : request unlock
    """
    total = confidence.total

    if current_state == LockState.FOLLOWING:
        if total < config.unlock:
            return ("unlock", f"confidence_below_unlock({total:.3f})")
        elif total < config.stop:
            return ("stop", f"confidence_below_stop({total:.3f})")

    if current_state in (LockState.CANDIDATE, LockState.FOLLOWING):
        if not confidence.alx_fresh:
            return ("stop", "alx_stale")

    return ("ok", None)