"""APF (Artificial Potential Field) motion planning."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import APFConfig, OutputConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    ConfidenceResult,
    MotionIntent,
    PreprocessedCloud,
    StopReason,
    StopSnapshot,
    StrategyResult,
)


@dataclass(frozen=True)
class APFInput:
    """Input to APF computation."""

    cloud: PreprocessedCloud
    target_center_xy: Tuple[float, float]
    target_mask: np.ndarray  # Boolean mask in downsampled domain
    confidence: ConfidenceResult
    sequence: int
    alx_body_xy: Optional[np.ndarray] = None


@dataclass(frozen=True)
class APFResult:
    """Result of APF computation."""

    intent: MotionIntent
    stop_reason: Optional[StopReason]


def compute_apf(
    input_data: APFInput,
    config: APFConfig,
    output_config: OutputConfig,
) -> APFResult:
    """Compute APF-based motion intent.

    Parameters
    ----------
    input_data : APFInput
        Cloud, target, and confidence data.
    config : APFConfig
        APF configuration.
    output_config : OutputConfig
        Output limits configuration.

    Returns
    -------
    APFResult
        Motion intent and stop reason.
    """
    # Validate inputs
    if not _validate_apf_inputs(input_data):
        return APFResult(
            intent=_zero_intent(input_data.sequence),
            stop_reason=StopReason.INVALID_INPUT,
        )

    target_xy = np.array(input_data.target_center_xy, dtype=np.float64)
    target_distance = float(np.linalg.norm(target_xy))

    # Check 1: Target too close (hard stop, zero motion)
    if target_distance <= config.target_stop_distance_m:
        return APFResult(
            intent=_zero_intent(input_data.sequence),
            stop_reason=StopReason.TARGET_TOO_CLOSE,
        )

    # Check 2: Non-target hard clearance (hard stop, zero motion)
    non_target_points = _get_non_target_points(
        input_data.cloud.points_xyz,
        input_data.target_mask,
    )

    for point_xy in non_target_points:
        clearance = _compute_body_clearance(point_xy[:2], config)
        if clearance <= config.non_target_hard_stop_clearance_m:
            return APFResult(
                intent=_zero_intent(input_data.sequence),
                stop_reason=StopReason.NON_TARGET_HARD_CLEARANCE,
            )

    # Check 3: Confidence not in FOLLOWING state
    if input_data.confidence.total < 0.70:  # confidence.stop threshold
        return APFResult(
            intent=_zero_intent(input_data.sequence),
            stop_reason=StopReason.CONFIDENCE_LOW,
        )

    # Compute attraction force
    r = target_distance
    u = target_xy / r  # Unit vector toward target
    f_att_magnitude = config.attraction_gain * max(r - config.target_desired_distance_m, 0.0)
    F_att = f_att_magnitude * u

    # Compute repulsion force
    F_rep = np.zeros(2, dtype=np.float64)
    for point_xy in non_target_points:
        d = float(np.linalg.norm(point_xy[:2]))
        if 0 < d < config.non_target_influence_m:
            # Repulsion: (1/d - 1/influence) / d^2 * (-point/d)
            repulsion_contribution = (
                config.repulsion_gain
                * (1.0 / d - 1.0 / config.non_target_influence_m)
                / (d ** 2)
            )
            direction = -point_xy[:2] / d
            F_rep += repulsion_contribution * direction

    # Cap repulsion force
    F_rep_norm = float(np.linalg.norm(F_rep))
    if F_rep_norm > config.repulsion_cap:
        F_rep = F_rep * (config.repulsion_cap / F_rep_norm)

    # Total force
    F = F_att + F_rep
    F_norm = float(np.linalg.norm(F))

    # Target bearing angle
    target_bearing_rad = math.atan2(target_xy[1], target_xy[0])
    target_bearing_deg = math.degrees(target_bearing_rad)

    # Check 4: Turn in place if bearing > 35 degrees
    if abs(target_bearing_deg) > config.turn_in_place_bearing_deg:
        # Check if rotation is safe
        rotation_radius = math.hypot(config.body_half_length_m, config.body_half_width_m)
        rotation_clearance = rotation_radius + config.swept_margin_m

        rotation_safe = True
        for point_xy in non_target_points:
            d = float(np.linalg.norm(point_xy[:2]))
            if d < rotation_clearance:
                rotation_safe = False
                break

        if rotation_safe:
            # Pure rotation command
            omega = float(np.clip(
                config.omega_gain * target_bearing_rad,
                -output_config.max_abs_omega,
                output_config.max_abs_omega,
            ))
            return APFResult(
                intent=MotionIntent(
                    sequence=input_data.sequence,
                    vx=0.0,
                    vy=0.0,
                    omega=omega,
                    reason="turn_in_place",
                    generated_at_s=time.monotonic(),
                ),
                stop_reason=None,
            )
        else:
            # Unsafe rotation
            return APFResult(
                intent=_zero_intent(input_data.sequence),
                stop_reason=StopReason.UNSAFE_ROTATION,
            )

    # Normal translation case
    if F_norm < 1e-6:
        # Zero force
        return APFResult(
            intent=_zero_intent(input_data.sequence),
            stop_reason=None,
        )

    # Compute requested speed
    max_speed = math.hypot(output_config.max_vx, output_config.max_abs_vy)
    requested_speed = min(F_norm, max_speed)

    # Force direction
    force_angle_rad = math.atan2(F[1], F[0])

    # Try candidate headings
    candidate_angles_rad = [math.radians(deg) for deg in config.candidate_angles_deg]

    # Sort by angular difference from force direction
    candidate_angles_rad = sorted(
        candidate_angles_rad,
        key=lambda a: abs(a - force_angle_rad),
    )

    # Find first safe candidate
    for theta in candidate_angles_rad:
        vx = float(np.clip(requested_speed * math.cos(theta), 0.0, output_config.max_vx))
        vy = float(np.clip(requested_speed * math.sin(theta), -output_config.max_abs_vy, output_config.max_abs_vy))

        # Check swept envelope
        if _is_swept_envelope_safe(
            vx, vy,
            non_target_points,
            config,
        ):
            omega = float(np.clip(
                config.omega_gain * target_bearing_rad,
                -output_config.max_abs_omega,
                output_config.max_abs_omega,
            ))
            return APFResult(
                intent=MotionIntent(
                    sequence=input_data.sequence,
                    vx=vx,
                    vy=vy,
                    omega=omega,
                    reason="follow",
                    generated_at_s=time.monotonic(),
                ),
                stop_reason=None,
            )

    # No safe direction
    return APFResult(
        intent=_zero_intent(input_data.sequence),
        stop_reason=StopReason.ALL_DIRECTIONS_UNSAFE,
    )


def _validate_apf_inputs(input_data: APFInput) -> bool:
    """Validate APF inputs are finite and non-empty."""
    if input_data.cloud.points_xyz.shape[0] == 0:
        return False

    if not np.all(np.isfinite(input_data.cloud.points_xyz)):
        return False

    if not np.all(np.isfinite(input_data.target_center_xy)):
        return False

    if input_data.target_mask.shape[0] != input_data.cloud.points_xyz.shape[0]:
        return False

    return True


def _zero_intent(sequence: int) -> MotionIntent:
    """Create zero motion intent."""
    return MotionIntent(
        sequence=sequence,
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason="stop",
        generated_at_s=time.monotonic(),
    )


def _get_non_target_points(
    points_xyz: np.ndarray,
    target_mask: np.ndarray,
) -> np.ndarray:
    """Extract non-target points."""
    return points_xyz[~target_mask]


def _compute_body_clearance(
    point_xy: np.ndarray,
    config: APFConfig,
) -> float:
    """Compute clearance from body envelope to a point.

    Body envelope: [-body_half_length, body_half_length] x [-body_half_width, body_half_width]

    Clearance = hypot(max(|x| - body_half_length, 0), max(|y| - body_half_width, 0))
    """
    dx = max(abs(point_xy[0]) - config.body_half_length_m, 0.0)
    dy = max(abs(point_xy[1]) - config.body_half_width_m, 0.0)
    return float(math.hypot(dx, dy))


def _is_swept_envelope_safe(
    vx: float,
    vy: float,
    non_target_points: np.ndarray,
    config: APFConfig,
) -> bool:
    """Check if swept envelope is clear of obstacles.

    Swept envelope: body rectangle extended along velocity direction.

    Parameters
    ----------
    vx, vy : float
        Velocity components (m/s).
    non_target_points : np.ndarray
        Non-target obstacle points (N, 3).
    config : APFConfig
        APF configuration.

    Returns
    -------
    bool
        True if swept envelope is safe.
    """
    if len(non_target_points) == 0:
        return True

    speed = math.hypot(vx, vy)
    if speed < 1e-6:
        # Zero velocity is always safe
        return True

    # Direction unit vector
    theta = math.atan2(vy, vx)
    e = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])

    # Sweep distance
    sweep_distance = max(config.swept_min_distance_m, speed * config.swept_horizon_s)

    # Check each obstacle
    for point_xy in non_target_points:
        longitudinal = float(point_xy[:2] @ e)
        lateral = float(point_xy[:2] @ n)

        # Check if in swept rectangle
        in_longitudinal = (
            -config.body_half_length_m <= longitudinal
            <= config.body_half_length_m + sweep_distance + config.swept_margin_m
        )
        in_lateral = abs(lateral) <= config.body_half_width_m + config.swept_margin_m

        if in_longitudinal and in_lateral:
            return False

    return True