"""Continuous ALX follow/avoidance inspired by ``jie_deamon``.

For each fresh point cloud, the controller combines three velocity terms:

``target tracking + target-corridor centering + APF repulsion``.

Unlike the previous first-safe-heading planner, obstacles inside the influence
zone steer and slow the robot.  Only a point inside the emergency *body
clearance* boundary produces an immediate zero command.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    OutputMotionConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    MotionIntent,
    PreprocessedCloud,
    StopReason,
)


@dataclass(frozen=True)
class APFInput:
    """Input for one cloud-synchronous follow/avoidance pass."""

    cloud: PreprocessedCloud
    target_center_xy: Tuple[float, float]
    target_mask: np.ndarray  # ALX target returns, excluded from obstacle terms
    sequence: int
    alx_body_xy: Optional[np.ndarray] = None
    # Transitional constructor compatibility for offline callers. It is not a
    # motion gate in the direct-ALX pipeline.
    confidence: object | None = None


@dataclass(frozen=True)
class APFResult:
    """Requested motion and any frame-local safety stop reason."""

    intent: MotionIntent
    stop_reason: Optional[StopReason]


def compute_apf(
    input_data: APFInput,
    config: APFConfig,
    output_config: OutputMotionConfig,
) -> APFResult:
    """Compute a continuous target/corridor/repulsion follow command.

    This adapts the ``jie_deamon`` LaserScan controller to our time-matched
    ALX target and 3D LiDAR cloud.  Point-cloud preprocessing and
    ``target_mask`` happen before this function, so the remaining points are
    obstacle candidates in the body frame.
    """
    if not _validate_apf_inputs(input_data):
        return APFResult(_zero_intent(input_data.sequence), StopReason.INVALID_INPUT)

    target_xy = np.asarray(input_data.target_center_xy, dtype=np.float64)
    target_distance = float(np.linalg.norm(target_xy))
    if target_distance <= config.target_stop_distance_m:
        return APFResult(
            _zero_intent(input_data.sequence), StopReason.TARGET_TOO_CLOSE
        )

    obstacles = _get_non_target_points(input_data.cloud.points_xyz, input_data.target_mask)
    min_clearance = _minimum_body_clearance(obstacles, config)
    if min_clearance <= config.non_target_hard_stop_clearance_m:
        return APFResult(
            _zero_intent(input_data.sequence), StopReason.NON_TARGET_HARD_CLEARANCE
        )

    # 1. Target tracking: use forward target error for translation and the
    # bearing for yaw, matching jie_deamon's follow controller. Reverse is
    # deliberately disabled for G1 follow tests; a target behind the robot
    # creates yaw but no backward drive.
    forward_error = float(target_xy[0] - config.target_desired_distance_m)
    vx_target = 0.0 if abs(forward_error) < 0.05 else config.attraction_gain * forward_error
    omega = config.omega_gain * math.atan2(float(target_xy[1]), float(target_xy[0]))

    # 2. Target corridor centering: find the closest inward obstacle on each
    # side of the body-to-target corridor. An obstacle on the right produces a
    # positive vy (move left), and vice versa.
    vy_corridor = _corridor_lateral_velocity(target_xy, obstacles, config)

    # 3. APF repulsion: sum inverse-distance repulsive velocity contributions
    # inside the influence radius, then cap their resultant magnitude.
    repulse = _repulsion_velocity(obstacles, config)

    vx = vx_target + float(repulse[0])
    vy = vy_corridor + float(repulse[1])

    # jie_deamon slows only forward motion near an obstacle, retaining lateral
    # authority so the robot can continue to sidestep around it.
    if min_clearance < config.slowdown_clearance_m:
        slowdown = (min_clearance - config.non_target_hard_stop_clearance_m) / (
            config.slowdown_clearance_m - config.non_target_hard_stop_clearance_m
        )
        vx *= float(np.clip(slowdown, 0.1, 1.0))

    vx = float(np.clip(vx, 0.0, output_config.max_vx))
    vy = float(np.clip(vy, -output_config.max_abs_vy, output_config.max_abs_vy))
    omega = float(np.clip(omega, -output_config.max_abs_omega, output_config.max_abs_omega))

    return APFResult(
        intent=MotionIntent(
            sequence=input_data.sequence,
            vx=vx,
            vy=vy,
            omega=omega,
            reason="follow_avoid" if len(obstacles) else "follow",
            generated_at_s=time.monotonic(),
        ),
        stop_reason=None,
    )


def _validate_apf_inputs(input_data: APFInput) -> bool:
    """Validate finite, non-empty cloud data and matching target mask."""
    return bool(
        input_data.cloud.points_xyz.shape[0]
        and np.all(np.isfinite(input_data.cloud.points_xyz))
        and np.all(np.isfinite(input_data.target_center_xy))
        and input_data.target_mask.shape[0] == input_data.cloud.points_xyz.shape[0]
    )


def _zero_intent(sequence: int) -> MotionIntent:
    return MotionIntent(
        sequence=sequence,
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason="stop",
        generated_at_s=time.monotonic(),
    )


def _get_non_target_points(points_xyz: np.ndarray, target_mask: np.ndarray) -> np.ndarray:
    return points_xyz[~target_mask]


def _minimum_body_clearance(obstacles: np.ndarray, config: APFConfig) -> float:
    if len(obstacles) == 0:
        return float("inf")
    return min(_compute_body_clearance(point[:2], config) for point in obstacles)


def _compute_body_clearance(point_xy: np.ndarray, config: APFConfig) -> float:
    """Return exterior clearance from the rectangular G1 body envelope."""
    dx = max(abs(float(point_xy[0])) - config.body_half_length_m, 0.0)
    dy = max(abs(float(point_xy[1])) - config.body_half_width_m, 0.0)
    return float(math.hypot(dx, dy))


def _corridor_lateral_velocity(
    target_xy: np.ndarray,
    obstacles: np.ndarray,
    config: APFConfig,
) -> float:
    """Compute jie_deamon-style lateral centering within the target corridor."""
    target_distance = float(np.linalg.norm(target_xy))
    if target_distance < 1e-6:
        return 0.0

    forward = target_xy / target_distance
    left_normal = np.array([-forward[1], forward[0]], dtype=np.float64)
    half_width = config.body_half_width_m + config.swept_margin_m
    left_inner = -half_width
    right_inner = half_width

    for point in obstacles:
        xy = point[:2]
        longitudinal = float(xy @ forward)
        lateral = float(xy @ left_normal)
        if 0.0 <= longitudinal <= target_distance and abs(lateral) <= half_width:
            if lateral > 0.0:
                left_inner = max(left_inner, lateral)
            else:
                right_inner = min(right_inner, lateral)

    lateral_error = -(left_inner + right_inner)
    if abs(lateral_error) < 0.03:
        return 0.0
    return config.corridor_gain * lateral_error


def _repulsion_velocity(obstacles: np.ndarray, config: APFConfig) -> np.ndarray:
    """Sum and cap APF repulsion from forward/side obstacles."""
    repulse = np.zeros(2, dtype=np.float64)
    for point in obstacles:
        xy = point[:2]
        distance = float(np.linalg.norm(xy))
        if not (0.0 < distance < config.non_target_influence_m):
            continue
        # Match jie_deamon's choice to ignore obstacles materially behind the
        # robot for repulsion; body-clearance emergency checking still applies.
        if xy[0] <= -config.swept_margin_m:
            continue
        force = (
            config.repulsion_gain
            * (1.0 / distance - 1.0 / config.non_target_influence_m)
            / (distance**2)
        )
        repulse -= force * xy / distance

    magnitude = float(np.linalg.norm(repulse))
    if magnitude > config.repulsion_cap:
        repulse *= config.repulsion_cap / magnitude
    return repulse
