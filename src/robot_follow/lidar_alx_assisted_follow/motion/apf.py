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
from typing import Optional

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    OutputMotionConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    MotionIntent,
    StopReason,
)
from robot_follow.lidar_alx_assisted_follow.motion.avoidance_types import (
    AvoidanceInput,
    AvoidanceResult,
)


# Compatibility aliases: callers of the existing APF module keep working while
# all avoidance modes now share one cloud-synchronous contract.
APFInput = AvoidanceInput
APFResult = AvoidanceResult


def compute_apf(
    input_data: AvoidanceInput,
    config: APFConfig,
    output_config: OutputMotionConfig,
) -> AvoidanceResult:
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
    obstacles = _get_non_target_points(input_data.cloud.points_xyz, input_data.target_mask)
    obstacles = _filter_obstacle_processing_region(obstacles, config)
    if config.automatic_target_retreat_enabled and (
        input_data.target_retreat_active
        or target_distance < config.target_retreat_start_distance_m
    ):
        retreat_intent = _safe_target_retreat_intent(
            target_xy,
            obstacles,
            input_data.sequence,
            config,
            output_config,
        )
        if retreat_intent is not None:
            return APFResult(retreat_intent, stop_reason=None)
        return APFResult(
            _zero_intent(input_data.sequence), StopReason.TARGET_TOO_CLOSE
        )

    if target_distance <= config.target_stop_distance_m:
        return APFResult(
            _zero_intent(input_data.sequence), StopReason.TARGET_TOO_CLOSE
        )

    min_clearance = _minimum_body_clearance(obstacles, config)
    if _has_hard_clearance_violation(obstacles, config):
        escape_intent = _safe_escape_intent(
            obstacles,
            input_data.sequence,
            config,
            output_config,
        )
        if escape_intent is not None:
            return APFResult(escape_intent, stop_reason=None)
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

    # 2. Lateral avoidance: points in the configured robot-local rectangle
    # steer sideways before they enter the near-field APF zone. An obstacle on
    # the right produces positive vy (move left), and vice versa.
    vy_corridor = _corridor_lateral_velocity(obstacles, config)

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


def _filter_obstacle_processing_region(
    obstacles: np.ndarray, config: APFConfig
) -> np.ndarray:
    """Keep only the robot-local region relevant to collision and retreat."""
    if obstacles.size == 0:
        return obstacles
    mask = (
        (obstacles[:, 0] >= config.obstacle_x_min_m)
        & (obstacles[:, 0] <= config.obstacle_x_max_m)
        & (np.abs(obstacles[:, 1]) <= config.obstacle_abs_y_max_m)
    )
    return obstacles[mask]


def _minimum_body_clearance(obstacles: np.ndarray, config: APFConfig) -> float:
    if len(obstacles) == 0:
        return float("inf")
    return min(_compute_body_clearance(point[:2], config) for point in obstacles)


def _safe_escape_intent(
    obstacles: np.ndarray,
    sequence: int,
    config: APFConfig,
    output_config: OutputMotionConfig,
) -> Optional[MotionIntent]:
    """Build an escape command only if a short point-cloud sweep is safe.

    Only a forward emergency obstacle may trigger automatic escape.  The
    command is a straight, low-speed reverse rather than a lateral sidestep,
    preventing left/right oscillation between walls in narrow corridors.
    """
    if not config.automatic_escape_enabled or obstacles.size == 0:
        return None

    obstacle_xy = obstacles[:, :2]
    clearances = np.array(
        [_compute_body_clearance(point, config) for point in obstacle_xy],
        dtype=np.float64,
    )
    required_clearances = np.array(
        [_required_hard_clearance(point, config) for point in obstacle_xy],
        dtype=np.float64,
    )
    violating = clearances <= required_clearances
    forward_violating = violating & (obstacle_xy[:, 0] > 0.0)
    if not np.any(forward_violating):
        return None

    forward_indices = np.where(forward_violating)[0]
    critical_index = forward_indices[
        int(np.argmin((clearances - required_clearances)[forward_indices]))
    ]
    if obstacle_xy[critical_index, 0] <= 1e-6:
        return None

    direction = np.array([-1.0, 0.0], dtype=np.float64)
    if not _escape_path_is_safe(
        obstacle_xy,
        clearances,
        direction,
        config,
        config.escape_probe_distance_m,
    ):
        return None

    vx = -min(config.escape_speed_mps, output_config.max_reverse_vx)
    if vx >= -1e-6:
        return None

    return MotionIntent(
        sequence=sequence,
        vx=vx,
        vy=0.0,
        omega=0.0,
        reason="automatic_escape",
        generated_at_s=time.monotonic(),
    )


def _safe_target_retreat_intent(
    target_xy: np.ndarray,
    obstacles: np.ndarray,
    sequence: int,
    config: APFConfig,
    output_config: OutputMotionConfig,
) -> Optional[MotionIntent]:
    """Retreat from a too-close ALX target only along a verified clear path."""
    target_distance = float(np.linalg.norm(target_xy))
    if target_distance <= 1e-6:
        return None

    direction = -target_xy / target_distance
    if obstacles.size:
        obstacle_xy = obstacles[:, :2]
        clearances = np.array(
            [_compute_body_clearance(point, config) for point in obstacle_xy],
            dtype=np.float64,
        )
        if not _escape_path_is_safe(
            obstacle_xy,
            clearances,
            direction,
            config,
            config.target_retreat_probe_distance_m,
        ):
            return None

    velocity_xy = direction * config.target_retreat_speed_mps
    vx = float(np.clip(velocity_xy[0], -output_config.max_reverse_vx, output_config.max_vx))
    vy = float(
        np.clip(
            velocity_xy[1],
            -output_config.max_abs_vy,
            output_config.max_abs_vy,
        )
    )
    if math.hypot(vx, vy) <= 1e-6:
        return None

    return MotionIntent(
        sequence=sequence,
        vx=vx,
        vy=vy,
        omega=0.0,
        reason="target_retreat",
        generated_at_s=time.monotonic(),
    )


def _escape_path_is_safe(
    obstacle_xy: np.ndarray,
    initial_clearances: np.ndarray,
    direction: np.ndarray,
    config: APFConfig,
    probe_distance_m: float,
) -> bool:
    """Verify a short translational sweep against all observed obstacles."""
    for fraction in (0.25, 0.50, 0.75, 1.00):
        displacement = direction * probe_distance_m * fraction
        future_points = obstacle_xy - displacement
        future_clearances = np.array(
            [
                _compute_body_clearance(point, config)
                for point in future_points
            ],
            dtype=np.float64,
        )

        # A retreat must not reduce clearance to any observed point.
        if np.any(future_clearances + 1e-6 < initial_clearances):
            return False

        initial_required = np.array(
            [_required_hard_clearance(point, config) for point in obstacle_xy],
            dtype=np.float64,
        )
        future_required = np.array(
            [_required_hard_clearance(point, config) for point in future_points],
            dtype=np.float64,
        )

        # A point initially outside its direction-specific emergency boundary
        # must remain outside after the robot moves.
        previously_safe = initial_clearances > initial_required
        if np.any(future_clearances[previously_safe] <= future_required[previously_safe]):
            return False

        if fraction == 1.00:
            return bool(
                np.all(
                    future_clearances
                    >= future_required + config.escape_min_clearance_gain_m
                )
            )

    return False


def _compute_body_clearance(point_xy: np.ndarray, config: APFConfig) -> float:
    """Return exterior clearance from the rectangular G1 body envelope."""
    dx, dy = _body_clearance_components(point_xy, config)
    return float(math.hypot(dx, dy))


def _body_clearance_components(
    point_xy: np.ndarray, config: APFConfig
) -> tuple[float, float]:
    """Return longitudinal and lateral distances outside the body rectangle."""
    dx = max(abs(float(point_xy[0])) - config.body_half_length_m, 0.0)
    dy = max(abs(float(point_xy[1])) - config.body_half_width_m, 0.0)
    return dx, dy


def _required_hard_clearance(point_xy: np.ndarray, config: APFConfig) -> float:
    """Use the side clearance only when the point is predominantly lateral."""
    dx, dy = _body_clearance_components(point_xy, config)
    if dy > dx:
        return config.side_hard_clearance_m
    return config.non_target_hard_stop_clearance_m


def _has_hard_clearance_violation(obstacles: np.ndarray, config: APFConfig) -> bool:
    """Return True when any point violates its directional hard clearance."""
    # A point specified exactly on a configured boundary (for example
    # 0.45 - 0.25 == 0.20 m) must not flip between stop and pass solely from
    # binary floating-point representation.  This is many orders below the
    # sensor uncertainty and does not materially relax the safety boundary.
    comparison_tolerance_m = 1e-9
    return any(
        _compute_body_clearance(point[:2], config)
        < _required_hard_clearance(point[:2], config) - comparison_tolerance_m
        for point in obstacles
    )


def _corridor_lateral_velocity(
    obstacles: np.ndarray,
    config: APFConfig,
) -> float:
    """Compute lateral centering in the robot-local avoidance rectangle."""
    half_width = config.lateral_avoidance_half_width_m
    left_inner = -half_width
    right_inner = half_width

    for point in obstacles:
        xy = point[:2]
        longitudinal = float(xy[0])
        lateral = float(xy[1])
        if (
            0.0 <= longitudinal <= config.lateral_avoidance_length_m
            and abs(lateral) <= half_width
        ):
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
