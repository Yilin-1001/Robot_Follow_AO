"""Tests for continuous jie_deamon-style ALX follow/avoidance."""

from __future__ import annotations

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    OutputMotionConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    ConfidenceResult,
    PreprocessedCloud,
    StopReason,
)
from robot_follow.lidar_alx_assisted_follow.motion.apf import APFInput, compute_apf


def make_input(
    points: np.ndarray,
    target: tuple[float, float] = (1.5, 0.0),
    target_mask: np.ndarray | None = None,
    sequence: int = 1,
) -> APFInput:
    if target_mask is None:
        target_mask = np.zeros(len(points), dtype=bool)
    return APFInput(
        cloud=PreprocessedCloud(
            points_xyz=points,
            source_indices=np.arange(len(points), dtype=np.int64),
        ),
        target_center_xy=target,
        target_mask=target_mask,
        sequence=sequence,
    )


def default_output() -> OutputMotionConfig:
    return OutputMotionConfig(max_vx=0.20, max_abs_vy=0.12, max_abs_omega=0.45)


def test_target_at_stop_distance_stops() -> None:
    result = compute_apf(
        make_input(np.array([[0.8, 0.0, 0.2]]), target=(0.8, 0.0), target_mask=np.array([True])),
        APFConfig(target_stop_distance_m=0.8),
        default_output(),
    )
    assert result.stop_reason == StopReason.TARGET_TOO_CLOSE
    assert result.intent.vx == result.intent.vy == result.intent.omega == 0.0


def test_clear_target_generates_forward_follow_command() -> None:
    result = compute_apf(
        make_input(np.array([[1.5, 0.0, 0.2]]), target_mask=np.array([True])),
        APFConfig(repulsion_gain=0.0),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "follow"
    assert result.intent.vx > 0.0
    assert result.intent.vy == 0.0


def test_right_corridor_obstacle_steers_left_instead_of_stopping() -> None:
    points = np.array([[1.5, 0.0, 0.2], [0.70, -0.15, 0.2]])
    result = compute_apf(
        make_input(points, target_mask=np.array([True, False])),
        APFConfig(non_target_hard_stop_clearance_m=0.25, slowdown_clearance_m=0.60),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "follow_avoid"
    assert result.intent.vy > 0.0
    assert result.intent.vx > 0.0


def test_left_corridor_obstacle_steers_right_instead_of_stopping() -> None:
    points = np.array([[1.5, 0.0, 0.2], [0.70, 0.15, 0.2]])
    result = compute_apf(
        make_input(points, target_mask=np.array([True, False])),
        APFConfig(non_target_hard_stop_clearance_m=0.25, slowdown_clearance_m=0.60),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vy < 0.0


def test_obstacle_at_1_8m_in_lateral_avoidance_rectangle_steers_sideways() -> None:
    """The long-range robot-local zone begins a sidestep before near-field APF."""
    points = np.array([[2.5, 0.0, 0.2], [1.75, -0.20, 0.2]])
    result = compute_apf(
        make_input(points, target=(2.5, 0.0), target_mask=np.array([True, False])),
        APFConfig(
            non_target_influence_m=1.8,
            lateral_avoidance_length_m=1.8,
            lateral_avoidance_half_width_m=0.8,
            non_target_hard_stop_clearance_m=0.25,
            slowdown_clearance_m=0.60,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vy > 0.0


def test_obstacle_outside_lateral_avoidance_rectangle_does_not_create_sidestep() -> None:
    points = np.array([[2.5, 0.0, 0.2], [1.81, -0.20, 0.2]])
    result = compute_apf(
        make_input(points, target=(2.5, 0.0), target_mask=np.array([True, False])),
        APFConfig(
            non_target_influence_m=1.8,
            lateral_avoidance_length_m=1.8,
            lateral_avoidance_half_width_m=0.8,
            repulsion_gain=0.0,
            non_target_hard_stop_clearance_m=0.25,
            slowdown_clearance_m=0.60,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vy == 0.0


def test_obstacle_reduces_forward_speed_but_keeps_lateral_authority() -> None:
    clear = compute_apf(
        make_input(np.array([[1.5, 0.0, 0.2]]), target_mask=np.array([True])),
        APFConfig(repulsion_gain=0.0),
        default_output(),
    )
    obstacle = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.70, 0.0, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(non_target_hard_stop_clearance_m=0.25, slowdown_clearance_m=0.60),
        default_output(),
    )
    assert obstacle.stop_reason is None
    assert 0.0 < obstacle.intent.vx < clear.intent.vx
    assert abs(obstacle.intent.vy) > 0.0


def test_emergency_body_clearance_still_stops() -> None:
    # Body half-length is 0.30 m. An obstacle at x=0.50 m is only 0.20 m
    # outside the body and must remain an emergency stop.
    points = np.array([[1.5, 0.0, 0.2], [0.50, 0.0, 0.2]])
    result = compute_apf(
        make_input(points, target_mask=np.array([True, False])),
        APFConfig(non_target_hard_stop_clearance_m=0.25),
        default_output(),
    )
    assert result.stop_reason == StopReason.NON_TARGET_HARD_CLEARANCE
    assert result.intent.vx == result.intent.vy == result.intent.omega == 0.0


def test_target_bearing_is_fused_as_yaw_while_translating() -> None:
    result = compute_apf(
        make_input(
            np.array([[1.2, 0.8, 0.2]]),
            target=(1.2, 0.8),
            target_mask=np.array([True]),
        ),
        APFConfig(repulsion_gain=0.0),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vx > 0.0
    assert result.intent.omega > 0.0


def test_target_mask_excludes_alx_returns_from_avoidance() -> None:
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2]]),
            target_mask=np.array([True]),
        ),
        APFConfig(),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "follow"


def test_legacy_confidence_is_not_a_motion_gate() -> None:
    low_confidence = ConfidenceResult(0.0, 0.0, 0.0, 0.1, True, "legacy")
    input_data = make_input(
        np.array([[1.5, 0.0, 0.2]]), target_mask=np.array([True])
    )
    input_data = APFInput(**{**input_data.__dict__, "confidence": low_confidence})
    result = compute_apf(input_data, APFConfig(), default_output())
    assert result.stop_reason is None
    assert result.intent.vx > 0.0


def test_invalid_or_empty_cloud_stops() -> None:
    result = compute_apf(
        make_input(np.empty((0, 3)), target_mask=np.array([], dtype=bool)),
        APFConfig(),
        default_output(),
    )
    assert result.stop_reason == StopReason.INVALID_INPUT


def test_sequence_is_preserved() -> None:
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2]]), target_mask=np.array([True]), sequence=42
        ),
        APFConfig(),
        default_output(),
    )
    assert result.intent.sequence == 42
