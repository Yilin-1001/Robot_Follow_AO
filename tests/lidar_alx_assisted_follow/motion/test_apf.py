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


def test_close_target_retreats_at_configured_conservative_speed() -> None:
    """A nearby ALX target causes verified distance-recovery retreat, not APF circling."""
    result = compute_apf(
        make_input(
            np.array([[0.70, 0.0, 0.2]]),
            target=(0.70, 0.0),
            target_mask=np.array([True]),
        ),
        APFConfig(
            automatic_target_retreat_enabled=True,
            target_retreat_start_distance_m=0.85,
            target_retreat_exit_distance_m=1.00,
            target_retreat_speed_mps=0.10,
        ),
        OutputMotionConfig(max_reverse_vx=0.10),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "target_retreat"
    assert result.intent.vx == -0.10
    assert result.intent.vy == 0.0


def test_active_target_retreat_continues_until_exit_distance() -> None:
    """The runtime hysteresis flag keeps retreat active between 0.85 m and 1.0 m."""
    input_data = make_input(
        np.array([[0.90, 0.0, 0.2]]),
        target=(0.90, 0.0),
        target_mask=np.array([True]),
    )
    input_data = APFInput(**{**input_data.__dict__, "target_retreat_active": True})
    result = compute_apf(
        input_data,
        APFConfig(automatic_target_retreat_enabled=True),
        OutputMotionConfig(max_reverse_vx=0.10),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "target_retreat"


def test_close_target_retreat_stops_when_rear_path_is_blocked() -> None:
    """A close person alone is not a stop; an unsafe reverse sweep is."""
    result = compute_apf(
        make_input(
            np.array([[0.70, 0.0, 0.2], [-0.45, 0.0, 0.2]]),
            target=(0.70, 0.0),
            target_mask=np.array([True, False]),
        ),
        APFConfig(automatic_target_retreat_enabled=True),
        OutputMotionConfig(max_reverse_vx=0.10),
    )
    assert result.stop_reason == StopReason.TARGET_TOO_CLOSE
    assert result.intent.vx == 0.0


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


def test_obstacle_in_target_corridor_steers_sideways() -> None:
    """The robot-to-target corridor begins a sidestep before near-field APF."""
    points = np.array([[2.5, 0.0, 0.2], [1.75, -0.20, 0.2]])
    result = compute_apf(
        make_input(points, target=(2.5, 0.0), target_mask=np.array([True, False])),
        APFConfig(
            non_target_influence_m=1.8,
            lateral_avoidance_half_width_m=0.8,
            non_target_hard_stop_clearance_m=0.25,
            slowdown_clearance_m=0.60,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vy > 0.0


def test_obstacle_beyond_target_corridor_does_not_create_sidestep() -> None:
    points = np.array([[1.8, 0.0, 0.2], [1.81, -0.20, 0.2]])
    result = compute_apf(
        make_input(points, target=(1.8, 0.0), target_mask=np.array([True, False])),
        APFConfig(
            non_target_influence_m=1.8,
            lateral_avoidance_half_width_m=0.8,
            repulsion_gain=0.0,
            non_target_hard_stop_clearance_m=0.25,
            slowdown_clearance_m=0.60,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.vy == 0.0


def test_obstacle_in_angled_target_corridor_is_considered_for_sidestep() -> None:
    """The corridor rotates with ALX instead of remaining fixed along +x."""
    points = np.array([[1.5, 1.0, 0.2], [0.53, 0.65, 0.2]])
    result = compute_apf(
        make_input(points, target=(1.5, 1.0), target_mask=np.array([True, False])),
        APFConfig(
            lateral_avoidance_half_width_m=0.30,
            repulsion_gain=0.0,
            non_target_hard_stop_clearance_m=0.05,
            side_hard_clearance_m=0.05,
            slowdown_clearance_m=0.20,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    # This point is outside the old fixed-forward rectangle (y > 0.30), but
    # lies in the rotated rectangle from the robot to the ALX target.
    assert result.intent.vy < 0.0


def test_obstacle_outside_local_processing_region_is_not_used_by_apf() -> None:
    """Far side points remain available for visualization but skip APF work."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [1.0, 1.30, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            obstacle_x_min_m=-0.8,
            obstacle_x_max_m=2.5,
            obstacle_abs_y_max_m=1.2,
            repulsion_gain=0.0,
        ),
        default_output(),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "follow"


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


def test_side_clearance_can_be_lower_than_front_hard_clearance() -> None:
    """A side wall may use a smaller corridor clearance without weakening front stop."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.0, 0.45, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.25,
            side_hard_clearance_m=0.20,
        ),
        default_output(),
    )
    assert result.stop_reason is None


def test_hard_clearance_uses_safe_automatic_escape_away_from_front_obstacle() -> None:
    """A clear rear path permits only a low-speed retreat."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.50, 0.0, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.25,
            automatic_escape_enabled=True,
            escape_speed_mps=0.08,
            escape_probe_distance_m=0.08,
            escape_min_clearance_gain_m=0.02,
        ),
        OutputMotionConfig(
            max_vx=0.20,
            max_reverse_vx=0.08,
            max_abs_vy=0.12,
            max_abs_omega=0.45,
        ),
    )
    assert result.stop_reason is None
    assert result.intent.reason == "automatic_escape"
    assert result.intent.vx == -0.08
    assert result.intent.vy == 0.0


def test_automatic_escape_refuses_to_move_toward_rear_obstacle() -> None:
    """A point behind the robot invalidates the backward escape sweep."""
    result = compute_apf(
        make_input(
            np.array(
                [[1.5, 0.0, 0.2], [0.50, 0.0, 0.2], [-0.45, 0.0, 0.2]]
            ),
            target_mask=np.array([True, False, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.25,
            automatic_escape_enabled=True,
            escape_speed_mps=0.08,
            escape_probe_distance_m=0.08,
            escape_min_clearance_gain_m=0.02,
        ),
        OutputMotionConfig(max_reverse_vx=0.08),
    )
    assert result.stop_reason == StopReason.NON_TARGET_HARD_CLEARANCE
    assert result.intent.vx == 0.0


def test_automatic_escape_does_not_sidestep_away_from_side_wall() -> None:
    """A lateral hard-clearance breach must not cause left/right oscillation."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.0, 0.40, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.25,
            side_hard_clearance_m=0.20,
            automatic_escape_enabled=True,
        ),
        OutputMotionConfig(max_reverse_vx=0.08),
    )
    assert result.stop_reason == StopReason.NON_TARGET_HARD_CLEARANCE
    assert result.intent.vx == 0.0
    assert result.intent.vy == 0.0


def test_clearance_recovery_uses_body_clearance_repulsion_without_target_motion() -> None:
    """A side hard-clearance breach produces only a bounded side-step."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.0, 0.40, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.20,
            side_hard_clearance_m=0.20,
            body_half_width_m=0.225,
            clearance_recovery_enabled=True,
            repulsion_gain=0.01,
        ),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
    )

    assert result.stop_reason is None
    assert result.intent.reason == "clearance_recovery"
    assert result.intent.vx == 0.0
    assert result.intent.vy < 0.0
    assert result.intent.omega == 0.0
    assert 0.10 <= np.hypot(result.intent.vx, result.intent.vy) <= 0.15


def test_clearance_recovery_holds_minimum_speed_for_slight_intrusion() -> None:
    """A barely breached side boundary still commands G1's 0.10 m/s minimum."""
    result = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.0, 0.424, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.20,
            side_hard_clearance_m=0.20,
            body_half_width_m=0.225,
            clearance_recovery_enabled=True,
        ),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
    )

    assert result.stop_reason is None
    assert result.intent.reason == "clearance_recovery"
    assert np.isclose(np.hypot(result.intent.vx, result.intent.vy), 0.10025)


def test_clearance_recovery_continues_until_exit_margin_is_clear() -> None:
    """Recovery hysteresis keeps pure repulsion until clearance exceeds 0.22 m."""
    input_data = make_input(
        np.array([[1.5, 0.0, 0.2], [0.0, 0.430, 0.2]]),
        target_mask=np.array([True, False]),
    )
    input_data = APFInput(
        **{**input_data.__dict__, "clearance_recovery_active": True}
    )
    result = compute_apf(
        input_data,
        APFConfig(
            non_target_hard_stop_clearance_m=0.20,
            side_hard_clearance_m=0.20,
            body_half_width_m=0.225,
            clearance_recovery_enabled=True,
        ),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
    )

    assert result.stop_reason is None
    assert result.intent.reason == "clearance_recovery"


def test_clearance_recovery_does_not_move_for_rear_only_or_body_internal_points() -> None:
    """First release never moves forward or reacts to a point inside the body."""
    rear = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [-0.49, 0.0, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(
            non_target_hard_stop_clearance_m=0.20,
            clearance_recovery_enabled=True,
        ),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
    )
    internal = compute_apf(
        make_input(
            np.array([[1.5, 0.0, 0.2], [0.20, 0.0, 0.2]]),
            target_mask=np.array([True, False]),
        ),
        APFConfig(clearance_recovery_enabled=True),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
    )

    assert rear.stop_reason == StopReason.NON_TARGET_HARD_CLEARANCE
    assert internal.stop_reason == StopReason.NON_TARGET_HARD_CLEARANCE


def test_clearance_recovery_does_not_restart_after_its_ten_second_timeout() -> None:
    """A still-blocked cloud must not open another automatic recovery window."""
    input_data = make_input(
        np.array([[1.5, 0.0, 0.2], [0.0, 0.40, 0.2]]),
        target_mask=np.array([True, False]),
    )
    input_data = APFInput(
        **{**input_data.__dict__, "clearance_recovery_timed_out": True}
    )
    result = compute_apf(
        input_data,
        APFConfig(
            non_target_hard_stop_clearance_m=0.20,
            side_hard_clearance_m=0.20,
            body_half_width_m=0.225,
            clearance_recovery_enabled=True,
        ),
        OutputMotionConfig(max_reverse_vx=0.20, max_abs_vy=0.20),
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
