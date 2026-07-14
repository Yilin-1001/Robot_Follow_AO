"""Tests for confidence computation."""

from __future__ import annotations

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    ConfidenceConfig,
    TargetConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain import (
    ConfidenceResult,
    GatedTargetObservation,
)
from robot_follow.lidar_alx_assisted_follow.tracking.confidence import (
    check_confidence_state,
    compute_confidence,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import LockState


def make_observation(
    point_count: int = 10,
    vertical_extent_m: float = 0.5,
    horizontal_footprint_m: float = 0.3,
    radial_mad_m: float = 0.05,
    center_xy: tuple[float, float] = (1.0, 0.0),
) -> GatedTargetObservation:
    """Helper to create a test observation."""
    support_points = np.array([
        [center_xy[0], center_xy[1], 0.5],
        [center_xy[0] + 0.05, center_xy[1], 0.55],
        [center_xy[0], center_xy[1] + 0.05, 0.52],
    ] * 3)[:point_count]

    target_mask = np.zeros(100, dtype=bool)
    target_mask[:point_count] = True

    return GatedTargetObservation(
        center_xy=center_xy,
        target_mask=target_mask,
        support_points_xyz=support_points,
        point_count=point_count,
        vertical_extent_m=vertical_extent_m,
        horizontal_footprint_m=horizontal_footprint_m,
        radial_mad_m=radial_mad_m,
        observed_at_s=1.0,
    )


class TestComputeConfidence:
    """Tests for compute_confidence function."""

    def test_selected_seed_has_c_t_1(self) -> None:
        """Selected seed observation should have c_t = 1.0."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        obs = make_observation()

        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        assert result.c_t == 1.0
        assert result.total > 0.8  # Should be high with all components

    def test_track_following_within_limits_has_c_t_1(self) -> None:
        """Accepted track within displacement and interval limits should have c_t = 1.0."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        obs = make_observation()

        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=False,
            center_displacement=0.1,
            frame_interval=0.1,
        )

        assert result.c_t == 1.0

    def test_excessive_displacement_sets_c_t_0(self) -> None:
        """Excessive center displacement should set c_t = 0.0."""
        config = ConfidenceConfig()
        target_config = TargetConfig(max_center_displacement_m=0.35)

        obs = make_observation()

        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=False,
            center_displacement=0.5,  # > 0.35
            frame_interval=0.1,
        )

        assert result.c_t == 0.0

    def test_excessive_frame_interval_sets_c_t_0(self) -> None:
        """Excessive frame interval should set c_t = 0.0."""
        config = ConfidenceConfig()
        target_config = TargetConfig(max_frame_interval_s=0.2)

        obs = make_observation()

        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=False,
            center_displacement=0.1,
            frame_interval=0.3,  # > 0.2
        )

        assert result.c_t == 0.0

    def test_geometric_c_g_scales_with_point_count(self) -> None:
        """c_g should increase with point count."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        # Low point count
        obs_low = make_observation(point_count=2)
        result_low = compute_confidence(
            observation=obs_low,
            target_center_xy=obs_low.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        # High point count
        obs_high = make_observation(point_count=50)
        result_high = compute_confidence(
            observation=obs_high,
            target_center_xy=obs_high.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        # More points should give higher c_g
        assert result_high.c_g >= result_low.c_g

    def test_alx_c_a_scales_with_proximity(self) -> None:
        """c_a should increase as ALX gets closer to target center."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        obs = make_observation(center_xy=(1.0, 0.0))

        # Close ALX
        result_close = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        # Far ALX
        result_far = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([2.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        assert result_close.c_a > result_far.c_a

    def test_no_observation_sets_c_t_c_g_0(self) -> None:
        """No observation should set c_t = 0 and c_g = 0."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        result = compute_confidence(
            observation=None,
            target_center_xy=(1.0, 0.0),
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=False,
        )

        assert result.c_t == 0.0
        assert result.c_g == 0.0

    def test_no_alx_sets_c_a_0(self) -> None:
        """No ALX data should set c_a = 0.0."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        obs = make_observation()

        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=None,
            alx_fresh=False,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        assert result.c_a == 0.0

    def test_total_clipped_to_0_1(self) -> None:
        """Total confidence should be clipped to [0, 1]."""
        config = ConfidenceConfig()
        target_config = TargetConfig()

        # Should never exceed 1.0
        obs = make_observation(point_count=9999)
        result = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config,
            target_config=target_config,
            is_selected_seed=True,
        )

        assert 0.0 <= result.total <= 1.0

    def test_weights_affect_total(self) -> None:
        """Different weight configs should produce different totals."""
        target_config = TargetConfig()

        obs = make_observation(center_xy=(1.0, 0.0))

        # Geometry-heavy config
        config_geom = ConfidenceConfig(
            weight_temporal=0.0,
            weight_geometry=1.0,
            weight_alx=0.0,
        )

        # ALX-heavy config
        config_alx = ConfidenceConfig(
            weight_temporal=0.0,
            weight_geometry=0.0,
            weight_alx=1.0,
        )

        result_geom = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config_geom,
            target_config=target_config,
            is_selected_seed=True,
        )

        result_alx = compute_confidence(
            observation=obs,
            target_center_xy=obs.center_xy,
            alx_body_xy=np.array([1.0, 0.0]),
            alx_fresh=True,
            config=config_alx,
            target_config=target_config,
            is_selected_seed=True,
        )

        # c_g and c_a should differ given same observation and matching ALX
        assert result_geom.total != result_alx.total


class TestCheckConfidenceState:
    """Tests for confidence state transitions."""

    def test_high_confidence_following_returns_ok(self) -> None:
        """High confidence in FOLLOWING state should return ok."""
        config = ConfidenceConfig()
        result = ConfidenceResult(
            c_t=1.0, c_g=0.9, c_a=0.95, total=0.95, alx_fresh=True, reason="nominal"
        )

        action, reason = check_confidence_state(result, LockState.FOLLOWING, config)

        assert action == "ok"

    def test_confidence_below_stop_requests_stop(self) -> None:
        """Confidence < stop_threshold should request stop."""
        config = ConfidenceConfig(stop=0.70)
        result = ConfidenceResult(
            c_t=0.0, c_g=0.3, c_a=0.0, total=0.5, alx_fresh=True, reason="low_confidence"
        )

        action, reason = check_confidence_state(result, LockState.FOLLOWING, config)

        assert action == "stop"

    def test_confidence_below_unlock_requests_unlock(self) -> None:
        """Confidence < unlock_threshold should request unlock."""
        config = ConfidenceConfig(unlock=0.45)
        result = ConfidenceResult(
            c_t=0.0, c_g=0.0, c_a=0.0, total=0.3, alx_fresh=True, reason="low_confidence"
        )

        action, reason = check_confidence_state(result, LockState.FOLLOWING, config)

        assert action == "unlock"

    def test_alx_stale_triggers_stop(self) -> None:
        """Stale ALX should trigger stop."""
        config = ConfidenceConfig()
        result = ConfidenceResult(
            c_t=1.0, c_g=0.9, c_a=0.0, total=0.65, alx_fresh=False, reason="alx_stale"
        )

        action, reason = check_confidence_state(result, LockState.FOLLOWING, config)

        assert action == "stop"

    def test_empty_state_returns_ok(self) -> None:
        """EMPTY state should return ok regardless of confidence."""
        config = ConfidenceConfig()
        result = ConfidenceResult(
            c_t=0.0, c_g=0.0, c_a=0.0, total=0.0, alx_fresh=True, reason="no_observation"
        )

        action, reason = check_confidence_state(result, LockState.EMPTY, config)

        assert action == "ok"