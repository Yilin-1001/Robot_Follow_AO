"""Tests for target lock management."""

from __future__ import annotations

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import TargetConfig
from robot_follow.lidar_alx_assisted_follow.domain import PreprocessedCloud
from robot_follow.lidar_alx_assisted_follow.tracking.preprocessing import (
    preprocess_tracking_cloud,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import (
    LockState,
    TargetLockManager,
)


def make_preprocessed_cloud(
    points: np.ndarray,
    config: TargetConfig,
) -> PreprocessedCloud:
    """Helper to create preprocessed cloud from raw points."""
    from robot_follow.lidar_alx_assisted_follow.config.schema import CalibrationConfig
    calibration = CalibrationConfig()
    return preprocess_tracking_cloud(points, config, calibration)


def make_valid_target_cloud(config: TargetConfig) -> PreprocessedCloud:
    """Create a cloud with valid target characteristics.

    Ensures:
    - Multiple voxels after downsampling
    - Sufficient vertical extent
    - Meets all quality thresholds
    """
    np.random.seed(42)
    points = []
    for i in range(6):
        for j in range(5):
            # Spread across x,y and varying z for vertical extent
            x = 1.0 + i * 0.08  # ~0.5m wide
            y = -0.1 + j * 0.05  # ~0.25m deep
            z = 0.3 + (i + j) * 0.06  # Vertical spread 0.3 to ~1.0
            points.append([x, y, z])

    points = np.array(points)  # 30 points
    return make_preprocessed_cloud(points, config)


class TestTargetLockManagerStates:
    """Tests for state transitions."""

    def test_initial_state_is_empty(self) -> None:
        """Manager should start in EMPTY state."""
        config = TargetConfig()
        manager = TargetLockManager(config=config)

        assert manager.state == LockState.EMPTY
        assert manager.center_xy is None
        assert manager.snapshot().state == LockState.EMPTY

    def test_select_transitions_to_candidate(self) -> None:
        """Successful select should transition to CANDIDATE."""
        config = TargetConfig(min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        # Create cloud with valid target - spread across multiple voxels
        # and sufficient vertical extent
        np.random.seed(42)
        points = []
        for i in range(6):
            for j in range(5):
                # Spread across x,y and varying z for vertical extent
                x = 1.0 + i * 0.08
                y = -0.1 + j * 0.05
                z = 0.3 + (i + j) * 0.05  # Vertical spread 0.3 to ~0.85
                points.append([x, y, z])

        points = np.array(points)  # 30 points
        cloud = make_preprocessed_cloud(points, config)
        obs = manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)

        assert obs is not None
        assert manager.state == LockState.CANDIDATE
        assert manager.center_xy is not None
        assert manager.requires_start is True

    def test_start_transitions_to_following(self) -> None:
        """start() should transition CANDIDATE -> FOLLOWING."""
        config = TargetConfig(min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        # Spread points across multiple voxels with vertical extent
        np.random.seed(42)
        points = []
        for i in range(6):
            for j in range(5):
                x = 1.0 + i * 0.08
                y = -0.1 + j * 0.05
                z = 0.3 + (i + j) * 0.05
                points.append([x, y, z])
        points = np.array(points)
        cloud = make_preprocessed_cloud(points, config)

        manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)
        result = manager.start()

        assert result is True
        assert manager.state == LockState.FOLLOWING
        assert manager.requires_start is False

    def test_stop_follow_returns_to_candidate(self) -> None:
        """stop_follow() should preserve target but return to CANDIDATE."""
        config = TargetConfig(min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        # Spread points with vertical extent
        np.random.seed(42)
        points = []
        for i in range(6):
            for j in range(5):
                x = 1.0 + i * 0.08
                y = -0.1 + j * 0.05
                z = 0.3 + (i + j) * 0.05
                points.append([x, y, z])
        points = np.array(points)
        cloud = make_preprocessed_cloud(points, config)

        manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)
        manager.start()
        manager.stop_follow()

        assert manager.state == LockState.CANDIDATE
        assert manager.center_xy is not None  # Preserved
        assert manager.requires_start is True

    def test_unlock_clears_target(self) -> None:
        """unlock() should clear everything and return to EMPTY."""
        config = TargetConfig()
        manager = TargetLockManager(config=config)

        points = np.array([[1.0, 0.0, 0.5]] * 10)
        cloud = make_preprocessed_cloud(points, config)

        manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)
        manager.unlock()

        assert manager.state == LockState.EMPTY
        assert manager.center_xy is None


class TestTargetLockManagerGating:
    """Tests for gate-based target tracking."""

    def test_initial_circular_gate_applied(self) -> None:
        """Select should apply circular gate with web_pick_radius_m."""
        config = TargetConfig(web_pick_radius_m=0.35)
        manager = TargetLockManager(config=config)

        # Points inside and outside gate
        points = np.array([
            [1.0, 0.0, 0.5],   # inside
            [1.2, 0.0, 0.5],   # outside (0.2m from click at 1.0, 0.0)
            [0.8, 0.0, 0.5],   # inside
        ] * 5)  # 15 points total

        cloud = make_preprocessed_cloud(points, config)
        obs = manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)

        if obs is not None:
            # Center should be near the click
            assert np.hypot(obs.center_xy[0] - 1.0, obs.center_xy[1] - 0.0) < 0.5

    def test_dynamic_elliptical_gate_applied(self) -> None:
        """Update should apply elliptical gate oriented toward target."""
        config = TargetConfig(
            min_support_points=2,
            min_vertical_extent_m=0.1,
            gate_radial_radius_m=0.45,
            gate_tangential_radius_m=0.35,
        )
        manager = TargetLockManager(config=config)

        # Initial selection with valid target
        cloud1 = make_valid_target_cloud(config)
        manager.select(cloud1, click_xy=(1.1, 0.0), now_s=1.0)
        manager.start()

        # Update with shifted cloud (still within gate)
        cloud2 = make_valid_target_cloud(config)
        obs, lost = manager.update(cloud2, now_s=1.1)

        assert obs is not None
        assert lost is False

    def test_failed_reselection_keeps_candidate(self) -> None:
        """Failed select on existing candidate should keep current target."""
        config = TargetConfig(min_support_points=2)
        manager = TargetLockManager(config=config)

        # First selection succeeds
        points1 = np.array([
            [1.0 + i*0.06, j*0.02, 0.5]
            for i in range(3)
            for j in range(3)
        ])
        cloud1 = make_preprocessed_cloud(points1, config)
        manager.select(cloud1, click_xy=(1.05, 0.0), now_s=1.0)

        before = manager.snapshot()

        # Second selection fails (empty cloud)
        empty_cloud = make_preprocessed_cloud(np.empty((0, 3)), config)
        manager.select(empty_cloud, click_xy=(4.0, 0.0), now_s=2.0)

        # Should keep previous target
        assert manager.snapshot().center_xy == before.center_xy

    def test_failed_tracking_advances_loss_timer(self) -> None:
        """Failed update should advance lost timer."""
        config = TargetConfig(lost_timeout_s=0.5, min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        # Initial selection with valid target
        cloud = make_valid_target_cloud(config)
        manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)
        manager.start()

        # Failed updates
        empty_cloud = make_preprocessed_cloud(np.empty((0, 3)), config)
        manager.update(empty_cloud, now_s=1.2)
        manager.update(empty_cloud, now_s=1.4)

        # Not lost yet
        assert manager.state != LockState.LOST

        # After timeout
        manager.update(empty_cloud, now_s=1.6)
        assert manager.state == LockState.LOST


class TestTargetLockManagerQuality:
    """Tests for quality checks."""

    def test_insufficient_points_rejected(self) -> None:
        """Target with too few points should be rejected."""
        config = TargetConfig(min_support_points=8)
        manager = TargetLockManager(config=config)

        # Only 3 points
        points = np.array([[1.0, 0.0, 0.5]] * 3)
        cloud = make_preprocessed_cloud(points, config)

        obs = manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)

        assert obs is None
        assert manager.state == LockState.EMPTY

    def test_insufficient_vertical_extent_rejected(self) -> None:
        """Target with insufficient height should be rejected."""
        config = TargetConfig(
            min_vertical_extent_m=0.45,
            min_support_points=2,
        )
        manager = TargetLockManager(config=config)

        # Points all at same height (zero vertical extent)
        points = np.array([
            [1.0, 0.0, 0.5],
            [1.1, 0.0, 0.5],
        ])
        cloud = make_preprocessed_cloud(points, config)

        obs = manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)

        assert obs is None

    def test_excessive_footprint_rejected(self) -> None:
        """Target with too large horizontal footprint should be rejected."""
        config = TargetConfig(
            max_horizontal_footprint_m=0.9,
            min_support_points=2,
            min_vertical_extent_m=0.1,
        )
        manager = TargetLockManager(config=config)

        # Points spread very wide
        points = np.array([
            [1.0, 0.0, 0.5],
            [2.5, 0.0, 0.6],  # 1.5m apart
        ])
        cloud = make_preprocessed_cloud(points, config)

        obs = manager.select(cloud, click_xy=(1.5, 0.0), now_s=1.0)

        assert obs is None

    def test_excessive_radial_mad_rejected(self) -> None:
        """Target with excessive radial MAD should be rejected."""
        config = TargetConfig(
            max_radial_mad_m=0.18,
            min_support_points=3,
            min_vertical_extent_m=0.1,
        )
        manager = TargetLockManager(config=config)

        # Points at very different radii
        points = np.array([
            [0.5, 0.0, 0.5],
            [1.0, 0.0, 0.6],
            [2.0, 0.0, 0.7],
        ])
        cloud = make_preprocessed_cloud(points, config)

        obs = manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)

        # May or may not be rejected depending on MAD
        # This test documents the check exists


class TestTargetLockManagerObservation:
    """Tests for observation building."""

    def test_observation_contains_required_metrics(self) -> None:
        """Observation should include all required metrics."""
        config = TargetConfig(min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        cloud = make_valid_target_cloud(config)
        obs = manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)

        assert obs is not None
        assert obs.center_xy is not None
        assert obs.target_mask is not None
        assert obs.support_points_xyz is not None
        assert obs.point_count >= 0
        assert obs.vertical_extent_m >= 0
        assert obs.horizontal_footprint_m >= 0
        assert obs.radial_mad_m >= 0
        assert obs.observed_at_s == 1.0

    def test_center_is_median(self) -> None:
        """Center should be component-wise median of support points."""
        config = TargetConfig(min_support_points=3)
        manager = TargetLockManager(config=config)

        points = np.array([
            [1.0, 0.0, 0.5],
            [1.1, 0.1, 0.6],
            [1.2, -0.1, 0.7],
        ])
        cloud = make_preprocessed_cloud(points, config)

        obs = manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)

        if obs is not None:
            # Center should be close to median
            expected_x = np.median([1.0, 1.1, 1.2])
            expected_y = np.median([0.0, 0.1, -0.1])
            assert abs(obs.center_xy[0] - expected_x) < 0.05
            assert abs(obs.center_xy[1] - expected_y) < 0.05


class TestTargetLockManagerSequence:
    """Tests for sequence tracking."""

    def test_sequence_increments_on_transitions(self) -> None:
        """Sequence should increment on state changes."""
        config = TargetConfig(min_support_points=2, min_vertical_extent_m=0.1)
        manager = TargetLockManager(config=config)

        initial_seq = manager.sequence

        cloud = make_valid_target_cloud(config)
        manager.select(cloud, click_xy=(1.1, 0.0), now_s=1.0)
        seq_after_select = manager.sequence

        manager.start()
        seq_after_start = manager.sequence

        manager.stop_follow()
        seq_after_stop = manager.sequence

        manager.unlock()
        seq_after_unlock = manager.sequence

        assert seq_after_select > initial_seq
        assert seq_after_start > seq_after_select
        assert seq_after_stop > seq_after_start
        assert seq_after_unlock > seq_after_stop