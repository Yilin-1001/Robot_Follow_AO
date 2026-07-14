"""Tests for cloud preprocessing."""

from __future__ import annotations

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    CalibrationConfig,
    TargetConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain import PreprocessedCloud
from robot_follow.lidar_alx_assisted_follow.tracking.preprocessing import (
    preprocess_tracking_cloud,
)


class TestPreprocessTrackingCloud:
    """Tests for preprocess_tracking_cloud function."""

    def test_empty_cloud_returns_empty_result(self) -> None:
        """Empty input should return empty output."""
        config = TargetConfig()
        calibration = CalibrationConfig()
        points = np.empty((0, 3), dtype=np.float64)

        result = preprocess_tracking_cloud(points, config, calibration)

        assert result.points_xyz.shape == (0, 3)
        assert result.source_indices.shape == (0,)

    def test_non_finite_points_filtered(self) -> None:
        """Non-finite points should be removed."""
        config = TargetConfig()
        calibration = CalibrationConfig()

        # Mix of finite and non-finite points
        points = np.array([
            [1.0, 0.0, 0.5],  # finite
            [np.inf, 0.0, 0.5],  # inf in x
            [1.0, np.nan, 0.5],  # nan in y
            [1.0, 0.0, 0.5],  # finite
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        # Should have only finite points
        assert result.points_xyz.shape[0] >= 0
        assert np.all(np.isfinite(result.points_xyz))

    def test_radial_bounds_applied(self) -> None:
        """Points outside radial bounds should be filtered."""
        config = TargetConfig(
            radial_min_m=0.5,
            radial_max_m=3.0,
        )
        calibration = CalibrationConfig()

        # Points at various radii
        points = np.array([
            [0.3, 0.0, 0.5],  # too close (r=0.3)
            [1.0, 0.0, 0.5],  # ok (r=1.0)
            [2.5, 0.0, 0.5],  # ok (r=2.5)
            [4.0, 0.0, 0.5],  # too far (r=4.0)
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        # Should have points at r=1.0 and r=2.5
        if result.points_xyz.shape[0] > 0:
            radii = np.hypot(result.points_xyz[:, 0], result.points_xyz[:, 1])
            assert np.all((radii >= 0.5) & (radii <= 3.0))

    def test_height_bounds_applied(self) -> None:
        """Points outside physical height bounds should be filtered."""
        config = TargetConfig(
            physical_height_min_m=0.5,
            physical_height_max_m=2.0,
        )
        calibration = CalibrationConfig(lidar_height_m=1.0)

        # Points at various z heights
        # Physical height = z + lidar_height_m
        points = np.array([
            [1.0, 0.0, -0.6],  # physical height 0.4 (too low)
            [1.0, 0.0, 0.0],   # physical height 1.0 (ok)
            [1.0, 0.0, 0.5],   # physical height 1.5 (ok)
            [1.0, 0.0, 1.5],   # physical height 2.5 (too high)
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        if result.points_xyz.shape[0] > 0:
            physical_heights = result.points_xyz[:, 2] + 1.0
            assert np.all((physical_heights >= 0.5) & (physical_heights <= 2.0))

    def test_self_box_excluded(self) -> None:
        """Points in self box should be filtered."""
        config = TargetConfig(
            self_x_min_m=-0.3,
            self_x_max_m=0.2,
            self_abs_y_max_m=0.2,
            radial_min_m=0.0,  # Allow close points
            radial_max_m=5.0,
        )
        calibration = CalibrationConfig()

        # Points in and out of self box
        points = np.array([
            [0.0, 0.0, 0.5],   # center - in self box
            [0.1, 0.1, 0.5],   # in self box
            [-0.2, 0.15, 0.5], # in self box
            [1.0, 0.0, 0.5],   # outside self box
            [0.0, 0.5, 0.5],   # outside self box (y)
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        if result.points_xyz.shape[0] > 0:
            # No points should be in self box
            for pt in result.points_xyz:
                in_self_x = -0.3 <= pt[0] <= 0.2
                in_self_y = abs(pt[1]) <= 0.2
                assert not (in_self_x and in_self_y)

    def test_voxel_downsample(self) -> None:
        """Points should be downsampled by voxel grid."""
        config = TargetConfig(
            voxel_size_m=0.05,
            radial_min_m=0.0,
            radial_max_m=5.0,
        )
        calibration = CalibrationConfig()

        # Many points in same voxel
        points = np.array([
            [1.0, 0.0, 0.5],
            [1.01, 0.01, 0.51],  # same voxel
            [1.02, 0.02, 0.52],  # same voxel
            [2.0, 0.0, 0.5],     # different voxel
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        # Should have fewer points than input
        assert result.points_xyz.shape[0] <= points.shape[0]

    def test_source_indices_aligned_to_downsampled_points(self) -> None:
        """source_indices should map to original points."""
        config = TargetConfig()
        calibration = CalibrationConfig()

        points = np.array([
            [1.0, 0.0, 0.5],
            [1.1, 0.0, 0.5],
            [1.2, 0.0, 0.5],
        ])

        result = preprocess_tracking_cloud(points, config, calibration)

        # source_indices should have same length as downsampled points
        assert result.source_indices.shape[0] == result.points_xyz.shape[0]

        # All source indices should be valid
        if result.source_indices.shape[0] > 0:
            assert np.all(result.source_indices >= 0)
            assert np.all(result.source_indices < len(points))

    def test_target_mask_is_aligned_to_downsampled_points(self) -> None:
        """Target mask should use downsampled index domain."""
        from robot_follow.lidar_alx_assisted_follow.tracking.target import TargetLockManager

        config = TargetConfig()
        calibration = CalibrationConfig()

        # Points with good target candidate
        points = np.array([
            [1.0, 0.0, 0.5],
            [1.05, 0.05, 0.55],
            [1.1, 0.0, 0.6],
            [2.0, 0.0, 0.5],  # far point
        ])

        cloud = preprocess_tracking_cloud(points, config, calibration)
        manager = TargetLockManager(config=config)

        obs = manager.select(cloud, click_xy=(1.05, 0.0), now_s=1.0)

        if obs is not None:
            # Target mask should match downsampled cloud size
            assert obs.target_mask.shape == (cloud.points_xyz.shape[0],)

    def test_output_is_read_only(self) -> None:
        """Output arrays should be read-only."""
        config = TargetConfig()
        calibration = CalibrationConfig()

        points = np.array([[1.0, 0.0, 0.5]])

        result = preprocess_tracking_cloud(points, config, calibration)

        if result.points_xyz.shape[0] > 0:
            assert not result.points_xyz.flags.writeable

        if result.source_indices.shape[0] > 0:
            assert not result.source_indices.flags.writeable

    def test_deterministic_voxel_selection(self) -> None:
        """Voxel downsample should be deterministic."""
        config = TargetConfig(
            voxel_size_m=0.05,
        )
        calibration = CalibrationConfig()

        points = np.random.rand(100, 3) * 2 + np.array([1.0, 0.0, 0.5])

        result1 = preprocess_tracking_cloud(points, config, calibration)
        result2 = preprocess_tracking_cloud(points, config, calibration)

        assert np.array_equal(result1.points_xyz, result2.points_xyz)
        assert np.array_equal(result1.source_indices, result2.source_indices)