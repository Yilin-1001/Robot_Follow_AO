"""Tests for ALX buffering and transform (Task 2 Step 3)."""

from __future__ import annotations

import time

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import ALXConfig, CalibrationConfig
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot, ALXSnapshotBuffer


@pytest.fixture
def alx_config() -> ALXConfig:
    """Default ALX config for tests."""
    return ALXConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        stale_timeout_s=0.50,
        cloud_max_skew_s=0.15,
        min_distance_m=0.10,
        max_distance_m=10.0,
    )


@pytest.fixture
def calibration_config() -> CalibrationConfig:
    """Default calibration config for tests."""
    return CalibrationConfig(
        x_m=0.0,
        y_m=0.0,
        yaw_deg=0.0,
        azimuth_sign=1,
        azimuth_zero_offset_deg=0.0,
        lidar_height_m=1.25,
        alx_height_m=1.10,
        allow_provisional_calibration_for_prototype=True,
    )


class TestALXSnapshotBufferDeduplication:
    """Test that snapshots are deduplicated by received_at."""

    def test_append_deduplicates_by_exact_received_at(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Snapshots with same received_at are deduplicated."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config, buffer_duration_s=1.0)

        snapshot1 = ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=45.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )
        snapshot2 = ALXSnapshot(
            received_at=1.0,  # Same timestamp
            distance_m=3.0,  # Different distance
            azimuth_deg=60.0,
            elevation_deg=10.0,
            tag_id=200,
            error=False,
        )

        buffer.append(snapshot1, now=2.0)
        buffer.append(snapshot2, now=2.0)

        # Should only keep one
        assert buffer.for_cloud(1.05, 100) is not None
        assert buffer.for_cloud(1.05, 100).distance_m == 2.0  # First one kept


class TestALXSnapshotBufferCausality:
    """Test causal ALX selection (no future samples)."""

    def test_for_cloud_rejects_future_snapshots(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """ALX snapshot received after cloud is not selected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config, buffer_duration_s=1.0)

        # ALX received at t=1.2 (in the future relative to cloud at t=1.0)
        buffer.append(ALXSnapshot(
            received_at=1.2,
            distance_m=3.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        ), now=2.0)

        # Query for cloud at t=1.0
        result = buffer.for_cloud(1.0, 100)
        assert result is None  # No ALX at or before t=1.0 (1.2 is in the future)

    def test_for_cloud_rejects_excess_skew(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """ALX snapshot with skew > cloud_max_skew_s is rejected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config, buffer_duration_s=1.0)

        # ALX at t=0.5, cloud at t=1.0, skew=0.5s > 0.15s
        buffer.append(ALXSnapshot(
            received_at=0.5,
            distance_m=2.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        ), now=1.5)

        result = buffer.for_cloud(1.0, 100)
        assert result is None  # Skew too large

    def test_for_cloud_rejects_tag_mismatch(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """ALX snapshot with different tag_id is rejected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config, buffer_duration_s=1.0)

        buffer.append(ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        ), now=2.0)

        # Query with different tag
        result = buffer.for_cloud(1.05, 999)
        assert result is None

    def test_for_cloud_selects_newest_valid(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """When multiple valid ALX samples exist, select the newest."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config, buffer_duration_s=1.0)

        buffer.append(ALXSnapshot(
            received_at=0.95,
            distance_m=2.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        ), now=2.0)
        buffer.append(ALXSnapshot(
            received_at=1.0,
            distance_m=3.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        ), now=2.0)

        # Cloud at t=1.05
        result = buffer.for_cloud(1.05, 100)
        assert result is not None
        assert result.distance_m == 3.0  # Newest valid


class TestALXTransformToBody:
    """Test ALX polar-to-body coordinate transform."""

    def test_transform_valid_snapshot(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Transform valid ALX snapshot to body XY."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config)

        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=0.0,  # Forward
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )

        body_xy = buffer.transform_to_body(snapshot)
        assert body_xy is not None
        # distance=2.0, azimuth=0 -> x=2.0, y=0.0
        assert np.allclose(body_xy, [2.0, 0.0])

    def test_transform_rejects_error_snapshot(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Snapshots with error=True are rejected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config)

        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=True,
        )

        body_xy = buffer.transform_to_body(snapshot)
        assert body_xy is None

    def test_transform_rejects_out_of_range_distance(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Snapshots with distance outside [min, max] are rejected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config)

        # Too close
        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=0.05,  # < min_distance_m
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )
        assert buffer.transform_to_body(snapshot) is None

        # Too far
        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=15.0,  # > max_distance_m
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )
        assert buffer.transform_to_body(snapshot) is None

    def test_transform_rejects_invalid_azimuth(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Snapshots with azimuth outside [-180, 180] are rejected."""
        buffer = ALXSnapshotBuffer(alx_config, calibration_config)

        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=200.0,  # Out of range
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )

        assert buffer.transform_to_body(snapshot) is None

    def test_transform_with_calibration_offset(
        self, alx_config: ALXConfig, calibration_config: CalibrationConfig
    ) -> None:
        """Calibration translation and rotation are applied correctly."""
        # Calibration: x=0.1, y=0.2, yaw=90deg
        calib = CalibrationConfig(
            x_m=0.1,
            y_m=0.2,
            yaw_deg=90.0,
            azimuth_sign=1,
            azimuth_zero_offset_deg=0.0,
            lidar_height_m=1.25,
            alx_height_m=1.10,
            allow_provisional_calibration_for_prototype=True,
        )
        buffer = ALXSnapshotBuffer(alx_config, calib)

        # ALX at distance=2.0, azimuth=0 (forward in ALX frame)
        # After yaw=90deg rotation: [0, 2]
        # After translation [0.1, 0.2]: [0.1, 2.2]
        snapshot = ALXSnapshot(
            received_at=1.0,
            distance_m=2.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            tag_id=100,
            error=False,
        )

        body_xy = buffer.transform_to_body(snapshot)
        assert body_xy is not None
        assert np.allclose(body_xy, [0.1, 2.2], atol=1e-6)