"""Tests for the pure ALX-to-ROS position publishing core."""

from __future__ import annotations

import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    ALXConfig,
    ALXRosConfig,
    CalibrationConfig,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
from robot_follow.lidar_alx_assisted_follow.sensors.alx_ros_position import (
    ALXPositionPublisherCore,
)


def _snapshot(
    *,
    received_at: float,
    distance_m: float = 2.0,
    azimuth_deg: float = 0.0,
    error: bool = False,
) -> ALXSnapshot:
    return ALXSnapshot(
        received_at=received_at,
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        elevation_deg=0.0,
        tag_id=7,
        error=error,
    )


@pytest.fixture
def publisher() -> ALXPositionPublisherCore:
    return ALXPositionPublisherCore(
        alx_config=ALXConfig(),
        calibration_config=CalibrationConfig(),
        ros_config=ALXRosConfig(
            kalman_process_noise=0.05,
            kalman_measurement_noise=0.25,
            kalman_max_interval_s=0.20,
        ),
    )


def test_first_valid_sample_is_published_in_body_frame(
    publisher: ALXPositionPublisherCore,
) -> None:
    position = publisher.update(_snapshot(received_at=10.0, distance_m=2.0))

    assert position is not None
    assert position.received_at == pytest.approx(10.0)
    assert position.x_m == pytest.approx(2.0)
    assert position.y_m == pytest.approx(0.0)


def test_same_serial_sample_is_not_republished(
    publisher: ALXPositionPublisherCore,
) -> None:
    sample = _snapshot(received_at=10.0)

    assert publisher.update(sample) is not None
    assert publisher.update(sample) is None


def test_kalman_smooths_a_large_lateral_measurement_jump(
    publisher: ALXPositionPublisherCore,
) -> None:
    assert publisher.update(_snapshot(received_at=10.0, distance_m=2.0)) is not None

    position = publisher.update(
        _snapshot(received_at=10.025, distance_m=2.0, azimuth_deg=30.0)
    )

    assert position is not None
    # The raw y coordinate is 1.0 m.  A filtered first correction must move
    # toward it without passing it, preventing the raw jump from reaching ROS.
    assert 0.0 < position.y_m < 1.0


def test_invalid_sample_is_not_published(
    publisher: ALXPositionPublisherCore,
) -> None:
    assert publisher.update(_snapshot(received_at=10.0, error=True)) is None
    assert publisher.update(_snapshot(received_at=10.025, distance_m=99.0)) is None


def test_long_gap_resets_kalman_before_publishing_new_measurement(
    publisher: ALXPositionPublisherCore,
) -> None:
    assert publisher.update(_snapshot(received_at=10.0, distance_m=2.0)) is not None

    position = publisher.update(_snapshot(received_at=10.25, distance_m=3.0))

    assert position is not None
    assert position.x_m == pytest.approx(3.0)
