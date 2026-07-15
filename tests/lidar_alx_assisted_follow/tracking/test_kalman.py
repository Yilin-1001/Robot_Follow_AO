"""Tests for the APF-only two-dimensional Kalman target smoother."""

from __future__ import annotations

import pytest

from robot_follow.lidar_alx_assisted_follow.tracking.kalman import KalmanFilter2D


def test_first_measurement_initializes_to_raw_position() -> None:
    """The first confirmed target point is not displaced."""
    kalman = KalmanFilter2D(
        process_noise=0.10,
        measurement_noise=0.05,
        max_interval_s=0.20,
    )

    assert kalman.update(1.0, -0.2, timestamp_s=10.0) == pytest.approx((1.0, -0.2))
    assert kalman.is_initialized


def test_second_measurement_is_smoothed() -> None:
    """A later measurement produces a corrected point between estimate and raw input."""
    kalman = KalmanFilter2D(0.10, 0.05, max_interval_s=0.20)
    kalman.update(1.0, 0.0, timestamp_s=10.0)

    filtered_x, filtered_y = kalman.update(1.2, 0.0, timestamp_s=10.1)

    assert 1.0 < filtered_x < 1.2
    assert filtered_y == pytest.approx(0.0)


def test_non_positive_dt_preserves_previous_filtered_position() -> None:
    """Out-of-order samples do not change the APF target."""
    kalman = KalmanFilter2D(0.10, 0.05, max_interval_s=0.20)
    kalman.update(1.0, 0.0, timestamp_s=10.0)
    previous = kalman.update(1.2, 0.0, timestamp_s=10.1)

    assert kalman.update(2.0, 3.0, timestamp_s=10.1) == pytest.approx(previous)


def test_excessive_interval_resets_to_current_measurement() -> None:
    """A gap beyond the configured interval cannot reuse velocity."""
    kalman = KalmanFilter2D(0.10, 0.05, max_interval_s=0.20)
    kalman.update(1.0, 0.0, timestamp_s=10.0)
    kalman.update(1.2, 0.0, timestamp_s=10.1)

    assert kalman.update(2.0, 0.5, timestamp_s=10.4) == pytest.approx((2.0, 0.5))
