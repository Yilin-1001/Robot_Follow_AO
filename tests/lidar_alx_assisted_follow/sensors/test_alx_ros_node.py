"""Unit tests for ROS timestamp conversion without requiring ROS2."""

from __future__ import annotations

from robot_follow.lidar_alx_assisted_follow.alx_ros_node import (
    MonotonicRosStampBridge,
)


def test_monotonic_receive_time_maps_to_ros_time_without_polling_delay() -> None:
    bridge = MonotonicRosStampBridge(
        monotonic_origin_s=100.0,
        ros_origin_ns=1_700_000_000_000_000_000,
    )

    assert bridge.to_nanoseconds(100.025) == 1_700_000_000_025_000_000
