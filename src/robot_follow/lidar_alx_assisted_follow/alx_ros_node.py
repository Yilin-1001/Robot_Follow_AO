"""Publish Kalman-smoothed ALX positions as one ROS2 topic.

ROS imports are deliberately lazy so pure-Python tests and non-ROS modes do
not require a Jetson ROS2 installation.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Optional, Sequence

from robot_follow.lidar_alx_assisted_follow.config.loader import load_prototype_config
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
from robot_follow.lidar_alx_assisted_follow.sensors.alx_ros_position import (
    ALXPositionPublisherCore,
)
from robot_follow.real.alx_source import ALXLiveSource


class MonotonicRosStampBridge:
    """Map a serial receive monotonic time onto the node's ROS time basis."""

    def __init__(self, monotonic_origin_s: float, ros_origin_ns: int) -> None:
        self._monotonic_origin_s = monotonic_origin_s
        self._ros_origin_ns = ros_origin_ns

    def to_nanoseconds(self, received_at_s: float) -> int:
        """Return the ROS timestamp corresponding to a serial receive time."""
        elapsed_ns = int(round((received_at_s - self._monotonic_origin_s) * 1e9))
        return self._ros_origin_ns + elapsed_ns


def _snapshot_from_live(live_snapshot) -> ALXSnapshot:
    """Convert the reusable live-source snapshot to the filtered ROS input."""
    return ALXSnapshot(
        received_at=live_snapshot.received_at,
        distance_m=live_snapshot.distance_m,
        azimuth_deg=live_snapshot.azimuth_deg,
        elevation_deg=live_snapshot.elevation_deg,
        tag_id=live_snapshot.tag_id,
        error=live_snapshot.error is not None,
    )


def _run(config) -> int:
    """Run the ROS2 publisher until interrupted."""
    try:
        import rclpy
        from geometry_msgs.msg import PointStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Foxy with rclpy and geometry_msgs is required for "
            "robot-follow-alx-ros"
        ) from exc

    class ALXPositionNode(Node):
        """Own the ALX reader and publish only its newest filtered position."""

        def __init__(self) -> None:
            super().__init__("robot_follow_alx_position")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = self.create_publisher(
                PointStamped,
                config.alx_ros.topic,
                qos,
            )
            self._source = ALXLiveSource(
                port=config.alx.port,
                baudrate=config.alx.baudrate,
            )
            self._core = ALXPositionPublisherCore(
                alx_config=config.alx,
                calibration_config=config.calibration,
                ros_config=config.alx_ros,
            )
            self._stamp_bridge = MonotonicRosStampBridge(
                monotonic_origin_s=time.monotonic(),
                ros_origin_ns=self.get_clock().now().nanoseconds,
            )
            self._last_wait_log_at = 0.0
            self._last_seen_received_at = None
            self._published_once = False
            self._source.start()
            self._timer = self.create_timer(
                1.0 / config.alx_ros.poll_hz,
                self._publish_latest,
            )
            self.get_logger().info(
                "ALX position publisher started: port=%s topic=%s"
                % (config.alx.port, config.alx_ros.topic),
            )

        def _log_waiting(self, reason: str) -> None:
            """Emit a bounded diagnostic without creating another topic."""
            now = time.monotonic()
            if now - self._last_wait_log_at >= 1.0:
                self.get_logger().warning("ALX position not published: %s" % reason)
                self._last_wait_log_at = now

        def _publish_latest(self) -> None:
            # A position is useful only while the wristband protocol has
            # positively identified an enabled state.  We intentionally do
            # not republish a previous value when the serial stream is stale.
            live_snapshot = self._source.snapshot()
            if live_snapshot is not None and live_snapshot.error is not None:
                self._log_waiting("serial error: %s" % live_snapshot.error)
                return

            wristband_state = self._source.wristband_state()
            if wristband_state.enabled is not True:
                self._log_waiting(
                    "waiting for wristband ON marker "
                    "(state=%s)" % wristband_state.enabled
                )
                return

            if live_snapshot is None:
                self._log_waiting("waiting for first ALX Position frame")
                return

            # Polling intentionally runs faster than the 40 Hz device.  Seeing
            # the same immutable snapshot on several timer callbacks is normal
            # and must not be reported as an error.
            if live_snapshot.received_at == self._last_seen_received_at:
                return
            self._last_seen_received_at = live_snapshot.received_at

            position = self._core.update(_snapshot_from_live(live_snapshot))
            if position is None:
                self._log_waiting(
                    "new ALX frame is invalid: distance=%.3f azimuth=%.1f elevation=%.1f"
                    % (
                        live_snapshot.distance_m,
                        live_snapshot.azimuth_deg,
                        live_snapshot.elevation_deg,
                    )
                )
                return

            stamp_ns = self._stamp_bridge.to_nanoseconds(position.received_at)
            message = PointStamped()
            message.header.frame_id = config.alx_ros.frame_id
            message.header.stamp.sec = stamp_ns // 1_000_000_000
            message.header.stamp.nanosec = stamp_ns % 1_000_000_000
            message.point.x = position.x_m
            message.point.y = position.y_m
            message.point.z = 0.0
            self._publisher.publish(message)
            if not self._published_once:
                self.get_logger().info(
                    "First filtered ALX position published: x=%.3f y=%.3f"
                    % (position.x_m, position.y_m)
                )
                self._published_once = True

        def close(self) -> None:
            """Stop serial IO before releasing ROS resources."""
            self._source.stop()
            self.destroy_node()

    rclpy.init(args=None)
    node = ALXPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse the minimal ALX ROS2 publisher command line."""
    parser = argparse.ArgumentParser(
        description="Publish filtered ALX body-frame position to ROS2",
    )
    parser.add_argument("config", type=Path, help="LiDAR + ALX YAML configuration")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point for the single-topic ALX ROS2 publisher."""
    args = parse_args(argv)
    config = load_prototype_config(args.config)
    return _run(config)


if __name__ == "__main__":
    raise SystemExit(main())
