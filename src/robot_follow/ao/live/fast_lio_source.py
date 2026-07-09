from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class PointCloudFrame:
    """Latest-only point cloud frame from FAST-LIO."""

    timestamp: float
    points: list[tuple[float, float, float]]

    def age_s(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.timestamp

    def is_stale(self, *, now: float, max_age_s: float) -> bool:
        return self.age_s(now) > max_age_s


def _stamp_to_seconds(stamp) -> float:
    """Convert ROS2 timestamp to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fast_lio_pointcloud2_rows_to_frame(
    msg,
    rows,
    *,
    expected_frame_id: str = "body",
    require_body_frame: bool = True,
    received_at_s=None,
):
    """Convert ROS2 PointCloud2 message rows to PointCloudFrame.

    This function is copied from Robot_FollowAO_Radar project where it was
    verified to work correctly with FAST-LIO point clouds.

    Parameters
    ----------
    msg
        ROS2 PointCloud2 message.
    rows
        Iterator of (x, y, z) rows from point_cloud2.read_points().
    expected_frame_id
        Expected frame ID for body frame (default: "body").
    require_body_frame
        If True, raise ValueError if frame_id does not match.
    received_at_s
        Monotonic timestamp when the message was received.

    Returns
    -------
    robot_follow.ao.core.types.PointCloudFrame
        Converted point cloud frame.
    """
    # Import here to avoid circular dependency and match AO types
    from robot_follow.ao.core.types import PointCloudFrame as AOPointCloudFrame

    if require_body_frame and msg.header.frame_id != expected_frame_id:
        raise ValueError(
            f"expected FAST-LIO body frame {expected_frame_id!r}, got {msg.header.frame_id!r}"
        )

    points = np.asarray([(row[0], row[1], row[2]) for row in rows], dtype=np.float64)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float64)

    return AOPointCloudFrame(
        timestamp=_stamp_to_seconds(msg.header.stamp),
        frame_id=msg.header.frame_id,
        points_xyz=points,
        valid=True,
        empty_is_clear=points.shape[0] == 0,
        reason="fast_lio_body",
        received_at_s=received_at_s,
    )


class FastLIOPointCloudSource:
    """ROS2 FAST-LIO point cloud source with latest-only semantics.

    Subscribes to /cloud_registered_body topic and provides the most recent
    frame via latest(). Uses Ros2FastLioPointCloudSource-style conversion
    from the Robot_FollowAO_Radar project.
    """

    def __init__(
        self,
        topic: str = "/cloud_registered_body",
        expected_frame_id: str = "body",
        require_body_frame: bool = True,
    ) -> None:
        self.topic = topic
        self._expected_frame_id = expected_frame_id
        self._require_body_frame = require_body_frame
        self._lock = threading.Lock()
        self._latest = None  # Type: Optional[robot_follow.ao.core.types.PointCloudFrame]
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        """Start the ROS2 subscription thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_ros2, name="fast-lio-source", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the ROS2 subscription thread."""
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def latest(self):
        """Return the latest PointCloudFrame (AO type) or None."""
        with self._lock:
            return self._latest

    def update_for_test(self, frame) -> None:
        """Update latest frame for testing (thread-safe)."""
        with self._lock:
            self._latest = frame

    def _run_ros2(self) -> None:
        """ROS2 thread: subscribe to PointCloud2 and convert to AO frame."""
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        rclpy.init(args=None)
        node = Node("robot_follow_ao_fast_lio_source")

        def on_cloud(msg: PointCloud2) -> None:
            received_at_s = time.monotonic()
            rows = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
            frame = fast_lio_pointcloud2_rows_to_frame(
                msg,
                rows,
                expected_frame_id=self._expected_frame_id,
                require_body_frame=self._require_body_frame,
                received_at_s=received_at_s,
            )
            self.update_for_test(frame)

        node.create_subscription(PointCloud2, self.topic, on_cloud, 1)
        try:
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()