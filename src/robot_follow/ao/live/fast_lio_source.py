from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True)
class PointCloudFrame:
    timestamp: float
    points: list[tuple[float, float, float]]

    def age_s(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.timestamp

    def is_stale(self, *, now: float, max_age_s: float) -> bool:
        return self.age_s(now) > max_age_s


class FastLIOPointCloudSource:
    def __init__(self, topic: str = "/cloud_registered_body") -> None:
        self.topic = topic
        self._lock = threading.Lock()
        self._latest: Optional[PointCloudFrame] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_ros2, name="fast-lio-source", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def latest(self) -> Optional[PointCloudFrame]:
        with self._lock:
            return self._latest

    def update_for_test(self, frame: PointCloudFrame) -> None:
        with self._lock:
            self._latest = frame

    def _run_ros2(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        rclpy.init(args=None)
        node = Node("robot_follow_ao_fast_lio_source")

        def on_cloud(msg: PointCloud2) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0:
                stamp = time.monotonic()
            points = [
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ]
            self.update_for_test(PointCloudFrame(timestamp=stamp, points=points))

        node.create_subscription(PointCloud2, self.topic, on_cloud, 1)
        try:
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()