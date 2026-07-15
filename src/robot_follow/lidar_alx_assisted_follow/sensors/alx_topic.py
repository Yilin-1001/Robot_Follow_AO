"""Filtered ALX ROS2 topic input with bounded causal matching."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import ALXConfig


@dataclass(frozen=True)
class ALXPositionSample:
    """One Kalman-filtered ALX body-frame position from ROS2."""

    timestamp_s: float
    received_at: float
    x_m: float
    y_m: float
    # PointStamped cannot carry the wristband tag.  A stable synthetic tag
    # preserves the existing lifecycle state machine until explicit wristband
    # lifecycle ownership is moved into the publisher node.
    tag_id: int = 0
    error: bool = False


class ALXPositionBuffer:
    """Thread-safe bounded history for causal point-cloud matching."""

    def __init__(
        self,
        alx_config: ALXConfig,
        buffer_duration_s: float = 0.5,
        max_samples: int = 16,
    ) -> None:
        self._alx_config = alx_config
        self._buffer_duration_s = buffer_duration_s
        self._max_samples = max_samples
        self._buffer: Deque[ALXPositionSample] = deque()
        self._lock = threading.Lock()

    def append(
        self,
        sample: ALXPositionSample,
        now: Optional[float] = None,
    ) -> None:
        """Append one position, deduplicating by its ROS timestamp."""
        del now  # Trimming uses the sample clock, not the monotonic clock.
        with self._lock:
            if self._buffer and self._buffer[-1].timestamp_s == sample.timestamp_s:
                return

            self._buffer.append(sample)
            cutoff = sample.timestamp_s - self._buffer_duration_s
            while self._buffer and self._buffer[0].timestamp_s < cutoff:
                self._buffer.popleft()
            while len(self._buffer) > self._max_samples:
                self._buffer.popleft()

    def for_cloud(
        self,
        cloud_timestamp_s: float,
        session_tag_id: Optional[int],
    ) -> Optional[ALXPositionSample]:
        """Return the newest non-future sample within the configured skew."""
        del session_tag_id  # PointStamped carries no hardware tag identifier.
        with self._lock:
            for sample in reversed(self._buffer):
                if sample.timestamp_s > cloud_timestamp_s:
                    continue
                if cloud_timestamp_s - sample.timestamp_s > self._alx_config.cloud_max_skew_s:
                    continue
                return sample
        return None

    def transform_to_body(self, sample: ALXPositionSample) -> Optional[np.ndarray]:
        """Return the already transformed and filtered body-frame XY."""
        if sample.error or not math.isfinite(sample.x_m) or not math.isfinite(sample.y_m):
            return None
        body_xy = np.array([sample.x_m, sample.y_m], dtype=np.float64)
        body_xy.flags.writeable = False
        return body_xy


class ALXPositionTopicSource:
    """Subscribe to ``geometry_msgs/PointStamped`` without touching serial IO."""

    def __init__(
        self,
        *,
        topic: str,
        expected_frame_id: str,
        stale_timeout_s: float,
        buffer: ALXPositionBuffer,
    ) -> None:
        self.topic = topic
        self._expected_frame_id = expected_frame_id
        self._stale_timeout_s = stale_timeout_s
        self._buffer = buffer
        self._lock = threading.Lock()
        self._latest: Optional[ALXPositionSample] = None
        self._last_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        """Start the ROS2 subscription thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_ros2,
            name="alx-position-topic-source",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the ROS2 subscription thread."""
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._thread = None

    def latest_snapshot(self) -> Optional[ALXPositionSample]:
        """Return the most recently received filtered position."""
        with self._lock:
            return self._latest

    def is_ok(self, now: Optional[float] = None) -> bool:
        """Return whether the topic has no error and its latest frame is fresh."""
        if now is None:
            now = time.monotonic()
        with self._lock:
            latest = self._latest
            error = self._last_error
        if error is not None:
            return False
        # Before the first frame, remain in WRISTBAND_OFF rather than declaring
        # a fault.  Once publishing has begun, silence is the temporary OFF or
        # transport-failure signal used by the existing lifecycle guardian.
        if latest is None:
            return True
        return now - latest.received_at <= self._stale_timeout_s

    def update_for_test(self, sample: ALXPositionSample) -> None:
        """Inject a validated position without requiring ROS2."""
        self._accept_sample(sample)

    def accept_position(
        self,
        *,
        timestamp_s: float,
        received_at: float,
        frame_id: str,
        x_m: float,
        y_m: float,
    ) -> bool:
        """Validate and store fields extracted from a PointStamped message."""
        if frame_id != self._expected_frame_id:
            return False
        if not all(math.isfinite(value) for value in (timestamp_s, x_m, y_m)):
            return False
        self._accept_sample(
            ALXPositionSample(
                timestamp_s=timestamp_s,
                received_at=received_at,
                x_m=x_m,
                y_m=y_m,
            )
        )
        return True

    def _accept_sample(self, sample: ALXPositionSample) -> None:
        self._buffer.append(sample)
        with self._lock:
            self._latest = sample
            self._last_error = None

    def _run_ros2(self) -> None:
        """Own a separate ROS2 context so FAST-LIO can keep its existing one."""
        try:
            import rclpy
            from geometry_msgs.msg import PointStamped
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            context = Context()
            rclpy.init(args=None, context=context)
            node = Node("robot_follow_alx_position_source", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            def on_position(message: PointStamped) -> None:
                timestamp_s = (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) * 1e-9
                )
                self.accept_position(
                    timestamp_s=timestamp_s,
                    received_at=time.monotonic(),
                    frame_id=message.header.frame_id,
                    x_m=float(message.point.x),
                    y_m=float(message.point.y),
                )

            node.create_subscription(PointStamped, self.topic, on_position, qos)
            try:
                while not self._stop.is_set():
                    executor.spin_once(timeout_sec=0.05)
            finally:
                executor.remove_node(node)
                node.destroy_node()
                executor.shutdown()
                rclpy.shutdown(context=context)
        except Exception as exc:
            with self._lock:
                self._last_error = "%s: %s" % (type(exc).__name__, exc)
