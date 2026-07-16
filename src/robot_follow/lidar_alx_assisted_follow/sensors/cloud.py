"""Sequenced cloud source that wraps FAST-LIO with sequence numbers."""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np

from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.lidar_alx_assisted_follow.domain import CloudSnapshot


class SequencedFastLIOPointCloudSource(FastLIOPointCloudSource):
    """FAST-LIO cloud source with per-frame sequence numbers and threading.Event signaling.

    Each call to ``update_for_test`` (or the internal ROS2 callback) produces
    one ``CloudSnapshot`` with an incrementing sequence number and sets a
    threading.Event so the control thread can wait for fresh data.
    """

    def __init__(
        self,
        topic: str = "/cloud_registered_body",
        expected_frame_id: str = "body",
        require_body_frame: bool = True,
        cloud_y_sign: int = 1,
    ) -> None:
        super().__init__(
            topic=topic,
            expected_frame_id=expected_frame_id,
            require_body_frame=require_body_frame,
        )
        self._sequence_lock = threading.Lock()
        self._next_sequence: int = 1
        self._latest_snapshot: Optional[CloudSnapshot] = None
        self._event = threading.Event()
        if cloud_y_sign not in (1, -1):
            raise ValueError("cloud_y_sign must be +1 or -1")
        self._cloud_y_sign = cloud_y_sign

    @property
    def event(self) -> threading.Event:
        """Event that is set when a new cloud snapshot is available."""
        return self._event

    def latest_snapshot(self) -> Optional[CloudSnapshot]:
        """Return the latest CloudSnapshot or None."""
        with self._sequence_lock:
            return self._latest_snapshot

    def wait_for_update(self, timeout_s: float) -> Optional[CloudSnapshot]:
        """Wait up to timeout_s for a new cloud snapshot.

        Parameters
        ----------
        timeout_s : float
            Maximum time to wait in seconds.

        Returns
        -------
        CloudSnapshot or None
            The latest snapshot if one arrived, or None on timeout.
        """
        if not self._event.wait(timeout=timeout_s):
            return None

        with self._sequence_lock:
            snapshot = self._latest_snapshot
            self._event.clear()
            return snapshot

    def update_for_test(self, frame) -> None:
        """Update with a new frame, wrapping it in a sequenced CloudSnapshot.

        Thread-safe. Overrides the base class method.
        """
        # Let the base class record the raw AO frame
        super().update_for_test(frame)

        # Build sequenced CloudSnapshot
        # FAST-LIO's incoming frame has been observed mirrored laterally on
        # this deployment.  Normalize it here, before tracking, APF and Web
        # serialization, while retaining +x=forward and the configured body
        # frame identity.  ALX uses its own already-correct calibration.
        points = np.array(frame.points_xyz, dtype=np.float64, copy=True)
        if points.ndim == 1 and points.size == 0:
            points = np.empty((0, 3), dtype=np.float64)
        elif points.ndim == 2 and points.shape[1] >= 2:
            points[:, 1] *= self._cloud_y_sign

        # Make read-only to guarantee immutability
        points.flags.writeable = False

        with self._sequence_lock:
            seq = self._next_sequence
            self._next_sequence += 1
            self._latest_snapshot = CloudSnapshot(
                sequence=seq,
                sensor_timestamp_s=frame.timestamp,
                received_at_s=frame.received_at_s
                if frame.received_at_s is not None
                else 0.0,
                frame_id=frame.frame_id,
                points_xyz=points,
            )
            self._event.set()
