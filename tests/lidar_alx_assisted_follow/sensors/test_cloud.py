"""Tests for cloud sequencing."""

from __future__ import annotations

import time

import numpy as np
import pytest

from robot_follow.ao.core.types import PointCloudFrame
from robot_follow.lidar_alx_assisted_follow.sensors.cloud import (
    SequencedFastLIOPointCloudSource,
)


def body_frame(
    points: list = None,
    received_at_s: float = None,
    timestamp: float = None,
) -> PointCloudFrame:
    """Create a test body-frame PointCloudFrame."""
    if points is None:
        points = []
    if timestamp is None:
        timestamp = time.monotonic()
    if received_at_s is None:
        received_at_s = timestamp

    points_xyz = np.array(points, dtype=np.float64) if points else np.empty((0, 3), dtype=np.float64)

    return PointCloudFrame(
        timestamp=timestamp,
        frame_id="body",
        points_xyz=points_xyz,
        valid=True,
        empty_is_clear=len(points) == 0,
        reason="test",
        received_at_s=received_at_s,
    )


class TestSequencedFastLIOPointCloudSource:
    """Test sequenced cloud source."""

    def test_each_source_update_gets_one_sequence_and_sets_event(self) -> None:
        """Each update should increment sequence and set event."""
        source = SequencedFastLIOPointCloudSource()

        frame = body_frame(received_at_s=10.0, timestamp=9.9)
        source.update_for_test(frame)

        # Event should be set
        assert source.event.is_set()

        # Check snapshot
        snapshot = source.latest_snapshot()
        assert snapshot is not None
        assert snapshot.sequence == 1
        assert snapshot.received_at_s == 10.0
        assert snapshot.sensor_timestamp_s == 9.9
        assert snapshot.frame_id == "body"

        # Second update
        frame2 = body_frame(received_at_s=11.0, timestamp=10.9)
        source.update_for_test(frame2)

        snapshot2 = source.latest_snapshot()
        assert snapshot2 is not None
        assert snapshot2.sequence == 2

    def test_wait_for_update_returns_latest_on_event(self) -> None:
        """wait_for_update should return latest snapshot when event is set."""
        source = SequencedFastLIOPointCloudSource()

        frame = body_frame(received_at_s=10.0)
        source.update_for_test(frame)

        # Event already set, should return immediately
        snapshot = source.wait_for_update(0.0)
        assert snapshot is not None
        assert snapshot.sequence == 1

        # Event should be cleared after wait
        assert not source.event.is_set()

    def test_wait_for_update_times_out(self) -> None:
        """wait_for_update should return None on timeout."""
        source = SequencedFastLIOPointCloudSource()

        # No updates, should timeout
        snapshot = source.wait_for_update(0.01)
        assert snapshot is None

    def test_wait_for_update_does_not_replay_consumed_snapshot_on_timeout(self) -> None:
        """A watchdog timeout must not pretend that the last cloud is new."""
        source = SequencedFastLIOPointCloudSource()
        source.update_for_test(body_frame(received_at_s=10.0))

        consumed = source.wait_for_update(0.0)
        assert consumed is not None

        assert source.wait_for_update(0.0) is None

    def test_points_are_read_only(self) -> None:
        """Points array should be read-only."""
        source = SequencedFastLIOPointCloudSource()

        points = [[1.0, 0.0, 0.5], [1.1, 0.1, 0.6]]
        frame = body_frame(points=points, received_at_s=10.0)
        source.update_for_test(frame)

        snapshot = source.latest_snapshot()
        assert snapshot is not None

        # Should be read-only
        assert not snapshot.points_xyz.flags.writeable

        # Attempt to modify should fail
        with pytest.raises(ValueError, match="read-only"):
            snapshot.points_xyz[0, 0] = 999.0

    def test_negative_cloud_y_sign_corrects_lateral_mirroring(self) -> None:
        source = SequencedFastLIOPointCloudSource(cloud_y_sign=-1)
        source.update_for_test(
            body_frame(points=[[1.2, 0.8, 0.3], [0.7, -0.4, 0.1]])
        )

        snapshot = source.latest_snapshot()

        assert snapshot is not None
        assert np.array_equal(
            snapshot.points_xyz,
            np.array([[1.2, -0.8, 0.3], [0.7, 0.4, 0.1]]),
        )
