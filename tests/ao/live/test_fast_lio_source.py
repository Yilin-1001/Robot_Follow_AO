"""Tests for FastLIOPointCloudSource and point cloud conversion."""

import numpy as np

from robot_follow.ao.live.fast_lio_source import (
    FastLIOPointCloudSource,
    PointCloudFrame,
    fast_lio_pointcloud2_rows_to_frame,
)


class FakeStamp:
    """Fake ROS2 timestamp."""
    def __init__(self, sec: int, nanosec: int):
        self.sec = sec
        self.nanosec = nanosec


class FakeHeader:
    """Fake ROS2 message header."""
    def __init__(self, frame_id: str, stamp: FakeStamp):
        self.frame_id = frame_id
        self.stamp = stamp


class FakePointCloud2:
    """Fake ROS2 PointCloud2 message for testing."""
    def __init__(self, frame_id: str, sec: int, nanosec: int):
        self.header = FakeHeader(frame_id, FakeStamp(sec, nanosec))


def test_fast_lio_pointcloud2_rows_to_frame_converts_correctly() -> None:
    """Test that ROS2 PointCloud2 rows are converted to AO PointCloudFrame."""
    msg = FakePointCloud2(frame_id="body", sec=10, nanosec=500_000_000)  # 10.5s
    rows = [(1.0, 0.0, 0.0), (2.0, 0.5, 0.0), (3.0, 0.0, 1.0)]

    frame = fast_lio_pointcloud2_rows_to_frame(
        msg,
        rows,
        expected_frame_id="body",
        require_body_frame=True,
        received_at_s=100.0,
    )

    assert frame.timestamp == 10.5
    assert frame.frame_id == "body"
    assert isinstance(frame.points_xyz, np.ndarray)
    assert frame.points_xyz.shape == (3, 3)
    assert frame.valid is True
    assert frame.empty_is_clear is False
    assert frame.received_at_s == 100.0


def test_fast_lio_pointcloud2_rows_to_frame_empty_cloud() -> None:
    """Test that empty point cloud is handled correctly."""
    msg = FakePointCloud2(frame_id="body", sec=5, nanosec=0)
    rows = []

    frame = fast_lio_pointcloud2_rows_to_frame(
        msg,
        rows,
        expected_frame_id="body",
        require_body_frame=True,
        received_at_s=50.0,
    )

    assert frame.timestamp == 5.0
    assert frame.points_xyz.shape == (0, 3)
    assert frame.empty_is_clear is True


def test_fast_lio_pointcloud2_rows_to_frame_rejects_wrong_frame_id() -> None:
    """Test that wrong frame_id raises ValueError when require_body_frame=True."""
    msg = FakePointCloud2(frame_id="livox_frame", sec=10, nanosec=0)
    rows = [(1.0, 0.0, 0.0)]

    try:
        fast_lio_pointcloud2_rows_to_frame(
            msg,
            rows,
            expected_frame_id="body",
            require_body_frame=True,
            received_at_s=100.0,
        )
        raise AssertionError("expected ValueError")
    except ValueError as e:
        assert "expected FAST-LIO body frame" in str(e)


def test_fast_lio_pointcloud2_rows_to_frame_allows_wrong_frame_when_disabled() -> None:
    """Test that wrong frame_id is allowed when require_body_frame=False."""
    msg = FakePointCloud2(frame_id="livox_frame", sec=10, nanosec=0)
    rows = [(1.0, 0.0, 0.0)]

    frame = fast_lio_pointcloud2_rows_to_frame(
        msg,
        rows,
        expected_frame_id="body",
        require_body_frame=False,
        received_at_s=100.0,
    )

    assert frame.frame_id == "livox_frame"
    assert frame.valid is True


def test_latest_frame_replaces_previous_frame() -> None:
    """Test that latest() returns the most recent frame."""
    from robot_follow.ao.core.types import PointCloudFrame as AOPointCloudFrame

    source = FastLIOPointCloudSource(topic="/cloud_registered_body")

    # Create AO-style frames for testing
    frame1 = AOPointCloudFrame(
        timestamp=10.0,
        frame_id="body",
        points_xyz=np.array([[1.0, 0.0, 0.0]]),
        valid=True,
        empty_is_clear=False,
        reason="test",
    )
    frame2 = AOPointCloudFrame(
        timestamp=10.1,
        frame_id="body",
        points_xyz=np.array([[2.0, 0.0, 0.0]]),
        valid=True,
        empty_is_clear=False,
        reason="test",
    )

    source.update_for_test(frame1)
    source.update_for_test(frame2)

    latest = source.latest()

    assert latest is not None
    assert latest.timestamp == 10.1
    assert latest.points_xyz.shape == (1, 3)


def test_frame_age_and_staleness() -> None:
    """Test PointCloudFrame age calculation and staleness detection."""
    frame = PointCloudFrame(timestamp=10.0, points=[])

    assert frame.age_s(now=10.25) == 0.25
    assert frame.is_stale(now=10.6, max_age_s=0.5) is True
    assert frame.is_stale(now=10.4, max_age_s=0.5) is False