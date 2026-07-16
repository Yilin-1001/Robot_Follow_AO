"""Tests for the filtered ALX ROS2 topic input path."""

from __future__ import annotations

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import ALXConfig
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import (
    ALXPositionBuffer,
    ALXPositionSample,
    ALXPositionTopicSource,
)
from robot_follow.lidar_alx_assisted_follow.sensors.wristband import (
    WristbandTopicState,
)


def sample(
    stamp_s: float,
    *,
    received_at: float | None = None,
    x_m: float = 1.0,
    y_m: float = 2.0,
) -> ALXPositionSample:
    return ALXPositionSample(
        timestamp_s=stamp_s,
        received_at=stamp_s if received_at is None else received_at,
        x_m=x_m,
        y_m=y_m,
    )


def test_buffer_selects_newest_causal_position() -> None:
    buffer = ALXPositionBuffer(ALXConfig(cloud_max_skew_s=0.08))
    buffer.append(sample(10.00, x_m=1.0))
    buffer.append(sample(10.04, x_m=2.0))
    buffer.append(sample(10.09, x_m=3.0))

    selected = buffer.for_cloud(10.07, session_tag_id=0)

    assert selected is not None
    assert selected.timestamp_s == 10.04
    assert selected.x_m == 2.0


def test_buffer_rejects_future_and_excessively_old_positions() -> None:
    buffer = ALXPositionBuffer(ALXConfig(cloud_max_skew_s=0.08))
    buffer.append(sample(10.10))
    assert buffer.for_cloud(10.00, session_tag_id=0) is None

    buffer.append(sample(10.20))
    assert buffer.for_cloud(10.19, session_tag_id=0) is None


def test_buffer_returns_already_filtered_body_coordinates() -> None:
    buffer = ALXPositionBuffer(ALXConfig())
    position = sample(10.0, x_m=3.25, y_m=-1.5)

    body_xy = buffer.transform_to_body(position)

    assert np.array_equal(body_xy, np.array([3.25, -1.5]))
    assert body_xy.flags.writeable is False


def test_topic_source_freshness_uses_local_receive_time() -> None:
    buffer = ALXPositionBuffer(ALXConfig())
    source = ALXPositionTopicSource(
        topic="/alx/position",
        expected_frame_id="body",
        stale_timeout_s=0.50,
        buffer=buffer,
    )
    source.update_for_test(sample(1_700_000_000.0, received_at=20.0))

    assert source.is_ok(now=20.49)
    assert not source.is_ok(now=20.51)


def test_topic_source_rejects_wrong_frame_and_non_finite_coordinates() -> None:
    buffer = ALXPositionBuffer(ALXConfig())
    source = ALXPositionTopicSource(
        topic="/alx/position",
        expected_frame_id="body",
        stale_timeout_s=0.50,
        buffer=buffer,
    )

    assert not source.accept_position(
        timestamp_s=10.0,
        received_at=20.0,
        frame_id="map",
        x_m=1.0,
        y_m=2.0,
    )
    assert not source.accept_position(
        timestamp_s=10.0,
        received_at=20.0,
        frame_id="body",
        x_m=float("nan"),
        y_m=2.0,
    )
    assert source.latest_snapshot() is None


def test_topic_source_tracks_explicit_wristband_heartbeat_separately() -> None:
    buffer = ALXPositionBuffer(ALXConfig())
    source = ALXPositionTopicSource(
        topic="/alx/position",
        wristband_topic="/alx/wristband_state",
        expected_frame_id="body",
        stale_timeout_s=0.50,
        wristband_timeout_s=0.25,
        buffer=buffer,
    )

    source.update_wristband_for_test(WristbandTopicState.ON, received_at=20.0)

    assert source.wristband_status().enabled is True
    assert source.is_wristband_status_fresh(now=20.24)
    assert not source.is_wristband_status_fresh(now=20.26)
