"""Runtime integration tests for cloud-driven filtered ALX topic matching."""

from __future__ import annotations

from unittest.mock import Mock

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    ALXConfig,
    PrototypeConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import CloudSnapshot
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.controller import (
    LidarALXAssistedFollowRuntime,
)
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import (
    ALXPositionBuffer,
    ALXPositionSample,
)
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import TargetLockManager


def make_runtime(buffer: ALXPositionBuffer) -> LidarALXAssistedFollowRuntime:
    config = PrototypeConfig(alx=ALXConfig(cloud_max_skew_s=0.08))
    lifecycle = WristbandLifecycleGuardian(config.sensors.ready_timeout_s)
    lifecycle._state = LifecycleState.SENSORS_READY
    lifecycle._session_tag_id = 0
    return LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=Mock(),
        alx_reader=Mock(),
        alx_buffer=buffer,
        lifecycle=lifecycle,
        radar_services=Mock(),
        target_manager=TargetLockManager(config.target),
        stop_latch=StopLatch(control_thread_id=0),
        event_inbox=ControlEventInbox(),
        latest_status=LatestStatus(),
        limiter=CommandSlewRateLimiter(config.output_motion),
        bridge=Mock(),
        csv_logger=Mock(),
    )


def cloud(stamp_s: float) -> CloudSnapshot:
    return CloudSnapshot(
        sequence=7,
        sensor_timestamp_s=stamp_s,
        received_at_s=50.0,
        frame_id="body",
        points_xyz=np.empty((0, 3), dtype=np.float64),
    )


def test_runtime_matches_by_sensor_stamp_and_records_offset() -> None:
    buffer = ALXPositionBuffer(ALXConfig(cloud_max_skew_s=0.08))
    buffer.append(ALXPositionSample(100.00, 49.98, 1.0, 0.0))
    buffer.append(ALXPositionSample(100.04, 49.99, 2.0, -0.5))
    buffer.append(ALXPositionSample(100.08, 50.00, 3.0, 0.0))
    runtime = make_runtime(buffer)

    body_xy, fresh = runtime._match_alx_for_cloud(cloud(100.07), now=50.01)

    assert fresh
    assert body_xy is not None
    assert np.array_equal(body_xy, np.array([2.0, -0.5]))
    assert runtime._state.last_alx_stamp_s == 100.04
    assert runtime._state.last_alx_offset_ms == pytest.approx(30.0)
    assert runtime._state.last_alx_synchronized


def test_runtime_rejects_a_future_only_alx_sample() -> None:
    buffer = ALXPositionBuffer(ALXConfig(cloud_max_skew_s=0.08))
    buffer.append(ALXPositionSample(100.08, 50.00, 3.0, 0.0))
    runtime = make_runtime(buffer)

    body_xy, fresh = runtime._match_alx_for_cloud(cloud(100.07), now=50.01)

    assert body_xy is None
    assert not fresh
    assert not runtime._state.last_alx_synchronized


def test_web_status_contains_the_same_alx_match_metadata() -> None:
    buffer = ALXPositionBuffer(ALXConfig(cloud_max_skew_s=0.08))
    buffer.append(ALXPositionSample(100.04, 49.99, 2.0, -0.5))
    runtime = make_runtime(buffer)
    matched_cloud = cloud(100.07)
    runtime._match_alx_for_cloud(matched_cloud, now=50.01)

    runtime._publish_status(matched_cloud, preprocessed=None)
    _, status = runtime.latest_status.snapshot()

    assert status["cloud_sequence"] == 7
    assert status["alx"]["body_xy"] == [2.0, -0.5]
    assert status["alx"]["stamp_s"] == 100.04
    assert status["alx"]["offset_ms"] == pytest.approx(30.0)
    assert status["alx"]["synchronized"] is True
