"""Regression tests for Web visualization while motion is stopped."""

from unittest.mock import Mock, patch

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import PrototypeConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    CloudSnapshot,
    PreprocessedCloud,
    StopKind,
)
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.controller import (
    LidarALXAssistedFollowRuntime,
)
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshotBuffer
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import TargetLockManager


def test_hard_stop_still_publishes_preprocessed_cloud_for_web():
    """A controller-disconnect stop must not blank the LiDAR monitor."""
    config = PrototypeConfig()
    lifecycle = WristbandLifecycleGuardian(config.sensors.ready_timeout_s)
    lifecycle._state = LifecycleState.SENSORS_READY
    latest_status = LatestStatus()
    stop_latch = StopLatch(control_thread_id=0)
    stop_latch.request("controller_disconnected", StopKind.HARD)
    runtime = LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=Mock(),
        alx_reader=Mock(),
        alx_buffer=ALXSnapshotBuffer(config.alx, config.calibration),
        lifecycle=lifecycle,
        radar_services=Mock(),
        target_manager=TargetLockManager(config.target),
        stop_latch=stop_latch,
        event_inbox=ControlEventInbox(),
        latest_status=latest_status,
        limiter=CommandSlewRateLimiter(config.output_motion),
        bridge=Mock(),
        csv_logger=Mock(),
    )
    points = np.array([[1.0, 0.0, 0.4], [1.1, 0.1, 0.5]], dtype=np.float64)
    preprocessed = PreprocessedCloud(
        points_xyz=points,
        source_indices=np.array([0, 1], dtype=np.int64),
    )
    cloud = CloudSnapshot(
        sequence=1,
        sensor_timestamp_s=10.0,
        received_at_s=10.0,
        frame_id="body",
        points_xyz=points,
    )

    with patch(
        "robot_follow.lidar_alx_assisted_follow.runtime.controller.preprocess_tracking_cloud",
        return_value=preprocessed,
    ) as preprocess:
        runtime._process_cloud(cloud, alx_snapshot=None, now=10.0)

    _, status = latest_status.snapshot()
    preprocess.assert_called_once()
    assert status["points"] == points.tolist()
    assert status["stop"]["reason"] == "controller_disconnected"


def test_no_target_still_publishes_preprocessed_cloud_for_web():
    """The monitor must remain useful before a target has been selected."""
    config = PrototypeConfig()
    lifecycle = WristbandLifecycleGuardian(config.sensors.ready_timeout_s)
    lifecycle._state = LifecycleState.SENSORS_READY
    latest_status = LatestStatus()
    points = np.array([[1.0, 0.0, 0.4], [1.1, 0.1, 0.5]], dtype=np.float64)
    preprocessed = PreprocessedCloud(
        points_xyz=points,
        source_indices=np.array([0, 1], dtype=np.int64),
    )
    runtime = LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=Mock(),
        alx_reader=Mock(),
        alx_buffer=ALXSnapshotBuffer(config.alx, config.calibration),
        lifecycle=lifecycle,
        radar_services=Mock(),
        target_manager=TargetLockManager(config.target),
        stop_latch=StopLatch(control_thread_id=0),
        event_inbox=ControlEventInbox(),
        latest_status=latest_status,
        limiter=CommandSlewRateLimiter(config.output_motion),
        bridge=Mock(),
        csv_logger=Mock(),
    )
    cloud = CloudSnapshot(
        sequence=1,
        sensor_timestamp_s=10.0,
        received_at_s=10.0,
        frame_id="body",
        points_xyz=points,
    )

    with patch(
        "robot_follow.lidar_alx_assisted_follow.runtime.controller.preprocess_tracking_cloud",
        return_value=preprocessed,
    ):
        runtime._process_cloud(cloud, alx_snapshot=None, now=10.0)

    _, status = latest_status.snapshot()
    assert status["points"] == points.tolist()
    assert status["stop"]["reason"] == "alx_stale"
