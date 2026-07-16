"""Lifecycle tests for explicit ALX wristband ROS control status."""

from __future__ import annotations

from unittest.mock import Mock

from robot_follow.lidar_alx_assisted_follow.config.schema import PrototypeConfig
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.controller import (
    LidarALXAssistedFollowRuntime,
)
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import ALXPositionBuffer
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import ALXPositionSample
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)
from robot_follow.lidar_alx_assisted_follow.sensors.wristband import (
    WristbandStatus,
    WristbandTopicState,
)
from robot_follow.lidar_alx_assisted_follow.tracking.target import TargetLockManager


class FakeALXReader:
    def __init__(self, state: WristbandTopicState, fresh: bool = True) -> None:
        self.status = WristbandStatus(state=state, received_at=10.0)
        self.fresh = fresh

    def latest_snapshot(self):
        return None

    def is_ok(self) -> bool:
        return True

    def wristband_status(self) -> WristbandStatus:
        return self.status

    def is_wristband_status_fresh(self, now=None) -> bool:
        del now
        return self.fresh


def make_runtime(reader: FakeALXReader):
    config = PrototypeConfig()
    lifecycle = WristbandLifecycleGuardian(config.sensors.ready_timeout_s)
    radar_services = Mock()
    return LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=Mock(),
        alx_reader=reader,
        alx_buffer=ALXPositionBuffer(config.alx),
        lifecycle=lifecycle,
        radar_services=radar_services,
        target_manager=TargetLockManager(config.target),
        stop_latch=StopLatch(control_thread_id=0),
        event_inbox=ControlEventInbox(),
        latest_status=LatestStatus(),
        limiter=CommandSlewRateLimiter(config.output_motion),
        bridge=Mock(),
        csv_logger=Mock(),
    )


def test_explicit_on_starts_radar_without_waiting_for_position() -> None:
    reader = FakeALXReader(WristbandTopicState.ON)
    runtime = make_runtime(reader)

    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    assert runtime.lifecycle.state == LifecycleState.STARTING_SENSORS
    runtime.radar_services.ensure_running.assert_called_once_with()


def test_explicit_off_stops_radar_immediately_even_without_position_timeout() -> None:
    reader = FakeALXReader(WristbandTopicState.ON)
    runtime = make_runtime(reader)
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    reader.status = WristbandStatus(WristbandTopicState.OFF, received_at=10.01)
    # A cached position must not override an explicit OFF control message.
    runtime._update_lifecycle(
        alx_snapshot=ALXPositionSample(10.01, 10.01, 1.0, 0.0),
        cloud_received=False,
        now=10.01,
    )

    assert runtime.lifecycle.state == LifecycleState.WRISTBAND_OFF
    runtime.radar_services.stop.assert_called_once_with()
    assert runtime.stop_latch.snapshot().reason == "wristband_off"


def test_first_explicit_off_stops_managed_services_even_if_lifecycle_is_already_off() -> None:
    reader = FakeALXReader(WristbandTopicState.OFF)
    runtime = make_runtime(reader)

    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    runtime.radar_services.stop.assert_called_once_with()
    assert runtime.stop_latch.snapshot().reason == "wristband_off"


def test_missing_wristband_heartbeat_fails_closed() -> None:
    reader = FakeALXReader(WristbandTopicState.ON)
    runtime = make_runtime(reader)
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    reader.fresh = False
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.6)

    assert runtime.lifecycle.state == LifecycleState.WRISTBAND_OFF
    runtime.radar_services.stop.assert_called_once_with()
    assert runtime.stop_latch.snapshot().reason == "serial_error"
