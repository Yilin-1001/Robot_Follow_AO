"""Lifecycle tests for explicit ALX wristband ROS control status."""

from __future__ import annotations

import threading
import time
from unittest.mock import Mock

from robot_follow.lidar_alx_assisted_follow.config.schema import PrototypeConfig
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.runtime.controller import (
    LidarALXAssistedFollowRuntime,
)
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEvent,
    ControlEventInbox,
    ControlEventKind,
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
from robot_follow.lidar_alx_assisted_follow.domain.models import StartState
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


def test_missing_initial_heartbeat_waits_without_latching_serial_error() -> None:
    """ROS subscriber startup must not poison a healthy ON transition."""
    reader = FakeALXReader(WristbandTopicState.UNKNOWN, fresh=False)
    reader.status = WristbandStatus(WristbandTopicState.UNKNOWN, received_at=None)
    runtime = make_runtime(reader)

    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    assert not runtime.stop_latch.snapshot().requested
    assert runtime.lifecycle.state == LifecycleState.WRISTBAND_OFF

    reader.status = WristbandStatus(WristbandTopicState.ON, received_at=10.05)
    reader.fresh = True
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.05)

    assert runtime.lifecycle.state == LifecycleState.STARTING_SENSORS
    assert not runtime.stop_latch.snapshot().requested


def test_missing_wristband_heartbeat_fails_closed() -> None:
    reader = FakeALXReader(WristbandTopicState.ON)
    runtime = make_runtime(reader)
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.0)

    reader.fresh = False
    runtime._update_lifecycle(alx_snapshot=None, cloud_received=False, now=10.6)

    assert runtime.lifecycle.state == LifecycleState.WRISTBAND_OFF
    runtime.radar_services.stop.assert_called_once_with()
    assert runtime.stop_latch.snapshot().reason == "serial_error"


def test_web_start_requires_live_physical_inputs_then_enables_follow() -> None:
    reader = FakeALXReader(WristbandTopicState.ON)
    runtime = make_runtime(reader)
    runtime.stop_latch.bind_control_thread(threading.get_ident())

    runtime._process_control_event(ControlEvent(ControlEventKind.START_FOLLOW, {}))
    assert runtime._state.follow_enabled is False
    assert runtime._state.last_start_result.state == StartState.REJECTED
    assert runtime._state.last_start_result.reason == "lidar_not_ready"

    runtime.lifecycle._state = LifecycleState.SENSORS_READY
    runtime._state.last_cloud_received_at_s = time.monotonic()
    runtime._state.last_alx_synchronized = True
    runtime._process_control_event(ControlEvent(ControlEventKind.START_FOLLOW, {}))

    assert runtime._state.follow_enabled is True
    assert runtime._state.last_start_result.state == StartState.ACCEPTED


def test_controller_disconnect_event_cannot_revoke_web_follow_enable() -> None:
    runtime = make_runtime(FakeALXReader(WristbandTopicState.ON))
    runtime._state.follow_enabled = True
    runtime._process_control_event(
        ControlEvent(ControlEventKind.CONTROLLER_DISCONNECT, {})
    )
    assert runtime._state.follow_enabled is True
