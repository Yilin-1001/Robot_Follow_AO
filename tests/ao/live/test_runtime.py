from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import pytest

from robot_follow.ao.config import AOALXConfig, AOLatencyConfig, AOOutputConfig, AORadarConfig, AORadarServiceConfig
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.ao.live.fast_lio_source import PointCloudFrame
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core.pipeline import PipelineStepResult
from robot_follow.core.types import MotionCommand, TargetSample


@dataclass(frozen=True)
class FakeWristbandState:
    enabled: Optional[bool] = None
    updated_at: Optional[float] = None


@dataclass(frozen=True)
class FakeSnapshot:
    timestamp: float
    distance_m: float = 1.0
    azimuth_deg: float = 0.0
    error: Optional[str] = None

    def age_s(self, now=None) -> float:
        return (100.0 if now is None else now) - self.timestamp


class FakeALXSource:
    def __init__(self, snapshot, wristband_state: Optional[FakeWristbandState] = None):
        self._snapshot = snapshot
        self._wristband_state = wristband_state or FakeWristbandState(enabled=None, updated_at=None)

    def snapshot(self):
        return self._snapshot

    def wristband_state(self) -> FakeWristbandState:
        return self._wristband_state


class FakeRadarManager:
    def __init__(self):
        self.position_times = []
        self.ensure_count = 0
        self.stop_checks = []
        self.stop_calls = 0

    def on_position_frame(self, timestamp: float) -> None:
        self.position_times.append(timestamp)

    def ensure_running(self) -> None:
        self.ensure_count += 1

    def stop_if_idle(self, *, now: float) -> None:
        self.stop_checks.append(now)

    def stop(self) -> None:
        self.stop_calls += 1


class FakeCloudSource:
    def __init__(self, frame):
        self._frame = frame

    def latest(self):
        return self._frame


class FakePipeline:
    def __init__(self):
        self.calls = []

    def process(self, sample: TargetSample):
        self.calls.append(sample)
        command = MotionCommand(kind="move", vx=0.2, vy=0.0, omega=0.0, reason="follow")
        return PipelineStepResult(sample=sample, filtered=None, verdict=None, command=command)


class FakePlanner:
    def filter_command(self, envelope: CommandEnvelope, report, *, now: float) -> MotionCommand:
        return envelope.command


class FakeRadarPipeline:
    def process(self, frame):
        return object()


class FakeBridge:
    def __init__(self):
        self.commands = []
        self.ticks = []

    def dispatch(self, command: MotionCommand) -> None:
        self.commands.append(command)

    def tick(self, *, now: float) -> None:
        self.ticks.append(now)


def config() -> AORadarConfig:
    return AORadarConfig(
        alx=AOALXConfig(stale_timeout_s=0.5),
        radar=AORadarServiceConfig(idle_stop_s=5.0, livox_start_command="livox", fast_lio_start_command="fast"),
        latency=AOLatencyConfig(max_command_age_s=0.2, max_cloud_age_s=0.5, max_ao_process_ms=20.0),
        output=AOOutputConfig(mode="g1", require_real_g1_confirmation=True, confirm_real_g1=True),
    )


def test_fresh_position_starts_radar_and_dispatches_safe_command() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.position_times == [99.95]
    assert radar_manager.ensure_count == 1
    assert bridge.commands[-1].kind == "move"
    assert bridge.ticks == [100.0]
    assert runtime.last_frame is not None
    assert runtime.last_frame.raw_command is not None
    assert runtime.last_frame.to_log_row().safe_command_kind == "move"


def test_stale_position_stops_g1_and_does_not_start_radar() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.0),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.ensure_count == 0
    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "data_timeout"
    assert runtime.last_frame is not None
    assert runtime.last_frame.stop_reason == "data_timeout"


def test_missing_or_stale_cloud_stops_before_ao_planner() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(None),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "radar_not_ready"


class SlowClock:
    def __init__(self):
        self.values = iter([100.0, 100.0, 100.05, 100.05, 100.05])

    def __call__(self) -> float:
        return next(self.values)


def test_slow_ao_result_is_discarded() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=SlowClock(),
    )

    runtime.tick()

    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "ao_timeout"


def test_wristband_off_stops_radar_and_does_not_start() -> None:
    """手环OFF时，点云不应启动。即使位置数据有效，也不应该ensure_running。"""
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=False, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.ensure_count == 0, "手环OFF时不应启动点云"
    assert radar_manager.stop_calls == 1, "手环OFF时应立即调用stop()"
    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "wristband_off"


def test_wristband_on_with_valid_position_starts_radar() -> None:
    """手环ON + 位置有效时，点云应正常启动。"""
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.ensure_count == 1, "手环ON时应启动点云"
    assert bridge.commands[-1].kind == "move"


def test_wristband_unknown_does_not_start_radar() -> None:
    """手环信号从未到达时（enabled=None），点云应保持停止。"""
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=None, updated_at=None),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.ensure_count == 0, "手环状态未知时不应启动点云"
    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "wristband_unknown"


def test_wristband_on_to_off_stops_radar_during_runtime() -> None:
    """运行中手环从ON变OFF，点云应停止。"""
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()
    assert radar_manager.ensure_count == 1, "手环ON时应启动点云"

    # 模拟手环变为OFF
    runtime.alx_source = FakeALXSource(
        FakeSnapshot(timestamp=99.95),
        wristband_state=FakeWristbandState(enabled=False, updated_at=100.0),
    )
    runtime.tick()

    assert radar_manager.ensure_count == 1, "手环OFF后不应再调用ensure_running"
    assert radar_manager.stop_calls == 1, "手环OFF时应调用stop()"
    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "wristband_off"


def test_cloud_freshness_uses_received_at_when_available() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.95),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=1.0, points=[], received_at_s=99.9)),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert bridge.commands[-1].kind == "move"
    assert runtime.last_frame is not None
    assert runtime.last_frame.cloud_age_ms == pytest.approx(100.0)
