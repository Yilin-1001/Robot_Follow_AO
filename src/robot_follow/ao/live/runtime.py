from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

from robot_follow.ao.config import AORadarConfig
from robot_follow.ao.logging.csv_logger import AORadarLogRow
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand, TargetSample


Clock = Callable[[], float]


@dataclass(frozen=True)
class AORadarFrameResult:
    timestamp: float
    command: MotionCommand
    raw_command: MotionCommand | None
    alx_age_ms: float
    command_age_ms: float
    cloud_age_ms: float
    loop_dt_ms: float
    ao_process_ms: float
    g1_dispatch_ms: float
    radar_state: str
    wristband_gate_state: str
    stop_reason: str

    def to_log_row(self) -> AORadarLogRow:
        raw = self.raw_command
        return AORadarLogRow(
            timestamp=self.timestamp,
            alx_age_ms=self.alx_age_ms,
            command_age_ms=self.command_age_ms,
            cloud_age_ms=self.cloud_age_ms,
            loop_dt_ms=self.loop_dt_ms,
            ao_process_ms=self.ao_process_ms,
            g1_dispatch_ms=self.g1_dispatch_ms,
            raw_command_kind="" if raw is None else raw.kind,
            raw_command_reason="" if raw is None else raw.reason,
            safe_command_kind=self.command.kind,
            safe_command_reason=self.command.reason,
            radar_state=self.radar_state,
            wristband_gate_state=self.wristband_gate_state,
        )


class AORadarRuntime:
    def __init__(
        self,
        *,
        config: AORadarConfig,
        alx_source,
        pipeline,
        radar_manager,
        cloud_source,
        radar_pipeline,
        planner,
        bridge,
        clock: Clock = time.monotonic,
    ) -> None:
        self.config = config
        self.alx_source = alx_source
        self.pipeline = pipeline
        self.radar_manager = radar_manager
        self.cloud_source = cloud_source
        self.radar_pipeline = radar_pipeline
        self.planner = planner
        self.bridge = bridge
        self.clock = clock
        self.last_frame: AORadarFrameResult | None = None
        self._previous_tick_at: float | None = None

    def tick(self) -> MotionCommand:
        now = self.clock()
        loop_dt_ms = 0.0 if self._previous_tick_at is None else (now - self._previous_tick_at) * 1000.0
        self._previous_tick_at = now

        # 手环门控检查 - 必须最先执行
        wristband_state = self._get_wristband_state()
        if wristband_state is False:
            # 手环OFF - 立即停止点云
            self.radar_manager.stop()
            return self._dispatch_stop(
                "wristband_off",
                now=now,
                loop_dt_ms=loop_dt_ms,
            )
        if wristband_state is None:
            # 手环状态未知 - 不启动点云，保持停止
            return self._dispatch_stop(
                "wristband_unknown",
                now=now,
                loop_dt_ms=loop_dt_ms,
            )

        # 手环ON - 继续正常的控制流程
        snapshot = self.alx_source.snapshot()
        if snapshot is None or getattr(snapshot, "error", None):
            return self._dispatch_stop(
                "serial_error",
                now=now,
                loop_dt_ms=loop_dt_ms,
            )
        alx_age_ms = snapshot.age_s(now) * 1000.0
        if snapshot.age_s(now) > self.config.alx.stale_timeout_s:
            self.radar_manager.stop_if_idle(now=now)
            return self._dispatch_stop(
                "data_timeout",
                now=now,
                loop_dt_ms=loop_dt_ms,
                alx_age_ms=alx_age_ms,
            )

        self.radar_manager.on_position_frame(snapshot.timestamp)
        self.radar_manager.ensure_running()

        sample = TargetSample(
            timestamp=snapshot.timestamp,
            distance_m=snapshot.distance_m,
            azimuth_deg=float(snapshot.azimuth_deg),
            valid=True,
        )
        step = self.pipeline.process(sample)
        envelope = CommandEnvelope(
            command=step.command,
            generated_at=now,
            source="robot_follow",
        )

        cloud = self.cloud_source.latest()
        if cloud is None:
            return self._dispatch_stop(
                "radar_not_ready",
                now=now,
                loop_dt_ms=loop_dt_ms,
                raw_command=step.command,
                alx_age_ms=alx_age_ms,
                command_age_s=envelope.age_s(now),
            )
        cloud_age_ms = self._cloud_age_ms(cloud, now=now)
        if cloud_age_ms / 1000.0 > self.config.latency.max_cloud_age_s:
            return self._dispatch_stop(
                "radar_timeout",
                now=now,
                loop_dt_ms=loop_dt_ms,
                raw_command=step.command,
                alx_age_ms=alx_age_ms,
                command_age_s=envelope.age_s(now),
                cloud_age_ms=cloud_age_ms,
            )
        if envelope.is_stale(now=now, max_age_s=self.config.latency.max_command_age_s):
            return self._dispatch_stop(
                "command_timeout",
                now=now,
                loop_dt_ms=loop_dt_ms,
                raw_command=step.command,
                alx_age_ms=alx_age_ms,
                command_age_s=envelope.age_s(now),
                cloud_age_ms=cloud_age_ms,
            )

        ao_start = self.clock()
        report = self.radar_pipeline.process(cloud)
        safe = self.planner.filter_command(envelope, report, now=now)
        ao_ms = (self.clock() - ao_start) * 1000.0
        if ao_ms > self.config.latency.max_ao_process_ms:
            return self._dispatch_stop(
                "ao_timeout",
                now=now,
                loop_dt_ms=loop_dt_ms,
                raw_command=step.command,
                alx_age_ms=alx_age_ms,
                command_age_s=envelope.age_s(now),
                cloud_age_ms=cloud_age_ms,
                ao_process_ms=ao_ms,
            )

        dispatch_start = self.clock()
        self.bridge.dispatch(safe)
        dispatch_ms = (self.clock() - dispatch_start) * 1000.0
        self.last_frame = AORadarFrameResult(
            timestamp=now,
            command=safe,
            raw_command=step.command,
            alx_age_ms=alx_age_ms,
            command_age_ms=envelope.age_s(now) * 1000.0,
            cloud_age_ms=cloud_age_ms,
            loop_dt_ms=loop_dt_ms,
            ao_process_ms=ao_ms,
            g1_dispatch_ms=dispatch_ms,
            radar_state=self._radar_state(),
            wristband_gate_state=self._wristband_gate_state(),
            stop_reason="" if safe.kind != "stop" else safe.reason,
        )
        self._tick_bridge(now)
        return safe

    def _dispatch_stop(
        self,
        reason: str,
        *,
        now: float,
        loop_dt_ms: float,
        raw_command: MotionCommand | None = None,
        alx_age_ms: float = 0.0,
        command_age_s: float = 0.0,
        cloud_age_ms: float = 0.0,
        ao_process_ms: float = 0.0,
    ) -> MotionCommand:
        command = MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)
        dispatch_start = self.clock()
        self.bridge.dispatch(command)
        dispatch_ms = (self.clock() - dispatch_start) * 1000.0
        self.last_frame = AORadarFrameResult(
            timestamp=now,
            command=command,
            raw_command=raw_command,
            alx_age_ms=alx_age_ms,
            command_age_ms=command_age_s * 1000.0,
            cloud_age_ms=cloud_age_ms,
            loop_dt_ms=loop_dt_ms,
            ao_process_ms=ao_process_ms,
            g1_dispatch_ms=dispatch_ms,
            radar_state=self._radar_state(),
            wristband_gate_state=self._wristband_gate_state(),
            stop_reason=reason,
        )
        self._tick_bridge(now)
        return command

    def _cloud_age_ms(self, cloud, *, now: float) -> float:
        received_at = getattr(cloud, "received_at_s", None)
        if received_at is not None:
            return max(0.0, (now - float(received_at)) * 1000.0)
        age_s = getattr(cloud, "age_s", None)
        if callable(age_s):
            return max(0.0, float(age_s(now)) * 1000.0)
        return max(0.0, (now - float(getattr(cloud, "timestamp", now))) * 1000.0)

    def _radar_state(self) -> str:
        state = getattr(self.radar_manager, "state", "")
        return getattr(state, "value", str(state))

    def _wristband_gate_state(self) -> str:
        enabled = self._get_wristband_state()
        if enabled is True:
            return "on"
        if enabled is False:
            return "off"
        return "unknown"

    def _get_wristband_state(self) -> bool | None:
        """Return the wristband enabled state: True=ON, False=OFF, None=unknown."""
        wristband_state = getattr(self.alx_source, "wristband_state", None)
        if not callable(wristband_state):
            return None
        state = wristband_state()
        return getattr(state, "enabled", None)

    def _tick_bridge(self, now: float) -> None:
        tick = getattr(self.bridge, "tick", None)
        if callable(tick):
            tick(now=now)
