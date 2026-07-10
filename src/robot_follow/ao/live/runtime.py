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
    raw_azimuth_deg: float | None = None
    filtered_azimuth_deg: float | None = None
    g1_sent_omega: float | None = None
    distance_m: float | None = None
    tag_status: int | None = None
    robot_yaw_deg: float | None = None
    robot_yaw_rate_rps: float | None = None
    # 新增：完整延迟追踪时间戳
    raw_command_generated_at_ms: float | None = None  # raw命令生成时刻（相对于tick开始）
    safe_command_generated_at_ms: float | None = None  # safe命令生成时刻（相对于tick开始）
    g1_response_at_ms: float | None = None  # G1 SDK响应时刻（相对于tick开始）
    total_latency_ms: float | None = None  # 从raw到G1的总延迟

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
            raw_azimuth_deg=self.raw_azimuth_deg,
            filtered_azimuth_deg=self.filtered_azimuth_deg,
            g1_sent_omega=self.g1_sent_omega,
            distance_m=self.distance_m,
            tag_status=self.tag_status,
            robot_yaw_deg=self.robot_yaw_deg,
            robot_yaw_rate_rps=self.robot_yaw_rate_rps,
            raw_command_generated_at_ms=self.raw_command_generated_at_ms,
            safe_command_generated_at_ms=self.safe_command_generated_at_ms,
            g1_response_at_ms=self.g1_response_at_ms,
            total_latency_ms=self.total_latency_ms,
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
        tick_start = now  # 记录tick开始时刻作为基准
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
                tick_start=tick_start,
            )
        if wristband_state is None:
            # 手环状态未知 - 不启动点云，保持停止
            return self._dispatch_stop(
                "wristband_unknown",
                now=now,
                loop_dt_ms=loop_dt_ms,
                tick_start=tick_start,
            )

        # 手环ON - 继续正常的控制流程
        snapshot = self.alx_source.snapshot()
        if snapshot is None or getattr(snapshot, "error", None):
            return self._dispatch_stop(
                "serial_error",
                now=now,
                loop_dt_ms=loop_dt_ms,
                tick_start=tick_start,
            )
        alx_age_ms = snapshot.age_s(now) * 1000.0
        distance_m = float(snapshot.distance_m)
        raw_azimuth_deg = float(snapshot.azimuth_deg)
        tag_status_value = getattr(snapshot, "tag_status", None)
        tag_status = None if tag_status_value is None else int(tag_status_value)
        if snapshot.age_s(now) > self.config.alx.stale_timeout_s:
            self.radar_manager.stop_if_idle(now=now)
            return self._dispatch_stop(
                "data_timeout",
                now=now,
                loop_dt_ms=loop_dt_ms,
                alx_age_ms=alx_age_ms,
                distance_m=distance_m,
                tag_status=tag_status,
                raw_azimuth_deg=raw_azimuth_deg,
                tick_start=tick_start,
            )

        self.radar_manager.on_position_frame(snapshot.timestamp)
        self.radar_manager.ensure_running()

        sample = TargetSample(
            timestamp=snapshot.timestamp,
            distance_m=distance_m,
            azimuth_deg=float(snapshot.azimuth_deg),
            valid=True,
        )
        step = self.pipeline.process(sample)
        raw_elapsed_ms = (self.clock() - tick_start) * 1000.0  # raw命令生成耗时（从tick开始）
        filtered_azimuth_deg = (
            None if step.filtered is None else float(step.filtered.azimuth_deg)
        )
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
                distance_m=distance_m,
                tag_status=tag_status,
                raw_azimuth_deg=raw_azimuth_deg,
                filtered_azimuth_deg=filtered_azimuth_deg,
                tick_start=tick_start,
                raw_elapsed_ms=raw_elapsed_ms,
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
                distance_m=distance_m,
                tag_status=tag_status,
                raw_azimuth_deg=raw_azimuth_deg,
                filtered_azimuth_deg=filtered_azimuth_deg,
                tick_start=tick_start,
                raw_elapsed_ms=raw_elapsed_ms,
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
                distance_m=distance_m,
                tag_status=tag_status,
                raw_azimuth_deg=raw_azimuth_deg,
                filtered_azimuth_deg=filtered_azimuth_deg,
                tick_start=tick_start,
                raw_elapsed_ms=raw_elapsed_ms,
            )

        ao_start = self.clock()
        report = self.radar_pipeline.process(cloud)
        safe = self.planner.filter_command(envelope, report, now=now)
        ao_mark = self.clock()
        ao_ms = (ao_mark - ao_start) * 1000.0
        safe_elapsed_ms = (ao_mark - tick_start) * 1000.0  # safe命令生成耗时（从tick开始）
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
                distance_m=distance_m,
                tag_status=tag_status,
                raw_azimuth_deg=raw_azimuth_deg,
                filtered_azimuth_deg=filtered_azimuth_deg,
                tick_start=tick_start,
                raw_elapsed_ms=raw_elapsed_ms,
            )

        dispatch_start = self.clock()
        self.bridge.dispatch(safe)
        dispatch_mark = self.clock()
        dispatch_ms = (dispatch_mark - dispatch_start) * 1000.0
        g1_elapsed_ms = (dispatch_mark - tick_start) * 1000.0  # G1响应耗时（从tick开始）
        total_latency_ms = g1_elapsed_ms - raw_elapsed_ms if raw_elapsed_ms > 0 else None  # 从raw到G1的总延迟

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
            raw_azimuth_deg=raw_azimuth_deg,
            filtered_azimuth_deg=filtered_azimuth_deg,
            g1_sent_omega=self._g1_sent_omega(),
            distance_m=distance_m,
            tag_status=tag_status,
            robot_yaw_deg=self._robot_yaw_deg(),
            robot_yaw_rate_rps=self._robot_yaw_rate_rps(),
            # 新增延迟追踪字段
            raw_command_generated_at_ms=raw_elapsed_ms,
            safe_command_generated_at_ms=safe_elapsed_ms,
            g1_response_at_ms=g1_elapsed_ms,
            total_latency_ms=total_latency_ms,
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
        distance_m: float | None = None,
        tag_status: int | None = None,
        raw_azimuth_deg: float | None = None,
        filtered_azimuth_deg: float | None = None,
        tick_start: float | None = None,  # 新增参数
        raw_elapsed_ms: float | None = None,  # 新增参数
    ) -> MotionCommand:
        command = MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)
        dispatch_start = self.clock()
        self.bridge.dispatch(command)
        dispatch_ms = (self.clock() - dispatch_start) * 1000.0

        # 计算延迟时间戳
        g1_elapsed_ms = (self.clock() - tick_start) * 1000.0 if tick_start else None
        total_latency_ms = None
        if g1_elapsed_ms is not None and raw_elapsed_ms is not None:
            total_latency_ms = g1_elapsed_ms - raw_elapsed_ms

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
            raw_azimuth_deg=raw_azimuth_deg,
            filtered_azimuth_deg=filtered_azimuth_deg,
            g1_sent_omega=self._g1_sent_omega(),
            distance_m=distance_m,
            tag_status=tag_status,
            robot_yaw_deg=self._robot_yaw_deg(),
            robot_yaw_rate_rps=self._robot_yaw_rate_rps(),
            # 新增延迟追踪字段
            raw_command_generated_at_ms=raw_elapsed_ms,
            safe_command_generated_at_ms=None,  # stop命令无safe阶段
            g1_response_at_ms=g1_elapsed_ms,
            total_latency_ms=total_latency_ms,
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

    def _g1_sent_omega(self) -> float | None:
        """Return the omega most recently attempted at the output bridge.

        ``G1MotionBridge`` exposes this after applying yaw-sign and final
        limits. Other bridge implementations can omit it, in which case the
        log intentionally records an empty value rather than guessing.
        """
        value = getattr(self.bridge, "last_sent_omega", None)
        if value is None:
            return None
        return float(value)

    def _robot_yaw_deg(self) -> float | None:
        state = getattr(self.bridge, "attitude_state", None)
        if state is None:
            return None
        return float(state.yaw_deg)

    def _robot_yaw_rate_rps(self) -> float | None:
        state = getattr(self.bridge, "attitude_state", None)
        if state is None:
            return None
        return float(state.yaw_rate_rps)
