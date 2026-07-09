from __future__ import annotations

import time
from typing import Callable, Protocol

from robot_follow.ao.config import AORadarConfig
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand, TargetSample


Clock = Callable[[], float]


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

    def tick(self) -> MotionCommand:
        now = self.clock()
        snapshot = self.alx_source.snapshot()
        if snapshot is None or getattr(snapshot, "error", None):
            return self._dispatch_stop("serial_error")
        if snapshot.age_s(now) > self.config.alx.stale_timeout_s:
            self.radar_manager.stop_if_idle(now=now)
            return self._dispatch_stop("data_timeout")

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
            return self._dispatch_stop("radar_not_ready")
        if cloud.is_stale(now=now, max_age_s=self.config.latency.max_cloud_age_s):
            return self._dispatch_stop("radar_timeout")
        if envelope.is_stale(now=now, max_age_s=self.config.latency.max_command_age_s):
            return self._dispatch_stop("command_timeout")

        ao_start = self.clock()
        report = self.radar_pipeline.process(cloud)
        safe = self.planner.filter_command(envelope, report, now=now)
        ao_ms = (self.clock() - ao_start) * 1000.0
        if ao_ms > self.config.latency.max_ao_process_ms:
            return self._dispatch_stop("ao_timeout")

        self.bridge.dispatch(safe)
        return safe

    def _dispatch_stop(self, reason: str) -> MotionCommand:
        command = MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)
        self.bridge.dispatch(command)
        return command