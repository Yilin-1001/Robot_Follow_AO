from __future__ import annotations

import time
from dataclasses import dataclass

from robot_follow.ao.avoidance.decision import plan_and_limit
from robot_follow.ao.core.types import MotionCommand
from robot_follow.ao.radar.pipeline import process_radar_frame
from robot_follow.ao.web.serialization import visualization_payload


@dataclass(frozen=True)
class ConstantCommandSource:
    source: str = "scripted"
    vx: float = 0.18
    vy: float = 0.0
    omega: float = 0.0
    reason: str = "web_visualizer"

    def command_for(self, *, timestamp: float) -> MotionCommand:
        return MotionCommand.move(
            timestamp,
            source=self.source,
            vx=self.vx,
            vy=self.vy,
            omega=self.omega,
            reason=self.reason,
        )


def build_visualization_payload(
    frame,
    *,
    command_source,
    config,
    planner=None,
    now_s=None,
    min_points=None,
    max_points: int = 2000,
) -> dict:
    current_time = time.monotonic() if now_s is None else float(now_s)
    pipeline = process_radar_frame(
        frame,
        config=config,
        now_s=current_time,
        min_points=min_points,
    )
    desired = command_source.command_for(timestamp=frame.timestamp)
    if planner is None:
        decision = plan_and_limit(desired, pipeline.report, config)
    else:
        decision = planner.plan_and_limit(desired, pipeline.report, now_s=current_time)
    return visualization_payload(
        raw_frame=pipeline.raw_frame,
        filtered_frame=pipeline.filtered_frame,
        report=pipeline.report,
        desired_command=desired,
        decision=decision,
        raw_count=pipeline.raw_count,
        filtered_count=pipeline.filtered_count,
        now_s=current_time,
        max_points=max_points,
        zone_bounds=pipeline.zone_bounds,
    )
