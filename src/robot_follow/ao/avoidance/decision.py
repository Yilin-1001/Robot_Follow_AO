from __future__ import annotations

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.avoidance.limiter import limit_decision
from robot_follow.ao.avoidance.planner import plan_avoidance
from robot_follow.ao.core.types import AvoidanceDecision, MotionCommand, ObstacleReport


def plan_and_limit(
    desired: MotionCommand,
    report: ObstacleReport,
    config: AppConfig,
) -> AvoidanceDecision:
    candidate = plan_avoidance(
        desired,
        report,
        lateral_vy_mps=config.avoidance.lateral_vy_mps,
        forward_vx_when_avoiding_mps=config.avoidance.forward_vx_when_avoiding_mps,
        slowdown_distance_m=config.zones.slowdown_distance_m,
        min_slowdown_ratio=config.avoidance.min_slowdown_ratio,
        allow_left_avoidance=config.avoidance.allow_left_avoidance,
        allow_right_avoidance=config.avoidance.allow_right_avoidance,
    )
    return limit_decision(
        candidate,
        max_vx=config.avoidance.max_vx,
        max_vy=config.avoidance.max_vy,
        max_omega=config.avoidance.max_omega,
    )
