"""Avoidance planning and safety command limiting.

NOTE: the stateless ``plan_and_limit`` wrapper (formerly re-exported from
``decision``) is intentionally not imported here. ``decision.py`` depends on
the AO ``app.config`` module which is not part of the copied runtime yet, so
pulling it in at package import time would break importing this package. The
stateful planner in ``planner.py`` and the limiter in ``limiter.py`` remain
available and are the path used by the AO runtime adapter.
"""
from robot_follow.ao.avoidance.limiter import limit_decision
from robot_follow.ao.avoidance.planner import (
    AvoidanceState,
    StatefulAvoidancePlanner,
    plan_avoidance,
)

__all__ = [
    "AvoidanceState",
    "StatefulAvoidancePlanner",
    "limit_decision",
    "plan_avoidance",
]
