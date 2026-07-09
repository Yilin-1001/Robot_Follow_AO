from __future__ import annotations

import math

from robot_follow.ao.core.types import AvoidanceDecision, CommandKind, MotionCommand


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def limit_decision(
    decision: AvoidanceDecision,
    *,
    max_vx=0.20,
    max_vy=0.10,
    max_omega=0.50,
) -> AvoidanceDecision:
    """Turn a candidate avoidance decision into the *final* decision to emit.

    Enforces the §13 hard constraints on the candidate ``safe_command``:
    non-finite velocities are forced to stop, and move velocities are clamped to
    configured maxima. The returned :class:`AvoidanceDecision` is final — its
    ``safe_command`` is the value that should be sent to motion output.
    """
    cmd = decision.safe_command
    values = (cmd.vx, cmd.vy, cmd.omega)
    if not all(math.isfinite(v) for v in values):
        return AvoidanceDecision.stop(
            timestamp=decision.timestamp,
            source="limiter",
            reason="non_finite_velocity",
            rules=decision.triggered_safety_rules + ("non_finite_velocity",),
        )
    if cmd.kind is CommandKind.STOP:
        safe = MotionCommand.stop(cmd.timestamp, source=cmd.source, reason=cmd.reason)
        return AvoidanceDecision(
            timestamp=decision.timestamp,
            safe_command=safe,
            decision_kind=decision.decision_kind,
            recommended_direction=decision.recommended_direction,
            slowdown_ratio=decision.slowdown_ratio,
            triggered_safety_rules=decision.triggered_safety_rules,
            reason=decision.reason,
        )
    vx = _clamp(cmd.vx, max_vx)
    vy = _clamp(cmd.vy, max_vy)
    omega = _clamp(cmd.omega, max_omega)
    rules = decision.triggered_safety_rules
    if (vx, vy, omega) != (cmd.vx, cmd.vy, cmd.omega):
        rules = rules + ("velocity_limited",)
    safe = MotionCommand.move(
        cmd.timestamp,
        source=cmd.source,
        vx=vx,
        vy=vy,
        omega=omega,
        reason=cmd.reason,
    )
    return AvoidanceDecision(
        timestamp=decision.timestamp,
        safe_command=safe,
        decision_kind=decision.decision_kind,
        recommended_direction=decision.recommended_direction,
        slowdown_ratio=decision.slowdown_ratio,
        triggered_safety_rules=rules,
        reason=decision.reason,
    )
