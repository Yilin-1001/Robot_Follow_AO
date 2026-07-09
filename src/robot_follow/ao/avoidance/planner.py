from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from robot_follow.ao.avoidance.limiter import limit_decision
from robot_follow.ao.core.types import (
    AvoidanceDecision,
    DecisionKind,
    Direction,
    MotionCommand,
    ObstacleReport,
)


class AvoidanceState(str, Enum):
    NORMAL = "normal"
    AVOID_LEFT = "avoid_left"
    AVOID_RIGHT = "avoid_right"
    CLEARING = "clearing"
    RETURN_FORWARD = "return_forward"
    SAFE_STOP = "safe_stop"


def _zone_distance_for_choice(zone):
    if zone.min_distance_m is None:
        return math.inf
    return float(zone.min_distance_m)


def _side_zone(report: ObstacleReport, side: str):
    if side == "left":
        return report.left
    if side == "right":
        return report.right
    raise ValueError("side must be 'left' or 'right'")


def _valid_tie_break(config) -> str:
    side = getattr(config, "tie_break_side", "left")
    if side not in {"left", "right"}:
        return "left"
    return side


def side_is_dangerous(zone, config) -> bool:
    if zone.blocked:
        return True
    if zone.min_distance_m is None:
        return False
    return float(zone.min_distance_m) <= float(config.side_danger_distance_m)


def _rear_is_clear(report: ObstacleReport) -> bool:
    return not report.rear.blocked and not report.rear.occupied


def choose_avoidance_side(report: ObstacleReport, *, current_side=None, config=None):
    if config is None:
        raise ValueError("config is required")
    candidates = []
    for side in ("left", "right"):
        zone = _side_zone(report, side)
        if not zone.blocked:
            candidates.append(side)
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    left = report.left
    right = report.right
    left_dist = _zone_distance_for_choice(left)
    right_dist = _zone_distance_for_choice(right)
    margin = float(config.side_score_min_margin_m)
    if abs(left_dist - right_dist) >= margin:
        return "left" if left_dist > right_dist else "right"

    if left.occupied != right.occupied:
        return "right" if left.occupied else "left"

    if left.point_count != right.point_count:
        return "left" if left.point_count < right.point_count else "right"

    if current_side in {"left", "right"} and current_side in candidates:
        return current_side
    return _valid_tie_break(config)


def _decision(timestamp, command, kind, direction, *, ratio=1.0, rules=(), reason=""):
    return AvoidanceDecision(
        timestamp=timestamp,
        safe_command=command,
        decision_kind=kind,
        recommended_direction=direction,
        slowdown_ratio=ratio,
        triggered_safety_rules=tuple(rules),
        reason=reason,
    )


def plan_avoidance(
    desired: MotionCommand,
    report: ObstacleReport,
    *,
    lateral_vy_mps=0.08,
    forward_vx_when_avoiding_mps=0.03,
    min_slowdown_ratio=0.25,
    slowdown_distance_m=1.0,
    allow_left_avoidance=True,
    allow_right_avoidance=True,
    allow_planner_backward_escape=False,
) -> AvoidanceDecision:
    """Produce a *candidate* avoidance decision from a desired command and report.

    The returned :class:`AvoidanceDecision` is a proposal: its ``safe_command``
    carries the command the planner would like to execute, *before* velocity
    limiting and finite-value enforcement. A later ``limit_decision`` call turns
    this candidate into the *final* decision whose ``safe_command`` is the value
    actually emitted to motion output.
    """
    if not report.valid:
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason=report.reason or "radar_invalid",
            rules=("radar_invalid_or_timeout",),
        )
    if desired.kind.value == "stop":
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source=desired.source,
            reason=desired.reason or "manual_stop",
            rules=("manual_stop",),
        )
    if desired.omega != 0.0 and report.rotation.blocked:
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason="rotation_blocked",
            rules=("rotation_blocked",),
        )
    if desired.vy > 0.0 and report.left.blocked:
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason="left_blocked",
            rules=("desired_direction_blocked",),
        )
    if desired.vy < 0.0 and report.right.blocked:
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason="right_blocked",
            rules=("desired_direction_blocked",),
        )
    if desired.vx < 0.0 and report.rear.blocked:
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason="rear_blocked",
            rules=("desired_direction_blocked",),
        )
    if desired.vx > 0.0 and report.front.blocked:
        if allow_left_avoidance and not report.left.blocked:
            cmd = MotionCommand.move(
                desired.timestamp,
                source="avoidance",
                vx=forward_vx_when_avoiding_mps,
                vy=lateral_vy_mps,
                reason="avoid_left",
            )
            return _decision(
                desired.timestamp,
                cmd,
                DecisionKind.AVOID_LEFT,
                Direction.LEFT,
                rules=("front_blocked",),
                reason="avoid_left",
            )
        if allow_right_avoidance and not report.right.blocked:
            cmd = MotionCommand.move(
                desired.timestamp,
                source="avoidance",
                vx=forward_vx_when_avoiding_mps,
                vy=-lateral_vy_mps,
                reason="avoid_right",
            )
            return _decision(
                desired.timestamp,
                cmd,
                DecisionKind.AVOID_RIGHT,
                Direction.RIGHT,
                rules=("front_blocked",),
                reason="avoid_right",
            )
        if allow_planner_backward_escape and _rear_is_clear(report):
            cmd = MotionCommand.move(
                desired.timestamp,
                source="avoidance",
                vx=-forward_vx_when_avoiding_mps,
                vy=0.0,
                omega=0.0,
                reason="escape_backward",
            )
            return _decision(
                desired.timestamp,
                cmd,
                DecisionKind.NORMAL,
                Direction.BACK,
                rules=("front_left_right_blocked_escape",),
                reason="escape_backward",
            )
        return AvoidanceDecision.stop(
            timestamp=desired.timestamp,
            source="safety",
            reason="front_left_right_blocked",
            rules=("desired_direction_blocked",),
        )
    if desired.vx > 0.0 and report.front.occupied and report.front.min_distance_m is not None:
        if report.front.min_distance_m <= slowdown_distance_m:
            ratio = max(min_slowdown_ratio, report.front.min_distance_m / slowdown_distance_m)
            cmd = MotionCommand.move(
                desired.timestamp,
                source="avoidance",
                vx=desired.vx * ratio,
                vy=desired.vy,
                omega=desired.omega,
                reason="front_slowdown",
            )
            return _decision(
                desired.timestamp,
                cmd,
                DecisionKind.SLOWDOWN,
                Direction.FORWARD,
                ratio=ratio,
                rules=("slowdown",),
                reason="front_slowdown",
            )
    direction = Direction.FORWARD
    if desired.vy > 0.0:
        direction = Direction.LEFT
    elif desired.vy < 0.0:
        direction = Direction.RIGHT
    elif desired.vx < 0.0:
        direction = Direction.BACK
    return _decision(
        desired.timestamp,
        desired,
        DecisionKind.NORMAL,
        direction,
        reason=desired.reason,
    )


@dataclass
class StatefulAvoidancePlanner:
    config: object
    state: AvoidanceState = AvoidanceState.NORMAL
    selected_side: str = ""
    front_clear_count: int = 0
    state_entered_at: float = 0.0
    last_side_switch_at: float = -1.0e9
    return_vy: float = 0.0

    @property
    def _avoidance_config(self):
        return self.config.avoidance

    def _stateless(self, desired: MotionCommand, report: ObstacleReport) -> AvoidanceDecision:
        cfg = self._avoidance_config
        return plan_avoidance(
            desired,
            report,
            lateral_vy_mps=cfg.lateral_vy_mps,
            forward_vx_when_avoiding_mps=cfg.forward_vx_when_avoiding_mps,
            slowdown_distance_m=self.config.zones.slowdown_distance_m,
            min_slowdown_ratio=cfg.min_slowdown_ratio,
            allow_left_avoidance=cfg.allow_left_avoidance,
            allow_right_avoidance=cfg.allow_right_avoidance,
            allow_planner_backward_escape=cfg.allow_planner_backward_escape,
        )

    def _set_state(self, state: AvoidanceState, *, now_s: float, side: str = "") -> None:
        if self.state is not state:
            self.state = state
            self.state_entered_at = now_s
        if side:
            self.selected_side = side

    def _front_is_clear(self, report: ObstacleReport) -> bool:
        return not report.front.blocked and not report.front.occupied

    def _front_needs_avoidance(self, report: ObstacleReport) -> bool:
        if report.front.blocked:
            return True
        if report.front.occupied and report.front.min_distance_m is not None:
            return report.front.min_distance_m <= self.config.zones.slowdown_distance_m
        return False

    def _current_side_zone(self, report: ObstacleReport):
        return _side_zone(report, self.selected_side)

    def _can_switch_to(self, side: str, *, now_s: float) -> bool:
        return bool(side) and side != self.selected_side and (
            now_s - self.last_side_switch_at >= self._avoidance_config.side_switch_cooldown_s
        )

    def _avoid_decision(self, desired: MotionCommand, side: str, *, reason: str = "") -> AvoidanceDecision:
        cfg = self._avoidance_config
        vy = cfg.lateral_vy_mps if side == "left" else -cfg.lateral_vy_mps
        kind = DecisionKind.AVOID_LEFT if side == "left" else DecisionKind.AVOID_RIGHT
        direction = Direction.LEFT if side == "left" else Direction.RIGHT
        label = reason or ("avoid_left" if side == "left" else "avoid_right")
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=cfg.forward_vx_when_avoiding_mps,
            vy=vy,
            omega=0.0,
            reason=label,
        )
        return _decision(
            desired.timestamp,
            cmd,
            kind,
            direction,
            rules=("front_blocked",),
            reason=label,
        )

    def _clearing_decision(self, desired: MotionCommand) -> AvoidanceDecision:
        cfg = self._avoidance_config
        side = self.selected_side or _valid_tie_break(cfg)
        vy = cfg.lateral_vy_mps if side == "left" else -cfg.lateral_vy_mps
        kind = DecisionKind.AVOID_LEFT if side == "left" else DecisionKind.AVOID_RIGHT
        direction = Direction.LEFT if side == "left" else Direction.RIGHT
        reason = f"clearing_{side}"
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=desired.vx,
            vy=vy,
            omega=0.0,
            reason=reason,
        )
        return _decision(desired.timestamp, cmd, kind, direction, rules=("clearing",), reason=reason)

    def _escape_from_blocked_back_or_rotation(
        self,
        desired: MotionCommand,
        report: ObstacleReport,
        *,
        now_s: float,
    ) -> AvoidanceDecision:
        cfg = self._avoidance_config
        self._set_state(AvoidanceState.NORMAL, now_s=now_s)
        self.selected_side = ""
        self.front_clear_count = 0
        if self._front_is_clear(report):
            cmd = MotionCommand.move(
                desired.timestamp,
                source="avoidance",
                vx=cfg.forward_vx_when_avoiding_mps,
                vy=0.0,
                omega=0.0,
                reason="escape_forward",
            )
            return _decision(
                desired.timestamp,
                cmd,
                DecisionKind.NORMAL,
                Direction.FORWARD,
                rules=("rear_or_rotation_blocked_escape",),
                reason="escape_forward",
            )

        side = choose_avoidance_side(report, current_side=None, config=cfg)
        if side is None:
            return self._safe_stop(desired, now_s=now_s)
        vy = cfg.lateral_vy_mps if side == "left" else -cfg.lateral_vy_mps
        kind = DecisionKind.AVOID_LEFT if side == "left" else DecisionKind.AVOID_RIGHT
        direction = Direction.LEFT if side == "left" else Direction.RIGHT
        reason = f"escape_{side}"
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=0.0,
            vy=vy,
            omega=0.0,
            reason=reason,
        )
        return _decision(
            desired.timestamp,
            cmd,
            kind,
            direction,
            rules=("rear_or_rotation_blocked_escape",),
            reason=reason,
        )

    def _escape_backward_from_blocked_front(
        self,
        desired: MotionCommand,
        report: ObstacleReport,
        *,
        now_s: float,
    ) -> AvoidanceDecision:
        cfg = self._avoidance_config
        if not cfg.allow_planner_backward_escape or not _rear_is_clear(report):
            return self._safe_stop(desired, now_s=now_s)
        self._set_state(AvoidanceState.NORMAL, now_s=now_s)
        self.selected_side = ""
        self.front_clear_count = 0
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=-cfg.forward_vx_when_avoiding_mps,
            vy=0.0,
            omega=0.0,
            reason="escape_backward",
        )
        return _decision(
            desired.timestamp,
            cmd,
            DecisionKind.NORMAL,
            Direction.BACK,
            rules=("front_left_right_blocked_escape",),
            reason="escape_backward",
        )

    def _suppress_blocked_rotation_while_backing(
        self,
        desired: MotionCommand,
        *,
        now_s: float,
    ) -> AvoidanceDecision:
        self._set_state(AvoidanceState.NORMAL, now_s=now_s)
        self.selected_side = ""
        self.front_clear_count = 0
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=desired.vx,
            vy=desired.vy,
            omega=0.0,
            reason="backward_rotation_suppressed",
        )
        return _decision(
            desired.timestamp,
            cmd,
            DecisionKind.NORMAL,
            Direction.BACK,
            rules=("rotation_blocked_suppressed",),
            reason="backward_rotation_suppressed",
        )

    def _return_forward_decision(self, desired: MotionCommand, *, now_s: float) -> AvoidanceDecision:
        cfg = self._avoidance_config
        self.return_vy *= cfg.return_vy_decay
        if abs(self.return_vy) <= cfg.return_vy_epsilon_mps:
            self.return_vy = 0.0
            self._set_state(AvoidanceState.NORMAL, now_s=now_s)
        cmd = MotionCommand.move(
            desired.timestamp,
            source="avoidance",
            vx=desired.vx,
            vy=self.return_vy,
            omega=0.0,
            reason="return_forward" if self.return_vy else desired.reason,
        )
        return _decision(
            desired.timestamp,
            cmd,
            DecisionKind.NORMAL,
            Direction.FORWARD,
            rules=("return_forward",) if self.return_vy else (),
            reason="return_forward" if self.return_vy else desired.reason,
        )

    def _safe_stop(self, desired: MotionCommand, *, now_s: float) -> AvoidanceDecision:
        self._set_state(AvoidanceState.SAFE_STOP, now_s=now_s)
        self.front_clear_count = 0
        return AvoidanceDecision.stop(
            desired.timestamp,
            source="safety",
            reason="stateful_safe_stop",
            rules=("stateful_safe_stop",),
        )

    def _handle_side_danger(self, desired: MotionCommand, report: ObstacleReport, *, now_s: float):
        cfg = self._avoidance_config
        side = choose_avoidance_side(report, current_side=self.selected_side, config=cfg)
        if self._can_switch_to(side, now_s=now_s):
            self.selected_side = side
            self.last_side_switch_at = now_s
            next_state = AvoidanceState.AVOID_LEFT if side == "left" else AvoidanceState.AVOID_RIGHT
            self._set_state(next_state, now_s=now_s, side=side)
            return self._avoid_decision(desired, side)
        return self._safe_stop(desired, now_s=now_s)

    def plan(self, desired: MotionCommand, report: ObstacleReport, *, now_s: float) -> AvoidanceDecision:
        cfg = self._avoidance_config
        if not report.valid:
            return self._safe_stop(desired, now_s=now_s)
        if desired.kind.value == "stop":
            self._set_state(AvoidanceState.NORMAL, now_s=now_s)
            self.selected_side = ""
            self.front_clear_count = 0
            return self._stateless(desired, report)
        if desired.vx < 0.0 and report.rear.blocked:
            return self._escape_from_blocked_back_or_rotation(
                desired,
                report,
                now_s=now_s,
            )
        if desired.vx < 0.0 and desired.omega != 0.0 and report.rotation.blocked:
            return self._suppress_blocked_rotation_while_backing(
                desired,
                now_s=now_s,
            )
        if desired.vx <= 0.0:
            self._set_state(AvoidanceState.NORMAL, now_s=now_s)
            self.selected_side = ""
            self.front_clear_count = 0
            return self._stateless(desired, report)

        if self.state in {AvoidanceState.NORMAL, AvoidanceState.SAFE_STOP}:
            if self._front_needs_avoidance(report):
                side = choose_avoidance_side(report, current_side=self.selected_side or None, config=cfg)
                if side is None:
                    return self._escape_backward_from_blocked_front(desired, report, now_s=now_s)
                next_state = AvoidanceState.AVOID_LEFT if side == "left" else AvoidanceState.AVOID_RIGHT
                self._set_state(next_state, now_s=now_s, side=side)
                self.front_clear_count = 0
                return self._avoid_decision(desired, side)
            self._set_state(AvoidanceState.NORMAL, now_s=now_s)
            self.selected_side = ""
            self.front_clear_count = 0
            return self._stateless(desired, report)

        if self.state in {AvoidanceState.AVOID_LEFT, AvoidanceState.AVOID_RIGHT}:
            if side_is_dangerous(self._current_side_zone(report), cfg):
                return self._handle_side_danger(desired, report, now_s=now_s)
            if self._front_is_clear(report):
                self.front_clear_count += 1
            else:
                self.front_clear_count = 0
            if self.front_clear_count >= cfg.avoid_clear_required_frames:
                self._set_state(AvoidanceState.CLEARING, now_s=now_s)
                self.return_vy = cfg.lateral_vy_mps if self.selected_side == "left" else -cfg.lateral_vy_mps
                return self._clearing_decision(desired)
            return self._avoid_decision(desired, self.selected_side)

        if self.state is AvoidanceState.CLEARING:
            if side_is_dangerous(self._current_side_zone(report), cfg):
                return self._handle_side_danger(desired, report, now_s=now_s)
            if now_s - self.state_entered_at < cfg.avoid_clearance_hold_s:
                return self._clearing_decision(desired)
            self._set_state(AvoidanceState.RETURN_FORWARD, now_s=now_s)
            return self._return_forward_decision(desired, now_s=now_s)

        if self.state is AvoidanceState.RETURN_FORWARD:
            if self._front_needs_avoidance(report):
                side = choose_avoidance_side(report, current_side=self.selected_side or None, config=cfg)
                if side is None:
                    return self._escape_backward_from_blocked_front(desired, report, now_s=now_s)
                next_state = AvoidanceState.AVOID_LEFT if side == "left" else AvoidanceState.AVOID_RIGHT
                self._set_state(next_state, now_s=now_s, side=side)
                self.front_clear_count = 0
                return self._avoid_decision(desired, side)
            if self.selected_side and side_is_dangerous(self._current_side_zone(report), cfg):
                return self._handle_side_danger(desired, report, now_s=now_s)
            return self._return_forward_decision(desired, now_s=now_s)

        return self._safe_stop(desired, now_s=now_s)

    def plan_and_limit(self, desired: MotionCommand, report: ObstacleReport, *, now_s: float) -> AvoidanceDecision:
        candidate = self.plan(desired, report, now_s=now_s)
        cfg = self._avoidance_config
        return limit_decision(
            candidate,
            max_vx=cfg.max_vx,
            max_vy=cfg.max_vy,
            max_omega=cfg.max_omega,
        )


# ---------------------------------------------------------------------------
# Robot Follow AO 运行时适配器
#
# 把 Robot Follow 的 ``MotionCommand``（跟随管线产物）与上面 stateful
# 避障 planner 使用的 AO 内部 ``MotionCommand`` 互相转换。避障规则与状态机
# 完全不变，仅做命令类型的进出翻译。
# ---------------------------------------------------------------------------
from dataclasses import dataclass, field

from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.ao.core.types import MotionCommand as AOMotionCommand
from robot_follow.core.types import MotionCommand as RobotFollowMotionCommand


@dataclass(frozen=True)
class DefaultAvoidanceConfig:
    lateral_vy_mps: float = 0.08
    forward_vx_when_avoiding_mps: float = 0.03
    min_slowdown_ratio: float = 0.25
    max_vx: float = 0.5
    max_vy: float = 0.2
    max_omega: float = 0.8
    allow_left_avoidance: bool = True
    allow_right_avoidance: bool = True
    allow_planner_backward_escape: bool = False
    side_switch_cooldown_s: float = 0.5
    avoid_clear_required_frames: int = 3
    avoid_clearance_hold_s: float = 0.3
    return_vy_decay: float = 0.5
    return_vy_epsilon_mps: float = 0.01
    side_score_min_margin_m: float = 0.1
    tie_break_side: str = "left"
    side_danger_distance_m: float = 0.35


@dataclass(frozen=True)
class DefaultZoneConfig:
    slowdown_distance_m: float = 1.0


@dataclass(frozen=True)
class DefaultPlannerConfig:
    avoidance: DefaultAvoidanceConfig = field(default_factory=DefaultAvoidanceConfig)
    zones: DefaultZoneConfig = field(default_factory=DefaultZoneConfig)


class AvoidancePlanner:
    def __init__(self, config: object | None = None) -> None:
        self._stateful = StatefulAvoidancePlanner(config or DefaultPlannerConfig())

    def filter_command(
        self,
        envelope: CommandEnvelope,
        report: ObstacleReport,
        *,
        now: float,
    ) -> RobotFollowMotionCommand:
        desired = _to_ao_motion_command(envelope)
        decision = self._stateful.plan_and_limit(desired, report, now_s=now)
        return _to_robot_follow_motion_command(decision.safe_command)


def _to_ao_motion_command(envelope: CommandEnvelope) -> AOMotionCommand:
    command = envelope.command
    if command.kind == "stop":
        return AOMotionCommand.stop(
            envelope.generated_at,
            source=envelope.source,
            reason=command.reason,
        )
    return AOMotionCommand.move(
        envelope.generated_at,
        source=envelope.source,
        vx=command.vx,
        vy=command.vy,
        omega=command.omega,
        reason=command.reason,
    )


def _to_robot_follow_motion_command(command: AOMotionCommand) -> RobotFollowMotionCommand:
    kind = "stop" if command.kind.value == "stop" else "move"
    return RobotFollowMotionCommand(
        kind=kind,
        vx=command.vx,
        vy=command.vy,
        omega=command.omega,
        reason=command.reason,
    )
