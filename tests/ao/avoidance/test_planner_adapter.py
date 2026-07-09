from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand


def test_ao_planner_accepts_robot_follow_motion_command() -> None:
    from robot_follow.ao.avoidance.planner import AvoidancePlanner
    from robot_follow.ao.core.types import ObstacleReport

    planner = AvoidancePlanner()
    raw = MotionCommand(kind="move", vx=0.3, vy=0.0, omega=0.0, reason="follow")
    envelope = CommandEnvelope(command=raw, generated_at=10.0, source="robot_follow")
    clear_report = ObstacleReport.clear(timestamp=10.0)

    safe = planner.filter_command(envelope, clear_report, now=10.01)

    assert isinstance(safe, MotionCommand)
    assert safe.kind == "move"
    assert safe.vx == raw.vx
    assert safe.reason in {"follow", "clear"}


def test_ao_planner_overrides_command_when_front_zone_blocked() -> None:
    from robot_follow.ao.avoidance.planner import AvoidancePlanner
    from robot_follow.ao.core.types import ObstacleReport

    planner = AvoidancePlanner()
    raw = MotionCommand(kind="move", vx=0.3, vy=0.0, omega=0.0, reason="follow")
    envelope = CommandEnvelope(command=raw, generated_at=10.0, source="robot_follow")
    blocked_report = ObstacleReport.front_blocked(timestamp=10.0, min_distance_m=0.35)

    safe = planner.filter_command(envelope, blocked_report, now=10.01)

    assert isinstance(safe, MotionCommand)
    assert safe.kind == "stop"
    assert safe.reason != "follow"
