from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand


def test_command_envelope_reports_age() -> None:
    command = MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="follow")
    envelope = CommandEnvelope(command=command, generated_at=10.0, source="robot_follow")

    assert envelope.age_s(now=10.125) == 0.125


def test_stale_envelope_detection() -> None:
    command = MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="follow")
    envelope = CommandEnvelope(command=command, generated_at=10.0, source="robot_follow")

    assert envelope.is_stale(now=10.25, max_age_s=0.2) is True
    assert envelope.is_stale(now=10.19, max_age_s=0.2) is False


def test_stop_factory_uses_robot_follow_motion_command() -> None:
    envelope = CommandEnvelope.stop(reason="radar_not_ready", generated_at=20.0)

    assert envelope.command.kind == "stop"
    assert envelope.command.reason == "radar_not_ready"
    assert envelope.generated_at == 20.0
    assert envelope.source == "ao_runtime"
