import pytest

from robot_follow.core.types import MotionCommand


def test_motion_command_rejects_unknown_kind():
    with pytest.raises(ValueError):
        MotionCommand(kind="pause", vx=0.0, vy=0.0, omega=0.0, reason="bad")


def test_stop_command_must_have_zero_velocity():
    with pytest.raises(ValueError):
        MotionCommand(kind="stop", vx=0.1, vy=0.0, omega=0.0, reason="bad")


def test_move_command_accepts_zero_velocity_for_holding():
    cmd = MotionCommand(kind="move", vx=0.0, vy=0.0, omega=0.0, reason="holding")
    assert cmd.kind == "move"
