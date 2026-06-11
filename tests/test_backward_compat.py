"""Backward compatibility tests: verify old import paths still work."""

import pytest


def test_filtering_old_import():
    """Legacy path robot_follow.filtering should still work."""
    from robot_follow.filtering import FilteredTarget, TargetFilter

    f = TargetFilter()
    result = f.update(distance_m=2.0, azimuth_deg=5.0)
    assert result.initialized
    assert isinstance(result, FilteredTarget)


def test_safety_old_import():
    """Legacy path robot_follow.safety should still work."""
    from robot_follow.safety import SafetyConfig, SafetyInput, SafetyManager, SafetyVerdict

    mgr = SafetyManager()
    inp = SafetyInput(
        timestamp=0.0,  # Will trigger data_timeout
        distance_m=1.5,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    )
    v = mgr.check(inp)
    assert isinstance(v, SafetyVerdict)
    assert not v.safe  # data_timeout


def test_follow_controller_old_import():
    """Legacy path robot_follow.follow_controller should still work."""
    from robot_follow.follow_controller import FollowConfig, FollowController, FollowOutput

    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=0.0)
    assert isinstance(out, FollowOutput)
    assert out.vx > 0.0


def test_core_import():
    """New path robot_follow.core should work."""
    from robot_follow.core import (
        TargetSample,
        MotionCommand,
        FilteredTarget,
        TargetFilter,
        SafetyConfig,
        SafetyInput,
        SafetyManager,
        SafetyVerdict,
        FollowConfig,
        FollowController,
        FollowOutput,
    )

    sample = TargetSample(timestamp=0.0, distance_m=1.5, azimuth_deg=0.0)
    assert sample.valid

    cmd = MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="test")
    assert cmd.kind == "move"