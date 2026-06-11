"""Unit tests for the follow controller."""

from robot_follow.follow_controller import FollowConfig, FollowController


def test_too_close_stop():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=0.6, azimuth_deg=0.0)
    assert out.vx == 0.0
    assert out.omega == 0.0
    assert out.reason == "too_close"


def test_large_angle_turn_in_place():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=30.0)
    assert out.vx == 0.0
    assert out.omega != 0.0
    assert out.reason == "aligning"


def test_forward_and_close_to_target():
    ctrl = FollowController()
    # Target distance is 1.3 m; at 1.3 m with 0 deg the robot should hold.
    out = ctrl.compute(distance_m=1.3, azimuth_deg=0.0)
    assert abs(out.vx) < 0.01
    assert abs(out.omega) < 0.01
    assert out.reason == "holding"


def test_medium_angle_slow_forward_and_turn():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=10.0)
    assert out.vx > 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_turning"
