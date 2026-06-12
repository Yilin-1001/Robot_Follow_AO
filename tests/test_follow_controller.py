"""Unit tests for the follow controller."""

import pytest

from robot_follow.follow_controller import FollowConfig, FollowController


def test_too_close_stop():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=0.49, azimuth_deg=0.0)
    assert out.vx == 0.0
    assert out.omega == 0.0
    assert out.reason == "too_close"


def test_large_angle_turn_in_place():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=30.0)
    assert out.vx == 0.0
    assert out.omega != 0.0
    assert out.reason == "aligning"


@pytest.mark.parametrize("distance_m", [1.35, 1.5, 1.65])
def test_hold_band_holds_position(distance_m):
    ctrl = FollowController()
    out = ctrl.compute(distance_m=distance_m, azimuth_deg=0.0)
    assert abs(out.vx) < 0.01
    assert abs(out.omega) < 0.01
    assert out.reason == "holding"


def test_close_but_safe_distance_reverses():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=1.0, azimuth_deg=0.0)
    assert out.vx < 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_backward"


def test_backward_speed_is_clamped():
    ctrl = FollowController(FollowConfig(max_backward_vx=0.05))
    out = ctrl.compute(distance_m=0.5, azimuth_deg=0.0)
    assert out.vx == pytest.approx(-0.05)
    assert out.reason == "tracking_backward"


def test_far_distance_moves_forward():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=0.0)
    assert out.vx > 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_forward"


def test_small_static_angle_stays_in_deadzone():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=6.0)
    assert out.vx > 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_forward"


def test_medium_angle_slow_forward_and_turn():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=8.0)
    assert out.vx > 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_turning"
