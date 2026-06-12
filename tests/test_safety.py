"""Unit tests for the safety manager."""

import time

from robot_follow.safety import SafetyConfig, SafetyInput, SafetyManager


def _make_input(
    *,
    distance_m: float = 1.5,
    azimuth_deg: float = 0.0,
    distance_jump: float = 0.0,
    angle_jump: float = 0.0,
    data_valid: bool = True,
    serial_error: bool = False,
    sdk_error: bool = False,
    manual_stop: bool = False,
) -> SafetyInput:
    return SafetyInput(
        timestamp=time.monotonic(),
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        distance_jump=distance_jump,
        angle_jump=angle_jump,
        data_valid=data_valid,
        serial_error=serial_error,
        sdk_error=sdk_error,
        manual_stop=manual_stop,
    )


def test_normal_is_safe():
    mgr = SafetyManager()
    v = mgr.check(_make_input())
    assert v.safe
    assert v.reason == "ok"


def test_distance_at_minimum_safe_boundary_is_safe():
    mgr = SafetyManager()
    v = mgr.check(_make_input(distance_m=0.5))
    assert v.safe
    assert v.reason == "ok"


def test_below_minimum_safe_distance_is_too_close():
    mgr = SafetyManager()
    v = mgr.check(_make_input(distance_m=0.49))
    assert not v.safe
    assert v.reason == "too_close"


def test_distance_jump():
    mgr = SafetyManager()
    v = mgr.check(_make_input(distance_jump=0.6))
    assert not v.safe
    assert v.reason == "distance_jump"


def test_angle_jump():
    mgr = SafetyManager()
    v = mgr.check(_make_input(angle_jump=50.0))
    assert not v.safe
    assert v.reason == "angle_jump"
