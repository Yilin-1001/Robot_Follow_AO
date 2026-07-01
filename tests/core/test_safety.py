"""Unit tests for the safety manager."""

import math
import time

from robot_follow.core.safety import SafetyConfig, SafetyInput, SafetyManager


def _make_input(
    *,
    distance_m: float = 1.5,
    azimuth_deg: float = 0.0,
    distance_jump: float = 0.0,
    angle_jump: float = 0.0,
    data_valid: bool = True,
    serial_error: bool = False,
    sdk_timeout: bool = False,
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
        sdk_timeout=sdk_timeout,
        sdk_error=sdk_error,
        manual_stop=manual_stop,
    )


def test_normal_is_safe():
    mgr = SafetyManager()
    v = mgr.check(_make_input())
    assert v.safe
    assert v.reason == "ok"


def test_distance_at_minimum_safe_boundary_is_safe():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=0.5,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert v.safe
    assert v.reason == "ok"


def test_below_minimum_safe_distance_is_too_close():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=0.49,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
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


def test_clock_injection_default_is_monotonic():
    """Default clock should be time.monotonic."""
    import time as _time

    mgr = SafetyManager()
    assert mgr.clock is _time.monotonic


def test_clock_injection_data_timeout_with_fake_clock():
    """Using an injected clock, verify data_timeout without real sleep."""
    fake_time = 100.0

    def fake_clock():
        return fake_time

    mgr = SafetyManager(clock=fake_clock)

    # Sample timestamp is 100.0, current fake time is 100.0 -> not timed out
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert v.safe

    # Advance fake clock past the 0.5s timeout
    fake_time = 100.6
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_timeout"


def test_data_invalid():
    mgr = SafetyManager()
    v = mgr.check(_make_input(data_valid=False))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_rejects_nan_distance():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_rejects_nan_azimuth():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=math.nan,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_rejects_infinite_jump_values():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=0.0,
        distance_jump=math.inf,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_manual_stop():
    mgr = SafetyManager()
    v = mgr.check(_make_input(manual_stop=True))
    assert not v.safe
    assert v.reason == "manual_stop"


def test_sdk_timeout():
    mgr = SafetyManager()
    v = mgr.check(_make_input(sdk_timeout=True))
    assert not v.safe
    assert v.reason == "sdk_timeout"


def test_sdk_error_takes_priority_over_sdk_timeout():
    mgr = SafetyManager()
    v = mgr.check(_make_input(sdk_timeout=True, sdk_error=True))
    assert not v.safe
    assert v.reason == "sdk_error"
