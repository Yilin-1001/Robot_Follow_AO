"""Tests for command slew rate limiter."""

from __future__ import annotations

import time

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.config.schema import OutputMotionConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import MotionIntent
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter


def make_intent(
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    sequence: int = 1,
    reason: str = "follow",
) -> MotionIntent:
    """Create a motion intent."""
    return MotionIntent(
        sequence=sequence,
        vx=vx,
        vy=vy,
        omega=omega,
        reason=reason,
        generated_at_s=time.monotonic(),
    )


class TestBasicLimiting:
    """Test basic slew rate limiting behavior."""

    def test_first_call_returns_zero(self):
        """First call should return zero motion."""
        config = OutputMotionConfig(
            max_vx=0.10,
            max_abs_vy=0.08,
            max_abs_omega=0.25,
            accel_vx=0.15,
            decel_vx=0.30,
            accel_vy=0.15,
            decel_vy=0.30,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        intent = make_intent(vx=0.10)
        now_s = time.monotonic()
        limited = limiter.limit(intent, now_s=now_s)

        # First call always returns zero
        assert limited.vx == 0.0
        assert limited.vy == 0.0
        assert limited.omega == 0.0

    def test_acceleration_is_limited(self):
        """Acceleration should be limited by config."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=0.15,  # 0.15 m/s^2
            decel_vx=0.30,
            accel_vy=0.15,
            decel_vy=0.30,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call returns zero
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Second call after 10ms
        now_s += 0.010
        intent = make_intent(vx=0.10)
        limited = limiter.limit(intent, now_s=now_s)

        # After 10ms, max acceleration = 0.15 * 0.010 = 0.0015 m/s
        # So vx should be 0.0015, not 0.10
        expected_vx = 0.15 * 0.010
        assert abs(limited.vx - expected_vx) < 1e-6

    def test_velocity_is_clipped_to_max(self):
        """Velocity should be clipped to max limits."""
        config = OutputMotionConfig(
            max_vx=0.10,
            max_abs_vy=0.08,
            max_abs_omega=0.25,
            accel_vx=10.0,  # Very high acceleration
            accel_vy=10.0,
            accel_omega=10.0,
            decel_vx=10.0,
            decel_vy=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=1.0, vy=1.0, omega=1.0), now_s=now_s)

        # Second call after sufficient time
        now_s += 1.0
        limited = limiter.limit(
            make_intent(vx=1.0, vy=1.0, omega=1.0),
            now_s=now_s,
        )

        # Should be clipped to max
        assert limited.vx == config.max_vx
        assert limited.vy == config.max_abs_vy
        assert limited.omega == config.max_abs_omega

    def test_negative_vx_is_clipped_to_zero(self):
        """Negative vx should be clipped to zero (no reverse)."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=10.0,
            decel_vx=10.0,
            accel_vy=10.0,
            decel_vy=10.0,
            accel_omega=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=-0.10), now_s=now_s)

        # Second call
        now_s += 1.0
        limited = limiter.limit(make_intent(vx=-0.10), now_s=now_s)

        # vx should be zero (no reverse allowed)
        assert limited.vx == 0.0


class TestTimeClamping:
    """Test time delta clamping behavior."""

    def test_dt_clamped_to_minimum_1ms(self):
        """Time delta should be clamped to minimum 1ms."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=0.15,
            decel_vx=0.30,
            accel_vy=0.15,
            decel_vy=0.30,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Second call with very small dt (< 1ms)
        now_s += 0.0001
        limited = limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Should use dt=0.001 (1ms) for calculation
        # max delta = 0.15 * 0.001 = 0.00015
        expected_vx = 0.15 * 0.001
        assert abs(limited.vx - expected_vx) < 1e-6

    def test_dt_clamped_to_maximum_100ms(self):
        """Time delta should be clamped to maximum 100ms."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=0.15,
            decel_vx=0.30,
            accel_vy=0.15,
            decel_vy=0.30,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Second call with large dt (> 100ms)
        now_s += 1.0
        limited = limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Should use dt=0.10 (100ms) for calculation
        # max delta = 0.15 * 0.10 = 0.015
        expected_vx = 0.15 * 0.10
        assert abs(limited.vx - expected_vx) < 1e-6


class TestDeceleration:
    """Test deceleration limiting."""

    def test_deceleration_is_limited(self):
        """Deceleration should be limited separately from acceleration."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=0.15,
            decel_vx=0.30,  # Higher decel than accel
            accel_vy=0.15,
            decel_vy=0.30,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # Ramp up to 0.10 m/s
        now_s = time.monotonic()
        for _ in range(20):
            now_s += 0.050
            limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Now request zero (deceleration)
        now_s += 0.010
        limited = limiter.limit(make_intent(vx=0.0), now_s=now_s)

        # After 10ms, max deceleration = 0.30 * 0.010 = 0.003 m/s
        # vx should decrease from ~0.10 by at most 0.003
        assert limited.vx > 0.10 - 0.003 - 1e-6


class TestReset:
    """Test limiter reset functionality."""

    def test_reset_returns_to_zero(self):
        """Reset should return limiter to zero state."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=10.0,
            decel_vx=10.0,
            accel_vy=10.0,
            decel_vy=10.0,
            accel_omega=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # Ramp up
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=0.10), now_s=now_s)
        now_s += 1.0
        limiter.limit(make_intent(vx=0.10), now_s=now_s)

        # Reset
        limiter.reset()

        # Next call should return zero (first-call behavior)
        now_s += 0.050
        limited = limiter.limit(make_intent(vx=0.10), now_s=now_s)
        assert limited.vx == 0.0


class TestIntentPreservation:
    """Test that intent metadata is preserved."""

    def test_sequence_is_preserved(self):
        """Sequence should be preserved in limited intent."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=10.0,
            decel_vx=10.0,
            accel_vy=10.0,
            decel_vy=10.0,
            accel_omega=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)

        intent = make_intent(vx=0.10, sequence=42)
        now_s = time.monotonic()
        limited = limiter.limit(intent, now_s=now_s)

        assert limited.sequence == 42

    def test_reason_is_preserved(self):
        """Reason should be preserved in limited intent."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=10.0,
            decel_vx=10.0,
            accel_vy=10.0,
            decel_vy=10.0,
            accel_omega=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)

        intent = make_intent(vx=0.10, reason="turn_in_place")
        now_s = time.monotonic()
        limited = limiter.limit(intent, now_s=now_s)

        assert limited.reason == "turn_in_place"


class TestMultiComponentLimiting:
    """Test limiting of multiple velocity components."""

    def test_all_components_limited_independently(self):
        """Each velocity component should be limited independently."""
        config = OutputMotionConfig(
            max_vx=0.10,
            max_abs_vy=0.08,
            max_abs_omega=0.25,
            accel_vx=0.15,
            decel_vx=0.30,
            accel_vy=0.20,
            decel_vy=0.40,
            accel_omega=0.50,
            decel_omega=1.00,
        )
        limiter = CommandSlewRateLimiter(config=config)

        # First call
        now_s = time.monotonic()
        limiter.limit(make_intent(vx=0.10, vy=0.08, omega=0.25), now_s=now_s)

        # Second call
        now_s += 0.010
        limited = limiter.limit(
            make_intent(vx=0.10, vy=0.08, omega=0.25),
            now_s=now_s,
        )

        # Each component should have different max delta
        # vx: 0.15 * 0.010 = 0.0015
        # vy: 0.20 * 0.010 = 0.0020
        # omega: 0.50 * 0.010 = 0.0050
        assert abs(limited.vx - 0.0015) < 1e-6
        assert abs(limited.vy - 0.0020) < 1e-6
        assert abs(limited.omega - 0.0050) < 1e-6
