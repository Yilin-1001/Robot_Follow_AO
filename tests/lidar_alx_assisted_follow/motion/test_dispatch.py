"""Tests for strategy dispatch to G1 bridge."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.lidar_alx_assisted_follow.config.schema import OutputMotionConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    MotionIntent,
    StopKind,
    StopReason,
    StopSnapshot,
    StrategyResult,
)
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.motion.dispatch import dispatch_strategy_or_stop


@dataclass
class FakeBridge:
    """Fake bridge for testing."""

    commands: List[MotionCommand] = field(default_factory=list)
    tick_count: int = 0

    def dispatch(self, command: MotionCommand) -> None:
        """Record dispatched command."""
        self.commands.append(command)

    def tick(self, now: Optional[float] = None) -> None:
        """Record tick."""
        self.tick_count += 1


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


def make_strategy(
    sequence: int = 1,
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    stop_reason: Optional[StopReason] = None,
) -> StrategyResult:
    """Create a strategy result."""
    apf_intent = make_intent(vx=vx, vy=vy, omega=omega, sequence=sequence)
    dispatched_intent = make_intent(vx=vx, vy=vy, omega=omega, sequence=sequence)

    return StrategyResult(
        sequence=sequence,
        apf_requested=apf_intent,
        dispatched=dispatched_intent,
        safety_complete=True,
        stop_reason=stop_reason,
        compute_ms=5.0,
    )


def make_latch_snapshot(
    requested: bool = False,
    kind: Optional[StopKind] = None,
    reason: Optional[StopReason] = None,
    start_after_sequence: int = 0,
) -> StopSnapshot:
    """Create a stop latch snapshot."""
    return StopSnapshot(
        requested=requested,
        kind=kind,
        reason=reason,
        start_after_sequence=start_after_sequence,
        version=1,
    )


class TestStopLatchPriority:
    """Test that StopLatch has highest priority."""

    def test_hard_stop_latch_dispatches_stop(self):
        """HARD stop latch should dispatch stop immediately."""
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
        bridge = FakeBridge()

        # Strategy wants to move
        strategy = make_strategy(sequence=10, vx=0.10)

        # But stop latch is set
        latch_snapshot = make_latch_snapshot(
            requested=True,
            kind=StopKind.HARD,
            reason=StopReason.CONTROLLER_DISCONNECTED,
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        # Should dispatch stop, not move
        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].vx == 0.0
        assert bridge.commands[0].reason == "controller_disconnected"

    def test_soft_stop_latch_dispatches_stop(self):
        """SOFT stop latch should also dispatch stop."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(
            requested=True,
            kind=StopKind.SOFT,
            reason=StopReason.CONFIDENCE_LOW,
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].reason == "confidence_low"

    def test_latch_reason_is_preserved(self):
        """StopLatch reason should be preserved in stop command."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(
            requested=True,
            kind=StopKind.HARD,
            reason=StopReason.WRISTBAND_OFF,
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert bridge.commands[0].reason == "wristband_off"

    def test_live_latch_is_rechecked_immediately_before_nonzero_dispatch(self):
        """A STOP arriving after strategy evaluation must still win at the bridge."""
        class LateStopLatch:
            def snapshot(self):
                return make_latch_snapshot(
                    requested=True,
                    kind=StopKind.HARD,
                    reason=StopReason.CONTROLLER_DISCONNECTED,
                )

        limiter = CommandSlewRateLimiter(
            OutputMotionConfig(
                accel_vx=10.0,
                decel_vx=10.0,
                accel_vy=10.0,
                decel_vy=10.0,
                accel_omega=10.0,
                decel_omega=10.0,
            )
        )
        bridge = FakeBridge()

        limiter.limit(make_intent(sequence=1, vx=0.1), now_s=1.0)

        dispatch_strategy_or_stop(
            make_strategy(sequence=2, vx=0.1),
            make_latch_snapshot(requested=False, start_after_sequence=0),
            bridge,
            limiter,
            now_s=1.1,
            latch=LateStopLatch(),
        )

        assert bridge.commands[-1].kind == "stop"
        assert bridge.commands[-1].reason == "controller_disconnected"


class TestStrategyStopReason:
    """Test that strategy stop_reason triggers stop."""

    def test_target_too_close_dispatches_stop(self):
        """Strategy stop_reason should dispatch stop."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(
            sequence=10,
            stop_reason=StopReason.TARGET_TOO_CLOSE,
        )
        latch_snapshot = make_latch_snapshot(requested=False)

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].reason == "target_too_close"

    def test_all_directions_unsafe_dispatches_stop(self):
        """All directions unsafe should dispatch stop."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(
            sequence=10,
            stop_reason=StopReason.ALL_DIRECTIONS_UNSAFE,
        )
        latch_snapshot = make_latch_snapshot(requested=False)

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert bridge.commands[0].kind == "stop"


class TestSequenceGate:
    """Test sequence > start_after_sequence gate."""

    def test_sequence_equal_to_start_after_dispatches_stop(self):
        """Sequence equal to start_after_sequence should dispatch stop."""
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
        bridge = FakeBridge()

        # Strategy sequence = 10
        strategy = make_strategy(sequence=10, vx=0.10)

        # start_after_sequence = 10 (same)
        latch_snapshot = make_latch_snapshot(
            requested=False,
            start_after_sequence=10,
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        # Should dispatch stop (sequence must be > start_after_sequence)
        assert bridge.commands[0].kind == "stop"

    def test_sequence_greater_than_start_after_allows_motion(self):
        """Sequence > start_after_sequence should allow motion."""
        config = OutputMotionConfig(
            max_vx=0.10,
            accel_vx=10.0,  # High accel for quick ramp
            decel_vx=10.0,
            accel_vy=10.0,
            decel_vy=10.0,
            accel_omega=10.0,
            decel_omega=10.0,
        )
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        # First call to initialize limiter
        now_s = time.monotonic()
        init_strategy = make_strategy(sequence=10, vx=0.10)
        init_latch = make_latch_snapshot(requested=False, start_after_sequence=8)
        dispatch_strategy_or_stop(init_strategy, init_latch, bridge, limiter, now_s=now_s)

        # Clear commands
        bridge.commands.clear()

        # Second call with sufficient time for accel
        now_s += 1.0
        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(
            requested=False,
            start_after_sequence=8,  # 8 < 10, so motion allowed
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter, now_s=now_s)

        # Should dispatch move (not stop)
        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "move"
        assert bridge.commands[0].vx > 0


class TestZeroIntent:
    """Test zero intent handling."""

    def test_zero_intent_dispatches_stop(self):
        """Zero motion intent should dispatch stop."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(sequence=10, vx=0.0, vy=0.0, omega=0.0)
        latch_snapshot = make_latch_snapshot(requested=False)

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "stop"


class TestSlewRateLimiting:
    """Test that slew rate limiting is applied."""

    def test_motion_is_limited(self):
        """Motion should pass through slew rate limiter."""
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
        bridge = FakeBridge()

        # First call
        now_s = time.monotonic()
        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(requested=False, start_after_sequence=8)
        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter, now_s=now_s)

        # First call returns zero
        assert bridge.commands[0].vx == 0.0

        # Second call
        bridge.commands.clear()
        now_s += 0.010
        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter, now_s=now_s)

        # Should be limited by acceleration
        # max delta = 0.15 * 0.010 = 0.0015
        assert 0 < bridge.commands[0].vx < 0.01  # Limited, not full 0.10


class TestLimiterReset:
    """Test that mandatory stop resets the limiter."""

    def test_stop_latch_resets_limiter(self):
        """Stop latch should reset limiter state."""
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
        bridge = FakeBridge()

        # Ramp up limiter
        now_s = time.monotonic()
        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(requested=False, start_after_sequence=8)
        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter, now_s=now_s)

        # Stop latch
        bridge.commands.clear()
        stop_latch = make_latch_snapshot(
            requested=True,
            kind=StopKind.HARD,
            reason=StopReason.USER_STOP,
        )
        dispatch_strategy_or_stop(strategy, stop_latch, bridge, limiter)

        # Limiter should be reset
        assert limiter.last_vx == 0.0
        assert limiter.last_time_s is None


class TestTickCall:
    """Test that bridge.tick is called."""

    def test_tick_called_on_motion(self):
        """tick should be called when motion is dispatched."""
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
        bridge = FakeBridge()

        # First call
        now_s = time.monotonic()
        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(requested=False, start_after_sequence=8)
        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter, now_s=now_s)

        # tick should be called
        assert bridge.tick_count > 0

    def test_tick_not_called_on_stop(self):
        """tick should not be called for mandatory stop."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(sequence=10, vx=0.10)
        latch_snapshot = make_latch_snapshot(
            requested=True,
            kind=StopKind.HARD,
            reason=StopReason.USER_STOP,
        )

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        # tick should not be called for mandatory stop
        assert bridge.tick_count == 0


class TestRealStopReasons:
    """Test all real stop reasons from StopReason enum."""

    @pytest.mark.parametrize("stop_reason", [
        StopReason.USER_STOP,
        StopReason.CONTROLLER_DISCONNECTED,
        StopReason.WRISTBAND_OFF,
        StopReason.SERIAL_ERROR,
        StopReason.READY_TIMEOUT,
        StopReason.CLOUD_STALE,
        StopReason.TARGET_LOST,
        StopReason.BRIDGE_FAULT,
        StopReason.LIFECYCLE_NOT_READY,
        StopReason.TARGET_TOO_CLOSE,
        StopReason.NON_TARGET_HARD_CLEARANCE,
        StopReason.UNSAFE_ROTATION,
        StopReason.ALL_DIRECTIONS_UNSAFE,
        StopReason.STRATEGY_TIMEOUT,
        StopReason.INVALID_INPUT,
    ])
    def test_all_stop_reasons_dispatch_stop(self, stop_reason: StopReason):
        """All stop reasons should dispatch stop command."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        strategy = make_strategy(sequence=10, stop_reason=stop_reason)
        latch_snapshot = make_latch_snapshot(requested=False)

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert len(bridge.commands) == 1
        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].reason == stop_reason.value

    def test_soft_stop_reasons_preserved(self):
        """SOFT stop reasons should be preserved correctly."""
        config = OutputMotionConfig()
        limiter = CommandSlewRateLimiter(config=config)
        bridge = FakeBridge()

        # Test SOFT stop: confidence_low
        strategy = make_strategy(sequence=10, stop_reason=StopReason.CONFIDENCE_LOW)
        latch_snapshot = make_latch_snapshot(requested=False)

        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].reason == "confidence_low"

        # Test SOFT stop: alx_stale
        bridge.commands.clear()
        strategy = make_strategy(sequence=10, stop_reason=StopReason.ALX_STALE)
        dispatch_strategy_or_stop(strategy, latch_snapshot, bridge, limiter)

        assert bridge.commands[0].kind == "stop"
        assert bridge.commands[0].reason == "alx_stale"
