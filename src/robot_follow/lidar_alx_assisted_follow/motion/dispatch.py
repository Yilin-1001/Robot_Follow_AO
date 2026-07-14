"""Dispatch strategy result or stop to G1 bridge."""

from __future__ import annotations

import time
from typing import Optional, Protocol

from robot_follow.core.types import MotionCommand
from robot_follow.lidar_alx_assisted_follow.domain.models import (
    MotionIntent,
    StopReason,
    StopSnapshot,
    StrategyResult,
)
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter


class G1BridgeProtocol(Protocol):
    """Protocol for G1 bridge interface."""

    def dispatch(self, command: MotionCommand) -> None:
        """Dispatch motion command to G1."""
        ...

    def tick(self, now: Optional[float] = None) -> None:
        """Watchdog tick."""
        ...


class StopLatchProtocol(Protocol):
    """Minimal live STOP interface used at the final dispatch boundary."""

    def snapshot(self) -> StopSnapshot:
        ...


def dispatch_strategy_or_stop(
    strategy: StrategyResult,
    latch_snapshot: StopSnapshot,
    bridge: G1BridgeProtocol,
    limiter: CommandSlewRateLimiter,
    now_s: Optional[float] = None,
    latch: Optional[StopLatchProtocol] = None,
) -> None:
    """Dispatch strategy result or stop command to G1 bridge.

    This function implements the final safety gate before motion commands
    are sent to the robot. It checks:

    1. StopLatch state (HARD or SOFT stops) — propagate the real root cause
    2. Strategy stop_reason (safety stops)
    3. Motion sequence vs start_after_sequence
    4. Applies slew rate limiting

    Mandatory stop bypasses the limiter and dispatches zero immediately.

    Parameters
    ----------
    strategy : StrategyResult
        Strategy output from APF computation.
    latch_snapshot : StopSnapshot
        Current stop latch state (reason is the real root cause, not generic).
    bridge : G1BridgeProtocol
        G1 motion bridge.
    limiter : CommandSlewRateLimiter
        Command slew rate limiter.
    now_s : float or None
        Current monotonic time, or None to use time.monotonic().
    """
    if now_s is None:
        now_s = time.monotonic()

    # Check 1: StopLatch requested
    # The StopLatch.reason preserves the real root cause (e.g. "controller_disconnected",
    # "wristband_off", "cloud_stale"), NOT a generic "latch_stop".
    if latch_snapshot.requested:
        latch_reason = latch_snapshot.reason or "stop"
        _dispatch_mandatory_stop(bridge, limiter, reason=latch_reason)
        return

    # Check 2: Strategy has stop_reason
    if strategy.stop_reason is not None:
        _dispatch_mandatory_stop(bridge, limiter, reason=strategy.stop_reason.value)
        return

    # Check 3: Sequence must be > start_after_sequence
    if strategy.sequence <= latch_snapshot.start_after_sequence:
        _dispatch_mandatory_stop(bridge, limiter, reason="motion_not_allowed")
        return

    # Check 4: Zero motion intent (safety stop or no motion)
    apf_requested = strategy.apf_requested
    if _is_zero_intent(apf_requested):
        _dispatch_mandatory_stop(bridge, limiter, reason=apf_requested.reason)
        return

    # Apply slew rate limiting
    limited_intent = limiter.limit(strategy.apf_requested, now_s=now_s)

    # Final check: limited intent is zero
    if _is_zero_intent(limited_intent):
        _dispatch_zero_motion(bridge, limiter, now_s=now_s, reason=limited_intent.reason)
        return

    # Re-read the latch immediately before a nonzero bridge call.  A Web or
    # lifecycle thread may have requested STOP while APF or limiting ran.
    if latch is not None:
        live_snapshot = latch.snapshot()
        if live_snapshot.requested:
            _dispatch_mandatory_stop(
                bridge,
                limiter,
                reason=live_snapshot.reason or "stop",
            )
            return

    # Dispatch limited motion
    command = MotionCommand(
        kind="move",
        vx=limited_intent.vx,
        vy=limited_intent.vy,
        omega=limited_intent.omega,
        reason=limited_intent.reason,
    )
    bridge.dispatch(command)
    bridge.tick(now_s)


def _dispatch_mandatory_stop(
    bridge: G1BridgeProtocol,
    limiter: CommandSlewRateLimiter,
    reason: str = "stop",
) -> None:
    """Dispatch mandatory stop, bypassing limiter.

    Mandatory stop resets the limiter and dispatches zero immediately.
    This is used for all stop cases: latch stops, safety stops, etc.

    Parameters
    ----------
    bridge : G1BridgeProtocol
        G1 motion bridge.
    limiter : CommandSlewRateLimiter
        Command slew rate limiter (will be reset).
    reason : str
        Root cause reason string from StopReason enum.
    """
    limiter.reset()
    command = MotionCommand(
        kind="stop",
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason=reason,
    )
    bridge.dispatch(command)


def _dispatch_zero_motion(
    bridge: G1BridgeProtocol,
    limiter: CommandSlewRateLimiter,
    now_s: float,
    reason: str = "stop",
) -> None:
    """Dispatch zero motion without resetting limiter state.

    This is used when the strategy naturally produces zero motion,
    not a mandatory stop. The limiter is not reset so it can smoothly
    ramp up again.

    Parameters
    ----------
    bridge : G1BridgeProtocol
        G1 motion bridge.
    limiter : CommandSlewRateLimiter
        Command slew rate limiter (not reset).
    now_s : float
        Current monotonic time.
    reason : str
        Root cause reason string.
    """
    command = MotionCommand(
        kind="move",
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason=reason,
    )
    bridge.dispatch(command)
    bridge.tick(now_s)


def _is_zero_intent(intent: MotionIntent) -> bool:
    """Check if motion intent is zero."""
    return (
        abs(intent.vx) < 1e-6
        and abs(intent.vy) < 1e-6
        and abs(intent.omega) < 1e-6
    )
