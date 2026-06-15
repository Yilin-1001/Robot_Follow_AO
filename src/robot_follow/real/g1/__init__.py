"""G1 Unitree robot motion bridge.

This package provides the hardware output adapter for Unitree G1.
It converts MotionCommand (from core.types) to LocoClient SDK calls.

First-release scope:
- High-level velocity control via LocoClient.Move/StopMove
- Defensive validation (NaN/inf, limits, state checks)
- Watchdog timeout for automatic stop
- Delayed SDK import for dry-run compatibility

Explicitly NOT in scope:
- Low-level joint motor control
- Path planning or obstacle avoidance
- Vision fusion
"""

from robot_follow.real.g1.types import G1BridgeConfig, BridgeState

__all__ = [
    "G1BridgeConfig",
    "BridgeState",
]
