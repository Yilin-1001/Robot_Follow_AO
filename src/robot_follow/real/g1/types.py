"""G1 bridge configuration and state types."""

from dataclasses import dataclass
from enum import Enum


@dataclass
class G1BridgeConfig:
    """Tunable safety parameters for the G1 motion bridge.

    First-release defaults are chosen to be safe for an early hardware test
    (per design doc §5 "first on-site verification" range), not the absolute
    upper bounds. Raise these only after the verification chain
    (docs/alx_g1_verification_chain.md) passes.
    """

    max_abs_vx: float = 0.10  # m/s, conservative for first hardware test
    max_abs_vy: float = 0.0   # m/s, lateral disabled in first release
    max_abs_omega: float = 0.30  # rad/s, conservative for first hardware test
    yaw_sign: int = 1  # +1 or -1, calibrate on-site if rotation is reversed
    sdk_timeout_s: float = 10.0  # SDK call timeout
    sdk_timeout_code: int = 3104  # Unitree SDK request timeout return code
    sdk_timeout_retry_interval_s: float = 1.0  # backoff for zero-velocity retries
    max_consecutive_sdk_timeouts: int = 3  # escalate repeated timeouts to ERROR
    command_timeout_s: float = 0.5  # watchdog interval before auto-stop
    # NOTE: continuous_move is intentionally NOT exposed. The SDK's
    # continuous_move=True sets duration=864000s (~10 days), which means a
    # crashed main loop would let the robot keep moving until the on-board FSM
    # intervenes. The bridge always uses the SDK default (duration=1s) so a
    # crash bounds the worst case to <=1s of residual motion. The software
    # watchdog (tick + command_timeout_s) handles the normal case.

    def __post_init__(self):
        if self.yaw_sign not in (1, -1):
            raise ValueError(f"yaw_sign must be +1 or -1, got {self.yaw_sign}")
        if self.max_abs_vx < 0:
            raise ValueError(f"max_abs_vx must be non-negative, got {self.max_abs_vx}")
        if self.max_abs_vy < 0:
            raise ValueError(f"max_abs_vy must be non-negative, got {self.max_abs_vy}")
        if self.max_abs_omega < 0:
            raise ValueError(f"max_abs_omega must be non-negative, got {self.max_abs_omega}")
        if self.command_timeout_s <= 0:
            raise ValueError(f"command_timeout_s must be > 0, got {self.command_timeout_s}")
        if self.sdk_timeout_s <= 0:
            raise ValueError(f"sdk_timeout_s must be > 0, got {self.sdk_timeout_s}")
        if self.sdk_timeout_retry_interval_s <= 0:
            raise ValueError(
                "sdk_timeout_retry_interval_s must be > 0, "
                f"got {self.sdk_timeout_retry_interval_s}"
            )
        if self.max_consecutive_sdk_timeouts <= 0:
            raise ValueError(
                "max_consecutive_sdk_timeouts must be > 0, "
                f"got {self.max_consecutive_sdk_timeouts}"
            )


class BridgeState(Enum):
    """Internal state machine for the motion bridge.

    State transitions:
    - CREATED -> INITIALIZED: init() success
    - INITIALIZED/STOPPED -> ACTIVE: move() success
    - ACTIVE/INITIALIZED -> STOPPED: stop() or watchdog timeout
    - ANY -> SDK_TIMEOUT: SDK request timeout return code
    - ANY -> ERROR: SDK exception, validation failure, init failure,
      or repeated SDK request timeouts
    - ERROR -> STOPPED: stop() clears motion output
    """

    CREATED = "created"      # SDK not yet initialized
    INITIALIZED = "initialized"  # DDS and LocoClient ready
    ACTIVE = "active"        # Last command was a successful move
    STOPPED = "stopped"      # Last command was StopMove
    SDK_TIMEOUT = "sdk_timeout"  # SDK request timed out; retry stop with backoff
    ERROR = "error"          # SDK error or validation failure
