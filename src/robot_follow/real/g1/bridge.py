"""G1 motion bridge - converts MotionCommand to Unitree LocoClient calls.

Hot-path SDK contract (verified against ``unitree_sdk2_python`` source at
``unitree_sdk2py/g1/loco/g1_loco_client.py``):

The high-level ``LocoClient.Move`` / ``StopMove`` wrappers call ``SetVelocity``
internally but DISCARD its return code and return ``None``. Only
``SetVelocity(vx, vy, omega, duration=1.0)`` returns a ``code``
(0 = success, non-zero = failure; e.g. RPC timeout, send error, or "robot not
in motion state"). ``Init`` likewise returns ``None``.

To observe SDK failures on the hot path, this bridge calls ``SetVelocity``
directly instead of the ``Move`` / ``StopMove`` wrappers:

    move -> SetVelocity(vx, 0, omega, duration=1.0)   # == Move(vx, 0, omega) non-continuous
    stop -> SetVelocity(0.0, 0.0, 0.0, duration=1.0)  # == StopMove()

These are byte-equivalent to the wrappers (both use ``duration=1`` in the
non-continuous case) but expose the real return code, so a non-zero result
(e.g. G1 not yet in motion state after the operator skipped the manual boot
sequence) is detected and reported as ``sdk_error`` instead of being silently
swallowed. ``Init`` returns ``None`` on the real SDK, so init failure is
detected via exceptions (``ChannelFactoryInitialize`` / ``LocoClient()`` /
``SetTimeout`` / ``Init``), not via a return code.

See ``docs/superpowers/plans/2026-06-12-g1-command-bridge.md`` Task 0 Step 5
for the verified API contract.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Callable, Union

from robot_follow.core.types import MotionCommand
from robot_follow.real.g1.types import G1BridgeConfig, BridgeState


logger = logging.getLogger(__name__)


# SDK per-call velocity duration (seconds). Matches the non-continuous
# Move(...) wrapper and StopMove(), both of which call SetVelocity with
# duration=1. A crashed process therefore holds the last velocity for <=1s
# before the on-board FSM intervenes; the software watchdog (tick +
# command_timeout_s) handles the normal refresh case.
_MOVE_DURATION_S = 1.0


class G1MotionBridge:
    """Converts MotionCommand to Unitree G1 LocoClient calls.

    This bridge is a thin but conservative hardware output adapter. It:
    - Performs final defensive validation (NaN/inf, velocity limits)
    - Manages internal state machine (CREATED, INITIALIZED, ACTIVE, STOPPED, ERROR)
    - Implements watchdog timeout for automatic stop
    - Uses delayed SDK import (init() time, not module import time)

    Hot-path safety rule: ``move`` / ``stop`` / ``tick`` MUST NOT raise. A
    runtime or SDK error is recorded as ``sdk_error`` + best-effort stop +
    ERROR state, so the control loop never dies on a recoverable error; the
    next frame sees ``sdk_error`` and the pipeline outputs a stop. Only
    ``init()`` raises (pre-loop, fatal — the run cannot start).

    Usage
    -----
    >>> bridge = G1MotionBridge("enp2s0")
    >>> bridge.init()  # Initializes DDS and LocoClient
    >>> bridge.dispatch(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="track"))
    >>> bridge.tick()  # Watchdog tick (uses the injected clock)
    >>> bridge.stop("safety")
    >>> bridge.close()

    For testing without real SDK, inject a fake_client and a controllable clock:

    >>> fake = FakeLocoClient()
    >>> bridge = G1MotionBridge("eth0", fake_client=fake)
    >>> bridge.init()
    """

    def __init__(
        self,
        network_interface: str,
        config: G1BridgeConfig | None = None,
        fake_client: object | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._network_interface = network_interface
        self._config = config or G1BridgeConfig()
        self._fake_client = fake_client
        self._clock = clock  # injectable clock (default monotonic, matches the loop)
        self._client: object | None = None
        self._state = BridgeState.CREATED
        self._sdk_error = False
        self._last_dispatch_time: float | None = None

    @property
    def state(self) -> BridgeState:
        """Current bridge state."""
        return self._state

    @property
    def sdk_error(self) -> bool:
        """Whether an SDK error has occurred."""
        return self._sdk_error

    @property
    def config(self) -> G1BridgeConfig:
        """Bridge configuration."""
        return self._config

    def init(self) -> None:
        """Initialize DDS and LocoClient.

        Uses a delayed import so the module can be imported without the SDK
        installed (the import lives in the real-hardware branch only).

        Failure here is fatal and raises ``RuntimeError`` (this runs before the
        control loop starts). On the real SDK, ``LocoClient.Init`` returns
        ``None`` (it only registers APIs locally and does not talk to the
        robot), so init failure is detected via exceptions from
        ``ChannelFactoryInitialize`` / ``LocoClient()`` / ``SetTimeout`` /
        ``Init`` rather than via a return code.
        """
        try:
            if self._fake_client is not None:
                self._client = self._fake_client
                logger.info("G1MotionBridge initialized with fake client (test mode)")
            else:
                # Delayed import - only when actually initializing on real hardware.
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize
                from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

                # NOTE: ChannelFactoryInitialize(0, iface) initializes a
                # PROCESS-GLOBAL DDS factory. It is NOT idempotent and NOT
                # re-callable: calling init() again (e.g. after an ERROR) will
                # not cleanly re-create the factory. Recovery from a real init
                # failure requires a process restart, not a second init() call.
                ChannelFactoryInitialize(0, self._network_interface)
                self._client = LocoClient()
                logger.info(f"G1MotionBridge initialized on {self._network_interface}")

            # Common to fake and real paths: SetTimeout + Init. On the real SDK
            # Init() returns None (local API registration only), so a failure
            # here surfaces as a Python exception, caught below.
            self._client.SetTimeout(self._config.sdk_timeout_s)
            self._client.Init()
            self._state = BridgeState.INITIALIZED
        except Exception as e:
            self._state = BridgeState.ERROR
            self._sdk_error = True
            logger.error(f"G1 SDK init failed: {e}")
            raise RuntimeError(f"G1 SDK init failed: {e}") from e

    def dispatch(self, command: MotionCommand) -> None:
        """Dispatch a MotionCommand to the G1.

        Routes to move() or stop() based on command.kind.
        """
        if command.kind == "stop":
            self.stop(command)
            return
        self.move(command)

    def move(self, command: MotionCommand) -> None:
        """Send a move command with defensive validation.

        Steps:
        1. Refuse if not in a valid move state (record error, do not raise)
        2. Validate command (kind, finite values)
        3. Clamp velocities to config limits
        4. Apply yaw_sign to omega
        5. Call SetVelocity(vx, 0, omega, duration=1.0) — byte-equivalent to
           Move(vx, 0, omega) but returns the real code so failures are detected.
        """
        if self._state in (BridgeState.CREATED, BridgeState.ERROR):
            # Refuse move; never crash the loop. Flag sdk_error so the next
            # frame's pipeline outputs a stop.
            self._record_error(f"move rejected (state={self._state.value})")
            return
        if not self._validate_move_command(command):
            return  # validation failure already recorded + best-effort stop

        vx = _clamp(command.vx, -self._config.max_abs_vx, self._config.max_abs_vx)
        vy = 0.0  # lateral disabled in first release
        omega = _clamp(
            self._config.yaw_sign * command.omega,
            -self._config.max_abs_omega,
            self._config.max_abs_omega,
        )

        try:
            code = self._client.SetVelocity(vx, vy, omega, duration=_MOVE_DURATION_S)
            if _is_error_code(code):
                self._record_error(f"G1 SetVelocity returned error code {code}")
                return
            self._state = BridgeState.ACTIVE
            self._last_dispatch_time = self._clock()
            logger.debug(f"G1 move: vx={vx:.3f} omega={omega:.3f}")
        except Exception as e:
            self._record_error(f"G1 SetVelocity failed: {e}")

    def stop(self, reason: Union[str, MotionCommand] = "stop") -> None:
        """Send a stop command (zero velocity).

        Calls ``SetVelocity(0.0, 0.0, 0.0, duration=1.0)``, byte-equivalent to
        ``StopMove()`` but exposing the real return code. Never raises; a
        non-zero return code is recorded as ``sdk_error`` + ERROR state so the
        control loop keeps running. A failed stop is NOT retried here — the
        retry would be byte-identical to the call that just failed.
        """
        if self._client is None:
            logger.warning("G1 stop called before init, ignoring")
            return
        try:
            code = self._client.SetVelocity(0.0, 0.0, 0.0, duration=_MOVE_DURATION_S)
            if _is_error_code(code):
                self._sdk_error = True
                self._state = BridgeState.ERROR
                logger.error(f"G1 stop returned error code {code}")
                return
            self._state = BridgeState.STOPPED
            self._last_dispatch_time = self._clock()
            logger.info(f"G1 stop: reason={reason if isinstance(reason, str) else reason.reason}")
        except Exception as e:
            self._sdk_error = True
            self._state = BridgeState.ERROR
            logger.error(f"G1 stop failed: {e}")

    def tick(self, now: float | None = None) -> None:
        """Watchdog tick - send stop if command timeout expired.

        Call this regularly (e.g., every main loop iteration) to ensure an
        automatic stop if new commands stop arriving. The ``now`` parameter
        allows the main loop to pass its own clock value (typically
        ``time.monotonic()``) so that the bridge's timestamps are consistent
        with the loop's time domain.
        """
        if now is None:
            now = self._clock()
        if self._last_dispatch_time is None:
            return
        if self._state == BridgeState.STOPPED:
            return
        if self._state == BridgeState.ERROR:
            return
        elapsed = now - self._last_dispatch_time
        if elapsed > self._config.command_timeout_s:
            logger.warning(
                f"G1 watchdog timeout ({elapsed:.2f}s > {self._config.command_timeout_s}s)"
            )
            self.stop("watchdog_timeout")

    def close(self) -> None:
        """Clean shutdown. Best-effort: stop failure does not prevent cleanup."""
        try:
            self.stop("close")
        except Exception as e:
            logger.warning(f"G1 close: best-effort stop failed: {e}")
        self._client = None
        self._state = BridgeState.CREATED
        logger.info("G1MotionBridge closed")

    def _record_error(self, message: str) -> None:
        """Record an SDK or validation error without raising.

        Sets ``sdk_error`` + ERROR state and attempts a best-effort stop. The
        next pipeline frame sees ``sdk_error=True`` via ``SafetyManager`` and
        emits a stop, so the loop converges safely without an exception.
        """
        self._sdk_error = True
        self._state = BridgeState.ERROR
        self._best_effort_stop()
        logger.error(message)

    def _validate_move_command(self, command: MotionCommand) -> bool:
        """Validate move command before sending to SDK.

        Returns True if valid; False (with error recorded) if invalid. Never raises.

        Checks:
        - kind must be "move"
        - vx and omega must be finite (not NaN/inf)
        """
        if command.kind != "move":
            self._record_error(f"expected move command, got {command.kind}")
            return False
        if not _is_finite(command.vx):
            self._record_error(f"vx is not finite: {command.vx}")
            return False
        if not _is_finite(command.omega):
            self._record_error(f"omega is not finite: {command.omega}")
            return False
        return True

    def _best_effort_stop(self) -> None:
        """Attempt a zero-velocity SetVelocity without raising on failure.

        Used from ``_record_error`` to physically halt the robot when an error
        is detected mid-move. Does not touch ``sdk_error`` / state (the caller
        already set them).
        """
        if self._client is None:
            return
        try:
            code = self._client.SetVelocity(0.0, 0.0, 0.0, duration=_MOVE_DURATION_S)
            if _is_error_code(code):
                logger.error(f"best-effort stop returned error code {code}")
        except Exception as e:
            logger.error(f"best-effort stop failed: {e}")


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi] range."""
    return max(lo, min(hi, value))


def _is_finite(value: float) -> bool:
    """Check if value is finite (not NaN or inf)."""
    return math.isfinite(value)


def _is_error_code(code: object) -> bool:
    """Return True if an SDK return value represents failure.

    ``SetVelocity`` returns ``0`` on success and a non-zero code on failure.
    Other SDK methods (``Move``/``StopMove``/``Init``) return ``None``. Treat
    ``None`` as success (not an error code) and non-zero integers as errors.
    The bridge only feeds ``SetVelocity`` results into this function on the hot
    path; ``Init`` failure is handled via exceptions in ``init()``.
    """
    return code is not None and code != 0
