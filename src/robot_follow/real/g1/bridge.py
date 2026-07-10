"""G1 motion bridge - converts MotionCommand to Unitree LocoClient calls.

Hot-path SDK contract (verified against ``unitree_sdk2_python`` source at
``unitree_sdk2py/g1/loco/g1_loco_client.py``):

The high-level ``LocoClient.Move`` / ``StopMove`` wrappers call ``SetVelocity``
internally but DISCARD its return code and return ``None``. Only
``SetVelocity(vx, vy, omega, duration=1.0)`` returns a ``code``
(0 = success, non-zero = failure). Unitree documents return code ``3104`` as
a request timeout; other non-zero codes remain hard SDK errors. ``Init``
likewise returns ``None``.

To observe SDK failures on the hot path, this bridge calls ``SetVelocity``
directly instead of the ``Move`` / ``StopMove`` wrappers:

    move -> SetVelocity(vx, 0, omega, duration=1.0)   # == Move(vx, 0, omega) non-continuous
    stop -> SetVelocity(0.0, 0.0, 0.0, duration=1.0)  # == StopMove()

These are byte-equivalent to the wrappers (both use ``duration=1`` in the
non-continuous case) but expose the real return code, so a non-zero result
is detected instead of being silently swallowed. Request timeouts enter a
degraded ``sdk_timeout`` state with throttled zero-velocity retries; repeated
timeouts or non-timeout failures are reported as ``sdk_error``. ``Init`` returns
``None`` on the real SDK, so init failure is detected via exceptions
(``ChannelFactoryInitialize`` / ``LocoClient()`` / ``SetTimeout`` / ``Init``),
not via a return code.

See ``docs/superpowers/plans/2026-06-12-g1-command-bridge.md`` Task 0 Step 5
for the verified API contract.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
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
_LOW_STATE_TOPIC = "rt/lowstate"


@dataclass(frozen=True)
class G1AttitudeState:
    """Latest pelvis IMU yaw diagnostics received from G1 LowState."""

    yaw_deg: float
    yaw_rate_rps: float


class G1MotionBridge:
    """Converts MotionCommand to Unitree G1 LocoClient calls.

    This bridge is a thin but conservative hardware output adapter. It:
    - Performs final defensive validation (NaN/inf, velocity limits)
    - Manages internal state machine (CREATED, INITIALIZED, ACTIVE, STOPPED,
      SDK_TIMEOUT, ERROR)
    - Implements watchdog timeout for automatic stop
    - Uses delayed SDK import (init() time, not module import time)

    Hot-path safety rule: ``move`` / ``stop`` / ``tick`` MUST NOT raise. A
    SDK request timeouts are recorded as ``sdk_timeout`` and retried with
    backoff. Other runtime or SDK errors are recorded as ``sdk_error`` +
    best-effort stop + ERROR state, so the control loop never dies on a
    recoverable error. Only ``init()`` raises (pre-loop, fatal — the run cannot
    start).

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
        fake_state_subscriber: object | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._network_interface = network_interface
        self._config = config or G1BridgeConfig()
        self._fake_client = fake_client
        self._fake_state_subscriber = fake_state_subscriber
        self._clock = clock  # injectable clock (default monotonic, matches the loop)
        self._client: object | None = None
        self._state_subscriber: object | None = None
        self._attitude_state: G1AttitudeState | None = None
        self._state = BridgeState.CREATED
        self._sdk_error = False
        self._consecutive_sdk_timeouts = 0
        self._last_stop_attempt_time: float | None = None
        self._last_dispatch_time: float | None = None
        self._last_sent_omega: float | None = None

    @property
    def state(self) -> BridgeState:
        """Current bridge state."""
        return self._state

    @property
    def sdk_error(self) -> bool:
        """Whether an SDK error has occurred."""
        return self._sdk_error

    @property
    def sdk_timeout(self) -> bool:
        """Whether the bridge is degraded by SDK request timeouts."""
        return self._state == BridgeState.SDK_TIMEOUT

    @property
    def config(self) -> G1BridgeConfig:
        """Bridge configuration."""
        return self._config

    @property
    def last_sent_omega(self) -> float | None:
        """Most recent omega attempted at ``SetVelocity`` after final limits.

        This is intended for runtime diagnostics. ``None`` means the most
        recent dispatch did not reach a ``SetVelocity`` call.
        """
        return self._last_sent_omega

    @property
    def attitude_state(self) -> G1AttitudeState | None:
        """Latest actual yaw and yaw rate reported by the G1 pelvis IMU."""
        return self._attitude_state

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

            self._init_state_subscriber()

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
        self._last_sent_omega = None
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
        self._last_sent_omega = None
        if self._state in (BridgeState.CREATED, BridgeState.ERROR):
            # Refuse move; never crash the loop. Flag sdk_error so the next
            # frame's pipeline outputs a stop.
            self._record_error(f"move rejected (state={self._state.value})")
            return
        if self._state == BridgeState.SDK_TIMEOUT:
            logger.warning("move suppressed while recovering from SDK timeout")
            self.stop("sdk_timeout")
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
            self._last_sent_omega = omega
            code = self._client.SetVelocity(vx, vy, omega, duration=_MOVE_DURATION_S)
            if _is_timeout_code(code, self._config):
                self._record_timeout(f"G1 SetVelocity timed out with code {code}")
                self.stop("sdk_timeout")
                return
            if _is_error_code(code):
                self._record_error(f"G1 SetVelocity returned error code {code}")
                return
            self._clear_timeout_state()
            self._state = BridgeState.ACTIVE
            self._last_dispatch_time = self._clock()
            logger.debug(f"G1 move: vx={vx:.3f} omega={omega:.3f}")
        except Exception as e:
            self._record_error(f"G1 SetVelocity failed: {e}")

    def stop(self, reason: Union[str, MotionCommand] = "stop") -> None:
        """Send a stop command (zero velocity).

        Calls ``SetVelocity(0.0, 0.0, 0.0, duration=1.0)``, byte-equivalent to
        ``StopMove()`` but exposing the real return code. Never raises; a
        Unitree request timeout code 3104 is treated as a degraded timeout
        state and retried with backoff. Other non-zero return codes are
        recorded as ``sdk_error`` + ERROR state so the control loop keeps
        running.
        """
        self._last_sent_omega = None
        if self._client is None:
            logger.warning("G1 stop called before init, ignoring")
            return
        now = self._clock()
        if not self._should_attempt_stop(now):
            return
        self._last_stop_attempt_time = now
        try:
            self._last_sent_omega = 0.0
            code = self._client.SetVelocity(0.0, 0.0, 0.0, duration=_MOVE_DURATION_S)
            if _is_timeout_code(code, self._config):
                self._record_timeout(f"G1 stop timed out with code {code}")
                return
            if _is_error_code(code):
                self._sdk_error = True
                self._state = BridgeState.ERROR
                logger.error(f"G1 stop returned error code {code}")
                return
            self._clear_timeout_state()
            self._state = BridgeState.STOPPED
            self._last_dispatch_time = now
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
        self._state_subscriber = None
        self._attitude_state = None
        self._state = BridgeState.CREATED
        logger.info("G1MotionBridge closed")

    def _init_state_subscriber(self) -> None:
        """Start a best-effort G1 LowState subscription for diagnostics.

        Motion output remains available when the diagnostic topic or message
        type is unavailable; in that case the CSV yaw fields stay empty.
        """
        try:
            if self._fake_state_subscriber is not None:
                subscriber = self._fake_state_subscriber
            elif self._fake_client is not None:
                return
            else:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

                subscriber = ChannelSubscriber(_LOW_STATE_TOPIC, LowState_)
            subscriber.Init(self._on_low_state, 10)
            self._state_subscriber = subscriber
            logger.info("G1 LowState diagnostics subscribed on %s", _LOW_STATE_TOPIC)
        except Exception as exc:
            self._state_subscriber = None
            logger.warning("G1 LowState diagnostics unavailable: %s", exc)

    def _on_low_state(self, message: object) -> None:
        """Update actual yaw diagnostics from a Unitree ``LowState_`` sample."""
        try:
            imu_state = _read_member(message, "imu_state")
            rpy = _read_member(imu_state, "rpy")
            gyroscope = _read_member(imu_state, "gyroscope")
            yaw_rad = float(rpy[2])
            yaw_rate_rps = float(gyroscope[2])
            if not (_is_finite(yaw_rad) and _is_finite(yaw_rate_rps)):
                return
            self._attitude_state = G1AttitudeState(
                yaw_deg=math.degrees(yaw_rad),
                yaw_rate_rps=yaw_rate_rps,
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            logger.debug("Ignoring malformed G1 LowState IMU sample", exc_info=True)

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

    def _record_timeout(self, message: str) -> None:
        """Record a Unitree request timeout without immediately hard-failing."""
        if self._sdk_error or self._state == BridgeState.ERROR:
            self._state = BridgeState.ERROR
            logger.warning(message)
            return
        self._consecutive_sdk_timeouts += 1
        if self._consecutive_sdk_timeouts >= self._config.max_consecutive_sdk_timeouts:
            self._sdk_error = True
            self._state = BridgeState.ERROR
            logger.error(
                f"{message}; escalating after "
                f"{self._consecutive_sdk_timeouts} consecutive timeouts"
            )
            return
        self._state = BridgeState.SDK_TIMEOUT
        logger.warning(message)

    def _clear_timeout_state(self) -> None:
        """Clear degraded timeout accounting after a successful SDK request."""
        self._consecutive_sdk_timeouts = 0

    def _should_attempt_stop(self, now: float) -> bool:
        """Throttle stop retries while recovering from SDK request timeouts."""
        if self._state != BridgeState.SDK_TIMEOUT:
            return True
        if self._last_stop_attempt_time is None:
            return True
        elapsed = now - self._last_stop_attempt_time
        if elapsed >= self._config.sdk_timeout_retry_interval_s:
            return True
        logger.debug(
            "G1 stop retry throttled after SDK timeout "
            f"({elapsed:.2f}s < {self._config.sdk_timeout_retry_interval_s:.2f}s)"
        )
        return False

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
            self._last_sent_omega = 0.0
            code = self._client.SetVelocity(0.0, 0.0, 0.0, duration=_MOVE_DURATION_S)
            if _is_error_code(code):
                logger.error(f"best-effort stop returned error code {code}")
        except Exception as e:
            logger.error(f"best-effort stop failed: {e}")


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi] range."""
    return max(lo, min(hi, value))


def _read_member(value: object, name: str) -> object:
    """Read either an IDL Python attribute or a generated callable accessor."""
    member = getattr(value, name)
    return member() if callable(member) else member


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


def _is_timeout_code(code: object, config: G1BridgeConfig) -> bool:
    """Return True if an SDK return value represents a request timeout."""
    return code == config.sdk_timeout_code
