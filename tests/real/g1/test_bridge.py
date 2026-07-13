"""Unit tests for G1MotionBridge with fake client injection.

The ``FakeLocoClient`` here mirrors the REAL ``LocoClient`` return contract
(verified against ``unitree_sdk2py/g1/loco/g1_loco_client.py``):

- ``SetTimeout`` / ``Init`` / ``Move`` / ``StopMove`` return ``None``.
- ``SetVelocity(vx, vy, omega, duration=1.0)`` is the only method that returns
  a ``code`` (0 = success, non-zero = failure).
- ``Move`` / ``StopMove`` are thin wrappers that call ``SetVelocity`` and
  discard the code.

Because the bridge calls ``SetVelocity`` directly on the hot path (Option A),
all move/stop assertions go through ``set_velocity_calls``. The wrappers are
kept on the fake only so it stays a faithful stand-in for the real SDK; a
dedicated test locks the contract that the bridge never invokes them.
"""

import subprocess
import sys
import textwrap

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.real.g1.bridge import G1MotionBridge
from robot_follow.real.g1.types import G1BridgeConfig, BridgeState

# On the real SDK, a stop == SetVelocity(0.0, 0.0, 0.0, duration=1.0).
_ZERO_VEL_CALL = (0.0, 0.0, 0.0, 1.0)


def _move(vx: float = 0.1, vy: float = 0.0, omega: float = 0.0, reason: str = "test"):
    return MotionCommand(kind="move", vx=vx, vy=vy, omega=omega, reason=reason)


def _stop_call_count(fake: "FakeLocoClient") -> int:
    """Number of zero-velocity SetVelocity calls recorded on the fake."""
    return sum(1 for c in fake.set_velocity_calls if c == _ZERO_VEL_CALL)


class FakeLocoClient:
    """Faithful fake of ``unitree_sdk2py.g1.loco.g1_loco_client.LocoClient``.

    Failure injection mirrors the real SDK's failure modes:
    - ``fail_next``: the next ``SetVelocity`` returns code 1 (one-shot).
    - ``persistent_fail``: every ``SetVelocity`` returns code 1.
    - ``timeout_next``: the next ``SetVelocity`` returns code 3104 (one-shot).
    - ``persistent_timeout``: every ``SetVelocity`` returns code 3104.
    - ``init_fails``: ``Init`` raises (the real init failure mode, since real
      ``Init`` returns ``None`` rather than a code).
    """

    def __init__(
        self,
        fail_next: bool = False,
        persistent_fail: bool = False,
        timeout_next: bool = False,
        persistent_timeout: bool = False,
        init_fails: bool = False,
    ):
        self.set_velocity_calls = []  # list of (vx, vy, omega, duration)
        self.init_called = False
        self.timeout = None
        self.fail_next = fail_next
        self.persistent_fail = persistent_fail
        self.timeout_next = timeout_next
        self.persistent_timeout = persistent_timeout
        self.init_fails = init_fails
        # Counters for the wrapper methods the bridge must NOT call.
        self.move_calls = 0
        self.stopmove_calls = 0

    def _code(self):
        if self.persistent_timeout:
            return 3104
        if self.persistent_fail:
            return 1
        if self.timeout_next:
            self.timeout_next = False
            return 3104
        if self.fail_next:
            self.fail_next = False
            return 1
        return 0

    def SetTimeout(self, timeout: float):
        self.timeout = timeout
        return None  # mirrors real SDK (ClientBase.SetTimeout)

    def Init(self):
        self.init_called = True
        if self.init_fails:
            raise RuntimeError("fake Init failure")
        return None  # mirrors real SDK

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        self.set_velocity_calls.append((vx, vy, omega, duration))
        return self._code()  # the only method that returns a code on the real SDK

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        # Mirrors real SDK: duration = 864000 if continous else 1, calls
        # SetVelocity, returns None. The bridge must NOT use this wrapper.
        self.move_calls += 1
        duration = 864000.0 if continous_move else 1
        self.SetVelocity(vx, vy, vyaw, duration)
        return None

    def StopMove(self):
        # Mirrors real SDK: SetVelocity(0, 0, 0) default duration, returns None.
        # The bridge must NOT use this wrapper.
        self.stopmove_calls += 1
        self.SetVelocity(0.0, 0.0, 0.0, 1.0)
        return None


# ---------------------------------------------------------------------------
# init()
# ---------------------------------------------------------------------------


def test_fake_client_init():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    assert bridge.state == BridgeState.INITIALIZED
    assert fake.init_called
    assert fake.timeout == pytest.approx(10.0)


def test_bridge_does_not_expose_low_state_subscription():
    """Motion output must not create or expose a G1 LowState telemetry path."""
    bridge = G1MotionBridge("eth0", fake_client=FakeLocoClient())

    assert not hasattr(bridge, "attitude_state")


def test_init_failure_raises_fatal_error():
    """Init failure on the real SDK surfaces as an exception (Init returns None,
    not a code). The bridge must treat it as fatal and stop startup."""
    fake = FakeLocoClient(init_fails=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    with pytest.raises(RuntimeError, match="G1 SDK init failed"):
        bridge.init()
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


# ---------------------------------------------------------------------------
# move()
# ---------------------------------------------------------------------------


def test_move_command_calls_set_velocity():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move(vx=0.1, omega=0.2))
    assert fake.set_velocity_calls == [(0.1, 0.0, 0.2, 1.0)]
    assert bridge.state == BridgeState.ACTIVE


def test_holding_move_sends_zero_velocity():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move(vx=0.0, omega=0.0, reason="holding"))
    assert fake.set_velocity_calls == [_ZERO_VEL_CALL]
    assert bridge.state == BridgeState.ACTIVE


def test_nan_velocity_triggers_stop_and_error():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move(vx=float("nan")))
    assert _stop_call_count(fake) >= 1  # best-effort stop attempted
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


def test_inf_velocity_triggers_stop_and_error():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move(vx=float("inf")))
    assert _stop_call_count(fake) >= 1
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


def test_over_limit_velocity_is_clamped():
    fake = FakeLocoClient()
    cfg = G1BridgeConfig(max_abs_vx=0.10, max_abs_omega=0.30)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()
    bridge.move(_move(vx=0.5, omega=1.0, reason="fast"))
    assert fake.set_velocity_calls == [(0.10, 0.0, 0.30, 1.0)]


def test_yaw_sign_flips_omega():
    fake = FakeLocoClient()
    cfg = G1BridgeConfig(yaw_sign=-1)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()
    # omega=0.2 is within default max_abs_omega=0.30, so no clamping
    bridge.move(_move(vx=0.1, omega=0.2, reason="turn"))
    assert fake.set_velocity_calls == [(0.1, 0.0, -0.2, 1.0)]


def test_last_sent_omega_records_yaw_signed_and_limited_output():
    fake = FakeLocoClient()
    cfg = G1BridgeConfig(yaw_sign=-1, max_abs_omega=0.30)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()

    bridge.dispatch(_move(vx=0.1, omega=1.0, reason="turn"))

    assert bridge.last_sent_omega == pytest.approx(-0.30)
    bridge.dispatch(
        MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason="test_stop")
    )
    assert bridge.last_sent_omega == pytest.approx(0.0)


def test_move_before_init_records_error_without_raising():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.move(_move())  # no init()
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    assert fake.set_velocity_calls == []  # nothing sent before init


def test_move_set_velocity_return_code_nonzero_triggers_error():
    """A non-zero SetVelocity code (e.g. G1 not in motion state) must be
    detected on the hot path — this is the real failure mode the Move wrapper
    would silently swallow."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    fake.fail_next = True
    bridge.move(_move())
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    # move's SetVelocity failed, then a best-effort stop was attempted.
    assert fake.set_velocity_calls[-1] == _ZERO_VEL_CALL


def test_move_timeout_attempts_stop_without_hard_error_when_stop_succeeds():
    """A single Unitree 3104 request timeout stops motion but can recover if
    the following zero-velocity request succeeds."""
    fake = FakeLocoClient(timeout_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()

    bridge.move(_move())

    assert not bridge.sdk_error
    assert not bridge.sdk_timeout
    assert bridge.state == BridgeState.STOPPED
    assert fake.set_velocity_calls == [(0.1, 0.0, 0.0, 1.0), _ZERO_VEL_CALL]


def test_move_timeout_stays_degraded_when_stop_also_times_out():
    fake = FakeLocoClient(persistent_timeout=True)
    cfg = G1BridgeConfig(max_consecutive_sdk_timeouts=3)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()

    bridge.move(_move())

    assert not bridge.sdk_error
    assert bridge.sdk_timeout
    assert bridge.state == BridgeState.SDK_TIMEOUT
    assert fake.set_velocity_calls == [(0.1, 0.0, 0.0, 1.0), _ZERO_VEL_CALL]


def test_move_in_error_state_is_rejected_without_raising():
    """Once in ERROR, move() refuses to send its velocity but never raises."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    fake.fail_next = True
    bridge.move(_move(reason="first"))
    assert bridge.state == BridgeState.ERROR
    nonzero_before = sum(1 for c in fake.set_velocity_calls if c != _ZERO_VEL_CALL)
    bridge.move(_move(reason="second"))
    nonzero_after = sum(1 for c in fake.set_velocity_calls if c != _ZERO_VEL_CALL)
    # The second move's velocity was NOT sent (only a best-effort zero may add).
    assert nonzero_after == nonzero_before
    assert bridge.state == BridgeState.ERROR


# ---------------------------------------------------------------------------
# stop()
# ---------------------------------------------------------------------------


def test_stop_command_sends_zero_velocity():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move())
    bridge.stop("safety_stop")
    assert _stop_call_count(fake) >= 1
    assert fake.set_velocity_calls[-1] == _ZERO_VEL_CALL
    assert bridge.state == BridgeState.STOPPED


def test_stop_set_velocity_return_code_nonzero_triggers_error():
    """A non-zero code from stop's SetVelocity is recorded without raising.
    Under Option A, stop() is a single SetVelocity(0,0,0,1.0); a failed stop is
    NOT retried (the retry would be byte-identical), so exactly one call."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    fake.fail_next = True
    calls_before = len(fake.set_velocity_calls)
    bridge.stop("test")
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    assert len(fake.set_velocity_calls) == calls_before + 1
    assert fake.set_velocity_calls[-1] == _ZERO_VEL_CALL


def test_stop_timeout_enters_degraded_state_without_hard_error():
    fake = FakeLocoClient(timeout_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()

    bridge.stop("test")

    assert not bridge.sdk_error
    assert bridge.sdk_timeout
    assert bridge.state == BridgeState.SDK_TIMEOUT
    assert fake.set_velocity_calls == [_ZERO_VEL_CALL]


def test_stop_timeout_retries_are_throttled():
    fake = FakeLocoClient(persistent_timeout=True)
    t = [0.0]
    cfg = G1BridgeConfig(sdk_timeout_retry_interval_s=1.0)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake, clock=lambda: t[0])
    bridge.init()

    bridge.stop("first")
    bridge.stop("too_soon")
    t[0] = 0.5
    bridge.stop("still_too_soon")

    assert len(fake.set_velocity_calls) == 1

    t[0] = 1.0
    bridge.stop("retry")
    assert len(fake.set_velocity_calls) == 2


def test_consecutive_timeouts_escalate_to_sdk_error():
    fake = FakeLocoClient(persistent_timeout=True)
    t = [0.0]
    cfg = G1BridgeConfig(
        sdk_timeout_retry_interval_s=1.0,
        max_consecutive_sdk_timeouts=3,
    )
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake, clock=lambda: t[0])
    bridge.init()

    bridge.stop("first")
    t[0] = 1.0
    bridge.stop("second")
    t[0] = 2.0
    bridge.stop("third")

    assert bridge.sdk_error
    assert not bridge.sdk_timeout
    assert bridge.state == BridgeState.ERROR


def test_successful_stop_clears_timeout_state():
    fake = FakeLocoClient(timeout_next=True)
    t = [0.0]
    bridge = G1MotionBridge("eth0", fake_client=fake, clock=lambda: t[0])
    bridge.init()

    bridge.stop("timeout")
    assert bridge.sdk_timeout

    t[0] = 1.0
    bridge.stop("recovered")
    assert not bridge.sdk_error
    assert not bridge.sdk_timeout
    assert bridge.state == BridgeState.STOPPED


def test_best_effort_stop_when_everything_fails():
    """When move's SetVelocity fails AND the best-effort stop also fails
    (persistent_fail), the bridge must still not raise and must end in ERROR."""
    fake = FakeLocoClient(persistent_fail=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move())
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    # move's SetVelocity + best-effort stop's SetVelocity both attempted
    assert len(fake.set_velocity_calls) == 2
    assert fake.set_velocity_calls[-1] == _ZERO_VEL_CALL


# ---------------------------------------------------------------------------
# dispatch()
# ---------------------------------------------------------------------------


def test_dispatch_routes_stop():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.dispatch(MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason="safety"))
    assert fake.set_velocity_calls == [_ZERO_VEL_CALL]
    assert bridge.state == BridgeState.STOPPED


def test_dispatch_routes_move():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.dispatch(_move(omega=0.1, reason="track"))
    assert fake.set_velocity_calls == [(0.1, 0.0, 0.1, 1.0)]
    assert bridge.state == BridgeState.ACTIVE


# ---------------------------------------------------------------------------
# tick() / watchdog
# ---------------------------------------------------------------------------


def test_watchdog_timeout_calls_stop():
    fake = FakeLocoClient()
    t = [0.0]
    bridge = G1MotionBridge(
        "eth0",
        config=G1BridgeConfig(command_timeout_s=0.3),
        fake_client=fake,
        clock=lambda: t[0],
    )
    bridge.init()
    bridge.move(_move())
    t[0] = 0.5  # advance injected clock past the timeout
    bridge.tick()
    assert _stop_call_count(fake) >= 1
    assert bridge.state == BridgeState.STOPPED


def test_clock_injection_is_used_by_move_and_tick():
    fake = FakeLocoClient()
    t = [100.0]
    bridge = G1MotionBridge(
        "eth0",
        config=G1BridgeConfig(command_timeout_s=1.0),
        fake_client=fake,
        clock=lambda: t[0],
    )
    bridge.init()
    bridge.move(_move())
    t[0] = 100.5  # under timeout
    bridge.tick()
    assert _stop_call_count(fake) == 0
    t[0] = 101.5  # over timeout
    bridge.tick()
    assert _stop_call_count(fake) >= 1


# ---------------------------------------------------------------------------
# close()
# ---------------------------------------------------------------------------


def test_close_is_best_effort_when_stop_fails():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    fake.fail_next = True
    bridge.close()  # stop() inside returns code 1; close must still complete
    assert bridge.state == BridgeState.CREATED
    bridge.close()  # second close: client is None -> stop returns early, no raise


def test_close_skips_blocking_stop_when_already_stopped():
    """When the bridge is already STOPPED the robot is already halted; close()
    must not issue another SetVelocity that could only block or fail."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move())
    bridge.stop("safety")  # state -> STOPPED
    calls_before = len(fake.set_velocity_calls)
    bridge.close()
    assert len(fake.set_velocity_calls) == calls_before  # no new dispatch
    assert bridge.state == BridgeState.CREATED


def test_close_skips_blocking_stop_in_sdk_timeout():
    """A closing stop() in SDK_TIMEOUT would block up to sdk_timeout_s on an
    unreachable G1 and return 3104 - skip it. The degraded retries already
    command zero velocity, so there is nothing new to communicate."""
    fake = FakeLocoClient(persistent_timeout=True)
    cfg = G1BridgeConfig(max_consecutive_sdk_timeouts=3)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()
    bridge.stop("first")  # 3104 -> state -> SDK_TIMEOUT
    assert bridge.state == BridgeState.SDK_TIMEOUT
    calls_before = len(fake.set_velocity_calls)
    bridge.close()
    assert len(fake.set_velocity_calls) == calls_before  # no blocking retry
    assert bridge.state == BridgeState.CREATED


def test_close_skips_blocking_stop_in_error():
    """Once escalated to ERROR the SDK is known-broken; a closing stop can only
    block and fail. Skip it so shutdown is prompt and quiet."""
    fake = FakeLocoClient(persistent_timeout=True)
    t = [0.0]
    cfg = G1BridgeConfig(
        sdk_timeout_retry_interval_s=1.0,
        max_consecutive_sdk_timeouts=3,
    )
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake, clock=lambda: t[0])
    bridge.init()
    bridge.stop("first")
    t[0] = 1.0
    bridge.stop("second")
    t[0] = 2.0
    bridge.stop("third")  # 3rd consecutive 3104 -> ERROR
    assert bridge.state == BridgeState.ERROR
    calls_before = len(fake.set_velocity_calls)
    bridge.close()
    assert len(fake.set_velocity_calls) == calls_before
    assert bridge.state == BridgeState.CREATED


def test_close_still_stops_when_active():
    """When the last command was a successful move (ACTIVE), close() must still
    issue a stop - the robot may still be in motion and a responsive G1 returns
    in milliseconds, so the stop is cheap and necessary."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move())  # state -> ACTIVE
    calls_before = len(fake.set_velocity_calls)
    bridge.close()
    assert len(fake.set_velocity_calls) == calls_before + 1
    assert fake.set_velocity_calls[-1] == _ZERO_VEL_CALL
    assert bridge.state == BridgeState.CREATED


# ---------------------------------------------------------------------------
# import contract + Option A contract
# ---------------------------------------------------------------------------


def test_import_without_sdk():
    """Importing the package must not pull in unitree_sdk2py (delayed import)."""
    script = textwrap.dedent(
        """
        import sys
        assert "unitree_sdk2py" not in sys.modules
        from robot_follow.real.g1 import G1MotionBridge, G1BridgeConfig, BridgeState
        assert G1MotionBridge is not None
        assert "unitree_sdk2py" not in sys.modules
        """
    )
    subprocess.run([sys.executable, "-c", script], check=True)


def test_bridge_never_calls_move_or_stopmove_wrappers():
    """Option A contract: the hot path uses SetVelocity, NOT the Move/StopMove
    wrappers (which discard the return code on the real SDK)."""
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(_move())
    bridge.stop("explicit")
    bridge.dispatch(_move(reason="d_move"))
    bridge.dispatch(
        MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason="d_stop")
    )
    assert fake.move_calls == 0
    assert fake.stopmove_calls == 0
    # ...and SetVelocity WAS used for all four commands
    assert len(fake.set_velocity_calls) == 4
