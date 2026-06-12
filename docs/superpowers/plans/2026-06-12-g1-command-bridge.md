# G1 Command Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a thin but conservative hardware output adapter that converts internal `MotionCommand` to Unitree G1 `LocoClient.Move/StopMove` calls, with delayed import, velocity clamping, yaw sign calibration, watchdog timeout, exception feedback, return-code checking, and explicit motion confirmation switches.

**Architecture:** Keep the existing core pipeline untouched. `G1MotionBridge` sits at the output edge, receiving `MotionCommand` from `FollowPipeline`. It performs final defensive checks (NaN/inf, velocity limits, state validation, SDK return code) before calling SDK. The bridge does NOT implement ALX parsing, filtering, safety decisions, or follow control algorithms—those remain in `SafetyManager` and `FollowController`.

**G1 hardware prerequisite (MANUAL):** G1 does **NOT** boot into standing form. Before the bridge can command motion, an operator must manually prepare the robot via the remote control boot sequence:

```text
Power on → wait ~1 min → zero-torque state
  → press L1+A (remote)        → damping state
  → human holds robot + L1+UP  → preparation state
  → press R2+A (remote)        → motion state → G1 adjusts gait → STANDS UP
```

Only after G1 is standing in motion state can the bridge call `Move`/`StopMove`. The bridge must **NOT** perform FSM transitions itself (no `Damp`/`Start`/`Squat2StandUp` on the hot path — these are dangerous to call blindly). The bridge's responsibility is: assume the operator has prepared the robot; if a `Move` call fails (non-zero return code, e.g. because the robot is not in motion state), treat it as `sdk_error` and report it so the pipeline stops.

**Debug mode:** During SDK development, press L2+R2 on the remote to enter debug mode, which stops the built-in motion control program and avoids command conflicts.

**Tech Stack:** Python 3.10 (robot-follow env), dataclasses, pytest, `unitree_sdk2_python` installed from **GitHub source** (NOT PyPI — see Task 0), existing `robot_follow.core` modules.

---

## Pre-Execution Notes

Before running pytest or any Python project command, follow the repository instruction in `CLAUDE.md`: use the `robot-follow-conda-run-guard` skill, run `conda env list`, and get the user's confirmation for which conda environment to use.

Do not touch the existing unstaged `.gitignore` change unless the user explicitly asks.

This plan follows the design document at [docs/g1_command_bridge_design.md](docs/g1_command_bridge_design.md). Read it fully before implementation.

> **Divergence from the design doc (intentional):** The design doc §7 proposes a single
> file `src/robot_follow/g1_motion.py`. This plan uses a
> `src/robot_follow/real/g1/` package instead (`bridge.py` + `types.py` +
> `__init__.py`), which keeps physical-device adapters under the existing real
> layer and scales better. The design
> doc also references a CLI entry point that does not match the codebase; this plan
> targets the existing `src/robot_follow/main.py` package entry point
> (`robot_follow.main:main`).

## File Structure

- Create `src/robot_follow/ports.py`
  - Add `MotionBridge` Protocol (shared interface for Mock + G1 bridges).
- Modify `src/robot_follow/sim/mock_motion_bridge.py`
  - Add no-op `init()`, `tick(now)`, `close()` and `sdk_error` property so it satisfies `MotionBridge`.
- Create `src/robot_follow/real/g1/__init__.py`
  - Package init, exports `G1MotionBridge`, `G1BridgeConfig`, `BridgeState`.
- Create `src/robot_follow/real/g1/bridge.py`
  - Owns `G1MotionBridge`, state machine, watchdog.
  - Imports existing `MotionCommand` from `robot_follow.core.types`.
- Create `src/robot_follow/real/g1/types.py`
  - Owns `G1BridgeConfig` dataclass and `BridgeState` enum.
- Create `src/robot_follow/real/g1/check.py`
  - Dedicated hardware verification entry point for init-only, stop-only, and
    single short move burst tests that do NOT read ALX data or run the follow loop.
- Create `tests/real/g1/test_bridge.py`
  - Unit tests with fake client injection, covering move, stop, NaN handling, clamping, yaw sign, state transitions.
- Modify `src/robot_follow/main.py`
  - The existing package entry point for headless live ALX dry-run
    (`robot-follow = robot_follow.main:main`).
  - Add `--dry-run`, `--enable-motion`, `--i-understand-this-moves-robot` switches.
  - Integrate `MotionBridge` into `run_live_alx_simulation` (sdk_error feedback, watchdog tick, finally cleanup).
- Create `docs/alx_g1_verification_chain.md` (if not exists)
  - Step-by-step G1 hardware verification procedure.
- Use existing `src/robot_follow/core/types.py` for `MotionCommand` (DO NOT redefine).
- Keep `robot_follow.core` hardware-independent; output-side interfaces live in
  `robot_follow.ports`, while real hardware adapters live under
  `robot_follow.real`.
- Use existing `src/robot_follow/sim/mock_motion_bridge.py` for dry-run mode (DO NOT recreate; EXTEND it in Task 2).

---

### Task 0: Install Unitree SDK From GitHub Source (Prerequisite)

**Files:**
- Environment: `robot-follow` conda env
- Reference: `/tmp/unitree_sdk2_python` (clone location)

> **CRITICAL:** Do NOT use `pip install unitree-sdk2` from PyPI. The PyPI package (v1.0.1) only contains `go2` and `b2` modules — it has **no `g1` module** and ships a broken `__init__.py` with a circular import (`from . import ... b2` fails). The G1 `LocoClient` only exists in the official GitHub source. See memory `unitree-sdk-github-install`.

- [ ] **Step 1: Confirm conda env**

Use the `robot-follow-conda-run-guard` skill. Run `conda env list`, confirm `robot-follow` with the user.

- [ ] **Step 2: Clone the official SDK**

Run:

```bash
rm -rf /tmp/unitree_sdk2_python
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2_python
```

- [ ] **Step 3: Install into robot-follow env**

Run:

```bash
conda run -n robot-follow pip install -e /tmp/unitree_sdk2_python
```

Dependencies (`cyclonedds==0.10.2`, `numpy`, `opencv-python`) install automatically.

- [ ] **Step 4: Verify G1 module imports**

Run:

```bash
conda run -n robot-follow python -c "
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
methods = [m for m in dir(LocoClient) if not m.startswith('_')]
assert 'Move' in methods and 'StopMove' in methods and 'SetVelocity' in methods
print('G1 SDK OK:', methods)
"
```

Expected: prints the method list including `Move`, `StopMove`, `SetVelocity`, `Init`, `SetTimeout`. No import errors.

- [ ] **Step 5: Record the verified LocoClient contract**

Confirmed API (from source `unitree_sdk2py/g1/loco/g1_loco_client.py`):

```text
Init()                                       # registers RPC APIs; does NOT change robot FSM state
SetTimeout(timeout: float)                   # set SDK call timeout
Move(vx, vy, vyaw, continous_move=False)     # duration=1 by default, 864000 if continuous
StopMove()                                   # equivalent to SetVelocity(0,0,0)
SetVelocity(vx, vy, omega, duration=1.0)     # lower-level velocity with custom duration

# All of the above return a `code` (0 = success, non-zero = failure; they do NOT raise).
# Non-zero typically means G1 is not in motion state (e.g. still in zero-torque/damping,
# or the operator has not finished the manual boot sequence). The bridge reports this
# as sdk_error rather than retrying FSM transitions.

# FSM methods exist but are NOT used by the bridge in the first release:
#   ZeroTorque() -> FSM 0
#   Damp()       -> FSM 1
#   Start()      -> FSM 500
#   Squat2StandUp() -> FSM 706  (requires Damp() first, 0.5s delay)
#   Lie2StandUp()   -> FSM 702
# G1 state transitions to standing are done MANUALLY by the operator via the remote
# control boot sequence (see header "G1 hardware prerequisite"), NOT by the bridge.
```

No commit needed for this task (it only modifies the conda environment, not the repo).

---

### Task 1: Create g1 Package Structure And Define G1BridgeConfig/BridgeState

**Files:**
- Create: `src/robot_follow/real/g1/__init__.py`
- Create: `src/robot_follow/real/g1/types.py`
- Test: `python -c "from robot_follow.real.g1 import G1BridgeConfig, BridgeState"`

- [ ] **Step 1: Create the g1 package directory**

Run:

```bash
mkdir -p src/robot_follow/real/g1
touch src/robot_follow/real/g1/__init__.py
```

- [ ] **Step 2: Create types.py with G1BridgeConfig and BridgeState**

Create `src/robot_follow/real/g1/types.py`:

```python
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


class BridgeState(Enum):
    """Internal state machine for the motion bridge.

    State transitions:
    - CREATED -> INITIALIZED: init() success
    - INITIALIZED/STOPPED -> ACTIVE: move() success
    - ACTIVE/INITIALIZED -> STOPPED: stop() or watchdog timeout
    - ANY -> ERROR: SDK exception, validation failure, init failure
    - ERROR -> STOPPED: stop() clears motion output
    """

    CREATED = "created"      # SDK not yet initialized
    INITIALIZED = "initialized"  # DDS and LocoClient ready
    ACTIVE = "active"        # Last command was a successful move
    STOPPED = "stopped"      # Last command was StopMove
    ERROR = "error"          # SDK error or validation failure
```

- [ ] **Step 3: Create __init__.py with exports**

Create `src/robot_follow/real/g1/__init__.py`:

```python
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
```

- [ ] **Step 4: Run import test**

Run:

```bash
python -c "from robot_follow.real.g1 import G1BridgeConfig, BridgeState; print('OK')"
```

Expected: PASS without importing `unitree_sdk2py` or accessing network.

- [ ] **Step 5: Commit package skeleton**

Run:

```bash
git add src/robot_follow/real/g1/__init__.py src/robot_follow/real/g1/types.py
git commit -m "feat: add g1 package skeleton with G1BridgeConfig and BridgeState"
```

---

### Task 2: Define MotionBridge Protocol And Extend MockMotionBridge

**Why this comes before the bridge implementation:** `G1MotionBridge` and
`MockMotionBridge` must be interchangeable so dry-run and real-run share the same main
loop. Currently `MockMotionBridge` lacks `init()`, `tick()`, `close()`, and `sdk_error` —
calling `bridge.init()` on a mock in the dry-run path raises `AttributeError`. A shared
`MotionBridge` Protocol makes the interchange guarantee structural rather than by convention.

**Files:**
- Create: `src/robot_follow/ports.py` — add `MotionBridge` Protocol
- Modify: `src/robot_follow/sim/mock_motion_bridge.py` — add no-op `init/tick/close` + `sdk_error`
- Modify: `tests/sim/test_mock_motion_bridge.py` — add tests for the new methods
- Test: `pytest tests/sim/test_mock_motion_bridge.py`

- [ ] **Step 1: Add `MotionBridge` Protocol to `ports.py`**

Create `src/robot_follow/ports.py`. Keep this outside `robot_follow.core` so the
core package remains algorithm/data only and does not start owning hardware
output boundaries:

```python
"""Output-side interfaces for robot_follow adapters."""

from __future__ import annotations

from typing import Protocol, Union

from robot_follow.core.types import MotionCommand


class MotionBridge(Protocol):
    """Interface for motion output bridges.

    Both ``MockMotionBridge`` (simulation / dry-run) and ``G1MotionBridge``
    (real robot) implement this protocol. The main loop depends only on this
    interface, so dry-run and real-run share the same control flow.
    """

    @property
    def sdk_error(self) -> bool:
        """Whether an SDK error has occurred (always False for mock bridges)."""
        ...

    def init(self) -> None:
        """Initialize the bridge (DDS + SDK client, or no-op for mock)."""
        ...

    def dispatch(self, command: "MotionCommand") -> None:
        """Dispatch a MotionCommand to the appropriate handler."""
        ...

    def move(self, command: "MotionCommand") -> None:
        """Handle a move command."""
        ...

    def stop(self, reason: Union[str, "MotionCommand"]) -> None:
        """Handle a stop command."""
        ...

    def tick(self, now: float) -> None:
        """Watchdog tick — call once per main-loop iteration."""
        ...

    def close(self) -> None:
        """Clean shutdown."""
        ...
```

- [ ] **Step 2: Do not export `MotionBridge` from `robot_follow.core`**

Callers should import it from `robot_follow.ports`:

```python
from robot_follow.ports import MotionBridge
```

- [ ] **Step 3: Extend `MockMotionBridge` with the protocol methods**

Add these members to `src/robot_follow/sim/mock_motion_bridge.py` (inside the class body):

```python
    # -----------------------------------------------------------------------
    # MotionBridge protocol (no-op for simulation)
    # -----------------------------------------------------------------------
    @property
    def sdk_error(self) -> bool:
        """Mock bridges never report SDK errors."""
        return False

    def init(self) -> None:
        """No-op: a mock bridge needs no initialization."""
        return None

    def tick(self, now: float) -> None:
        """No-op: a mock bridge has no watchdog."""
        return None

    def close(self) -> None:
        """No-op: a mock bridge has nothing to clean up (history preserved)."""
        return None
```

- [ ] **Step 4: Add tests for the new methods**

Append to `tests/sim/test_mock_motion_bridge.py`:

```python
class TestMockMotionBridgeProtocol:
    def test_init_is_noop(self):
        bridge = MockMotionBridge()
        bridge.init()  # must not raise
        assert bridge.move_count == 0

    def test_tick_is_noop(self):
        import time
        bridge = MockMotionBridge()
        bridge.tick(time.monotonic())  # must not raise

    def test_close_is_noop_and_preserves_history(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        bridge.close()
        assert bridge.move_count == 1  # close does not clear history

    def test_sdk_error_always_false(self):
        bridge = MockMotionBridge()
        assert bridge.sdk_error is False
        bridge.dispatch(_move())
        assert bridge.sdk_error is False
```

- [ ] **Step 5: Verify mock bridge tests pass**

Run:

```bash
pytest tests/sim/test_mock_motion_bridge.py -v
```

Expected: all existing tests plus the four new protocol tests pass.

- [ ] **Step 6: Commit Protocol and MockMotionBridge extension**

Run:

```bash
git add src/robot_follow/core/types.py src/robot_follow/core/__init__.py \
        src/robot_follow/sim/mock_motion_bridge.py tests/sim/test_mock_motion_bridge.py
git commit -m "feat: add MotionBridge Protocol and extend MockMotionBridge for interchange"
```

---

### Task 3: Implement G1MotionBridge Core Logic With Fake Client Injection

**Files:**
- Create: `src/robot_follow/real/g1/bridge.py`
- Update: `src/robot_follow/real/g1/__init__.py`
- Create: `tests/real/g1/__init__.py`
- Create: `tests/real/g1/test_bridge.py`
- Test: `tests/real/g1/test_bridge.py`

> `G1MotionBridge` implements the `MotionBridge` Protocol defined in Task 2. The four
> protocol methods (`init`, `dispatch`, `move`, `stop`, `tick`, `close`, `sdk_error`)
> must match the Protocol signatures exactly so it is a drop-in replacement for
> `MockMotionBridge` in `run_live_alx_simulation`.

- [ ] **Step 1: Create test directory**

Run:

```bash
mkdir -p tests/real/g1
touch tests/real/g1/__init__.py
```

- [ ] **Step 2: Create FakeLocoClient for testing**

Create `tests/real/g1/test_bridge.py` starting with the fake client:

```python
"""Unit tests for G1MotionBridge with fake client injection."""

import subprocess
import sys
import textwrap

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.real.g1.types import G1BridgeConfig, BridgeState
from robot_follow.real.g1.bridge import G1MotionBridge


class FakeLocoClient:
    """Fake LocoClient for testing without SDK.

    Mimics the unitree_sdk2py.g1.loco.g1_loco_client.LocoClient interface.
    IMPORTANT: like the real SDK, Move/StopMove/SetVelocity RETURN a `code`
    (0 = success, non-zero = failure) instead of raising. Tests can force a
    failure via `fail_next=True`.
    """

    def __init__(self, fail_next: bool = False):
        # NOTE: G1MotionBridge calls Move(vx, vy, vyaw) with only 3 positional
        # args (it deliberately does NOT use continuous_move — see
        # G1BridgeConfig docstring for the watchdog rationale). The 4th tuple
        # slot below stays for FakeLocoClient.Move's default-arg path.
        self.moves = []          # list of (vx, vy, vyaw, continuous=False)
        self.set_velocity_calls = []  # list of (vx, vy, omega, duration)
        self.stops = 0
        self.init_called = False
        self.timeout = None
        self.fail_next = fail_next  # if True, next call returns code 1

    def _code(self) -> int:
        if self.fail_next:
            self.fail_next = False
            return 1
        return 0

    def SetTimeout(self, timeout: float):
        self.timeout = timeout

    def Init(self):
        self.init_called = True

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        self.set_velocity_calls.append((vx, vy, omega, duration))
        return self._code()

    def Move(self, vx: float, vy: float, vyaw: float, continuous: bool = False):
        # Mirror real SDK: duration = 864000 if continuous else 1
        duration = 864000.0 if continuous else 1
        self.set_velocity_calls.append((vx, vy, vyaw, duration))
        self.moves.append((vx, vy, vyaw, continuous))
        return self._code()

    def StopMove(self):
        # Real SDK: SetVelocity(0,0,0)
        self.set_velocity_calls.append((0.0, 0.0, 0.0, 1.0))
        self.stops += 1
        return self._code()
```

- [ ] **Step 3: Implement G1MotionBridge in bridge.py**

Create `src/robot_follow/real/g1/bridge.py`:

```python
"""G1 motion bridge - converts MotionCommand to Unitree LocoClient calls."""

from __future__ import annotations

import logging
import math
import time
from typing import Callable, Union

from robot_follow.core.types import MotionCommand
from robot_follow.real.g1.types import G1BridgeConfig, BridgeState


logger = logging.getLogger(__name__)


class G1MotionBridge:
    """Converts MotionCommand to Unitree G1 LocoClient calls.

    This bridge is a thin but conservative hardware output adapter. It:
    - Performs final defensive validation (NaN/inf, velocity limits)
    - Manages internal state machine (CREATED, INITIALIZED, ACTIVE, STOPPED, ERROR)
    - Implements watchdog timeout for automatic stop
    - Uses delayed SDK import (init() time, not module import time)

    Hot-path safety rule: ``move``/``stop``/``tick`` MUST NOT raise. A runtime
    or SDK error is recorded as ``sdk_error`` + best-effort stop + ERROR state,
    so the control loop never dies on a recoverable error; the next frame sees
    ``sdk_error`` and the pipeline outputs a stop. Only ``init()`` raises
    (pre-loop, fatal — the run cannot start).

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

        Uses delayed import so module can be imported without SDK installed.
        """
        if self._fake_client is not None:
            self._client = self._fake_client
            self._state = BridgeState.INITIALIZED
            logger.info("G1MotionBridge initialized with fake client (test mode)")
            return

        # Delayed import - only when actually initializing
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        # NOTE: ChannelFactoryInitialize(0, iface) initializes a PROCESS-GLOBAL
        # DDS factory. It is NOT idempotent and NOT re-callable: calling init()
        # again (e.g. after an ERROR) will not cleanly re-create the factory.
        # Recovery from a real init failure requires a process restart, not a
        # second init() call.
        try:
            ChannelFactoryInitialize(0, self._network_interface)
            self._client = LocoClient()
            self._client.SetTimeout(self._config.sdk_timeout_s)
            self._client.Init()
            self._state = BridgeState.INITIALIZED
            logger.info(f"G1MotionBridge initialized on {self._network_interface}")
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
        5. Call LocoClient.Move (continuous_move omitted -> SDK default
           duration=1, so a crashed process holds the last velocity <=1s)
        """
        if self._state in (BridgeState.CREATED, BridgeState.ERROR):
            # Refuse move; never crash the loop. Flag sdk_error so the next
            # frame's pipeline outputs a stop.
            self._record_error(f"move rejected (state={self._state.value})")
            return
        if not self._validate_move_command(command):
            return  # validation failure already recorded + best-effort stop

        vx = _clamp(command.vx, -self._config.max_abs_vx, self._config.max_abs_vx)
        vy = 0.0  # Force lateral to zero in first release
        omega = _clamp(
            self._config.yaw_sign * command.omega,
            -self._config.max_abs_omega,
            self._config.max_abs_omega,
        )

        try:
            code = self._client.Move(vx, vy, omega)
            if code != 0:
                self._record_error(f"G1 Move returned error code {code}")
                return
            self._state = BridgeState.ACTIVE
            self._last_dispatch_time = self._clock()
            logger.debug(f"G1 Move: vx={vx:.3f} omega={omega:.3f}")
        except Exception as e:
            self._record_error(f"G1 Move failed: {e}")

    def stop(self, reason: Union[str, MotionCommand] = "stop") -> None:
        """Send a stop command.

        Always calls StopMove regardless of reason. This preserves the safety
        semantics: stop commands must result in StopMove being called. Never
        raises; a StopMove failure is recorded as sdk_error + ERROR state so the
        control loop keeps running.
        """
        if self._client is None:
            logger.warning("G1 StopMove called before init, ignoring")
            return
        try:
            code = self._client.StopMove()
            if code != 0:
                self._sdk_error = True
                self._state = BridgeState.ERROR
                logger.error(f"G1 StopMove returned error code {code}")
                return
            self._state = BridgeState.STOPPED
            self._last_dispatch_time = self._clock()
            logger.info(f"G1 StopMove: reason={reason if isinstance(reason, str) else reason.reason}")
        except Exception as e:
            self._sdk_error = True
            self._state = BridgeState.ERROR
            logger.error(f"G1 StopMove failed: {e}")

    def tick(self, now: float | None = None) -> None:
        """Watchdog tick - call StopMove if command timeout expired.

        Call this regularly (e.g., every main loop iteration) to ensure
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
            logger.warning(f"G1 watchdog timeout ({elapsed:.2f}s > {self._config.command_timeout_s}s)")
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

        Sets sdk_error + ERROR state and attempts a best-effort StopMove. The
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
        """Attempt StopMove without raising on failure."""
        if self._client is None:
            return
        try:
            self._client.StopMove()
        except Exception:
            pass  # Suppress secondary failure


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi] range."""
    return max(lo, min(hi, value))


def _is_finite(value: float) -> bool:
    """Check if value is finite (not NaN or inf)."""
    return math.isfinite(value)
```

- [ ] **Step 4: Update __init__.py to export G1MotionBridge**

Update `src/robot_follow/real/g1/__init__.py` to add:

```python
from robot_follow.real.g1.bridge import G1MotionBridge
from robot_follow.real.g1.types import G1BridgeConfig, BridgeState

__all__ = [
    "G1MotionBridge",
    "G1BridgeConfig",
    "BridgeState",
]
```

- [ ] **Step 5: Add unit tests to test_bridge.py**

Complete `tests/real/g1/test_bridge.py`:

```python
def test_fake_client_init():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    assert bridge.state == BridgeState.INITIALIZED
    assert fake.init_called
    assert fake.timeout == pytest.approx(10.0)


def test_move_command_calls_move():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="test"))
    assert len(fake.moves) == 1
    vx, vy, omega, _ = fake.moves[0]
    assert vx == pytest.approx(0.1)
    assert vy == pytest.approx(0.0)
    assert omega == pytest.approx(0.2)
    assert bridge.state == BridgeState.ACTIVE


def test_stop_command_calls_stopmove():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="test"))
    bridge.stop("safety_stop")
    assert fake.stops >= 1
    assert bridge.state == BridgeState.STOPPED


def test_holding_move_calls_move_zero():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.0, vy=0.0, omega=0.0, reason="holding"))
    assert len(fake.moves) == 1
    vx, vy, omega, _ = fake.moves[0]
    assert vx == pytest.approx(0.0)
    assert omega == pytest.approx(0.0)


def test_nan_velocity_triggers_stop_and_error():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=float("nan"), vy=0.0, omega=0.0, reason="bad"))
    assert fake.stops >= 1  # Best-effort stop attempted
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


def test_inf_velocity_triggers_stop_and_error():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=float("inf"), vy=0.0, omega=0.0, reason="bad"))
    assert fake.stops >= 1
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


def test_over_limit_velocity_is_clamped():
    fake = FakeLocoClient()
    cfg = G1BridgeConfig(max_abs_vx=0.10, max_abs_omega=0.30)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.5, vy=0.0, omega=1.0, reason="fast"))
    vx, vy, omega, _ = fake.moves[0]
    assert vx == pytest.approx(0.10)  # Clamped
    assert omega == pytest.approx(0.30)  # Clamped


def test_yaw_sign_flips_omega():
    fake = FakeLocoClient()
    cfg = G1BridgeConfig(yaw_sign=-1)
    bridge = G1MotionBridge("eth0", config=cfg, fake_client=fake)
    bridge.init()
    # omega=0.2 is within the default max_abs_omega=0.30, so no clamping
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="turn"))
    vx, vy, omega, _ = fake.moves[0]
    assert omega == pytest.approx(-0.2)  # Sign flipped, not clamped


def test_move_before_init_records_error_without_raising():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    # Do NOT call init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="early"))
    # Must NOT raise; must flag sdk_error so the loop survives
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    assert len(fake.moves) == 0  # No Move sent before init


def test_watchdog_timeout_calls_stop():
    fake = FakeLocoClient()
    t = [0.0]  # controllable injected clock
    bridge = G1MotionBridge(
        "eth0",
        config=G1BridgeConfig(command_timeout_s=0.3),
        fake_client=fake,
        clock=lambda: t[0],
    )
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="test"))
    t[0] = 0.5  # advance the injected clock past the timeout
    bridge.tick()  # uses the injected clock
    assert fake.stops >= 1
    assert bridge.state == BridgeState.STOPPED


def test_dispatch_routes_stop():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.dispatch(MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason="safety"))
    assert fake.stops >= 1
    assert bridge.state == BridgeState.STOPPED


def test_dispatch_routes_move():
    fake = FakeLocoClient()
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.dispatch(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.1, reason="track"))
    assert len(fake.moves) == 1
    assert bridge.state == BridgeState.ACTIVE


def test_import_without_sdk():
    """Verify module imports without unitree_sdk2py in a fresh process."""
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


def test_move_sdk_return_code_nonzero_triggers_error():
    """SDK Move returns non-zero code (no exception); bridge records it."""
    fake = FakeLocoClient(fail_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="test"))
    # Must NOT raise; records error so the loop stays alive
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR
    assert fake.stops >= 1  # Best-effort stop attempted


def test_stop_sdk_return_code_nonzero_triggers_error():
    """SDK StopMove returns non-zero code; bridge records it without raising."""
    fake = FakeLocoClient(fail_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.stop("test")
    # Must NOT raise
    assert bridge.sdk_error
    assert bridge.state == BridgeState.ERROR


def test_close_is_best_effort_when_stop_fails():
    """close() must complete cleanup even if the underlying StopMove fails."""
    fake = FakeLocoClient(fail_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.close()  # must not raise even though StopMove returns code 1
    assert bridge.state == BridgeState.CREATED  # cleanup ran
    # _client is reset to None so a second close() is safe too
    bridge.close()


def test_move_in_error_state_is_rejected_without_raising():
    """Once in ERROR, move() refuses to send but never raises."""
    fake = FakeLocoClient(fail_next=True)
    bridge = G1MotionBridge("eth0", fake_client=fake)
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="first"))
    assert bridge.state == BridgeState.ERROR
    moves_before = len(fake.moves)
    # Subsequent moves: no raise, no new SDK Move call
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="second"))
    assert len(fake.moves) == moves_before
    assert bridge.state == BridgeState.ERROR


def test_clock_injection_is_used_by_move_and_tick():
    """move() and tick() must use the injected clock, not time.time()."""
    fake = FakeLocoClient()
    t = [100.0]
    bridge = G1MotionBridge(
        "eth0",
        config=G1BridgeConfig(command_timeout_s=1.0),
        fake_client=fake,
        clock=lambda: t[0],
    )
    bridge.init()
    bridge.move(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="test"))
    # 0.5s elapsed: under timeout, no stop
    t[0] = 100.5
    bridge.tick()
    assert fake.stops == 0
    # 1.5s elapsed: over timeout, stop fires
    t[0] = 101.5
    bridge.tick()
    assert fake.stops >= 1
```

---

### Task 4: Add CLI Safety Switches To main.py

**Files:**
- Modify: `src/robot_follow/main.py`
- Create: `src/robot_follow/real/g1/check.py`
- Test: manual CLI verification

> `main.py` is the package top-level entry point (`robot-follow =
> robot_follow.main:main`). The `robot-follow` console command works
> without changes to `pyproject.toml`.

- [ ] **Step 1: Add motion control switches to `_parse_args`**

Add these flags to the `argparse.ArgumentParser` in `_parse_args`:

```text
--dry-run              Use MockMotionBridge (default if no --enable-motion)
--enable-motion        Enable real G1MotionBridge (requires confirmation)
--i-understand-this-moves-robot  Explicit user consent for real motion
--net-iface <name>     Network interface for DDS (default: enp2s0)
--max-vx <float>       Override max_abs_vx (default: G1BridgeConfig.max_abs_vx)
--max-omega <float>    Override max_abs_omega (default: G1BridgeConfig.max_abs_omega)
--yaw-sign <int>       Override yaw_sign (+1 or -1, default: +1)
```

Implementation:

```python
parser.add_argument("--dry-run", action="store_true", default=False,
                    help="Use MockMotionBridge (no real robot motion).")
parser.add_argument("--enable-motion", action="store_true", default=False,
                    help="Enable G1MotionBridge (requires --i-understand-this-moves-robot).")
parser.add_argument("--i-understand-this-moves-robot", action="store_true", default=False,
                    help="Explicit consent: the robot WILL physically move.")
parser.add_argument("--net-iface", default="enp2s0",
                    help="DDS network interface for G1 SDK.")
parser.add_argument("--max-vx", type=float, default=None,
                    help="Override max forward/backward velocity (m/s).")
parser.add_argument("--max-omega", type=float, default=None,
                    help="Override max rotation velocity (rad/s).")
parser.add_argument("--yaw-sign", type=int, default=1, choices=[1, -1],
                    help="Yaw sign calibration (+1 or -1).")
```

After parsing, reject contradictory intent:

```python
if args.dry_run and args.enable_motion:
    parser.error("--dry-run and --enable-motion are mutually exclusive")
```

- [ ] **Step 2: Add `create_motion_bridge` factory function**

Add to `main.py` (before `run_live_alx_simulation`). The current
`main.py` uses `print` via the `emit` injection and has NO module-level
logger; we add one alongside this factory so the safety warnings are clearly
distinguished from frame logs:

```python
import logging

logger = logging.getLogger(__name__)


def create_motion_bridge(args: argparse.Namespace) -> "MotionBridge":
    """Create the appropriate motion bridge based on CLI flags.

    Returns a MotionBridge (MockMotionBridge or G1MotionBridge).
    G1MotionBridge requires both --enable-motion AND
    --i-understand-this-moves-robot.
    """
    from robot_follow.sim.mock_motion_bridge import MockMotionBridge

    if not args.enable_motion:
        logger.warning("DRY-RUN MODE: Using MockMotionBridge, robot will NOT move")
        return MockMotionBridge()

    if not args.i_understand_this_moves_robot:
        logger.error("Real motion requires --i-understand-this-moves-robot consent")
        raise SystemExit(1)

    from robot_follow.real.g1 import G1MotionBridge, G1BridgeConfig

    logger.warning("REAL MOTION ENABLED: Robot will move!")
    cfg_kwargs = {"yaw_sign": args.yaw_sign}
    if args.max_vx is not None:
        cfg_kwargs["max_abs_vx"] = args.max_vx
    if args.max_omega is not None:
        cfg_kwargs["max_abs_omega"] = args.max_omega
    cfg = G1BridgeConfig(**cfg_kwargs)
    return G1MotionBridge(args.net_iface, config=cfg)
```

Also add the matching import at the top of `main.py` (in the existing
`from robot_follow.core...` block) so the `MotionBridge` type annotation works
without quotes elsewhere:

```python
from robot_follow.ports import MotionBridge
```

Note: `MotionBridge` is a `Protocol`; the existing `from __future__ import annotations`
at the top of `main.py` makes the string-quoted return type above safe even
without the import, but Task 7 will use it as a real parameter type so import it now.

- [ ] **Step 3: Add dedicated G1 hardware check entry point**

Create `src/robot_follow/real/g1/check.py` for staged hardware validation.
This entry point must not import or read ALX data and must not run
`run_live_alx_simulation`; it is only for direct bridge checks.

Required modes:

```text
--init-only       Initialize bridge, then close.
--stop-only       Initialize bridge, send StopMove, then close.
--single-move     Initialize bridge, send one short bounded Move, then StopMove.
```

All real modes require `--i-understand-this-moves-robot`. `--single-move` must
also require explicit low command values, defaulting to `vx=0.05`, `omega=0.0`,
and `duration=0.5`.

Implementation shape:

```python
def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if not args.i_understand_this_moves_robot:
        raise SystemExit("--i-understand-this-moves-robot is required")

    cfg = G1BridgeConfig(max_abs_vx=args.max_vx, max_abs_omega=args.max_omega)
    bridge = G1MotionBridge(args.net_iface, config=cfg)
    try:
        bridge.init()
        if args.init_only:
            return
        if args.stop_only:
            bridge.stop("g1_check_stop_only")
            return
        if args.single_move:
            bridge.dispatch(MotionCommand(
                kind="move",
                vx=args.vx,
                vy=0.0,
                omega=args.omega,
                reason="g1_check_single_move",
            ))
            time.sleep(args.duration)
            bridge.stop("g1_check_single_move_complete")
            return
    finally:
        bridge.stop("g1_check_shutdown")
        bridge.close()
```

- [ ] **Step 4: Verify CLI help output**

Run:

```bash
python -m robot_follow.main --help
# Or, after pip install -e .:
robot-follow --help
```

Expected: Shows `--dry-run`, `--enable-motion`, `--i-understand-this-moves-robot`, `--net-iface`, `--max-vx`, `--max-omega`, `--yaw-sign` alongside existing flags. The `robot-follow` command now works because `main.py` matches the `pyproject.toml` entry point.

- [ ] **Step 5: Commit CLI changes**

Run:

```bash
git add src/robot_follow/main.py src/robot_follow/real/g1/check.py
git commit -m "feat: add CLI safety switches for G1 motion control"
```

---

### Task 5: Verify Delayed-Import Contract

**Files:**
- Test: import-contract test on the robot-follow machine

> Now that the SDK is installed (Task 0), the goal is NOT "can we import without SDK" — it is "does importing `robot_follow.real.g1` leave `unitree_sdk2py` untouched until `init()` is called." This keeps dry-run (fake_client) paths from dragging in the DDS stack, and keeps the package importable on machines without the SDK.

- [ ] **Step 1: Verify module import does not pull in the SDK**

Run:

```bash
conda run -n robot-follow python -c "
import sys
import robot_follow.real.g1  # import the package
assert 'unitree_sdk2py' not in sys.modules, 'SDK imported at module load time!'
from robot_follow.real.g1 import G1MotionBridge, G1BridgeConfig, BridgeState
assert 'unitree_sdk2py' not in sys.modules, 'SDK imported on class access!'
print('Delayed-import contract OK: SDK not touched until init()')
"
```

Expected: prints the OK line; `unitree_sdk2py` is absent from `sys.modules`.

- [ ] **Step 2: Verify fake_client init still does not import SDK**

Run:

```bash
conda run -n robot-follow python -c "
import sys
from robot_follow.real.g1 import G1MotionBridge

class FakeLocoClient:
    def SetTimeout(self, t): pass
    def Init(self): pass
    def Move(self, vx, vy, vyaw, continuous=False): return 0
    def SetVelocity(self, vx, vy, omega, duration=1.0): return 0
    def StopMove(self): return 0

bridge = G1MotionBridge('eth0', fake_client=FakeLocoClient())
bridge.init()
bridge.move(__import__('robot_follow.core.types', fromlist=['MotionCommand']).MotionCommand(kind='move', vx=0.1, vy=0.0, omega=0.0, reason='t'))
bridge.stop()
assert 'unitree_sdk2py' not in sys.modules, 'SDK imported via fake_client path!'
print('Fake-client path OK: SDK never imported')
"
```

Expected: prints the OK line; SDK never imported even after a full init/move/stop cycle with the fake client.

- [ ] **Step 3: Verify a real (non-fake) init DOES import the SDK**

Run **without** a connected robot — `ChannelFactoryInitialize` will still import the SDK module even though it cannot reach the robot, which is what we want to confirm (lazy import only fires at init):

```bash
conda run -n robot-follow python -c "
import sys
from robot_follow.real.g1 import G1MotionBridge
bridge = G1MotionBridge('lo')  # dummy interface; no robot connected
try:
    bridge.init()
except Exception as e:
    print(f'init raised (expected, no robot): {type(e).__name__}')
# The SDK MUST now be imported (lazy import fired at init time)
assert 'unitree_sdk2py' in sys.modules, 'SDK not imported after init()!'
print('Real-init path OK: SDK imported at init() time')
"
```

Expected: prints the "init raised" line (acceptable — no robot) and the "Real-init path OK" line confirming the SDK was imported.

---

### Task 6: Create G1 Hardware Verification Chain Document

**Files:**
- Create: `docs/alx_g1_verification_chain.md`

- [ ] **Step 1: Create the verification document**

Create `docs/alx_g1_verification_chain.md`:

```markdown
# ALX-G1 Verification Chain

This document describes the step-by-step hardware verification procedure before closing the full follow loop.

## Prerequisites

- Jetson Orin connected to G1 via Ethernet
- `unitree_sdk2_python` installed from **GitHub source** (not PyPI) — see Task 0
- Remote controller online and functional
- Safety operator ready with emergency stop

## ⚠️ Stage 0: Manual Boot Preparation (MUST be done by operator)

G1 does **NOT** boot into standing form. An operator must manually prepare the robot
before any SDK motion commands can succeed. This is done via the remote control, NOT
through our bridge software.

Official boot sequence (source: https://support.unitree.com/home/zh/G1_developer/quick_start):

```text
Step 1: Power on G1 → wait ~1 minute → G1 enters zero-torque state
Step 2: Press L1+A (remote)        → G1 enters damping state
Step 3: Hold G1 shoulder + press L1+UP → human assists G1 into preparation state
Step 4: Press R2+A (remote)        → G1 enters motion state, adjusts gait, STANDS UP
```

Verify G1 is standing and stable before proceeding to Stage 1.

If developing/debugging with SDK: press L2+R2 on remote to enter **debug mode**
(stops built-in motion control to avoid command conflicts).

## Stage 1: SDK Import Verification

```bash
python -c "from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; print('SDK OK')"
```

Expected: No import errors.

## Stage 2: Official SDK Example Test

Run the official example to confirm SDK ↔ G1 communication:

```bash
cd /tmp/unitree_sdk2_python
python3 example/g1/high_level/g1_loco_client_example.py enp2s0
```

Try: `3` (move forward 0.3 m/s for 1 s), then observe the robot moves.

Expected: Robot moves forward briefly, then stops after 1 second (duration=1).

## Stage 3: Bridge Init Only

Run the dedicated G1 check entry point. This does not read ALX data and does not
enter the follow loop:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --init-only \
  --i-understand-this-moves-robot
```

Expected: bridge initializes and closes without sending a move command.

## Stage 4: StopMove Only (No Motion)

Run with real bridge but only send stop:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --stop-only \
  --i-understand-this-moves-robot
```

Expected: StopMove called successfully (code=0), robot stands still.

## Stage 5: Single Move Burst (0.5-1.0s)

Use most conservative parameters:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --i-understand-this-moves-robot \
  --max-vx 0.05 \
  --max-omega 0.20 \
  --vx 0.05 \
  --omega 0.0 \
  --duration 0.5
```

This sends a single short move command, then stops. It does not read ALX data.

Expected: Robot moves slowly forward for <1s, then stops. Verify return code is 0 in logs.

## Stage 6: Continuous Move + Watchdog

Test watchdog timeout behavior:

```bash
# Send move, then pause input for >0.5s
# Watchdog should auto-stop
```

Expected: Robot stops automatically after command timeout.

## Stage 7: SDK Error Code Test

Two ways to trigger a non-zero SDK return code (remember: SDK calls do NOT raise — they return a code):

1. **G1 not in motion state**: skip Stage 0 (leave G1 in damping/zero-torque) and call Move.
   The bridge should detect non-zero code and set sdk_error.
2. **Link loss**: disconnect Ethernet mid-run while G1 is in motion state.

```bash
# Scenario 1: do NOT complete Stage 0, then send a single bounded move:
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --i-understand-this-moves-robot \
  --max-vx 0.05 \
  --vx 0.05 \
  --duration 0.5
# Bridge should log "error code", transition to ERROR, and attempt StopMove.
```

Expected: Bridge logs "error code", transitions to ERROR state, and sends a
best-effort StopMove. Do not use the ALX closed-loop command for this test.

## Stage 8: Yaw Sign Calibration

Test rotation direction:

```bash
# Target at left (azimuth > 0)
# Observe G1 turn direction
```

If G1 turns wrong direction, set `--yaw-sign -1`.

## Stage 9: Full ALX-G1 Closed Loop

Only after all stages pass:

```bash
python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.10 \
  --max-omega 0.30
```

Safety operator must be ready at all times.
```

- [ ] **Step 2: Commit verification document**

Run:

```bash
git add docs/alx_g1_verification_chain.md
git commit -m "docs: add G1 hardware verification chain"
```

---

### Task 7: Integrate With FollowPipeline For Real Run

**Files:**
- Modify: `src/robot_follow/main.py` (the existing `run_live_alx_simulation` loop)
- Test: dry-run integration test

> **Do NOT reinvent the loop.** A complete ALX → pipeline → bridge loop already exists
> in `run_live_alx_simulation`. This task MODIFIES it to support a real `MotionBridge`,
> rather than writing a new `real_run_main` sketch with a `read_alx_sample()` placeholder
> (the real `ALXLiveSource` + `sample_from_snapshot` already work).
>
> The existing loop calls `pipeline.process(sample, serial_error=serial_error)` and
> `bridge.dispatch(step.command)`. We add: (1) `bridge.init()`, (2) `sdk_error` feedback,
> (3) `bridge.tick(now)`, (4) `bridge.stop()` in finally.

- [ ] **Step 1: Generalize the bridge parameter type**

In `run_live_alx_simulation`, change the `bridge` parameter type from `MockMotionBridge`
to `MotionBridge` (the Protocol from Task 2). Import it at the top:

```python
from robot_follow.ports import MotionBridge
```

Change the signature and result class:

```python
def run_live_alx_simulation(
    config: LiveALXConfig,
    *,
    source: Optional[ALXLiveSource] = None,
    pipeline: Optional[FollowPipeline] = None,
    bridge: Optional[MotionBridge] = None,   # was: Optional[MockMotionBridge]
    emit: Emitter = print,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
    csv_file: Optional[TextIO] = None,
) -> LiveALXRunResult:
```

Also relax `LiveALXRunResult.bridge` typing to `MotionBridge`. The result's
convenience properties (`move_count`/`stop_count`/`last`) call
`MockMotionBridge`-specific methods that do not exist on `G1MotionBridge`.
Guard them with `hasattr` or `isinstance`:

```python
from robot_follow.sim.mock_motion_bridge import MockMotionBridge

@dataclass
class LiveALXRunResult:
    frames: list[LiveALXFrame] = field(default_factory=list)
    bridge: MotionBridge = field(default_factory=MockMotionBridge)

    @property
    def move_count(self) -> int:
        return self.bridge.move_count if isinstance(self.bridge, MockMotionBridge) else 0

    @property
    def stop_count(self) -> int:
        return self.bridge.stop_count if isinstance(self.bridge, MockMotionBridge) else 0

    @property
    def last(self) -> Optional[MotionCommand]:
        return self.bridge.last if isinstance(self.bridge, MockMotionBridge) else None
```

- [ ] **Step 2: Add `bridge.init()` and `sdk_error` feedback**

Call `bridge.init()` after constructing the bridge. Put both `bridge.init()` and
`source.start()` inside the same `try/finally`; otherwise a serial start failure
after real bridge initialization can skip the shutdown stop. In the loop body,
pass `sdk_error` into `pipeline.process`:

```python
    try:
        bridge.init()
        source.start()
        frame_index = 0
        while True:
            # ... existing time/limit checks ...
            snapshot = source.snapshot()
            sample, status, serial_error, alx_fps, alx_age_ms = sample_from_snapshot(
                snapshot, now=now, stale_timeout_s=config.stale_timeout_s,
            )
            # ... existing target_xy calculation ...
            step = pipeline.process(
                sample,
                serial_error=serial_error,
                sdk_error=bridge.sdk_error,   # NEW: feedback to safety manager
            )
            bridge.dispatch(step.command)
            bridge.tick(now)                  # NEW: watchdog tick each frame
            # ... rest of loop body unchanged ...
```

- [ ] **Step 3: Add `bridge.stop()` and `bridge.close()` in the `finally` block**

The current `finally` only calls `source.stop()`. For a real G1 bridge, the last
`Move(duration=1)` would keep the robot moving for up to 1 second after Ctrl-C if we do
not stop it. Add a best-effort stop and close:

```python
    finally:
        try:
            bridge.stop("shutdown")
        except Exception as e:
            emit(f"bridge shutdown stop failed: {e}")  # best-effort, do not mask
        try:
            bridge.close()
        except Exception as e:
            emit(f"bridge close failed: {e}")  # best-effort, do not mask
        source.stop()
        if csv_handle is not None:
            csv_handle.close()
```

- [ ] **Step 4: Wire `main()` to `create_motion_bridge`**

Update `main()` to build the bridge from CLI flags and pass it in:

```python
def main(argv: Optional[Iterable[str]] = None) -> None:
    args = _parse_args(argv)
    config = LiveALXConfig(
        port=args.port,
        baudrate=args.baudrate,
        hz=args.hz,
        stale_timeout_s=args.stale_timeout,
        duration_s=args.duration,
        frame_limit=args.frames,
        print_every_s=args.print_every,
        csv_path=args.csv,
    )
    bridge = create_motion_bridge(args)   # NEW: from Task 4
    run_live_alx_simulation(config, bridge=bridge)
```

> **Clock consistency (resolved):** `G1MotionBridge` now accepts an injectable
> `clock` (default `time.monotonic`, see Task 3). This loop also uses
> `time.monotonic`, and `bridge.tick(now)` receives the loop's monotonic `now`.
> The bridge's internal `_last_dispatch_time` is set via the same injected
> `time.monotonic`, so the watchdog elapsed-time math is consistent. If a future
> loop changes its clock, pass the SAME clock to the `G1MotionBridge` constructor.

- [ ] **Step 5: Test dry-run integration**

Run with the mock bridge (default):

```bash
python -m robot_follow.main --port /dev/ttyUSB0 --dry-run --duration 2
```

Expected: Logs show MotionCommand flow. `bridge.init()` runs (no-op for mock),
`bridge.tick(now)` runs each frame, `sdk_error` stays False, and `bridge.stop("shutdown")`
runs on exit. No real SDK calls.

- [ ] **Step 6: Commit integration changes**

Run:

```bash
git add src/robot_follow/main.py
git commit -m "feat: integrate MotionBridge into live ALX loop (sdk_error, tick, shutdown stop)"
```

---

### Task 8: Full Test Suite And Final Verification

**Files:**
- Test: full test suite

- [ ] **Step 1: Run full test suite**

Run:

```bash
pytest -q
```

Expected: All existing tests plus new G1 tests pass.

- [ ] **Step 2: Check final diff**

Run:

```bash
git diff --stat HEAD
git diff -- src/robot_follow/real/g1/
```

Expected:
- `g1/types.py` contains `G1BridgeConfig` and `BridgeState`.
- `g1/bridge.py` contains state machine, validation, delayed import.
- `g1/__init__.py` exports all three.
- CLI has safety switches.

- [ ] **Step 3: Check import behavior**

Run:

```bash
python -c "
from robot_follow.real.g1 import G1MotionBridge, G1BridgeConfig, BridgeState
from robot_follow.sim.mock_motion_bridge import MockMotionBridge
from robot_follow.core.types import MotionCommand
print('All imports OK without SDK')
"
```

Expected: PASS without `unitree_sdk2py`.

- [ ] **Step 4: Check git status**

Run:

```bash
git status --short
```

Expected: Implementation changes committed; only pre-existing unstaged changes may remain.

---

## Post-Implementation Notes

After this plan is complete:

1. **On Jetson**: Install `unitree_sdk2_python` from **GitHub source** (`pip install -e /path/to/unitree_sdk2_python`). Do NOT use PyPI — it lacks the `g1` module.
2. **Manual boot first**: G1 does NOT auto-stand. Follow `docs/alx_g1_verification_chain.md` **Stage 0** to manually prepare the robot before any SDK commands.
3. **Debug mode**: During development, press L2+R2 on the remote to enter debug mode (avoids conflicts with built-in motion control).
4. **Official example**: Run the official `g1_loco_client_example.py` (Stage 2) before using our bridge — it validates the SDK ↔ G1 link independently.
5. **Hardware verification**: Follow `docs/alx_g1_verification_chain.md` Stage 0 through Stage 9.
6. **Yaw calibration**: Adjust `yaw_sign` based on actual rotation behavior (Stage 8).
7. **Conservative defaults**: Use `max_abs_vx=0.05` for first real motion tests (Stage 5).
8. **Safety**: Always have emergency stop ready during first closed-loop runs.
9. **Return codes**: Watch for non-zero SDK return codes in logs — they indicate failures that do NOT raise Python exceptions. Common cause: G1 not in motion state (operator forgot Stage 0).
10. **FSM states**: The bridge does NOT do FSM transitions. If G1 leaves standing state (e.g., after a fall), the operator must manually recover it via remote before the bridge can command motion again.
