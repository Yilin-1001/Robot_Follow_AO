# G1 ALX-G100 Follow MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python 3 MVP that reads ALX-G100 positioning frames on Jetson, computes low-speed follow commands, and controls Unitree G1 through `unitree_sdk2_python`.

**Architecture:** The implementation is a small Python package with isolated modules for protocol parsing, serial IO, filtering/safety, follow control, and G1 motion bridging. Unit tests cover the parser, controller, and safety logic before hardware integration. The runtime CLI wires the modules together and guarantees `StopMove()` on abnormal states, `q + Enter`, or `Ctrl+C`.

**Tech Stack:** Python 3, `pyserial`, `numpy`, `unitree_sdk2_python`, `cyclonedds == 0.10.2`, `pytest`.

---

## File Structure

Create this structure:

```text
pyproject.toml
src/robot_follow/__init__.py
src/robot_follow/alx_g100_protocol.py
src/robot_follow/serial_reader.py
src/robot_follow/filtering.py
src/robot_follow/safety.py
src/robot_follow/follow_controller.py
src/robot_follow/g1_motion.py
src/robot_follow/main.py
tests/test_alx_g100_protocol.py
tests/test_safety.py
tests/test_follow_controller.py
```

Responsibilities:

```text
alx_g100_protocol.py
  Pure parser for ALX-G100 0x2001 and 0x2002 frames. No serial or robot dependency.

serial_reader.py
  pyserial wrapper that reads bytes and yields parsed positioning samples.

filtering.py
  Lightweight smoothing and jump detection for distance and azimuth. Jump detection output is consumed by the safety module.

safety.py
  Decides when to stop based on stale data, too-close distance, jump detection (from filtering module), parser/SDK failures, and manual stop.

follow_controller.py
  Converts valid filtered target state into vx, vy, omega.

g1_motion.py
  Wraps unitree_sdk2_python G1 LocoClient behind a small interface. Can be developed in parallel with filtering/safety/controller tasks.

main.py
  CLI entrypoint for Jetson runtime. Connects serial reader, controller, safety, and G1 bridge.
```

---

## Task 1: Project Scaffold

**Files:**
- Create: `pyproject.toml`
- Create: `src/robot_follow/__init__.py`

- [ ] **Step 1: Create package metadata**

Create `pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "robot-follow"
version = "0.1.0"
description = "ALX-G100 to Unitree G1 low-speed follow MVP"
requires-python = ">=3.8"
dependencies = [
  "pyserial>=3.5",
  "numpy>=1.23",
]

[project.optional-dependencies]
dev = [
  "pytest>=7.0",
]

[project.scripts]
robot-follow = "robot_follow.main:main"

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["src"]
```

Create `src/robot_follow/__init__.py`:

```python
"""ALX-G100 based low-speed follow MVP for Unitree G1."""
```

- [ ] **Step 2: Install local package in development mode**

Run:

```bash
python3 -m pip install -e ".[dev]"
```

Expected:

```text
Successfully installed robot-follow
```

- [ ] **Step 3: Run empty test suite**

Run:

```bash
pytest -q
```

Expected:

```text
no tests ran
```

- [ ] **Step 4: Commit scaffold**

Run:

```bash
git add pyproject.toml src/robot_follow/__init__.py
git commit -m "chore: scaffold Python follow package"
```

---

## Task 2: ALX-G100 Protocol Parser

**Files:**
- Create: `src/robot_follow/alx_g100_protocol.py`
- Create: `tests/test_alx_g100_protocol.py`

- [ ] **Step 1: Write parser tests**

Create `tests/test_alx_g100_protocol.py`:

```python
from robot_follow.alx_g100_protocol import (
    ALXG100FrameError,
    COMMAND_HEARTBEAT,
    COMMAND_POSITION,
    PositionSample,
    parse_frame,
)


def test_parse_position_frame_positive_angle():
    raw = bytes.fromhex(
        "FF FF FF FF 00 25 00 0B 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 19 "
        "00 12 FF CA 12 34 00 0B 00 00 00 00 1E"
    )

    sample = parse_frame(raw, timestamp=10.0)

    assert isinstance(sample, PositionSample)
    assert sample.command == COMMAND_POSITION
    assert sample.version_id == 0x0100
    assert sample.anchor_id == 0x0000AAA2
    assert sample.tag_id == 0x0000AAA1
    assert sample.distance_cm == 25
    assert sample.distance_m == 0.25
    assert sample.azimuth_deg == 18
    assert sample.elevation_deg == -54
    assert sample.tag_status == 0x1234
    assert sample.batch_sn == 0x000B
    assert sample.timestamp == 10.0


def test_parse_position_frame_negative_angle():
    raw = bytes.fromhex(
        "FF FF FF FF 00 25 00 27 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 62 "
        "FF D9 00 0D 12 34 00 27 00 00 00 00 69"
    )

    sample = parse_frame(raw, timestamp=11.0)

    assert sample.distance_cm == 98
    assert sample.distance_m == 0.98
    assert sample.azimuth_deg == -39
    assert sample.elevation_deg == 13
    assert sample.timestamp == 11.0


def test_parse_heartbeat_frame():
    raw = bytes.fromhex("FF FF FF FF 00 10 00 0B 20 02 01 01 00 00 AA A1")

    sample = parse_frame(raw, timestamp=12.0)

    assert sample.command == COMMAND_HEARTBEAT
    assert sample.anchor_id == 0x0000AAA1
    assert sample.timestamp == 12.0


def test_reject_bad_header():
    raw = bytes.fromhex("00 FF FF FF 00 10 00 0B 20 02 01 01 00 00 AA A1")

    try:
        parse_frame(raw, timestamp=0.0)
    except ALXG100FrameError as exc:
        assert "header" in str(exc)
    else:
        raise AssertionError("expected ALXG100FrameError")


def test_reject_bad_xor_for_position_frame():
    raw = bytearray.fromhex(
        "FF FF FF FF 00 25 00 0B 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 19 "
        "00 12 FF CA 12 34 00 0B 00 00 00 00 1E"
    )
    raw[-1] ^= 0x01

    try:
        parse_frame(bytes(raw), timestamp=0.0)
    except ALXG100FrameError as exc:
        assert "xor" in str(exc).lower()
    else:
        raise AssertionError("expected ALXG100FrameError")


def test_reject_bad_packet_length_for_position_frame():
    raw = bytearray.fromhex(
        "FF FF FF FF 00 25 00 0B 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 19 "
        "00 12 FF CA 12 34 00 0B 00 00 00 00 1E"
    )
    raw[5] = 0x24
    raw[-1] = 0
    for byte in raw[:-1]:
        raw[-1] ^= byte

    try:
        parse_frame(bytes(raw), timestamp=0.0)
    except ALXG100FrameError as exc:
        assert "packet length" in str(exc).lower()
    else:
        raise AssertionError("expected ALXG100FrameError")
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_alx_g100_protocol.py -q
```

Expected:

```text
ModuleNotFoundError: No module named 'robot_follow.alx_g100_protocol'
```

- [ ] **Step 3: Implement parser**

Create `src/robot_follow/alx_g100_protocol.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Union

FRAME_HEADER = b"\xFF\xFF\xFF\xFF"
COMMAND_POSITION = 0x2001
COMMAND_HEARTBEAT = 0x2002
POSITION_PACKET_LENGTH = 0x0025
HEARTBEAT_PACKET_LENGTH = 0x0010
POSITION_FRAME_LENGTH = 37
HEARTBEAT_FRAME_LENGTH = 16


class ALXG100FrameError(ValueError):
    """Raised when an ALX-G100 frame is malformed."""


@dataclass(frozen=True)
class PositionSample:
    command: int
    version_id: int
    anchor_id: int
    tag_id: int
    distance_cm: int
    azimuth_deg: int
    elevation_deg: int
    tag_status: int
    batch_sn: int
    timestamp: float

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0


@dataclass(frozen=True)
class HeartbeatSample:
    command: int
    version_id: int
    anchor_id: int
    timestamp: float


ParsedSample = Union[PositionSample, HeartbeatSample]


def _u16_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 2], byteorder="big", signed=False)


def _s16_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 2], byteorder="big", signed=True)


def _u32_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 4], byteorder="big", signed=False)


def _validate_xor(raw: bytes) -> None:
    expected = 0
    for byte in raw[:-1]:
        expected ^= byte
    actual = raw[-1]
    if expected != actual:
        raise ALXG100FrameError(f"invalid xor byte: expected 0x{expected:02X}, got 0x{actual:02X}")


def parse_frame(raw: bytes, timestamp: float) -> ParsedSample:
    if len(raw) < 10:
        raise ALXG100FrameError(f"frame too short: {len(raw)}")
    if raw[:4] != FRAME_HEADER:
        raise ALXG100FrameError("invalid frame header")

    packet_length = _u16_be(raw, 4)
    command = _u16_be(raw, 8)

    if command == COMMAND_POSITION:
        if len(raw) != POSITION_FRAME_LENGTH:
            raise ALXG100FrameError(f"invalid position frame length: {len(raw)}")
        if packet_length != POSITION_PACKET_LENGTH:
            raise ALXG100FrameError(f"invalid position packet length: 0x{packet_length:04X}")
        _validate_xor(raw)
        return PositionSample(
            command=command,
            version_id=_u16_be(raw, 10),
            anchor_id=_u32_be(raw, 12),
            tag_id=_u32_be(raw, 16),
            distance_cm=_u32_be(raw, 20),
            azimuth_deg=_s16_be(raw, 24),
            elevation_deg=_s16_be(raw, 26),
            tag_status=_u16_be(raw, 28),
            batch_sn=_u16_be(raw, 30),
            timestamp=timestamp,
        )

    if command == COMMAND_HEARTBEAT:
        if len(raw) != HEARTBEAT_FRAME_LENGTH:
            raise ALXG100FrameError(f"invalid heartbeat frame length: {len(raw)}")
        if packet_length != HEARTBEAT_PACKET_LENGTH:
            raise ALXG100FrameError(f"invalid heartbeat packet length: 0x{packet_length:04X}")
        # Heartbeat examples in the ALX-G100 spec do not include XorByte.
        return HeartbeatSample(
            command=command,
            version_id=_u16_be(raw, 10),
            anchor_id=_u32_be(raw, 12),
            timestamp=timestamp,
        )

    raise ALXG100FrameError(f"unsupported command: 0x{command:04X}")
```

- [ ] **Step 4: Run parser tests**

Run:

```bash
pytest tests/test_alx_g100_protocol.py -q
```

Expected:

```text
6 passed
```

- [ ] **Step 5: Commit parser**

Run:

```bash
git add src/robot_follow/alx_g100_protocol.py tests/test_alx_g100_protocol.py
git commit -m "feat: parse ALX-G100 positioning frames"
```

---

## Task 3: Serial Reader

**Files:**
- Create: `src/robot_follow/serial_reader.py`
- Modify: `pyproject.toml`

- [ ] **Step 1: Implement serial reader**

Create `src/robot_follow/serial_reader.py`:

```python
from __future__ import annotations

import time
from typing import Optional

import serial

from robot_follow.alx_g100_protocol import (
    FRAME_HEADER,
    HEARTBEAT_FRAME_LENGTH,
    POSITION_FRAME_LENGTH,
    ALXG100FrameError,
    ParsedSample,
    parse_frame,
)


class ALXG100SerialReader:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

    def open(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "ALXG100SerialReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def poll_sample(self) -> Optional[ParsedSample]:
        if self._serial is None:
            raise RuntimeError("serial reader is not open")

        chunk = self._serial.read(64)
        if chunk:
            self._buffer.extend(chunk)
        return self._try_pop_sample()

    def _try_pop_sample(self) -> Optional[ParsedSample]:
        header_index = self._buffer.find(FRAME_HEADER)
        if header_index < 0:
            # Keep a possible partial FF FF FF FF header across serial reads.
            if len(self._buffer) > len(FRAME_HEADER) - 1:
                del self._buffer[: -(len(FRAME_HEADER) - 1)]
            return None
        if header_index > 0:
            del self._buffer[:header_index]

        if len(self._buffer) < 10:
            return None

        command = int.from_bytes(self._buffer[8:10], byteorder="big", signed=False)
        if command == 0x2001:
            frame_len = POSITION_FRAME_LENGTH
        elif command == 0x2002:
            frame_len = HEARTBEAT_FRAME_LENGTH
        else:
            del self._buffer[:4]
            return None

        if len(self._buffer) < frame_len:
            return None

        raw = bytes(self._buffer[:frame_len])
        del self._buffer[:frame_len]
        try:
            return parse_frame(raw, timestamp=time.monotonic())
        except ALXG100FrameError:
            return None
```

- [ ] **Step 2: Verify import**

Run:

```bash
python3 -c "from robot_follow.serial_reader import ALXG100SerialReader; print(ALXG100SerialReader)"
```

Expected:

```text
<class 'robot_follow.serial_reader.ALXG100SerialReader'>
```

- [ ] **Step 3: Commit serial reader**

Run:

```bash
git add src/robot_follow/serial_reader.py pyproject.toml
git commit -m "feat: add ALX-G100 serial reader"
```

---

## Task 4: Filtering

**Files:**
- Create: `src/robot_follow/filtering.py`

> **Dependency note:** This task MUST be completed before Task 5 (Safety Manager), because the safety module consumes `distance_jump_m` and `angle_jump_deg` produced here.

- [ ] **Step 1: Implement target state filter**

Create `src/robot_follow/filtering.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class FilteredTarget:
    distance_m: float
    azimuth_deg: float
    distance_jump_m: float
    angle_jump_deg: float


class TargetFilter:
    def __init__(self, alpha: float = 0.35):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self._distance_m: Optional[float] = None
        self._azimuth_deg: Optional[float] = None

    def update(self, distance_m: float, azimuth_deg: float) -> FilteredTarget:
        if self._distance_m is None or self._azimuth_deg is None:
            self._distance_m = distance_m
            self._azimuth_deg = azimuth_deg
            return FilteredTarget(distance_m, azimuth_deg, 0.0, 0.0)

        distance_jump = distance_m - self._distance_m
        angle_jump = azimuth_deg - self._azimuth_deg

        self._distance_m = self.alpha * distance_m + (1.0 - self.alpha) * self._distance_m
        self._azimuth_deg = self.alpha * azimuth_deg + (1.0 - self.alpha) * self._azimuth_deg

        return FilteredTarget(
            distance_m=self._distance_m,
            azimuth_deg=self._azimuth_deg,
            distance_jump_m=distance_jump,
            angle_jump_deg=angle_jump,
        )
```

- [ ] **Step 2: Verify import**

Run:

```bash
python3 -c "from robot_follow.filtering import TargetFilter; print(TargetFilter().update(1.2, 3.0))"
```

Expected:

```text
FilteredTarget(distance_m=1.2, azimuth_deg=3.0, distance_jump_m=0.0, angle_jump_deg=0.0)
```

- [ ] **Step 3: Run all tests**

Run:

```bash
pytest -q
```

Expected:

```text
6 passed
```

- [ ] **Step 4: Commit filtering**

Run:

```bash
git add src/robot_follow/filtering.py
git commit -m "feat: add target filtering"
```

---

## Task 5: Safety Manager

**Files:**
- Create: `src/robot_follow/safety.py`
- Create: `tests/test_safety.py`

> **Dependency note:** Jump detection values (`distance_jump_m`, `angle_jump_deg`) come from the filtering module (Task 4).

- [ ] **Step 1: Write safety tests**

Create `tests/test_safety.py`:

```python
from robot_follow.safety import SafetyConfig, SafetyManager


def test_stale_data_stops_robot():
    safety = SafetyManager(SafetyConfig(data_timeout_s=0.5))
    assert safety.should_stop(now=10.6, last_sample_time=10.0, distance_m=1.5, distance_jump_m=0.0, angle_jump_deg=0.0) is True


def test_too_close_stops_robot():
    safety = SafetyManager(SafetyConfig(min_safe_distance_m=0.75))
    assert safety.should_stop(now=10.1, last_sample_time=10.0, distance_m=0.7, distance_jump_m=0.0, angle_jump_deg=0.0) is True


def test_large_distance_jump_stops_robot():
    safety = SafetyManager(SafetyConfig(max_distance_jump_m=0.5))
    assert safety.should_stop(now=10.1, last_sample_time=10.0, distance_m=1.5, distance_jump_m=0.6, angle_jump_deg=0.0) is True


def test_valid_data_allows_motion():
    safety = SafetyManager(SafetyConfig())
    assert safety.should_stop(now=10.1, last_sample_time=10.0, distance_m=1.5, distance_jump_m=0.1, angle_jump_deg=5.0) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_safety.py -q
```

Expected:

```text
ModuleNotFoundError: No module named 'robot_follow.safety'
```

- [ ] **Step 3: Implement safety manager**

Create `src/robot_follow/safety.py`:

```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyConfig:
    data_timeout_s: float = 0.5
    min_safe_distance_m: float = 0.75
    max_distance_jump_m: float = 0.5
    max_angle_jump_deg: float = 45.0


class SafetyManager:
    def __init__(self, config: SafetyConfig):
        self.config = config

    def should_stop(
        self,
        now: float,
        last_sample_time: float,
        distance_m: float,
        distance_jump_m: float,
        angle_jump_deg: float,
    ) -> bool:
        cfg = self.config
        if now - last_sample_time > cfg.data_timeout_s:
            return True
        if distance_m < cfg.min_safe_distance_m:
            return True
        if abs(distance_jump_m) > cfg.max_distance_jump_m:
            return True
        if abs(angle_jump_deg) > cfg.max_angle_jump_deg:
            return True
        return False
```

- [ ] **Step 4: Run safety tests**

Run:

```bash
pytest tests/test_safety.py -q
```

Expected:

```text
4 passed
```

- [ ] **Step 5: Commit safety manager**

Run:

```bash
git add src/robot_follow/safety.py tests/test_safety.py
git commit -m "feat: add follow safety manager"
```

---

## Task 6: Follow Controller

**Files:**
- Create: `src/robot_follow/follow_controller.py`
- Create: `tests/test_follow_controller.py`

> **Dependency note:** Consumes filtered distance and azimuth from the filtering module (Task 4).

- [ ] **Step 1: Write controller tests**

Create `tests/test_follow_controller.py`:

```python
from robot_follow.follow_controller import FollowConfig, FollowController


def test_stop_when_too_close():
    controller = FollowController(FollowConfig())
    command = controller.compute(distance_m=0.6, azimuth_deg=0.0)
    assert command.vx == 0.0
    assert command.vy == 0.0
    assert command.omega == 0.0
    assert command.reason == "too_close"


def test_rotate_in_place_when_angle_large():
    controller = FollowController(FollowConfig(max_omega_rad_s=0.3))
    command = controller.compute(distance_m=1.6, azimuth_deg=30.0)
    assert command.vx == 0.0
    assert command.vy == 0.0
    assert command.omega == 0.3
    assert command.reason == "aligning"


def test_move_forward_when_centered_and_far():
    controller = FollowController(FollowConfig(target_distance_m=1.3, max_vx_m_s=0.2))
    command = controller.compute(distance_m=1.8, azimuth_deg=2.0)
    assert command.vx == 0.2
    assert command.vy == 0.0
    assert command.omega == 0.0
    assert command.reason == "tracking"


def test_low_speed_with_medium_angle():
    controller = FollowController(FollowConfig())
    command = controller.compute(distance_m=1.8, azimuth_deg=-10.0)
    assert command.vx > 0.0
    assert command.vx < 0.2
    assert command.omega < 0.0
    assert command.reason == "tracking_turning"
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_follow_controller.py -q
```

Expected:

```text
ModuleNotFoundError: No module named 'robot_follow.follow_controller'
```

- [ ] **Step 3: Implement controller**

Create `src/robot_follow/follow_controller.py`:

```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    omega: float
    reason: str


@dataclass(frozen=True)
class FollowConfig:
    target_distance_m: float = 1.3
    min_safe_distance_m: float = 0.75
    max_vx_m_s: float = 0.2
    max_omega_rad_s: float = 0.3
    angle_deadband_deg: float = 5.0
    large_angle_deg: float = 20.0
    distance_gain: float = 0.6
    angle_gain: float = 0.02


class FollowController:
    def __init__(self, config: FollowConfig):
        self.config = config

    def compute(self, distance_m: float, azimuth_deg: float) -> VelocityCommand:
        cfg = self.config

        if distance_m <= cfg.min_safe_distance_m:
            return VelocityCommand(0.0, 0.0, 0.0, "too_close")

        abs_angle = abs(azimuth_deg)
        omega = self._clamp(azimuth_deg * cfg.angle_gain, -cfg.max_omega_rad_s, cfg.max_omega_rad_s)

        if abs_angle > cfg.large_angle_deg:
            return VelocityCommand(0.0, 0.0, omega, "aligning")

        distance_error = max(0.0, distance_m - cfg.target_distance_m)
        vx = self._clamp(distance_error * cfg.distance_gain, 0.0, cfg.max_vx_m_s)

        if abs_angle <= cfg.angle_deadband_deg:
            return VelocityCommand(vx, 0.0, 0.0, "tracking")

        return VelocityCommand(vx * 0.5, 0.0, omega, "tracking_turning")

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))
```

- [ ] **Step 4: Run controller tests**

Run:

```bash
pytest tests/test_follow_controller.py -q
```

Expected:

```text
4 passed
```

- [ ] **Step 5: Commit controller**

Run:

```bash
git add src/robot_follow/follow_controller.py tests/test_follow_controller.py
git commit -m "feat: add follow controller"
```

---

## Task 7: G1 Motion Bridge (parallelizable with Tasks 4-6)

**Files:**
- Create: `src/robot_follow/g1_motion.py`

> **Parallelization note:** This module only depends on `unitree_sdk2_python` and has no dependency on the ALX-G100 data pipeline. It can be developed in parallel with Tasks 4-6 to surface SDK integration issues earlier.

- [ ] **Step 1: Implement motion bridge**

Create `src/robot_follow/g1_motion.py`:

```python
from __future__ import annotations


class G1MotionBridge:
    def __init__(self, network_interface: str, timeout_s: float = 10.0):
        self.network_interface = network_interface
        self.timeout_s = timeout_s
        self._client = None

    def init(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        ChannelFactoryInitialize(0, self.network_interface)
        client = LocoClient()
        client.SetTimeout(self.timeout_s)
        client.Init()
        self._client = client

    def move(self, vx: float, vy: float, omega: float) -> None:
        if self._client is None:
            raise RuntimeError("G1MotionBridge is not initialized")
        self._client.Move(vx, vy, omega)

    def stop(self) -> None:
        if self._client is not None:
            self._client.StopMove()
```

- [ ] **Step 2: Verify import without SDK import side effects**

Run:

```bash
python3 -c "from robot_follow.g1_motion import G1MotionBridge; print(G1MotionBridge)"
```

Expected:

```text
<class 'robot_follow.g1_motion.G1MotionBridge'>
```

- [ ] **Step 3: Commit G1 bridge**

Run:

```bash
git add src/robot_follow/g1_motion.py
git commit -m "feat: add G1 motion bridge"
```

---

## Task 8: Runtime CLI

**Files:**
- Create: `src/robot_follow/main.py`

- [ ] **Step 1: Implement runtime entrypoint**

Create `src/robot_follow/main.py`:

```python
from __future__ import annotations

import argparse
import select
import signal
import sys
import time

from robot_follow.alx_g100_protocol import PositionSample
from robot_follow.filtering import TargetFilter
from robot_follow.follow_controller import FollowConfig, FollowController
from robot_follow.g1_motion import G1MotionBridge
from robot_follow.safety import SafetyConfig, SafetyManager
from robot_follow.serial_reader import ALXG100SerialReader


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ALX-G100 low-speed follow control for Unitree G1")
    parser.add_argument("--serial-port", required=True, help="ALX-G100 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--net-iface", required=True, help="Network interface connected to G1, e.g. enp2s0")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without controlling G1")
    return parser.parse_args()


def key_pressed() -> bool:
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return False
    return sys.stdin.readline().strip().lower() == "q"


def main() -> int:
    args = parse_args()
    stop_requested = False

    def handle_signal(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    controller = FollowController(FollowConfig())
    safety = SafetyManager(SafetyConfig())
    target_filter = TargetFilter()
    motion = None if args.dry_run else G1MotionBridge(args.net_iface)

    try:
        if motion is not None:
            motion.init()

        last_position_time = None
        filtered = None

        with ALXG100SerialReader(args.serial_port) as reader:
            while not stop_requested:
                if key_pressed():
                    break

                sample = reader.poll_sample()
                if isinstance(sample, PositionSample):
                    filtered = target_filter.update(sample.distance_m, sample.azimuth_deg)
                    last_position_time = sample.timestamp
                    print(f"tag_status=0x{sample.tag_status:04X}", flush=True)

                now = time.monotonic()
                if filtered is None or last_position_time is None:
                    time.sleep(0.02)
                    continue

                if safety.should_stop(
                    now=now,
                    last_sample_time=last_position_time,
                    distance_m=filtered.distance_m,
                    distance_jump_m=filtered.distance_jump_m,
                    angle_jump_deg=filtered.angle_jump_deg,
                ):
                    if motion is not None:
                        motion.stop()
                    print("STOP safety", flush=True)
                    time.sleep(0.02)
                    continue

                command = controller.compute(filtered.distance_m, filtered.azimuth_deg)
                print(
                    f"distance={filtered.distance_m:.2f}m "
                    f"azimuth={filtered.azimuth_deg:.1f}deg "
                    f"vx={command.vx:.2f} omega={command.omega:.2f} reason={command.reason}",
                    flush=True,
                )
                if motion is not None:
                    motion.move(command.vx, command.vy, command.omega)
                time.sleep(0.02)
    finally:
        if motion is not None:
            motion.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Verify CLI help**

Run:

```bash
python3 -m robot_follow.main --help
```

Expected:

```text
usage: main.py [-h] --serial-port SERIAL_PORT --net-iface NET_IFACE [--dry-run]
```

- [ ] **Step 3: Run all tests**

Run:

```bash
pytest -q
```

Expected:

```text
14 passed
```

- [ ] **Step 4: Commit CLI**

Run:

```bash
git add src/robot_follow/main.py
git commit -m "feat: add follow runtime CLI"
```

---

## Task 9: Hardware Bring-Up on Jetson

**Files:**
- No repository file changes required unless observed port names or commands need to be recorded.

- [ ] **Step 1: Identify ALX-G100 serial port**

Run on Jetson after plugging in ALX-G100 USB/USB-TTL:

```bash
dmesg | tail -n 40
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Expected:

```text
/dev/ttyUSB0
```

- [ ] **Step 2: Verify dry-run serial parsing**

> **Parallelization note:** This step (ALX-G100 data pipeline) and Step 4 (G1 SDK verification) are independent and can be run in parallel.

Run:

```bash
robot-follow --serial-port /dev/ttyUSB0 --net-iface enp2s0 --dry-run
```

Expected while moving the hand ring:

```text
distance=1.25m azimuth=...
```

- [ ] **Step 3: Confirm azimuth sign**

Move the hand ring to the robot's left and right.

Record:

```text
left side azimuth sign: positive or negative
right side azimuth sign: positive or negative
```

If turning direction is reversed during closed-loop testing, change `angle_gain` sign in `FollowConfig`:

```python
angle_gain: float = -0.02
```

- [ ] **Step 4: Verify Python G1 SDK example before closed-loop control**

> **Parallelization note:** This step (G1 SDK verification) and Steps 2-3 (ALX-G100 data verification) are independent and can be run in parallel.

Run the official G1 high-level example from `unitree_sdk2_python`:

```bash
python3 example/g1/high_level/g1_loco_client_example.py enp2s0
```

Expected:

```text
Input "list" to list all test option ...
```

Run only low-speed move/rotate test options in an empty indoor area.

- [ ] **Step 5: Run closed-loop MVP at low speed**

Run:

```bash
robot-follow --serial-port /dev/ttyUSB0 --net-iface enp2s0
```

Expected:

```text
G1 rotates to face the hand ring and moves forward only at low speed when target distance is above the configured target.
```

- [ ] **Step 6: Verify software stop**

Run while robot is moving:

```text
Type q and press Enter in the SSH terminal
```

Expected:

```text
G1 receives StopMove and stops.
```

Then run again and press `Ctrl+C`.

Expected:

```text
G1 receives StopMove and exits safely.
```

---

## Self-Review

Spec coverage:

```text
ALX-G100 protocol and cm/degree units: Task 2.
Jetson direct serial path: Task 3 and Task 9.
Python 3 language decision: Task 1, Task 7, Task 8.
unitree_sdk2_python G1 control: Task 7 and Task 9.
Follow behavior facing the person: Task 6 and Task 8.
Safety stop on stale data, too close, jumps, SDK failures, q + Enter, Ctrl+C: Task 5 and Task 8.
Jump detection for distance and azimuth: Task 4.
No autonomous obstacle avoidance: no task implements avoidance, matching the spec boundary.
```

Placeholder scan:

```text
No unresolved placeholders are present.
All code-bearing tasks include concrete code blocks.
All hardware tasks include concrete commands and expected outputs.
```

Type consistency:

```text
PositionSample.distance_m, PositionSample.azimuth_deg, VelocityCommand.vx/vy/omega, SafetyManager.should_stop, and G1MotionBridge.move/stop are consistent across tasks.
```
