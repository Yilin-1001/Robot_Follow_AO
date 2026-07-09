# Robot Follow AO Radar Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a lightweight `Robot Follow AO Radar` runtime that reuses the existing Robot Follow serial parsing/follow pipeline, starts Livox/FAST-LIO from fresh ALX position frames, filters Robot Follow commands through radar AO avoidance, and outputs directly to G1.

**Architecture:** Keep Robot Follow core as the upstream command producer and add a separate `robot_follow.ao` package for AO config, latest-only envelopes, radar service lifecycle, FAST-LIO point cloud source, copied AO radar/avoidance logic, and the AO runtime. The same AO runtime owns the single USB serial reader through `ALXLiveSource`; no supervisor process and no second serial parser are introduced. All live data paths are latest-only, age-gated, and stop-on-stale.

**Tech Stack:** Python dataclasses, existing `robot_follow` package, existing YAML loader pattern, pytest, ROS2/rclpy lazy imports for FAST-LIO, subprocess process management for Livox/FAST-LIO, existing G1 bridge.

---

## Boundaries Locked By This Plan

- Do not change `src/robot_follow/core/types.py::MotionCommand`.
- Do not add a new wristband parser. Reuse `ALXG100SerialReader` position/OFF frame behavior.
- Do not add a supervisor process, socket relay, or pipe relay.
- Do not add `combined_source.py`.
- Do not modify old AO source in `/home/yilin/Code/Robot_FollowAO_Radar`; copy the needed modules into this repo.
- `RadarServiceManager` only manages Livox and FAST-LIO. It never sends G1 commands.
- G1 stop/dispatch is owned by AO runtime and `G1MotionBridge`.
- Real G1 output must require explicit config/CLI confirmation. Missing confirmation raises an error instead of silently using mock output.

## File Structure

### Minimal Changes To Existing Robot Follow Files

- Modify `src/robot_follow/serial_reader.py`
  - Add a structured `WristbandGateState` dataclass.
  - Add `ALXG100SerialReader.wristband_state()`.
  - Record `updated_at` when current tested ON/OFF parsing fires.
- Modify `src/robot_follow/real/alx_source.py`
  - Add `ALXLiveSource.wristband_state()` by copying the current reader state into a cached, thread-safe field.
  - Do not change `ALXSnapshot` fields.
- Modify `pyproject.toml`
  - Add console script `robot-follow-ao-radar = robot_follow.ao.live.cli:main`.
- Modify `configs/jetson_g1_safe.yaml`
  - Add an `ao_radar` section while preserving existing G1 safety values.

### New AO Package Files

- Create `src/robot_follow/ao/__init__.py`
- Create `src/robot_follow/ao/config.py`
  - Dataclasses for AO runtime, radar services, FAST-LIO, latency budgets, and G1 output guard.
- Create `src/robot_follow/ao/live/__init__.py`
- Create `src/robot_follow/ao/live/command_envelope.py`
  - `CommandEnvelope` wrapper around existing `MotionCommand`.
- Create `src/robot_follow/ao/live/radar_service_manager.py`
  - Start/stop Livox and FAST-LIO based on fresh position-frame activity.
- Create `src/robot_follow/ao/live/fast_lio_source.py`
  - Latest-only `/cloud_registered_body` point cloud source with lazy ROS2 imports.
- Create `src/robot_follow/ao/live/runtime.py`
  - Single main control loop: ALX source -> Robot Follow pipeline -> AO -> G1.
- Create `src/robot_follow/ao/live/cli.py`
  - CLI entrypoint for `robot-follow-ao-radar`.
- Create `src/robot_follow/ao/logging/__init__.py`
- Create `src/robot_follow/ao/logging/csv_logger.py`
  - Per-frame timing/age/decision CSV logs.
- Create `src/robot_follow/ao/radar/*`
  - Copy/adapt old AO radar modules.
- Create `src/robot_follow/ao/avoidance/*`
  - Copy/adapt old AO avoidance modules.
- Create `src/robot_follow/ao/core/*`
  - Copy only AO-specific geometry/types that are not already covered by Robot Follow core.

### Test Files

- Modify `tests/test_serial_reader.py`
- Modify `tests/real/test_alx_source.py`
- Create `tests/ao/test_config.py`
- Create `tests/ao/live/test_command_envelope.py`
- Create `tests/ao/live/test_radar_service_manager.py`
- Create `tests/ao/live/test_fast_lio_source.py`
- Create `tests/ao/live/test_runtime.py`
- Create `tests/ao/avoidance/test_planner_adapter.py`
- Create `tests/ao/logging/test_csv_logger.py`

---

## Task 1: Add AO Config Dataclasses And YAML Loading

**Files:**
- Create: `src/robot_follow/ao/__init__.py`
- Create: `src/robot_follow/ao/config.py`
- Create: `tests/ao/test_config.py`
- Modify: `configs/jetson_g1_safe.yaml`

- [x] **Step 1: Write config tests**

Create `tests/ao/test_config.py`:

```python
from pathlib import Path

import pytest

from robot_follow.ao.config import AORadarConfig, load_ao_radar_config


def test_loads_default_config_values_from_empty_mapping(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    config = load_ao_radar_config(path)

    assert isinstance(config, AORadarConfig)
    assert config.alx.port == "/dev/ttyUSB0"
    assert config.alx.stale_timeout_s == 0.5
    assert config.radar.idle_stop_s == 5.0
    assert config.latency.max_command_age_s == 0.2
    assert config.latency.max_cloud_age_s == 0.5
    assert config.output.require_real_g1_confirmation is True


def test_rejects_unknown_ao_section_key(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        "ao_radar:\n"
        "  latency:\n"
        "    not_a_real_key: 1\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="unknown keys in ao_radar.latency"):
        load_ao_radar_config(path)


def test_loads_runtime_overrides(tmp_path: Path) -> None:
    path = tmp_path / "ao.yaml"
    path.write_text(
        "ao_radar:\n"
        "  alx:\n"
        "    port: /dev/ttyUSB9\n"
        "    stale_timeout_s: 0.35\n"
        "  radar:\n"
        "    idle_stop_s: 5.0\n"
        "    livox_start_command: ros2 launch livox_ros_driver2 msg_MID360_launch.py\n"
        "    fast_lio_start_command: ros2 launch fast_lio mapping.launch.py\n"
        "  latency:\n"
        "    ao_output_loop_hz: 20.0\n"
        "    max_command_age_s: 0.2\n"
        "    max_cloud_age_s: 0.5\n"
        "    max_ao_process_ms: 20.0\n"
        "    max_g1_dispatch_ms: 30.0\n"
        "    warn_loop_dt_ms: 75.0\n"
        "    hard_loop_dt_ms: 150.0\n"
        "  output:\n"
        "    mode: g1\n"
        "    confirm_real_g1: true\n",
        encoding="utf-8",
    )

    config = load_ao_radar_config(path)

    assert config.alx.port == "/dev/ttyUSB9"
    assert config.alx.stale_timeout_s == 0.35
    assert config.radar.idle_stop_s == 5.0
    assert config.radar.livox_start_command.startswith("ros2 launch livox")
    assert config.latency.ao_output_loop_hz == 20.0
    assert config.output.mode == "g1"
    assert config.output.confirm_real_g1 is True
```

- [x] **Step 2: Run the failing config tests**

Run:

```bash
pytest tests/ao/test_config.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'robot_follow.ao'`.

- [x] **Step 3: Add config implementation**

Create `src/robot_follow/ao/__init__.py`:

```python
"""AO radar extension package for Robot Follow."""
```

Create `src/robot_follow/ao/config.py`:

```python
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeVar

from robot_follow.main import ConfigMapping, load_yaml_config


@dataclass(frozen=True)
class AOALXConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    stale_timeout_s: float = 0.5


@dataclass(frozen=True)
class AORadarServiceConfig:
    idle_stop_s: float = 5.0
    livox_start_command: str = ""
    fast_lio_start_command: str = ""
    stop_timeout_s: float = 3.0
    ready_timeout_s: float = 10.0


@dataclass(frozen=True)
class AOLatencyConfig:
    ao_output_loop_hz: float = 20.0
    max_command_age_s: float = 0.2
    max_cloud_age_s: float = 0.5
    max_ao_process_ms: float = 20.0
    max_g1_dispatch_ms: float = 30.0
    warn_loop_dt_ms: float = 75.0
    hard_loop_dt_ms: float = 150.0


@dataclass(frozen=True)
class AOOutputConfig:
    mode: str = "g1"
    require_real_g1_confirmation: bool = True
    confirm_real_g1: bool = False


@dataclass(frozen=True)
class AORadarConfig:
    alx: AOALXConfig = field(default_factory=AOALXConfig)
    radar: AORadarServiceConfig = field(default_factory=AORadarServiceConfig)
    latency: AOLatencyConfig = field(default_factory=AOLatencyConfig)
    output: AOOutputConfig = field(default_factory=AOOutputConfig)


T = TypeVar("T")


def load_ao_radar_config(path: Path) -> AORadarConfig:
    raw = load_yaml_config(path)
    section = _mapping_section(raw, "ao_radar")
    return AORadarConfig(
        alx=_dataclass_from_section(section, "alx", AOALXConfig),
        radar=_dataclass_from_section(section, "radar", AORadarServiceConfig),
        latency=_dataclass_from_section(section, "latency", AOLatencyConfig),
        output=_dataclass_from_section(section, "output", AOOutputConfig),
    )


def _mapping_section(config: ConfigMapping, key: str) -> ConfigMapping:
    value = config.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"config section '{key}' must be a mapping")
    return value


def _dataclass_from_section(config: ConfigMapping, key: str, cls: type[T]) -> T:
    section = _mapping_section(config, key)
    allowed = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
    unknown = sorted(set(section) - allowed)
    if unknown:
        raise ValueError(f"unknown keys in ao_radar.{key}: {', '.join(unknown)}")
    kwargs: dict[str, Any] = {name: section[name] for name in section}
    return cls(**kwargs)
```

- [x] **Step 4: Add combined config section**

Append this section to `configs/jetson_g1_safe.yaml` without changing existing Robot Follow values:

```yaml
ao_radar:
  alx:
    port: /dev/ttyUSB0
    baudrate: 115200
    stale_timeout_s: 0.5
  radar:
    idle_stop_s: 5.0
    livox_start_command: ""
    fast_lio_start_command: ""
    stop_timeout_s: 3.0
    ready_timeout_s: 10.0
  latency:
    ao_output_loop_hz: 20.0
    max_command_age_s: 0.2
    max_cloud_age_s: 0.5
    max_ao_process_ms: 20.0
    max_g1_dispatch_ms: 30.0
    warn_loop_dt_ms: 75.0
    hard_loop_dt_ms: 150.0
  output:
    mode: g1
    require_real_g1_confirmation: true
    confirm_real_g1: false
```

- [x] **Step 5: Run config tests**

Run:

```bash
pytest tests/ao/test_config.py -v
```

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git add src/robot_follow/ao/__init__.py src/robot_follow/ao/config.py tests/ao/test_config.py configs/jetson_g1_safe.yaml
git commit -m "feat: add ao radar runtime config"
```

---

## Task 2: Expose Existing Wristband Gate State Without New Parsing

**Files:**
- Modify: `src/robot_follow/serial_reader.py`
- Modify: `src/robot_follow/real/alx_source.py`
- Modify: `tests/test_serial_reader.py`
- Modify: `tests/real/test_alx_source.py`

- [x] **Step 1: Add failing serial reader tests**

Append to `tests/test_serial_reader.py`:

```python
def test_wristband_state_starts_unknown() -> None:
    reader = ALXG100SerialReader("/dev/null")

    state = reader.wristband_state()

    assert state.enabled is None
    assert state.updated_at is None


def test_position_frame_updates_wristband_state_to_on() -> None:
    reader = ALXG100SerialReader("/dev/ttyUSB0")
    reader._buffer.extend(POSITION_FRAME)

    sample = reader._try_pop_sample()
    state = reader.wristband_state()

    assert sample is not None
    assert state.enabled is True
    assert state.updated_at is not None


def test_off_frame_updates_wristband_state_to_off() -> None:
    reader = ALXG100SerialReader("/dev/ttyUSB0")
    reader._buffer.extend(
        bytes.fromhex("FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA")
    )

    assert reader._try_pop_sample() is None
    state = reader.wristband_state()

    assert state.enabled is False
    assert state.updated_at is not None
```

- [x] **Step 2: Run serial tests and verify failure**

Run:

```bash
pytest tests/test_serial_reader.py -v
```

Expected: FAIL with `AttributeError: 'ALXG100SerialReader' object has no attribute 'wristband_state'`.

- [x] **Step 3: Implement reader state**

Modify `src/robot_follow/serial_reader.py`:

```python
from dataclasses import dataclass
```

Add above `class ALXG100SerialReader`:

```python
@dataclass(frozen=True)
class WristbandGateState:
    enabled: Optional[bool]
    updated_at: Optional[float]
```

In `__init__`, add:

```python
self._wristband_updated_at: Optional[float] = None
```

Add method:

```python
def wristband_state(self) -> WristbandGateState:
    return WristbandGateState(
        enabled=self._wristband_enabled,
        updated_at=self._wristband_updated_at,
    )
```

At the end of `_emit_wristband_on`, set:

```python
self._wristband_updated_at = now
```

At the start of `_emit_wristband_off`, set:

```python
now = time.monotonic()
```

At the end of `_emit_wristband_off`, set:

```python
self._wristband_updated_at = now
```

- [x] **Step 4: Add failing ALXLiveSource gate-state tests**

Append to `tests/real/test_alx_source.py`:

```python
from robot_follow.serial_reader import WristbandGateState


class GateStateReader:
    def __init__(self, *args, **kwargs) -> None:
        self.closed = False
        self._state = WristbandGateState(enabled=None, updated_at=None)

    def open(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def read_samples(self):
        self._state = WristbandGateState(enabled=True, updated_at=123.0)
        return iter(())

    def wristband_state(self) -> WristbandGateState:
        return self._state


def test_live_source_exposes_cached_wristband_state() -> None:
    source = ALXLiveSource(reader_factory=GateStateReader, reconnect_delay_s=0.01)

    source.start()
    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            state = source.wristband_state()
            if state.enabled is True:
                break
            time.sleep(0.01)
    finally:
        source.stop()

    assert source.wristband_state().enabled is True
    assert source.wristband_state().updated_at == 123.0
```

- [x] **Step 5: Run ALX source test and verify failure**

Run:

```bash
pytest tests/real/test_alx_source.py::test_live_source_exposes_cached_wristband_state -v
```

Expected: FAIL with `AttributeError: 'ALXLiveSource' object has no attribute 'wristband_state'`.

- [x] **Step 6: Implement ALXLiveSource cached state**

Modify `src/robot_follow/real/alx_source.py`:

```python
from robot_follow.serial_reader import ALXG100SerialReader, WristbandGateState
```

In `__init__`, add:

```python
self._wristband_state = WristbandGateState(enabled=None, updated_at=None)
```

Add method:

```python
def wristband_state(self) -> WristbandGateState:
    with self._lock:
        return self._wristband_state
```

Inside `_run`, after each parsed sample is received and before sample type checks, add:

```python
with self._lock:
    self._wristband_state = reader.wristband_state()
```

Inside `finally`, before closing the reader, add:

```python
try:
    state = reader.wristband_state()
except Exception:
    state = WristbandGateState(enabled=False, updated_at=time.monotonic())
with self._lock:
    self._wristband_state = state
```

- [x] **Step 7: Run parser/source tests**

Run:

```bash
pytest tests/test_serial_reader.py tests/real/test_alx_source.py -v
```

Expected: PASS.

- [x] **Step 8: Commit**

```bash
git add src/robot_follow/serial_reader.py src/robot_follow/real/alx_source.py tests/test_serial_reader.py tests/real/test_alx_source.py
git commit -m "feat: expose alx wristband gate state"
```

---

## Task 3: Add Latest-Only CommandEnvelope

**Files:**
- Create: `src/robot_follow/ao/live/__init__.py`
- Create: `src/robot_follow/ao/live/command_envelope.py`
- Create: `tests/ao/live/test_command_envelope.py`

- [x] **Step 1: Write envelope tests**

Create `tests/ao/live/test_command_envelope.py`:

```python
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand


def test_command_envelope_reports_age() -> None:
    command = MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="follow")
    envelope = CommandEnvelope(command=command, generated_at=10.0, source="robot_follow")

    assert envelope.age_s(now=10.125) == 0.125


def test_stale_envelope_detection() -> None:
    command = MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.2, reason="follow")
    envelope = CommandEnvelope(command=command, generated_at=10.0, source="robot_follow")

    assert envelope.is_stale(now=10.25, max_age_s=0.2) is True
    assert envelope.is_stale(now=10.19, max_age_s=0.2) is False


def test_stop_factory_uses_robot_follow_motion_command() -> None:
    envelope = CommandEnvelope.stop(reason="radar_not_ready", generated_at=20.0)

    assert envelope.command.kind == "stop"
    assert envelope.command.reason == "radar_not_ready"
    assert envelope.generated_at == 20.0
    assert envelope.source == "ao_runtime"
```

- [x] **Step 2: Run failing envelope tests**

Run:

```bash
pytest tests/ao/live/test_command_envelope.py -v
```

Expected: FAIL with `ModuleNotFoundError` or missing `CommandEnvelope`.

- [x] **Step 3: Implement envelope**

Create `src/robot_follow/ao/live/__init__.py`:

```python
"""Live AO radar runtime components."""
```

Create `src/robot_follow/ao/live/command_envelope.py`:

```python
from __future__ import annotations

import time
from dataclasses import dataclass

from robot_follow.core.types import MotionCommand


@dataclass(frozen=True)
class CommandEnvelope:
    command: MotionCommand
    generated_at: float
    source: str

    def age_s(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.generated_at

    def is_stale(self, *, now: float, max_age_s: float) -> bool:
        return self.age_s(now) > max_age_s

    @classmethod
    def stop(cls, *, reason: str, generated_at: float | None = None) -> "CommandEnvelope":
        now = time.monotonic() if generated_at is None else generated_at
        return cls(
            command=MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason),
            generated_at=now,
            source="ao_runtime",
        )
```

- [x] **Step 4: Run envelope tests**

Run:

```bash
pytest tests/ao/live/test_command_envelope.py -v
```

Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/robot_follow/ao/live/__init__.py src/robot_follow/ao/live/command_envelope.py tests/ao/live/test_command_envelope.py
git commit -m "feat: add latest command envelope"
```

---

## Task 4: Add RadarServiceManager For Livox/FAST-LIO Lifecycle

**Files:**
- Create: `src/robot_follow/ao/live/radar_service_manager.py`
- Create: `tests/ao/live/test_radar_service_manager.py`

- [x] **Step 1: Write lifecycle tests**

Create `tests/ao/live/test_radar_service_manager.py`:

```python
from robot_follow.ao.config import AORadarServiceConfig
from robot_follow.ao.live.radar_service_manager import RadarServiceManager, RadarServiceState


class FakeProcess:
    def __init__(self, name: str) -> None:
        self.name = name
        self.terminated = False
        self.killed = False

    def poll(self):
        return None

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True

    def wait(self, timeout=None) -> int:
        return 0


class FakePopenFactory:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.processes: list[FakeProcess] = []

    def __call__(self, command: str, shell: bool, start_new_session: bool):
        self.commands.append(command)
        process = FakeProcess(command)
        self.processes.append(process)
        return process


def test_ensure_running_starts_livox_then_fast_lio_once() -> None:
    popen = FakePopenFactory()
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
            idle_stop_s=5.0,
        ),
        popen_factory=popen,
    )

    manager.on_position_frame(10.0)
    manager.ensure_running()
    manager.ensure_running()

    assert popen.commands == ["start_livox", "start_fast_lio"]
    assert manager.state == RadarServiceState.RUNNING


def test_idle_stop_uses_position_frame_age_not_motion_command_age() -> None:
    popen = FakePopenFactory()
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
            idle_stop_s=5.0,
        ),
        popen_factory=popen,
    )

    manager.on_position_frame(10.0)
    manager.ensure_running()
    manager.stop_if_idle(now=14.9)

    assert manager.state == RadarServiceState.RUNNING
    assert popen.processes[0].terminated is False

    manager.stop_if_idle(now=15.1)

    assert manager.state == RadarServiceState.STOPPED
    assert popen.processes[0].terminated is True
    assert popen.processes[1].terminated is True


def test_missing_start_command_raises_clear_error() -> None:
    manager = RadarServiceManager(AORadarServiceConfig())
    manager.on_position_frame(10.0)

    try:
        manager.ensure_running()
    except ValueError as exc:
        assert "livox_start_command" in str(exc)
    else:
        raise AssertionError("expected ValueError")
```

- [x] **Step 2: Run failing lifecycle tests**

Run:

```bash
pytest tests/ao/live/test_radar_service_manager.py -v
```

Expected: FAIL with missing module.

- [x] **Step 3: Implement manager**

Create `src/robot_follow/ao/live/radar_service_manager.py`:

```python
from __future__ import annotations

import subprocess
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Protocol

from robot_follow.ao.config import AORadarServiceConfig


class ManagedProcess(Protocol):
    def poll(self):
        pass

    def terminate(self) -> None:
        pass

    def kill(self) -> None:
        pass

    def wait(self, timeout=None) -> int:
        pass


class PopenFactory(Protocol):
    def __call__(self, command: str, *, shell: bool, start_new_session: bool) -> ManagedProcess:
        pass


class RadarServiceState(str, Enum):
    STOPPED = "stopped"
    RUNNING = "running"


@dataclass
class RadarServiceManager:
    config: AORadarServiceConfig
    popen_factory: PopenFactory = subprocess.Popen

    def __post_init__(self) -> None:
        self._livox: Optional[ManagedProcess] = None
        self._fast_lio: Optional[ManagedProcess] = None
        self._last_position_at: Optional[float] = None

    @property
    def state(self) -> RadarServiceState:
        if self._livox is None and self._fast_lio is None:
            return RadarServiceState.STOPPED
        return RadarServiceState.RUNNING

    def on_position_frame(self, timestamp: float) -> None:
        self._last_position_at = timestamp

    def ensure_running(self) -> None:
        if not self.config.livox_start_command:
            raise ValueError("ao_radar.radar.livox_start_command must be configured")
        if not self.config.fast_lio_start_command:
            raise ValueError("ao_radar.radar.fast_lio_start_command must be configured")
        if self._livox is None or self._livox.poll() is not None:
            self._livox = self.popen_factory(
                self.config.livox_start_command,
                shell=True,
                start_new_session=True,
            )
        if self._fast_lio is None or self._fast_lio.poll() is not None:
            self._fast_lio = self.popen_factory(
                self.config.fast_lio_start_command,
                shell=True,
                start_new_session=True,
            )

    def stop_if_idle(self, *, now: float) -> None:
        if self._last_position_at is None:
            return
        if now - self._last_position_at >= self.config.idle_stop_s:
            self.stop()

    def stop(self) -> None:
        self._stop_process(self._fast_lio)
        self._stop_process(self._livox)
        self._fast_lio = None
        self._livox = None

    def _stop_process(self, process: Optional[ManagedProcess]) -> None:
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=self.config.stop_timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)
```

- [x] **Step 4: Run lifecycle tests**

Run:

```bash
pytest tests/ao/live/test_radar_service_manager.py -v
```

Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/robot_follow/ao/live/radar_service_manager.py tests/ao/live/test_radar_service_manager.py
git commit -m "feat: manage radar service lifecycle"
```

---

## Task 5: Copy And Adapt AO Radar/Avoidance Core

**Files:**
- Create: `src/robot_follow/ao/core/types.py`
- Create: `src/robot_follow/ao/core/geometry.py`
- Create: `src/robot_follow/ao/radar/__init__.py`
- Create: `src/robot_follow/ao/radar/transform.py`
- Create: `src/robot_follow/ao/radar/preprocessing.py`
- Create: `src/robot_follow/ao/radar/occupancy.py`
- Create: `src/robot_follow/ao/radar/pipeline.py`
- Create: `src/robot_follow/ao/avoidance/__init__.py`
- Create: `src/robot_follow/ao/avoidance/zones.py`
- Create: `src/robot_follow/ao/avoidance/limiter.py`
- Create: `src/robot_follow/ao/avoidance/decision.py`
- Create: `src/robot_follow/ao/avoidance/planner.py`
- Create: `tests/ao/avoidance/test_planner_adapter.py`

- [ ] **Step 1: Copy old AO modules**

Run:

```bash
rsync -av /home/yilin/Code/Robot_FollowAO_Radar/src/g1_mid360_ao/core/ src/robot_follow/ao/core/
rsync -av /home/yilin/Code/Robot_FollowAO_Radar/src/g1_mid360_ao/radar/ src/robot_follow/ao/radar/
rsync -av /home/yilin/Code/Robot_FollowAO_Radar/src/g1_mid360_ao/avoidance/ src/robot_follow/ao/avoidance/
```

Expected: files copied into `src/robot_follow/ao/core`, `src/robot_follow/ao/radar`, and `src/robot_follow/ao/avoidance`.

- [ ] **Step 2: Replace package imports**

Run:

```bash
rg "g1_mid360_ao" src/robot_follow/ao
```

For every hit under `src/robot_follow/ao`, replace `g1_mid360_ao.` with `robot_follow.ao.`.

Run again:

```bash
rg "g1_mid360_ao" src/robot_follow/ao
```

Expected: no output.

- [ ] **Step 3: Add planner adapter tests using Robot Follow MotionCommand**

Create `tests/ao/avoidance/test_planner_adapter.py`:

```python
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand


def test_ao_planner_accepts_robot_follow_motion_command() -> None:
    from robot_follow.ao.avoidance.planner import AvoidancePlanner
    from robot_follow.ao.radar.pipeline import ObstacleReport

    planner = AvoidancePlanner()
    raw = MotionCommand(kind="move", vx=0.3, vy=0.0, omega=0.0, reason="follow")
    envelope = CommandEnvelope(command=raw, generated_at=10.0, source="robot_follow")
    clear_report = ObstacleReport.clear(timestamp=10.0)

    safe = planner.filter_command(envelope, clear_report, now=10.01)

    assert isinstance(safe, MotionCommand)
    assert safe.kind == "move"
    assert safe.vx == raw.vx
    assert safe.reason in {"follow", "clear"}


def test_ao_planner_overrides_command_when_front_zone_blocked() -> None:
    from robot_follow.ao.avoidance.planner import AvoidancePlanner
    from robot_follow.ao.radar.pipeline import ObstacleReport

    planner = AvoidancePlanner()
    raw = MotionCommand(kind="move", vx=0.3, vy=0.0, omega=0.0, reason="follow")
    envelope = CommandEnvelope(command=raw, generated_at=10.0, source="robot_follow")
    blocked_report = ObstacleReport.front_blocked(timestamp=10.0, min_distance_m=0.35)

    safe = planner.filter_command(envelope, blocked_report, now=10.01)

    assert isinstance(safe, MotionCommand)
    assert safe.kind == "stop"
    assert safe.reason != "follow"
```

- [ ] **Step 4: Run planner adapter tests and inspect failures**

Run:

```bash
pytest tests/ao/avoidance/test_planner_adapter.py -v
```

Expected: FAIL because copied modules still expose old interfaces.

- [ ] **Step 5: Adapt interfaces without changing avoidance rules**

Modify copied modules so these public interfaces exist:

In `src/robot_follow/ao/radar/pipeline.py`, ensure `ObstacleReport` has:

```python
@classmethod
def clear(cls, *, timestamp: float) -> "ObstacleReport":
    return cls(timestamp=timestamp, front_blocked=False, min_distance_m=None)

@classmethod
def front_blocked(cls, *, timestamp: float, min_distance_m: float) -> "ObstacleReport":
    return cls(timestamp=timestamp, front_blocked=True, min_distance_m=min_distance_m)
```

Use the existing copied fields if names differ; add compatibility properties instead of deleting old fields.

In `src/robot_follow/ao/avoidance/planner.py`, keep the copied `plan_avoidance`, `StatefulAvoidancePlanner`, and state-machine methods. Add this adapter code at the bottom of the file:

```python
from dataclasses import dataclass, field

from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand as RobotFollowMotionCommand
from robot_follow.ao.core.types import MotionCommand as AOMotionCommand


@dataclass(frozen=True)
class DefaultAvoidanceConfig:
    lateral_vy_mps: float = 0.08
    forward_vx_when_avoiding_mps: float = 0.03
    min_slowdown_ratio: float = 0.25
    max_vx: float = 0.5
    max_vy: float = 0.2
    max_omega: float = 0.8
    allow_left_avoidance: bool = True
    allow_right_avoidance: bool = True
    allow_planner_backward_escape: bool = False
    side_switch_cooldown_s: float = 0.5
    avoid_clear_required_frames: int = 3
    avoid_clearance_hold_s: float = 0.3
    return_vy_decay: float = 0.5
    return_vy_epsilon_mps: float = 0.01
    side_score_min_margin_m: float = 0.1
    tie_break_side: str = "left"
    side_danger_distance_m: float = 0.35


@dataclass(frozen=True)
class DefaultZoneConfig:
    slowdown_distance_m: float = 1.0


@dataclass(frozen=True)
class DefaultPlannerConfig:
    avoidance: DefaultAvoidanceConfig = field(default_factory=DefaultAvoidanceConfig)
    zones: DefaultZoneConfig = field(default_factory=DefaultZoneConfig)


class AvoidancePlanner:
    def __init__(self, config: object | None = None) -> None:
        self._stateful = StatefulAvoidancePlanner(config or DefaultPlannerConfig())

    def filter_command(
        self,
        envelope: CommandEnvelope,
        report: ObstacleReport,
        *,
        now: float,
    ) -> RobotFollowMotionCommand:
        desired = _to_ao_motion_command(envelope)
        decision = self._stateful.plan_and_limit(desired, report, now_s=now)
        return _to_robot_follow_motion_command(decision.safe_command)


def _to_ao_motion_command(envelope: CommandEnvelope) -> AOMotionCommand:
    command = envelope.command
    if command.kind == "stop":
        return AOMotionCommand.stop(
            envelope.generated_at,
            source=envelope.source,
            reason=command.reason,
        )
    return AOMotionCommand.move(
        envelope.generated_at,
        source=envelope.source,
        vx=command.vx,
        vy=command.vy,
        omega=command.omega,
        reason=command.reason,
    )


def _to_robot_follow_motion_command(command: AOMotionCommand) -> RobotFollowMotionCommand:
    kind = "stop" if command.kind.value == "stop" else "move"
    return RobotFollowMotionCommand(
        kind=kind,
        vx=command.vx,
        vy=command.vy,
        omega=command.omega,
        reason=command.reason,
    )
```

- [ ] **Step 6: Run planner adapter tests**

Run:

```bash
pytest tests/ao/avoidance/test_planner_adapter.py -v
```

Expected: PASS.

- [ ] **Step 7: Run copied AO package import check**

Run:

```bash
python -m compileall src/robot_follow/ao
```

Expected: all files compile without import errors.

- [ ] **Step 8: Commit**

```bash
git add src/robot_follow/ao/core src/robot_follow/ao/radar src/robot_follow/ao/avoidance tests/ao/avoidance/test_planner_adapter.py
git commit -m "feat: adapt ao avoidance core to robot follow commands"
```

---

## Task 6: Add Latest-Only FAST-LIO Point Cloud Source

**Files:**
- Create: `src/robot_follow/ao/live/fast_lio_source.py`
- Create: `tests/ao/live/test_fast_lio_source.py`

- [x] **Step 1: Write latest-only source tests**

Create `tests/ao/live/test_fast_lio_source.py`:

```python
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource, PointCloudFrame


def test_latest_frame_replaces_previous_frame() -> None:
    source = FastLIOPointCloudSource(topic="/cloud_registered_body")

    source.update_for_test(PointCloudFrame(timestamp=10.0, points=[(1.0, 0.0, 0.0)]))
    source.update_for_test(PointCloudFrame(timestamp=10.1, points=[(2.0, 0.0, 0.0)]))

    latest = source.latest()

    assert latest is not None
    assert latest.timestamp == 10.1
    assert latest.points == [(2.0, 0.0, 0.0)]


def test_frame_age_and_staleness() -> None:
    frame = PointCloudFrame(timestamp=10.0, points=[])

    assert frame.age_s(now=10.25) == 0.25
    assert frame.is_stale(now=10.6, max_age_s=0.5) is True
    assert frame.is_stale(now=10.4, max_age_s=0.5) is False
```

- [x] **Step 2: Run failing source tests**

Run:

```bash
pytest tests/ao/live/test_fast_lio_source.py -v
```

Expected: FAIL with missing module.

- [x] **Step 3: Implement source with lazy ROS2 imports**

Create `src/robot_follow/ao/live/fast_lio_source.py`:

```python
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True)
class PointCloudFrame:
    timestamp: float
    points: list[tuple[float, float, float]]

    def age_s(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.timestamp

    def is_stale(self, *, now: float, max_age_s: float) -> bool:
        return self.age_s(now) > max_age_s


class FastLIOPointCloudSource:
    def __init__(self, topic: str = "/cloud_registered_body") -> None:
        self.topic = topic
        self._lock = threading.Lock()
        self._latest: Optional[PointCloudFrame] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_ros2, name="fast-lio-source", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def latest(self) -> Optional[PointCloudFrame]:
        with self._lock:
            return self._latest

    def update_for_test(self, frame: PointCloudFrame) -> None:
        with self._lock:
            self._latest = frame

    def _run_ros2(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        rclpy.init(args=None)
        node = Node("robot_follow_ao_fast_lio_source")

        def on_cloud(msg: PointCloud2) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0:
                stamp = time.monotonic()
            points = [
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ]
            self.update_for_test(PointCloudFrame(timestamp=stamp, points=points))

        node.create_subscription(PointCloud2, self.topic, on_cloud, 1)
        try:
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()
```

- [x] **Step 4: Run source tests**

Run:

```bash
pytest tests/ao/live/test_fast_lio_source.py -v
```

Expected: PASS without importing ROS2.

- [x] **Step 5: Commit**

```bash
git add src/robot_follow/ao/live/fast_lio_source.py tests/ao/live/test_fast_lio_source.py
git commit -m "feat: add latest fast lio point cloud source"
```

---

## Task 7: Add AO Runtime Latest-Only Control Loop

**Files:**
- Create: `src/robot_follow/ao/live/runtime.py`
- Create: `tests/ao/live/test_runtime.py`

- [x] **Step 1: Write runtime behavior tests**

Create `tests/ao/live/test_runtime.py`:

```python
from dataclasses import dataclass

from robot_follow.ao.config import AOALXConfig, AOLatencyConfig, AOOutputConfig, AORadarConfig, AORadarServiceConfig
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.ao.live.fast_lio_source import PointCloudFrame
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core.pipeline import PipelineStepResult
from robot_follow.core.types import MotionCommand, TargetSample


@dataclass(frozen=True)
class FakeSnapshot:
    timestamp: float
    distance_m: float = 1.0
    azimuth_deg: float = 0.0
    error: str | None = None

    def age_s(self, now=None) -> float:
        return (100.0 if now is None else now) - self.timestamp


class FakeALXSource:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def snapshot(self):
        return self._snapshot


class FakeRadarManager:
    def __init__(self):
        self.position_times = []
        self.ensure_count = 0
        self.stop_checks = []

    def on_position_frame(self, timestamp: float) -> None:
        self.position_times.append(timestamp)

    def ensure_running(self) -> None:
        self.ensure_count += 1

    def stop_if_idle(self, *, now: float) -> None:
        self.stop_checks.append(now)


class FakeCloudSource:
    def __init__(self, frame):
        self._frame = frame

    def latest(self):
        return self._frame


class FakePipeline:
    def __init__(self):
        self.calls = []

    def process(self, sample: TargetSample):
        self.calls.append(sample)
        command = MotionCommand(kind="move", vx=0.2, vy=0.0, omega=0.0, reason="follow")
        return PipelineStepResult(sample=sample, command=command, status="ok")


class FakePlanner:
    def filter_command(self, envelope: CommandEnvelope, report, *, now: float) -> MotionCommand:
        return envelope.command


class FakeRadarPipeline:
    def process(self, frame):
        return object()


class FakeBridge:
    def __init__(self):
        self.commands = []

    def dispatch(self, command: MotionCommand) -> None:
        self.commands.append(command)


def config() -> AORadarConfig:
    return AORadarConfig(
        alx=AOALXConfig(stale_timeout_s=0.5),
        radar=AORadarServiceConfig(idle_stop_s=5.0, livox_start_command="livox", fast_lio_start_command="fast"),
        latency=AOLatencyConfig(max_command_age_s=0.2, max_cloud_age_s=0.5, max_ao_process_ms=20.0),
        output=AOOutputConfig(mode="g1", require_real_g1_confirmation=True, confirm_real_g1=True),
    )


def test_fresh_position_starts_radar_and_dispatches_safe_command() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(FakeSnapshot(timestamp=99.95)),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.position_times == [99.95]
    assert radar_manager.ensure_count == 1
    assert bridge.commands[-1].kind == "move"


def test_stale_position_stops_g1_and_does_not_start_radar() -> None:
    bridge = FakeBridge()
    radar_manager = FakeRadarManager()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(FakeSnapshot(timestamp=99.0)),
        pipeline=FakePipeline(),
        radar_manager=radar_manager,
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert radar_manager.ensure_count == 0
    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "data_timeout"


def test_missing_or_stale_cloud_stops_before_ao_planner() -> None:
    bridge = FakeBridge()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(FakeSnapshot(timestamp=99.95)),
        pipeline=FakePipeline(),
        radar_manager=FakeRadarManager(),
        cloud_source=FakeCloudSource(None),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "radar_not_ready"
```

- [x] **Step 2: Run failing runtime tests**

Run:

```bash
pytest tests/ao/live/test_runtime.py -v
```

Expected: FAIL with missing runtime module.

- [x] **Step 3: Implement runtime tick**

Create `src/robot_follow/ao/live/runtime.py`:

```python
from __future__ import annotations

import time
from typing import Callable, Protocol

from robot_follow.ao.config import AORadarConfig
from robot_follow.ao.live.command_envelope import CommandEnvelope
from robot_follow.core.types import MotionCommand, TargetSample


Clock = Callable[[], float]


class AORadarRuntime:
    def __init__(
        self,
        *,
        config: AORadarConfig,
        alx_source,
        pipeline,
        radar_manager,
        cloud_source,
        radar_pipeline,
        planner,
        bridge,
        clock: Clock = time.monotonic,
    ) -> None:
        self.config = config
        self.alx_source = alx_source
        self.pipeline = pipeline
        self.radar_manager = radar_manager
        self.cloud_source = cloud_source
        self.radar_pipeline = radar_pipeline
        self.planner = planner
        self.bridge = bridge
        self.clock = clock

    def tick(self) -> MotionCommand:
        now = self.clock()
        snapshot = self.alx_source.snapshot()
        if snapshot is None or getattr(snapshot, "error", None):
            return self._dispatch_stop("serial_error")
        if snapshot.age_s(now) > self.config.alx.stale_timeout_s:
            self.radar_manager.stop_if_idle(now=now)
            return self._dispatch_stop("data_timeout")

        self.radar_manager.on_position_frame(snapshot.timestamp)
        self.radar_manager.ensure_running()

        sample = TargetSample(
            timestamp=snapshot.timestamp,
            distance_m=snapshot.distance_m,
            azimuth_deg=float(snapshot.azimuth_deg),
            valid=True,
        )
        step = self.pipeline.process(sample)
        envelope = CommandEnvelope(
            command=step.command,
            generated_at=now,
            source="robot_follow",
        )

        cloud = self.cloud_source.latest()
        if cloud is None:
            return self._dispatch_stop("radar_not_ready")
        if cloud.is_stale(now=now, max_age_s=self.config.latency.max_cloud_age_s):
            return self._dispatch_stop("radar_timeout")
        if envelope.is_stale(now=now, max_age_s=self.config.latency.max_command_age_s):
            return self._dispatch_stop("command_timeout")

        ao_start = self.clock()
        report = self.radar_pipeline.process(cloud)
        safe = self.planner.filter_command(envelope, report, now=now)
        ao_ms = (self.clock() - ao_start) * 1000.0
        if ao_ms > self.config.latency.max_ao_process_ms:
            return self._dispatch_stop("ao_timeout")

        self.bridge.dispatch(safe)
        return safe

    def _dispatch_stop(self, reason: str) -> MotionCommand:
        command = MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)
        self.bridge.dispatch(command)
        return command
```

- [x] **Step 4: Run runtime tests**

Run:

```bash
pytest tests/ao/live/test_runtime.py -v
```

Expected: PASS.

- [x] **Step 5: Add missing timeout tests**

Append to `tests/ao/live/test_runtime.py`:

```python
class SlowClock:
    def __init__(self):
        self.values = iter([100.0, 100.0, 100.05])

    def __call__(self) -> float:
        return next(self.values)


def test_slow_ao_result_is_discarded() -> None:
    bridge = FakeBridge()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(FakeSnapshot(timestamp=99.95)),
        pipeline=FakePipeline(),
        radar_manager=FakeRadarManager(),
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.9, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=SlowClock(),
    )

    runtime.tick()

    assert bridge.commands[-1].kind == "stop"
    assert bridge.commands[-1].reason == "ao_timeout"
```

- [x] **Step 6: Run runtime tests again**

Run:

```bash
pytest tests/ao/live/test_runtime.py -v
```

Expected: PASS.

- [x] **Step 7: Commit**

```bash
git add src/robot_follow/ao/live/runtime.py tests/ao/live/test_runtime.py
git commit -m "feat: add ao radar runtime loop"
```

---

## Task 8: Add Per-Frame CSV Timing Logger

**Files:**
- Create: `src/robot_follow/ao/logging/__init__.py`
- Create: `src/robot_follow/ao/logging/csv_logger.py`
- Create: `tests/ao/logging/test_csv_logger.py`

- [ ] **Step 1: Write logger tests**

Create `tests/ao/logging/test_csv_logger.py`:

```python
from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, AORadarLogRow


def test_csv_logger_writes_required_latency_fields(tmp_path) -> None:
    path = tmp_path / "ao.csv"
    logger = AORadarCSVLogger(path)

    logger.write(
        AORadarLogRow(
            timestamp=10.0,
            alx_age_ms=5.0,
            command_age_ms=3.0,
            cloud_age_ms=20.0,
            loop_dt_ms=50.0,
            ao_process_ms=4.0,
            g1_dispatch_ms=2.0,
            raw_command_kind="move",
            raw_command_reason="follow",
            safe_command_kind="stop",
            safe_command_reason="front_blocked",
            radar_state="running",
            wristband_gate_state="on",
        )
    )
    logger.close()

    text = path.read_text(encoding="utf-8")

    assert "alx_age_ms,command_age_ms,cloud_age_ms" in text
    assert "front_blocked" in text
```

- [ ] **Step 2: Run failing logger tests**

Run:

```bash
pytest tests/ao/logging/test_csv_logger.py -v
```

Expected: FAIL with missing module.

- [ ] **Step 3: Implement logger**

Create `src/robot_follow/ao/logging/__init__.py`:

```python
"""AO radar logging helpers."""
```

Create `src/robot_follow/ao/logging/csv_logger.py`:

```python
from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class AORadarLogRow:
    timestamp: float
    alx_age_ms: float
    command_age_ms: float
    cloud_age_ms: float
    loop_dt_ms: float
    ao_process_ms: float
    g1_dispatch_ms: float
    raw_command_kind: str
    raw_command_reason: str
    safe_command_kind: str
    safe_command_reason: str
    radar_state: str
    wristband_gate_state: str


class AORadarCSVLogger:
    def __init__(self, path: Path) -> None:
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=list(AORadarLogRow.__dataclass_fields__))
        self._writer.writeheader()

    def write(self, row: AORadarLogRow) -> None:
        self._writer.writerow(asdict(row))
        self._file.flush()

    def close(self) -> None:
        self._file.close()
```

- [ ] **Step 4: Run logger tests**

Run:

```bash
pytest tests/ao/logging/test_csv_logger.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_follow/ao/logging tests/ao/logging/test_csv_logger.py
git commit -m "feat: add ao radar csv timing logger"
```

---

## Task 9: Add CLI Entrypoint With Real G1 Guard

**Files:**
- Create: `src/robot_follow/ao/live/cli.py`
- Modify: `pyproject.toml`
- Create: `tests/ao/live/test_cli.py`

- [x] **Step 1: Write CLI guard tests**

Create `tests/ao/live/test_cli.py`:

```python
from pathlib import Path

import pytest

from robot_follow.ao.live.cli import build_runtime_from_config


def test_real_g1_requires_explicit_confirmation(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: g1\n"
        "    require_real_g1_confirmation: true\n"
        "    confirm_real_g1: false\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="confirm_real_g1"):
        build_runtime_from_config(path)
```

- [x] **Step 2: Run failing CLI tests**

Run:

```bash
pytest tests/ao/live/test_cli.py -v
```

Expected: FAIL with missing module.

- [x] **Step 3: Implement CLI skeleton**

Create `src/robot_follow/ao/live/cli.py`:

```python
from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_follow.ao.avoidance.planner import AvoidancePlanner
from robot_follow.ao.config import load_ao_radar_config
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.live.radar_service_manager import RadarServiceManager
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core import FollowConfig, FollowController, FollowPipeline, SafetyConfig, SafetyManager, TargetFilter
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.real.g1.bridge import G1MotionBridge


def build_runtime_from_config(config_path: Path) -> AORadarRuntime:
    config = load_ao_radar_config(config_path)
    if (
        config.output.mode == "g1"
        and config.output.require_real_g1_confirmation
        and not config.output.confirm_real_g1
    ):
        raise ValueError("real G1 output requires ao_radar.output.confirm_real_g1: true")

    alx_source = ALXLiveSource(port=config.alx.port, baudrate=config.alx.baudrate)
    pipeline = FollowPipeline(
        target_filter=TargetFilter(),
        safety=SafetyManager(SafetyConfig()),
        controller=FollowController(FollowConfig()),
    )
    radar_manager = RadarServiceManager(config.radar)
    cloud_source = FastLIOPointCloudSource()
    from robot_follow.ao.radar.pipeline import RadarPipeline

    radar_pipeline = RadarPipeline()
    planner = AvoidancePlanner()
    bridge = G1MotionBridge()
    return AORadarRuntime(
        config=config,
        alx_source=alx_source,
        pipeline=pipeline,
        radar_manager=radar_manager,
        cloud_source=cloud_source,
        radar_pipeline=radar_pipeline,
        planner=planner,
        bridge=bridge,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/jetson_g1_safe.yaml"))
    args = parser.parse_args(argv)
    runtime = build_runtime_from_config(args.config)
    runtime.alx_source.start()
    runtime.cloud_source.start()
    period_s = 1.0 / runtime.config.latency.ao_output_loop_hz
    try:
        while True:
            started = time.monotonic()
            runtime.tick()
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period_s - elapsed))
    finally:
        runtime.cloud_source.stop()
        runtime.alx_source.stop()
        runtime.radar_manager.stop()
```

- [x] **Step 4: Add console script**

Modify `pyproject.toml` under `[project.scripts]`:

```toml
robot-follow-ao-radar = "robot_follow.ao.live.cli:main"
```

- [x] **Step 5: Run CLI tests**

Run:

```bash
pytest tests/ao/live/test_cli.py -v
```

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git add src/robot_follow/ao/live/cli.py tests/ao/live/test_cli.py pyproject.toml
git commit -m "feat: add ao radar live cli"
```

---

## Task 10: Integration Verification And Documentation Update

**Files:**
- Modify: `docs/superpowers/specs/2026-07-09-robot-follow-ao-radar-design.md`
- Optional Create: `docs/superpowers/notes/2026-07-09-robot-follow-ao-radar-runbook.md`

- [x] **Step 1: Run focused test suite**

Run:

```bash
pytest tests/test_serial_reader.py tests/real/test_alx_source.py tests/ao -v
```

Expected: PASS.

- [x] **Step 2: Run package compile check**

Run:

```bash
python -m compileall src/robot_follow
```

Expected: PASS.

- [x] **Step 3: Verify no accidental second serial parser**

Run:

```bash
rg "Serial\\(|ALXG100SerialReader\\(" src/robot_follow/ao src/robot_follow
```

Expected:
- `ALXG100SerialReader(` appears in `src/robot_follow/real/alx_source.py`.
- AO runtime uses `ALXLiveSource`, not `serial.Serial` directly.
- No new independent wristband parser exists under `src/robot_follow/ao`.

- [x] **Step 4: Verify no command queues were introduced**

Run:

```bash
rg "Queue|deque|append\\(" src/robot_follow/ao/live src/robot_follow/ao/avoidance
```

Expected:
- No `Queue` in AO live command path.
- No command history list that stores pending `MotionCommand`.
- Existing avoidance state machine may keep state, but it must not enqueue commands for later dispatch.

- [x] **Step 5: Update spec implementation status**

Append to `docs/superpowers/specs/2026-07-09-robot-follow-ao-radar-design.md`:

```markdown
## Implementation Plan

Implementation plan: `docs/superpowers/plans/2026-07-09-robot-follow-ao-radar.md`

The implementation keeps Robot Follow core unchanged, exposes only structured wristband gate state from the existing parser, adds AO runtime under `robot_follow.ao`, and keeps the command/point-cloud paths latest-only.
```

- [x] **Step 6: Commit**

```bash
git add docs/superpowers/specs/2026-07-09-robot-follow-ao-radar-design.md
git commit -m "docs: link ao radar implementation plan"
```

---

## Verification Checklist

- [ ] `pytest tests/test_serial_reader.py tests/real/test_alx_source.py tests/ao -v` passes.
- [ ] `python -m compileall src/robot_follow` passes.
- [ ] `MotionCommand` still has only `kind`, `vx`, `vy`, `omega`, `reason`.
- [ ] Only `ALXLiveSource` owns `ALXG100SerialReader` in the live AO chain.
- [ ] Radar ON/OFF uses fresh ALX position frame age, not MotionCommand queue state.
- [ ] Radar services stop after 5 seconds without fresh position frames.
- [ ] G1 stops immediately on stale ALX, missing cloud, stale cloud, stale command, AO timeout, or serial error.
- [ ] AO planner accepts `CommandEnvelope + ObstacleReport` and returns current Robot Follow `MotionCommand`.
- [ ] Missing real G1 confirmation raises an error.

## Execution Notes

Use the `robot_followao_radar` Python environment for implementation and verification. If a command must run through conda, use:

```bash
conda run -n robot_followao_radar pytest tests/ao -v
```

For live hardware, do not run the CLI until `ao_radar.output.confirm_real_g1: true`, Livox/FAST-LIO commands, ROS2 environment, and G1 SDK prerequisites are explicitly configured.
