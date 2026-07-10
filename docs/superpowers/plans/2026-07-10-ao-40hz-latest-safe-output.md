# AO 40 Hz Raw/Safe Output Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make AO safe command output run at 40 Hz and stay aligned with each tick's raw motion command, without introducing the removed async sender/mailbox/stop-epoch architecture.

**Architecture:** Keep the existing synchronous `AORadarRuntime.tick()` path: ALX snapshot -> raw follow command -> AO planner/safety -> existing bridge dispatch -> one CSV row. Extract the duplicated CLI/Web loop timing into a small deterministic helper so tests can prove 25 ms scheduling, while preserving the existing runtime and logger data model.

**Tech Stack:** Python dataclasses, existing Robot Follow AO runtime, pytest, monotonic clocks, existing CSV logger, existing CLI and Web CLI entrypoints.

## Global Constraints

- `ao_radar.latency.ao_output_loop_hz` MUST remain `40.0` in `configs/jetson_g1_ao_radar.yaml`.
- A configured 40 Hz AO loop means a 25 ms target period.
- Do not add async G1 sender, latest-safe mailbox, stop epoch, or a new stop-latch state machine.
- Do not change AO planner obstacle geometry, follow gains, velocity limits, or G1 bridge semantics.
- `--web-hz` affects only Web payload publishing frequency and MUST NOT reduce AO tick frequency.
- Each AO tick MUST produce one `AORadarLogRow` when logging is enabled.
- Raw-before-safety stop paths MUST preserve the tick's raw command diagnostics in the same log row.
- Raw-precondition stop paths such as wristband OFF/unknown MAY leave raw command fields empty.
- Use monotonic timing for loop period and latency fields.
- Preserve unrelated worktree changes. Do not reset, checkout, or reformat unrelated files.

---

## File Structure

- Create: `src/robot_follow/ao/live/loop_timing.py`
  - Owns deterministic 40 Hz scheduling math shared by CLI and Web CLI.
  - Exposes `LoopTiming` and `LoopTimingConfig`.
- Modify: `src/robot_follow/ao/live/cli.py`
  - Uses `LoopTiming` instead of open-coded elapsed/sleep calculation.
  - Keeps existing runtime startup, logging, summary and shutdown behavior.
- Modify: `src/robot_follow/ao/live/web_cli.py`
  - Uses `LoopTiming` for AO tick cadence.
  - Keeps Web update throttling separate via `--web-hz`.
- Modify: `src/robot_follow/ao/live/runtime.py`
  - Tightens command-age and latency logging so raw/safe rows reflect same-tick timing.
  - Keeps existing synchronous dispatch path.
- Modify: `src/robot_follow/ao/logging/csv_logger.py`
  - Ensures required latency fields are always present in CSV headers.
  - Keeps one row per AO tick.
- Create: `tests/ao/live/test_loop_timing.py`
  - Unit tests for 25 ms period, overrun behavior, and Web throttling separation.
- Modify: `tests/ao/live/test_cli.py`
  - Verifies CLI constructs a 25 ms AO loop when config is 40 Hz.
- Modify: `tests/ao/live/test_web_cli.py`
  - Verifies Web update rate is independent of AO tick rate.
- Modify: `tests/ao/live/test_runtime.py`
  - Verifies raw/safe timing alignment and stop diagnostics.
- Modify: `tests/ao/logging/test_csv_logger.py`
  - Verifies all required latency columns are serialized.
- Create: `scripts/analyze_ao_40hz_log.py`
  - Offline validator for CSV evidence from mock or real runs.
- Create: `tests/ao/live/test_analyze_ao_40hz_log.py`
  - Unit tests for the validator's p50/p95/p99 and failure messages.

---

### Task 1: Add Deterministic AO Loop Timing Helper

**Files:**
- Create: `src/robot_follow/ao/live/loop_timing.py`
- Create: `tests/ao/live/test_loop_timing.py`

**Interfaces:**
- Produces: `LoopTimingConfig(target_hz: float)` with property `period_s: float`.
- Produces: `LoopTiming(config: LoopTimingConfig, clock: Callable[[], float], sleeper: Callable[[float], None])`.
- Produces: `LoopTiming.mark_tick_start() -> float`.
- Produces: `LoopTiming.sleep_until_next_tick(started_at: float) -> float`, returning the requested sleep duration.
- Later tasks consume `LoopTiming` from CLI and Web CLI.

- [ ] **Step 1: Write failing tests for 40 Hz period and sleep duration**

Add `tests/ao/live/test_loop_timing.py`:

```python
from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig


class FakeClock:
    def __init__(self, values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


def test_40hz_config_has_25ms_period():
    config = LoopTimingConfig(target_hz=40.0)

    assert config.period_s == 0.025


def test_sleep_until_next_tick_uses_remaining_25ms_period():
    sleeps = []
    timing = LoopTiming(
        config=LoopTimingConfig(target_hz=40.0),
        clock=FakeClock([10.010]),
        sleeper=sleeps.append,
    )

    requested = timing.sleep_until_next_tick(started_at=10.000)

    assert requested == 0.015
    assert sleeps == [0.015]
```

- [ ] **Step 2: Run the tests and verify they fail**

Run:

```bash
pytest tests/ao/live/test_loop_timing.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'robot_follow.ao.live.loop_timing'`.

- [ ] **Step 3: Implement the helper**

Create `src/robot_follow/ao/live/loop_timing.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


Clock = Callable[[], float]
Sleeper = Callable[[float], None]


@dataclass(frozen=True)
class LoopTimingConfig:
    target_hz: float

    def __post_init__(self) -> None:
        if self.target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

    @property
    def period_s(self) -> float:
        return 1.0 / self.target_hz


class LoopTiming:
    def __init__(self, *, config: LoopTimingConfig, clock: Clock, sleeper: Sleeper) -> None:
        self.config = config
        self._clock = clock
        self._sleeper = sleeper

    def mark_tick_start(self) -> float:
        return self._clock()

    def sleep_until_next_tick(self, *, started_at: float) -> float:
        elapsed_s = self._clock() - started_at
        sleep_s = max(0.0, self.config.period_s - elapsed_s)
        if sleep_s > 0.0:
            self._sleeper(sleep_s)
        return sleep_s
```

- [ ] **Step 4: Add overrun and invalid-Hz tests**

Append to `tests/ao/live/test_loop_timing.py`:

```python
import pytest


def test_overrun_does_not_sleep():
    sleeps = []
    timing = LoopTiming(
        config=LoopTimingConfig(target_hz=40.0),
        clock=FakeClock([10.035]),
        sleeper=sleeps.append,
    )

    requested = timing.sleep_until_next_tick(started_at=10.000)

    assert requested == 0.0
    assert sleeps == []


def test_rejects_non_positive_target_hz():
    with pytest.raises(ValueError, match="target_hz"):
        LoopTimingConfig(target_hz=0.0)
```

- [ ] **Step 5: Run the helper tests**

Run:

```bash
pytest tests/ao/live/test_loop_timing.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/robot_follow/ao/live/loop_timing.py tests/ao/live/test_loop_timing.py
git commit -m "test: add deterministic AO loop timing helper"
```

---

### Task 2: Wire CLI Main Loop to the Shared 40 Hz Timing Helper

**Files:**
- Modify: `src/robot_follow/ao/live/cli.py`
- Modify: `tests/ao/live/test_cli.py`

**Interfaces:**
- Consumes: `LoopTiming` and `LoopTimingConfig` from Task 1.
- Produces: CLI behavior where `ao_output_loop_hz=40.0` means `time.sleep(0.025 - elapsed)`.

- [ ] **Step 1: Add a focused CLI loop helper test**

Append to `tests/ao/live/test_cli.py`:

```python
from robot_follow.ao.live.loop_timing import LoopTimingConfig


def test_ao_cli_40hz_period_is_25ms() -> None:
    config = LoopTimingConfig(target_hz=40.0)

    assert config.period_s == 0.025
```

- [ ] **Step 2: Run the focused CLI test**

Run:

```bash
pytest tests/ao/live/test_cli.py::test_ao_cli_40hz_period_is_25ms -v
```

Expected: PASS after Task 1.

- [ ] **Step 3: Replace open-coded CLI sleep math**

Modify `src/robot_follow/ao/live/cli.py` imports:

```python
from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig
```

In `main()`, replace:

```python
period_s = 1.0 / runtime.config.latency.ao_output_loop_hz
while True:
    started = time.monotonic()
    runtime.tick()
    ...
    elapsed = time.monotonic() - started
    sleep_time = max(0.0, period_s - elapsed)
    if sleep_time > 0:
        time.sleep(sleep_time)
```

with:

```python
timing = LoopTiming(
    config=LoopTimingConfig(target_hz=runtime.config.latency.ao_output_loop_hz),
    clock=time.monotonic,
    sleeper=time.sleep,
)
while True:
    started = timing.mark_tick_start()
    runtime.tick()
    if logger is not None and runtime.last_frame is not None:
        logger.write(runtime.last_frame.to_log_row())
    last_summary_at = _maybe_print_summary(
        runtime,
        last_summary_at=last_summary_at,
        now=started,
    )
    timing.sleep_until_next_tick(started_at=started)
```

- [ ] **Step 4: Run CLI tests**

Run:

```bash
pytest tests/ao/live/test_cli.py tests/ao/live/test_loop_timing.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_follow/ao/live/cli.py tests/ao/live/test_cli.py
git commit -m "fix: drive AO CLI loop from configured 40hz timing"
```

---

### Task 3: Keep Web Payload Rate Independent from AO Tick Rate

**Files:**
- Modify: `src/robot_follow/ao/live/web_cli.py`
- Modify: `tests/ao/live/test_web_cli.py`
- Modify: `tests/ao/live/test_loop_timing.py`

**Interfaces:**
- Consumes: `LoopTiming` and `LoopTimingConfig` from Task 1.
- Produces: Web CLI where AO tick timing uses `runtime.config.latency.ao_output_loop_hz`; Web publishing still uses `--web-hz`.

- [ ] **Step 1: Add a pure Web throttle test**

Append to `tests/ao/live/test_loop_timing.py`:

```python
def should_update_web(*, last_web_update_at, started_at, web_period_s):
    return last_web_update_at is None or (started_at - last_web_update_at) >= web_period_s


def test_web_10hz_throttle_allows_ao_40hz_ticks_between_updates():
    web_period_s = 0.1

    assert should_update_web(last_web_update_at=None, started_at=10.000, web_period_s=web_period_s)
    assert not should_update_web(last_web_update_at=10.000, started_at=10.025, web_period_s=web_period_s)
    assert not should_update_web(last_web_update_at=10.000, started_at=10.075, web_period_s=web_period_s)
    assert should_update_web(last_web_update_at=10.000, started_at=10.100, web_period_s=web_period_s)
```

- [ ] **Step 2: Move Web throttle predicate into production**

Add to `src/robot_follow/ao/live/web_cli.py`:

```python
def _should_update_web(
    *,
    last_web_update_at: float | None,
    started_at: float,
    web_period_s: float,
) -> bool:
    return last_web_update_at is None or (started_at - last_web_update_at) >= web_period_s
```

Move the test import to production by replacing the local test helper with:

```python
from robot_follow.ao.live.web_cli import _should_update_web
```

and update the assertions to call `_should_update_web(...)`.

- [ ] **Step 3: Replace Web CLI open-coded AO sleep math**

Modify `src/robot_follow/ao/live/web_cli.py` imports:

```python
from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig
```

In `main()`, replace:

```python
period_s = 1.0 / runtime.config.latency.ao_output_loop_hz
while True:
    started = time.monotonic()
    runtime.tick()
    if last_web_update_at is None or (started - last_web_update_at) >= web_period_s:
        ...
    elapsed = time.monotonic() - started
    sleep_time = max(0.0, period_s - elapsed)
    if sleep_time > 0:
        time.sleep(sleep_time)
```

with:

```python
timing = LoopTiming(
    config=LoopTimingConfig(target_hz=runtime.config.latency.ao_output_loop_hz),
    clock=time.monotonic,
    sleeper=time.sleep,
)
while True:
    started = timing.mark_tick_start()

    runtime.tick()

    if _should_update_web(
        last_web_update_at=last_web_update_at,
        started_at=started,
        web_period_s=web_period_s,
    ):
        _update_web_hub(
            runtime,
            hub,
            app_config,
            max_points=args.max_points,
            now_s=started,
        )
        last_web_update_at = started

    timing.sleep_until_next_tick(started_at=started)
```

- [ ] **Step 4: Run Web and timing tests**

Run:

```bash
pytest tests/ao/live/test_web_cli.py tests/ao/live/test_loop_timing.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_follow/ao/live/web_cli.py tests/ao/live/test_web_cli.py tests/ao/live/test_loop_timing.py
git commit -m "fix: decouple AO web tick rate from web publish rate"
```

---

### Task 4: Tighten Runtime Raw/Safe Timing and Stop Diagnostics

**Files:**
- Modify: `src/robot_follow/ao/live/runtime.py`
- Modify: `tests/ao/live/test_runtime.py`

**Interfaces:**
- Consumes: existing `AORadarRuntime.tick() -> MotionCommand`.
- Produces: `AORadarFrameResult` rows where raw and safe timestamps are same-tick diagnostics.

- [ ] **Step 1: Add a clock that exposes raw, safe, dispatch ordering**

Append to `tests/ao/live/test_runtime.py`:

```python
class SequenceClock:
    def __init__(self, values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)
```

- [ ] **Step 2: Add same-tick raw/safe alignment test**

Append to `tests/ao/live/test_runtime.py`:

```python
def test_successful_tick_logs_raw_safe_and_g1_same_tick_timing() -> None:
    bridge = FakeBridge()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.990),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(
            command=MotionCommand(kind="move", vx=0.2, vy=0.0, omega=0.1, reason="follow")
        ),
        radar_manager=FakeRadarManager(),
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.990, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=SequenceClock([100.000, 100.002, 100.004, 100.005, 100.007, 100.008]),
    )

    runtime.tick()

    frame = runtime.last_frame
    assert frame is not None
    assert frame.raw_command is not None
    assert frame.raw_command.kind == "move"
    assert frame.command.kind == "move"
    assert frame.raw_command_generated_at_ms == pytest.approx(2.0)
    assert frame.safe_command_generated_at_ms == pytest.approx(5.0)
    assert frame.g1_response_at_ms == pytest.approx(8.0)
    assert frame.total_latency_ms == pytest.approx(6.0)
    assert frame.safe_command_generated_at_ms >= frame.raw_command_generated_at_ms
```

- [ ] **Step 3: Add raw-preserving radar stop test**

Append to `tests/ao/live/test_runtime.py`:

```python
def test_radar_timeout_stop_preserves_raw_command_in_log_row() -> None:
    bridge = FakeBridge()
    runtime = AORadarRuntime(
        config=config(),
        alx_source=FakeALXSource(
            FakeSnapshot(timestamp=99.990),
            wristband_state=FakeWristbandState(enabled=True, updated_at=99.9),
        ),
        pipeline=FakePipeline(
            command=MotionCommand(kind="move", vx=0.2, vy=0.0, omega=0.0, reason="follow")
        ),
        radar_manager=FakeRadarManager(),
        cloud_source=FakeCloudSource(PointCloudFrame(timestamp=99.0, points=[])),
        radar_pipeline=FakeRadarPipeline(),
        planner=FakePlanner(),
        bridge=bridge,
        clock=lambda: 100.0,
    )

    runtime.tick()

    row = runtime.last_frame.to_log_row()
    assert row.raw_command_kind == "move"
    assert row.raw_command_reason == "follow"
    assert row.safe_command_kind == "stop"
    assert row.safe_command_reason == "radar_timeout"
```

- [ ] **Step 4: Run the new runtime tests**

Run:

```bash
pytest tests/ao/live/test_runtime.py::test_successful_tick_logs_raw_safe_and_g1_same_tick_timing tests/ao/live/test_runtime.py::test_radar_timeout_stop_preserves_raw_command_in_log_row -v
```

Expected: FAIL if current timing calls do not match the explicit sequence, or PASS if the existing implementation already satisfies the spec.

- [ ] **Step 5: Adjust runtime timing only where tests show mismatch**

If the Step 4 tests fail because command age or timestamps are sampled from inconsistent phases, update `src/robot_follow/ao/live/runtime.py` so the successful path computes:

```python
raw_elapsed_ms = (self.clock() - tick_start) * 1000.0
...
ao_start = self.clock()
report = self.radar_pipeline.process(cloud)
safe = self.planner.filter_command(envelope, report, now=now)
ao_ms = (self.clock() - ao_start) * 1000.0
safe_elapsed_ms = (self.clock() - tick_start) * 1000.0
...
g1_elapsed_ms = (self.clock() - tick_start) * 1000.0
total_latency_ms = g1_elapsed_ms - raw_elapsed_ms
```

For stop paths after raw generation, pass `raw_command=step.command` and `raw_elapsed_ms=raw_elapsed_ms` into `_dispatch_stop(...)`.

- [ ] **Step 6: Run all runtime tests**

Run:

```bash
pytest tests/ao/live/test_runtime.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/robot_follow/ao/live/runtime.py tests/ao/live/test_runtime.py
git commit -m "test: lock AO raw safe timing diagnostics"
```

---

### Task 5: Ensure CSV Contains Required 40 Hz Evidence Fields

**Files:**
- Modify: `src/robot_follow/ao/logging/csv_logger.py`
- Modify: `tests/ao/logging/test_csv_logger.py`

**Interfaces:**
- Consumes: `AORadarLogRow`.
- Produces: CSV header and rows containing all FR-11 fields.

- [ ] **Step 1: Strengthen CSV header test**

Replace `test_csv_logger_writes_required_latency_fields` in `tests/ao/logging/test_csv_logger.py` with:

```python
def test_csv_logger_writes_required_40hz_evidence_fields(tmp_path) -> None:
    path = tmp_path / "ao.csv"
    logger = AORadarCSVLogger(path)

    logger.write(
        AORadarLogRow(
            timestamp=10.0,
            alx_age_ms=5.0,
            command_age_ms=3.0,
            cloud_age_ms=20.0,
            loop_dt_ms=25.0,
            ao_process_ms=4.0,
            g1_dispatch_ms=2.0,
            raw_command_kind="move",
            raw_command_reason="follow",
            safe_command_kind="stop",
            safe_command_reason="front_blocked",
            radar_state="running",
            wristband_gate_state="on",
            raw_azimuth_deg=31.0,
            filtered_azimuth_deg=28.5,
            g1_sent_omega=-0.3,
            distance_m=1.25,
            tag_status=3,
            robot_yaw_deg=42.5,
            robot_yaw_rate_rps=-0.27,
            raw_command_generated_at_ms=2.0,
            safe_command_generated_at_ms=5.0,
            g1_response_at_ms=8.0,
            total_latency_ms=6.0,
        )
    )
    logger.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")

    for field in [
        "timestamp",
        "loop_dt_ms",
        "ao_process_ms",
        "g1_dispatch_ms",
        "raw_command_kind",
        "raw_command_reason",
        "safe_command_kind",
        "safe_command_reason",
        "command_age_ms",
        "cloud_age_ms",
        "raw_command_generated_at_ms",
        "safe_command_generated_at_ms",
        "g1_response_at_ms",
        "total_latency_ms",
    ]:
        assert field in header
    assert "25.0" in lines[1]
    assert "front_blocked" in lines[1]
```

- [ ] **Step 2: Run CSV logger tests**

Run:

```bash
pytest tests/ao/logging/test_csv_logger.py -v
```

Expected: PASS if current logger already includes these fields.

- [ ] **Step 3: Fix logger only if the test fails**

If the header omits fields, update `AORadarLogRow` in `src/robot_follow/ao/logging/csv_logger.py` to include:

```python
raw_command_generated_at_ms: float | None = None
safe_command_generated_at_ms: float | None = None
g1_response_at_ms: float | None = None
total_latency_ms: float | None = None
```

Keep the writer using:

```python
self._writer = csv.DictWriter(self._file, fieldnames=list(AORadarLogRow.__dataclass_fields__))
```

- [ ] **Step 4: Commit**

```bash
git add src/robot_follow/ao/logging/csv_logger.py tests/ao/logging/test_csv_logger.py
git commit -m "test: require AO 40hz evidence fields in csv logs"
```

---

### Task 6: Add Offline CSV Acceptance Analyzer

**Files:**
- Create: `scripts/analyze_ao_40hz_log.py`
- Create: `tests/ao/live/test_analyze_ao_40hz_log.py`

**Interfaces:**
- Produces: `analyze_rows(rows: Iterable[Mapping[str, str]]) -> AnalysisResult`.
- Produces: CLI command `python scripts/analyze_ao_40hz_log.py logs/ao_radar_frames_*.csv`.
- Later release gates use this for AC-1, AC-2 and AC-6 evidence.

- [ ] **Step 1: Write analyzer tests**

Create `tests/ao/live/test_analyze_ao_40hz_log.py`:

```python
from scripts.analyze_ao_40hz_log import analyze_rows


def test_analyzer_accepts_25ms_loop_distribution():
    rows = [
        {"loop_dt_ms": str(v), "ao_process_ms": "3.0", "g1_dispatch_ms": "2.0"}
        for v in [0.0, 24.8, 25.0, 25.1, 25.2, 25.3, 25.4, 25.5]
    ]

    result = analyze_rows(rows)

    assert result.row_count == 8
    assert result.loop_p50_ms == 25.15
    assert result.passes_40hz_gate


def test_analyzer_rejects_100ms_loop_distribution():
    rows = [
        {"loop_dt_ms": str(v), "ao_process_ms": "3.0", "g1_dispatch_ms": "2.0"}
        for v in [0.0, 100.0, 100.1, 100.2, 100.3, 100.4, 100.5, 101.0]
    ]

    result = analyze_rows(rows)

    assert not result.passes_40hz_gate
    assert "loop_dt_ms p50" in result.failure_reasons[0]
```

- [ ] **Step 2: Run analyzer tests and verify failure**

Run:

```bash
pytest tests/ao/live/test_analyze_ao_40hz_log.py -v
```

Expected: FAIL with missing `scripts.analyze_ao_40hz_log`.

- [ ] **Step 3: Implement analyzer**

Create `scripts/analyze_ao_40hz_log.py`:

```python
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Iterable, Mapping


@dataclass(frozen=True)
class AnalysisResult:
    row_count: int
    loop_p50_ms: float
    loop_p95_ms: float
    loop_p99_ms: float
    ao_plus_g1_p99_ms: float
    passes_40hz_gate: bool
    failure_reasons: tuple[str, ...]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percentile / 100.0)
    return ordered[int(index)]


def _float_field(row: Mapping[str, str], field: str) -> float:
    value = (row.get(field) or "").strip()
    return 0.0 if value == "" else float(value)


def analyze_rows(rows: Iterable[Mapping[str, str]]) -> AnalysisResult:
    materialized = list(rows)
    loop_values = [_float_field(row, "loop_dt_ms") for row in materialized]
    non_initial_loop_values = [value for value in loop_values if value > 0.0]
    ao_plus_g1 = [
        _float_field(row, "ao_process_ms") + _float_field(row, "g1_dispatch_ms")
        for row in materialized
    ]

    p50 = median(non_initial_loop_values) if non_initial_loop_values else 0.0
    p95 = _percentile(non_initial_loop_values, 95.0)
    p99 = _percentile(non_initial_loop_values, 99.0)
    ao_g1_p99 = _percentile(ao_plus_g1, 99.0)

    failures: list[str] = []
    if not (22.0 <= p50 <= 28.0):
        failures.append(f"loop_dt_ms p50 {p50:.3f} is outside 25 +/- 3 ms")
    if p95 > 30.0:
        failures.append(f"loop_dt_ms p95 {p95:.3f} exceeds 30 ms")
    if p99 > 40.0:
        failures.append(f"loop_dt_ms p99 {p99:.3f} exceeds 40 ms")

    return AnalysisResult(
        row_count=len(materialized),
        loop_p50_ms=float(p50),
        loop_p95_ms=float(p95),
        loop_p99_ms=float(p99),
        ao_plus_g1_p99_ms=float(ao_g1_p99),
        passes_40hz_gate=not failures,
        failure_reasons=tuple(failures),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze AO 40 Hz CSV logs")
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args(argv)

    with args.csv_path.open(newline="", encoding="utf-8") as handle:
        result = analyze_rows(csv.DictReader(handle))

    print(f"rows={result.row_count}")
    print(f"loop_dt_ms p50={result.loop_p50_ms:.3f} p95={result.loop_p95_ms:.3f} p99={result.loop_p99_ms:.3f}")
    print(f"ao_plus_g1_p99_ms={result.ao_plus_g1_p99_ms:.3f}")
    for reason in result.failure_reasons:
        print(f"FAIL: {reason}")
    return 0 if result.passes_40hz_gate else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run analyzer tests**

Run:

```bash
pytest tests/ao/live/test_analyze_ao_40hz_log.py -v
```

Expected: PASS.

- [ ] **Step 5: Run analyzer on the reference 10 Hz log**

Run:

```bash
python scripts/analyze_ao_40hz_log.py logs/ao_radar_frames_20260710_154014.csv
```

Expected: exit code 1 and output showing `loop_dt_ms p50` near 100 ms, confirming the analyzer catches the existing failure.

- [ ] **Step 6: Commit**

```bash
git add scripts/analyze_ao_40hz_log.py tests/ao/live/test_analyze_ao_40hz_log.py
git commit -m "test: add AO 40hz log acceptance analyzer"
```

---

### Task 7: Final Verification Against the Spec

**Files:**
- Modify only if earlier tasks exposed a direct mismatch:
  - `configs/jetson_g1_ao_radar.yaml`
  - `docs/superpowers/specs/2026-07-10-ao-40hz-latest-safe-output-design.md`

**Interfaces:**
- Consumes all previous tasks.
- Produces final evidence for AC-1 through AC-6.

- [ ] **Step 1: Verify config still requests 40 Hz**

Run:

```bash
python - <<'PY'
from pathlib import Path
from robot_follow.ao.config import load_ao_radar_config
cfg = load_ao_radar_config(Path("configs/jetson_g1_ao_radar.yaml"))
print(cfg.latency.ao_output_loop_hz)
assert cfg.latency.ao_output_loop_hz == 40.0
PY
```

Expected: prints `40.0`.

- [ ] **Step 2: Run the focused test suite**

Run:

```bash
pytest tests/ao/live/test_loop_timing.py tests/ao/live/test_cli.py tests/ao/live/test_web_cli.py tests/ao/live/test_runtime.py tests/ao/logging/test_csv_logger.py tests/ao/live/test_analyze_ao_40hz_log.py -v
```

Expected: PASS.

- [ ] **Step 3: Run a mock 120-second AO log capture**

Use mock mode or a lab-safe config that does not move the real G1. The command should produce a timestamped `logs/ao_radar_frames_*.csv`.

```bash
robot-follow-ao-radar --config configs/jetson_g1_ao_radar.yaml
```

Stop after at least 120 seconds with Ctrl-C.

Expected: a new CSV log exists under `logs/`.

- [ ] **Step 4: Analyze the new log**

Run:

```bash
python - <<'PY'
from pathlib import Path
from scripts.analyze_ao_40hz_log import main
logs = sorted(Path("logs").glob("ao_radar_frames_*.csv"), key=lambda path: path.stat().st_mtime)
assert logs, "no AO radar CSV logs found"
raise SystemExit(main([str(logs[-1])]))
PY
```

Expected: exit code 0, `loop_dt_ms p50` in 25 +/- 3 ms, p95 <= 30 ms, p99 <= 40 ms.

- [ ] **Step 5: Confirm the reference log still fails**

Run:

```bash
python scripts/analyze_ao_40hz_log.py logs/ao_radar_frames_20260710_154014.csv
```

Expected: exit code 1 with `loop_dt_ms p50` near 100 ms. This proves the validator distinguishes old 10 Hz behavior from the target 40 Hz behavior.

- [ ] **Step 6: Commit final verification updates**

```bash
git add configs/jetson_g1_ao_radar.yaml docs/superpowers/specs/2026-07-10-ao-40hz-latest-safe-output-design.md
git commit -m "docs: record AO 40hz verification gates"
```

Skip this commit if neither file changed.

---

## Spec Coverage Review

- FR-1, FR-3, NFR-1: Task 1 and Task 2.
- FR-2, EC-2: Task 3.
- FR-4, FR-5, FR-6, FR-7, FR-8, FR-10, NFR-3, NFR-4: Task 4.
- FR-9: Task 4 preserves synchronous bridge dispatch and `g1_dispatch_ms`.
- FR-11, AC-6: Task 5 and Task 6.
- FR-12: Task 7.
- FR-13, NFR-2: Task 6 exposes delay source evidence; Task 7 validates logs.
- AC-1: Task 2, Task 6 and Task 7.
- AC-2: Task 3 and Task 7.
- AC-3, AC-4, AC-5: Task 4.
- Non-goals: No task adds async sender, mailbox, stop epoch, planner redesign, physical feedback, or Web UI redesign.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-10-ao-40hz-latest-safe-output.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
