# Core Quality Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix quality and safety issues in the existing `core` modules, including improvements to the already-created `core/pipeline.py` orchestration layer.

**Architecture:** Phase 1 covers module-level hardening in `types.py`, `filtering.py`, `safety.py`, `follow_controller.py`, and their tests. Phase 2 assumes `FollowPipeline` and `PipelineStepResult` already exist and focuses on correctness, safety semantics, exports, and test coverage improvements.

**Tech Stack:** Python 3.10, dataclasses, pytest, Conda environment `robot-follow`.

---

## Scope

### Phase 1: Do Now

- Modify: `src/robot_follow/core/safety.py`
  - Reject non-finite numeric inputs before any motion command can be computed downstream.
- Modify: `src/robot_follow/core/follow_controller.py`
  - Fix angular gain semantics so configured `max_omega` is reachable under realistic heading errors.
- Modify: `src/robot_follow/core/filtering.py`
  - Add wrap-aware angle delta for jump detection.
- Modify: `src/robot_follow/core/types.py`
  - Add runtime validation for `MotionCommand.kind` and stop-command zero velocity.
- Modify or create tests under `tests/core/`
  - Add focused unit tests for the above behaviors only.

### Phase 2: Pipeline Improvements

- Modify: `src/robot_follow/core/pipeline.py`
  - Add pre-filter validation, consistent safety verdicts, safety event inputs, follow output reporting, and clearer constructor naming.
- Modify: `src/robot_follow/core/__init__.py`
  - Export `FollowPipeline` and `PipelineStepResult`.
- Modify: `tests/core/test_pipeline.py`
  - Add focused regression tests for pipeline safety and API behavior.

## Task 1: Safety Rejects Non-Finite Inputs

**Files:**
- Modify: `src/robot_follow/core/safety.py`
- Modify: `tests/core/test_safety.py`

- [ ] **Step 1: Write failing tests**

Add these imports and tests to `tests/core/test_safety.py`:

```python
import math
```

```python
def test_rejects_nan_distance():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_rejects_nan_azimuth():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=math.nan,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"


def test_rejects_infinite_jump_values():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=1.5,
        azimuth_deg=0.0,
        distance_jump=math.inf,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "data_invalid"
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_safety.py -q
```

Expected: at least one new test fails because `SafetyManager` does not yet check `math.isfinite`.

- [ ] **Step 3: Implement finite validation**

In `src/robot_follow/core/safety.py`, add:

```python
import math
```

Then add this block after the `data_valid` check and before timeout checks:

```python
        numeric_values = (
            inp.timestamp,
            inp.distance_m,
            inp.azimuth_deg,
            inp.distance_jump,
            inp.angle_jump,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            return SafetyVerdict(safe=False, reason="data_invalid")
```

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_safety.py -q
```

Expected: all safety tests pass.

## Task 2: Fix Angular Velocity Gain Semantics

**Files:**
- Modify: `src/robot_follow/core/follow_controller.py`
- Modify: `tests/core/test_follow_controller.py`

- [ ] **Step 1: Write magnitude test**

Add to `tests/core/test_follow_controller.py`:

```python
def test_large_angle_uses_meaningful_turn_rate():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=30.0)
    assert out.reason == "aligning"
    assert 0.05 <= abs(out.omega) <= ctrl.config.max_omega
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_follow_controller.py::test_large_angle_uses_meaningful_turn_rate -q
```

Expected: fails because current 30 degree output is about `0.00785`.

- [ ] **Step 3: Implement fix**

Recommended interpretation for Phase 1: keep `azimuth_deg` as the public input and make `angle_gain` per degree.

In `src/robot_follow/core/follow_controller.py`, remove the unused `math` import after changing the formulas.

Change:

```python
    angle_gain: float = 0.015     # omega = gain * azimuth_rad
```

to:

```python
    angle_gain: float = 0.01      # omega = gain * azimuth_deg
```

Replace both occurrences of:

```python
cfg.angle_gain * math.radians(az)
```

with:

```python
cfg.angle_gain * az
```

Replace the deadzone expression:

```python
cfg.angle_gain * math.radians(az) * 0.3
```

with:

```python
cfg.angle_gain * az * 0.3
```

- [ ] **Step 4: Verify controller tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_follow_controller.py -q
```

Expected: all controller tests pass.

## Task 3: Add Wrap-Aware Angle Jump Detection

**Files:**
- Modify: `src/robot_follow/core/filtering.py`
- Modify: `tests/core/test_filtering.py`

- [ ] **Step 1: Write failing test**

Add to `tests/core/test_filtering.py`:

```python
def test_angle_jump_uses_shortest_wrapped_delta():
    f = TargetFilter(alpha=1.0)
    f.update(distance_m=2.0, azimuth_deg=179.0)
    result = f.update(distance_m=2.0, azimuth_deg=-179.0)
    assert result.angle_jump == 2.0
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_filtering.py::test_angle_jump_uses_shortest_wrapped_delta -q
```

Expected: fails with `angle_jump == 358.0`.

- [ ] **Step 3: Implement helper**

Add this helper near the top of `src/robot_follow/core/filtering.py`:

```python
def _angle_delta_deg(current: float, previous: float) -> float:
    return abs((current - previous + 180.0) % 360.0 - 180.0)
```

Replace:

```python
angle_jump = abs(azimuth_deg - prev_a)
```

with:

```python
angle_jump = _angle_delta_deg(float(azimuth_deg), prev_a)
```

- [ ] **Step 4: Verify filtering tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_filtering.py -q
```

Expected: all filtering tests pass.

## Task 4: Add Runtime Validation To MotionCommand

**Files:**
- Modify: `src/robot_follow/core/types.py`
- Create: `tests/core/test_types.py`

- [ ] **Step 1: Write failing tests**

Create `tests/core/test_types.py`:

```python
import pytest

from robot_follow.core.types import MotionCommand


def test_motion_command_rejects_unknown_kind():
    with pytest.raises(ValueError):
        MotionCommand(kind="pause", vx=0.0, vy=0.0, omega=0.0, reason="bad")


def test_stop_command_must_have_zero_velocity():
    with pytest.raises(ValueError):
        MotionCommand(kind="stop", vx=0.1, vy=0.0, omega=0.0, reason="bad")


def test_move_command_accepts_zero_velocity_for_holding():
    cmd = MotionCommand(kind="move", vx=0.0, vy=0.0, omega=0.0, reason="holding")
    assert cmd.kind == "move"
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_types.py -q
```

Expected: first two tests fail because validation does not exist.

- [ ] **Step 3: Implement validation**

Add to `MotionCommand` in `src/robot_follow/core/types.py`:

```python
    def __post_init__(self) -> None:
        if self.kind not in {"move", "stop"}:
            raise ValueError(f"invalid motion command kind: {self.kind}")
        if self.kind == "stop" and (
            abs(self.vx) > 1e-12 or abs(self.vy) > 1e-12 or abs(self.omega) > 1e-12
        ):
            raise ValueError("stop command velocities must be zero")
```

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_types.py -q
```

Expected: all type tests pass.

## Task 5: Keep Controller/Safety Responsibility Compatible For Phase 1

**Files:**
- Review only: `src/robot_follow/core/follow_controller.py`
- Review only: `src/robot_follow/core/safety.py`
- Review only: `tests/core/test_follow_controller.py`
- Review only: `tests/core/test_safety.py`

- [ ] **Step 1: Preserve current public behavior**

For Phase 1, keep the existing `FollowController.compute(distance_m=0.6, azimuth_deg=0.0)` behavior returning `reason == "too_close"` because there is no pipeline yet to make this the canonical safety-stop boundary.

The existing test should remain:

```python
def test_too_close_stop():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=0.6, azimuth_deg=0.0)
    assert out.vx == 0.0
    assert out.omega == 0.0
    assert out.reason == "too_close"
```

- [ ] **Step 2: Document Phase 2 responsibility**

Do not change code in this task. Record this design note in the PR/commit message or follow-up issue:

```text
Phase 2 will introduce FollowPipeline. At that point SafetyManager/FollowPipeline should own forced stops, while FollowController should only map already-safe filtered target state to velocity.
```

- [ ] **Step 3: Verify existing tests still cover compatibility**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_follow_controller.py tests/core/test_safety.py -q
```

Expected: all selected tests pass.

## Task 6: Final Verification For Phase 1

**Files:**
- All modified existing core and tests files.

- [ ] **Step 1: Compile core**

Run:

```bash
conda run -n robot-follow python -m compileall -q src/robot_follow/core tests/core
```

Expected: no output and exit code 0.

- [ ] **Step 2: Run core tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core -q
```

Expected: all core tests pass.

- [ ] **Step 3: Run full tests**

Run:

```bash
conda run -n robot-follow python -m pytest -q
```

Expected: all repository tests pass.

- [ ] **Step 4: Verify pipeline tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

## Phase 2: Pipeline Improvement Plan

Pipeline construction is now complete. Do not create a new `pipeline.py` task. Replace any remaining "create pipeline" work with the following improvement tasks.

### Pipeline Review Findings

- **High:** `FollowPipeline.process()` updates `TargetFilter` before any non-finite numeric validation. A valid sample with `distance_m=nan` enters `TargetFilter.update()` at `src/robot_follow/core/pipeline.py:128`, poisons filter state, and can keep later outputs unsafe or stale. Add a pre-filter input validation path.
- **High:** `robot_follow.core` does not export `FollowPipeline` or `PipelineStepResult`. `from robot_follow.core import FollowPipeline` currently fails because `src/robot_follow/core/__init__.py` only exports types, filter, safety, and controller symbols.
- **Important:** Invalid samples return `PipelineStepResult(verdict=None)` at `src/robot_follow/core/pipeline.py:120-125`. This makes downstream logging and safety accounting branch on `None`, even though invalid data is a concrete safety verdict. Return `SafetyVerdict(safe=False, reason="data_invalid")` for invalid samples.
- **Important:** The pipeline has no way to carry `manual_stop`, `serial_error`, or `sdk_error` into `SafetyInput`, even though `SafetyManager` supports those fields. This means the central pipeline cannot be the only reusable safety path for real runners unless those errors are handled outside the pipeline.
- **Medium:** `PipelineStepResult` drops the raw `FollowOutput`. The command preserves velocity and reason, but tests and logs cannot distinguish controller output from final command mapping.
- **Medium:** `FollowPipeline.__init__()` uses a parameter named `filter`, shadowing Python's built-in `filter`. This is not a runtime bug, but the name is easy to misread. Prefer `target_filter`.

### Task 7: Pre-Filter Numeric Validation In Pipeline

**Files:**
- Modify: `src/robot_follow/core/pipeline.py`
- Modify: `tests/core/test_pipeline.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/core/test_pipeline.py`:

```python
import math
```

```python
def test_nan_sample_stops_and_does_not_update_filter():
    pipe = FollowPipeline(clock=lambda: 100.0)
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "stop"
    assert result.command.reason == "data_invalid"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_nan_sample_does_not_poison_next_valid_sample():
    pipe = FollowPipeline(clock=lambda: 100.0)
    pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        valid=True,
    ))
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=2.0,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "move"
    assert result.filtered is not None
    assert result.filtered.distance_m == 2.0
    assert result.filtered.distance_jump == 0.0
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py::test_nan_sample_stops_and_does_not_update_filter tests/core/test_pipeline.py::test_nan_sample_does_not_poison_next_valid_sample -q
```

Expected: tests fail because the current pipeline calls `TargetFilter.update()` before numeric validation.

- [ ] **Step 3: Implement pre-filter validation**

In `src/robot_follow/core/pipeline.py`, add:

```python
import math
```

Add helper:

```python
def _invalid_result(sample: TargetSample, reason: str) -> PipelineStepResult:
    verdict = SafetyVerdict(safe=False, reason=reason)
    return PipelineStepResult(
        sample=sample,
        filtered=None,
        verdict=verdict,
        command=_stop_command(reason),
    )
```

At the start of `FollowPipeline.process()`, replace the invalid-sample return with:

```python
        if not sample.valid:
            return _invalid_result(sample, "data_invalid")

        if not all(math.isfinite(value) for value in (
            sample.timestamp,
            sample.distance_m,
            sample.azimuth_deg,
        )):
            return _invalid_result(sample, "data_invalid")
```

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

### Task 8: Export Pipeline Symbols From Core Package

**Files:**
- Modify: `src/robot_follow/core/__init__.py`
- Modify: `tests/test_backward_compat.py` or `tests/core/test_pipeline.py`

- [ ] **Step 1: Write failing import test**

Add to `tests/core/test_pipeline.py`:

```python
def test_pipeline_exported_from_core_package():
    from robot_follow.core import FollowPipeline as ExportedFollowPipeline
    from robot_follow.core import PipelineStepResult as ExportedPipelineStepResult

    assert ExportedFollowPipeline is FollowPipeline
    assert ExportedPipelineStepResult is PipelineStepResult
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py::test_pipeline_exported_from_core_package -q
```

Expected: import failure because `FollowPipeline` and `PipelineStepResult` are not exported from `robot_follow.core`.

- [ ] **Step 3: Export symbols**

In `src/robot_follow/core/__init__.py`, add:

```python
from robot_follow.core.pipeline import FollowPipeline, PipelineStepResult
```

Add to `__all__`:

```python
    "FollowPipeline",
    "PipelineStepResult",
```

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

### Task 9: Add Pipeline Safety Event Inputs

**Files:**
- Modify: `src/robot_follow/core/pipeline.py`
- Modify: `tests/core/test_pipeline.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/core/test_pipeline.py`:

```python
def test_pipeline_manual_stop_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), manual_stop=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "manual_stop"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_pipeline_serial_error_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), serial_error=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "serial_error"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_pipeline_sdk_error_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), sdk_error=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "sdk_error"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py::test_pipeline_manual_stop_stops_before_filter_update tests/core/test_pipeline.py::test_pipeline_serial_error_stops_before_filter_update tests/core/test_pipeline.py::test_pipeline_sdk_error_stops_before_filter_update -q
```

Expected: tests fail because `FollowPipeline.process()` does not accept these keyword arguments.

- [ ] **Step 3: Extend process signature and safety check**

Change the method signature in `src/robot_follow/core/pipeline.py`:

```python
    def process(
        self,
        sample: TargetSample,
        *,
        manual_stop: bool = False,
        serial_error: bool = False,
        sdk_error: bool = False,
    ) -> PipelineStepResult:
```

At the start of `process()`, before filter update, add:

```python
        pre_filter_safety = SafetyInput(
            timestamp=sample.timestamp,
            distance_m=sample.distance_m,
            azimuth_deg=sample.azimuth_deg,
            distance_jump=0.0,
            angle_jump=0.0,
            data_valid=sample.valid,
            manual_stop=manual_stop,
            serial_error=serial_error,
            sdk_error=sdk_error,
        )
        pre_filter_verdict = self._safety.check(pre_filter_safety)
        if not pre_filter_verdict.safe and pre_filter_verdict.reason in {
            "manual_stop",
            "serial_error",
            "sdk_error",
            "data_invalid",
        }:
            return PipelineStepResult(
                sample=sample,
                filtered=None,
                verdict=pre_filter_verdict,
                command=_stop_command(pre_filter_verdict.reason),
            )
```

When building the post-filter `SafetyInput`, pass through:

```python
            manual_stop=manual_stop,
            serial_error=serial_error,
            sdk_error=sdk_error,
```

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

### Task 10: Include FollowOutput In PipelineStepResult

**Files:**
- Modify: `src/robot_follow/core/pipeline.py`
- Modify: `tests/core/test_pipeline.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/core/test_pipeline.py`:

```python
def test_safe_move_result_keeps_follow_output():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    assert result.follow is not None
    assert result.follow.vx == result.command.vx
    assert result.follow.vy == result.command.vy
    assert result.follow.omega == result.command.omega
    assert result.follow.reason == result.command.reason


def test_stop_result_has_no_follow_output():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=0.5, azimuth_deg=0.0))

    assert result.command.kind == "stop"
    assert result.follow is None
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py::test_safe_move_result_keeps_follow_output tests/core/test_pipeline.py::test_stop_result_has_no_follow_output -q
```

Expected: tests fail because `PipelineStepResult` has no `follow` field.

- [ ] **Step 3: Extend result type**

In `src/robot_follow/core/pipeline.py`, update `PipelineStepResult`:

```python
    follow: Optional[FollowOutput]
```

Set `follow=None` for invalid and unsafe returns. Set `follow=output` for safe controller returns.

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

### Task 11: Rename Constructor Parameter `filter`

**Files:**
- Modify: `src/robot_follow/core/pipeline.py`
- Modify: `tests/core/test_pipeline.py`

- [ ] **Step 1: Write compatibility test**

Add to `tests/core/test_pipeline.py`:

```python
def test_pipeline_accepts_target_filter_keyword():
    target_filter = TargetFilter(alpha=1.0)
    pipe = FollowPipeline(target_filter=target_filter)
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    assert pipe.filter is target_filter
    assert result.filtered is not None
    assert result.filtered.distance_m == 2.0
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py::test_pipeline_accepts_target_filter_keyword -q
```

Expected: fails because constructor currently uses `filter=`.

- [ ] **Step 3: Rename argument**

Change constructor signature:

```python
        target_filter: Optional[TargetFilter] = None,
```

Replace:

```python
        self._filter = filter or TargetFilter()
```

with:

```python
        self._filter = target_filter or TargetFilter()
```

Do not keep a `filter` compatibility alias unless existing callers outside tests already use it.

- [ ] **Step 4: Verify**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

### Task 12: Final Pipeline Verification

**Files:**
- All modified core pipeline and test files.

- [ ] **Step 1: Compile core**

Run:

```bash
conda run -n robot-follow python -m compileall -q src/robot_follow/core tests/core
```

Expected: no output and exit code 0.

- [ ] **Step 2: Run pipeline tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core/test_pipeline.py -q
```

Expected: all pipeline tests pass.

- [ ] **Step 3: Run full core tests**

Run:

```bash
conda run -n robot-follow python -m pytest tests/core -q
```

Expected: all core tests pass.

- [ ] **Step 4: Run full test suite**

Run:

```bash
conda run -n robot-follow python -m pytest -q
```

Expected: all repository tests pass.

## Self-Review

- Spec coverage: Phase 1 covers non-finite input rejection, angular velocity magnitude, angle wrap jump detection, `MotionCommand` validation, and current controller/safety compatibility. Phase 2 now covers improvement of the already-created pipeline.
- Pipeline scope: pipeline creation tasks have been removed and replaced with pipeline improvement tasks.
- Placeholder scan: No task contains TBD/TODO/fill-in placeholders.
- Type consistency: `TargetSample`, `MotionCommand`, `SafetyInput`, `TargetFilter`, `SafetyManager`, and `FollowController` names match the current core API.
