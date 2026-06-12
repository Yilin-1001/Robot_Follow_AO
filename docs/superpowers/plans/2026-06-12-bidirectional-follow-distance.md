# Bidirectional Follow Distance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the robot maintain a realistic human-following distance by moving forward, holding, or reversing within configured distance bands.

**Architecture:** Keep the existing core pipeline. `SafetyManager` remains the hard safety gate for invalid data, timeouts, jumps, manual stops, SDK/serial errors, and `distance < 0.5m`. `FollowController` computes bidirectional velocity inside the valid follow range, while `FollowPipeline` converts `distance > 3.0m` into a real `stop` command so far-target stopping is distinct from holding.

**Tech Stack:** Python 3, dataclasses, pytest, existing `robot_follow.core` modules and backward-compatible re-export modules.

---

## Pre-Execution Notes

Before running pytest or any Python project command, follow the repository instruction in `CLAUDE.md`: use the `robot-follow-conda-run-guard` skill, run `conda env list`, and get the user's confirmation for which conda environment to use.

Do not touch the existing unstaged `.gitignore` change unless the user explicitly asks.

## File Structure

- Modify `src/robot_follow/core/follow_controller.py`
  - Owns follow distance configuration and velocity calculation.
  - Add realistic distance band parameters and asymmetric forward/backward speed limits.
- Modify `src/robot_follow/core/safety.py`
  - Owns hard safety stops.
  - Change default `min_safe_distance_m` from `0.75` to `0.5`.
- Modify `src/robot_follow/core/pipeline.py`
  - Owns conversion from per-frame inputs to `MotionCommand`.
  - Add a far-target stop before calling the controller.
- Modify `tests/core/test_follow_controller.py`
  - Primary tests for bidirectional distance control.
- Modify `tests/core/test_safety.py`
  - Primary tests for the new minimum safe distance.
- Modify `tests/core/test_pipeline.py`
  - Pipeline tests for negative `vx` and far-target stop command kind.
- Modify `tests/test_follow_controller.py`
  - Legacy import-path mirror for controller behavior.
- Modify `tests/test_safety.py`
  - Legacy import-path mirror for safety behavior.
- Leave `src/robot_follow/follow_controller.py` and `src/robot_follow/safety.py` unchanged unless imports fail; they are re-export wrappers.

---

### Task 1: Add Core Controller Tests For Bidirectional Distance Bands

**Files:**
- Modify: `tests/core/test_follow_controller.py`
- Test: `tests/core/test_follow_controller.py`

- [ ] **Step 1: Replace and extend distance behavior tests**

Update `tests/core/test_follow_controller.py` so the core behavior is expressed with the new realistic distance bands:

```python
"""Unit tests for the follow controller."""

import pytest

from robot_follow.core.follow_controller import FollowConfig, FollowController


def test_too_close_stop():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=0.49, azimuth_deg=0.0)
    assert out.vx == 0.0
    assert out.omega == 0.0
    assert out.reason == "too_close"


def test_large_angle_turn_in_place():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=30.0)
    assert out.vx == 0.0
    assert out.omega != 0.0
    assert out.reason == "aligning"


@pytest.mark.parametrize("distance_m", [1.35, 1.5, 1.65])
def test_hold_band_holds_position(distance_m):
    ctrl = FollowController()
    out = ctrl.compute(distance_m=distance_m, azimuth_deg=0.0)
    assert abs(out.vx) < 0.01
    assert abs(out.omega) < 0.01
    assert out.reason == "holding"


def test_close_but_safe_distance_reverses():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=1.0, azimuth_deg=0.0)
    assert out.vx < 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_backward"


def test_backward_speed_is_clamped():
    ctrl = FollowController(FollowConfig(max_backward_vx=0.05))
    out = ctrl.compute(distance_m=0.5, azimuth_deg=0.0)
    assert out.vx == pytest.approx(-0.05)
    assert out.reason == "tracking_backward"


def test_far_distance_moves_forward():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=0.0)
    assert out.vx > 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_forward"


def test_forward_speed_is_clamped():
    ctrl = FollowController(FollowConfig(max_forward_vx=0.05))
    out = ctrl.compute(distance_m=3.0, azimuth_deg=0.0)
    assert out.vx == pytest.approx(0.05)
    assert out.reason == "tracking_forward"


def test_medium_angle_slow_forward_and_turn():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=10.0)
    assert out.vx > 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_turning"


def test_medium_angle_slow_backward_and_turn():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=1.0, azimuth_deg=-10.0)
    assert out.vx < 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_turning"
```

- [ ] **Step 2: Run core controller tests and verify failure**

Run:

```bash
pytest tests/core/test_follow_controller.py -v
```

Expected: FAIL because `FollowConfig` does not accept `max_backward_vx` or `max_forward_vx`, default `0.49m` may still be `too_close`, and close-but-safe distances currently clamp negative `vx` to zero.

---

### Task 2: Implement Bidirectional Follow Controller

**Files:**
- Modify: `src/robot_follow/core/follow_controller.py`
- Test: `tests/core/test_follow_controller.py`

- [ ] **Step 1: Update `FollowConfig` fields**

Replace the existing `FollowConfig` dataclass with:

```python
@dataclass
class FollowConfig:
    """Tunable parameters for the follow controller."""

    target_distance_m: float = 1.5
    min_safe_distance_m: float = 0.5
    hold_tolerance_m: float = 0.15
    max_forward_vx: float = 0.20
    max_backward_vx: float = 0.12
    max_follow_distance_m: float = 3.0
    max_omega: float = 0.30       # rad/s
    angle_deadzone_deg: float = 5.0
    large_angle_deg: float = 20.0
    distance_gain_forward: float = 0.18
    distance_gain_backward: float = 0.15
    angle_gain: float = 0.015     # omega = gain * azimuth_rad

    @property
    def hold_min_distance_m(self) -> float:
        return self.target_distance_m - self.hold_tolerance_m

    @property
    def hold_max_distance_m(self) -> float:
        return self.target_distance_m + self.hold_tolerance_m
```

- [ ] **Step 2: Add distance velocity helper**

Inside `FollowController`, add this helper before `_clamp_omega`:

```python
    def _distance_vx(self, distance_m: float) -> float:
        cfg = self._config
        if distance_m < cfg.hold_min_distance_m:
            error = distance_m - cfg.hold_min_distance_m
            return max(
                -cfg.max_backward_vx,
                cfg.distance_gain_backward * error,
            )
        if distance_m > cfg.hold_max_distance_m:
            error = distance_m - cfg.hold_max_distance_m
            return min(
                cfg.max_forward_vx,
                cfg.distance_gain_forward * error,
            )
        return 0.0
```

- [ ] **Step 3: Update `compute()` to use bidirectional velocity**

In `compute()`, keep the `too_close` and large-angle branches. Replace both existing `dist_err`/`vx = max(0.0, ...)` blocks with calls to `_distance_vx(dist)`.

Medium-angle branch should become:

```python
        if abs_az > cfg.angle_deadzone_deg:
            omega = self._clamp_omega(
                cfg.angle_gain * math.radians(az)
            )
            vx = self._distance_vx(dist)
            vx *= 1.0 - (abs_az / cfg.large_angle_deg) * 0.6
            return FollowOutput(
                vx=vx, vy=0.0, omega=omega, reason="tracking_turning"
            )
```

Small-angle branch should become:

```python
        vx = self._distance_vx(dist)
        omega = self._clamp_omega(
            cfg.angle_gain * math.radians(az) * 0.3
        )
        if vx < -1e-4:
            reason = "tracking_backward"
        elif vx > 1e-4:
            reason = "tracking_forward"
        elif abs(omega) < 1e-4:
            reason = "holding"
        else:
            reason = "tracking_turning"
        return FollowOutput(vx=vx, vy=0.0, omega=omega, reason=reason)
```

- [ ] **Step 4: Remove obsolete `_clamp_vx()`**

Delete the old `_clamp_vx()` method:

```python
    def _clamp_vx(self, vx: float) -> float:
        return max(-self._config.max_vx, min(self._config.max_vx, vx))
```

- [ ] **Step 5: Run core controller tests**

Run:

```bash
pytest tests/core/test_follow_controller.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit controller changes**

Run:

```bash
git add src/robot_follow/core/follow_controller.py tests/core/test_follow_controller.py
git commit -m "feat: add bidirectional follow controller"
```

---

### Task 3: Update Safety Minimum Distance Tests And Defaults

**Files:**
- Modify: `tests/core/test_safety.py`
- Modify: `tests/test_safety.py`
- Modify: `src/robot_follow/core/safety.py`
- Test: `tests/core/test_safety.py`, `tests/test_safety.py`

- [ ] **Step 1: Update core safety tests**

In `tests/core/test_safety.py`, replace `test_too_close()` with these two tests:

```python
def test_distance_at_minimum_safe_boundary_is_safe():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=0.5,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert v.safe
    assert v.reason == "ok"


def test_below_minimum_safe_distance_is_too_close():
    mgr = SafetyManager(clock=lambda: 100.0)
    v = mgr.check(SafetyInput(
        timestamp=100.0,
        distance_m=0.49,
        azimuth_deg=0.0,
        distance_jump=0.0,
        angle_jump=0.0,
    ))
    assert not v.safe
    assert v.reason == "too_close"
```

- [ ] **Step 2: Update legacy safety tests**

In `tests/test_safety.py`, replace `test_too_close()` with:

```python
def test_distance_at_minimum_safe_boundary_is_safe():
    mgr = SafetyManager()
    v = mgr.check(_make_input(distance_m=0.5))
    assert v.safe
    assert v.reason == "ok"


def test_below_minimum_safe_distance_is_too_close():
    mgr = SafetyManager()
    v = mgr.check(_make_input(distance_m=0.49))
    assert not v.safe
    assert v.reason == "too_close"
```

- [ ] **Step 3: Run safety tests and verify failure**

Run:

```bash
pytest tests/core/test_safety.py tests/test_safety.py -v
```

Expected: FAIL because `SafetyConfig.min_safe_distance_m` still defaults to `0.75`, so `distance_m=0.5` is incorrectly unsafe.

- [ ] **Step 4: Update safety default**

In `src/robot_follow/core/safety.py`, change:

```python
    min_safe_distance_m: float = 0.75
```

to:

```python
    min_safe_distance_m: float = 0.5
```

- [ ] **Step 5: Run safety tests**

Run:

```bash
pytest tests/core/test_safety.py tests/test_safety.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit safety changes**

Run:

```bash
git add src/robot_follow/core/safety.py tests/core/test_safety.py tests/test_safety.py
git commit -m "feat: lower follow minimum safe distance"
```

---

### Task 4: Add Pipeline Behavior For Reverse Motion And Far-Target Stop

**Files:**
- Modify: `tests/core/test_pipeline.py`
- Modify: `src/robot_follow/core/pipeline.py`
- Test: `tests/core/test_pipeline.py`

- [ ] **Step 1: Add pipeline tests**

Add these tests near the normal movement and safety tests in `tests/core/test_pipeline.py`:

```python
def test_close_but_safe_distance_reverses():
    pipe = FollowPipeline(clock=lambda: 100.0)
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=1.0,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "move"
    assert result.command.vx < 0.0
    assert result.command.omega == pytest.approx(0.0)
    assert result.command.reason == "tracking_backward"
    assert result.verdict is not None
    assert result.verdict.safe


def test_distance_at_minimum_safe_boundary_can_reverse():
    pipe = FollowPipeline(clock=lambda: 100.0)
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=0.5,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "move"
    assert result.command.vx < 0.0
    assert result.command.reason == "tracking_backward"


def test_far_target_stops_instead_of_chasing():
    pipe = FollowPipeline(clock=lambda: 100.0)
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=3.1,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "stop"
    assert result.command.reason == "target_too_far"
    assert result.command.vx == 0.0
    assert result.command.omega == 0.0
    assert result.follow is None
```

- [ ] **Step 2: Run pipeline tests and verify failure**

Run:

```bash
pytest tests/core/test_pipeline.py -v
```

Expected: FAIL because close-but-safe distances currently do not reverse, and `distance_m=3.1` currently becomes a forward `move` command instead of a `stop`.

- [ ] **Step 3: Add far-target stop in pipeline**

In `src/robot_follow/core/pipeline.py`, after the safety verdict is known safe and before calling `self._controller.compute(...)`, add:

```python
        if filtered.distance_m > self._controller.config.max_follow_distance_m:
            command = _stop_command("target_too_far")
            return PipelineStepResult(
                sample=sample,
                filtered=filtered,
                verdict=verdict,
                command=command,
            )
```

The new block belongs after:

```python
        if not verdict.safe:
            command = _stop_command(verdict.reason)
            return PipelineStepResult(
                sample=sample,
                filtered=filtered,
                verdict=verdict,
                command=command,
            )
```

and before:

```python
        output = self._controller.compute(
            filtered.distance_m, filtered.azimuth_deg
        )
```

- [ ] **Step 4: Run pipeline tests**

Run:

```bash
pytest tests/core/test_pipeline.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit pipeline changes**

Run:

```bash
git add src/robot_follow/core/pipeline.py tests/core/test_pipeline.py
git commit -m "feat: stop follow pipeline when target is too far"
```

---

### Task 5: Update Legacy Controller Import Tests

**Files:**
- Modify: `tests/test_follow_controller.py`
- Test: `tests/test_follow_controller.py`, `tests/test_backward_compat.py`

- [ ] **Step 1: Mirror core controller expectations in legacy tests**

Update `tests/test_follow_controller.py` to:

```python
"""Unit tests for the follow controller."""

import pytest

from robot_follow.follow_controller import FollowConfig, FollowController


def test_too_close_stop():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=0.49, azimuth_deg=0.0)
    assert out.vx == 0.0
    assert out.omega == 0.0
    assert out.reason == "too_close"


def test_large_angle_turn_in_place():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=30.0)
    assert out.vx == 0.0
    assert out.omega != 0.0
    assert out.reason == "aligning"


@pytest.mark.parametrize("distance_m", [1.35, 1.5, 1.65])
def test_hold_band_holds_position(distance_m):
    ctrl = FollowController()
    out = ctrl.compute(distance_m=distance_m, azimuth_deg=0.0)
    assert abs(out.vx) < 0.01
    assert abs(out.omega) < 0.01
    assert out.reason == "holding"


def test_close_but_safe_distance_reverses():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=1.0, azimuth_deg=0.0)
    assert out.vx < 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_backward"


def test_backward_speed_is_clamped():
    ctrl = FollowController(FollowConfig(max_backward_vx=0.05))
    out = ctrl.compute(distance_m=0.5, azimuth_deg=0.0)
    assert out.vx == pytest.approx(-0.05)
    assert out.reason == "tracking_backward"


def test_far_distance_moves_forward():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=0.0)
    assert out.vx > 0.0
    assert out.omega == pytest.approx(0.0)
    assert out.reason == "tracking_forward"


def test_medium_angle_slow_forward_and_turn():
    ctrl = FollowController()
    out = ctrl.compute(distance_m=2.0, azimuth_deg=10.0)
    assert out.vx > 0.0
    assert out.omega != 0.0
    assert out.reason == "tracking_turning"
```

- [ ] **Step 2: Run legacy controller and compatibility tests**

Run:

```bash
pytest tests/test_follow_controller.py tests/test_backward_compat.py -v
```

Expected: PASS.

- [ ] **Step 3: Commit legacy test updates**

Run:

```bash
git add tests/test_follow_controller.py
git commit -m "test: update legacy follow controller expectations"
```

---

### Task 6: Full Verification And Documentation Check

**Files:**
- Read: `docs/superpowers/specs/2026-06-12-bidirectional-follow-distance-design.md`
- Test: full test suite

- [ ] **Step 1: Run full test suite**

Run:

```bash
pytest -q
```

Expected: PASS for the full suite.

- [ ] **Step 2: Check final diff**

Run:

```bash
git diff --stat HEAD
git diff -- src/robot_follow/core/follow_controller.py src/robot_follow/core/safety.py src/robot_follow/core/pipeline.py
```

Expected:

- Controller defaults match `target_distance_m=1.5`, `hold_tolerance_m=0.15`, `min_safe_distance_m=0.5`, `max_follow_distance_m=3.0`.
- Controller emits negative `vx` for close-but-safe distances.
- Safety default uses `min_safe_distance_m=0.5`.
- Pipeline emits a `stop` command with reason `target_too_far` for filtered distances above `3.0m`.

- [ ] **Step 3: Check git status**

Run:

```bash
git status --short
```

Expected: only the pre-existing `.gitignore` change may remain unstaged; implementation changes should be committed.

