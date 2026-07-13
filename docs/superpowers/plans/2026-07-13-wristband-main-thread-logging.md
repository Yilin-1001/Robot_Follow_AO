# Wristband Main-Thread Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent serial-reader wristband frames from printing after Ctrl+C while preserving normal ON/OFF transition visibility and AO restart behavior.

**Architecture:** The serial reader remains responsible only for parsing frames and publishing `WristbandGateState`. A small state-edge printer owned by each AO CLI main thread renders transitions before `runtime.tick()`. Because background serial IO never writes stdout, it cannot print while the main thread is blocked in a G1 SDK call or is exiting.

**Tech Stack:** Python 3, pytest, existing AO CLI and ALX serial-reader modules.

---

### Task 1: Specify silent serial parsing and main-thread transition logging

**Files:**
- Modify: `tests/test_serial_reader.py:12-71`
- Modify: `tests/ao/live/test_cli.py:1-135`

- [ ] **Step 1: Write the failing tests**

```python
def test_wristband_frames_update_state_without_writing_stdout(capsys):
    reader = ALXG100SerialReader(port="unused")
    reader._buffer.extend(POSITION_FRAME + OFF_FRAME)

    assert reader._try_pop_sample() is not None
    assert reader._try_pop_sample() is None
    assert reader.wristband_state().enabled is False
    assert capsys.readouterr().out == ""
```

```python
def test_wristband_status_printer_prints_each_main_thread_transition_once(capsys):
    source = FakeALXSource(enabled=None)
    printer = WristbandStatusPrinter()

    printer.emit_if_changed(source)
    source.enabled = True
    printer.emit_if_changed(source)
    printer.emit_if_changed(source)
    source.enabled = False
    printer.emit_if_changed(source)

    assert capsys.readouterr().out == "[WRISTBAND] ON\\n[WRISTBAND] OFF\\n"
```

- [ ] **Step 2: Run the two tests to verify they fail**

Run: `conda run -n <confirmed-env> pytest tests/test_serial_reader.py::test_wristband_frames_update_state_without_writing_stdout tests/ao/live/test_cli.py::test_wristband_status_printer_prints_each_main_thread_transition_once -q`

Expected: FAIL because the serial reader currently prints directly and `WristbandStatusPrinter` does not exist.

### Task 2: Move stdout ownership to AO CLI main threads

**Files:**
- Modify: `src/robot_follow/serial_reader.py:25-177`
- Modify: `src/robot_follow/ao/live/cli.py:1-215`
- Modify: `src/robot_follow/ao/live/web_cli.py:17-237`

- [ ] **Step 1: Make the serial reader state-only**

Remove `WRISTBAND_ON_PRINT_INTERVAL_S`, `_last_wristband_on_print_s`, and both direct
`print()` calls. Keep `_wristband_enabled`, `_wristband_updated_at`, and
`_notify_wristband_state()` unchanged.

- [ ] **Step 2: Add the main-thread status printer**

```python
class WristbandStatusPrinter:
    def __init__(self) -> None:
        self._last_enabled: bool | None = None

    def emit_if_changed(self, source) -> None:
        enabled = source.wristband_state().enabled
        if enabled is None or enabled == self._last_enabled:
            return
        self._last_enabled = enabled
        print(f"[WRISTBAND] {'ON' if enabled else 'OFF'}", flush=True)
```

Instantiate it in each CLI entrypoint and call `emit_if_changed(runtime.alx_source)`
immediately before `runtime.tick()`.

- [ ] **Step 3: Update pre-existing serial-reader stdout assertions**

Change the existing parser tests to assert empty stdout while retaining their frame and
state assertions. This documents that parser output is intentionally silent.

- [ ] **Step 4: Run focused tests to verify the implementation**

Run: `conda run -n <confirmed-env> pytest tests/test_serial_reader.py tests/ao/live/test_cli.py tests/ao/live/test_web_cli.py -q`

Expected: PASS.

### Task 3: Regression verification

**Files:**
- Verify: `tests/real/test_alx_source.py`
- Verify: `tests/ao/live/test_runtime.py`

- [ ] **Step 1: Run affected runtime and source tests**

Run: `conda run -n <confirmed-env> pytest tests/real/test_alx_source.py tests/ao/live/test_runtime.py -q`

Expected: PASS; the unchanged state callback still drives OFF safety stopping and ON restart.

- [ ] **Step 2: Inspect the scoped diff**

Run: `git diff -- src/robot_follow/serial_reader.py src/robot_follow/ao/live/cli.py src/robot_follow/ao/live/web_cli.py tests/test_serial_reader.py tests/ao/live/test_cli.py`

Expected: only logging ownership and its tests change; no AO gating or G1 behavior changes.
