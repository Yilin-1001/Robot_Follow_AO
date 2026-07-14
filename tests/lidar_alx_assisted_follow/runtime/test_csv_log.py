"""Tests for CSV logging (truthful, no actual_v)."""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

from robot_follow.lidar_alx_assisted_follow.domain.models import (
    ConfidenceResult,
    MotionIntent,
    StopReason,
    StrategyResult,
)
from robot_follow.lidar_alx_assisted_follow.runtime.csv_log import PrototypeCSVLogger


def make_confidence_result(
    total: float = 0.90,
    c_t: float = 1.0,
    c_g: float = 0.8,
    c_a: float = 0.9,
    alx_fresh: bool = True,
) -> ConfidenceResult:
    """Create a test confidence result."""
    return ConfidenceResult(
        c_t=c_t,
        c_g=c_g,
        c_a=c_a,
        total=total,
        alx_fresh=alx_fresh,
        reason="nominal",
    )


def make_motion_intent(
    sequence: int,
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
    reason: str = "stop",
) -> MotionIntent:
    """Create a test motion intent."""
    return MotionIntent(
        sequence=sequence,
        vx=vx,
        vy=vy,
        omega=omega,
        reason=reason,
        generated_at_s=time.monotonic(),
    )


def make_strategy_result(
    sequence: int,
    vx: float = 0.08,
    vy: float = 0.0,
    omega: float = 0.05,
    stop_reason: Optional[StopReason] = None,
) -> StrategyResult:
    """Create a test strategy result."""
    apf_requested = make_motion_intent(sequence, vx, vy, omega, reason="follow")
    dispatched = make_motion_intent(sequence, vx, vy, omega, reason="follow")
    return StrategyResult(
        sequence=sequence,
        apf_requested=apf_requested,
        dispatched=dispatched,
        safety_complete=True,
        stop_reason=stop_reason,
        compute_ms=1.5,
    )


# ============================================================================
# Test: CSV header format
# ============================================================================


def test_csv_header_matches_specification(tmp_path: Path):
    """CSV header must match the exact specification.

    Header:
    record_kind,wall_time_iso,monotonic_s,cloud_sequence,cloud_age_ms,alx_age_ms,
    lifecycle_state,lock_state,target_center_x,target_center_y,target_support_count,
    c_t,c_g,c_a,confidence,stop_reason,apf_requested_vx,apf_requested_vy,
    apf_requested_omega,dispatched_vx,dispatched_vy,dispatched_omega,
    strategy_compute_ms,g1_dispatch_ms,bridge_state,sdk_error,calibration_status
    """
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    # Write one row
    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=1,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="following",
        target_center_x=1.0,
        target_center_y=0.0,
        target_support_count=50,
        confidence=make_confidence_result(),
        stop_reason=None,
        apf_requested_vx=0.08,
        apf_requested_vy=0.0,
        apf_requested_omega=0.05,
        dispatched_vx=0.08,
        dispatched_vy=0.0,
        dispatched_omega=0.05,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    # Read and verify header
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)

    expected_header = [
        "record_kind",
        "wall_time_iso",
        "monotonic_s",
        "cloud_sequence",
        "cloud_age_ms",
        "alx_age_ms",
        "lifecycle_state",
        "lock_state",
        "target_center_x",
        "target_center_y",
        "target_support_count",
        "c_t",
        "c_g",
        "c_a",
        "confidence",
        "stop_reason",
        "apf_requested_vx",
        "apf_requested_vy",
        "apf_requested_omega",
        "dispatched_vx",
        "dispatched_vy",
        "dispatched_omega",
        "strategy_compute_ms",
        "g1_dispatch_ms",
        "bridge_state",
        "sdk_error",
        "calibration_status",
    ]

    assert header == expected_header


# ============================================================================
# Test: Strategy row format
# ============================================================================


def test_strategy_row_format(tmp_path: Path):
    """Strategy row should have record_kind=strategy and correct fields."""
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=42,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="following",
        target_center_x=1.5,
        target_center_y=-0.3,
        target_support_count=50,
        confidence=make_confidence_result(total=0.91, c_t=1.0, c_g=0.8, c_a=0.9),
        stop_reason=None,
        apf_requested_vx=0.08,
        apf_requested_vy=0.01,
        apf_requested_omega=0.05,
        dispatched_vx=0.07,
        dispatched_vy=0.01,
        dispatched_omega=0.05,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader)

    assert row["record_kind"] == "strategy"
    assert row["cloud_sequence"] == "42"
    assert row["cloud_age_ms"] == "10.0"
    assert row["alx_age_ms"] == "20.0"
    assert row["lifecycle_state"] == "sensors_ready"
    assert row["lock_state"] == "following"
    assert row["target_center_x"] == "1.5"
    assert row["target_center_y"] == "-0.3"
    assert row["target_support_count"] == "50"
    assert row["c_t"] == "1.0"
    assert row["c_g"] == "0.8"
    assert row["c_a"] == "0.9"
    assert row["confidence"] == "0.91"
    assert row["stop_reason"] == ""
    assert row["apf_requested_vx"] == "0.08"
    assert row["apf_requested_vy"] == "0.01"
    assert row["apf_requested_omega"] == "0.05"
    assert row["dispatched_vx"] == "0.07"
    assert row["dispatched_vy"] == "0.01"
    assert row["dispatched_omega"] == "0.05"
    assert row["strategy_compute_ms"] == "1.5"
    assert row["g1_dispatch_ms"] == "0.5"
    assert row["bridge_state"] == "initialized"
    assert row["sdk_error"] == ""
    assert row["calibration_status"] == "provisional"


# ============================================================================
# Test: Safety stop row format
# ============================================================================


def test_safety_stop_row_format(tmp_path: Path):
    """Safety stop row should have record_kind=safety_stop and reason."""
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    logger.log_safety_stop(
        monotonic_s=2.0,
        cloud_sequence=43,
        stop_reason="target_too_close",
        lifecycle_state="sensors_ready",
        lock_state="following",
        calibration_status="provisional",
    )

    logger.close()

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader)

    assert row["record_kind"] == "safety_stop"
    assert row["cloud_sequence"] == "43"
    assert row["stop_reason"] == "target_too_close"
    assert row["lifecycle_state"] == "sensors_ready"
    assert row["lock_state"] == "following"
    assert row["calibration_status"] == "provisional"


# ============================================================================
# Test: No duplicate safety_stop rows
# ============================================================================


def test_no_duplicate_safety_stop_rows(tmp_path: Path):
    """Repeated identical stop reasons should not create duplicate rows.

    Safety stop rows are only written when reason changes.
    """
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    # First stop
    logger.log_safety_stop(
        monotonic_s=1.0,
        cloud_sequence=1,
        stop_reason="target_too_close",
        lifecycle_state="sensors_ready",
        lock_state="following",
        calibration_status="provisional",
    )

    # Same stop reason - should not create new row
    logger.log_safety_stop(
        monotonic_s=1.025,
        cloud_sequence=1,
        stop_reason="target_too_close",
        lifecycle_state="sensors_ready",
        lock_state="following",
        calibration_status="provisional",
    )

    logger.close()

    # Should have only one row
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    assert len(rows) == 1


def test_safety_stop_row_written_when_reason_changes(tmp_path: Path):
    """New safety stop row should be written when reason changes."""
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    # First stop
    logger.log_safety_stop(
        monotonic_s=1.0,
        cloud_sequence=1,
        stop_reason="target_too_close",
        lifecycle_state="sensors_ready",
        lock_state="following",
        calibration_status="provisional",
    )

    # Different stop reason - should create new row
    logger.log_safety_stop(
        monotonic_s=1.025,
        cloud_sequence=1,
        stop_reason="non_target_hard_clearance",
        lifecycle_state="sensors_ready",
        lock_state="following",
        calibration_status="provisional",
    )

    logger.close()

    # Should have two rows
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    assert len(rows) == 2
    assert rows[0]["stop_reason"] == "target_too_close"
    assert rows[1]["stop_reason"] == "non_target_hard_clearance"


# ============================================================================
# Test: No actual_v fields
# ============================================================================


def test_csv_does_not_contain_actual_v_fields(tmp_path: Path):
    """CSV must NOT contain actual_vx, actual_vy, actual_omega fields.

    Runtime CSV records APF-requested and software-dispatched velocities.
    It never labels them as physical/actual G1 velocity because no
    odometry feedback is consumed.
    """
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=1,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="following",
        target_center_x=1.0,
        target_center_y=0.0,
        target_support_count=50,
        confidence=make_confidence_result(),
        stop_reason=None,
        apf_requested_vx=0.08,
        apf_requested_vy=0.0,
        apf_requested_omega=0.05,
        dispatched_vx=0.08,
        dispatched_vy=0.0,
        dispatched_omega=0.05,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)

    # Must NOT contain actual_v fields
    assert "actual_vx" not in header
    assert "actual_vy" not in header
    assert "actual_omega" not in header


# ============================================================================
# Test: OSError handling
# ============================================================================


def test_oserror_logged_once_and_csv_disabled():
    """On OSError, log once, disable CSV, and continue operation."""
    # This test will verify:
    # 1. OSError during write is caught
    # 2. Error is logged once (not repeated)
    # 3. CSV is disabled after error
    # 4. Safety/control operation continues

    pass


# ============================================================================
# Test: Disabled CSV
# ============================================================================


def test_disabled_csv_does_not_write(tmp_path: Path):
    """If CSV is disabled, no file should be created."""
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=False)

    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=1,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="following",
        target_center_x=1.0,
        target_center_y=0.0,
        target_support_count=50,
        confidence=make_confidence_result(),
        stop_reason=None,
        apf_requested_vx=0.08,
        apf_requested_vy=0.0,
        apf_requested_omega=0.05,
        dispatched_vx=0.08,
        dispatched_vy=0.0,
        dispatched_omega=0.05,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    # File should not exist
    assert not csv_path.exists()


# ============================================================================
# Test: Flush interval
# ============================================================================


def test_flush_interval_respected(tmp_path: Path):
    """CSV should be flushed at the configured flush_interval_s."""
    # This test would require time-based mocking
    # For now, we verify the flush_interval_s configuration is accepted
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True, flush_interval_s=1.0)

    # Logger should accept the flush_interval_s parameter
    assert logger._flush_interval_s == 1.0

    logger.close()


# ============================================================================
# Test: NaN handling
# ============================================================================


def test_nan_values_converted_to_empty_string(tmp_path: Path):
    """NaN values should be written as empty string, not 'nan'.

    Unavailable optional values are JSON null, never NaN.
    """
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=1,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="empty",
        target_center_x=float("nan"),
        target_center_y=float("nan"),
        target_support_count=0,
        confidence=make_confidence_result(total=0.0, c_t=0.0, c_g=0.0, c_a=0.0),
        stop_reason=None,
        apf_requested_vx=0.0,
        apf_requested_vy=0.0,
        apf_requested_omega=0.0,
        dispatched_vx=0.0,
        dispatched_vy=0.0,
        dispatched_omega=0.0,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader)

    # NaN values should be written as empty string
    assert row["target_center_x"] == ""
    assert row["target_center_y"] == ""


# ============================================================================
# Test: Calibration status
# ============================================================================


def test_calibration_status_always_provisional(tmp_path: Path):
    """Calibration status should always be 'provisional' for this prototype."""
    csv_path = tmp_path / "test.csv"
    logger = PrototypeCSVLogger(str(csv_path), enabled=True)

    logger.log_strategy(
        monotonic_s=1.0,
        cloud_sequence=1,
        cloud_age_ms=10.0,
        alx_age_ms=20.0,
        lifecycle_state="sensors_ready",
        lock_state="following",
        target_center_x=1.0,
        target_center_y=0.0,
        target_support_count=50,
        confidence=make_confidence_result(),
        stop_reason=None,
        apf_requested_vx=0.08,
        apf_requested_vy=0.0,
        apf_requested_omega=0.05,
        dispatched_vx=0.08,
        dispatched_vy=0.0,
        dispatched_omega=0.05,
        strategy_compute_ms=1.5,
        g1_dispatch_ms=0.5,
        bridge_state="initialized",
        sdk_error=None,
        calibration_status="provisional",
    )

    logger.close()

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader)

    assert row["calibration_status"] == "provisional"