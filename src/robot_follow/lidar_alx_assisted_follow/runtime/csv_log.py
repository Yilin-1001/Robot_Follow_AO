"""CSV logging for LiDAR + ALX Assisted Follow prototype.

Records strategy and safety_stop rows with exact fields per spec.
No actual_v fields (no odometry feedback consumed).

Header:
record_kind,wall_time_iso,monotonic_s,cloud_sequence,cloud_age_ms,alx_age_ms,
lifecycle_state,lock_state,target_center_x,target_center_y,target_support_count,
c_t,c_g,c_a,confidence,stop_reason,apf_requested_vx,apf_requested_vy,
apf_requested_omega,dispatched_vx,dispatched_vy,dispatched_omega,
strategy_compute_ms,g1_dispatch_ms,bridge_state,sdk_error,calibration_status
"""

from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import IO, Any, Dict, Optional

from robot_follow.lidar_alx_assisted_follow.domain.models import ConfidenceResult


class PrototypeCSVLogger:
    """CSV logger for LiDAR + ALX Assisted Follow prototype.

    Writes strategy and safety_stop rows.
    No actual_v fields (no odometry feedback).
    Catches OSError, logs once, disables CSV, continues operation.
    """

    def __init__(
        self,
        csv_path: str,
        enabled: bool = True,
        flush_interval_s: float = 1.0,
    ) -> None:
        """Initialize CSV logger.

        Parameters
        ----------
        csv_path : str
            Path to CSV file.
        enabled : bool
            Whether CSV logging is enabled.
        flush_interval_s : float
            Interval between flushes (seconds).
        """
        self._csv_path = csv_path
        self._enabled = enabled
        self._flush_interval_s = flush_interval_s

        self._file: Optional[IO[str]] = None
        self._writer: Optional[csv.DictWriter] = None
        self._last_flush_time: float = 0.0
        self._last_safety_stop_reason: Optional[str] = None
        self._error_logged: bool = False

        self._fieldnames = [
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

        if self._enabled:
            self._open_file()

    def _open_file(self) -> None:
        """Open CSV file and write header."""
        try:
            # Create directory if needed
            Path(self._csv_path).parent.mkdir(parents=True, exist_ok=True)

            # Open file
            self._file = open(self._csv_path, "w", newline="")
            self._writer = csv.DictWriter(
                self._file,
                fieldnames=self._fieldnames,
                extrasaction="ignore",
            )
            self._writer.writeheader()
            self._file.flush()
            self._last_flush_time = time.monotonic()

        except OSError as e:
            self._log_error_once(f"Failed to open CSV file {self._csv_path}: {e}")
            self._enabled = False

    def _log_error_once(self, message: str) -> None:
        """Log error once, then disable CSV."""
        if not self._error_logged:
            # Log to stderr (or could use logging module)
            print(f"[CSVLogger] {message}")
            self._error_logged = True

    def _format_value(self, value: Any) -> str:
        """Format value for CSV.

        - None -> empty string
        - NaN -> empty string
        - Others -> str(value)
        """
        if value is None:
            return ""

        if isinstance(value, float):
            if math.isnan(value) or math.isinf(value):
                return ""

        return str(value)

    def log_strategy(
        self,
        monotonic_s: float,
        cloud_sequence: int,
        cloud_age_ms: float,
        alx_age_ms: float,
        lifecycle_state: str,
        lock_state: str,
        target_center_x: float,
        target_center_y: float,
        target_support_count: int,
        confidence: Optional[ConfidenceResult],
        stop_reason: Optional[str],
        apf_requested_vx: float,
        apf_requested_vy: float,
        apf_requested_omega: float,
        dispatched_vx: float,
        dispatched_vy: float,
        dispatched_omega: float,
        strategy_compute_ms: float,
        g1_dispatch_ms: float,
        bridge_state: str,
        sdk_error: Optional[int],
        calibration_status: str,
    ) -> None:
        """Log a strategy row.

        Parameters
        ----------
        monotonic_s : float
            Monotonic time (seconds).
        cloud_sequence : int
            Cloud sequence number.
        cloud_age_ms : float
            Cloud age (milliseconds).
        alx_age_ms : float
            ALX age (milliseconds).
        lifecycle_state : str
            Lifecycle state.
        lock_state : str
            Lock state.
        target_center_x, target_center_y : float
            Target center coordinates.
        target_support_count : int
            Number of support points.
        confidence : ConfidenceResult or None
            Legacy confidence record. The direct-ALX runtime passes None.
        stop_reason : str or None
            Stop reason (if any).
        apf_requested_vx, vy, omega : float
            APF-requested velocities.
        dispatched_vx, vy, omega : float
            Dispatched velocities.
        strategy_compute_ms : float
            Strategy computation time (milliseconds).
        g1_dispatch_ms : float
            G1 dispatch time (milliseconds).
        bridge_state : str
            Bridge state.
        sdk_error : int or None
            SDK error code (if any).
        calibration_status : str
            Calibration status.
        """
        if not self._enabled or self._writer is None:
            return

        try:
            wall_time_iso = datetime.now(timezone.utc).isoformat()

            row: Dict[str, str] = {
                "record_kind": "strategy",
                "wall_time_iso": wall_time_iso,
                "monotonic_s": self._format_value(monotonic_s),
                "cloud_sequence": self._format_value(cloud_sequence),
                "cloud_age_ms": self._format_value(cloud_age_ms),
                "alx_age_ms": self._format_value(alx_age_ms),
                "lifecycle_state": lifecycle_state,
                "lock_state": lock_state,
                "target_center_x": self._format_value(target_center_x),
                "target_center_y": self._format_value(target_center_y),
                "target_support_count": self._format_value(target_support_count),
                "c_t": self._format_value(None if confidence is None else confidence.c_t),
                "c_g": self._format_value(None if confidence is None else confidence.c_g),
                "c_a": self._format_value(None if confidence is None else confidence.c_a),
                "confidence": self._format_value(None if confidence is None else confidence.total),
                "stop_reason": self._format_value(stop_reason),
                "apf_requested_vx": self._format_value(apf_requested_vx),
                "apf_requested_vy": self._format_value(apf_requested_vy),
                "apf_requested_omega": self._format_value(apf_requested_omega),
                "dispatched_vx": self._format_value(dispatched_vx),
                "dispatched_vy": self._format_value(dispatched_vy),
                "dispatched_omega": self._format_value(dispatched_omega),
                "strategy_compute_ms": self._format_value(strategy_compute_ms),
                "g1_dispatch_ms": self._format_value(g1_dispatch_ms),
                "bridge_state": bridge_state,
                "sdk_error": self._format_value(sdk_error),
                "calibration_status": calibration_status,
            }

            self._writer.writerow(row)
            self._maybe_flush()

        except OSError as e:
            self._log_error_once(f"Failed to write strategy row: {e}")
            self._enabled = False

    def log_safety_stop(
        self,
        monotonic_s: float,
        cloud_sequence: int,
        stop_reason: str,
        lifecycle_state: str,
        lock_state: str,
        calibration_status: str,
    ) -> None:
        """Log a safety_stop row.

        Safety stop rows are only written when reason changes
        (no 40 Hz duplicate rows).

        Parameters
        ----------
        monotonic_s : float
            Monotonic time (seconds).
        cloud_sequence : int
            Cloud sequence number.
        stop_reason : str
            Stop reason.
        lifecycle_state : str
            Lifecycle state.
        lock_state : str
            Lock state.
        calibration_status : str
            Calibration status.
        """
        if not self._enabled or self._writer is None:
            return

        # Skip if same reason as last safety stop
        if self._last_safety_stop_reason == stop_reason:
            return

        self._last_safety_stop_reason = stop_reason

        try:
            wall_time_iso = datetime.now(timezone.utc).isoformat()

            row: Dict[str, str] = {
                "record_kind": "safety_stop",
                "wall_time_iso": wall_time_iso,
                "monotonic_s": self._format_value(monotonic_s),
                "cloud_sequence": self._format_value(cloud_sequence),
                "stop_reason": stop_reason,
                "lifecycle_state": lifecycle_state,
                "lock_state": lock_state,
                "calibration_status": calibration_status,
            }

            self._writer.writerow(row)
            self._maybe_flush()

        except OSError as e:
            self._log_error_once(f"Failed to write safety_stop row: {e}")
            self._enabled = False

    def _maybe_flush(self) -> None:
        """Flush if flush_interval_s has elapsed."""
        if self._file is None:
            return

        now = time.monotonic()
        if now - self._last_flush_time >= self._flush_interval_s:
            try:
                self._file.flush()
                self._last_flush_time = now
            except OSError as e:
                self._log_error_once(f"Failed to flush CSV: {e}")
                self._enabled = False

    def flush(self) -> None:
        """Flush CSV file."""
        if self._file is not None:
            try:
                self._file.flush()
                self._last_flush_time = time.monotonic()
            except OSError as e:
                self._log_error_once(f"Failed to flush CSV: {e}")
                self._enabled = False

    def close(self) -> None:
        """Close CSV file."""
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except OSError:
                pass
            finally:
                self._file = None
                self._writer = None

    def __enter__(self) -> "PrototypeCSVLogger":
        """Enter context manager."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit context manager."""
        self.close()
