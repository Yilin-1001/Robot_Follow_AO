from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from datetime import datetime
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
    raw_azimuth_deg: float | None = None
    filtered_azimuth_deg: float | None = None
    g1_sent_omega: float | None = None
    distance_m: float | None = None
    tag_status: int | None = None
    robot_yaw_deg: float | None = None
    robot_yaw_rate_rps: float | None = None
    raw_command_generated_at_ms: float | None = None
    safe_command_generated_at_ms: float | None = None
    g1_response_at_ms: float | None = None
    total_latency_ms: float | None = None


class AORadarCSVLogger:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=list(AORadarLogRow.__dataclass_fields__))
        self._writer.writeheader()

    def write(self, row: AORadarLogRow) -> None:
        self._writer.writerow(asdict(row))
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def timestamped_log_path(path: Path, *, time_format: str, now: datetime | None = None) -> Path:
    stamp = (datetime.now() if now is None else now).strftime(time_format)
    return path.with_name(f"{path.stem}_{stamp}{path.suffix}")
