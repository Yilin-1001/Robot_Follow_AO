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