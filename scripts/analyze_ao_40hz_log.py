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
