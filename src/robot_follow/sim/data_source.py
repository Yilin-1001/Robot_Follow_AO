"""Simulation data sources.

A data source is any iterable of TargetSample. Two concrete sources are
provided here:

* BuiltinScenarioSource — wraps a named builtin scenario.
* CsvReplaySource        — replays frames from a CSV file or file-like object.

CSV format
----------
The first row is a header. Required columns:

    timestamp, distance_m, azimuth_deg

Optional column:

    valid   (any of: true/false, 1/0, yes/no, case-insensitive)

When the ``valid`` column is absent or a cell is empty, the sample is
treated as valid (valid=True).

Example::

    timestamp,distance_m,azimuth_deg,valid
    1000.00,2.50,0.0,true
    1000.05,2.40,0.0,
    1000.10,0.0,0.0,false
"""

from __future__ import annotations

import csv
from io import IOBase
from pathlib import Path
from typing import Iterator, Union

from robot_follow.core.types import TargetSample
from robot_follow.sim.scenarios import Scenario, get_scenario

CsvSourceLike = Union[str, "Path", IOBase]


_TRUE_TOKENS = {"1", "true", "yes", "y", "t"}
_FALSE_TOKENS = {"0", "false", "no", "n", "f"}
_REQUIRED_COLUMNS = ("timestamp", "distance_m", "azimuth_deg")


def _to_bool(token: str) -> bool:
    """Parse a truthiness token from a CSV cell."""
    value = token.strip().lower()
    if value in _TRUE_TOKENS:
        return True
    if value in _FALSE_TOKENS:
        return False
    raise ValueError(f"Cannot parse 'valid' value (expected true/false/1/0): {token!r}")


def _build_samples(reader: csv.DictReader) -> Iterator[TargetSample]:
    """Convert CSV DictReader rows into TargetSample instances.

    The DictReader already consumed the header row; we check its fieldnames
    for required columns and then iterate over the data rows. A completely
    empty file (fieldnames is None) yields no samples rather than raising.
    """
    fieldnames = reader.fieldnames
    if not fieldnames:
        # Empty file (no header at all) -> empty source.
        return

    missing = [c for c in _REQUIRED_COLUMNS if c not in fieldnames]
    if missing:
        raise ValueError(f"CSV missing required column(s): {missing}")

    has_valid = "valid" in fieldnames

    for row in reader:
        try:
            timestamp = float(row["timestamp"])
            distance_m = float(row["distance_m"])
            azimuth_deg = float(row["azimuth_deg"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"CSV row has non-numeric value: {row!r}") from exc

        if has_valid:
            raw_valid = row.get("valid")
            valid = True if raw_valid is None or raw_valid == "" else _to_bool(raw_valid)
        else:
            valid = True

        yield TargetSample(
            timestamp=timestamp,
            distance_m=distance_m,
            azimuth_deg=azimuth_deg,
            valid=valid,
        )


# ---------------------------------------------------------------------------
# BuiltinScenarioSource
# ---------------------------------------------------------------------------
class BuiltinScenarioSource:
    """Iterable data source backed by a builtin scenario."""

    def __init__(self, scenario: Union[str, Scenario] = "straight_approach") -> None:
        if isinstance(scenario, Scenario):
            self._scenario = scenario
        else:
            self._scenario = get_scenario(scenario)

    @property
    def scenario(self) -> Scenario:
        return self._scenario

    @property
    def name(self) -> str:
        return self._scenario.name

    @property
    def samples(self) -> tuple[TargetSample, ...]:
        return self._scenario.samples

    def __iter__(self) -> Iterator[TargetSample]:
        return iter(self._scenario.samples)

    def __len__(self) -> int:
        return len(self._scenario.samples)


# ---------------------------------------------------------------------------
# CsvReplaySource
# ---------------------------------------------------------------------------
class CsvReplaySource:
    """Iterable data source that replays TargetSample frames from CSV.

    Parameters
    ----------
    source
        A filesystem path (str / Path) to a CSV file, or an already-open
        text-mode file-like object (e.g. io.StringIO). File-like objects
        are read but not closed.
    """

    def __init__(self, source: CsvSourceLike) -> None:
        self._source = source
        self._samples: list[TargetSample] = list(self._read(source))

    @property
    def samples(self) -> list[TargetSample]:
        return list(self._samples)

    def __iter__(self) -> Iterator[TargetSample]:
        return iter(self._samples)

    def __len__(self) -> int:
        return len(self._samples)

    @staticmethod
    def _read(source: CsvSourceLike) -> Iterator[TargetSample]:
        if hasattr(source, "read"):
            # File-like object: read without taking ownership (do not close).
            reader = csv.DictReader(source)  # type: ignore[arg-type]
            yield from _build_samples(reader)
        else:
            path = Path(source)  # type: ignore[arg-type]
            with path.open("r", newline="", encoding="utf-8") as handle:
                yield from _build_samples(csv.DictReader(handle))