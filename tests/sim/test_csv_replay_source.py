"""Tests for sim/data_source.py — CsvReplaySource."""

import io
from pathlib import Path

import pytest

from robot_follow.core.types import TargetSample
from robot_follow.sim.data_source import CsvReplaySource


SIMPLE_CSV = """timestamp,distance_m,azimuth_deg
1000.00,2.50,0.0
1000.05,2.40,0.0
1000.10,2.30,0.0
"""

CSV_WITH_VALID = """timestamp,distance_m,azimuth_deg,valid
1000.00,2.50,0.0,true
1000.05,2.40,0.0,false
1000.10,0.0,0.0,
"""

CSV_EMPTY_VALID_TRUE = """timestamp,distance_m,azimuth_deg,valid
1000.00,2.50,0.0,true
1000.05,2.40,0.0,1
1000.10,2.30,0.0,yes
"""

CSV_INVALID_FALSE = """timestamp,distance_m,azimuth_deg,valid
1000.00,2.50,0.0,false
1000.05,2.40,0.0,0
1000.10,2.30,0.0,no
"""


class TestCsvReplaySourceBasic:
    def test_reads_from_string_io(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        assert len(src) == 3

    def test_yields_target_samples(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        for s in src:
            assert isinstance(s, TargetSample)

    def test_parses_numeric_values(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        samples = list(src)
        assert samples[0].distance_m == pytest.approx(2.50)
        assert samples[0].azimuth_deg == pytest.approx(0.0)
        assert samples[0].timestamp == pytest.approx(1000.00)
        assert samples[2].distance_m == pytest.approx(2.30)

    def test_valid_defaults_to_true_without_column(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        for s in src:
            assert s.valid is True


class TestCsvReplaySourceValidColumn:
    def test_parses_true_false_tokens(self):
        src = CsvReplaySource(io.StringIO(CSV_WITH_VALID))
        samples = list(src)
        assert samples[0].valid is True   # true
        assert samples[1].valid is False  # false
        assert samples[2].valid is True   # empty -> default True

    def test_parses_various_true_tokens(self):
        src = CsvReplaySource(io.StringIO(CSV_EMPTY_VALID_TRUE))
        for s in src:
            assert s.valid is True

    def test_parses_various_false_tokens(self):
        src = CsvReplaySource(io.StringIO(CSV_INVALID_FALSE))
        for s in src:
            assert s.valid is False

    def test_invalid_valid_token_raises(self):
        bad = (
            "timestamp,distance_m,azimuth_deg,valid\n"
            "1000.00,2.50,0.0,maybe\n"
        )
        with pytest.raises(ValueError, match="Cannot parse 'valid'"):
            CsvReplaySource(io.StringIO(bad))


class TestCsvReplaySourceErrors:
    def test_missing_required_column_raises(self):
        bad = "timestamp,distance_m\n1000.0,2.5\n"
        with pytest.raises(ValueError, match="missing required column"):
            CsvReplaySource(io.StringIO(bad))

    def test_non_numeric_value_raises(self):
        bad = "timestamp,distance_m,azimuth_deg\n1000.0,abc,0.0\n"
        with pytest.raises(ValueError, match="non-numeric"):
            CsvReplaySource(io.StringIO(bad))

    def test_empty_csv_yields_no_samples(self):
        src = CsvReplaySource(io.StringIO(""))
        assert len(src) == 0

    def test_header_only_yields_no_samples(self):
        src = CsvReplaySource(io.StringIO("timestamp,distance_m,azimuth_deg\n"))
        assert len(src) == 0


class TestCsvReplaySourceFromFile:
    def test_reads_from_path(self, tmp_path: Path):
        csv_path = tmp_path / "samples.csv"
        csv_path.write_text(SIMPLE_CSV, encoding="utf-8")
        src = CsvReplaySource(csv_path)
        assert len(src) == 3
        assert list(src)[0].distance_m == pytest.approx(2.50)

    def test_reads_from_string_path(self, tmp_path: Path):
        csv_path = tmp_path / "samples.csv"
        csv_path.write_text(SIMPLE_CSV, encoding="utf-8")
        src = CsvReplaySource(str(csv_path))
        assert len(src) == 3

    def test_missing_file_raises(self):
        with pytest.raises((FileNotFoundError, OSError)):
            CsvReplaySource("/nonexistent/path/samples.csv")


class TestCsvReplaySourceReuse:
    def test_iterable_multiple_times(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        first = [s.distance_m for s in src]
        second = [s.distance_m for s in src]
        assert first == second

    def test_samples_property_is_a_copy(self):
        src = CsvReplaySource(io.StringIO(SIMPLE_CSV))
        samples = src.samples
        samples.clear()  # mutating the returned copy must not affect the source
        assert len(src) == 3