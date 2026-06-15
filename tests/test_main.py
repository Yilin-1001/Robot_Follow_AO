"""Tests for headless live ALX simulation CLI helpers."""

from __future__ import annotations

from io import StringIO

import pytest

from robot_follow.real.alx_source import ALXSnapshot
from robot_follow.main import (
    LiveALXConfig,
    run_live_alx_simulation,
    sample_from_snapshot,
    target_xy_from_sample,
)
from robot_follow.core.types import TargetSample


class FakeClock:
    def __init__(self, now: float = 1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def sleep(self, seconds: float):
        self.now += seconds


class FakeALXSource:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)
        self.index = 0
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def snapshot(self):
        if not self.snapshots:
            return None
        value = self.snapshots[min(self.index, len(self.snapshots) - 1)]
        self.index += 1
        return value


def _snapshot(*, timestamp=1000.0, distance_m=2.0, azimuth_deg=10.0, error=None):
    return ALXSnapshot(
        timestamp=timestamp,
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        elevation_deg=0.0,
        tag_status=0,
        anchor_id=1,
        tag_id=2,
        fps=40.0,
        received_at=timestamp,
        heartbeat_at=None,
        error=error,
    )


def test_sample_from_live_snapshot_is_valid():
    sample, status, serial_error, fps, age_ms = sample_from_snapshot(
        _snapshot(timestamp=1000.0),
        now=1000.1,
        stale_timeout_s=0.5,
    )

    assert sample.valid is True
    assert sample.distance_m == 2.0
    assert sample.azimuth_deg == 10.0
    assert status == "live"
    assert serial_error is False
    assert fps == 40.0
    assert age_ms == pytest.approx(100.0)


def test_missing_snapshot_generates_invalid_sample():
    sample, status, serial_error, _, _ = sample_from_snapshot(
        None,
        now=1000.0,
        stale_timeout_s=0.5,
    )

    assert sample.valid is False
    assert status == "waiting_for_position"
    assert serial_error is False


def test_stale_snapshot_preserves_timestamp_for_timeout():
    sample, status, serial_error, _, age_ms = sample_from_snapshot(
        _snapshot(timestamp=1000.0),
        now=1001.0,
        stale_timeout_s=0.5,
    )

    assert sample.valid is True
    assert sample.timestamp == 1000.0
    assert status == "stale 1000ms"
    assert serial_error is False
    assert age_ms == 1000.0


def test_error_snapshot_sets_serial_error():
    sample, status, serial_error, _, _ = sample_from_snapshot(
        _snapshot(timestamp=1000.0, error="boom"),
        now=1000.0,
        stale_timeout_s=0.5,
    )

    assert sample.valid is False
    assert status == "error: boom"
    assert serial_error is True


def test_target_xy_from_sample_uses_receiver_frame():
    x_m, y_m = target_xy_from_sample(
        TargetSample(timestamp=1000.0, distance_m=2.0, azimuth_deg=30.0)
    )

    assert x_m == pytest.approx(1.7320508)
    assert y_m == pytest.approx(1.0)


def test_run_live_alx_simulation_records_commands_and_stops_source():
    clock = FakeClock()
    source = FakeALXSource(
        [
            _snapshot(timestamp=1000.0, distance_m=2.0, azimuth_deg=0.0),
            _snapshot(timestamp=1000.025, distance_m=2.0, azimuth_deg=0.0),
        ]
    )
    lines = []

    result = run_live_alx_simulation(
        LiveALXConfig(frame_limit=2, print_every_s=0.0),
        source=source,
        emit=lines.append,
        clock=clock,
        sleep=clock.sleep,
    )

    assert source.started is True
    assert source.stopped is True
    assert len(result.frames) == 2
    assert result.frames[0].target_x_m == pytest.approx(2.0)
    assert result.frames[0].target_y_m == pytest.approx(0.0)
    assert result.move_count == 2
    # Task 7: the loop now stops the bridge in finally on shutdown, so a clean
    # all-move run records exactly one stop -- the shutdown stop -- and it must
    # be the last command in history (guarding against a coincidental count
    # from the wrong source, e.g. a watchdog_timeout leaking in).
    assert result.stop_count == 1
    assert result.bridge.history[-1].reason == "shutdown"
    assert any(line.startswith("frame=") for line in lines)
    assert any(line.startswith("summary") for line in lines)


def test_run_live_alx_simulation_writes_csv():
    clock = FakeClock()
    source = FakeALXSource([_snapshot(timestamp=1000.0)])
    csv_file = StringIO()

    run_live_alx_simulation(
        LiveALXConfig(frame_limit=1, print_every_s=999.0),
        source=source,
        emit=lambda _line: None,
        clock=clock,
        sleep=clock.sleep,
        csv_file=csv_file,
    )

    output = csv_file.getvalue()
    assert "frame,elapsed_s,status" in output
    assert "target_x_m,target_y_m" in output
    assert "move" in output
    assert "tracking_turning" in output
