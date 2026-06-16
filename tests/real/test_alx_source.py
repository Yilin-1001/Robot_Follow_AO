from __future__ import annotations

import time

from robot_follow.alx_g100_protocol import COMMAND_POSITION, PositionSample
from robot_follow.real.alx_source import ALXLiveSource


class RecoveringReader:
    attempts = 0

    def __init__(self, **_kwargs):
        self.attempt = RecoveringReader.attempts
        RecoveringReader.attempts += 1
        self.closed = False

    def open(self):
        pass

    def close(self):
        self.closed = True

    def read_samples(self):
        if self.attempt == 0:
            raise RuntimeError("transient serial failure")
        yield PositionSample(
            command=COMMAND_POSITION,
            version_id=1,
            anchor_id=1,
            tag_id=2,
            distance_cm=210,
            azimuth_deg=8,
            elevation_deg=0,
            tag_status=0,
            batch_sn=1,
            timestamp=time.monotonic(),
        )
        while not self.closed:
            time.sleep(0.001)


def _wait_for(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    return None


def test_live_source_reconnects_after_reader_error():
    RecoveringReader.attempts = 0
    source = ALXLiveSource(
        reconnect_delay_s=0.01,
        reader_factory=RecoveringReader,
    )

    source.start()
    try:
        error_snapshot = _wait_for(
            lambda: (snap if (snap := source.snapshot()) and snap.error else None),
            timeout_s=0.5,
        )
        assert error_snapshot is not None
        assert "transient serial failure" in error_snapshot.error

        recovered_snapshot = _wait_for(
            lambda: (
                snap
                if (snap := source.snapshot())
                and snap.error is None
                and snap.distance_m == 2.1
                else None
            ),
            timeout_s=1.0,
        )

        assert recovered_snapshot is not None
        assert recovered_snapshot.azimuth_deg == 8.0
        assert RecoveringReader.attempts >= 2
    finally:
        source.stop()
