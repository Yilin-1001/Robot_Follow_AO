from __future__ import annotations

import time

from robot_follow.alx_g100_protocol import COMMAND_POSITION, PositionSample
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.serial_reader import ALXG100SerialReader, WristbandGateState


POSITION_FRAME = bytes.fromhex(
    "FF FF FF FF 00 25 34 7E 20 01 01 02 "
    "00 00 10 AA 00 00 10 AA 00 00 01 EA "
    "FF A8 00 25 12 34 00 0C BE 00 00 00 40"
)
OFF_FRAME = bytes.fromhex(
    "FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA"
)


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

    def wristband_state(self):
        return WristbandGateState(enabled=None, updated_at=None)

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


class GateStateReader:
    def __init__(self, *args, **kwargs) -> None:
        self.closed = False
        self._state = WristbandGateState(enabled=None, updated_at=None)

    def open(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def read_samples(self):
        self._state = WristbandGateState(enabled=True, updated_at=123.0)
        return iter(())

    def wristband_state(self) -> WristbandGateState:
        return self._state


def test_live_source_exposes_cached_wristband_state() -> None:
    source = ALXLiveSource(reader_factory=GateStateReader, reconnect_delay_s=0.01)

    source.start()
    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            state = source.wristband_state()
            if state.enabled is True:
                break
            time.sleep(0.01)
    finally:
        source.stop()

    assert source.wristband_state().enabled is True
    assert source.wristband_state().updated_at == 123.0


class ScriptedSerial:
    def __init__(self) -> None:
        self.is_open = True
        self._reads = [POSITION_FRAME, OFF_FRAME]

    def read(self, _size):
        if self._reads:
            return self._reads.pop(0)
        time.sleep(0.001)
        return b""

    def close(self) -> None:
        self.is_open = False


class ProtocolGateReader(ALXG100SerialReader):
    def open(self) -> None:
        self._serial = ScriptedSerial()


def test_explicit_off_reaches_live_source_without_followup_position_sample() -> None:
    source = ALXLiveSource(reader_factory=ProtocolGateReader)

    source.start()
    try:
        state = _wait_for(
            lambda: (
                current
                if (current := source.wristband_state()).enabled is False
                else None
            )
        )
        snapshot = source.snapshot()
    finally:
        source.stop()

    assert state is not None
    assert state.updated_at is not None
    assert snapshot is not None
    assert snapshot.distance_m == 4.9
