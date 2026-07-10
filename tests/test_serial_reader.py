from __future__ import annotations

from robot_follow.alx_g100_protocol import PositionSample
from robot_follow.serial_reader import ALXG100SerialReader


POSITION_FRAME = bytes.fromhex(
    "FF FF FF FF 00 25 34 7E 20 01 01 02 "
    "00 00 10 AA 00 00 10 AA 00 00 01 EA "
    "FF A8 00 25 12 34 00 0C BE 00 00 00 40"
)


def test_position_frame_with_wristband_on_signature_is_not_dropped(capsys):
    reader = ALXG100SerialReader(port="unused")
    reader._buffer.extend(POSITION_FRAME)

    sample = reader._try_pop_sample()

    assert isinstance(sample, PositionSample)
    assert sample.command == 0x2001
    assert sample.version_id == 0x0102
    assert sample.anchor_id == 0x000010AA
    assert sample.tag_id == 0x000010AA
    assert sample.distance_cm == 0x000001EA
    assert sample.azimuth_deg == -88
    assert sample.elevation_deg == 37
    assert sample.tag_status == 0x1234
    assert "[WRISTBAND] ON" in capsys.readouterr().out


def test_wristband_off_frame_is_still_dropped(capsys):
    reader = ALXG100SerialReader(port="unused")
    reader._buffer.extend(
        bytes.fromhex(
            "FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA"
        )
    )

    sample = reader._try_pop_sample()

    assert sample is None
    assert "[WRISTBAND] OFF" in capsys.readouterr().out


def test_repeated_position_frames_do_not_spam_wristband_on(capsys):
    reader = ALXG100SerialReader(port="unused")
    reader._buffer.extend(POSITION_FRAME + POSITION_FRAME)

    first = reader._try_pop_sample()
    second = reader._try_pop_sample()

    output = capsys.readouterr().out
    assert isinstance(first, PositionSample)
    assert isinstance(second, PositionSample)
    assert output.count("[WRISTBAND] ON") == 1


def test_wristband_off_frame_is_prioritized_over_position_backlog(capsys):
    off_frame = bytes.fromhex(
        "FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA"
    )
    reader = ALXG100SerialReader(port="unused")
    reader._buffer.extend(POSITION_FRAME + off_frame)

    sample = reader._try_pop_sample()

    output = capsys.readouterr().out
    assert sample is None
    assert "[WRISTBAND] OFF" in output
    assert "[WRISTBAND] ON" not in output
    assert reader._buffer == bytearray()


def test_serial_read_timeout_does_not_emit_wristband_off(capsys):
    class FakeSerial:
        is_open = True

        def __init__(self):
            self._reads = [b"", b"", POSITION_FRAME]

        def read(self, size):
            return self._reads.pop(0)

    reader = ALXG100SerialReader(port="unused")
    reader._serial = FakeSerial()

    sample = next(reader.read_samples())

    assert isinstance(sample, PositionSample)
    assert "[WRISTBAND] OFF" not in capsys.readouterr().out


def test_wristband_state_starts_unknown() -> None:
    reader = ALXG100SerialReader("/dev/null")

    state = reader.wristband_state()

    assert state.enabled is None
    assert state.updated_at is None


def test_position_frame_updates_wristband_state_to_on() -> None:
    reader = ALXG100SerialReader("/dev/ttyUSB0")
    reader._buffer.extend(POSITION_FRAME)

    sample = reader._try_pop_sample()
    state = reader.wristband_state()

    assert sample is not None
    assert state.enabled is True
    assert state.updated_at is not None


def test_off_frame_updates_wristband_state_to_off() -> None:
    reader = ALXG100SerialReader("/dev/ttyUSB0")
    reader._buffer.extend(
        bytes.fromhex("FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA")
    )

    assert reader._try_pop_sample() is None
    state = reader.wristband_state()

    assert state.enabled is False
    assert state.updated_at is not None


def test_off_frame_notifies_state_callback_without_yielding_sample() -> None:
    transitions = []
    reader = ALXG100SerialReader(
        "/dev/ttyUSB0",
        wristband_state_callback=transitions.append,
    )
    reader._buffer.extend(
        bytes.fromhex("FF FF FF FF 00 10 0A 6B 20 02 01 02 00 00 10 AA")
    )

    assert reader._try_pop_sample() is None
    assert [state.enabled for state in transitions] == [False]
    assert transitions[0].updated_at is not None
