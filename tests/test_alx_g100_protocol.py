from robot_follow.alx_g100_protocol import (
    ALXG100FrameError,
    COMMAND_HEARTBEAT,
    COMMAND_POSITION,
    PositionSample,
    parse_frame,
)


def test_parse_position_frame_positive_angle():
    raw = bytes.fromhex(
        "FF FF FF FF 00 25 00 0B 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 19 "
        "00 12 FF CA 12 34 00 0B 00 00 00 00 1E"
    )

    sample = parse_frame(raw, timestamp=10.0)

    assert isinstance(sample, PositionSample)
    assert sample.command == COMMAND_POSITION
    assert sample.version_id == 0x0100
    assert sample.anchor_id == 0x0000AAA2
    assert sample.tag_id == 0x0000AAA1
    assert sample.distance_cm == 25
    assert sample.distance_m == 0.25
    assert sample.azimuth_deg == 18
    assert sample.elevation_deg == -54
    assert sample.tag_status == 0x1234
    assert sample.batch_sn == 0x000B
    assert sample.timestamp == 10.0


def test_parse_position_frame_negative_angle():
    raw = bytes.fromhex(
        "FF FF FF FF 00 25 00 27 20 01 01 00 "
        "00 00 AA A2 00 00 AA A1 00 00 00 62 "
        "FF D9 00 0D 12 34 00 27 00 00 00 00 69"
    )

    sample = parse_frame(raw, timestamp=11.0)

    assert sample.distance_cm == 98
    assert sample.distance_m == 0.98
    assert sample.azimuth_deg == -39
    assert sample.elevation_deg == 13
    assert sample.timestamp == 11.0


def test_parse_heartbeat_frame():
    raw = bytes.fromhex("FF FF FF FF 00 10 00 0B 20 02 01 01 00 00 AA A1")

    sample = parse_frame(raw, timestamp=12.0)

    assert sample.command == COMMAND_HEARTBEAT
    assert sample.anchor_id == 0x0000AAA1
    assert sample.timestamp == 12.0


def test_reject_bad_header():
    raw = bytes.fromhex("00 FF FF FF 00 10 00 0B 20 02 01 01 00 00 AA A1")

    try:
        parse_frame(raw, timestamp=0.0)
    except ALXG100FrameError as exc:
        assert "header" in str(exc)
    else:
        raise AssertionError("expected ALXG100FrameError")
