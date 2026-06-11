from __future__ import annotations

from dataclasses import dataclass
from typing import Union

FRAME_HEADER = b"\xFF\xFF\xFF\xFF"
COMMAND_POSITION = 0x2001
COMMAND_HEARTBEAT = 0x2002
POSITION_FRAME_LENGTH = 37
HEARTBEAT_FRAME_LENGTH = 16


class ALXG100FrameError(ValueError):
    """Raised when an ALX-G100 frame is malformed."""


@dataclass(frozen=True)
class PositionSample:
    command: int
    version_id: int
    anchor_id: int
    tag_id: int
    distance_cm: int
    azimuth_deg: int
    elevation_deg: int
    tag_status: int
    batch_sn: int
    timestamp: float

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0


@dataclass(frozen=True)
class HeartbeatSample:
    command: int
    version_id: int
    anchor_id: int
    timestamp: float


ParsedSample = Union[PositionSample, HeartbeatSample]


def _u16_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 2], byteorder="big", signed=False)


def _s16_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 2], byteorder="big", signed=True)


def _u32_be(raw: bytes, offset: int) -> int:
    return int.from_bytes(raw[offset : offset + 4], byteorder="big", signed=False)


def parse_frame(raw: bytes, timestamp: float) -> ParsedSample:
    if len(raw) < 10:
        raise ALXG100FrameError(f"frame too short: {len(raw)}")
    if raw[:4] != FRAME_HEADER:
        raise ALXG100FrameError("invalid frame header")

    command = _u16_be(raw, 8)

    if command == COMMAND_POSITION:
        if len(raw) != POSITION_FRAME_LENGTH:
            raise ALXG100FrameError(f"invalid position frame length: {len(raw)}")
        return PositionSample(
            command=command,
            version_id=_u16_be(raw, 10),
            anchor_id=_u32_be(raw, 12),
            tag_id=_u32_be(raw, 16),
            distance_cm=_u32_be(raw, 20),
            azimuth_deg=_s16_be(raw, 24),
            elevation_deg=_s16_be(raw, 26),
            tag_status=_u16_be(raw, 28),
            batch_sn=_u16_be(raw, 30),
            timestamp=timestamp,
        )

    if command == COMMAND_HEARTBEAT:
        if len(raw) != HEARTBEAT_FRAME_LENGTH:
            raise ALXG100FrameError(f"invalid heartbeat frame length: {len(raw)}")
        return HeartbeatSample(
            command=command,
            version_id=_u16_be(raw, 10),
            anchor_id=_u32_be(raw, 12),
            timestamp=timestamp,
        )

    raise ALXG100FrameError(f"unsupported command: 0x{command:04X}")
