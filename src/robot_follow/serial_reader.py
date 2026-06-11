from __future__ import annotations

import time
from typing import Iterator, Optional

import serial

from robot_follow.alx_g100_protocol import (
    ALXG100FrameError,
    FRAME_HEADER,
    HEARTBEAT_FRAME_LENGTH,
    POSITION_FRAME_LENGTH,
    ParsedSample,
    parse_frame,
)


class ALXG100SerialReader:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

    def open(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "ALXG100SerialReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def read_samples(self) -> Iterator[ParsedSample]:
        if self._serial is None:
            raise RuntimeError("serial reader is not open")

        while True:
            chunk = self._serial.read(64)
            if chunk:
                self._buffer.extend(chunk)
            sample = self._try_pop_sample()
            if sample is not None:
                yield sample

    def _try_pop_sample(self) -> Optional[ParsedSample]:
        header_index = self._buffer.find(FRAME_HEADER)
        if header_index < 0:
            self._buffer.clear()
            return None
        if header_index > 0:
            del self._buffer[:header_index]

        if len(self._buffer) < 10:
            return None

        command = int.from_bytes(self._buffer[8:10], byteorder="big", signed=False)
        if command == 0x2001:
            frame_len = POSITION_FRAME_LENGTH
        elif command == 0x2002:
            frame_len = HEARTBEAT_FRAME_LENGTH
        else:
            del self._buffer[:4]
            return None

        if len(self._buffer) < frame_len:
            return None

        raw = bytes(self._buffer[:frame_len])
        del self._buffer[:frame_len]
        try:
            return parse_frame(raw, timestamp=time.monotonic())
        except ALXG100FrameError:
            return None
