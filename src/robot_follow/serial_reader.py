from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterator, Optional

import serial

from robot_follow.alx_g100_protocol import (
    ALXG100FrameError,
    FRAME_HEADER,
    HEARTBEAT_FRAME_LENGTH,
    POSITION_FRAME_LENGTH,
    ParsedSample,
    parse_frame,
)


@dataclass(frozen=True)
class WristbandGateState:
    enabled: Optional[bool]
    updated_at: Optional[float]


class ALXG100SerialReader:
    WRISTBAND_ON_PRINT_INTERVAL_S = 0.25

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.05,
        wristband_state_callback: Optional[Callable[[WristbandGateState], None]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._wristband_enabled: Optional[bool] = None
        self._wristband_updated_at: Optional[float] = None
        self._last_wristband_on_print_s = 0.0
        self._wristband_state_callback = wristband_state_callback

    def wristband_state(self) -> WristbandGateState:
        return WristbandGateState(
            enabled=self._wristband_enabled,
            updated_at=self._wristband_updated_at,
        )

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
            else:
                if not self._serial.is_open:
                    self._emit_wristband_off()
                    return
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

        # Wristband ON shares bytes 8:12 with ALX position frames whose
        # command/version are 0x2001/0x0102. Emit the marker for the supervisor,
        # but keep parsing the frame as ALX position data.
        if len(self._buffer) >= 12:
            if self._pop_wristband_off_from_buffer():
                return None
            declared_len = int.from_bytes(self._buffer[4:6], byteorder="big", signed=False)
            if declared_len == 0x25 and len(self._buffer) >= declared_len:
                four_byte_sig = bytes(self._buffer[8:12])
                if four_byte_sig == b"\x20\x01\x01\x02":
                    self._emit_wristband_on()

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

    def _pop_wristband_off_from_buffer(self) -> bool:
        search_at = 0
        off_signature = b"\x20\x02\x01\x02"
        while True:
            frame_index = self._buffer.find(FRAME_HEADER, search_at)
            if frame_index < 0:
                return False
            if len(self._buffer) < frame_index + 12:
                return False
            declared_len = int.from_bytes(
                self._buffer[frame_index + 4:frame_index + 6],
                byteorder="big",
                signed=False,
            )
            if (
                declared_len == 0x10
                and len(self._buffer) >= frame_index + declared_len
                and bytes(self._buffer[frame_index + 8:frame_index + 12]) == off_signature
            ):
                self._emit_wristband_off()
                del self._buffer[:frame_index + declared_len]
                return True
            search_at = frame_index + 4

    def _emit_wristband_on(self) -> None:
        now = time.monotonic()
        if (
            self._wristband_enabled is not True
            or now - self._last_wristband_on_print_s >= self.WRISTBAND_ON_PRINT_INTERVAL_S
        ):
            print("[WRISTBAND] ON", flush=True)
            self._last_wristband_on_print_s = now
        self._wristband_enabled = True
        self._wristband_updated_at = now
        self._notify_wristband_state()

    def _emit_wristband_off(self) -> None:
        now = time.monotonic()
        print("[WRISTBAND] OFF", flush=True)
        self._wristband_enabled = False
        self._wristband_updated_at = now
        self._notify_wristband_state()

    def _notify_wristband_state(self) -> None:
        callback = self._wristband_state_callback
        if callback is not None:
            callback(self.wristband_state())
