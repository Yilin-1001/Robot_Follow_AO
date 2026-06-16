"""Live ALX-G100 input source.

The source owns a background reader thread so callers can sample the latest
position at a fixed control-loop rate without blocking on serial IO.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Optional

from robot_follow.alx_g100_protocol import HeartbeatSample, PositionSample
from robot_follow.serial_reader import ALXG100SerialReader


@dataclass(frozen=True)
class ALXSnapshot:
    """Latest ALX positioning data observed by the reader thread."""

    timestamp: float
    distance_m: float
    azimuth_deg: float
    elevation_deg: float
    tag_status: int
    anchor_id: int
    tag_id: int
    fps: float
    received_at: float
    heartbeat_at: Optional[float]
    error: Optional[str] = None

    def age_s(self, now: Optional[float] = None) -> float:
        """Return age of the latest position frame relative to monotonic time."""
        return (time.monotonic() if now is None else now) - self.timestamp


class ALXLiveSource:
    """Background ALX-G100 serial reader for GUI use."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 0.05,
        fps_window_s: float = 2.0,
        reconnect_delay_s: float = 0.5,
        reader_factory: Callable[..., ALXG100SerialReader] = ALXG100SerialReader,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fps_window_s = fps_window_s
        self.reconnect_delay_s = reconnect_delay_s
        self.reader_factory = reader_factory

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reader: Optional[ALXG100SerialReader] = None
        self._latest: Optional[ALXSnapshot] = None
        self._latest_heartbeat_at: Optional[float] = None
        self._error: Optional[str] = None
        self._position_times: Deque[float] = deque()

    def start(self) -> None:
        """Start the background reader thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="alx-g100-reader",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the reader thread and close the serial port."""
        self._stop.set()
        reader = self._reader
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def snapshot(self) -> Optional[ALXSnapshot]:
        """Return the latest immutable snapshot, if any."""
        with self._lock:
            latest = self._latest
            error = self._error
            heartbeat_at = self._latest_heartbeat_at
        if latest is None:
            if error is None:
                return None
            return ALXSnapshot(
                timestamp=0.0,
                distance_m=0.0,
                azimuth_deg=0.0,
                elevation_deg=0.0,
                tag_status=0,
                anchor_id=0,
                tag_id=0,
                fps=0.0,
                received_at=time.monotonic(),
                heartbeat_at=heartbeat_at,
                error=error,
            )
        if error is None:
            return latest
        return ALXSnapshot(
            timestamp=latest.timestamp,
            distance_m=latest.distance_m,
            azimuth_deg=latest.azimuth_deg,
            elevation_deg=latest.elevation_deg,
            tag_status=latest.tag_status,
            anchor_id=latest.anchor_id,
            tag_id=latest.tag_id,
            fps=latest.fps,
            received_at=latest.received_at,
            heartbeat_at=latest.heartbeat_at,
            error=error,
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            reader = self.reader_factory(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            self._reader = reader
            try:
                reader.open()
                with self._lock:
                    self._error = None
                for parsed in reader.read_samples():
                    if self._stop.is_set():
                        break
                    if isinstance(parsed, PositionSample):
                        self._update_position(parsed)
                    elif isinstance(parsed, HeartbeatSample):
                        with self._lock:
                            self._latest_heartbeat_at = parsed.timestamp
            except Exception as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    reader.close()
                except Exception:
                    pass
                if self._reader is reader:
                    self._reader = None

            if not self._stop.is_set():
                self._stop.wait(self.reconnect_delay_s)

    def _update_position(self, sample: PositionSample) -> None:
        now = time.monotonic()
        self._position_times.append(now)
        while self._position_times and now - self._position_times[0] > self.fps_window_s:
            self._position_times.popleft()
        if len(self._position_times) >= 2:
            span = self._position_times[-1] - self._position_times[0]
            fps = (len(self._position_times) - 1) / span if span > 0 else 0.0
        else:
            fps = 0.0

        with self._lock:
            self._latest = ALXSnapshot(
                timestamp=sample.timestamp,
                distance_m=sample.distance_m,
                azimuth_deg=float(sample.azimuth_deg),
                elevation_deg=float(sample.elevation_deg),
                tag_status=sample.tag_status,
                anchor_id=sample.anchor_id,
                tag_id=sample.tag_id,
                fps=fps,
                received_at=now,
                heartbeat_at=self._latest_heartbeat_at,
                error=None,
            )
            self._error = None
