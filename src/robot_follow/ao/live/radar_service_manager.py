from __future__ import annotations

import subprocess
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Protocol

from robot_follow.ao.config import AORadarServiceConfig


class ManagedProcess(Protocol):
    def poll(self):
        pass

    def terminate(self) -> None:
        pass

    def kill(self) -> None:
        pass

    def wait(self, timeout=None) -> int:
        pass


class PopenFactory(Protocol):
    def __call__(self, command: str, *, shell: bool, start_new_session: bool) -> ManagedProcess:
        pass


class RadarServiceState(str, Enum):
    STOPPED = "stopped"
    RUNNING = "running"


@dataclass
class RadarServiceManager:
    config: AORadarServiceConfig
    popen_factory: PopenFactory = subprocess.Popen

    def __post_init__(self) -> None:
        self._livox: Optional[ManagedProcess] = None
        self._fast_lio: Optional[ManagedProcess] = None
        self._last_position_at: Optional[float] = None

    @property
    def state(self) -> RadarServiceState:
        if self._livox is None and self._fast_lio is None:
            return RadarServiceState.STOPPED
        return RadarServiceState.RUNNING

    def on_position_frame(self, timestamp: float) -> None:
        self._last_position_at = timestamp

    def ensure_running(self) -> None:
        if not self.config.livox_start_command:
            raise ValueError("ao_radar.radar.livox_start_command must be configured")
        if not self.config.fast_lio_start_command:
            raise ValueError("ao_radar.radar.fast_lio_start_command must be configured")
        if self._livox is None or self._livox.poll() is not None:
            self._livox = self.popen_factory(
                self.config.livox_start_command,
                shell=True,
                start_new_session=True,
            )
        if self._fast_lio is None or self._fast_lio.poll() is not None:
            self._fast_lio = self.popen_factory(
                self.config.fast_lio_start_command,
                shell=True,
                start_new_session=True,
            )

    def stop_if_idle(self, *, now: float) -> None:
        if self._last_position_at is None:
            return
        if now - self._last_position_at >= self.config.idle_stop_s:
            self.stop()

    def stop(self) -> None:
        self._stop_process(self._fast_lio)
        self._stop_process(self._livox)
        self._fast_lio = None
        self._livox = None

    def _stop_process(self, process: Optional[ManagedProcess]) -> None:
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=self.config.stop_timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)
