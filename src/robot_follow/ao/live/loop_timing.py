from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


Clock = Callable[[], float]
Sleeper = Callable[[float], None]


@dataclass(frozen=True)
class LoopTimingConfig:
    target_hz: float

    def __post_init__(self) -> None:
        if self.target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

    @property
    def period_s(self) -> float:
        return 1.0 / self.target_hz


class LoopTiming:
    def __init__(self, *, config: LoopTimingConfig, clock: Clock, sleeper: Sleeper) -> None:
        self.config = config
        self._clock = clock
        self._sleeper = sleeper

    def mark_tick_start(self) -> float:
        return self._clock()

    def sleep_until_next_tick(self, *, started_at: float) -> float:
        elapsed_s = self._clock() - started_at
        sleep_s = max(0.0, self.config.period_s - elapsed_s)
        if sleep_s > 0.0:
            self._sleeper(sleep_s)
        return sleep_s
