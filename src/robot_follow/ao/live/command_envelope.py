from __future__ import annotations

import time
from dataclasses import dataclass

from robot_follow.core.types import MotionCommand


@dataclass(frozen=True)
class CommandEnvelope:
    command: MotionCommand
    generated_at: float
    source: str

    def age_s(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.generated_at

    def is_stale(self, *, now: float, max_age_s: float) -> bool:
        return self.age_s(now) > max_age_s

    @classmethod
    def stop(cls, *, reason: str, generated_at: float | None = None) -> "CommandEnvelope":
        now = time.monotonic() if generated_at is None else generated_at
        return cls(
            command=MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason),
            generated_at=now,
            source="ao_runtime",
        )
