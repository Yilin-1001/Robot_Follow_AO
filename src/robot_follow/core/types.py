"""Core types for the follow system.

This module contains only basic data types to avoid circular dependencies.
PipelineStepResult is defined in core/pipeline.py instead.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class TargetSample:
    """Raw or processed target observation from a data source.

    Attributes
    ----------
    timestamp
        Monotonic timestamp (typically from time.monotonic()).
    distance_m
        Distance to the target in metres.
    azimuth_deg
        Azimuth angle in degrees. Positive values mean the target is to one side.
    valid
        Whether this sample is reliable. Invalid samples should trigger stop
        and must NOT be fed to the filter.
    """

    timestamp: float
    distance_m: float
    azimuth_deg: float
    valid: bool = True


@dataclass(frozen=True)
class MotionCommand:
    """Command issued by the pipeline to the motion bridge.

    Attributes
    ----------
    kind
        Either "move" or "stop". "stop" commands must be honored immediately
        and are distinct from "move" with vx=0 (holding).
    vx
        Forward velocity in m/s.
    vy
        Lateral velocity in m/s (typically 0 for this MVP).
    omega
        Angular velocity in rad/s.
    reason
        Human-readable reason for this command.
    """

    kind: Literal["move", "stop"]
    vx: float
    vy: float
    omega: float
    reason: str

    def __post_init__(self) -> None:
        if self.kind not in {"move", "stop"}:
            raise ValueError(f"invalid motion command kind: {self.kind}")
        if self.kind == "stop" and (
            abs(self.vx) > 1e-12 or abs(self.vy) > 1e-12 or abs(self.omega) > 1e-12
        ):
            raise ValueError("stop command velocities must be zero")
