"""Safety manager for the follow system.

Determines whether the robot must stop based on data timeout, proximity,
jump detection, serial errors, SDK request timeouts, SDK errors, or manual
stop requests.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable


@dataclass
class SafetyConfig:
    """Thresholds used by the safety manager."""

    data_timeout_s: float = 0.5
    min_safe_distance_m: float = 0.5
    max_distance_jump_m: float = 0.5
    max_angle_jump_deg: float = 45.0


@dataclass
class SafetyInput:
    """Input snapshot fed to the safety manager each cycle."""

    timestamp: float
    distance_m: float
    azimuth_deg: float
    distance_jump: float
    angle_jump: float
    data_valid: bool = True
    serial_error: bool = False
    sdk_timeout: bool = False
    sdk_error: bool = False
    manual_stop: bool = False


@dataclass(frozen=True)
class SafetyVerdict:
    """Result of a safety check."""

    safe: bool
    reason: str


class SafetyManager:
    """Evaluates safety conditions and produces a stop/go verdict.

    Parameters
    ----------
    config
        Optional SafetyConfig. Defaults to SafetyConfig().
    clock
        Optional zero-argument callable returning the current time as a float,
        used for data-timeout checks. Defaults to time.monotonic. Injecting a
        deterministic clock makes pipeline/dropout tests possible without sleep.

    Usage::

        mgr = SafetyManager()
        verdict = mgr.check(safety_input)
        if not verdict.safe:
            g1.stop()
    """

    def __init__(
        self,
        config: SafetyConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._config = config or SafetyConfig()
        self._clock = clock

    @property
    def config(self) -> SafetyConfig:
        return self._config

    @property
    def clock(self) -> Callable[[], float]:
        return self._clock

    def check(self, inp: SafetyInput) -> SafetyVerdict:
        """Return a verdict for the current cycle.

        Conditions are checked in priority order; the first match wins.
        """
        cfg = self._config

        # 1. Manual stop
        if inp.manual_stop:
            return SafetyVerdict(safe=False, reason="manual_stop")

        # 2. Serial error
        if inp.serial_error:
            return SafetyVerdict(safe=False, reason="serial_error")

        # 3. SDK error
        if inp.sdk_error:
            return SafetyVerdict(safe=False, reason="sdk_error")

        # 4. SDK request timeout. This is distinct from a hard SDK error so
        #    the bridge can attempt throttled zero-velocity recovery.
        if inp.sdk_timeout:
            return SafetyVerdict(safe=False, reason="sdk_timeout")

        # 5. Data invalid or timeout — checked together because both mean
        #    we have no reliable position to act on.
        if not inp.data_valid:
            return SafetyVerdict(safe=False, reason="data_invalid")

        numeric_values = (
            inp.timestamp,
            inp.distance_m,
            inp.azimuth_deg,
            inp.distance_jump,
            inp.angle_jump,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            return SafetyVerdict(safe=False, reason="data_invalid")

        if inp.timestamp <= 0:
            return SafetyVerdict(safe=False, reason="data_timeout")

        now = self._clock()
        if now - inp.timestamp > cfg.data_timeout_s:
            return SafetyVerdict(safe=False, reason="data_timeout")

        # 6. Distance too close
        if inp.distance_m < cfg.min_safe_distance_m:
            return SafetyVerdict(safe=False, reason="too_close")

        # 7. Distance jump
        if inp.distance_jump > cfg.max_distance_jump_m:
            return SafetyVerdict(safe=False, reason="distance_jump")

        # 8. Angle jump
        if inp.angle_jump > cfg.max_angle_jump_deg:
            return SafetyVerdict(safe=False, reason="angle_jump")

        return SafetyVerdict(safe=True, reason="ok")
