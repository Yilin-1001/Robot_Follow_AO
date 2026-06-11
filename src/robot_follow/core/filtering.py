"""Lightweight low-pass filter with jump detection for ALX-G100 data."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FilteredTarget:
    """Output of the target filter after one update."""

    distance_m: float
    azimuth_deg: float
    distance_jump: float
    angle_jump: float
    initialized: bool


class TargetFilter:
    """First-order low-pass filter on distance and azimuth with jump detection.

    Parameters
    ----------
    alpha : float
        Smoothing factor in (0, 1].  Smaller values mean heavier smoothing.
        Default 0.35 balances responsiveness and noise rejection.
    """

    def __init__(self, alpha: float = 0.35) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self._alpha = alpha
        self._distance: float | None = None
        self._azimuth: float | None = None

    @property
    def alpha(self) -> float:
        return self._alpha

    @property
    def initialized(self) -> bool:
        return self._distance is not None

    def update(self, distance_m: float, azimuth_deg: float) -> FilteredTarget:
        """Feed a new raw sample and get the filtered result.

        The first call initialises the filter without smoothing; subsequent
        calls apply exponential moving average and report jumps.
        """
        if self._distance is None:
            # First sample – accept directly.
            self._distance = float(distance_m)
            self._azimuth = float(azimuth_deg)
            return FilteredTarget(
                distance_m=self._distance,
                azimuth_deg=self._azimuth,
                distance_jump=0.0,
                angle_jump=0.0,
                initialized=True,
            )

        prev_d = self._distance
        prev_a = self._azimuth

        dist_jump = abs(distance_m - prev_d)
        angle_jump = abs(azimuth_deg - prev_a)

        a = self._alpha
        self._distance = prev_d + a * (distance_m - prev_d)
        self._azimuth = prev_a + a * (azimuth_deg - prev_a)

        return FilteredTarget(
            distance_m=self._distance,
            azimuth_deg=self._azimuth,
            distance_jump=dist_jump,
            angle_jump=angle_jump,
            initialized=True,
        )

    def reset(self) -> None:
        """Clear filter state so the next update re-initialises."""
        self._distance = None
        self._azimuth = None