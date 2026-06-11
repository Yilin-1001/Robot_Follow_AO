"""Follow controller – converts filtered distance and azimuth to vx / omega.

Strategy: "align first, then approach".  The robot prioritises turning to face
the person before moving forward, which prevents diagonal approaches.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class FollowConfig:
    """Tunable parameters for the follow controller."""

    target_distance_m: float = 1.3
    min_safe_distance_m: float = 0.75
    max_vx: float = 0.20          # m/s
    max_omega: float = 0.30       # rad/s
    angle_deadzone_deg: float = 5.0
    large_angle_deg: float = 20.0
    distance_gain: float = 0.18   # vx = gain * distance_error
    angle_gain: float = 0.015     # omega = gain * azimuth_rad


@dataclass(frozen=True)
class FollowOutput:
    """Control output for one cycle."""

    vx: float
    vy: float
    omega: float
    reason: str


class FollowController:
    """Stateless P-controller for the follow behaviour."""

    def __init__(self, config: FollowConfig | None = None) -> None:
        self._config = config or FollowConfig()

    @property
    def config(self) -> FollowConfig:
        return self._config

    def compute(self, distance_m: float, azimuth_deg: float) -> FollowOutput:
        """Compute velocity commands from filtered sensor data.

        Parameters
        ----------
        distance_m
            Filtered distance to the tag (metres).
        azimuth_deg
            Filtered azimuth angle (degrees).  Positive values mean the tag
            is to one side; the sign convention is determined by the ALX-G100
            mounting and confirmed during hardware calibration.

        Returns
        -------
        FollowOutput with vx, vy (always 0), omega, and a reason tag.
        """
        cfg = self._config
        az = azimuth_deg
        dist = distance_m

        # --- Too close ---------------------------------------------------
        if dist < cfg.min_safe_distance_m:
            return FollowOutput(vx=0.0, vy=0.0, omega=0.0, reason="too_close")

        abs_az = abs(az)

        # --- Large angle: turn in place, no forward motion ---------------
        if abs_az > cfg.large_angle_deg:
            omega = self._clamp_omega(
                cfg.angle_gain * math.radians(az)
            )
            return FollowOutput(vx=0.0, vy=0.0, omega=omega, reason="aligning")

        # --- Medium angle: slow approach + turn correction ---------------
        if abs_az > cfg.angle_deadzone_deg:
            omega = self._clamp_omega(
                cfg.angle_gain * math.radians(az)
            )
            dist_err = dist - cfg.target_distance_m
            vx = max(0.0, self._clamp_vx(cfg.distance_gain * dist_err))
            # Reduce forward speed proportional to misalignment
            vx *= 1.0 - (abs_az / cfg.large_angle_deg) * 0.6
            return FollowOutput(
                vx=vx, vy=0.0, omega=omega, reason="tracking_turning"
            )

        # --- Within deadzone: approach / hold distance -------------------
        dist_err = dist - cfg.target_distance_m
        vx = max(0.0, self._clamp_vx(cfg.distance_gain * dist_err))
        # Small residual turn correction inside deadzone
        omega = self._clamp_omega(
            cfg.angle_gain * math.radians(az) * 0.3
        )
        if vx < 1e-4 and abs(omega) < 1e-4:
            reason = "holding"
        else:
            reason = "tracking_forward"
        return FollowOutput(vx=vx, vy=0.0, omega=omega, reason=reason)

    # ------------------------------------------------------------------
    def _clamp_vx(self, vx: float) -> float:
        return max(-self._config.max_vx, min(self._config.max_vx, vx))

    def _clamp_omega(self, omega: float) -> float:
        return max(-self._config.max_omega, min(self._config.max_omega, omega))