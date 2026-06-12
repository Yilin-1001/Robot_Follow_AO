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

    target_distance_m: float = 1.5
    min_safe_distance_m: float = 0.5
    hold_tolerance_m: float = 0.15
    max_forward_vx: float = 0.20
    max_backward_vx: float = 0.12
    max_follow_distance_m: float = 4.0
    max_omega: float = 0.60       # rad/s
    angle_deadzone_deg: float = 7.0
    large_angle_deg: float = 20.0
    distance_gain_forward: float = 0.18
    distance_gain_backward: float = 0.15
    angle_gain: float = 0.5      # omega = gain * azimuth_rad

    @property
    def hold_min_distance_m(self) -> float:
        return self.target_distance_m - self.hold_tolerance_m

    @property
    def hold_max_distance_m(self) -> float:
        return self.target_distance_m + self.hold_tolerance_m


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
            vx = self._distance_vx(dist)
            # Reduce forward speed proportional to misalignment
            vx *= 1.0 - (abs_az / cfg.large_angle_deg) * 0.6
            return FollowOutput(
                vx=vx, vy=0.0, omega=omega, reason="tracking_turning"
            )

        # --- Within deadzone: hold distance (bidirectional) --------------
        vx = self._distance_vx(dist)
        # Small residual turn correction inside deadzone
        omega = self._clamp_omega(
            cfg.angle_gain * math.radians(az) * 0.3
        )
        if vx < -1e-4:
            reason = "tracking_backward"
        elif vx > 1e-4:
            reason = "tracking_forward"
        elif abs(omega) < 1e-4:
            reason = "holding"
        else:
            reason = "tracking_turning"
        return FollowOutput(vx=vx, vy=0.0, omega=omega, reason=reason)

    # ------------------------------------------------------------------
    def _distance_vx(self, distance_m: float) -> float:
        cfg = self._config
        if distance_m < cfg.hold_min_distance_m:
            error = distance_m - cfg.hold_min_distance_m
            return max(
                -cfg.max_backward_vx,
                cfg.distance_gain_backward * error,
            )
        if distance_m > cfg.hold_max_distance_m:
            error = distance_m - cfg.hold_max_distance_m
            return min(
                cfg.max_forward_vx,
                cfg.distance_gain_forward * error,
            )
        return 0.0

    def _clamp_omega(self, omega: float) -> float:
        return max(-self._config.max_omega, min(self._config.max_omega, omega))
