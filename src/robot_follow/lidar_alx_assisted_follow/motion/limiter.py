"""Command slew rate limiter for smooth motion."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import OutputMotionConfig
from robot_follow.lidar_alx_assisted_follow.domain.models import MotionIntent


@dataclass
class CommandSlewRateLimiter:
    """Slew rate limiter for smooth velocity commands.

    Limits acceleration/deceleration for each velocity component independently.
    Tracks last commanded values and timestamps.

    The limiter uses elapsed time between accepted clouds, clamped to [0.001, 0.10] s.
    Mandatory stop calls reset() and dispatches a stop command without slew limiting.
    """

    config: OutputMotionConfig
    last_vx: float = 0.0
    last_vy: float = 0.0
    last_omega: float = 0.0
    last_time_s: Optional[float] = field(default=None)

    def limit(self, intent: MotionIntent, now_s: Optional[float] = None) -> MotionIntent:
        """Apply slew rate limiting to motion intent.

        Parameters
        ----------
        intent : MotionIntent
            Requested motion intent.
        now_s : float or None
            Current monotonic time, or None to use time.monotonic().

        Returns
        -------
        MotionIntent
            Limited motion intent with updated timestamp.
        """
        if now_s is None:
            now_s = time.monotonic()

        if not self.config.slew_rate_limit_enabled:
            limited_vx = float(
                np.clip(intent.vx, -self.config.max_reverse_vx, self.config.max_vx)
            )
            limited_vy = float(
                np.clip(
                    intent.vy,
                    -self.config.max_abs_vy,
                    self.config.max_abs_vy,
                )
            )
            limited_omega = float(
                np.clip(
                    intent.omega,
                    -self.config.max_abs_omega,
                    self.config.max_abs_omega,
                )
            )
            self.last_vx = limited_vx
            self.last_vy = limited_vy
            self.last_omega = limited_omega
            self.last_time_s = now_s
            return MotionIntent(
                sequence=intent.sequence,
                vx=limited_vx,
                vy=limited_vy,
                omega=limited_omega,
                reason=intent.reason,
                generated_at_s=now_s,
            )

        # First call or after reset
        if self.last_time_s is None:
            self.last_time_s = now_s
            self.last_vx = 0.0
            self.last_vy = 0.0
            self.last_omega = 0.0
            return MotionIntent(
                sequence=intent.sequence,
                vx=0.0,
                vy=0.0,
                omega=0.0,
                reason=intent.reason,
                generated_at_s=now_s,
            )

        dt = now_s - self.last_time_s

        # Clamp dt to reasonable range [1ms, 100ms] per plan
        dt = float(np.clip(dt, 0.001, 0.10))

        # Limit each component
        limited_vx = self._limit_component(
            intent.vx,
            self.last_vx,
            dt,
            self.config.accel_vx,
            self.config.decel_vx,
        )

        limited_vy = self._limit_component(
            intent.vy,
            self.last_vy,
            dt,
            self.config.accel_vy,
            self.config.decel_vy,
        )

        limited_omega = self._limit_component(
            intent.omega,
            self.last_omega,
            dt,
            self.config.accel_omega,
            self.config.decel_omega,
        )

        # Apply velocity limits
        limited_vx = float(
            np.clip(limited_vx, -self.config.max_reverse_vx, self.config.max_vx)
        )
        limited_vy = float(np.clip(limited_vy, -self.config.max_abs_vy, self.config.max_abs_vy))
        limited_omega = float(np.clip(limited_omega, -self.config.max_abs_omega, self.config.max_abs_omega))

        # Update state
        self.last_vx = limited_vx
        self.last_vy = limited_vy
        self.last_omega = limited_omega
        self.last_time_s = now_s

        return MotionIntent(
            sequence=intent.sequence,
            vx=limited_vx,
            vy=limited_vy,
            omega=limited_omega,
            reason=intent.reason,
            generated_at_s=now_s,
        )

    def reset(self) -> None:
        """Reset limiter state to zero.

        Called on mandatory stop to reset internal state.
        """
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0
        self.last_time_s = None

    def _limit_component(
        self,
        requested: float,
        last: float,
        dt: float,
        accel: float,
        decel: float,
    ) -> float:
        """Limit a single velocity component.

        Parameters
        ----------
        requested : float
            Requested velocity value.
        last : float
            Last commanded velocity.
        dt : float
            Time delta (s), clamped to [0.001, 0.10].
        accel : float
            Acceleration limit (m/s² or rad/s²).
        decel : float
            Deceleration limit (m/s² or rad/s²).

        Returns
        -------
        float
            Limited velocity value.
        """
        delta = requested - last

        if delta > 0:
            # Accelerating (increasing magnitude toward requested)
            max_delta = accel * dt
            limited_delta = min(delta, max_delta)
        else:
            # Decelerating (decreasing magnitude toward requested)
            max_delta = decel * dt
            limited_delta = max(delta, -max_delta)

        return last + limited_delta
