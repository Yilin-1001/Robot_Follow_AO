"""Compact two-dimensional constant-velocity Kalman filter."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


class KalmanFilter2D:
    """Smooth confirmed ``(x, y)`` target centers for APF consumption only."""

    def __init__(
        self,
        process_noise: float = 0.10,
        measurement_noise: float = 0.05,
        max_interval_s: float = 0.20,
    ) -> None:
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.max_interval_s = max_interval_s
        self._state = np.zeros(4, dtype=np.float64)
        self._covariance = np.zeros((4, 4), dtype=np.float64)
        self._last_timestamp_s: Optional[float] = None
        self._initialized = False

    @property
    def is_initialized(self) -> bool:
        """Whether a confirmed measurement has initialized this filter."""
        return self._initialized

    def reset(self, x: float, y: float, timestamp_s: float) -> Tuple[float, float]:
        """Initialize position from a confirmed measurement and clear velocity."""
        self._state[:] = (x, y, 0.0, 0.0)
        self._covariance.fill(0.0)
        self._covariance[0, 0] = 1.0
        self._covariance[1, 1] = 1.0
        self._covariance[2, 2] = 10.0
        self._covariance[3, 3] = 10.0
        self._last_timestamp_s = timestamp_s
        self._initialized = True
        return (float(x), float(y))

    def update(self, x: float, y: float, timestamp_s: float) -> Tuple[float, float]:
        """Predict then correct with one confirmed target-center observation."""
        if not self._initialized:
            return self.reset(x, y, timestamp_s)

        assert self._last_timestamp_s is not None
        dt = timestamp_s - self._last_timestamp_s
        if dt <= 0.0:
            return self._position()
        if dt > self.max_interval_s:
            return self.reset(x, y, timestamp_s)

        self._predict(dt)
        self._correct(x, y)
        self._last_timestamp_s = timestamp_s
        return self._position()

    def _predict(self, dt: float) -> None:
        transition = np.array(
            [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt],
             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dt2 = dt * dt
        dt3 = dt2 * dt
        process_covariance = self.process_noise * np.array(
            [[dt3 / 3.0, 0.0, dt2 / 2.0, 0.0],
             [0.0, dt3 / 3.0, 0.0, dt2 / 2.0],
             [dt2 / 2.0, 0.0, dt, 0.0],
             [0.0, dt2 / 2.0, 0.0, dt]],
            dtype=np.float64,
        )
        self._state = transition @ self._state
        self._covariance = transition @ self._covariance @ transition.T + process_covariance

    def _correct(self, x: float, y: float) -> None:
        innovation = np.array([x, y], dtype=np.float64) - self._state[:2]
        innovation_covariance = self._covariance[:2, :2] + self.measurement_noise * np.eye(2)
        gain = self._covariance[:, :2] @ np.linalg.inv(innovation_covariance)
        self._state += gain @ innovation
        self._covariance = (np.eye(4) - gain @ np.eye(2, 4)) @ self._covariance

    def _position(self) -> Tuple[float, float]:
        return (float(self._state[0]), float(self._state[1]))
