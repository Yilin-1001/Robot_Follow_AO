"""Pure ALX body-frame Kalman smoothing for ROS2 publication."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    ALXConfig,
    ALXRosConfig,
    CalibrationConfig,
)
from robot_follow.lidar_alx_assisted_follow.sensors.alx import (
    ALXSnapshot,
    ALXSnapshotBuffer,
)
from robot_follow.lidar_alx_assisted_follow.tracking.kalman import KalmanFilter2D


@dataclass(frozen=True)
class FilteredALXPosition:
    """A new Kalman-smoothed horizontal ALX body-frame position."""

    received_at: float
    x_m: float
    y_m: float


class ALXPositionPublisherCore:
    """Transform and smooth only newly received valid ALX samples.

    The class has no ROS dependency so its timing and filtering behavior can be
    tested locally.  The ROS node publishes a PointStamped only for a returned
    ``FilteredALXPosition``; a stale serial value is never republished.
    """

    def __init__(
        self,
        *,
        alx_config: ALXConfig,
        calibration_config: CalibrationConfig,
        ros_config: ALXRosConfig,
    ) -> None:
        self._transform = ALXSnapshotBuffer(
            alx_config=alx_config,
            calibration_config=calibration_config,
            buffer_duration_s=1.0,
        )
        self._filter = KalmanFilter2D(
            process_noise=ros_config.kalman_process_noise,
            measurement_noise=ros_config.kalman_measurement_noise,
            max_interval_s=ros_config.kalman_max_interval_s,
        )
        self._last_received_at: Optional[float] = None

    def update(self, snapshot: ALXSnapshot) -> Optional[FilteredALXPosition]:
        """Return one filtered position for a new valid serial sample.

        Duplicate snapshots are ignored.  Invalid data is deliberately not
        published: topic silence preserves a truthful freshness signal for
        downstream subscribers.
        """
        if self._last_received_at == snapshot.received_at:
            return None

        self._last_received_at = snapshot.received_at
        body_xy = self._transform.transform_to_body(snapshot)
        if body_xy is None:
            return None

        x_m, y_m = self._filter.update(
            float(body_xy[0]),
            float(body_xy[1]),
            snapshot.received_at,
        )
        return FilteredALXPosition(
            received_at=snapshot.received_at,
            x_m=x_m,
            y_m=y_m,
        )
