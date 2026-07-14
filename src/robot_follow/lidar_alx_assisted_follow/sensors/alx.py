"""ALX buffering and transform for LiDAR + ALX Assisted Follow."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import ALXConfig, CalibrationConfig


@dataclass(frozen=True)
class ALXSnapshot:
    """Immutable ALX serial sample."""

    received_at: float
    distance_m: float
    azimuth_deg: float
    elevation_deg: float
    tag_id: int
    error: bool


class ALXSnapshotBuffer:
    """Causal ALX buffer that maintains up to 1s of deduplicated snapshots.

    Snapshots are deduplicated by exact ``received_at`` timestamp.
    Buffer selects the newest snapshot with ``received_at <= cloud_time``
    and skew within ``cloud_max_skew_s``.
    """

    def __init__(
        self,
        alx_config: ALXConfig,
        calibration_config: CalibrationConfig,
        buffer_duration_s: float = 1.0,
    ) -> None:
        """Initialize ALX buffer with config objects.

        Parameters
        ----------
        alx_config : ALXConfig
            ALX configuration for distance bounds and skew tolerance.
        calibration_config : CalibrationConfig
            Calibration configuration for coordinate transform.
        buffer_duration_s : float
            Maximum buffer duration in seconds (default: 1.0).
        """
        self._alx_config = alx_config
        self._calibration_config = calibration_config
        self._buffer_duration_s = buffer_duration_s
        self._buffer: Deque[ALXSnapshot] = Deque()  # type: ignore[type-arg]
        self._last_received_at: Optional[float] = None

    def append(self, snapshot: ALXSnapshot, now: Optional[float] = None) -> None:
        """Append a new ALX snapshot, deduplicating by received_at.

        Parameters
        ----------
        snapshot : ALXSnapshot
            New ALX sample to add to buffer.
        now : float or None
            Current monotonic time for age-based trimming.
            Defaults to ``time.monotonic()`` when None.
        """
        # Deduplicate by exact received_at
        if (
            self._last_received_at is not None
            and snapshot.received_at == self._last_received_at
        ):
            return

        self._buffer.append(snapshot)
        self._last_received_at = snapshot.received_at

        # Trim old entries
        if now is None:
            now = time.monotonic()
        cutoff = now - self._buffer_duration_s
        while self._buffer and self._buffer[0].received_at < cutoff:
            self._buffer.popleft()

    def for_cloud(
        self,
        cloud_received_at_s: float,
        session_tag_id: Optional[int],
    ) -> Optional[ALXSnapshot]:
        """Find the best ALX snapshot for a given cloud.

        Selects the newest snapshot with:
        - received_at <= cloud_received_at_s
        - cloud_time - snapshot.received_at <= cloud_max_skew_s
        - matching session_tag_id (if set)

        Parameters
        ----------
        cloud_received_at_s : float
            Monotonic receive time of the cloud.
        session_tag_id : int or None
            Current session tag ID (from wristband binding), or None if unset.

        Returns
        -------
        ALXSnapshot or None
            Best matching snapshot, or None if none found.
        """
        # Walk backwards through buffer (newest first)
        best: Optional[ALXSnapshot] = None
        for snapshot in reversed(list(self._buffer)):
            # Must not be in the future
            if snapshot.received_at > cloud_received_at_s:
                continue

            # Must be within skew tolerance
            age = cloud_received_at_s - snapshot.received_at
            if age > self._alx_config.cloud_max_skew_s:
                continue

            # Must match session tag (if set)
            if session_tag_id is not None and snapshot.tag_id != session_tag_id:
                continue

            best = snapshot
            break

        return best

    def transform_to_body(self, snapshot: ALXSnapshot) -> Optional[np.ndarray]:
        """Transform ALX polar coordinates to body-frame XY.

        Parameters
        ----------
        snapshot : ALXSnapshot
            ALX sample to transform.

        Returns
        -------
        np.ndarray or None
            Body-frame XY coordinates [x, y], or None if invalid.
        """
        # Validate snapshot
        if snapshot.error:
            return None

        if not np.isfinite(snapshot.distance_m):
            return None

        if snapshot.distance_m < self._alx_config.min_distance_m:
            return None
        if snapshot.distance_m > self._alx_config.max_distance_m:
            return None

        if not (-180.0 <= snapshot.azimuth_deg <= 180.0):
            return None
        if not (-90.0 <= snapshot.elevation_deg <= 90.0):
            return None

        # Compute body-frame coordinates
        azimuth_rad = np.deg2rad(
            self._calibration_config.azimuth_sign
            * (snapshot.azimuth_deg + self._calibration_config.azimuth_zero_offset_deg)
        )

        raw_xy = np.array([
            snapshot.distance_m * np.cos(azimuth_rad),
            snapshot.distance_m * np.sin(azimuth_rad),
        ])

        # Apply calibration translation and rotation
        yaw_rad = np.deg2rad(self._calibration_config.yaw_deg)
        rotation = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad)],
            [np.sin(yaw_rad), np.cos(yaw_rad)],
        ])

        translation = np.array([
            self._calibration_config.x_m,
            self._calibration_config.y_m,
        ])

        body_xy = translation + rotation @ raw_xy

        # Make read-only
        body_xy.flags.writeable = False

        return body_xy