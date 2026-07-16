"""Target lock management with gate-based tracking."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import TargetConfig
from robot_follow.lidar_alx_assisted_follow.domain import (
    GatedTargetObservation,
    PreprocessedCloud,
)
from robot_follow.lidar_alx_assisted_follow.tracking.kalman import KalmanFilter2D


class LockState(str, Enum):
    """Target lock state machine states."""

    EMPTY = "empty"  # No target selected
    CANDIDATE = "candidate"  # Target selected but not following
    FOLLOWING = "following"  # Following active
    LOST = "lost"  # Target lost (temporary)


@dataclass
class TargetLockSnapshot:
    """Immutable snapshot of target lock state."""

    state: LockState
    center_xy: Optional[Tuple[float, float]]
    last_seen_at_s: float
    requires_start: bool
    sequence: int


@dataclass
class TargetLockManager:
    """Manages single local target lock with gate-based tracking.

    The manager implements:
    - Web-driven target selection with initial circular gate
    - Dynamic elliptical gate tracking
    - LOST timeout handling
    - Lock state transitions

    State transitions:
    - EMPTY -> CANDIDATE: select() with valid click
    - CANDIDATE -> FOLLOWING: start() with valid gate
    - FOLLOWING -> FOLLOWING: update() with accepted gate
    - FOLLOWING -> LOST: update() with lost_timeout_s elapsed
    - LOST -> CANDIDATE: select() with new click
    - * -> EMPTY: unlock()
    - * -> CANDIDATE: stop_follow() (preserves target)
    """

    config: TargetConfig
    state: LockState = LockState.EMPTY
    center_xy: Optional[Tuple[float, float]] = None
    last_seen_at_s: float = 0.0
    requires_start: bool = False
    sequence: int = 0

    # Internal tracking state
    _last_observation: Optional[GatedTargetObservation] = None
    _kalman: KalmanFilter2D = field(init=False, repr=False)
    _follow_center_xy: Optional[Tuple[float, float]] = field(
        default=None, init=False, repr=False
    )

    def __post_init__(self) -> None:
        self._kalman = self._new_kalman()

    @property
    def follow_center_xy(self) -> Optional[Tuple[float, float]]:
        """Return the APF-only target center, filtered when enabled."""
        return self._follow_center_xy

    def snapshot(self) -> TargetLockSnapshot:
        """Get immutable snapshot of current state."""
        return TargetLockSnapshot(
            state=self.state,
            center_xy=self.center_xy,
            last_seen_at_s=self.last_seen_at_s,
            requires_start=self.requires_start,
            sequence=self.sequence,
        )

    def select(
        self,
        cloud: PreprocessedCloud,
        click_xy: Tuple[float, float],
        now_s: float,
        measurement_timestamp_s: Optional[float] = None,
    ) -> Optional[GatedTargetObservation]:
        """Select a new target near the clicked point.

        Applies circular initial gate with radius web_pick_radius_m.
        If gate succeeds, transitions to CANDIDATE state.

        Parameters
        ----------
        cloud : PreprocessedCloud
            Preprocessed point cloud.
        click_xy : tuple[float, float]
            Clicked point in body frame.
        now_s : float
            Current monotonic time.

        Returns
        -------
        GatedTargetObservation or None
            Observation if gate succeeded, None otherwise.
        """
        if cloud.points_xyz.size == 0:
            return None

        # Initial circular gate
        click_x, click_y = click_xy
        distances = np.hypot(
            cloud.points_xyz[:, 0] - click_x,
            cloud.points_xyz[:, 1] - click_y,
        )

        initial_mask = distances <= self.config.web_pick_radius_m
        initial_indices = np.where(initial_mask)[0]

        if initial_indices.size == 0:
            return None

        # Build observation from initial gate
        observation = self._build_observation(
            cloud, initial_indices, click_xy, now_s
        )

        if observation is None:
            return None

        # Validate observation against quality checks
        if not self._check_observation_quality(observation):
            return None

        # Accept as candidate
        self.state = LockState.CANDIDATE
        self.center_xy = observation.center_xy
        self.last_seen_at_s = observation.observed_at_s
        self._last_observation = observation
        self._reset_follow_center(
            observation.center_xy,
            now_s if measurement_timestamp_s is None else measurement_timestamp_s,
        )
        self.requires_start = True
        self.sequence += 1

        return observation

    def update(
        self,
        cloud: PreprocessedCloud,
        now_s: float,
        measurement_timestamp_s: Optional[float] = None,
    ) -> Tuple[Optional[GatedTargetObservation], bool]:
        """Update target tracking with new cloud.

        Applies dynamic elliptical gate based on last accepted center.
        Advances LOST timer if gate fails.

        Parameters
        ----------
        cloud : PreprocessedCloud
            Preprocessed point cloud.
        now_s : float
            Current monotonic time.

        Returns
        -------
        tuple[Optional[GatedTargetObservation], bool]
            (observation, lost) where:
            - observation: GatedTargetObservation if gate succeeded, None otherwise
            - lost: True if target is now in LOST state
        """
        if self.state not in (LockState.CANDIDATE, LockState.FOLLOWING):
            return None, False

        if self.center_xy is None or cloud.points_xyz.size == 0:
            # No center or no points - advance lost timer
            elapsed = now_s - self.last_seen_at_s
            if elapsed > self.config.lost_timeout_s:
                self._enter_lost_state()
                return None, True
            return None, False

        # Dynamic elliptical gate
        observation = self._apply_dynamic_gate(cloud, now_s)

        if observation is not None and self._check_observation_quality(observation):
            # Gate succeeded
            self.center_xy = observation.center_xy
            self.last_seen_at_s = observation.observed_at_s
            self._last_observation = observation
            self._update_follow_center(
                observation.center_xy,
                now_s if measurement_timestamp_s is None else measurement_timestamp_s,
            )

            # Check displacement for c_t
            if self._last_observation is not None:
                displacement = np.hypot(
                    observation.center_xy[0] - self._last_observation.center_xy[0],
                    observation.center_xy[1] - self._last_observation.center_xy[1],
                )
                if displacement > self.config.max_center_displacement_m:
                    # Large displacement - still update center but c_t will be 0
                    pass

            return observation, False
        else:
            # Gate failed - advance lost timer
            elapsed = now_s - self.last_seen_at_s
            if elapsed > self.config.lost_timeout_s:
                self._enter_lost_state()
                return None, True
            return None, False

    def start(self) -> bool:
        """Transition from CANDIDATE to FOLLOWING.

        Returns
        -------
        bool
            True if transition succeeded.
        """
        if self.state != LockState.CANDIDATE:
            return False

        self.state = LockState.FOLLOWING
        self.requires_start = False
        self.sequence += 1
        return True

    def set_external_target(
        self,
        center_xy: Tuple[float, float],
        now_s: float,
    ) -> None:
        """Update the target from an already filtered external position source.

        The runtime uses this for a time-matched ALX body position. LiDAR can
        then remain an obstacle source instead of re-identifying the wearer.
        """
        self.center_xy = (float(center_xy[0]), float(center_xy[1]))
        self._follow_center_xy = self.center_xy
        self.last_seen_at_s = now_s
        self._last_observation = None
        if self.state in (LockState.EMPTY, LockState.LOST):
            self.state = LockState.CANDIDATE
            self.requires_start = True
            self.sequence += 1

    def stop_follow(self) -> None:
        """Stop following and return to CANDIDATE state.

        Preserves target center for tracking.
        """
        if self.state == LockState.FOLLOWING:
            self.state = LockState.CANDIDATE
            self.requires_start = True
            self.sequence += 1

    def unlock(self) -> None:
        """Unlock and clear target completely.

        Transitions to EMPTY state.
        """
        self.state = LockState.EMPTY
        self.center_xy = None
        self.last_seen_at_s = 0.0
        self._last_observation = None
        self._clear_follow_center()
        self.requires_start = False
        self.sequence += 1

    def _apply_dynamic_gate(
        self,
        cloud: PreprocessedCloud,
        now_s: float,
    ) -> Optional[GatedTargetObservation]:
        """Apply dynamic elliptical gate centered on last accepted center.

        Ellipse is oriented with radial axis toward target center.
        """
        cx, cy = self.center_xy

        # Compute radial unit vector u and tangential unit vector v
        r = np.hypot(cx, cy)
        if r < 1e-6:
            # Center at origin - use default axes
            u = np.array([1.0, 0.0])
            v = np.array([0.0, 1.0])
        else:
            u = np.array([cx, cy]) / r
            v = np.array([-u[1], u[0]])  # Left normal

        # Compute delta vectors from center
        delta = cloud.points_xyz[:, :2] - np.array([cx, cy])

        # Project onto radial (u) and tangential (v) axes
        radial_proj = delta @ u
        tangential_proj = delta @ v

        # Elliptical gate check
        inside_mask = (
            (radial_proj / self.config.gate_radial_radius_m) ** 2
            + (tangential_proj / self.config.gate_tangential_radius_m) ** 2
            <= 1.0
        )

        gate_indices = np.where(inside_mask)[0]

        if gate_indices.size == 0:
            return None

        return self._build_observation(cloud, gate_indices, (cx, cy), now_s)

    def _build_observation(
        self,
        cloud: PreprocessedCloud,
        indices: np.ndarray,
        expected_center_xy: Tuple[float, float],
        now_s: float,
    ) -> Optional[GatedTargetObservation]:
        """Build GatedTargetObservation from gate indices.

        Computes all required metrics for the gated points.
        """
        if indices.size == 0:
            return None

        points = cloud.points_xyz[indices]

        # Compute center as component-wise median
        center_x = float(np.median(points[:, 0]))
        center_y = float(np.median(points[:, 1]))
        center_xy = (center_x, center_y)

        # Build target mask (in downsampled index domain)
        target_mask = np.zeros(cloud.points_xyz.shape[0], dtype=bool)
        target_mask[indices] = True

        # Compute metrics
        point_count = indices.size

        # Vertical extent: max(z) - min(z)
        vertical_extent_m = float(np.max(points[:, 2]) - np.min(points[:, 2]))

        # Horizontal footprint: max(max(x)-min(x), max(y)-min(y))
        horizontal_footprint_m = float(
            max(
                np.max(points[:, 0]) - np.min(points[:, 0]),
                np.max(points[:, 1]) - np.min(points[:, 1]),
            )
        )

        # Radial MAD
        radii = np.hypot(points[:, 0], points[:, 1])
        median_radius = np.median(radii)
        radial_mad_m = float(np.median(np.abs(radii - median_radius)))

        # Make support points read-only
        support_points_xyz = points.copy()
        support_points_xyz.flags.writeable = False

        return GatedTargetObservation(
            center_xy=center_xy,
            target_mask=target_mask,
            support_points_xyz=support_points_xyz,
            point_count=point_count,
            vertical_extent_m=vertical_extent_m,
            horizontal_footprint_m=horizontal_footprint_m,
            radial_mad_m=radial_mad_m,
            observed_at_s=now_s,
        )

    def _check_observation_quality(self, obs: GatedTargetObservation) -> bool:
        """Check if observation meets quality thresholds."""
        # Point count
        if obs.point_count < self.config.min_support_points:
            return False

        # Vertical extent
        if obs.vertical_extent_m < self.config.min_vertical_extent_m:
            return False

        # Horizontal footprint
        if obs.horizontal_footprint_m > self.config.max_horizontal_footprint_m:
            return False

        # Radial MAD
        if obs.radial_mad_m > self.config.max_radial_mad_m:
            return False

        return True

    def _new_kalman(self) -> KalmanFilter2D:
        return KalmanFilter2D(
            process_noise=self.config.kalman_process_noise,
            measurement_noise=self.config.kalman_measurement_noise,
            max_interval_s=self.config.max_frame_interval_s,
        )

    def _reset_follow_center(
        self,
        raw_center_xy: Tuple[float, float],
        measurement_timestamp_s: float,
    ) -> None:
        self._kalman = self._new_kalman()
        if self.config.kalman_enabled:
            self._follow_center_xy = self._kalman.reset(
                raw_center_xy[0], raw_center_xy[1], measurement_timestamp_s
            )
        else:
            self._follow_center_xy = raw_center_xy

    def _update_follow_center(
        self,
        raw_center_xy: Tuple[float, float],
        measurement_timestamp_s: float,
    ) -> None:
        if self.config.kalman_enabled:
            self._follow_center_xy = self._kalman.update(
                raw_center_xy[0], raw_center_xy[1], measurement_timestamp_s
            )
        else:
            self._follow_center_xy = raw_center_xy

    def _clear_follow_center(self) -> None:
        self._kalman = self._new_kalman()
        self._follow_center_xy = None

    def _enter_lost_state(self) -> None:
        """Transition to LOST state and clear target."""
        self.state = LockState.LOST
        self.center_xy = None
        self._last_observation = None
        self._clear_follow_center()
        self.sequence += 1
