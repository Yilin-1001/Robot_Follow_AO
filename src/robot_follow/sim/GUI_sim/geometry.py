"""Geometry utilities for the GUI simulation.

World coordinates
-----------------
- x_m : metres, positive right
- y_m : metres, positive up
- heading_rad : robot heading in world frame

Screen coordinates
------------------
- x_px : pixels, positive right
- y_px : pixels, positive down

A :class:`SimGeometry` instance holds canvas dimensions and scale so that the
mouse-to-TargetSample conversion and world-to-screen drawing share the same
coordinate model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FOV_DEG: float = 120.0
HALF_FOV_DEG: float = 60.0
DEFAULT_PIXELS_PER_METER: int = 120
DEFAULT_MAX_RANGE_M: float = 10.0


# ---------------------------------------------------------------------------
# Detection result
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Detection:
    """Result of checking a target against the simulated sensor FOV."""

    distance_m: float
    azimuth_deg: float
    in_fov: bool
    in_range: bool
    valid: bool


# ---------------------------------------------------------------------------
# SimGeometry
# ---------------------------------------------------------------------------

class SimGeometry:
    """Coordinate transforms and detection-range logic for the GUI sim.

    Parameters
    ----------
    canvas_w, canvas_h
        Canvas dimensions in pixels.  The world origin maps to the centre.
    pixels_per_meter
        Scale factor (default 120).
    max_range_m
        Maximum detection range in metres (default 10).
    """

    def __init__(
        self,
        canvas_w: int = 800,
        canvas_h: int = 600,
        pixels_per_meter: int = DEFAULT_PIXELS_PER_METER,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
    ) -> None:
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h
        self.pixels_per_meter = pixels_per_meter
        self.max_range_m = max_range_m

    # ------------------------------------------------------------------
    # Coordinate transforms
    # ------------------------------------------------------------------

    def world_to_px(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Convert world (metres) to screen (pixels), origin at canvas centre."""
        cx = self.canvas_w / 2
        cy = self.canvas_h / 2
        px = cx + x_m * self.pixels_per_meter
        py = cy - y_m * self.pixels_per_meter  # y flipped
        return int(round(px)), int(round(py))

    def px_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """Convert screen (pixels) to world (metres), origin at canvas centre."""
        cx = self.canvas_w / 2
        cy = self.canvas_h / 2
        x_m = (px - cx) / self.pixels_per_meter
        y_m = (cy - py) / self.pixels_per_meter  # y flipped
        return x_m, y_m

    def polar_to_world(
        self,
        robot_x: float,
        robot_y: float,
        heading_rad: float,
        distance_m: float,
        azimuth_deg: float,
    ) -> Tuple[float, float]:
        """Convert robot-relative polar target observation to world metres."""
        angle = heading_rad + math.radians(azimuth_deg)
        return (
            robot_x + distance_m * math.cos(angle),
            robot_y + distance_m * math.sin(angle),
        )

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def detect(
        self,
        robot_x: float,
        robot_y: float,
        heading_rad: float,
        target_x: float,
        target_y: float,
    ) -> Detection:
        """Compute distance, azimuth, FOV and range validity.

        *azimuth_deg* is the signed angle from the robot heading to the
        target direction, normalised to [-180, 180].
        """
        dx = target_x - robot_x
        dy = target_y - robot_y

        distance_m = math.hypot(dx, dy)
        world_angle_rad = math.atan2(dy, dx)
        azimuth_rad = world_angle_rad - heading_rad
        azimuth_deg = self.normalize_angle_deg(math.degrees(azimuth_rad))

        in_fov = abs(azimuth_deg) <= HALF_FOV_DEG
        in_range = distance_m <= self.max_range_m
        valid = in_fov and in_range

        return Detection(
            distance_m=distance_m,
            azimuth_deg=azimuth_deg,
            in_fov=in_fov,
            in_range=in_range,
            valid=valid,
        )

    # ------------------------------------------------------------------
    # Angle helpers
    # ------------------------------------------------------------------

    @staticmethod
    def normalize_angle_deg(angle: float) -> float:
        """Normalise *angle* to the range ``[-180, 180)``.

        ``-180`` is mapped to ``+180`` by convention (half-open interval).
        """
        a = angle % 360.0
        if a > 180.0:
            a -= 360.0
        if a == -180.0:
            a = 180.0
        return a
