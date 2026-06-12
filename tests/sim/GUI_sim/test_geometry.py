"""Tests for ``robot_follow.sim.GUI_sim.geometry``.

Covers coordinate transforms, azimuth / distance computation, FOV and range
validity checks, and angle normalisation.
"""

from __future__ import annotations

import math

import pytest

from robot_follow.sim.GUI_sim.geometry import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_PIXELS_PER_METER,
    FOV_DEG,
    HALF_FOV_DEG,
    SimGeometry,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _azimuth_for_angle(angle_deg: float, dist_m: float = 2.0):
    """Return (robot_x, robot_y, robot_heading, target_x, target_y).

    The target is placed at *dist_m* metres from the robot at the given
    *angle_deg* relative to the robot heading.
    """
    heading_rad = 0.0
    abs_angle_rad = math.radians(angle_deg)
    tx = dist_m * math.cos(abs_angle_rad)
    ty = dist_m * math.sin(abs_angle_rad)
    return (0.0, 0.0, heading_rad, tx, ty)


# ---------------------------------------------------------------------------
# Coordinate transforms
# ---------------------------------------------------------------------------

class TestCoordinateTransforms:
    """World <-> screen coordinate conversion."""

    def test_world_to_screen_origin_center(self):
        geo = SimGeometry(canvas_w=800, canvas_h=600)
        sx, sy = geo.world_to_px(0.0, 0.0)
        assert sx == 400
        assert sy == 300

    def test_world_to_screen_positive_x(self):
        geo = SimGeometry(canvas_w=800, canvas_h=600)
        sx, sy = geo.world_to_px(1.0, 0.0)
        assert sx == 400 + int(round(DEFAULT_PIXELS_PER_METER))
        assert sy == 300  # y unchanged

    def test_world_to_screen_positive_y_goes_up(self):
        geo = SimGeometry(canvas_w=800, canvas_h=600)
        sx, sy = geo.world_to_px(0.0, 1.0)
        assert sx == 400
        assert sy == 300 - int(round(DEFAULT_PIXELS_PER_METER))

    def test_round_trip_world_px(self):
        geo = SimGeometry(canvas_w=800, canvas_h=600)
        wx, wy = 1.5, -2.3
        sx, sy = geo.world_to_px(wx, wy)
        rwx, rwy = geo.px_to_world(sx, sy)
        # Should be within half a pixel
        assert abs(rwx - wx) < 0.05
        assert abs(rwy - wy) < 0.05

    def test_px_to_world_origin_center(self):
        geo = SimGeometry(canvas_w=800, canvas_h=600)
        wx, wy = geo.px_to_world(400, 300)
        assert abs(wx) < 0.01
        assert abs(wy) < 0.01

    def test_polar_to_world_dead_ahead(self):
        geo = SimGeometry()
        wx, wy = geo.polar_to_world(
            robot_x=1.0,
            robot_y=2.0,
            heading_rad=0.0,
            distance_m=1.5,
            azimuth_deg=0.0,
        )
        assert wx == pytest.approx(2.5)
        assert wy == pytest.approx(2.0)

    def test_polar_to_world_respects_heading_and_azimuth(self):
        geo = SimGeometry()
        wx, wy = geo.polar_to_world(
            robot_x=0.0,
            robot_y=0.0,
            heading_rad=math.radians(90.0),
            distance_m=2.0,
            azimuth_deg=-90.0,
        )
        assert wx == pytest.approx(2.0)
        assert wy == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# Azimuth, distance, FOV
# ---------------------------------------------------------------------------

class TestAzimuthAndFOV:
    """Target detection geometry."""

    def test_target_dead_ahead(self):
        """Target directly in front of robot: azimuth 0, valid True."""
        geo = SimGeometry()
        rx, ry, rh, tx, ty = _azimuth_for_angle(0.0)
        det = geo.detect(rx, ry, rh, tx, ty)
        assert det.azimuth_deg == pytest.approx(0.0, abs=0.01)
        assert det.in_fov is True
        assert det.in_range is True
        assert det.valid is True

    def test_plus_60_deg_valid(self):
        """Target at exactly +60 deg relative to heading: valid."""
        geo = SimGeometry()
        rx, ry, rh, tx, ty = _azimuth_for_angle(60.0)
        det = geo.detect(rx, ry, rh, tx, ty)
        assert det.azimuth_deg == pytest.approx(60.0, abs=0.01)
        assert det.in_fov is True
        assert det.valid is True

    def test_minus_60_deg_valid(self):
        """Target at exactly -60 deg relative to heading: valid."""
        geo = SimGeometry()
        rx, ry, rh, tx, ty = _azimuth_for_angle(-60.0)
        det = geo.detect(rx, ry, rh, tx, ty)
        assert det.azimuth_deg == pytest.approx(-60.0, abs=0.01)
        assert det.in_fov is True
        assert det.valid is True

    def test_plus_60_1_deg_invalid(self):
        """Target at +60.1 deg: just outside FOV → invalid."""
        geo = SimGeometry()
        rx, ry, rh, tx, ty = _azimuth_for_angle(60.1)
        det = geo.detect(rx, ry, rh, tx, ty)
        assert det.azimuth_deg == pytest.approx(60.1, abs=0.05)
        assert det.in_fov is False
        assert det.valid is False

    def test_minus_60_1_deg_invalid(self):
        """Target at -60.1 deg: just outside FOV → invalid."""
        geo = SimGeometry()
        rx, ry, rh, tx, ty = _azimuth_for_angle(-60.1)
        det = geo.detect(rx, ry, rh, tx, ty)
        assert det.azimuth_deg == pytest.approx(-60.1, abs=0.05)
        assert det.in_fov is False
        assert det.valid is False

    def test_beyond_max_range_invalid(self):
        """Target beyond max detection range: invalid."""
        geo = SimGeometry(max_range_m=5.0)
        det = geo.detect(0, 0, 0, 6.0, 0.0)
        assert det.in_fov is True
        assert det.in_range is False
        assert det.valid is False

    def test_at_max_range_valid(self):
        """Target exactly at max range: valid (closed interval)."""
        geo = SimGeometry(max_range_m=5.0)
        det = geo.detect(0, 0, 0, 5.0, 0.0)
        assert det.in_range is True
        assert det.valid is True

    def test_distance_matches(self):
        """Distance is the Euclidean distance between robot and target."""
        geo = SimGeometry()
        det = geo.detect(0, 0, 0, 3.0, 4.0)
        assert det.distance_m == pytest.approx(5.0, abs=0.01)

    def test_heading_offset(self):
        """Azimuth accounts for non-zero robot heading."""
        geo = SimGeometry()
        # Robot facing +y (heading = pi/2), target at (2, 0)
        # World angle = 0 deg, heading = 90 deg, azimuth = -90 deg
        det = geo.detect(0, 0, math.pi / 2, 2.0, 0.0)
        assert det.azimuth_deg == pytest.approx(-90.0, abs=0.1)


# ---------------------------------------------------------------------------
# Angle normalisation
# ---------------------------------------------------------------------------

class TestAngleNormalisation:
    """``normalize_angle_deg`` clamps to [-180, 180]."""

    def test_zero(self):
        assert SimGeometry.normalize_angle_deg(0.0) == pytest.approx(0.0)

    def test_positive_within_range(self):
        assert SimGeometry.normalize_angle_deg(45.0) == pytest.approx(45.0)

    def test_negative_within_range(self):
        assert SimGeometry.normalize_angle_deg(-45.0) == pytest.approx(-45.0)

    def test_wrap_positive(self):
        assert SimGeometry.normalize_angle_deg(270.0) == pytest.approx(-90.0)

    def test_wrap_negative(self):
        assert SimGeometry.normalize_angle_deg(-270.0) == pytest.approx(90.0)

    def test_positive_360_maps_to_zero(self):
        assert SimGeometry.normalize_angle_deg(360.0) == pytest.approx(0.0)

    def test_negative_360_maps_to_zero(self):
        assert SimGeometry.normalize_angle_deg(-360.0) == pytest.approx(0.0)

    def test_cross_plus_180(self):
        """181 deg wraps to -179 deg."""
        assert SimGeometry.normalize_angle_deg(181.0) == pytest.approx(-179.0)

    def test_cross_minus_180(self):
        """-181 deg wraps to 179 deg."""
        assert SimGeometry.normalize_angle_deg(-181.0) == pytest.approx(179.0)

    def test_exactly_180(self):
        """+180 stays +180 (upper boundary)."""
        assert SimGeometry.normalize_angle_deg(180.0) == pytest.approx(180.0)

    def test_exactly_minus_180(self):
        """-180 maps to +180 (convention: [-180, 180) half-open at -180)."""
        assert SimGeometry.normalize_angle_deg(-180.0) == pytest.approx(180.0)


# ---------------------------------------------------------------------------
# FOV / range constants
# ---------------------------------------------------------------------------

class TestConstants:
    """Verify expected default values."""

    def test_fov_120(self):
        assert FOV_DEG == 120.0

    def test_half_fov_60(self):
        assert HALF_FOV_DEG == 60.0

    def test_default_ppm(self):
        assert DEFAULT_PIXELS_PER_METER == 120

    def test_default_max_range_positive(self):
        assert DEFAULT_MAX_RANGE_M > 0
