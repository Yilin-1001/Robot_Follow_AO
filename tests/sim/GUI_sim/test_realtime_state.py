"""Tests for ``robot_follow.sim.GUI_sim.realtime_state``.

Covers ``stop``, ``move``, ``omega`` and ``reset`` behaviour of
:class:`SimRobotState`.
"""

from __future__ import annotations

import math

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.sim.GUI_sim.realtime_state import SimRobotState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _move(vx: float = 0.1, omega: float = 0.0, reason: str = "test") -> MotionCommand:
    return MotionCommand(kind="move", vx=vx, vy=0.0, omega=omega, reason=reason)


def _stop(reason: str = "test") -> MotionCommand:
    return MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSimRobotState:

    def test_initial_pose(self):
        """Default pose is at origin facing +x."""
        s = SimRobotState()
        assert s.x_m == 0.0
        assert s.y_m == 0.0
        assert s.heading_rad == 0.0

    def test_stop_does_not_move(self):
        """A stop command leaves the pose unchanged."""
        s = SimRobotState()
        s.apply(_stop(), dt=0.025)
        assert s.x_m == 0.0
        assert s.y_m == 0.0
        assert s.heading_rad == 0.0

    def test_move_forward(self):
        """A move command with heading=0 moves in +x."""
        s = SimRobotState()
        s.apply(_move(vx=0.2), dt=0.025)
        assert s.x_m == pytest.approx(0.2 * 0.025)
        assert s.y_m == pytest.approx(0.0)

    def test_move_along_heading(self):
        """Move at heading pi/2 (facing +y) moves in +y."""
        s = SimRobotState(x_m=0, y_m=0, heading_rad=math.pi / 2)
        s.apply(_move(vx=0.2), dt=0.025)
        expected = 0.2 * 0.025
        assert s.x_m == pytest.approx(0.0, abs=1e-12)
        assert s.y_m == pytest.approx(expected, abs=1e-9)

    def test_omega_changes_heading(self):
        """Positive omega rotates heading counter-clockwise."""
        s = SimRobotState()
        s.apply(_move(vx=0.0, omega=0.3), dt=0.025)
        assert s.heading_rad == pytest.approx(0.3 * 0.025)

    def test_combined_move_and_turn(self):
        """Simultaneous vx and omega update both position and heading."""
        s = SimRobotState()
        vx, omega, dt = 0.2, 0.3, 0.025
        s.apply(_move(vx=vx, omega=omega), dt=dt)
        expected_heading = omega * dt
        expected_x = math.cos(expected_heading) * vx * dt
        expected_y = math.sin(expected_heading) * vx * dt
        assert s.heading_rad == pytest.approx(expected_heading)
        assert s.x_m == pytest.approx(expected_x)
        assert s.y_m == pytest.approx(expected_y)

    def test_reset_restores_initial(self):
        """reset() returns to the pose supplied at construction."""
        s = SimRobotState(x_m=1.0, y_m=2.0, heading_rad=math.pi / 4)
        s.apply(_move(vx=0.5, omega=0.1), dt=0.025)
        # Pose has changed
        assert s.x_m != 1.0 or s.y_m != 2.0 or s.heading_rad != math.pi / 4
        s.reset()
        assert s.x_m == 1.0
        assert s.y_m == 2.0
        assert s.heading_rad == pytest.approx(math.pi / 4)

    def test_fixed_dt_reproducible(self):
        """Applying the same command with dt=0.025 twice gives the same result."""
        s1 = SimRobotState()
        s2 = SimRobotState()
        cmd = _move(vx=0.15, omega=0.05)
        s1.apply(cmd, dt=0.025)
        s2.apply(cmd, dt=0.025)
        assert s1.x_m == pytest.approx(s2.x_m)
        assert s1.y_m == pytest.approx(s2.y_m)
        assert s1.heading_rad == pytest.approx(s2.heading_rad)

    def test_multiple_steps_accumulate(self):
        """Repeated applies accumulate position correctly."""
        s = SimRobotState()
        dt = 0.025
        vx = 0.2
        for _ in range(40):  # 1 second of simulation
            s.apply(_move(vx=vx), dt=dt)
        assert s.x_m == pytest.approx(vx * 1.0, abs=0.01)
        assert s.y_m == pytest.approx(0.0, abs=0.01)
