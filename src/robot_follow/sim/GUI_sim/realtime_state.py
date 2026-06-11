"""Simulated robot state for the GUI simulation.

:class:`SimRobotState` maintains the robot's 2-D pose and applies
:class:`~robot_follow.core.types.MotionCommand` updates using simple Euler
integration.

Integration order (heading-first):

1. ``heading += omega * dt``
2. ``x += cos(heading) * vx * dt``
3. ``y += sin(heading) * vx * dt``

``vy`` is ignored in this version because the current core controller
always outputs ``vy = 0.0``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_follow.core.types import MotionCommand


@dataclass
class SimRobotState:
    """Mutable 2-D pose updated by ``MotionCommand`` steps.

    Parameters
    ----------
    x_m, y_m
        Position in world metres.
    heading_rad
        Heading in radians (counter-clockwise from +x).
    """

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0

    def __post_init__(self) -> None:
        # Store initial values for reset().
        self._init_x = self.x_m
        self._init_y = self.y_m
        self._init_heading = self.heading_rad

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def apply(self, command: MotionCommand, dt: float = 0.025) -> None:
        """Integrate one time-step from *command*.

        For ``kind="stop"`` the pose is unchanged.  For ``kind="move"`` the
        heading is updated first, then position.
        """
        if command.kind == "stop":
            return

        # Heading update first (Euler integration).
        self.heading_rad += command.omega * dt

        # Position update using *new* heading.
        self.x_m += math.cos(self.heading_rad) * command.vx * dt
        self.y_m += math.sin(self.heading_rad) * command.vx * dt

    def reset(self) -> None:
        """Reset pose to the values supplied at construction."""
        self.x_m = self._init_x
        self.y_m = self._init_y
        self.heading_rad = self._init_heading
