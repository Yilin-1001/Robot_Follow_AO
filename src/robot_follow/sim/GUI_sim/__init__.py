"""GUI-based simulation testbed for ``robot_follow.core``.

``GUI_sim`` is a visualization harness that feeds a mouse-driven target into the
core :class:`~robot_follow.core.FollowPipeline` at a fixed 40 Hz and drives a
simulated robot point with the returned :class:`~robot_follow.core.MotionCommand`.

It is NOT a robot control interface and never imports real-hardware modules.
"""

from robot_follow.sim.GUI_sim.geometry import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_PIXELS_PER_METER,
    FOV_DEG,
    HALF_FOV_DEG,
    SimGeometry,
)
from robot_follow.sim.GUI_sim.realtime_state import SimRobotState
from robot_follow.sim.GUI_sim.logger import FrameState, SimLogger

__all__ = [
    "DEFAULT_MAX_RANGE_M",
    "DEFAULT_PIXELS_PER_METER",
    "FOV_DEG",
    "HALF_FOV_DEG",
    "SimGeometry",
    "SimRobotState",
    "FrameState",
    "SimLogger",
]
