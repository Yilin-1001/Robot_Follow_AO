"""Tkinter GUI simulation testbed for ``robot_follow.core``.

The mouse represents the person being followed.  Every 25 ms (40 Hz) the GUI
samples the mouse position, builds a :class:`~robot_follow.core.TargetSample`,
calls :meth:`~robot_follow.core.FollowPipeline.process`, and integrates the
returned :class:`~robot_follow.core.MotionCommand` into a simulated robot pose.

The GUI NEVER decides motion itself – every motion command comes from the core
pipeline.  The GUI only does coordinate conversion, FOV/range simulation,
drawing, logging and pose integration.

Run with::

    python -m robot_follow.sim.GUI_sim.gui_app

Keyboard
--------
Space       pause / resume
R           reset robot pose and pipeline filter
L           toggle verbose per-frame logging
Q / Escape  quit
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from typing import Optional, Tuple

from robot_follow.core import FollowPipeline, MotionCommand, TargetSample
from robot_follow.sim.GUI_sim.geometry import HALF_FOV_DEG, SimGeometry
from robot_follow.sim.GUI_sim.logger import FrameState, SimLogger
from robot_follow.sim.GUI_sim.realtime_state import SimRobotState

# Fixed timing – core pipeline input is always 40 Hz.
INPUT_HZ: float = 40.0
FIXED_DT_S: float = 0.025
TICK_MS: int = 25


# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

COL_BG = "#1e1e2e"
COL_FOV = "#3b4252"
COL_FOV_EDGE = "#4c566a"
COL_ROBOT = "#88c0d0"
COL_ARROW = "#ebcb8b"
COL_LINK_VALID = "#a3be8c"
COL_LINK_INVALID = "#bf616a"
COL_TARGET = "#d08770"
COL_TEXT = "#eceff4"
COL_HEADING = "#b48ead"


# ---------------------------------------------------------------------------
# GUI application
# ---------------------------------------------------------------------------

class GUIFollowApp:
    """Tkinter visualisation harness driving the core follow pipeline."""

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        pipeline: Optional[FollowPipeline] = None,
        geometry: Optional[SimGeometry] = None,
        logger: Optional[SimLogger] = None,
    ) -> None:
        self.width = width
        self.height = height

        self.geometry = geometry or SimGeometry(
            canvas_w=width, canvas_h=height
        )
        self.pipeline = pipeline or FollowPipeline()
        self.robot = SimRobotState()
        self.logger = logger or SimLogger(threshold_s=0.1)

        # Simulation state
        self.mouse_px: Tuple[int, int] = (width // 2 + 200, height // 2)
        self.mouse_inside: bool = False
        self.paused: bool = False
        self._sim_time: float = 0.0
        self._last_command: Optional[MotionCommand] = None

        # Tk widgets (created in build_ui)
        self.root: Optional[tk.Tk] = None
        self.canvas: Optional[tk.Canvas] = None

    # ==================================================================
    # Pure, GUI-independent tick logic (unit-testable in principle)
    # ==================================================================

    def process_tick(self, mouse_px: Tuple[int, int]) -> Tuple[TargetSample, MotionCommand]:
        """Run exactly one 40 Hz tick and update robot + log.

        This contains no GUI calls and is the heart of the sim.

        Parameters
        ----------
        mouse_px
            (x_px, y_px) mouse position.

        Returns
        -------
        (sample, command)
            The :class:`TargetSample` fed to the pipeline and the
            :class:`MotionCommand` returned by it.
        """
        # Robot centre in screen px (world origin maps to canvas centre).
        robot_sx, robot_sy = self.geometry.world_to_px(
            self.robot.x_m, self.robot.y_m
        )
        tx_m, ty_m = self.geometry.px_to_world(*mouse_px)

        det = self.geometry.detect(
            self.robot.x_m,
            self.robot.y_m,
            self.robot.heading_rad,
            tx_m,
            ty_m,
        )

        sample = TargetSample(
            timestamp=time.monotonic(),
            distance_m=det.distance_m,
            azimuth_deg=det.azimuth_deg,
            valid=det.valid,
        )

        step = self.pipeline.process(sample)
        command = step.command

        # Integrate motion with the FIXED step – never invent motion here.
        self.robot.apply(command, dt=FIXED_DT_S)
        self._last_command = command
        self._sim_time += FIXED_DT_S

        # Logging (rate-limited inside the logger)
        frame = FrameState(
            t=self._sim_time,
            dt=FIXED_DT_S,
            target_x=tx_m,
            target_y=ty_m,
            distance_m=det.distance_m,
            azimuth_deg=det.azimuth_deg,
            valid=det.valid,
            in_fov=det.in_fov,
            command_kind=command.kind,
            command_reason=command.reason,
            vx=command.vx,
            omega=command.omega,
            robot_x=self.robot.x_m,
            robot_y=self.robot.y_m,
            robot_heading_deg=math.degrees(self.robot.heading_rad),
        )
        self.logger.log_frame(frame)

        return sample, command

    # ==================================================================
    # GUI construction
    # ==================================================================

    def build_ui(self) -> None:
        root = tk.Tk()
        root.title("robot_follow GUI sim (core pipeline @ 40Hz)")
        self.root = root

        canvas = tk.Canvas(root, width=self.width, height=self.height, bg=COL_BG,
                           highlightthickness=0)
        canvas.pack()
        self.canvas = canvas

        # Events
        canvas.bind("<Motion>", self._on_motion)
        canvas.bind("<Leave>", self._on_leave)
        root.bind("<space>", self._on_toggle_pause)
        root.bind("r", self._on_reset)
        root.bind("R", self._on_reset)
        root.bind("l", self._on_toggle_verbose)
        root.bind("L", self._on_toggle_verbose)
        root.bind("q", self._on_quit)
        root.bind("Q", self._on_quit)
        root.bind("<Escape>", self._on_quit)

    # ==================================================================
    # Drawing
    # ==================================================================

    def _draw_fov_wedge(self, sx: int, sy: int) -> None:
        """Draw the 120 deg detection wedge in front of the robot."""
        radius = int(self.geometry.max_range_m * self.geometry.pixels_per_meter)
        heading = self.robot.heading_rad

        pts: list[int] = [sx, sy]
        steps = 24
        for i in range(steps + 1):
            frac = i / steps
            angle = heading + math.radians(HALF_FOV_DEG) \
                - frac * math.radians(2 * HALF_FOV_DEG)
            ex = sx + radius * math.cos(angle)
            ey = sy - radius * math.sin(angle)
            pts.extend([int(round(ex)), int(round(ey))])
        if self.canvas is not None:
            self.canvas.create_polygon(
                *pts, fill=COL_FOV, outline=COL_FOV_EDGE, width=1
            )

    def _draw_arrow(self, sx: int, sy: int) -> None:
        """Draw the heading arrow."""
        length = 28
        heading = self.robot.heading_rad
        ex = sx + length * math.cos(heading)
        ey = sy - length * math.sin(heading)
        if self.canvas is not None:
            self.canvas.create_line(
                sx, sy, int(round(ex)), int(round(ey)),
                fill=COL_ARROW, width=3, arrow=tk.LAST,
            )

    def redraw(self, sample: TargetSample, command: MotionCommand) -> None:
        if self.canvas is None:
            return
        self.canvas.delete("all")

        rsx, rsy = self.geometry.world_to_px(
            self.robot.x_m, self.robot.y_m
        )

        # FOV wedge first (background)
        self._draw_fov_wedge(rsx, rsy)

        # Robot point
        self.canvas.create_oval(
            rsx - 7, rsy - 7, rsx + 7, rsy + 7,
            fill=COL_ROBOT, outline=COL_HEADING, width=2,
        )
        # Heading arrow
        self._draw_arrow(rsx, rsy)

        # Target + link line
        tx_m, ty_m = self.geometry.px_to_world(*self.mouse_px)
        tsx, tsy = self.geometry.world_to_px(tx_m, ty_m)
        link_color = COL_LINK_VALID if sample.valid else COL_LINK_INVALID
        self.canvas.create_line(rsx, rsy, tsx, tsy, fill=link_color, width=2)
        if self.mouse_inside:
            self.canvas.create_oval(
                tsx - 6, tsy - 6, tsx + 6, tsy + 6,
                fill=COL_TARGET, outline=COL_TEXT,
            )

        # Status text
        status = self._format_status(sample, command)
        self.canvas.create_text(
            12, 12, anchor="nw", fill=COL_TEXT, font=("Consolas", 11),
            text=status,
        )
        # Help text
        self.canvas.create_text(
            12, self.height - 12, anchor="sw", fill=COL_TEXT,
            font=("Consolas", 10),
            text="Space pause   R reset   L verbose-log   Q/Esc quit",
        )

    def _format_status(self, sample: TargetSample, command: MotionCommand) -> str:
        lines = [
            f"state: {'PAUSED' if self.paused else 'RUN'}   "
            f"core @ {int(INPUT_HZ)}Hz   dt={FIXED_DT_S:.3f}s",
            f"distance = {sample.distance_m:6.2f} m   "
            f"azimuth = {sample.azimuth_deg:6.1f} deg",
            f"valid = {sample.valid}   command = {command.kind}   "
            f"reason = {command.reason}",
            f"vx = {command.vx:+.3f}   omega = {command.omega:+.4f}",
            f"robot = ({self.robot.x_m:+.2f}, {self.robot.y_m:+.2f}) m   "
            f"heading = {math.degrees(self.robot.heading_rad):+6.1f} deg",
        ]
        return "\n".join(lines)

    # ==================================================================
    # Event handlers
    # ==================================================================

    def _on_motion(self, event: tk.Event) -> None:
        self.mouse_px = (event.x, event.y)
        self.mouse_inside = True

    def _on_leave(self, _event: tk.Event) -> None:
        self.mouse_inside = False

    def _on_toggle_pause(self, _event: tk.Event) -> None:
        self.paused = not self.paused
        print(f"EVENT sim_{'paused' if self.paused else 'resumed'}")

    def _on_reset(self, _event: tk.Event) -> None:
        self.robot.reset()
        self.pipeline.reset()
        self._sim_time = 0.0
        self._last_command = None
        print("EVENT sim_reset robot_pose pipeline_filter")

    def _on_toggle_verbose(self, _event: tk.Event) -> None:
        self.logger.verbose = not self.logger.verbose
        print(f"EVENT frame_log_verbose={self.logger.verbose}")

    def _on_quit(self, _event: tk.Event) -> None:
        if self.root is not None:
            self.root.destroy()

    # ==================================================================
    # Main loop
    # ==================================================================

    def _tick(self) -> None:
        """One scheduled callback at ~40 Hz."""
        if not self.paused:
            sample, command = self.process_tick(self.mouse_px)
            self.redraw(sample, command)
        if self.root is not None:
            self.root.after(TICK_MS, self._tick)

    def run(self) -> None:
        self.build_ui()
        # Kick off the 40 Hz loop.
        self.root.after(TICK_MS, self._tick)
        try:
            self.root.mainloop()
        finally:
            print("GUI sim stopped.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    app = GUIFollowApp()
    app.run()


if __name__ == "__main__":
    main()
