"""Tkinter GUI simulation testbed for ``robot_follow.core``.

The mouse represents the person being followed.  Every 25 ms (40 Hz) the GUI
samples the mouse position, builds a :class:`~robot_follow.core.TargetSample`,
calls :meth:`~robot_follow.core.FollowPipeline.process`, and integrates the
returned :class:`~robot_follow.core.MotionCommand` into a simulated robot pose.

The GUI NEVER decides motion itself – every motion command comes from the core
pipeline.  The GUI only does coordinate conversion, FOV/range simulation,
drawing, logging and pose integration.

Run with mouse input::

    python -m robot_follow.sim.GUI_sim.gui_app

Run with live ALX-G100 input::

    python -m robot_follow.sim.GUI_sim.gui_app --source alx --port /dev/ttyUSB0

Keyboard
--------
Space       pause / resume
R           reset robot pose and pipeline filter
L           toggle verbose per-frame logging
Q / Escape  quit
"""

from __future__ import annotations

import argparse
import math
import time
import tkinter as tk
from dataclasses import replace
from typing import Optional, Tuple

from robot_follow.core import FollowPipeline, MotionCommand, TargetSample
from robot_follow.real.alx_source import ALXLiveSource, ALXSnapshot
from robot_follow.sim.GUI_sim.geometry import HALF_FOV_DEG, SimGeometry
from robot_follow.sim.GUI_sim.logger import FrameState, SimLogger
from robot_follow.sim.GUI_sim.realtime_state import SimRobotState

# Fixed timing – core pipeline input is always 40 Hz.
INPUT_HZ: float = 40.0
FIXED_DT_S: float = 0.025
TICK_MS: int = 25
TICK_S: float = TICK_MS / 1000.0


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
        source: str = "mouse",
        motion_mode: str = "full",
        pixels_per_meter: int = 120,
        port: str = "/dev/ttyUSB0",
        stale_timeout_s: float = 0.5,
        pipeline: Optional[FollowPipeline] = None,
        geometry: Optional[SimGeometry] = None,
        logger: Optional[SimLogger] = None,
        alx_source: Optional[ALXLiveSource] = None,
    ) -> None:
        self.width = width
        self.height = height
        if source not in {"mouse", "alx"}:
            raise ValueError(f"unsupported source: {source}")
        if motion_mode not in {"full", "rotate-only"}:
            raise ValueError(f"unsupported motion mode: {motion_mode}")
        self.source = source
        self.motion_mode = motion_mode
        self.port = port
        self.stale_timeout_s = stale_timeout_s

        self.geometry = geometry or SimGeometry(
            canvas_w=width,
            canvas_h=height,
            pixels_per_meter=pixels_per_meter,
        )
        self.pipeline = pipeline or FollowPipeline()
        self.robot = SimRobotState()
        self.logger = logger or SimLogger(threshold_s=math.inf)
        self.alx_source = alx_source
        if self.source == "alx" and self.alx_source is None:
            self.alx_source = ALXLiveSource(port=port)

        # Simulation state
        self.mouse_px: Tuple[int, int] = (width // 2 + 200, height // 2)
        self.mouse_inside: bool = False
        self.paused: bool = False
        self._sim_time: float = 0.0
        self._last_command: Optional[MotionCommand] = None
        self._last_target_world: Tuple[float, float] = (2.0, 0.0)
        self._last_alx: Optional[ALXSnapshot] = None
        self._last_alx_status: str = "not_started"
        self._alx_receiver_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._perf_process_ms: float = 0.0
        self._perf_redraw_ms: float = 0.0
        self._perf_tick_ms: float = 0.0
        self._perf_actual_fps: float = 0.0
        self._perf_canvas_items: int = 0
        self._perf_schedule_lag_ms: float = 0.0
        self._last_tick_started_at: Optional[float] = None
        self._next_tick_due_at: Optional[float] = None

        # Tk widgets (created in build_ui)
        self.root: Optional[tk.Tk] = None
        self.canvas: Optional[tk.Canvas] = None
        self._canvas_items: dict[str, int] = {}

    # ==================================================================
    # Pure, GUI-independent tick logic (unit-testable in principle)
    # ==================================================================

    def process_tick(
        self,
        mouse_px: Optional[Tuple[int, int]] = None,
    ) -> Tuple[TargetSample, MotionCommand]:
        """Run exactly one 40 Hz tick and update robot + log.

        This contains no GUI calls and is the heart of the sim.

        Parameters
        ----------
        mouse_px
            (x_px, y_px) mouse position. Ignored in ALX input mode.

        Returns
        -------
        (sample, command)
            The :class:`TargetSample` fed to the pipeline and the
            :class:`MotionCommand` returned by it.
        """
        if self.source == "alx":
            sample, tx_m, ty_m, in_fov = self._sample_from_alx()
        else:
            sample, tx_m, ty_m, in_fov = self._sample_from_mouse(
                mouse_px if mouse_px is not None else self.mouse_px
            )
        self._last_target_world = (tx_m, ty_m)

        step = self.pipeline.process(sample)
        command = step.command

        # Integrate motion with the FIXED step – never invent motion here.
        applied_command = self._command_for_motion_mode(command)
        self.robot.apply(applied_command, dt=FIXED_DT_S)
        self._last_command = command
        self._sim_time += FIXED_DT_S

        # Logging (rate-limited inside the logger)
        frame = FrameState(
            t=self._sim_time,
            dt=FIXED_DT_S,
            target_x=tx_m,
            target_y=ty_m,
            distance_m=sample.distance_m,
            azimuth_deg=sample.azimuth_deg,
            valid=sample.valid,
            in_fov=in_fov,
            command_kind=command.kind,
            command_reason=command.reason,
            vx=applied_command.vx,
            omega=applied_command.omega,
            robot_x=self.robot.x_m,
            robot_y=self.robot.y_m,
            robot_heading_deg=math.degrees(self.robot.heading_rad),
        )
        self.logger.log_frame(frame)

        return sample, command

    def _command_for_motion_mode(self, command: MotionCommand) -> MotionCommand:
        if self.motion_mode == "full" or command.kind == "stop":
            return command
        return replace(command, vx=0.0, vy=0.0)

    def _sample_from_mouse(
        self,
        mouse_px: Tuple[int, int],
    ) -> Tuple[TargetSample, float, float, bool]:
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
        return sample, tx_m, ty_m, det.in_fov

    def _sample_from_alx(self) -> Tuple[TargetSample, float, float, bool]:
        now = time.monotonic()
        source = self.alx_source
        snapshot = source.snapshot() if source is not None else None
        self._last_alx = snapshot

        if snapshot is None:
            self._last_alx_status = "waiting_for_position"
            sample = TargetSample(
                timestamp=now,
                distance_m=0.0,
                azimuth_deg=0.0,
                valid=False,
            )
            return sample, self.robot.x_m, self.robot.y_m, False

        if snapshot.error is not None:
            self._last_alx_status = f"error: {snapshot.error}"
            tx_m, ty_m = self._target_world_from_alx(snapshot)
            det = self.geometry.detect(
                self.robot.x_m,
                self.robot.y_m,
                self.robot.heading_rad,
                tx_m,
                ty_m,
            )
            sample = TargetSample(
                timestamp=now,
                distance_m=det.distance_m,
                azimuth_deg=det.azimuth_deg,
                valid=False,
            )
            return sample, tx_m, ty_m, False

        age_s = snapshot.age_s(now)
        tx_m, ty_m = self._target_world_from_alx(snapshot)
        det = self.geometry.detect(
            self.robot.x_m,
            self.robot.y_m,
            self.robot.heading_rad,
            tx_m,
            ty_m,
        )
        if age_s > self.stale_timeout_s:
            self._last_alx_status = f"stale {age_s * 1000:.0f}ms"
            sample = TargetSample(
                timestamp=snapshot.timestamp,
                distance_m=det.distance_m,
                azimuth_deg=det.azimuth_deg,
                valid=True,
            )
            return sample, tx_m, ty_m, det.in_fov

        self._last_alx_status = "live"
        sample = TargetSample(
            timestamp=snapshot.timestamp,
            distance_m=det.distance_m,
            azimuth_deg=det.azimuth_deg,
            valid=det.valid,
        )
        return sample, tx_m, ty_m, det.in_fov

    def _target_world_from_alx(self, snapshot: ALXSnapshot) -> Tuple[float, float]:
        rx, ry, heading = self._alx_receiver_world
        return self.geometry.polar_to_world(
            rx,
            ry,
            heading,
            snapshot.distance_m,
            snapshot.azimuth_deg,
        )

    # ==================================================================
    # GUI construction
    # ==================================================================

    def build_ui(self) -> None:
        root = tk.Tk()
        root.title(
            f"robot_follow GUI sim ({self.source} input, core @ 40Hz)"
        )
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

    def _fov_wedge_points(self, sx: int, sy: int) -> list[int]:
        """Return polygon points for the 120 deg detection wedge."""
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
        return pts

    def _heading_arrow_coords(self, sx: int, sy: int) -> Tuple[int, int, int, int]:
        """Return the heading arrow coordinates."""
        length = 28
        heading = self.robot.heading_rad
        ex = sx + length * math.cos(heading)
        ey = sy - length * math.sin(heading)
        return sx, sy, int(round(ex)), int(round(ey))

    def _create_canvas_items(
        self,
        rsx: int,
        rsy: int,
        tsx: int,
        tsy: int,
        sample: TargetSample,
        status: str,
        target_visible: bool,
    ) -> None:
        if self.canvas is None:
            return
        target_state = tk.NORMAL if target_visible else tk.HIDDEN
        link_color = COL_LINK_VALID if sample.valid else COL_LINK_INVALID
        self._canvas_items = {
            "fov": self.canvas.create_polygon(
                *self._fov_wedge_points(rsx, rsy),
                fill=COL_FOV,
                outline=COL_FOV_EDGE,
                width=1,
            ),
            "robot": self.canvas.create_oval(
                rsx - 7, rsy - 7, rsx + 7, rsy + 7,
                fill=COL_ROBOT,
                outline=COL_HEADING,
                width=2,
            ),
            "arrow": self.canvas.create_line(
                *self._heading_arrow_coords(rsx, rsy),
                fill=COL_ARROW,
                width=3,
                arrow=tk.LAST,
            ),
            "link": self.canvas.create_line(
                rsx, rsy, tsx, tsy, fill=link_color, width=2
            ),
            "target": self.canvas.create_oval(
                tsx - 6, tsy - 6, tsx + 6, tsy + 6,
                fill=COL_TARGET,
                outline=COL_TEXT,
                state=target_state,
            ),
            "status": self.canvas.create_text(
                12, 12, anchor="nw", fill=COL_TEXT, font=("Consolas", 11),
                text=status,
            ),
            "help": self.canvas.create_text(
                12, self.height - 12, anchor="sw", fill=COL_TEXT,
                font=("Consolas", 10),
                text="Space pause   R reset   L verbose-log   Q/Esc quit",
            ),
        }

    def redraw(self, sample: TargetSample, command: MotionCommand) -> None:
        if self.canvas is None:
            return

        rsx, rsy = self.geometry.world_to_px(
            self.robot.x_m, self.robot.y_m
        )
        tx_m, ty_m = self._last_target_world
        tsx, tsy = self.geometry.world_to_px(tx_m, ty_m)
        status = self._format_status(sample, command)
        target_visible = self.source == "alx" or self.mouse_inside

        if not self._canvas_items:
            self._create_canvas_items(
                rsx, rsy, tsx, tsy, sample, status, target_visible
            )
            self._update_canvas_item_count()
            return

        target_state = tk.NORMAL if target_visible else tk.HIDDEN
        link_color = COL_LINK_VALID if sample.valid else COL_LINK_INVALID
        self.canvas.coords(self._canvas_items["fov"], *self._fov_wedge_points(rsx, rsy))
        self.canvas.coords(
            self._canvas_items["robot"],
            rsx - 7,
            rsy - 7,
            rsx + 7,
            rsy + 7,
        )
        self.canvas.coords(
            self._canvas_items["arrow"],
            *self._heading_arrow_coords(rsx, rsy),
        )
        self.canvas.coords(self._canvas_items["link"], rsx, rsy, tsx, tsy)
        self.canvas.itemconfig(self._canvas_items["link"], fill=link_color)
        self.canvas.coords(
            self._canvas_items["target"],
            tsx - 6,
            tsy - 6,
            tsx + 6,
            tsy + 6,
        )
        self.canvas.itemconfig(self._canvas_items["target"], state=target_state)
        self.canvas.itemconfig(self._canvas_items["status"], text=status)
        self._update_canvas_item_count()

    def _update_canvas_item_count(self) -> None:
        if self.canvas is None or not hasattr(self.canvas, "find_all"):
            self._perf_canvas_items = len(self._canvas_items)
            return
        try:
            self._perf_canvas_items = len(self.canvas.find_all())
        except tk.TclError:
            self._perf_canvas_items = len(self._canvas_items)

    def _format_status(self, sample: TargetSample, command: MotionCommand) -> str:
        lines = [
            f"state: {'PAUSED' if self.paused else 'RUN'}   "
            f"source={self.source}   motion={self.motion_mode}   "
            f"scale={self.geometry.pixels_per_meter}px/m   "
            f"core @ {int(INPUT_HZ)}Hz   dt={FIXED_DT_S:.3f}s",
            f"distance = {sample.distance_m:6.2f} m   "
            f"azimuth = {sample.azimuth_deg:6.1f} deg",
            f"valid = {sample.valid}   command = {command.kind}   "
            f"reason = {command.reason}",
            f"vx = {command.vx:+.3f}   omega = {command.omega:+.4f}",
            f"perf fps = {self._perf_actual_fps:5.1f}   "
            f"tick = {self._perf_tick_ms:5.1f} ms   "
            f"process = {self._perf_process_ms:5.1f} ms   "
            f"redraw = {self._perf_redraw_ms:5.1f} ms   "
            f"lag = {self._perf_schedule_lag_ms:5.1f} ms   "
            f"items = {self._perf_canvas_items}",
            f"robot = ({self.robot.x_m:+.2f}, {self.robot.y_m:+.2f}) m   "
            f"heading = {math.degrees(self.robot.heading_rad):+6.1f} deg",
        ]
        if self.source == "alx":
            lines.append(
                "map=receiver-fixed   receiver=(0,0,+x)   robot_pos=(0,0)"
            )
            snap = self._last_alx
            if snap is None:
                lines.append(f"alx = {self._last_alx_status}   port={self.port}")
            else:
                age_ms = snap.age_s() * 1000.0 if snap.timestamp > 0 else 0.0
                lines.append(
                    f"alx = {self._last_alx_status}   port={self.port}   "
                    f"fps={snap.fps:4.1f}   age={age_ms:5.0f} ms"
                )
                lines.append(
                    f"raw distance = {snap.distance_m:6.2f} m   "
                    f"raw azimuth = {snap.azimuth_deg:6.1f} deg"
                )
                lines.append(
                    f"elev = {snap.elevation_deg:+6.1f} deg   "
                    f"tag_status = 0x{snap.tag_status:04X}"
                )
                lines.append(
                    f"anchor = 0x{snap.anchor_id:08X}   "
                    f"tag = 0x{snap.tag_id:08X}"
                )
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
        self._last_target_world = (2.0, 0.0)
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
        tick_started = time.perf_counter()
        if self._next_tick_due_at is None:
            self._next_tick_due_at = tick_started
        self._perf_schedule_lag_ms = max(
            0.0,
            (tick_started - self._next_tick_due_at) * 1000.0,
        )
        if self._last_tick_started_at is not None:
            interval_s = tick_started - self._last_tick_started_at
            if interval_s > 0.0:
                self._perf_actual_fps = 1.0 / interval_s
        self._last_tick_started_at = tick_started

        if not self.paused:
            process_started = time.perf_counter()
            sample, command = self.process_tick(self.mouse_px)
            process_finished = time.perf_counter()
            self.redraw(sample, command)
            redraw_finished = time.perf_counter()
            self._perf_process_ms = (process_finished - process_started) * 1000.0
            self._perf_redraw_ms = (redraw_finished - process_finished) * 1000.0
        self._perf_tick_ms = (time.perf_counter() - tick_started) * 1000.0
        if self.root is not None:
            self._schedule_next_tick()

    def _schedule_next_tick(self) -> None:
        if self.root is None:
            return
        now = time.perf_counter()
        if self._next_tick_due_at is None:
            self._next_tick_due_at = now + TICK_S
        else:
            self._next_tick_due_at += TICK_S
        if self._next_tick_due_at < now:
            missed_ticks = math.floor((now - self._next_tick_due_at) / TICK_S) + 1
            self._next_tick_due_at += missed_ticks * TICK_S
        delay_ms = max(0, int(round((self._next_tick_due_at - now) * 1000.0)))
        self.root.after(delay_ms, self._tick)

    def run(self) -> None:
        self.build_ui()
        if self.source == "alx" and self.alx_source is not None:
            self.alx_source.start()
            print(f"EVENT alx_reader_started port={self.port}")
        # Kick off the 40 Hz loop.
        self._next_tick_due_at = time.perf_counter() + TICK_S
        self.root.after(TICK_MS, self._tick)
        try:
            self.root.mainloop()
        finally:
            if self.alx_source is not None:
                self.alx_source.stop()
            print("GUI sim stopped.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the robot_follow GUI simulation."
    )
    parser.add_argument(
        "--source",
        choices=("mouse", "alx"),
        default="mouse",
        help="Input source: mouse simulation or live ALX-G100 serial data.",
    )
    parser.add_argument(
        "--motion-mode",
        choices=("full", "rotate-only"),
        default=None,
        help=(
            "Motion integration mode: full vx/omega or rotate-only omega. "
            "Defaults to rotate-only for --source alx, full for mouse."
        ),
    )
    parser.add_argument(
        "--pixels-per-meter",
        type=int,
        default=120,
        help="GUI map scale in pixels per metre.",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="ALX-G100 serial port used when --source alx.",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.5,
        help="Seconds before the latest ALX position is treated as stale.",
    )
    return parser.parse_args()


def _resolve_motion_mode(source: str, requested: Optional[str]) -> str:
    """Resolve CLI motion mode defaults by input source."""
    if requested is not None:
        return requested
    return "rotate-only" if source == "alx" else "full"


def main() -> None:
    args = _parse_args()
    motion_mode = _resolve_motion_mode(args.source, args.motion_mode)
    app = GUIFollowApp(
        source=args.source,
        motion_mode=motion_mode,
        pixels_per_meter=args.pixels_per_meter,
        port=args.port,
        stale_timeout_s=args.stale_timeout,
    )
    app.run()


if __name__ == "__main__":
    main()
