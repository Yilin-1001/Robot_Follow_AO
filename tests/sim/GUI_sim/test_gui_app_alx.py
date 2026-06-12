"""Tests for GUI sim live ALX input plumbing."""

from __future__ import annotations

import time

from robot_follow.real.alx_source import ALXSnapshot
from robot_follow.core.types import MotionCommand, TargetSample
from robot_follow.sim.GUI_sim.gui_app import GUIFollowApp, _resolve_motion_mode


class FakeALXSource:
    def __init__(self, snapshot):
        self._snapshot = snapshot
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def snapshot(self):
        return self._snapshot


class FakeCanvas:
    def __init__(self):
        self.next_id = 1
        self.created = []
        self.coords_calls = []
        self.itemconfig_calls = []
        self.delete_calls = []

    def _create(self, kind, args, kwargs):
        item_id = self.next_id
        self.next_id += 1
        self.created.append((kind, item_id, args, kwargs))
        return item_id

    def create_polygon(self, *args, **kwargs):
        return self._create("polygon", args, kwargs)

    def create_oval(self, *args, **kwargs):
        return self._create("oval", args, kwargs)

    def create_line(self, *args, **kwargs):
        return self._create("line", args, kwargs)

    def create_text(self, *args, **kwargs):
        return self._create("text", args, kwargs)

    def coords(self, *args):
        self.coords_calls.append(args)

    def itemconfig(self, *args, **kwargs):
        self.itemconfig_calls.append((args, kwargs))

    def delete(self, *args):
        self.delete_calls.append(args)

    def find_all(self):
        return tuple(item_id for _, item_id, _, _ in self.created)


class FakeRoot:
    def __init__(self):
        self.after_calls = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))


def _snapshot(*, timestamp=None, distance_m=2.0, azimuth_deg=0.0):
    now = time.monotonic()
    return ALXSnapshot(
        timestamp=now if timestamp is None else timestamp,
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        elevation_deg=-3.0,
        tag_status=0x1234,
        anchor_id=0x10AA,
        tag_id=0x10AA,
        fps=40.0,
        received_at=now,
        heartbeat_at=None,
    )


def test_alx_snapshot_feeds_pipeline_and_updates_target_world():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=0.0))
    app = GUIFollowApp(source="alx", alx_source=source)

    sample, command = app.process_tick()

    assert sample.valid is True
    assert sample.distance_m == 2.0
    assert sample.azimuth_deg == 0.0
    assert command.kind == "move"
    assert app._last_target_world[0] > app.robot.x_m


def test_pixels_per_meter_configures_gui_geometry():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=0.0))
    app = GUIFollowApp(
        source="alx",
        pixels_per_meter=60,
        alx_source=source,
    )

    assert app.geometry.pixels_per_meter == 60


def test_alx_status_identifies_fixed_receiver_map():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=0.0))
    app = GUIFollowApp(source="alx", alx_source=source)
    sample = TargetSample(timestamp=time.monotonic(), distance_m=2.0, azimuth_deg=0.0)
    command = MotionCommand(
        kind="move",
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason="holding",
    )

    app.process_tick()
    status = app._format_status(sample, command)

    assert "map=receiver-fixed" in status
    assert "receiver=(0,0,+x)" in status
    assert "robot_pos=(0,0)" in status


def test_status_includes_gui_performance_metrics():
    app = GUIFollowApp()
    app._perf_actual_fps = 38.5
    app._perf_tick_ms = 1.2
    app._perf_process_ms = 0.3
    app._perf_redraw_ms = 0.9
    sample = TargetSample(timestamp=time.monotonic(), distance_m=2.0, azimuth_deg=0.0)
    command = MotionCommand(
        kind="move",
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason="holding",
    )

    status = app._format_status(sample, command)

    assert "perf fps" in status
    assert "tick" in status
    assert "process" in status
    assert "redraw" in status
    assert "lag" in status
    assert "items" in status


def test_redraw_reuses_canvas_items_after_first_draw():
    app = GUIFollowApp()
    canvas = FakeCanvas()
    app.canvas = canvas
    app.mouse_inside = True
    sample = TargetSample(timestamp=time.monotonic(), distance_m=2.0, azimuth_deg=0.0)
    command = MotionCommand(
        kind="move",
        vx=0.0,
        vy=0.0,
        omega=0.0,
        reason="holding",
    )

    app.redraw(sample, command)
    created_after_first = len(canvas.created)
    app.redraw(sample, command)

    assert created_after_first == 7
    assert len(canvas.created) == created_after_first
    assert app._perf_canvas_items == created_after_first
    assert canvas.delete_calls == []
    assert len(canvas.coords_calls) >= 5
    assert any(
        kwargs.get("text") is not None
        for _, kwargs in canvas.itemconfig_calls
    )


def test_gui_default_logger_suppresses_frame_logs():
    lines = []
    app = GUIFollowApp(logger=None)
    app.logger._emit = lines.append

    app.process_tick((500, 300))

    assert not any(line.startswith("FRAME") for line in lines)


def test_tick_uses_fixed_rate_schedule_instead_of_full_tick_delay():
    app = GUIFollowApp()
    app.root = FakeRoot()
    app.paused = True

    app._tick()

    assert app.root.after_calls
    delay_ms, callback = app.root.after_calls[-1]
    assert callback == app._tick
    assert 0 <= delay_ms <= 25


def test_alx_cli_default_motion_mode_is_rotate_only():
    assert _resolve_motion_mode(source="alx", requested=None) == "rotate-only"


def test_mouse_cli_default_motion_mode_is_full():
    assert _resolve_motion_mode(source="mouse", requested=None) == "full"


def test_explicit_motion_mode_is_preserved():
    assert _resolve_motion_mode(source="alx", requested="full") == "full"


def test_constant_alx_snapshot_does_not_move_displayed_target_with_robot():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=0.0))
    app = GUIFollowApp(source="alx", alx_source=source)

    app.process_tick()
    first_target = app._last_target_world
    assert app.robot.x_m > 0.0

    app.process_tick()
    second_target = app._last_target_world

    assert second_target == first_target


def test_alx_sample_is_recomputed_from_robot_pose_to_fixed_target():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=0.0))
    app = GUIFollowApp(source="alx", alx_source=source)

    first_sample, _ = app.process_tick()
    second_sample, _ = app.process_tick()

    assert second_sample.distance_m < first_sample.distance_m


def test_rotate_only_ignores_forward_velocity_but_applies_omega():
    source = FakeALXSource(_snapshot(distance_m=2.0, azimuth_deg=10.0))
    app = GUIFollowApp(
        source="alx",
        motion_mode="rotate-only",
        alx_source=source,
    )

    _, command = app.process_tick()

    assert command.vx > 0.0
    assert command.omega != 0.0
    assert app.robot.x_m == 0.0
    assert app.robot.y_m == 0.0
    assert app.robot.heading_rad != 0.0


def test_missing_alx_position_generates_invalid_sample():
    source = FakeALXSource(None)
    app = GUIFollowApp(source="alx", alx_source=source)

    sample, command = app.process_tick()

    assert sample.valid is False
    assert command.kind == "stop"
    assert command.reason == "data_invalid"


def test_stale_alx_position_uses_old_timestamp_for_timeout():
    old_ts = time.monotonic() - 1.0
    source = FakeALXSource(_snapshot(timestamp=old_ts, distance_m=2.0))
    app = GUIFollowApp(
        source="alx",
        stale_timeout_s=0.5,
        alx_source=source,
    )

    sample, command = app.process_tick()

    assert sample.valid is True
    assert sample.timestamp == old_ts
    assert command.kind == "stop"
    assert command.reason == "data_timeout"
