"""Tests for ``robot_follow.sim.GUI_sim.logger``.

Covers frame log formatting, event detection on state change, rate-limiting,
and suppression of repeated events.
"""

from __future__ import annotations

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.sim.GUI_sim.logger import FrameState, SimLogger


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_state(
    *,
    t: float = 0.0,
    dt: float = 0.025,
    target_x: float = 2.0,
    target_y: float = 0.0,
    distance_m: float = 2.0,
    azimuth_deg: float = 0.0,
    valid: bool = True,
    in_fov: bool = True,
    kind: str = "move",
    reason: str = "tracking_forward",
    vx: float = 0.1,
    omega: float = 0.0,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_heading_deg: float = 0.0,
) -> FrameState:
    return FrameState(
        t=t,
        dt=dt,
        target_x=target_x,
        target_y=target_y,
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        valid=valid,
        in_fov=in_fov,
        command_kind=kind,
        command_reason=reason,
        vx=vx,
        omega=omega,
        robot_x=robot_x,
        robot_y=robot_y,
        robot_heading_deg=robot_heading_deg,
    )


def _collecting_logger(threshold: float = 0.0) -> tuple[SimLogger, list[str]]:
    """Create a SimLogger that collects output into a list."""
    lines: list[str] = []
    logger = SimLogger(threshold_s=threshold, emit=lines.append)
    return logger, lines


# ---------------------------------------------------------------------------
# Frame log content
# ---------------------------------------------------------------------------

class TestFrameLogContent:

    def test_frame_contains_required_fields(self):
        """Verbose frame log includes distance, azimuth, valid, command, etc."""
        logger, lines = _collecting_logger()
        state = _make_state()
        logger.log_frame(state, force=True)
        assert len(lines) >= 1
        msg = lines[0]
        assert "distance=" in msg
        assert "azimuth=" in msg
        assert "valid=" in msg
        assert "command=" in msg
        assert "reason=" in msg
        assert "vx=" in msg
        assert "omega=" in msg
        assert "robot=" in msg
        assert "target=" in msg

    def test_frame_includes_specific_values(self):
        """Frame log reflects the actual state values."""
        logger, lines = _collecting_logger()
        state = _make_state(distance_m=3.14, azimuth_deg=-12.5, valid=True,
                            kind="move", reason="tracking_forward", vx=0.13)
        logger.log_frame(state, force=True)
        msg = lines[0]
        assert "3.14" in msg
        assert "-12.5" in msg
        assert "tracking_forward" in msg
        assert "0.13" in msg


# ---------------------------------------------------------------------------
# Event detection
# ---------------------------------------------------------------------------

class TestEventDetection:

    def test_command_reason_change_emits_event(self):
        """When command reason changes, an EVENT is emitted."""
        logger, lines = _collecting_logger()
        s1 = _make_state(reason="tracking_forward", t=0.0)
        s2 = _make_state(reason="too_close", t=0.025)
        logger.log_frame(s1, force=True)
        n_after_first = len(lines)
        logger.log_frame(s2, force=True)
        assert len(lines) > n_after_first
        new_lines = lines[n_after_first:]
        events = [l for l in new_lines if "EVENT" in l]
        assert len(events) >= 1
        event = events[0]
        assert "command_changed" in event

    def test_valid_to_invalid_emits_event(self):
        """Target leaving valid range emits target_left_fov event."""
        logger, lines = _collecting_logger()
        s1 = _make_state(valid=True, in_fov=True, reason="tracking_forward", t=0.0)
        s2 = _make_state(valid=False, in_fov=False, reason="data_invalid", t=0.025)
        logger.log_frame(s1, force=True)
        n_after_first = len(lines)
        logger.log_frame(s2, force=True)
        assert len(lines) > n_after_first
        events = [l for l in lines[n_after_first:] if "EVENT" in l]
        assert len(events) >= 1
        assert any("target_left_fov" in e for e in events)

    def test_invalid_to_valid_emits_reacquired(self):
        """Target returning to FOV emits target_reacquired event."""
        logger, lines = _collecting_logger()
        s1 = _make_state(valid=False, in_fov=False, reason="data_invalid", t=0.0)
        s2 = _make_state(valid=True, in_fov=True, reason="tracking_forward", t=0.025)
        logger.log_frame(s1, force=True)
        n_after_first = len(lines)
        logger.log_frame(s2, force=True)
        events = [l for l in lines[n_after_first:] if "EVENT" in l]
        assert any("target_reacquired" in e for e in events)


# ---------------------------------------------------------------------------
# Rate-limiting and suppression
# ---------------------------------------------------------------------------

class TestRateLimitingAndSuppression:

    def test_rate_limit_suppresses_frames(self):
        """Frames closer than threshold_s apart are suppressed."""
        logger, lines = _collecting_logger(threshold=1.0)
        # Simulate monotonic clock by setting t
        s1 = _make_state(t=0.0)
        s2 = _make_state(t=0.025)  # only 25ms later, below 1.0s threshold
        logger.log_frame(s1)
        logger.log_frame(s2)
        # Only one FRAME line (second is suppressed)
        frames = [l for l in lines if l.startswith("FRAME")]
        assert len(frames) == 1

    def test_rate_limit_allows_after_threshold(self):
        """Frame log emitted when enough time has passed."""
        logger, lines = _collecting_logger(threshold=0.05)
        s1 = _make_state(t=0.0)
        s2 = _make_state(t=0.06)  # 60ms > 50ms threshold
        logger.log_frame(s1)
        logger.log_frame(s2)
        frames = [l for l in lines if l.startswith("FRAME")]
        assert len(frames) == 2

    def test_no_duplicate_events(self):
        """Consecutive identical states do NOT re-emit events."""
        logger, lines = _collecting_logger()
        s = _make_state(reason="tracking_forward", t=0.0)
        logger.log_frame(s, force=True)
        n1 = len(lines)
        logger.log_frame(s, force=True)
        n2 = len(lines)
        # Only FRAME lines, no new EVENT
        new_lines = lines[n1:n2]
        events = [l for l in new_lines if "EVENT" in l]
        assert len(events) == 0

    def test_force_bypasses_rate_limit(self):
        """force=True always emits a frame log."""
        logger, lines = _collecting_logger(threshold=100.0)
        s = _make_state(t=0.0)
        logger.log_frame(s, force=True)
        logger.log_frame(s, force=True)
        frames = [l for l in lines if l.startswith("FRAME")]
        assert len(frames) == 2

    def test_events_always_emitted_regardless_of_rate_limit(self):
        """Events are emitted even when frame log is rate-limited."""
        logger, lines = _collecting_logger(threshold=100.0)
        s1 = _make_state(reason="tracking_forward", t=0.0)
        s2 = _make_state(reason="too_close", t=0.025)
        logger.log_frame(s1)  # emits frame
        logger.log_frame(s2)  # frame suppressed, but event must emit
        events = [l for l in lines if "EVENT" in l]
        assert len(events) >= 1

    def test_verbose_mode_ignores_rate_limit(self):
        """When verbose=True, every frame log is emitted."""
        logger, lines = _collecting_logger(threshold=100.0)
        logger.verbose = True
        for i in range(5):
            logger.log_frame(_make_state(t=i * 0.025))
        frames = [l for l in lines if l.startswith("FRAME")]
        assert len(frames) == 5
