"""Tests for sim/mock_motion_bridge.py."""

import pytest

from robot_follow.core.types import MotionCommand
from robot_follow.sim.mock_motion_bridge import MockMotionBridge


def _move(reason="test") -> MotionCommand:
    return MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason=reason)


def _stop(reason="test_stop") -> MotionCommand:
    return MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)


class TestMockMotionBridgeConstruction:
    def test_new_bridge_has_empty_history(self):
        bridge = MockMotionBridge()
        assert len(bridge.history) == 0
        assert bridge.last is None
        assert bridge.move_count == 0
        assert bridge.stop_count == 0


class TestMockMotionBridgeDispatch:
    def test_dispatch_move_appends_move(self):
        bridge = MockMotionBridge()
        cmd = _move()
        bridge.dispatch(cmd)
        assert bridge.move_count == 1
        assert bridge.stop_count == 0
        assert bridge.last.kind == "move"

    def test_dispatch_stop_appends_stop(self):
        bridge = MockMotionBridge()
        cmd = _stop()
        bridge.dispatch(cmd)
        assert bridge.move_count == 0
        assert bridge.stop_count == 1
        assert bridge.last.kind == "stop"
        assert bridge.last.reason == "test_stop"

    def test_dispatch_preserves_command_object_for_move(self):
        bridge = MockMotionBridge()
        cmd = _move(reason="original")
        bridge.dispatch(cmd)
        # The command object itself is preserved in history.
        assert bridge.history[0] == cmd

    def test_dispatch_preserves_command_object_for_stop(self):
        bridge = MockMotionBridge()
        cmd = _stop(reason="original")
        bridge.dispatch(cmd)
        assert bridge.history[0] == cmd


class TestMockMotionBridgeMoveMethod:
    def test_move_accepts_move_command(self):
        bridge = MockMotionBridge()
        bridge.move(_move())
        assert bridge.move_count == 1
        assert bridge.last.kind == "move"


class TestMockMotionBridgeStopMethod:
    def test_stop_accepts_motion_command(self):
        bridge = MockMotionBridge()
        cmd = _stop(reason="from_stop")
        bridge.stop(cmd)
        assert bridge.stop_count == 1
        assert bridge.last == cmd

    def test_stop_accepts_string_reason(self):
        bridge = MockMotionBridge()
        bridge.stop("manual_abort")
        assert bridge.stop_count == 1
        assert bridge.last.kind == "stop"
        assert bridge.last.reason == "manual_abort"

    def test_stop_string_creates_zero_velocities(self):
        bridge = MockMotionBridge()
        bridge.stop("data_invalid")
        assert bridge.last.vx == 0.0
        assert bridge.last.vy == 0.0
        assert bridge.last.omega == 0.0


class TestMockMotionBridgeProperties:
    def test_moves_filters_moves_only(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move("a"))
        bridge.dispatch(_stop("b"))
        bridge.dispatch(_move("c"))
        assert len(bridge.moves) == 2
        assert all(c.kind == "move" for c in bridge.moves)

    def test_stops_filters_stops_only(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        bridge.dispatch(_stop())
        bridge.dispatch(_stop())
        assert len(bridge.stops) == 2
        assert all(c.kind == "stop" for c in bridge.stops)

    def test_last_returns_most_recent(self):
        bridge = MockMotionBridge()
        assert bridge.last is None
        bridge.dispatch(_move("first"))
        assert bridge.last.reason == "first"
        bridge.dispatch(_stop("second"))
        assert bridge.last.reason == "second"

    def test_history_returns_copy(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        history = bridge.history
        history.clear()
        assert len(bridge.history) == 1  # original unaffected


class TestMockMotionBridgeReset:
    def test_reset_clears_history(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        bridge.dispatch(_stop())
        bridge.reset()
        assert len(bridge.history) == 0
        assert bridge.last is None

    def test_reset_preserves_object_identity(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        bridge.reset()
        bridge.dispatch(_stop())
        assert bridge.stop_count == 1


class TestMockMotionBridgeMultiple:
    def test_counts_after_multiple_commands(self):
        bridge = MockMotionBridge()
        for i in range(5):
            bridge.dispatch(_move(f"m{i}"))
        for i in range(3):
            bridge.dispatch(_stop(f"s{i}"))
        assert bridge.move_count == 5
        assert bridge.stop_count == 3
        assert len(bridge.history) == 8


class TestMockMotionBridgeProtocol:
    def test_init_is_noop(self):
        bridge = MockMotionBridge()
        bridge.init()  # must not raise
        assert bridge.move_count == 0

    def test_tick_is_noop(self):
        import time
        bridge = MockMotionBridge()
        bridge.tick(time.monotonic())  # must not raise

    def test_close_is_noop_and_preserves_history(self):
        bridge = MockMotionBridge()
        bridge.dispatch(_move())
        bridge.close()
        assert bridge.move_count == 1  # close does not clear history

    def test_sdk_error_always_false(self):
        bridge = MockMotionBridge()
        assert bridge.sdk_error is False
        bridge.dispatch(_move())
        assert bridge.sdk_error is False

    def test_sdk_timeout_always_false(self):
        bridge = MockMotionBridge()
        assert bridge.sdk_timeout is False
        bridge.dispatch(_move())
        assert bridge.sdk_timeout is False
