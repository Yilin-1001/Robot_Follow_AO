"""Runtime state management tests for Task 2 Step 1."""

from __future__ import annotations

import threading

import pytest

from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    StopLatch,
    ControlEventInbox,
    ControlEvent,
    ControlEventKind,
    LatestStatus,
)


class TestStopLatchOwnership:
    """Test that only the control thread can clear stop latch."""

    def test_any_thread_can_request_but_only_control_thread_can_clear(self) -> None:
        """Any thread can request stop, but only control thread can clear."""
        latch = StopLatch(control_thread_id=threading.get_ident())

        # Any thread can request
        latch.request("controller_disconnected", StopKind.HARD)
        assert latch.snapshot().requested
        assert latch.snapshot().kind == StopKind.HARD
        assert latch.snapshot().reason == "controller_disconnected"

        # Control thread can clear
        latch.clear_for_start(start_after_sequence=8)
        assert not latch.snapshot().requested
        assert latch.snapshot().start_after_sequence == 8

    def test_non_control_thread_cannot_clear(self) -> None:
        """Non-control thread attempting to clear should raise RuntimeError."""
        main_thread_id = threading.get_ident()
        latch = StopLatch(control_thread_id=main_thread_id)

        latch.request("test_stop", StopKind.HARD)

        # Try to clear from another thread
        exception_raised = False

        def try_clear() -> None:
            nonlocal exception_raised
            try:
                latch.clear_for_start(start_after_sequence=10)
            except RuntimeError:
                exception_raised = True

        thread = threading.Thread(target=try_clear)
        thread.start()
        thread.join()
        assert exception_raised

    def test_nonzero_requires_post_start_cloud(self) -> None:
        """Motion not allowed for sequence <= start_after_sequence."""
        latch = StopLatch(control_thread_id=threading.get_ident())
        latch.clear_for_start(start_after_sequence=8)
        assert not latch.motion_allowed_for(sequence=8)
        assert latch.motion_allowed_for(sequence=9)


class TestControlEventInbox:
    """Test bounded event inbox with select_target coalescing."""

    def test_select_target_coalesces_to_newest(self) -> None:
        """New select_target replaces pending one, keeping queue compact."""
        inbox = ControlEventInbox(capacity=4)
        inbox.push(ControlEvent(ControlEventKind.SELECT_TARGET, {"x": 1.0, "y": 0.0}))
        inbox.push(ControlEvent(ControlEventKind.SELECT_TARGET, {"x": 2.0, "y": 0.5}))

        assert inbox.qsize == 1
        event = inbox.pop()
        assert event is not None
        assert event.payload["x"] == 2.0
        assert event.payload["y"] == 0.5

    def test_other_events_append_and_do_not_coalesce(self) -> None:
        """start_follow and stop_follow always append."""
        inbox = ControlEventInbox(capacity=4)
        inbox.push(ControlEvent(ControlEventKind.START_FOLLOW, {}))
        inbox.push(ControlEvent(ControlEventKind.START_FOLLOW, {}))

        assert inbox.qsize == 2

    def test_full_queue_evicts_oldest_select_target(self) -> None:
        """When full, remove oldest select_target to make room."""
        inbox = ControlEventInbox(capacity=2)
        inbox.push(ControlEvent(ControlEventKind.SELECT_TARGET, {"x": 1.0}))
        inbox.push(ControlEvent(ControlEventKind.START_FOLLOW, {}))

        assert inbox.qsize == 2
        # Now full; push stop_follow should evict the select_target
        result = inbox.push(ControlEvent(ControlEventKind.STOP_FOLLOW, {}))
        assert result
        assert inbox.qsize == 2

        events = inbox.snapshot()
        assert events[0][0] == ControlEventKind.START_FOLLOW
        assert events[1][0] == ControlEventKind.STOP_FOLLOW

    def test_full_queue_with_no_select_target_rejects(self) -> None:
        """If full and no select_target to evict, reject the new event."""
        inbox = ControlEventInbox(capacity=2)
        inbox.push(ControlEvent(ControlEventKind.START_FOLLOW, {}))
        inbox.push(ControlEvent(ControlEventKind.STOP_FOLLOW, {}))

        result = inbox.push(ControlEvent(ControlEventKind.START_FOLLOW, {}))
        assert not result
        assert inbox.qsize == 2


class TestLatestStatus:
    """Test versioned latest status container."""

    def test_update_replaces_and_bumps_version(self) -> None:
        """Each update increments version."""
        status = LatestStatus()
        v1 = status.update({"state": "idle"})
        v2 = status.update({"state": "following"})

        assert v2 == v1 + 1
        version, snapshot = status.snapshot()
        assert version == v2
        assert snapshot["state"] == "following"

    def test_snapshot_returns_copy(self) -> None:
        """Modifying snapshot does not affect internal state."""
        status = LatestStatus()
        status.update({"count": 10})

        _, snapshot = status.snapshot()
        snapshot["count"] = 20

        _, current = status.snapshot()
        assert current["count"] == 10