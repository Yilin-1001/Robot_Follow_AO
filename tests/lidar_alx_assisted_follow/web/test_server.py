"""Tests for Web server lease and broadcast behavior."""

from __future__ import annotations

import asyncio
import time
from typing import Any, Dict
from unittest.mock import MagicMock, patch

import pytest

pytestmark = pytest.mark.asyncio

from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    ControlEventKind,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.web.server import (
    ClientInfo,
    PrototypeWebHub,
)


class FakeStopLatch:
    """Fake StopLatch for testing."""

    def __init__(self) -> None:
        self.requests: list[tuple[str, StopKind]] = []
        self._requested = False
        self._kind: StopKind | None = None
        self._reason: str | None = None

    def request(self, reason: str, kind: StopKind) -> None:
        """Record stop request."""
        self.requests.append((reason, kind))
        self._requested = True
        self._kind = kind
        self._reason = reason

    def snapshot(self) -> Any:
        """Return fake snapshot."""
        from robot_follow.lidar_alx_assisted_follow.domain.models import StopSnapshot
        return StopSnapshot(
            requested=self._requested,
            kind=self._kind,
            reason=self._reason,
            start_after_sequence=0,
            version=1,
        )


class TestPrototypeWebHub:
    """Tests for PrototypeWebHub."""

    @pytest.mark.asyncio
    async def test_first_connection_becomes_controller(self) -> None:
        """First connection becomes controller."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        info = await hub.connect("client-a")
        assert info.role == "controller"
        assert hub.controller_id == "client-a"
        assert hub.role_of("client-a") == "controller"

    @pytest.mark.asyncio
    async def test_second_connection_becomes_observer(self) -> None:
        """Connection when controller exists becomes observer."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("client-a")
        info = await hub.connect("client-b")

        assert info.role == "observer"
        assert hub.controller_id == "client-a"
        assert hub.role_of("client-b") == "observer"

    @pytest.mark.asyncio
    async def test_controller_disconnect_stops_and_unlocks_without_promoting_observer(self) -> None:
        """Controller disconnect requests HARD stop and clears lease."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("controller")
        await hub.connect("observer")

        await hub.disconnect("controller")

        # StopLatch.request called with HARD
        assert len(latch.requests) == 1
        assert latch.requests[0] == ("controller_disconnected", StopKind.HARD)

        # Controller lease cleared
        assert hub.controller_id is None

        # Observer remains observer (no promotion)
        assert hub.role_of("observer") == "observer"

        # Disconnect event enqueued
        assert inbox.qsize == 1
        event = inbox.pop()
        assert event is not None
        assert event.kind == ControlEventKind.CONTROLLER_DISCONNECT

    @pytest.mark.asyncio
    async def test_new_connection_after_vacancy_becomes_controller(self) -> None:
        """First connection after controller disconnect becomes controller."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("old-controller")
        await hub.connect("observer")
        await hub.disconnect("old-controller")

        # New connection becomes controller
        info = await hub.connect("new-client")
        assert info.role == "controller"
        assert hub.controller_id == "new-client"

    @pytest.mark.asyncio
    async def test_observer_disconnect_does_not_stop(self) -> None:
        """Observer disconnect does not request stop."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("controller")
        await hub.connect("observer")

        await hub.disconnect("observer")

        # No stop requested
        assert len(latch.requests) == 0
        assert hub.controller_id == "controller"

    @pytest.mark.asyncio
    async def test_observer_cannot_send_control_messages(self) -> None:
        """Observer sending control message returns read_only error."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("controller")
        await hub.connect("observer")

        result = await hub.handle_client_message(
            client_id="observer",
            message_type="select_target",
            payload={"x": 1.0, "y": 0.5},
        )

        assert result["type"] == "error"
        assert result["payload"]["code"] == "read_only"

    @pytest.mark.asyncio
    async def test_controller_can_send_control_messages(self) -> None:
        """Controller can send control messages."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("controller")

        result = await hub.handle_client_message(
            client_id="controller",
            message_type="select_target",
            payload={"x": 1.0, "y": 0.5},
        )

        assert result["type"] == "ack"
        assert inbox.qsize == 1

    @pytest.mark.asyncio
    async def test_controller_stop_requests_hard_stop_before_enqueueing(self) -> None:
        """A user Stop must latch immediately, without waiting for control polling."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=LatestStatus(),
            max_clients=8,
            max_events_per_second=20.0,
        )
        await hub.connect("controller")

        result = await hub.handle_client_message(
            client_id="controller", message_type="stop_follow", payload={}
        )

        assert result == {"type": "ack"}
        assert latch.requests == [("user_stop", StopKind.HARD)]
        assert inbox.pop().kind == ControlEventKind.STOP_FOLLOW

    @pytest.mark.asyncio
    async def test_rate_limiting_enforced(self) -> None:
        """Rate limiting enforced per client."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=5.0,  # 5 events per second
        )

        await hub.connect("controller")

        # Send 6 messages rapidly (should trigger rate limit on 6th)
        for i in range(6):
            result = await hub.handle_client_message(
                client_id="controller",
                message_type="select_target",
                payload={"x": float(i), "y": 0.0},
            )

        # Last message should be rate_limited
        assert result["type"] == "error"
        assert result["payload"]["code"] == "rate_limited"

    @pytest.mark.asyncio
    async def test_max_clients_enforced(self) -> None:
        """Max clients enforced."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=2,
            max_events_per_second=20.0,
        )

        await hub.connect("client-a")  # controller
        await hub.connect("client-b")  # observer

        # Third client rejected
        with pytest.raises(ValueError, match="max_clients"):
            await hub.connect("client-c")

    @pytest.mark.asyncio
    async def test_status_broadcast_respects_version(self) -> None:
        """Status broadcast only sends newer versions."""
        latch = FakeStopLatch()
        inbox = ControlEventInbox()
        status = LatestStatus()
        hub = PrototypeWebHub(
            stop_latch=latch,  # type: ignore
            event_inbox=inbox,
            latest_status=status,
            max_clients=8,
            max_events_per_second=20.0,
        )

        await hub.connect("client-a")

        # Update status twice
        v1 = status.update({"cloud_sequence": 1})
        v2 = status.update({"cloud_sequence": 2})

        # Client already has v1, should get v2
        frame = await hub.get_status_frame("client-a", last_version=v1)
        assert frame is not None
        assert frame["payload"]["version"] == v2

        # Client already has v2, should get None
        frame = await hub.get_status_frame("client-a", last_version=v2)
        assert frame is None


class TestClientInfo:
    """Tests for ClientInfo."""

    def test_default_values(self) -> None:
        """ClientInfo has correct defaults."""
        info = ClientInfo(client_id="test", role="observer")
        assert info.client_id == "test"
        assert info.role == "observer"
        assert info.last_version == 0
        assert info.event_times == []

    def test_check_rate_limit_within_limit(self) -> None:
        """check_rate_limit returns True when within limit."""
        info = ClientInfo(client_id="test", role="controller")
        now = time.monotonic()

        # Add 5 events, all within 1 second
        info.event_times = [now - 0.1 * i for i in range(5)]

        # check_rate_limit returns True and adds one more event
        assert info.check_rate_limit(max_per_second=10.0, now=now) is True
        assert len(info.event_times) == 6

    def test_check_rate_limit_exceeds_limit(self) -> None:
        """check_rate_limit returns False when exceeds limit."""
        info = ClientInfo(client_id="test", role="controller")
        now = time.monotonic()

        # Add 10 events within 1 second (at limit already)
        info.event_times = [now - 0.05 * i for i in range(10)]

        # check_rate_limit returns False
        assert info.check_rate_limit(max_per_second=10.0, now=now) is False

    def test_event_times_pruned(self) -> None:
        """Old event_times are pruned."""
        info = ClientInfo(client_id="test", role="controller")
        now = time.monotonic()

        # Add old events (2+ seconds ago) plus one recent
        info.event_times = [now - 2.5, now - 3.0, now - 0.1]

        # check_rate_limit prunes old events and succeeds
        assert info.check_rate_limit(max_per_second=10.0, now=now) is True

        # Only recent event remains plus the new one added by check_rate_limit
        assert len(info.event_times) == 2
