"""Tests for Web-start control and bounded status broadcast."""

from __future__ import annotations

import time
from typing import Any

import pytest

from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind, StopSnapshot
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    ControlEventKind,
    LatestStatus,
)
from robot_follow.lidar_alx_assisted_follow.web.server import (
    ClientInfo,
    PrototypeWebHub,
    _close_websockets,
    create_app,
)

class FakeStopLatch:
    def __init__(self) -> None:
        self.requests: list[tuple[str, StopKind]] = []
        self._requested = False
        self._kind: StopKind | None = None
        self._reason: str | None = None

    def request(self, reason: str, kind: StopKind) -> None:
        self.requests.append((reason, kind))
        self._requested, self._kind, self._reason = True, kind, reason

    def snapshot(self) -> Any:
        return StopSnapshot(self._requested, self._kind, self._reason, 0, 1)


class FakeWebSocket:
    def __init__(self) -> None:
        self.closed = False
        self.close_calls: list[tuple[int, bytes]] = []

    async def close(self, *, code: int, message: bytes) -> None:
        self.close_calls.append((code, message))
        self.closed = True


def make_hub(*, max_clients: int = 8, rate: float = 20.0) -> tuple[PrototypeWebHub, FakeStopLatch, ControlEventInbox]:
    latch = FakeStopLatch()
    inbox = ControlEventInbox()
    return (
        PrototypeWebHub(latch, inbox, LatestStatus(), max_clients=max_clients, max_events_per_second=rate),  # type: ignore[arg-type]
        latch,
        inbox,
    )


@pytest.mark.asyncio
class TestPrototypeWebHub:
    async def test_connections_have_no_controller_lease(self) -> None:
        hub, _, _ = make_hub()
        first = await hub.connect("client-a")
        second = await hub.connect("client-b")
        assert first.client_id == "client-a"
        assert second.client_id == "client-b"
        assert hub.client_count == 2

    async def test_disconnect_never_stops_or_enqueues_control_event(self) -> None:
        hub, latch, inbox = make_hub()
        await hub.connect("client-a")
        await hub.connect("client-b")
        await hub.disconnect("client-a")
        assert latch.requests == []
        assert inbox.qsize == 0
        assert hub.client_count == 1

    async def test_any_connected_client_can_start_follow(self) -> None:
        hub, _, inbox = make_hub()
        await hub.connect("phone")
        result = await hub.handle_client_message("phone", "start_follow", {})
        assert result == {"type": "ack"}
        assert inbox.pop().kind == ControlEventKind.START_FOLLOW

    async def test_select_target_is_rejected_because_alx_is_the_only_target(self) -> None:
        hub, _, inbox = make_hub()
        await hub.connect("phone")
        result = await hub.handle_client_message("phone", "select_target", {"x": 1.0, "y": 0.5})
        assert result["type"] == "error"
        assert result["payload"]["code"] == "unsupported_control"
        assert inbox.qsize == 0

    async def test_stop_latches_immediately_and_enqueues_runtime_disable(self) -> None:
        hub, latch, inbox = make_hub()
        await hub.connect("phone")
        result = await hub.handle_client_message("phone", "stop_follow", {})
        assert result == {"type": "ack"}
        assert latch.requests == [("user_stop", StopKind.HARD)]
        assert inbox.pop().kind == ControlEventKind.STOP_FOLLOW

    async def test_rate_limiting_is_per_connection(self) -> None:
        hub, _, _ = make_hub(rate=2.0)
        await hub.connect("phone")
        await hub.handle_client_message("phone", "start_follow", {})
        await hub.handle_client_message("phone", "start_follow", {})
        result = await hub.handle_client_message("phone", "start_follow", {})
        assert result["payload"]["code"] == "rate_limited"

    async def test_max_clients_is_enforced(self) -> None:
        hub, _, _ = make_hub(max_clients=2)
        await hub.connect("a")
        await hub.connect("b")
        with pytest.raises(ValueError, match="max_clients"):
            await hub.connect("c")

    async def test_status_broadcast_respects_version(self) -> None:
        hub, _, _ = make_hub()
        await hub.connect("phone")
        status = hub._latest_status
        first = status.update({"cloud_sequence": 1})
        second = status.update({"cloud_sequence": 2})
        frame = await hub.get_status_frame("phone", first)
        assert frame is not None and frame["payload"]["version"] == second
        assert await hub.get_status_frame("phone", second) is None

    async def test_shutdown_closes_open_websockets(self, tmp_path) -> None:
        static_dir = tmp_path / "static"
        (static_dir / "assets").mkdir(parents=True)
        (static_dir / "index.html").write_text("<!doctype html>")
        hub, _, _ = make_hub()
        app = create_app(hub, str(static_dir))
        websocket = FakeWebSocket()
        app["websockets"].add(websocket)
        await _close_websockets(app)
        assert websocket.closed is True
        assert websocket.close_calls == [(1001, b"Server shutdown")]


class TestClientInfo:
    def test_default_values(self) -> None:
        info = ClientInfo(client_id="test")
        assert info.client_id == "test"
        assert info.last_version == 0
        assert info.event_times == []

    def test_rate_limit_prunes_old_events(self) -> None:
        info = ClientInfo(client_id="test")
        now = time.monotonic()
        info.event_times = [now - 2.0, now - 0.1]
        assert info.check_rate_limit(2.0, now) is True
        assert len(info.event_times) == 2
