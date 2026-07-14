"""Web server with controller lease and bounded broadcast."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import aiohttp
from aiohttp import web

from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEvent,
    ControlEventInbox,
    ControlEventKind,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.web.protocol import (
    ClientMessage,
    MessageType,
    parse_client_message,
)

logger = logging.getLogger(__name__)


@dataclass
class ClientInfo:
    """Per-client connection state."""

    client_id: str
    role: str  # "controller" or "observer"
    last_version: int = 0
    event_times: List[float] = field(default_factory=list)

    def check_rate_limit(self, max_per_second: float, now: float) -> bool:
        """Check if client is within rate limit.

        Prunes events older than 1 second before checking.

        Parameters
        ----------
        max_per_second : float
            Maximum events per second allowed.
        now : float
            Current monotonic time.

        Returns
        -------
        bool
            True if within limit, False if exceeded.
        """
        # Prune events older than 1 second
        cutoff = now - 1.0
        self.event_times = [t for t in self.event_times if t >= cutoff]

        # Check limit
        if len(self.event_times) >= max_per_second:
            return False

        self.event_times.append(now)
        return True


class PrototypeWebHub:
    """Controller lease manager and status broadcaster.

    Thread-safe for concurrent access from WebSocket handlers.
    """

    def __init__(
        self,
        stop_latch: StopLatch,
        event_inbox: ControlEventInbox,
        latest_status: LatestStatus,
        max_clients: int = 8,
        max_events_per_second: float = 20.0,
    ) -> None:
        self._stop_latch = stop_latch
        self._event_inbox = event_inbox
        self._latest_status = latest_status
        self._max_clients = max_clients
        self._max_events_per_second = max_events_per_second

        self._lock = asyncio.Lock()
        self._clients: Dict[str, ClientInfo] = {}
        self._controller_id: Optional[str] = None
        self._role_sent: Dict[str, bool] = {}

    @property
    def controller_id(self) -> Optional[str]:
        """Get the current controller client ID."""
        return self._controller_id

    @property
    def client_count(self) -> int:
        """Number of connected clients."""
        return len(self._clients)

    def role_of(self, client_id: str) -> Optional[str]:
        """Get the role of a connected client.

        Parameters
        ----------
        client_id : str
            Client ID to query.

        Returns
        -------
        str or None
            "controller", "observer", or None if unknown.
        """
        client = self._clients.get(client_id)
        if client is None:
            return None
        return client.role

    async def connect(self, client_id: str) -> ClientInfo:
        """Register a new WebSocket connection.

        First connection becomes controller; subsequent connections
        become observers.  Rejects if at max_clients.

        Parameters
        ----------
        client_id : str
            Unique client identifier (WebSocket session ID).

        Returns
        -------
        ClientInfo
            The new client's info with assigned role.

        Raises
        ------
        ValueError
            If max_clients already reached.
        """
        async with self._lock:
            if len(self._clients) >= self._max_clients:
                raise ValueError(f"max_clients ({self._max_clients}) reached")

            # Assign role: first connection is controller
            if self._controller_id is None:
                role = "controller"
                self._controller_id = client_id
            else:
                role = "observer"

            info = ClientInfo(client_id=client_id, role=role)
            self._clients[client_id] = info
            self._role_sent[client_id] = False

            return info

    async def disconnect(self, client_id: str) -> None:
        """Handle WebSocket disconnection.

        If the client is controller:
        1. Request HARD stop on StopLatch
        2. Enqueue CONTROLLER_DISCONNECT event
        3. Clear controller lease

        Parameters
        ----------
        client_id : str
            Disconnected client ID.
        """
        async with self._lock:
            client = self._clients.pop(client_id, None)
            self._role_sent.pop(client_id, None)

            if client is None:
                return

            if client.role == "controller":
                # Request HARD stop (any thread can call this)
                self._stop_latch.request("controller_disconnected", StopKind.HARD)

                # Enqueue disconnect event
                self._event_inbox.push(
                    ControlEvent(
                        kind=ControlEventKind.CONTROLLER_DISCONNECT,
                        payload={},
                    )
                )

                # Clear lease
                self._controller_id = None

    async def handle_client_message(
        self,
        client_id: str,
        message_type: str,
        payload: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Handle a parsed client message.

        Observers receive read_only error.  Controllers have their
        messages forwarded to the control event inbox.

        Parameters
        ----------
        client_id : str
            Sending client ID.
        message_type : str
            Message type string.
        payload : dict
            Message payload.

        Returns
        -------
        dict
            Response dict (ack or error).
        """
        async with self._lock:
            client = self._clients.get(client_id)

            # Unknown client
            if client is None:
                return {
                    "type": "error",
                    "payload": {"code": "invalid_state", "message": "unknown client"},
                }

            # Rate limit check
            if not client.check_rate_limit(self._max_events_per_second, time.monotonic()):
                return {
                    "type": "error",
                    "payload": {
                        "code": "rate_limited",
                        "message": "too many events per second",
                    },
                }

            # Observer cannot control
            if client.role == "observer" and message_type in (
                "select_target",
                "start_follow",
                "stop_follow",
            ):
                return {
                    "type": "error",
                    "payload": {
                        "code": "read_only",
                        "message": "observer cannot control",
                    },
                }

            # Process control messages
            if message_type == "select_target":
                self._event_inbox.push(
                    ControlEvent(
                        kind=ControlEventKind.SELECT_TARGET,
                        payload=payload,
                    )
                )
                return {"type": "ack"}

            elif message_type == "start_follow":
                self._event_inbox.push(
                    ControlEvent(
                        kind=ControlEventKind.START_FOLLOW,
                        payload={},
                    )
                )
                return {"type": "ack"}

            elif message_type == "stop_follow":
                self._stop_latch.request("user_stop", StopKind.HARD)
                self._event_inbox.push(
                    ControlEvent(
                        kind=ControlEventKind.STOP_FOLLOW,
                        payload={},
                    )
                )
                return {"type": "ack"}

            else:
                return {
                    "type": "error",
                    "payload": {
                        "code": "invalid_schema",
                        "message": f"unknown message type: {message_type}",
                    },
                }

    async def get_status_frame(
        self, client_id: str, last_version: int
    ) -> Optional[Dict[str, Any]]:
        """Get status frame for a client, only if newer than last_version.

        Parameters
        ----------
        client_id : str
            Client ID requesting status.
        last_version : int
            Last version the client has seen.

        Returns
        -------
        dict or None
            Status frame dict with "type":"frame" and "payload", or None
            if no new version available.
        """
        version, status = self._latest_status.snapshot()

        if version <= last_version:
            return None

        return {
            "type": "frame",
            "payload": {
                "version": version,
                **status,
            },
        }

    async def get_role_message(self, client_id: str) -> Optional[Dict[str, Any]]:
        """Get role message for a client, only if not yet sent.

        Parameters
        ----------
        client_id : str
            Client ID.

        Returns
        -------
        dict or None
            Role message or None if already sent.
        """
        async with self._lock:
            if self._role_sent.get(client_id, True):
                return None

            client = self._clients.get(client_id)
            if client is None:
                return None

            self._role_sent[client_id] = True
            return {
                "type": "role",
                "payload": {"role": client.role},
            }

    async def broadcast_excluding(
        self, message: Dict[str, Any], exclude_id: Optional[str] = None
    ) -> None:
        """Broadcast a message to all connected clients.

        This is a no-op in the hub itself; actual WebSocket send is
        managed by the aiohttp handler.  This method exists for
        potential future use and testability.

        Parameters
        ----------
        message : dict
            Message to broadcast.
        exclude_id : str or None
            Optional client ID to exclude.
        """
        pass  # Actual send is in the WebSocket handler


def create_app(
    hub: PrototypeWebHub,
    static_dir: str,
) -> web.Application:
    """Create the aiohttp web application.

    Parameters
    ----------
    hub : PrototypeWebHub
        Web hub instance.
    static_dir : str
        Path to frontend static files directory.

    Returns
    -------
    web.Application
        Configured aiohttp application.
    """
    app = web.Application()

    # Static files
    app.router.add_static("/assets", path=f"{static_dir}/assets/")
    app.router.add_get("/", _handle_index)
    app.router.add_get("/ws", _handle_websocket)

    # Store hub reference
    app["hub"] = hub
    app["static_dir"] = static_dir

    return app


async def _handle_index(request: web.Request) -> web.StreamResponse:
    """Serve the frontend index.html."""
    static_dir = request.app["static_dir"]
    return web.FileResponse(f"{static_dir}/index.html")


async def _handle_websocket(request: web.Request) -> web.WebSocketResponse:
    """Handle WebSocket connections.

    Manages controller lease, message parsing, and status broadcast.
    """
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    hub: PrototypeWebHub = request.app["hub"]
    client_id = id(ws)
    logger.info("WebSocket connection: %s", client_id)

    # Register client
    try:
        info = await hub.connect(client_id)
    except ValueError:
        await ws.send_json(
            {"type": "error", "payload": {"code": "invalid_state", "message": "server full"}}
        )
        await ws.close()
        return ws

    # Send role
    role_msg = await hub.get_role_message(client_id)
    if role_msg is not None:
        await ws.send_json(role_msg)

    # Start status broadcast task
    async def broadcast_status() -> None:
        """Periodic status broadcast for this client."""
        last_version = 0
        while not ws.closed:
            try:
                # Get status frame (only if newer)
                frame = await hub.get_status_frame(client_id, last_version)
                if frame is not None:
                    last_version = frame["payload"]["version"]
                    await ws.send_json(frame)

                await asyncio.sleep(0.1)  # 10 Hz
            except (ConnectionResetError, asyncio.CancelledError):
                break
            except Exception:
                logger.exception("Error broadcasting status to %s", client_id)
                break

    broadcast_task = asyncio.ensure_future(broadcast_status())

    try:
        # Handle messages from client
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    # Parse client message
                    parsed = parse_client_message(
                        msg.data, max_bytes=4096
                    )

                    # Build payload for hub
                    payload = {}
                    if parsed.x is not None:
                        payload["x"] = parsed.x
                    if parsed.y is not None:
                        payload["y"] = parsed.y

                    # Handle through hub
                    response = await hub.handle_client_message(
                        client_id=client_id,
                        message_type=parsed.type.value,
                        payload=payload,
                    )

                    await ws.send_json(response)

                except ValueError as e:
                    # Parse/validation error
                    error_code = str(e).split(":")[0].strip()
                    await ws.send_json(
                        {
                            "type": "error",
                            "payload": {
                                "code": error_code,
                                "message": str(e),
                            },
                        }
                    )

            elif msg.type == aiohttp.WSMsgType.ERROR:
                logger.error(
                    "WebSocket error for %s: %s",
                    client_id,
                    ws.exception(),
                )
                break

    except asyncio.CancelledError:
        pass
    finally:
        # Cleanup
        broadcast_task.cancel()
        await hub.disconnect(client_id)
        logger.info("WebSocket disconnected: %s", client_id)

    return ws


async def run_web_server(
    hub: PrototypeWebHub,
    host: str,
    port: int,
    static_dir: str,
) -> web.AppRunner:
    """Run the aiohttp web server.

    Parameters
    ----------
    hub : PrototypeWebHub
        Web hub instance.
    host : str
        Bind host address.
    port : int
        Bind port.
    static_dir : str
        Path to frontend static files directory.

    Returns
    -------
    web.AppRunner
        The app runner (caller must call runner.cleanup() on shutdown).
    """
    app = create_app(hub=hub, static_dir=static_dir)
    runner = web.AppRunner(app)
    await runner.setup()

    site = web.TCPSite(runner, host=host, port=port)
    await site.start()

    logger.info("Web server started on %s:%s", host, port)

    return runner
