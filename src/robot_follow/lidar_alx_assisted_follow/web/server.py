"""Web server for Web-started assisted follow and bounded broadcast."""

from __future__ import annotations

import asyncio
import json
import logging
import time
import weakref
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

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
from robot_follow.lidar_alx_assisted_follow.web.protocol import parse_client_message

logger = logging.getLogger(__name__)

WEB_SHUTDOWN_TIMEOUT_S = 1.0


@dataclass
class ClientInfo:
    """Per-client connection state."""

    client_id: str
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
    """Web control ingress and status broadcaster.

    Every connected Web client may request Start or Stop.  Connection state is
    intentionally not a motion-safety input: losing a browser must never stop
    an already enabled follow session.
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
    @property
    def client_count(self) -> int:
        """Number of connected clients."""
        return len(self._clients)

    async def connect(self, client_id: str) -> ClientInfo:
        """Register a new WebSocket connection.

        Rejects if ``max_clients`` is reached.  This is a monitoring/control
        connection only; it has no lease or safety authority.

        Parameters
        ----------
        client_id : str
            Unique client identifier (WebSocket session ID).

        Returns
        -------
        ClientInfo
            The new client's connection state.

        Raises
        ------
        ValueError
            If max_clients already reached.
        """
        async with self._lock:
            if len(self._clients) >= self._max_clients:
                raise ValueError(f"max_clients ({self._max_clients}) reached")

            info = ClientInfo(client_id=client_id)
            self._clients[client_id] = info
            return info

    async def disconnect(self, client_id: str) -> None:
        """Handle WebSocket disconnection.

        A browser disconnect only removes its monitoring connection.  The
        runtime keeps ``follow_enabled`` unchanged; wristband and sensor
        safety conditions remain the authoritative gates.

        Parameters
        ----------
        client_id : str
            Disconnected client ID.
        """
        async with self._lock:
            self._clients.pop(client_id, None)

    async def handle_client_message(
        self,
        client_id: str,
        message_type: str,
        payload: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Handle a parsed client message.

        Valid Start/Stop commands are forwarded to the runtime inbox.

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

            # Process control messages
            if message_type == "select_target":
                return {
                    "type": "error",
                    "payload": {
                        "code": "unsupported_control",
                        "message": "ALX is the only follow target",
                    },
                }

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
    # AppRunner otherwise waits for open WebSocket handlers until its generous
    # default shutdown timeout expires.  Keep weak references so shutdown can
    # actively close browser sessions before runner.cleanup() waits for them.
    app["websockets"] = weakref.WeakSet()
    app.on_shutdown.append(_close_websockets)

    # Static files
    app.router.add_static("/assets", path=f"{static_dir}/assets/")
    app.router.add_get("/", _handle_index)
    app.router.add_get("/ws", _handle_websocket)

    # Store hub reference
    app["hub"] = hub
    app["static_dir"] = static_dir

    return app


async def _close_websockets(app: web.Application) -> None:
    """Close active browser sessions so server shutdown cannot hang on a page."""
    websockets = tuple(app["websockets"])
    if not websockets:
        return
    await asyncio.gather(
        *(
            ws.close(
                code=aiohttp.WSCloseCode.GOING_AWAY,
                message=b"Server shutdown",
            )
            for ws in websockets
            if not ws.closed
        ),
        return_exceptions=True,
    )


async def _handle_index(request: web.Request) -> web.StreamResponse:
    """Serve the frontend index.html."""
    static_dir = request.app["static_dir"]
    return web.FileResponse(f"{static_dir}/index.html")


async def _handle_websocket(request: web.Request) -> web.WebSocketResponse:
    """Handle WebSocket connections.

    Manages Web commands, message parsing, and status broadcast.
    """
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    request.app["websockets"].add(ws)

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
        request.app["websockets"].discard(ws)
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
    runner = web.AppRunner(app, shutdown_timeout=WEB_SHUTDOWN_TIMEOUT_S)
    await runner.setup()

    site = web.TCPSite(runner, host=host, port=port)
    await site.start()

    logger.info("Web server started on %s:%s", host, port)

    return runner
