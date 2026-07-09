from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Optional

from aiohttp import WSMsgType, web

from robot_follow.ao.web.protocol import frame_message, parse_client_message


def frontend_dist_dir() -> Path:
    return Path(__file__).resolve().parent / "frontend" / "dist"


class VisualizerHub:
    def __init__(self):
        self._latest = None
        self._clients = set()
        self._client_messages = []

    def latest(self):
        if self._latest is None:
            return None
        return dict(self._latest)

    def record_client_message(self, message: dict) -> None:
        self._client_messages.append(dict(message))

    def drain_client_messages(self) -> list:
        messages = list(self._client_messages)
        self._client_messages.clear()
        return messages

    def update_frame(self, payload: dict) -> None:
        self._latest = dict(payload)
        message = json.dumps(frame_message(payload))
        dead = []
        for ws in list(self._clients):
            if ws.closed:
                dead.append(ws)
                continue
            asyncio.create_task(ws.send_str(message))
        for ws in dead:
            self._clients.discard(ws)

    async def websocket_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        self._clients.add(ws)
        if self._latest is not None:
            await ws.send_str(json.dumps(frame_message(self._latest)))
        try:
            async for msg in ws:
                if msg.type is WSMsgType.TEXT:
                    try:
                        self.record_client_message(parse_client_message(msg.data))
                    except ValueError as exc:
                        await ws.send_str(json.dumps({"type": "error", "payload": {"message": str(exc)}}))
                elif msg.type is WSMsgType.ERROR:
                    break
        finally:
            self._clients.discard(ws)
        return ws


async def latest_handler(request):
    hub = request.app["hub"]
    payload = hub.latest()
    if payload is None:
        return web.json_response({"status": "waiting_for_frame"})
    return web.json_response(payload)


async def index_handler(request):
    static_dir = request.app["static_dir"]
    index = static_dir / "index.html"
    if not index.exists():
        return web.Response(
            status=503,
            text="frontend has not been built; expected src/robot_follow/ao/web/frontend/dist/index.html",
        )
    return web.FileResponse(index)


def create_app(*, hub: VisualizerHub, static_dir: Optional[Path] = None) -> web.Application:
    app = web.Application()
    app["hub"] = hub
    app["static_dir"] = frontend_dist_dir() if static_dir is None else Path(static_dir)
    app.router.add_get("/", index_handler)
    app.router.add_get("/api/latest", latest_handler)
    app.router.add_get("/ws", hub.websocket_handler)
    if app["static_dir"].exists() and (app["static_dir"] / "assets").exists():
        app.router.add_static("/assets", app["static_dir"] / "assets")
    return app
