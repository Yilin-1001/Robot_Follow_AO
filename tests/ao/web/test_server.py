from __future__ import annotations

import asyncio
import json

from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from robot_follow.ao.web.server import VisualizerHub, create_app, frontend_dist_dir


def test_hub_keeps_latest_payload_and_client_messages():
    async def scenario():
        hub = VisualizerHub()
        assert hub.latest() is None

        hub.update_frame({"timestamp": 1.0, "points": []})
        assert hub.latest() == {"timestamp": 1.0, "points": []}

        hub.record_client_message({"type": "select_target", "x": 1.2, "y": -0.3})
        assert hub.drain_client_messages() == [{"type": "select_target", "x": 1.2, "y": -0.3}]

    asyncio.run(scenario())


def test_api_latest_returns_waiting_before_first_frame():
    async def scenario():
        hub = VisualizerHub()
        app = create_app(hub=hub, static_dir=frontend_dist_dir())
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/latest")
            assert resp.status == 200
            assert await resp.json() == {"status": "waiting_for_frame"}
        finally:
            await client.close()

    asyncio.run(scenario())


def test_websocket_receives_frames_and_records_target_selection():
    async def scenario():
        hub = VisualizerHub()
        app = create_app(hub=hub, static_dir=frontend_dist_dir())
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            ws = await client.ws_connect("/ws")
            hub.update_frame({"timestamp": 2.0, "points": [[0.3, 0.0, 0.2]]})
            msg = await ws.receive(timeout=1.0)
            assert msg.type is WSMsgType.TEXT
            data = json.loads(msg.data)
            assert data["type"] == "frame"
            assert data["payload"]["timestamp"] == 2.0

            await ws.send_str('{"type":"select_target","x":1.2,"y":-0.3}')
            await asyncio.sleep(0.05)
            assert hub.drain_client_messages() == [{"type": "select_target", "x": 1.2, "y": -0.3}]
            await ws.close()
        finally:
            await client.close()

    asyncio.run(scenario())
