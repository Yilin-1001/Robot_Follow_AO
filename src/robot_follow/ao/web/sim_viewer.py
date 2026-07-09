from __future__ import annotations

import argparse
import asyncio
import math

import numpy as np

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.core.types import PointCloudFrame
from robot_follow.ao.web.frames import ConstantCommandSource, build_visualization_payload


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run simulated Robot Follow AO Web visualizer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--max-points", type=int, default=2000)
    return parser.parse_args(argv)


def moving_front_obstacle_frames(*, start_s: float, count: int, hz: float):
    period = 1.0 / float(hz)
    for i in range(count):
        timestamp = round(start_s + i * period, 10)
        x = 1.4 + 0.2 * math.sin(i * 0.15)
        ys = np.linspace(-0.25, 0.25, 16)
        points = np.column_stack((np.full_like(ys, x), ys, np.full_like(ys, 0.2)))
        yield PointCloudFrame(
            timestamp=timestamp,
            frame_id="body",
            points_xyz=points,
            valid=True,
            empty_is_clear=False,
            reason="simulated_front_obstacle",
            received_at_s=timestamp,
        )


async def run_sim_viewer_async(*, host: str, port: int, hz: float, max_points: int):
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError("aiohttp is required for AO Web visualization") from exc
    from robot_follow.ao.web.server import VisualizerHub, create_app

    config = AppConfig.default()
    hub = VisualizerHub()
    app = create_app(hub=hub)
    command_source = ConstantCommandSource(source="scripted", vx=0.18)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    print(f"AO Web simulation listening on http://{host}:{port}", flush=True)

    frame_index = 0
    period = 1.0 / float(hz)
    try:
        while True:
            frame = next(moving_front_obstacle_frames(start_s=frame_index * period, count=1, hz=hz))
            payload = build_visualization_payload(
                frame,
                command_source=command_source,
                config=config,
                now_s=frame.timestamp,
                max_points=max_points,
            )
            hub.update_frame(payload)
            frame_index += 1
            await asyncio.sleep(period)
    finally:
        await runner.cleanup()


def main(argv=None):
    args = parse_args(argv)
    asyncio.run(
        run_sim_viewer_async(
            host=args.host,
            port=args.port,
            hz=args.hz,
            max_points=args.max_points,
        )
    )


if __name__ == "__main__":
    main()
