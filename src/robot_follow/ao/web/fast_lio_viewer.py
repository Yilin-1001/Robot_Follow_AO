from __future__ import annotations

import argparse
import asyncio
import time

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.avoidance.planner import StatefulAvoidancePlanner
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.web.frames import ConstantCommandSource, build_visualization_payload


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run FAST-LIO Web visualizer on the backpack computer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--max-points", type=int, default=2000)
    parser.add_argument("--topic", default="/cloud_registered_body")
    parser.add_argument("--expected-frame-id", default="body")
    parser.add_argument("--allow-non-body-frame", action="store_true")
    return parser.parse_args(argv)


async def produce_fast_lio_frames(
    *,
    hub,
    pointcloud_source,
    command_source,
    config,
    hz: float,
    max_points: int,
):
    planner = (
        StatefulAvoidancePlanner(config)
        if getattr(config.avoidance, "stateful_enabled", False)
        else None
    )
    period = 1.0 / float(hz)
    while True:
        frame = pointcloud_source.latest()
        if frame is not None:
            payload = build_visualization_payload(
                frame,
                command_source=command_source,
                config=config,
                planner=planner,
                now_s=time.monotonic(),
                max_points=max_points,
            )
            hub.update_frame(payload)
        await asyncio.sleep(period)


async def run_fast_lio_viewer_async(
    *,
    host: str,
    port: int,
    hz: float,
    max_points: int,
    topic: str,
    expected_frame_id: str,
    allow_non_body_frame: bool,
):
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("ROS2 rclpy is required for FAST-LIO Web visualization") from exc
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError("aiohttp is required for FAST-LIO Web visualization") from exc
    from robot_follow.ao.web.server import VisualizerHub, create_app

    config = AppConfig.default()
    hub = VisualizerHub()
    app = create_app(hub=hub)
    command_source = ConstantCommandSource(source="scripted", vx=0.18)
    pointcloud_source = FastLIOPointCloudSource(
        topic=topic,
        expected_frame_id=expected_frame_id,
        require_body_frame=not allow_non_body_frame,
    )
    pointcloud_source.start()
    asyncio.create_task(
        produce_fast_lio_frames(
            hub=hub,
            pointcloud_source=pointcloud_source,
            command_source=command_source,
            config=config,
            hz=hz,
            max_points=max_points,
        )
    )

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    print(f"FAST-LIO Web visualizer listening on http://{host}:{port}", flush=True)
    print(f"Subscribing to {topic}", flush=True)
    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        pointcloud_source.stop()


def main(argv=None):
    args = parse_args(argv)
    asyncio.run(
        run_fast_lio_viewer_async(
            host=args.host,
            port=args.port,
            hz=args.hz,
            max_points=args.max_points,
            topic=args.topic,
            expected_frame_id=args.expected_frame_id,
            allow_non_body_frame=args.allow_non_body_frame,
        )
    )


if __name__ == "__main__":
    main()
