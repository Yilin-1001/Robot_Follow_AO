"""AO radar runtime with integrated web visualization.

Provides the `robot-follow-ao-radar-web` console script that combines:
1. AO radar runtime with wristband-gated radar lifecycle
2. Web server for real-time point cloud visualization

The web server acts as an observer - it does NOT control radar start/stop.
Wristband ON/OFF controls the radar lifecycle (livox + FAST-LIO processes).
"""

from __future__ import annotations

import argparse
import asyncio
import threading
import time
from pathlib import Path

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.avoidance.planner import StatefulAvoidancePlanner
from robot_follow.ao.config import load_ao_radar_config
from robot_follow.ao.live.cli import build_runtime_from_config
from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig
from robot_follow.ao.web.frames import ConstantCommandSource, build_visualization_payload
from robot_follow.ao.web.server import VisualizerHub, create_app


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog="robot-follow-ao-radar-web",
        description="Run AO radar runtime with web visualization",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/jetson_g1_ao_radar.yaml"),
        help="Path to YAML config file",
    )
    parser.add_argument(
        "--web-host",
        default="0.0.0.0",
        help="Web server host (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=8080,
        help="Web server port (default: 8080)",
    )
    parser.add_argument(
        "--web-hz",
        type=float,
        default=10.0,
        help="Web visualization update rate in Hz (default: 10.0)",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=2000,
        help="Maximum points to send to web client (default: 2000)",
    )
    return parser.parse_args(argv)


def _run_web_server_in_thread(hub: VisualizerHub, host: str, port: int) -> threading.Thread:
    """Start aiohttp web server in a background thread."""
    from aiohttp import web

    loop = asyncio.new_event_loop()
    app = create_app(hub=hub)
    runner = web.AppRunner(app)

    async def start_server():
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        print(f"[AO_WEB] Server listening on http://{host}:{port}", flush=True)
        # Keep the event loop running
        while True:
            await asyncio.sleep(3600)

    def run_loop():
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(start_server())
        finally:
            loop.run_until_complete(runner.cleanup())
            loop.close()

    thread = threading.Thread(target=run_loop, daemon=True, name="ao-web-server")
    thread.start()
    return thread


def _should_update_web(
    *,
    last_web_update_at: float | None,
    started_at: float,
    web_period_s: float,
) -> bool:
    """Return True when a web payload should be published this tick.

    Web publishing is throttled to ``web_period_s`` (driven by ``--web-hz``)
    and is independent of the AO tick cadence: AO ticks still run at the full
    configured rate; only the web payload update is gated here.
    """
    return last_web_update_at is None or (started_at - last_web_update_at) >= web_period_s


def _update_web_hub(
    runtime,
    hub: VisualizerHub,
    app_config: AppConfig,
    *,
    max_points: int,
    now_s: float,
) -> None:
    """Update web hub with latest point cloud from runtime.

    Only sends data when wristband is ON and radar is running.
    Otherwise, sends status message to inform user why no data.
    """
    from robot_follow.ao.web.protocol import status_message

    # Check wristband state
    wristband_state = runtime._get_wristband_state()
    if wristband_state is False:
        hub.update_frame(status_message("wristband_off", level="warning"))
        return
    if wristband_state is None:
        hub.update_frame(status_message("wristband_unknown", level="info"))
        return

    # Check if radar is running
    radar_state = runtime._radar_state()
    if radar_state != "running":
        hub.update_frame(status_message(f"radar_{radar_state}", level="info"))
        return

    # Get point cloud from runtime's cloud_source
    cloud = runtime.cloud_source.latest()
    if cloud is None:
        hub.update_frame(status_message("radar_not_ready", level="info"))
        return

    # Build visualization payload
    # Use a constant command source for visualization (not from runtime)
    command_source = ConstantCommandSource(source="web_viewer", vx=0.18)

    # Get planner from runtime or create one
    planner = getattr(runtime, "_web_planner", None)
    if planner is None:
        planner = (
            StatefulAvoidancePlanner(app_config)
            if getattr(app_config.avoidance, "stateful_enabled", False)
            else None
        )
        runtime._web_planner = planner

    try:
        payload = build_visualization_payload(
            cloud,
            command_source=command_source,
            config=app_config,
            planner=planner,
            now_s=now_s,
            max_points=max_points,
        )
        hub.update_frame(payload)
    except Exception as exc:
        # Don't crash the runtime loop on visualization errors
        print(f"[AO_WEB] Error building payload: {exc}", flush=True)


def main(argv=None) -> int:
    """Main entrypoint for robot-follow-ao-radar-web CLI."""
    args = parse_args(argv)

    # Build runtime from config (same as cli.py)
    runtime = build_runtime_from_config(args.config)
    app_config = AppConfig.default()
    hub = VisualizerHub()

    # Start web server in background thread
    web_thread = _run_web_server_in_thread(
        hub=hub,
        host=args.web_host,
        port=args.web_port,
    )

    # Start runtime components
    runtime.alx_source.start()
    runtime.cloud_source.start()
    _init_bridge(runtime.bridge)

    print(f"[AO_RADAR] Runtime started with {runtime.config.latency.ao_output_loop_hz:.1f}Hz loop", flush=True)

    last_web_update_at: float | None = None
    web_period_s = 1.0 / args.web_hz

    try:
        timing = LoopTiming(
            config=LoopTimingConfig(target_hz=runtime.config.latency.ao_output_loop_hz),
            clock=time.monotonic,
            sleeper=time.sleep,
        )
        while True:
            started = timing.mark_tick_start()

            # Run runtime tick
            runtime.tick()

            # Update web hub at lower rate than runtime. The AO tick cadence is
            # driven by ``timing`` above and MUST NOT be reduced by --web-hz;
            # this throttle only gates web payload publishing.
            if _should_update_web(
                last_web_update_at=last_web_update_at,
                started_at=started,
                web_period_s=web_period_s,
            ):
                _update_web_hub(
                    runtime,
                    hub,
                    app_config,
                    max_points=args.max_points,
                    now_s=started,
                )
                last_web_update_at = started

            timing.sleep_until_next_tick(started_at=started)
    finally:
        print("[AO_RADAR] Shutting down...", flush=True)
        try:
            runtime.cloud_source.stop()
        finally:
            try:
                runtime.alx_source.stop()
            finally:
                try:
                    runtime.radar_manager.stop()
                finally:
                    try:
                        runtime.bridge.close()
                    finally:
                        print("[AO_WEB] Web server stopped", flush=True)

    return 0


def _init_bridge(bridge) -> None:
    """Initialize the motion bridge if it has an init method."""
    init = getattr(bridge, "init", None)
    if callable(init):
        init()


if __name__ == "__main__":
    main()
