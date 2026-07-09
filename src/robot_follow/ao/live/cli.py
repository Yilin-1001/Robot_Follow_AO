"""CLI entrypoint for robot-follow-ao-radar."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_follow.ao.avoidance.planner import AvoidancePlanner
from robot_follow.ao.config import load_ao_radar_config
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.live.radar_service_manager import RadarServiceManager
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core import FollowConfig, FollowController, FollowPipeline, SafetyConfig, SafetyManager, TargetFilter
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.real.g1.bridge import G1MotionBridge


def build_runtime_from_config(config_path: Path) -> AORadarRuntime:
    """Build an AO radar runtime from a config file.

    Raises
    ------
    ValueError
        If real G1 output is requested without explicit confirmation.
    """
    config = load_ao_radar_config(config_path)
    if (
        config.output.mode == "g1"
        and config.output.require_real_g1_confirmation
        and not config.output.confirm_real_g1
    ):
        raise ValueError("real G1 output requires ao_radar.output.confirm_real_g1: true")

    alx_source = ALXLiveSource(port=config.alx.port, baudrate=config.alx.baudrate)
    pipeline = FollowPipeline(
        target_filter=TargetFilter(),
        safety=SafetyManager(SafetyConfig()),
        controller=FollowController(FollowConfig()),
    )
    radar_manager = RadarServiceManager(config.radar)
    cloud_source = FastLIOPointCloudSource()
    from robot_follow.ao.radar.pipeline import RadarPipeline

    radar_pipeline = RadarPipeline()
    planner = AvoidancePlanner()
    bridge = G1MotionBridge()
    return AORadarRuntime(
        config=config,
        alx_source=alx_source,
        pipeline=pipeline,
        radar_manager=radar_manager,
        cloud_source=cloud_source,
        radar_pipeline=radar_pipeline,
        planner=planner,
        bridge=bridge,
    )


def main(argv: list[str] | None = None) -> int:
    """Main entrypoint for robot-follow-ao-radar CLI."""
    parser = argparse.ArgumentParser(description="Robot Follow AO Radar Runtime")
    parser.add_argument("--config", type=Path, default=Path("configs/jetson_g1_safe.yaml"))
    args = parser.parse_args(argv)
    runtime = build_runtime_from_config(args.config)
    runtime.alx_source.start()
    runtime.cloud_source.start()
    period_s = 1.0 / runtime.config.latency.ao_output_loop_hz
    try:
        while True:
            started = time.monotonic()
            runtime.tick()
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period_s - elapsed))
    finally:
        runtime.cloud_source.stop()
        runtime.alx_source.stop()
        runtime.radar_manager.stop()


if __name__ == "__main__":
    raise SystemExit(main())
