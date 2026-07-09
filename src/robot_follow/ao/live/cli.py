"""AO radar CLI entrypoint with real G1 guard.

Provides the `robot-follow-ao-radar` console script that:
1. Loads AO radar config from YAML
2. Validates real G1 output confirmation
3. Builds and runs the AO radar runtime
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_follow.ao.avoidance.planner import AvoidancePlanner
from robot_follow.ao.config import load_ao_radar_config
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.live.radar_service_manager import RadarServiceManager
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core import FollowPipeline, SafetyManager, TargetFilter
from robot_follow.core.follow_controller import FollowController, FollowConfig
from robot_follow.core.safety import SafetyConfig
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.real.g1.bridge import G1MotionBridge


def build_runtime_from_config(config_path: Path) -> AORadarRuntime:
    """Build the AO radar runtime from a config file.

    Raises ValueError if real G1 output is requested but not confirmed.
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

    # Use the AO radar pipeline directly (FastLIOPointCloudSource now
    # returns AO PointCloudFrame with np.ndarray)
    from robot_follow.ao.app.config import AppConfig
    from robot_follow.ao.radar.pipeline import process_radar_frame

    app_config = AppConfig.default()

    class RadarPipelineAdapter:
        """Adapter to process FastLIO point clouds through AO radar pipeline."""

        def __init__(self, app_config: AppConfig):
            self._config = app_config

        def process(self, cloud):
            """Process a PointCloudFrame and return an ObstacleReport."""
            if cloud is None:
                from robot_follow.ao.core.types import ObstacleReport
                return ObstacleReport.invalid(time.monotonic(), reason="cloud_missing")

            result = process_radar_frame(
                cloud,
                config=self._config,
                now_s=time.monotonic(),
            )
            return result.report

    radar_pipeline = RadarPipelineAdapter(app_config)
    planner = AvoidancePlanner()
    bridge = G1MotionBridge(network_interface=config.output.network_interface)

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
    parser = argparse.ArgumentParser(
        prog="robot-follow-ao-radar",
        description="Run Robot Follow AO radar live runtime",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/jetson_g1_safe.yaml"),
        help="Path to YAML config file",
    )
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
            sleep_time = max(0.0, period_s - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        runtime.cloud_source.stop()
        runtime.alx_source.stop()
        runtime.radar_manager.stop()
        runtime.bridge.close()

    return 0