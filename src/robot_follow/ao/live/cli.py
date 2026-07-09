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
from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, timestamped_log_path
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.live.radar_service_manager import RadarServiceManager
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core import FollowPipeline, SafetyManager, TargetFilter
from robot_follow.core.types import MotionCommand
from robot_follow.core.follow_controller import FollowController, FollowConfig
from robot_follow.core.safety import SafetyConfig
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.real.g1.bridge import G1MotionBridge


class MockMotionBridge:
    """Mock output for full AO live dry-runs without touching the G1 SDK."""

    def __init__(self) -> None:
        self.commands: list[MotionCommand] = []

    def init(self) -> None:
        print("[AO_OUTPUT] mode=mock initialized", flush=True)

    def dispatch(self, command: MotionCommand) -> None:
        self.commands.append(command)

    def tick(self, *, now: float) -> None:
        return None

    def close(self) -> None:
        print("[AO_OUTPUT] mode=mock closed", flush=True)


def build_runtime_from_config(config_path: Path) -> AORadarRuntime:
    """Build the AO radar runtime from a config file.

    Raises ValueError if real G1 output is requested but not confirmed.
    """
    config = load_ao_radar_config(config_path)
    if config.output.mode not in {"mock", "g1"}:
        raise ValueError("ao_radar.output.mode must be 'mock' or 'g1'")
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
    bridge = (
        MockMotionBridge()
        if config.output.mode == "mock"
        else G1MotionBridge(network_interface=config.output.network_interface)
    )

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
    logger = _build_logger(runtime.config)
    last_summary_at: float | None = None
    try:
        runtime.alx_source.start()
        runtime.cloud_source.start()
        _init_bridge(runtime.bridge)

        period_s = 1.0 / runtime.config.latency.ao_output_loop_hz
        while True:
            started = time.monotonic()
            runtime.tick()
            if logger is not None and runtime.last_frame is not None:
                logger.write(runtime.last_frame.to_log_row())
            last_summary_at = _maybe_print_summary(
                runtime,
                last_summary_at=last_summary_at,
                now=started,
            )
            elapsed = time.monotonic() - started
            sleep_time = max(0.0, period_s - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
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
                        if logger is not None:
                            logger.close()

    return 0


def _build_logger(config):
    if not config.logging.enabled:
        return None
    path = Path(config.logging.csv_path)
    if config.logging.timestamped_runs:
        path = timestamped_log_path(path, time_format=config.logging.filename_time_format)
    print(f"[AO_LOG] csv={path}", flush=True)
    return AORadarCSVLogger(path)


def _init_bridge(bridge) -> None:
    init = getattr(bridge, "init", None)
    if callable(init):
        init()


def _maybe_print_summary(
    runtime: AORadarRuntime,
    *,
    last_summary_at: float | None,
    now: float,
) -> float | None:
    interval = runtime.config.logging.terminal_summary_interval_s
    if interval <= 0.0:
        return last_summary_at
    if last_summary_at is not None and now - last_summary_at < interval:
        return last_summary_at
    frame = runtime.last_frame
    if frame is None:
        return last_summary_at
    raw = frame.raw_command
    raw_summary = "none" if raw is None else (
        f"{raw.kind} vx={raw.vx:.3f} vy={raw.vy:.3f} "
        f"omega={raw.omega:.3f} reason={raw.reason}"
    )
    safe = frame.command
    print(
        "[AO_FRAME] "
        f"raw={raw_summary} "
        f"safe={safe.kind} vx={safe.vx:.3f} vy={safe.vy:.3f} "
        f"omega={safe.omega:.3f} reason={safe.reason} "
        f"radar={frame.radar_state} wristband={frame.wristband_gate_state} "
        f"alx_age_ms={frame.alx_age_ms:.0f} cloud_age_ms={frame.cloud_age_ms:.0f} "
        f"cmd_age_ms={frame.command_age_ms:.0f} ao_ms={frame.ao_process_ms:.1f} "
        f"dispatch_ms={frame.g1_dispatch_ms:.1f}",
        flush=True,
    )
    return now
