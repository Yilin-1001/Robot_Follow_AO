"""AO radar CLI entrypoint with real G1 guard.

Provides the `robot-follow-ao-radar` console script that:
1. Loads AO radar config from YAML
2. Validates real G1 output confirmation
3. Builds and runs the AO radar runtime
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from robot_follow.ao.avoidance.planner import (
    AvoidancePlanner,
    DefaultAvoidanceConfig,
    DefaultPlannerConfig,
)
from robot_follow.ao.config import load_ao_radar_configs
from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, timestamped_log_path
from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource
from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig
from robot_follow.ao.live.radar_service_manager import RadarServiceManager
from robot_follow.ao.live.runtime import AORadarRuntime
from robot_follow.core import FollowPipeline, SafetyManager, TargetFilter
from robot_follow.core.types import MotionCommand
from robot_follow.core.follow_controller import FollowController, FollowConfig
from robot_follow.core.safety import SafetyConfig
from robot_follow.real.alx_source import ALXLiveSource
from robot_follow.real.g1 import G1BridgeConfig
from robot_follow.real.g1.bridge import G1MotionBridge


class MockMotionBridge:
    """Mock output for full AO live dry-runs without touching the G1 SDK."""

    def __init__(self) -> None:
        self.commands: list[MotionCommand] = []
        self.last_sent_omega: float | None = None

    def init(self) -> None:
        print("[AO_OUTPUT] mode=mock initialized", flush=True)

    def dispatch(self, command: MotionCommand) -> None:
        self.commands.append(command)
        self.last_sent_omega = command.omega

    def tick(self, *, now: float) -> None:
        return None

    def close(self) -> None:
        print("[AO_OUTPUT] mode=mock closed", flush=True)


class WristbandStatusPrinter:
    """Render wristband state transitions from the AO main thread.

    The serial reader only updates state, so a reader thread cannot keep
    writing to the terminal while the main thread is blocked or shutting down.
    """

    def __init__(self) -> None:
        self._last_enabled: bool | None = None

    def emit_if_changed(self, source) -> None:
        state = source.wristband_state()
        enabled = getattr(state, "enabled", None)
        if enabled is None or enabled == self._last_enabled:
            return
        self._last_enabled = enabled
        print(f"[WRISTBAND] {'ON' if enabled else 'OFF'}", flush=True)


def build_runtime_from_config(config_paths: Path | list[Path]) -> AORadarRuntime:
    """Build the AO radar runtime from a config file.

    Raises ValueError if real G1 output is requested but not confirmed.
    """
    paths = [config_paths] if isinstance(config_paths, Path) else config_paths
    config = load_ao_radar_configs(paths)
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
        target_filter=TargetFilter(alpha=config.filter.alpha),
        safety=SafetyManager(
            SafetyConfig(
                data_timeout_s=config.safety.data_timeout_s,
                min_safe_distance_m=config.safety.min_safe_distance_m,
                max_distance_jump_m=config.safety.max_distance_jump_m,
                max_angle_jump_deg=config.safety.max_angle_jump_deg,
            )
        ),
        controller=FollowController(
            FollowConfig(
                target_distance_m=config.follow.target_distance_m,
                min_safe_distance_m=config.follow.min_safe_distance_m,
                hold_tolerance_m=config.follow.hold_tolerance_m,
                max_forward_vx=config.follow.max_forward_vx,
                max_backward_vx=config.follow.max_backward_vx,
                max_follow_distance_m=config.follow.max_follow_distance_m,
                max_omega=config.follow.max_omega,
                angle_deadzone_deg=config.follow.angle_deadzone_deg,
                large_angle_deg=config.follow.large_angle_deg,
                distance_gain_forward=config.follow.distance_gain_forward,
                distance_gain_backward=config.follow.distance_gain_backward,
                angle_gain=config.follow.angle_gain,
            )
        ),
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
    planner = AvoidancePlanner(
        DefaultPlannerConfig(
            avoidance=replace(
                DefaultAvoidanceConfig(),
                lateral_vy_mps=config.avoidance.lateral_vy_mps,
                forward_vx_when_avoiding_mps=config.avoidance.forward_vx_when_avoiding_mps,
                max_vx=config.avoidance.max_vx,
                max_vy=config.avoidance.max_vy,
                max_omega=config.avoidance.max_omega,
            )
        )
    )
    bridge = (
        MockMotionBridge()
        if config.output.mode == "mock"
        else G1MotionBridge(
            network_interface=config.output.network_interface,
            config=G1BridgeConfig(
                max_abs_vx=config.g1.max_vx,
                max_abs_vy=config.g1.max_vy,
                max_abs_omega=config.g1.max_omega,
                yaw_sign=config.g1.yaw_sign,
                sdk_timeout_s=config.g1.sdk_timeout_s,
                sdk_timeout_code=config.g1.sdk_timeout_code,
                sdk_timeout_retry_interval_s=config.g1.sdk_timeout_retry_interval_s,
                max_consecutive_sdk_timeouts=config.g1.max_consecutive_sdk_timeouts,
                command_timeout_s=config.g1.command_timeout_s,
            ),
        )
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
        action="append",
        default=None,
        help=(
            "YAML config path; may be specified more than once. "
            "Later files override earlier values."
        ),
    )
    args = parser.parse_args(argv)

    config_paths = args.config or [Path("configs/jetson_g1_ao_radar.yaml")]
    runtime = build_runtime_from_config(config_paths)
    logger = _build_logger(runtime.config)
    last_summary_at: float | None = None
    wristband_status_printer = WristbandStatusPrinter()
    try:
        runtime.alx_source.start()
        runtime.cloud_source.start()
        _init_bridge(runtime.bridge)

        timing = LoopTiming(
            config=LoopTimingConfig(target_hz=runtime.config.latency.ao_output_loop_hz),
            clock=time.monotonic,
            sleeper=time.sleep,
        )
        while True:
            started = timing.mark_tick_start()
            wristband_status_printer.emit_if_changed(runtime.alx_source)
            runtime.tick()
            if logger is not None and runtime.last_frame is not None:
                logger.write(runtime.last_frame.to_log_row())
            last_summary_at = _maybe_print_summary(
                runtime,
                last_summary_at=last_summary_at,
                now=started,
            )
            timing.sleep_until_next_tick(started_at=started)
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
    raw_azimuth = "-" if frame.raw_azimuth_deg is None else f"{frame.raw_azimuth_deg:.1f}"
    filtered_azimuth = (
        "-" if frame.filtered_azimuth_deg is None else f"{frame.filtered_azimuth_deg:.1f}"
    )
    g1_omega = "-" if frame.g1_sent_omega is None else f"{frame.g1_sent_omega:.3f}"
    distance = "-" if frame.distance_m is None else f"{frame.distance_m:.3f}"
    tag_status = "-" if frame.tag_status is None else str(frame.tag_status)
    # 新增：延迟追踪信息
    raw_time = "-" if frame.raw_command_generated_at_ms is None else f"{frame.raw_command_generated_at_ms:.1f}"
    safe_time = "-" if frame.safe_command_generated_at_ms is None else f"{frame.safe_command_generated_at_ms:.1f}"
    g1_time = "-" if frame.g1_response_at_ms is None else f"{frame.g1_response_at_ms:.1f}"
    total_lat = "-" if frame.total_latency_ms is None else f"{frame.total_latency_ms:.1f}"

    print(
        "[AO_FRAME] "
        f"raw={raw_summary} "
        f"safe={safe.kind} vx={safe.vx:.3f} vy={safe.vy:.3f} "
        f"omega={safe.omega:.3f} reason={safe.reason} "
        f"radar={frame.radar_state} wristband={frame.wristband_gate_state} "
        f"alx_age_ms={frame.alx_age_ms:.0f} cloud_age_ms={frame.cloud_age_ms:.0f} "
        f"cmd_age_ms={frame.command_age_ms:.0f} ao_ms={frame.ao_process_ms:.1f} "
        f"dispatch_ms={frame.g1_dispatch_ms:.1f} "
        f"raw_azimuth_deg={raw_azimuth} filtered_azimuth_deg={filtered_azimuth} "
        f"distance_m={distance} tag_status={tag_status} "
        f"g1_sent_omega={g1_omega} "
        f"latency: raw@{raw_time}ms safe@{safe_time}ms g1@{g1_time}ms total={total_lat}ms",
        flush=True,
    )
    return now
