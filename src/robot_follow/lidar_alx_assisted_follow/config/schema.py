"""Strict configuration schema for LiDAR + ALX Assisted Follow."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np


def _validate_finite(value: float, name: str) -> None:
    """Validate that a value is finite."""
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")


def _validate_positive(value: float, name: str) -> None:
    """Validate that a value is positive."""
    _validate_finite(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")


def _validate_non_negative(value: float, name: str) -> None:
    """Validate that a value is non-negative."""
    _validate_finite(value, name)
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")


def _validate_in_range(
    value: float, name: str, min_val: float, max_val: float
) -> None:
    """Validate that a value is in a range."""
    _validate_finite(value, name)
    if not (min_val <= value <= max_val):
        raise ValueError(f"{name} must be in [{min_val}, {max_val}], got {value}")


@dataclass(frozen=True)
class ALXConfig:
    """ALX serial and distance configuration."""

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    stale_timeout_s: float = 0.50
    cloud_max_skew_s: float = 0.15
    min_distance_m: float = 0.10
    max_distance_m: float = 10.0

    def __post_init__(self) -> None:
        _validate_positive(self.stale_timeout_s, "alx.stale_timeout_s")
        _validate_positive(self.cloud_max_skew_s, "alx.cloud_max_skew_s")
        _validate_non_negative(self.min_distance_m, "alx.min_distance_m")
        _validate_positive(self.max_distance_m, "alx.max_distance_m")
        if self.min_distance_m >= self.max_distance_m:
            raise ValueError(
                f"alx.min_distance_m ({self.min_distance_m}) must be < "
                f"alx.max_distance_m ({self.max_distance_m})"
            )


@dataclass(frozen=True)
class ALXRosConfig:
    """ROS2 publication settings for the filtered ALX body-frame position."""

    topic: str = "/alx/position"
    wristband_topic: str = "/alx/wristband_state"
    frame_id: str = "body"
    poll_hz: float = 200.0
    wristband_publish_hz: float = 10.0
    wristband_timeout_s: float = 0.50
    kalman_process_noise: float = 0.05
    kalman_measurement_noise: float = 0.25
    kalman_max_interval_s: float = 0.20

    def __post_init__(self) -> None:
        if not self.topic.startswith("/"):
            raise ValueError(f"alx_ros.topic must be absolute, got {self.topic!r}")
        if not self.wristband_topic.startswith("/"):
            raise ValueError(
                "alx_ros.wristband_topic must be absolute, got "
                f"{self.wristband_topic!r}"
            )
        if self.wristband_topic == self.topic:
            raise ValueError("alx_ros.wristband_topic must differ from alx_ros.topic")
        if self.frame_id not in ("body", "base_link"):
            raise ValueError(
                "alx_ros.frame_id must be 'body' or 'base_link', got "
                f"{self.frame_id!r}"
            )
        _validate_positive(self.poll_hz, "alx_ros.poll_hz")
        _validate_positive(self.wristband_publish_hz, "alx_ros.wristband_publish_hz")
        _validate_positive(self.wristband_timeout_s, "alx_ros.wristband_timeout_s")
        _validate_positive(self.kalman_process_noise, "alx_ros.kalman_process_noise")
        _validate_positive(self.kalman_measurement_noise, "alx_ros.kalman_measurement_noise")
        _validate_positive(self.kalman_max_interval_s, "alx_ros.kalman_max_interval_s")


@dataclass(frozen=True)
class SensorsConfig:
    """Sensor topic, input-frame convention, and optional static frame transform."""

    topic: str = "/cloud_registered_body"
    expected_frame_id: str = "body"
    cloud_y_sign: int = 1
    output_frame_id: str = "body"
    apply_body_to_output_transform: bool = False
    body_to_output_roll_deg: float = 0.0
    body_to_output_pitch_deg: float = 0.0
    body_to_output_yaw_deg: float = 0.0
    body_to_output_x_m: float = 0.0
    body_to_output_y_m: float = 0.0
    body_to_output_z_m: float = 0.0
    static_tf_start_command: str = ""
    max_cloud_age_s: float = 0.25
    ready_timeout_s: float = 10.0
    stop_timeout_s: float = 3.0
    livox_start_command: str = ""
    fast_lio_start_command: str = ""

    def __post_init__(self) -> None:
        if self.expected_frame_id != "body":
            raise ValueError(
                f"sensors.expected_frame_id must be 'body', got '{self.expected_frame_id}'"
            )
        if self.cloud_y_sign not in (1, -1):
            raise ValueError(
                "sensors.cloud_y_sign must be +1 or -1, got "
                f"{self.cloud_y_sign}"
            )
        if not self.output_frame_id:
            raise ValueError("sensors.output_frame_id must be non-empty")
        for value, name in (
            (self.body_to_output_roll_deg, "sensors.body_to_output_roll_deg"),
            (self.body_to_output_pitch_deg, "sensors.body_to_output_pitch_deg"),
            (self.body_to_output_yaw_deg, "sensors.body_to_output_yaw_deg"),
            (self.body_to_output_x_m, "sensors.body_to_output_x_m"),
            (self.body_to_output_y_m, "sensors.body_to_output_y_m"),
            (self.body_to_output_z_m, "sensors.body_to_output_z_m"),
        ):
            _validate_finite(value, name)
        _validate_positive(self.max_cloud_age_s, "sensors.max_cloud_age_s")
        _validate_positive(self.ready_timeout_s, "sensors.ready_timeout_s")
        _validate_positive(self.stop_timeout_s, "sensors.stop_timeout_s")


@dataclass(frozen=True)
class G1Config:
    """G1 motion limits and SDK configuration."""

    network_interface: str = "eth0"
    max_vx: float = 0.10
    max_abs_vy: float = 0.08
    max_abs_omega: float = 0.25
    yaw_sign: int = 1
    sdk_timeout_s: float = 10.0
    sdk_timeout_code: int = 3104
    sdk_timeout_retry_interval_s: float = 1.0
    max_consecutive_sdk_timeouts: int = 3
    command_timeout_s: float = 0.50

    def __post_init__(self) -> None:
        if self.yaw_sign not in (1, -1):
            raise ValueError(f"g1.yaw_sign must be +1 or -1, got {self.yaw_sign}")
        _validate_non_negative(self.max_vx, "g1.max_vx")
        _validate_non_negative(self.max_abs_vy, "g1.max_abs_vy")
        _validate_non_negative(self.max_abs_omega, "g1.max_abs_omega")
        _validate_positive(self.sdk_timeout_s, "g1.sdk_timeout_s")
        _validate_positive(self.sdk_timeout_retry_interval_s, "g1.sdk_timeout_retry_interval_s")
        if self.max_consecutive_sdk_timeouts <= 0:
            raise ValueError(
                f"g1.max_consecutive_sdk_timeouts must be positive, "
                f"got {self.max_consecutive_sdk_timeouts}"
            )
        _validate_positive(self.command_timeout_s, "g1.command_timeout_s")


@dataclass(frozen=True)
class OutputConfig:
    """Output mode configuration."""

    mode: str = "mock"

    def __post_init__(self) -> None:
        if self.mode not in ("mock", "real"):
            raise ValueError(f"output.mode must be 'mock' or 'real', got '{self.mode}'")


@dataclass(frozen=True)
class RuntimeConfig:
    """Runtime timing and capacity configuration."""

    watchdog_hz: float = 40.0
    max_strategy_age_s: float = 0.15
    max_strategy_process_ms: float = 20.0
    control_event_capacity: int = 32

    def __post_init__(self) -> None:
        _validate_positive(self.watchdog_hz, "runtime.watchdog_hz")
        _validate_positive(self.max_strategy_age_s, "runtime.max_strategy_age_s")
        _validate_positive(self.max_strategy_process_ms, "runtime.max_strategy_process_ms")
        if self.control_event_capacity <= 0:
            raise ValueError(
                f"runtime.control_event_capacity must be positive, "
                f"got {self.control_event_capacity}"
            )


@dataclass(frozen=True)
class TargetConfig:
    """Target detection and tracking configuration."""

    radial_min_m: float = 0.25
    radial_max_m: float = 5.0
    physical_height_min_m: float = 0.10
    physical_height_max_m: float = 2.00
    self_x_min_m: float = -0.35
    self_x_max_m: float = 0.25
    self_abs_y_max_m: float = 0.25
    voxel_size_m: float = 0.05
    web_pick_radius_m: float = 0.35
    gate_radial_radius_m: float = 0.45
    gate_tangential_radius_m: float = 0.35
    min_support_points: int = 8
    min_vertical_extent_m: float = 0.45
    max_horizontal_footprint_m: float = 0.90
    max_radial_mad_m: float = 0.18
    max_center_displacement_m: float = 0.35
    max_frame_interval_s: float = 0.20
    kalman_enabled: bool = True
    kalman_process_noise: float = 0.10
    kalman_measurement_noise: float = 0.05
    lost_timeout_s: float = 0.75
    desired_distance_m: float = 1.00
    stop_distance_m: float = 0.80

    def __post_init__(self) -> None:
        _validate_non_negative(self.radial_min_m, "target.radial_min_m")
        _validate_positive(self.radial_max_m, "target.radial_max_m")
        if self.radial_min_m >= self.radial_max_m:
            raise ValueError(
                f"target.radial_min_m ({self.radial_min_m}) must be < "
                f"target.radial_max_m ({self.radial_max_m})"
            )
        _validate_non_negative(self.physical_height_min_m, "target.physical_height_min_m")
        _validate_positive(self.physical_height_max_m, "target.physical_height_max_m")
        if self.physical_height_min_m >= self.physical_height_max_m:
            raise ValueError(
                f"target.physical_height_min_m ({self.physical_height_min_m}) must be < "
                f"target.physical_height_max_m ({self.physical_height_max_m})"
            )
        _validate_positive(self.voxel_size_m, "target.voxel_size_m")
        _validate_positive(self.web_pick_radius_m, "target.web_pick_radius_m")
        _validate_positive(self.gate_radial_radius_m, "target.gate_radial_radius_m")
        _validate_positive(self.gate_tangential_radius_m, "target.gate_tangential_radius_m")
        if self.min_support_points <= 0:
            raise ValueError(
                f"target.min_support_points must be positive, got {self.min_support_points}"
            )
        _validate_positive(self.min_vertical_extent_m, "target.min_vertical_extent_m")
        _validate_positive(self.max_horizontal_footprint_m, "target.max_horizontal_footprint_m")
        _validate_positive(self.max_radial_mad_m, "target.max_radial_mad_m")
        _validate_positive(self.max_center_displacement_m, "target.max_center_displacement_m")
        _validate_positive(self.max_frame_interval_s, "target.max_frame_interval_s")
        if not isinstance(self.kalman_enabled, bool):
            raise ValueError("target.kalman_enabled must be a boolean")
        _validate_positive(self.kalman_process_noise, "target.kalman_process_noise")
        _validate_positive(self.kalman_measurement_noise, "target.kalman_measurement_noise")
        _validate_positive(self.lost_timeout_s, "target.lost_timeout_s")
        _validate_positive(self.desired_distance_m, "target.desired_distance_m")
        _validate_positive(self.stop_distance_m, "target.stop_distance_m")
        if self.stop_distance_m >= self.desired_distance_m:
            raise ValueError(
                f"target.stop_distance_m ({self.stop_distance_m}) must be < "
                f"target.desired_distance_m ({self.desired_distance_m})"
            )


@dataclass(frozen=True)
class CalibrationConfig:
    """Calibration parameters (provisional for prototype)."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    azimuth_sign: int = 1
    azimuth_zero_offset_deg: float = 0.0
    lidar_height_m: float = 1.25
    alx_height_m: float = 1.10
    allow_provisional_calibration_for_prototype: bool = True

    def __post_init__(self) -> None:
        _validate_finite(self.x_m, "calibration.x_m")
        _validate_finite(self.y_m, "calibration.y_m")
        _validate_finite(self.yaw_deg, "calibration.yaw_deg")
        if self.azimuth_sign not in (1, -1):
            raise ValueError(
                f"calibration.azimuth_sign must be +1 or -1, got {self.azimuth_sign}"
            )
        _validate_in_range(self.azimuth_zero_offset_deg, "calibration.azimuth_zero_offset_deg", -180, 180)
        _validate_positive(self.lidar_height_m, "calibration.lidar_height_m")
        _validate_positive(self.alx_height_m, "calibration.alx_height_m")


@dataclass(frozen=True)
class ConfidenceConfig:
    """Confidence computation configuration."""

    weight_temporal: float = 0.35
    weight_geometry: float = 0.30
    weight_alx: float = 0.35
    alx_body_to_wrist_tolerance_m: float = 0.80
    no_observation_baseline: float = 0.0
    start: float = 0.80
    stop: float = 0.70
    unlock: float = 0.45

    def __post_init__(self) -> None:
        # Check weights sum to 1.0 within tolerance
        weight_sum = self.weight_temporal + self.weight_geometry + self.weight_alx
        if abs(weight_sum - 1.0) > 1e-9:
            raise ValueError(
                f"confidence weights must sum to 1.0, "
                f"got {weight_sum} ({self.weight_temporal} + {self.weight_geometry} + {self.weight_alx})"
            )
        _validate_non_negative(self.weight_temporal, "confidence.weight_temporal")
        _validate_non_negative(self.weight_geometry, "confidence.weight_geometry")
        _validate_non_negative(self.weight_alx, "confidence.weight_alx")
        _validate_positive(self.alx_body_to_wrist_tolerance_m, "confidence.alx_body_to_wrist_tolerance_m")
        _validate_non_negative(self.no_observation_baseline, "confidence.no_observation_baseline")
        _validate_in_range(self.start, "confidence.start", 0.0, 1.0)
        _validate_in_range(self.stop, "confidence.stop", 0.0, 1.0)
        _validate_in_range(self.unlock, "confidence.unlock", 0.0, 1.0)
        if self.start < self.stop:
            raise ValueError(
                f"confidence.start ({self.start}) must be >= confidence.stop ({self.stop})"
            )
        if self.stop < self.unlock:
            raise ValueError(
                f"confidence.stop ({self.stop}) must be >= confidence.unlock ({self.unlock})"
            )


@dataclass(frozen=True)
class AvoidanceConfig:
    """Selection of the local obstacle-avoidance implementation.

    The strategy boundary is in place for the planned local grid/A* mode, but
    APF is the only selectable mode until that planner has safety tests and a
    production implementation.
    """

    mode: str = "apf"

    def __post_init__(self) -> None:
        if self.mode != "apf":
            raise ValueError(
                "avoidance.mode must be 'apf'; the planned grid_astar mode is not implemented"
            )


@dataclass(frozen=True)
class APFConfig:
    """Continuous follow/avoidance configuration.

    The controller follows the ``jie_deamon`` pattern: target tracking,
    corridor-centering and obstacle repulsion are summed into one command.
    ``non_target_hard_stop_clearance_m`` remains an emergency body-clearance
    boundary; it is not a general obstacle-blocking threshold.
    """

    target_desired_distance_m: float = 1.00
    target_stop_distance_m: float = 0.80
    non_target_influence_m: float = 1.20
    non_target_hard_stop_clearance_m: float = 0.60
    side_hard_clearance_m: float = 0.60
    slowdown_clearance_m: float = 0.80
    obstacle_x_min_m: float = -0.80
    obstacle_x_max_m: float = 2.50
    obstacle_abs_y_max_m: float = 1.20
    body_half_length_m: float = 0.30
    body_half_width_m: float = 0.25
    swept_margin_m: float = 0.10
    # Deprecated compatibility setting.  The target corridor now always ends
    # at the current ALX target, so its length is no longer configurable.
    lateral_avoidance_length_m: float = 1.20
    lateral_avoidance_half_width_m: float = 0.60
    swept_min_distance_m: float = 0.40
    swept_horizon_s: float = 1.00
    attraction_gain: float = 0.60
    corridor_gain: float = 0.50
    repulsion_gain: float = 0.02
    repulsion_cap: float = 0.25
    automatic_target_retreat_enabled: bool = False
    target_retreat_start_distance_m: float = 0.85
    target_retreat_exit_distance_m: float = 1.00
    target_retreat_speed_mps: float = 0.10
    target_retreat_probe_distance_m: float = 0.12
    automatic_escape_enabled: bool = False
    escape_speed_mps: float = 0.06
    escape_probe_distance_m: float = 0.08
    escape_max_duration_s: float = 1.0
    escape_min_clearance_gain_m: float = 0.02
    candidate_angles_deg: Tuple[float, ...] = (0, 30, -30, 60, -60, 90, -90)
    turn_in_place_bearing_deg: float = 35.0
    omega_gain: float = 0.80

    def __post_init__(self) -> None:
        _validate_positive(self.target_desired_distance_m, "apf.target_desired_distance_m")
        _validate_positive(self.target_stop_distance_m, "apf.target_stop_distance_m")
        if self.target_stop_distance_m >= self.target_desired_distance_m:
            raise ValueError(
                f"apf.target_stop_distance_m ({self.target_stop_distance_m}) must be < "
                f"apf.target_desired_distance_m ({self.target_desired_distance_m})"
            )
        _validate_positive(self.non_target_influence_m, "apf.non_target_influence_m")
        _validate_non_negative(self.non_target_hard_stop_clearance_m, "apf.non_target_hard_stop_clearance_m")
        _validate_non_negative(self.side_hard_clearance_m, "apf.side_hard_clearance_m")
        _validate_positive(self.slowdown_clearance_m, "apf.slowdown_clearance_m")
        if self.slowdown_clearance_m <= self.non_target_hard_stop_clearance_m:
            raise ValueError(
                "apf.slowdown_clearance_m must be > "
                "apf.non_target_hard_stop_clearance_m"
            )
        _validate_finite(self.obstacle_x_min_m, "apf.obstacle_x_min_m")
        _validate_positive(self.obstacle_x_max_m, "apf.obstacle_x_max_m")
        if self.obstacle_x_min_m >= self.obstacle_x_max_m:
            raise ValueError(
                "apf.obstacle_x_min_m must be < apf.obstacle_x_max_m"
            )
        _validate_positive(self.obstacle_abs_y_max_m, "apf.obstacle_abs_y_max_m")
        _validate_non_negative(self.body_half_length_m, "apf.body_half_length_m")
        _validate_non_negative(self.body_half_width_m, "apf.body_half_width_m")
        _validate_non_negative(self.swept_margin_m, "apf.swept_margin_m")
        _validate_positive(
            self.lateral_avoidance_length_m, "apf.lateral_avoidance_length_m"
        )
        _validate_positive(
            self.lateral_avoidance_half_width_m,
            "apf.lateral_avoidance_half_width_m",
        )
        _validate_positive(self.swept_min_distance_m, "apf.swept_min_distance_m")
        _validate_positive(self.swept_horizon_s, "apf.swept_horizon_s")
        _validate_non_negative(self.attraction_gain, "apf.attraction_gain")
        _validate_non_negative(self.corridor_gain, "apf.corridor_gain")
        _validate_non_negative(self.repulsion_gain, "apf.repulsion_gain")
        _validate_non_negative(self.repulsion_cap, "apf.repulsion_cap")
        if not isinstance(self.automatic_target_retreat_enabled, bool):
            raise ValueError("apf.automatic_target_retreat_enabled must be a boolean")
        _validate_positive(
            self.target_retreat_start_distance_m,
            "apf.target_retreat_start_distance_m",
        )
        _validate_positive(
            self.target_retreat_exit_distance_m,
            "apf.target_retreat_exit_distance_m",
        )
        if self.target_retreat_start_distance_m >= self.target_retreat_exit_distance_m:
            raise ValueError(
                "apf.target_retreat_start_distance_m must be < "
                "apf.target_retreat_exit_distance_m"
            )
        _validate_positive(
            self.target_retreat_speed_mps,
            "apf.target_retreat_speed_mps",
        )
        _validate_positive(
            self.target_retreat_probe_distance_m,
            "apf.target_retreat_probe_distance_m",
        )
        if not isinstance(self.automatic_escape_enabled, bool):
            raise ValueError("apf.automatic_escape_enabled must be a boolean")
        _validate_positive(self.escape_speed_mps, "apf.escape_speed_mps")
        _validate_positive(self.escape_probe_distance_m, "apf.escape_probe_distance_m")
        _validate_positive(self.escape_max_duration_s, "apf.escape_max_duration_s")
        _validate_non_negative(
            self.escape_min_clearance_gain_m,
            "apf.escape_min_clearance_gain_m",
        )
        _validate_non_negative(self.turn_in_place_bearing_deg, "apf.turn_in_place_bearing_deg")
        _validate_non_negative(self.omega_gain, "apf.omega_gain")


@dataclass(frozen=True)
class OutputMotionConfig:
    """Output motion limiting configuration."""

    slew_rate_limit_enabled: bool = True
    max_vx: float = 0.10
    max_reverse_vx: float = 0.0
    max_abs_vy: float = 0.08
    max_abs_omega: float = 0.25
    accel_vx: float = 0.15
    decel_vx: float = 0.30
    accel_vy: float = 0.15
    decel_vy: float = 0.30
    accel_omega: float = 0.50
    decel_omega: float = 1.00

    def __post_init__(self) -> None:
        if not isinstance(self.slew_rate_limit_enabled, bool):
            raise ValueError("output_motion.slew_rate_limit_enabled must be a boolean")
        _validate_non_negative(self.max_vx, "output_motion.max_vx")
        _validate_non_negative(self.max_reverse_vx, "output_motion.max_reverse_vx")
        _validate_non_negative(self.max_abs_vy, "output_motion.max_abs_vy")
        _validate_non_negative(self.max_abs_omega, "output_motion.max_abs_omega")
        _validate_positive(self.accel_vx, "output_motion.accel_vx")
        _validate_positive(self.decel_vx, "output_motion.decel_vx")
        _validate_positive(self.accel_vy, "output_motion.accel_vy")
        _validate_positive(self.decel_vy, "output_motion.decel_vy")
        _validate_positive(self.accel_omega, "output_motion.accel_omega")
        _validate_positive(self.decel_omega, "output_motion.decel_omega")


@dataclass(frozen=True)
class WebConfig:
    """Web server configuration."""

    host: str = "127.0.0.1"
    port: int = 8080
    max_clients: int = 8
    max_message_bytes: int = 4096
    max_events_per_second: float = 20.0
    status_hz: float = 10.0
    max_visualization_points: int = 5000

    def __post_init__(self) -> None:
        if self.max_clients <= 0:
            raise ValueError(f"web.max_clients must be positive, got {self.max_clients}")
        if self.max_message_bytes <= 0:
            raise ValueError(f"web.max_message_bytes must be positive, got {self.max_message_bytes}")
        _validate_positive(self.max_events_per_second, "web.max_events_per_second")
        _validate_positive(self.status_hz, "web.status_hz")
        if self.max_visualization_points <= 0:
            raise ValueError(
                f"web.max_visualization_points must be positive, "
                f"got {self.max_visualization_points}"
            )


@dataclass(frozen=True)
class LoggingConfig:
    """Logging configuration."""

    enabled: bool = True
    csv_path: str = "logs/lidar_alx_assisted_follow.csv"
    flush_interval_s: float = 1.0

    def __post_init__(self) -> None:
        _validate_positive(self.flush_interval_s, "logging.flush_interval_s")


@dataclass(frozen=True)
class PrototypeConfig:
    """Complete prototype configuration."""

    alx: ALXConfig = field(default_factory=ALXConfig)
    alx_ros: ALXRosConfig = field(default_factory=ALXRosConfig)
    sensors: SensorsConfig = field(default_factory=SensorsConfig)
    g1: G1Config = field(default_factory=G1Config)
    output: OutputConfig = field(default_factory=OutputConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    confidence: ConfidenceConfig = field(default_factory=ConfidenceConfig)
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    apf: APFConfig = field(default_factory=APFConfig)
    output_motion: OutputMotionConfig = field(default_factory=OutputMotionConfig)
    web: WebConfig = field(default_factory=WebConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
