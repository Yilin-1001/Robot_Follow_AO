"""Tests for LiDAR + ALX Assisted Follow configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest


def write_yaml(tmp_path: Path, content: str) -> Path:
    """Write YAML content to a temporary file."""
    path = tmp_path / "test_config.yaml"
    path.write_text(content, encoding="utf-8")
    return path


def write_valid_base_config(tmp_path: Path, **overrides: Any) -> Path:
    """Write a valid base config with optional overrides."""
    base = """
lidar_alx_assisted_follow:
  alx:
    port: /dev/ttyUSB0
    baudrate: 115200
    stale_timeout_s: 0.50
    cloud_max_skew_s: 0.15
    min_distance_m: 0.10
    max_distance_m: 10.0
  sensors:
    topic: /cloud_registered_body
    expected_frame_id: body
    max_cloud_age_s: 0.25
    ready_timeout_s: 10.0
    stop_timeout_s: 3.0
    livox_start_command: "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    fast_lio_start_command: "ros2 launch fast_lio mapping.launch.py"
  g1:
    network_interface: eth0
    max_vx: 0.10
    max_abs_vy: 0.08
    max_abs_omega: 0.25
    yaw_sign: 1
    sdk_timeout_s: 10.0
    sdk_timeout_code: 3104
    sdk_timeout_retry_interval_s: 1.0
    max_consecutive_sdk_timeouts: 3
    command_timeout_s: 0.50
  output:
    mode: mock
  runtime:
    watchdog_hz: 40.0
    max_strategy_age_s: 0.15
    max_strategy_process_ms: 20.0
    control_event_capacity: 32
  target:
    radial_min_m: 0.25
    radial_max_m: 5.0
    physical_height_min_m: 0.10
    physical_height_max_m: 2.00
    self_x_min_m: -0.35
    self_x_max_m: 0.25
    self_abs_y_max_m: 0.25
    voxel_size_m: 0.05
    web_pick_radius_m: 0.35
    gate_radial_radius_m: 0.45
    gate_tangential_radius_m: 0.35
    min_support_points: 8
    min_vertical_extent_m: 0.45
    max_horizontal_footprint_m: 0.90
    max_radial_mad_m: 0.18
    max_center_displacement_m: 0.35
    max_frame_interval_s: 0.20
    lost_timeout_s: 0.75
    desired_distance_m: 1.00
    stop_distance_m: 0.80
  calibration:
    x_m: 0.0
    y_m: 0.0
    yaw_deg: 0.0
    azimuth_sign: 1
    azimuth_zero_offset_deg: 0.0
    lidar_height_m: 1.25
    alx_height_m: 1.10
    allow_provisional_calibration_for_prototype: true
  confidence:
    weight_temporal: 0.35
    weight_geometry: 0.30
    weight_alx: 0.35
    alx_body_to_wrist_tolerance_m: 0.80
    no_observation_baseline: 0.0
    start: 0.80
    stop: 0.70
    unlock: 0.45
  apf:
    target_desired_distance_m: 1.00
    target_stop_distance_m: 0.80
    non_target_influence_m: 1.20
    non_target_hard_stop_clearance_m: 0.60
    body_half_length_m: 0.30
    body_half_width_m: 0.25
    swept_margin_m: 0.10
    swept_min_distance_m: 0.40
    swept_horizon_s: 1.00
    attraction_gain: 0.60
    repulsion_gain: 0.02
    repulsion_cap: 0.25
    candidate_angles_deg: [0, 30, -30, 60, -60, 90, -90]
    turn_in_place_bearing_deg: 35.0
    omega_gain: 0.80
  output_motion:
    max_vx: 0.10
    max_abs_vy: 0.08
    max_abs_omega: 0.25
    accel_vx: 0.15
    decel_vx: 0.30
    accel_vy: 0.15
    decel_vy: 0.30
    accel_omega: 0.50
    decel_omega: 1.00
  web:
    host: 127.0.0.1
    port: 8080
    max_clients: 8
    max_message_bytes: 4096
    max_events_per_second: 20
    status_hz: 10.0
    max_visualization_points: 5000
  logging:
    enabled: true
    csv_path: logs/lidar_alx_assisted_follow.csv
    flush_interval_s: 1.0
"""
    if overrides:
        # This is simplified - for real implementation, merge overrides
        pass
    return write_yaml(tmp_path, base)


class TestConfigValidation:
    """Test configuration validation rules."""

    def test_extends_preserves_base_settings_and_overrides_requested_values(
        self, tmp_path: Path
    ) -> None:
        """A reference profile may override only the parameters it changes."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        base_path = tmp_path / "base.yaml"
        base_path.write_text(
            """
lidar_alx_assisted_follow:
  alx:
    port: /dev/custom-alx
  apf:
    target_desired_distance_m: 1.0
    target_stop_distance_m: 0.8
    attraction_gain: 0.6
""",
            encoding="utf-8",
        )
        profile_path = tmp_path / "profile.yaml"
        profile_path.write_text(
            """
extends: base.yaml
lidar_alx_assisted_follow:
  apf:
    target_desired_distance_m: 0.4
    target_stop_distance_m: 0.1
    attraction_gain: 0.5
""",
            encoding="utf-8",
        )

        config = load_prototype_config(profile_path)

        assert config.alx.port == "/dev/custom-alx"
        assert config.apf.target_desired_distance_m == 0.4
        assert config.apf.target_stop_distance_m == 0.1
        assert config.apf.attraction_gain == 0.5

    def test_unknown_nested_key_is_rejected(self, tmp_path: Path) -> None:
        """Unknown keys in nested sections should raise ValueError."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  output:
    mode: mock
    typo: 1
""",
        )
        with pytest.raises(ValueError, match="unknown keys.*output.*typo"):
            load_prototype_config(path)

    def test_non_finite_number_rejected(self, tmp_path: Path) -> None:
        """Non-finite numbers should be rejected."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  g1:
    max_vx: .nan
""",
        )
        with pytest.raises(ValueError, match="finite"):
            load_prototype_config(path)

    def test_negative_timeout_rejected(self, tmp_path: Path) -> None:
        """Negative timeouts should be rejected."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  sensors:
    ready_timeout_s: -1.0
""",
        )
        with pytest.raises(ValueError, match="positive"):
            load_prototype_config(path)

    def test_invalid_yaw_sign_rejected(self, tmp_path: Path) -> None:
        """Yaw sign must be +1 or -1."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  g1:
    yaw_sign: 2
""",
        )
        with pytest.raises(ValueError, match="yaw_sign"):
            load_prototype_config(path)

    def test_weights_must_sum_to_one(self, tmp_path: Path) -> None:
        """Confidence weights must sum to 1.0 within tolerance."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  confidence:
    weight_temporal: 0.5
    weight_geometry: 0.3
    weight_alx: 0.3
""",
        )
        with pytest.raises(ValueError, match="weights.*sum"):
            load_prototype_config(path)

    def test_start_must_be_greater_than_stop(self, tmp_path: Path) -> None:
        """Start threshold must be >= stop threshold."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  confidence:
    start: 0.60
    stop: 0.70
""",
        )
        with pytest.raises(ValueError, match="start.*stop"):
            load_prototype_config(path)

    def test_stop_must_be_greater_than_unlock(self, tmp_path: Path) -> None:
        """Stop threshold must be >= unlock threshold."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  confidence:
    stop: 0.40
    unlock: 0.50
""",
        )
        with pytest.raises(ValueError, match="stop.*unlock"):
            load_prototype_config(path)

    def test_non_body_frame_id_rejected(self, tmp_path: Path) -> None:
        """Expected frame ID must be 'body'."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  sensors:
    expected_frame_id: base_link
""",
        )
        with pytest.raises(ValueError, match="frame_id.*body"):
            load_prototype_config(path)

    def test_valid_config_loads_successfully(self, tmp_path: Path) -> None:
        """A valid configuration should load without error."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_valid_base_config(tmp_path)
        config = load_prototype_config(path)
        assert config is not None
        assert config.avoidance.mode == "apf"

    def test_jie_tf_ground_profile_uses_gravity_aligned_body_to_base_link_transform(self) -> None:
        """The TF ground profile must use the rosbag-calibrated FRD-to-FLU transform."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        repository_root = Path(__file__).parents[3]
        config = load_prototype_config(
            repository_root
            / "configs/jetson_g1_lidar_alx_assisted_follow_jie_reference_tf_ground.yaml"
        )

        assert config.output.mode == "real"
        assert config.alx_ros.frame_id == "base_link"
        assert config.sensors.expected_frame_id == "body"
        assert config.sensors.output_frame_id == "base_link"
        assert config.sensors.apply_body_to_output_transform is True
        assert config.sensors.body_to_output_roll_deg == -178.081392
        assert config.sensors.body_to_output_pitch_deg == 32.651169
        assert config.sensors.body_to_output_yaw_deg == 1.035408
        assert config.sensors.body_to_output_z_m == 0.25
        assert config.sensors.cloud_y_sign == 1
        assert "static_transform_publisher" in config.sensors.static_tf_start_command
        assert config.calibration.x_m == 0.10
        assert config.calibration.lidar_height_m == 1.00

    def test_unimplemented_avoidance_mode_is_rejected(self, tmp_path: Path) -> None:
        """A future planner cannot be selected before it has a safe implementation."""
        from robot_follow.lidar_alx_assisted_follow.config import load_prototype_config

        path = write_yaml(
            tmp_path,
            """
lidar_alx_assisted_follow:
  avoidance:
    mode: grid_astar
""",
        )
        with pytest.raises(ValueError, match="grid_astar mode is not implemented"):
            load_prototype_config(path)


class TestConfigAdapters:
    """Test configuration adapter functions."""

    def test_build_radar_service_config(self, tmp_path: Path) -> None:
        """Test building AORadarServiceConfig from prototype config."""
        from robot_follow.lidar_alx_assisted_follow.config import (
            build_radar_service_config,
            load_prototype_config,
        )

        path = write_valid_base_config(tmp_path)
        prototype_config = load_prototype_config(path)
        radar_config = build_radar_service_config(prototype_config)

        assert radar_config.livox_start_command != ""
        assert radar_config.fast_lio_start_command != ""
        assert radar_config.stop_timeout_s == 3.0
        assert radar_config.ready_timeout_s == 10.0

    def test_build_g1_bridge_config(self, tmp_path: Path) -> None:
        """Test building G1BridgeConfig from prototype config."""
        from robot_follow.lidar_alx_assisted_follow.config import (
            build_g1_bridge_config,
            load_prototype_config,
        )

        path = write_valid_base_config(tmp_path)
        prototype_config = load_prototype_config(path)
        g1_config = build_g1_bridge_config(prototype_config)

        assert g1_config.max_abs_vx == 0.10
        assert g1_config.max_abs_vy == 0.08
        assert g1_config.max_abs_omega == 0.25
        assert g1_config.yaw_sign == 1
