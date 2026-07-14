from pathlib import Path

import pytest

from robot_follow.ao.config import AORadarConfig, load_ao_radar_config, load_ao_radar_configs


def test_loads_default_config_values_from_empty_mapping(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    config = load_ao_radar_config(path)

    assert isinstance(config, AORadarConfig)
    assert config.alx.port == "/dev/ttyUSB0"
    assert config.alx.stale_timeout_s == 0.5
    assert config.radar.idle_stop_s == 5.0
    assert config.avoidance.max_vx == 0.5
    assert config.avoidance.max_vy == 0.2
    assert config.avoidance.max_omega == 0.8
    assert config.latency.max_command_age_s == 0.2
    assert config.latency.max_cloud_age_s == 0.5
    assert config.output.require_real_g1_confirmation is True
    assert config.output.mode == "mock"
    assert config.output.network_interface == "eth0"
    assert config.logging.enabled is True
    assert config.logging.timestamped_runs is True
    assert config.logging.terminal_summary_interval_s == 0.5


def test_rejects_unknown_ao_section_key(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        "ao_radar:\n"
        "  latency:\n"
        "    not_a_real_key: 1\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="unknown keys in ao_radar.latency"):
        load_ao_radar_config(path)


def test_loads_runtime_overrides(tmp_path: Path) -> None:
    path = tmp_path / "ao.yaml"
    path.write_text(
        "ao_radar:\n"
        "  alx:\n"
        "    port: /dev/ttyUSB9\n"
        "    stale_timeout_s: 0.35\n"
        "  filter:\n"
        "    alpha: 0.55\n"
        "  follow:\n"
        "    target_distance_m: 1.2\n"
        "    distance_gain_forward: 0.30\n"
        "  safety:\n"
        "    min_safe_distance_m: 0.4\n"
        "  g1:\n"
        "    max_vx: 0.08\n"
        "    max_vy: 0.25\n"
        "    max_omega: 0.20\n"
        "  avoidance:\n"
        "    lateral_vy_mps: 0.25\n"
        "    forward_vx_when_avoiding_mps: 0.05\n"
        "    max_vx: 0.30\n"
        "    max_vy: 0.15\n"
        "    max_omega: 1.20\n"
        "  radar:\n"
        "    idle_stop_s: 5.0\n"
        "    livox_start_command: ros2 launch livox_ros_driver2 msg_MID360_launch.py\n"
        "    fast_lio_start_command: ros2 launch fast_lio mapping.launch.py\n"
        "  latency:\n"
        "    ao_output_loop_hz: 20.0\n"
        "    max_command_age_s: 0.2\n"
        "    max_cloud_age_s: 0.5\n"
        "    max_ao_process_ms: 20.0\n"
        "    max_g1_dispatch_ms: 30.0\n"
        "    warn_loop_dt_ms: 75.0\n"
        "    hard_loop_dt_ms: 150.0\n"
        "  output:\n"
        "    mode: g1\n"
        "    confirm_real_g1: true\n"
        "    network_interface: eth0\n"
        "  logging:\n"
        "    enabled: false\n"
        "    csv_path: logs/custom.csv\n",
        encoding="utf-8",
    )

    config = load_ao_radar_config(path)

    assert config.alx.port == "/dev/ttyUSB9"
    assert config.alx.stale_timeout_s == 0.35
    assert config.filter.alpha == 0.55
    assert config.follow.target_distance_m == 1.2
    assert config.follow.distance_gain_forward == 0.30
    assert config.safety.min_safe_distance_m == 0.4
    assert config.g1.max_vx == 0.08
    assert config.g1.max_vy == 0.25
    assert config.g1.max_omega == 0.20
    assert config.avoidance.lateral_vy_mps == 0.25
    assert config.avoidance.forward_vx_when_avoiding_mps == 0.05
    assert config.avoidance.max_vx == 0.30
    assert config.avoidance.max_vy == 0.15
    assert config.avoidance.max_omega == 1.20
    assert config.radar.idle_stop_s == 5.0
    assert config.radar.livox_start_command.startswith("ros2 launch livox")
    assert config.latency.ao_output_loop_hz == 20.0
    assert config.output.mode == "g1"
    assert config.output.confirm_real_g1 is True
    assert config.output.network_interface == "eth0"
    assert config.logging.enabled is False
    assert config.logging.csv_path == "logs/custom.csv"


def test_loads_dedicated_ao_radar_config() -> None:
    config = load_ao_radar_config(Path("configs/jetson_g1_ao_radar.yaml"))

    assert config.output.mode == "g1"
    assert config.output.network_interface == "eth0"
    assert config.radar.livox_start_command == "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    assert "/home/fk/g1_mid360_ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml" in (
        config.radar.fast_lio_start_command
    )
    assert config.logging.timestamped_runs is True
    assert config.filter.alpha == 0.65
    assert config.follow.target_distance_m == 1.0
    assert config.follow.min_safe_distance_m == 0.15
    assert config.follow.max_forward_vx == 2.0
    assert config.safety.min_safe_distance_m == 0.15
    assert config.g1.max_vx == 3.0
    assert config.g1.max_vy == 0.30
    assert config.g1.max_omega == 3.0
    assert config.avoidance.lateral_vy_mps == 0.30
    assert config.avoidance.forward_vx_when_avoiding_mps == 0.05
    assert config.avoidance.max_vx == 2.0
    assert config.avoidance.max_vy == 0.30
    assert config.avoidance.max_omega == 1.5


def test_later_config_overrides_legacy_base_at_field_level(tmp_path: Path) -> None:
    base = tmp_path / "base.yaml"
    base.write_text(
        "alx:\n"
        "  port: /dev/ttyUSB9\n"
        "  hz: 40.0\n"
        "filter:\n"
        "  alpha: 0.20\n"
        "follow:\n"
        "  target_distance_m: 1.8\n"
        "  max_forward_vx: 0.05\n"
        "safety:\n"
        "  min_safe_distance_m: 0.40\n"
        "g1:\n"
        "  net_iface: enp2s0\n"
        "  max_vx: 0.04\n"
        "ao_radar:\n"
        "  output:\n"
        "    mode: mock\n",
        encoding="utf-8",
    )
    overlay = tmp_path / "overlay.yaml"
    overlay.write_text(
        "ao_radar:\n"
        "  filter:\n"
        "    alpha: 0.60\n"
        "  follow:\n"
        "    target_distance_m: 1.2\n"
        "  g1:\n"
        "    max_vx: 0.08\n"
        "  output:\n"
        "    network_interface: eth0\n",
        encoding="utf-8",
    )

    config = load_ao_radar_configs([base, overlay])

    assert config.alx.port == "/dev/ttyUSB9"
    assert config.filter.alpha == 0.60
    assert config.follow.target_distance_m == 1.2
    assert config.follow.max_forward_vx == 0.05
    assert config.safety.min_safe_distance_m == 0.40
    assert config.g1.max_vx == 0.08
    assert config.output.network_interface == "eth0"
