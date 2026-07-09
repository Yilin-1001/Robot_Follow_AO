from pathlib import Path

import pytest

from robot_follow.ao.config import AORadarConfig, load_ao_radar_config


def test_loads_default_config_values_from_empty_mapping(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    config = load_ao_radar_config(path)

    assert isinstance(config, AORadarConfig)
    assert config.alx.port == "/dev/ttyUSB0"
    assert config.alx.stale_timeout_s == 0.5
    assert config.radar.idle_stop_s == 5.0
    assert config.latency.max_command_age_s == 0.2
    assert config.latency.max_cloud_age_s == 0.5
    assert config.output.require_real_g1_confirmation is True
    assert config.output.network_interface == "enp2s0"


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
        "    network_interface: eth0\n",
        encoding="utf-8",
    )

    config = load_ao_radar_config(path)

    assert config.alx.port == "/dev/ttyUSB9"
    assert config.alx.stale_timeout_s == 0.35
    assert config.radar.idle_stop_s == 5.0
    assert config.radar.livox_start_command.startswith("ros2 launch livox")
    assert config.latency.ao_output_loop_hz == 20.0
    assert config.output.mode == "g1"
    assert config.output.confirm_real_g1 is True
    assert config.output.network_interface == "eth0"
