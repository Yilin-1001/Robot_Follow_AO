from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Sequence, TypeVar

import yaml


ConfigMapping = Dict[str, Any]


def load_yaml_config(path: Path) -> ConfigMapping:
    """Load a project YAML config file."""
    text = path.read_text(encoding="utf-8")
    loaded = yaml.safe_load(text)
    if loaded is None:
        return {}
    if not isinstance(loaded, dict):
        raise ValueError(f"config file {path} must be a mapping")
    return loaded


@dataclass(frozen=True)
class AOALXConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    stale_timeout_s: float = 0.5


@dataclass(frozen=True)
class AOFilterConfig:
    """ALX target low-pass filter settings used by the AO live runner."""

    alpha: float = 0.35


@dataclass(frozen=True)
class AOFollowConfig:
    """Person-follow controller settings used by the AO live runner."""

    target_distance_m: float = 1.5
    min_safe_distance_m: float = 0.5
    hold_tolerance_m: float = 0.15
    max_forward_vx: float = 0.20
    max_backward_vx: float = 0.12
    max_follow_distance_m: float = 4.0
    max_omega: float = 0.60
    angle_deadzone_deg: float = 7.0
    large_angle_deg: float = 20.0
    distance_gain_forward: float = 0.18
    distance_gain_backward: float = 0.15
    angle_gain: float = 0.5


@dataclass(frozen=True)
class AOSafetyConfig:
    """Target-data safety checks applied before follow control."""

    data_timeout_s: float = 0.5
    min_safe_distance_m: float = 0.5
    max_distance_jump_m: float = 0.5
    max_angle_jump_deg: float = 45.0


@dataclass(frozen=True)
class AOG1Config:
    """G1 bridge limits and SDK timeout policy for the AO live runner."""

    max_vx: float = 0.10
    max_vy: float = 0.0
    max_omega: float = 0.30
    yaw_sign: int = 1
    sdk_timeout_s: float = 10.0
    sdk_timeout_code: int = 3104
    sdk_timeout_retry_interval_s: float = 1.0
    max_consecutive_sdk_timeouts: int = 3
    command_timeout_s: float = 0.5


@dataclass(frozen=True)
class AOAvoidanceConfig:
    """Avoidance planner velocity limits applied before G1 dispatch.

    These are the final clamps on the safe command: even if ``follow``
    produces a larger omega/vx, the planner caps it here. Defaults match the
    planner's built-in ``DefaultAvoidanceConfig`` so omitting the section keeps
    historical behavior. Raise ``max_omega`` so the robot can turn fast enough
    to keep the wristband inside the radar FOV while following.
    """

    lateral_vy_mps: float = 0.08
    forward_vx_when_avoiding_mps: float = 0.03
    max_vx: float = 0.5
    max_vy: float = 0.2
    max_omega: float = 0.8


@dataclass(frozen=True)
class AORadarServiceConfig:
    idle_stop_s: float = 5.0
    static_tf_start_command: str = ""
    livox_start_command: str = ""
    fast_lio_start_command: str = ""
    stop_timeout_s: float = 3.0
    ready_timeout_s: float = 10.0


@dataclass(frozen=True)
class AOLatencyConfig:
    ao_output_loop_hz: float = 20.0
    max_command_age_s: float = 0.2
    max_cloud_age_s: float = 0.5
    max_ao_process_ms: float = 20.0
    max_g1_dispatch_ms: float = 30.0
    warn_loop_dt_ms: float = 75.0
    hard_loop_dt_ms: float = 150.0


@dataclass(frozen=True)
class AOOutputConfig:
    mode: str = "mock"
    require_real_g1_confirmation: bool = True
    confirm_real_g1: bool = False
    network_interface: str = "eth0"


@dataclass(frozen=True)
class AOLoggingConfig:
    enabled: bool = True
    csv_path: str = "logs/ao_radar_frames.csv"
    timestamped_runs: bool = True
    filename_time_format: str = "%Y%m%d_%H%M%S"
    log_every_n_frames: int = 1
    terminal_summary_interval_s: float = 0.5


@dataclass(frozen=True)
class AORadarConfig:
    alx: AOALXConfig = field(default_factory=AOALXConfig)
    filter: AOFilterConfig = field(default_factory=AOFilterConfig)
    follow: AOFollowConfig = field(default_factory=AOFollowConfig)
    safety: AOSafetyConfig = field(default_factory=AOSafetyConfig)
    g1: AOG1Config = field(default_factory=AOG1Config)
    avoidance: AOAvoidanceConfig = field(default_factory=AOAvoidanceConfig)
    radar: AORadarServiceConfig = field(default_factory=AORadarServiceConfig)
    latency: AOLatencyConfig = field(default_factory=AOLatencyConfig)
    output: AOOutputConfig = field(default_factory=AOOutputConfig)
    logging: AOLoggingConfig = field(default_factory=AOLoggingConfig)


T = TypeVar("T")


def load_ao_radar_config(path: Path) -> AORadarConfig:
    """Load one AO configuration file (backward-compatible convenience API)."""
    return load_ao_radar_configs([path])


def load_ao_radar_configs(paths: Sequence[Path]) -> AORadarConfig:
    """Load and deep-merge AO configuration files in command-line order.

    Later files override earlier values at the individual-field level.  A
    legacy top-level ``alx``, ``filter``, ``follow``, ``safety``, or ``g1``
    section is treated as AO input too, so ``jetson_g1_safe.yaml`` can be used
    as a base configuration.  Its ``g1.net_iface`` is mapped to
    ``ao_radar.output.network_interface`` and its ALX-only ``hz`` setting is
    ignored by the AO runner.
    """
    if not paths:
        raise ValueError("at least one AO config path is required")

    section: ConfigMapping = {}
    for path in paths:
        _deep_merge(section, _ao_config_layer(load_yaml_config(path)))

    return AORadarConfig(
        alx=_dataclass_from_section(section, "alx", AOALXConfig),
        filter=_dataclass_from_section(section, "filter", AOFilterConfig),
        follow=_dataclass_from_section(section, "follow", AOFollowConfig),
        safety=_dataclass_from_section(section, "safety", AOSafetyConfig),
        g1=_dataclass_from_section(section, "g1", AOG1Config),
        avoidance=_dataclass_from_section(section, "avoidance", AOAvoidanceConfig),
        radar=_dataclass_from_section(section, "radar", AORadarServiceConfig),
        latency=_dataclass_from_section(section, "latency", AOLatencyConfig),
        output=_dataclass_from_section(section, "output", AOOutputConfig),
        logging=_dataclass_from_section(section, "logging", AOLoggingConfig),
    )


def _ao_config_layer(raw: ConfigMapping) -> ConfigMapping:
    """Extract AO-relevant settings from one mixed legacy/AO YAML file."""
    layer: ConfigMapping = {}
    for key in ("alx", "filter", "follow", "safety", "g1"):
        value = raw.get(key)
        if value is None:
            continue
        if not isinstance(value, dict):
            raise ValueError(f"config section '{key}' must be a mapping")
        legacy_section = dict(value)
        if key == "alx":
            legacy_section.pop("hz", None)
        if key == "g1":
            network_interface = legacy_section.pop("net_iface", None)
            if network_interface is not None:
                layer["output"] = {"network_interface": network_interface}
        if legacy_section:
            layer[key] = legacy_section

    ao_section = _mapping_section(raw, "ao_radar")
    _deep_merge(layer, ao_section)
    return layer


def _deep_merge(target: ConfigMapping, overlay: ConfigMapping) -> None:
    """Merge mappings recursively, replacing non-mapping values from overlay."""
    for key, value in overlay.items():
        existing = target.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            _deep_merge(existing, value)
        elif isinstance(value, dict):
            target[key] = dict(value)
        else:
            target[key] = value


def _mapping_section(config: ConfigMapping, key: str) -> ConfigMapping:
    value = config.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"config section '{key}' must be a mapping")
    return value


def _dataclass_from_section(config: ConfigMapping, key: str, cls: type[T]) -> T:
    section = _mapping_section(config, key)
    allowed = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
    unknown = sorted(set(section) - allowed)
    if unknown:
        raise ValueError(f"unknown keys in ao_radar.{key}: {', '.join(unknown)}")
    kwargs: dict[str, Any] = {name: section[name] for name in section}
    return cls(**kwargs)
