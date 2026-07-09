from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, TypeVar

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
class AORadarServiceConfig:
    idle_stop_s: float = 5.0
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
    radar: AORadarServiceConfig = field(default_factory=AORadarServiceConfig)
    latency: AOLatencyConfig = field(default_factory=AOLatencyConfig)
    output: AOOutputConfig = field(default_factory=AOOutputConfig)
    logging: AOLoggingConfig = field(default_factory=AOLoggingConfig)


T = TypeVar("T")


def load_ao_radar_config(path: Path) -> AORadarConfig:
    raw = load_yaml_config(path)
    section = _mapping_section(raw, "ao_radar")
    return AORadarConfig(
        alx=_dataclass_from_section(section, "alx", AOALXConfig),
        radar=_dataclass_from_section(section, "radar", AORadarServiceConfig),
        latency=_dataclass_from_section(section, "latency", AOLatencyConfig),
        output=_dataclass_from_section(section, "output", AOOutputConfig),
        logging=_dataclass_from_section(section, "logging", AOLoggingConfig),
    )


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
