from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeVar

from robot_follow.main import ConfigMapping, load_yaml_config


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
    mode: str = "g1"
    require_real_g1_confirmation: bool = True
    confirm_real_g1: bool = False


@dataclass(frozen=True)
class AORadarConfig:
    alx: AOALXConfig = field(default_factory=AOALXConfig)
    radar: AORadarServiceConfig = field(default_factory=AORadarServiceConfig)
    latency: AOLatencyConfig = field(default_factory=AOLatencyConfig)
    output: AOOutputConfig = field(default_factory=AOOutputConfig)


T = TypeVar("T")


def load_ao_radar_config(path: Path) -> AORadarConfig:
    raw = load_yaml_config(path)
    section = _mapping_section(raw, "ao_radar")
    return AORadarConfig(
        alx=_dataclass_from_section(section, "alx", AOALXConfig),
        radar=_dataclass_from_section(section, "radar", AORadarServiceConfig),
        latency=_dataclass_from_section(section, "latency", AOLatencyConfig),
        output=_dataclass_from_section(section, "output", AOOutputConfig),
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
