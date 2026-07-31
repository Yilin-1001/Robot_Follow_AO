"""Configuration loader with strict validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Set, Type, TypeVar

import yaml

from .schema import (
    APFConfig,
    AvoidanceConfig,
    ALXConfig,
    ALXRosConfig,
    CalibrationConfig,
    ConfidenceConfig,
    G1Config,
    LoggingConfig,
    OutputConfig,
    OutputMotionConfig,
    PrototypeConfig,
    RuntimeConfig,
    SensorsConfig,
    TargetConfig,
    WebConfig,
)


def load_prototype_config(path: Path) -> PrototypeConfig:
    """Load and validate prototype configuration from YAML file.

    Parameters
    ----------
    path : Path
        Path to YAML configuration file.

    Returns
    -------
    PrototypeConfig
        Validated configuration object.

    Raises
    ------
    ValueError
        If configuration is invalid or contains unknown keys.
    FileNotFoundError
        If configuration file does not exist.
    """
    return _build_config(_load_config_root(path.resolve(), set()))


def _load_config_root(path: Path, inheritance_chain: Set[Path]) -> Dict[str, Any]:
    """Load one config root, resolving an optional relative ``extends`` file."""
    if path in inheritance_chain:
        raise ValueError(f"configuration inheritance cycle detected at {path}")
    if not path.exists():
        raise FileNotFoundError(f"configuration file not found: {path}")

    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if loaded is None:
        raise ValueError(f"configuration file {path} is empty")
    if not isinstance(loaded, dict):
        raise ValueError(f"configuration file {path} must be a mapping")

    root = loaded.get("lidar_alx_assisted_follow")
    if root is None:
        raise ValueError(
            f"configuration file {path} must have 'lidar_alx_assisted_follow' root key"
        )
    if not isinstance(root, dict):
        raise ValueError("'lidar_alx_assisted_follow' must be a mapping")

    base_config = loaded.get("extends")
    if base_config is None:
        return root
    if not isinstance(base_config, str) or not base_config:
        raise ValueError(f"configuration file {path} has invalid 'extends' value")

    base_path = (path.parent / base_config).resolve()
    base_root = _load_config_root(base_path, inheritance_chain | {path})
    return _merge_mappings(base_root, root)


def _merge_mappings(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge an override mapping without mutating its base."""
    merged: Dict[str, Any] = dict(base)
    for key, value in override.items():
        base_value = merged.get(key)
        if isinstance(base_value, dict) and isinstance(value, dict):
            merged[key] = _merge_mappings(base_value, value)
        else:
            merged[key] = value
    return merged


T = TypeVar("T")


def _build_config(root: Dict[str, Any]) -> PrototypeConfig:
    """Build validated PrototypeConfig from loaded YAML mapping."""

    def section(name: str, cls: Type[T]) -> T:
        data = root.get(name)
        if data is None:
            return cls()

        if not isinstance(data, dict):
            raise ValueError(f"section '{name}' must be a mapping")

        return _validated_section(name, data, cls)

    return PrototypeConfig(
        alx=section("alx", ALXConfig),
        alx_ros=section("alx_ros", ALXRosConfig),
        sensors=section("sensors", SensorsConfig),
        g1=section("g1", G1Config),
        output=section("output", OutputConfig),
        runtime=section("runtime", RuntimeConfig),
        target=section("target", TargetConfig),
        calibration=section("calibration", CalibrationConfig),
        confidence=section("confidence", ConfidenceConfig),
        avoidance=section("avoidance", AvoidanceConfig),
        apf=section("apf", APFConfig),
        output_motion=section("output_motion", OutputMotionConfig),
        web=section("web", WebConfig),
        logging=section("logging", LoggingConfig),
    )


def _validated_section(
    section_name: str, data: Dict[str, Any], cls: Type[T]
) -> T:
    """Validate section data against dataclass fields and construct instance."""

    # Get allowed field names from dataclass
    allowed: Set[str] = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]

    # Check for unknown keys
    unknown = sorted(set(data.keys()) - allowed)
    if unknown:
        raise ValueError(
            f"unknown keys in {section_name}: {', '.join(unknown)}"
        )

    # Construct instance with validation (via __post_init__)
    try:
        return cls(**data)
    except TypeError as e:
        raise ValueError(f"invalid {section_name} configuration: {e}") from e
    except ValueError as e:
        # Re-raise without duplicating section context
        # The error message from __post_init__ already contains full context
        if str(e).startswith(f"{section_name}."):
            raise
        raise ValueError(f"{section_name}.{e}") from e
