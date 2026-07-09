"""AO application-level configuration types.

Simplified configuration for Robot Follow AO radar runtime.
Provides minimal config types needed by radar/avoidance modules.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RoiConfig:
    """Region of interest bounds for point cloud filtering."""
    x_min: float = -2.0
    x_max: float = 2.0
    y_min: float = -1.0
    y_max: float = 1.0
    z_min: float = -0.5
    z_max: float = 2.0
    ground_z_max: float = 0.1


@dataclass(frozen=True)
class SelfFilterConfig:
    """Self-body exclusion region (robot's own body)."""
    enabled: bool = True
    x_min: float = -0.3
    x_max: float = 0.3
    y_min: float = -0.2
    y_max: float = 0.2
    z_min: float = 0.0
    z_max: float = 0.8


@dataclass(frozen=True)
class TransformConfig:
    """Static transform from radar frame to body frame."""
    input_frame: str = "livox_frame"
    target_frame: str = "body"
    apply_static_transform: bool = True
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0


@dataclass(frozen=True)
class RadarConfig:
    """Radar pipeline configuration."""
    input: str = "fast_lio"
    pointcloud_topic: str = "/cloud_registered_body"
    target_frame: str = "body"
    timeout_s: float = 0.5
    voxel_size_m: float = 0.05


@dataclass(frozen=True)
class ZoneBoundsConfig:
    """Bounds for a single safety zone."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


@dataclass(frozen=True)
class ZonesConfig:
    """Safety zone configuration."""
    bounds: dict[str, ZoneBoundsConfig]
    min_points: int = 3
    emergency_stop_distance_m: float = 0.35
    slowdown_distance_m: float = 1.0
    rotation_emergency_stop_distance_m: float = 0.4


@dataclass(frozen=True)
class AppConfig:
    """Top-level AO application configuration."""
    radar: RadarConfig
    transform: TransformConfig
    roi: RoiConfig
    self_filter: SelfFilterConfig
    zones: ZonesConfig

    @classmethod
    def default(cls) -> "AppConfig":
        """Create a default configuration for testing."""
        return cls(
            radar=RadarConfig(),
            transform=TransformConfig(),
            roi=RoiConfig(),
            self_filter=SelfFilterConfig(),
            zones=ZonesConfig(
                min_points=3,
                emergency_stop_distance_m=0.35,
                slowdown_distance_m=1.0,
                rotation_emergency_stop_distance_m=0.4,
                bounds={
                    "front": ZoneBoundsConfig(
                        x_min=0.0,
                        x_max=1.5,
                        y_min=-0.4,
                        y_max=0.4,
                        z_min=-0.1,
                        z_max=1.5,
                    ),
                    "left": ZoneBoundsConfig(
                        x_min=-0.5,
                        x_max=1.0,
                        y_min=0.4,
                        y_max=1.0,
                        z_min=-0.1,
                        z_max=1.5,
                    ),
                    "right": ZoneBoundsConfig(
                        x_min=-0.5,
                        x_max=1.0,
                        y_min=-1.0,
                        y_max=-0.4,
                        z_min=-0.1,
                        z_max=1.5,
                    ),
                    "rear": ZoneBoundsConfig(
                        x_min=-1.0,
                        x_max=-0.3,
                        y_min=-0.4,
                        y_max=0.4,
                        z_min=-0.1,
                        z_max=1.5,
                    ),
                    "rotation": ZoneBoundsConfig(
                        x_min=-0.5,
                        x_max=0.5,
                        y_min=-0.6,
                        y_max=0.6,
                        z_min=0.0,
                        z_max=1.0,
                    ),
                },
            ),
        )