"""AO application-level configuration types.

Simplified configuration for Robot Follow AO radar runtime.
Provides minimal config types needed by radar/avoidance modules.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RoiConfig:
    """Region of interest bounds for point cloud filtering."""
    x_min: float = -1.2
    x_max: float = 3.0
    y_min: float = -1.8
    y_max: float = 1.8
    z_min: float = -1.2
    z_max: float = 1.8
    ground_z_max: float = -1.2


@dataclass(frozen=True)
class SelfFilterConfig:
    """Self-body exclusion region (robot's own body)."""
    enabled: bool = True
    x_min: float = -0.35
    x_max: float = 0.20
    y_min: float = -0.45
    y_max: float = 0.45
    z_min: float = -0.5
    z_max: float = 1.5


@dataclass(frozen=True)
class TransformConfig:
    """Static transform from radar frame to body frame."""
    input_frame: str = "livox_frame"
    target_frame: str = "base_link"
    apply_static_transform: bool = True
    roll_deg: float = 180.0
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
    target_frame: str = "base_link"
    timeout_s: float = 0.5
    voxel_size_m: float = 0.08


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
    min_points: int = 10
    emergency_stop_distance_m: float = 0.45
    slowdown_distance_m: float = 1.2
    rotation_emergency_stop_distance_m: float = 0.45


@dataclass(frozen=True)
class AvoidanceConfig:
    """Avoidance planner configuration used by the web visualizer."""
    lateral_vy_mps: float = 0.12
    forward_vx_when_avoiding_mps: float = 0.03
    min_slowdown_ratio: float = 0.25
    allow_left_avoidance: bool = True
    allow_right_avoidance: bool = True
    allow_planner_backward_escape: bool = False
    max_vx: float = 0.20
    max_vy: float = 0.15
    max_omega: float = 0.50
    stateful_enabled: bool = True
    avoid_clear_required_frames: int = 4
    avoid_clearance_hold_s: float = 0.4
    side_switch_cooldown_s: float = 1.0
    side_danger_distance_m: float = 0.35
    side_score_min_margin_m: float = 0.08
    tie_break_side: str = "left"
    return_vy_decay: float = 0.5
    return_vy_epsilon_mps: float = 0.01


@dataclass(frozen=True)
class AppConfig:
    """Top-level AO application configuration."""
    radar: RadarConfig
    transform: TransformConfig
    roi: RoiConfig
    self_filter: SelfFilterConfig
    zones: ZonesConfig
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)

    @classmethod
    def default(cls) -> "AppConfig":
        """Create a default configuration for testing."""
        return cls(
            radar=RadarConfig(),
            transform=TransformConfig(),
            roi=RoiConfig(),
            self_filter=SelfFilterConfig(),
            zones=ZonesConfig(
                min_points=10,
                emergency_stop_distance_m=0.45,
                slowdown_distance_m=1.2,
                rotation_emergency_stop_distance_m=0.45,
                bounds={
                    "front": ZoneBoundsConfig(
                        x_min=0.25,
                        x_max=1.2,
                        y_min=-0.4,
                        y_max=0.4,
                        z_min=-0.35,
                        z_max=1.5,
                    ),
                    "left": ZoneBoundsConfig(
                        x_min=-0.05,
                        x_max=0.7,
                        y_min=0.35,
                        y_max=0.75,
                        z_min=-0.35,
                        z_max=1.5,
                    ),
                    "right": ZoneBoundsConfig(
                        x_min=-0.05,
                        x_max=0.7,
                        y_min=-0.75,
                        y_max=-0.35,
                        z_min=-0.35,
                        z_max=1.5,
                    ),
                    "rear": ZoneBoundsConfig(
                        x_min=-0.6,
                        x_max=-0.1,
                        y_min=-0.35,
                        y_max=0.35,
                        z_min=-0.35,
                        z_max=1.5,
                    ),
                    "rotation": ZoneBoundsConfig(
                        x_min=-0.45,
                        x_max=0.45,
                        y_min=-0.55,
                        y_max=0.55,
                        z_min=-0.25,
                        z_max=1.3,
                    ),
                },
            ),
        )
