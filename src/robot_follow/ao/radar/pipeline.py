from __future__ import annotations

from dataclasses import dataclass

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.core.types import ObstacleReport, PointCloudFrame
from robot_follow.ao.avoidance.zones import zone_bounds_from_config
from robot_follow.ao.radar.occupancy import build_obstacle_report
from robot_follow.ao.radar.preprocessing import invalidate_stale_frame, preprocess_point_cloud
from robot_follow.ao.radar.transform import transform_point_cloud_frame


@dataclass(frozen=True)
class RadarPipelineResult:
    raw_frame: PointCloudFrame
    transformed_frame: PointCloudFrame
    filtered_frame: PointCloudFrame
    report: ObstacleReport
    raw_count: int
    filtered_count: int
    zone_bounds: object


def process_radar_frame(
    frame: PointCloudFrame,
    *,
    config: AppConfig,
    now_s=None,
    min_points=None,
) -> RadarPipelineResult:
    checked = frame
    if now_s is not None:
        checked = invalidate_stale_frame(
            checked,
            now_s=now_s,
            timeout_s=config.radar.timeout_s,
        )
    transformed = transform_point_cloud_frame(checked, config.transform)
    filtered = preprocess_point_cloud(
        transformed,
        config.roi,
        config.self_filter,
        voxel_size=config.radar.voxel_size_m,
    )
    zone_bounds = zone_bounds_from_config(config.zones)
    report = build_obstacle_report(
        filtered,
        min_points=config.zones.min_points if min_points is None else min_points,
        emergency_stop_distance_m=config.zones.emergency_stop_distance_m,
        rotation_emergency_stop_distance_m=config.zones.rotation_emergency_stop_distance_m,
        zone_bounds=zone_bounds,
    )
    return RadarPipelineResult(
        raw_frame=frame,
        transformed_frame=transformed,
        filtered_frame=filtered,
        report=report,
        raw_count=int(frame.points_xyz.shape[0]),
        filtered_count=int(filtered.points_xyz.shape[0]),
        zone_bounds=zone_bounds,
    )
