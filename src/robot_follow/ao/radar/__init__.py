"""Radar-independent point-cloud preprocessing and occupancy analysis."""
from robot_follow.ao.radar.occupancy import (
    build_obstacle_report,
    build_preprocessed_obstacle_report,
)
from robot_follow.ao.radar.pipeline import RadarPipelineResult, process_radar_frame
from robot_follow.ao.radar.preprocessing import (
    crop_roi,
    filter_finite_points,
    frame_age_s,
    invalidate_stale_frame,
    preprocess_point_cloud,
    remove_ground_points,
    remove_self_points,
)
from robot_follow.ao.radar.transform import (
    rotation_matrix_from_rpy_deg,
    transform_point_cloud_frame,
    transform_points,
)

__all__ = [
    "RadarPipelineResult",
    "build_obstacle_report",
    "build_preprocessed_obstacle_report",
    "crop_roi",
    "filter_finite_points",
    "frame_age_s",
    "invalidate_stale_frame",
    "preprocess_point_cloud",
    "process_radar_frame",
    "remove_ground_points",
    "remove_self_points",
    "rotation_matrix_from_rpy_deg",
    "transform_point_cloud_frame",
    "transform_points",
]
