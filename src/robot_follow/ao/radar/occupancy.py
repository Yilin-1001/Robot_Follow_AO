from __future__ import annotations

import numpy as np

from robot_follow.ao.avoidance.zones import default_zone_bounds
from robot_follow.ao.core.geometry import ensure_points_xyz
from robot_follow.ao.core.types import ObstacleReport, PointCloudFrame, ZoneResult
from robot_follow.ao.app.config import AppConfig


def _zone_result(name, points, box, *, min_points, blocked_distance_m) -> ZoneResult:
    mask = box.contains(points)
    zone_points = points[mask]
    count = int(zone_points.shape[0])
    if count == 0:
        return ZoneResult.empty(name)
    distances = box.distance_xy(zone_points)
    min_distance = float(np.min(distances))
    min_x = float(np.min(zone_points[:, 0]))
    occupied = count >= min_points
    blocked = occupied and min_distance <= blocked_distance_m
    return ZoneResult(
        name=name,
        point_count=count,
        min_x_m=min_x,
        min_distance_m=min_distance,
        occupied=occupied,
        blocked=blocked,
    )


def build_obstacle_report(
    frame: PointCloudFrame,
    *,
    min_points=10,
    emergency_stop_distance_m=0.45,
    rotation_emergency_stop_distance_m=0.45,
    zone_bounds=None,
) -> ObstacleReport:
    if not frame.valid:
        return ObstacleReport.invalid(timestamp=frame.timestamp, reason=frame.reason)
    points = ensure_points_xyz(frame.points_xyz)
    if points.shape[0] == 0:
        return ObstacleReport.clear(timestamp=frame.timestamp)

    zones = default_zone_bounds() if zone_bounds is None else zone_bounds
    front = _zone_result(
        "front",
        points,
        zones["front"],
        min_points=min_points,
        blocked_distance_m=emergency_stop_distance_m,
    )
    left = _zone_result(
        "left",
        points,
        zones["left"],
        min_points=min_points,
        blocked_distance_m=emergency_stop_distance_m,
    )
    right = _zone_result(
        "right",
        points,
        zones["right"],
        min_points=min_points,
        blocked_distance_m=emergency_stop_distance_m,
    )
    rear = _zone_result(
        "rear",
        points,
        zones["rear"],
        min_points=min_points,
        blocked_distance_m=emergency_stop_distance_m,
    )
    rotation = _zone_result(
        "rotation",
        points,
        zones["rotation"],
        min_points=min_points,
        blocked_distance_m=rotation_emergency_stop_distance_m,
    )
    return ObstacleReport(
        timestamp=frame.timestamp,
        valid=True,
        front=front,
        left=left,
        right=right,
        rear=rear,
        rotation=rotation,
        emergency_stop=front.blocked
        or left.blocked
        or right.blocked
        or rear.blocked
        or rotation.blocked,
        reason="",
    )


def build_preprocessed_obstacle_report(
    frame: PointCloudFrame,
    *,
    config: AppConfig,
    now_s=None,
    min_points=None,
) -> ObstacleReport:
    from robot_follow.ao.radar.pipeline import process_radar_frame

    return process_radar_frame(
        frame,
        config=config,
        now_s=now_s,
        min_points=min_points,
    ).report
