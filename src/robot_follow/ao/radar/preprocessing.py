from __future__ import annotations

from typing import Optional

import numpy as np

from robot_follow.ao.app.config import RoiConfig, SelfFilterConfig
from robot_follow.ao.core.geometry import AxisAlignedBox, ensure_points_xyz
from robot_follow.ao.core.types import PointCloudFrame


def filter_finite_points(points) -> np.ndarray:
    pts = ensure_points_xyz(points)
    return pts[np.isfinite(pts).all(axis=1)]


def crop_roi(points, roi: RoiConfig) -> np.ndarray:
    pts = ensure_points_xyz(points)
    box = AxisAlignedBox(
        roi.x_min,
        roi.x_max,
        roi.y_min,
        roi.y_max,
        roi.z_min,
        roi.z_max,
    )
    return pts[box.contains(pts)]


def remove_ground_points(points, roi: RoiConfig) -> np.ndarray:
    pts = ensure_points_xyz(points)
    return pts[pts[:, 2] > roi.ground_z_max]


def remove_self_points(points, self_filter: Optional[SelfFilterConfig]) -> np.ndarray:
    pts = ensure_points_xyz(points)
    if self_filter is None or not self_filter.enabled:
        return pts
    box = AxisAlignedBox(
        self_filter.x_min,
        self_filter.x_max,
        self_filter.y_min,
        self_filter.y_max,
        self_filter.z_min,
        self_filter.z_max,
    )
    return pts[~box.contains(pts)]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Pure-numpy voxel grid downsampling.

    Each voxel of side *voxel_size* keeps exactly one point (the centroid of
    all points that fall into that voxel).  Returns an empty (0, 3) array if
    *points* is empty or *voxel_size* <= 0.
    """
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points

    # Integer voxel indices
    indices = np.floor(points / voxel_size).astype(np.int64)

    # Stable sort so the first-occurring point in each voxel wins
    order = np.lexsort(indices.T)
    indices = indices[order]
    points = points[order]

    # Mark first occurrence of each unique (ix, iy, iz) tuple
    diff = np.any(np.diff(indices, axis=0) != 0, axis=1)
    keep = np.concatenate(([True], diff))

    kept = points[keep]
    return np.ascontiguousarray(kept)


def preprocess_point_cloud(
    frame: PointCloudFrame,
    roi: RoiConfig,
    self_filter: Optional[SelfFilterConfig] = None,
    *,
    voxel_size: float = 0.0,
) -> PointCloudFrame:
    if not frame.valid:
        return frame

    points = filter_finite_points(frame.points_xyz)
    points = crop_roi(points, roi)
    points = remove_self_points(points, self_filter)
    points = remove_ground_points(points, roi)
    points = voxel_downsample(points, voxel_size)
    if points.shape[0] == 0:
        return PointCloudFrame.empty_clear(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
        )
    return PointCloudFrame(
        timestamp=frame.timestamp,
        frame_id=frame.frame_id,
        points_xyz=points,
        valid=True,
        empty_is_clear=False,
        reason="",
    )


def frame_age_s(frame: PointCloudFrame, *, now_s: float) -> float:
    # ROS message stamps commonly use wall/ROS time, while the live runtime
    # uses monotonic time.  Compare like-for-like clocks when the subscriber
    # recorded a monotonic receive timestamp.
    reference = frame.timestamp if frame.received_at_s is None else frame.received_at_s
    return float(now_s - reference)


def invalidate_stale_frame(
    frame: PointCloudFrame,
    *,
    now_s: float,
    timeout_s: float,
) -> PointCloudFrame:
    if frame_age_s(frame, now_s=now_s) <= timeout_s:
        return frame
    return PointCloudFrame.invalid(
        timestamp=frame.timestamp,
        frame_id=frame.frame_id,
        reason="pointcloud_timeout",
    )
