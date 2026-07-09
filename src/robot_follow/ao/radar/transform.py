from __future__ import annotations

import math

import numpy as np

from robot_follow.ao.app.config import TransformConfig
from robot_follow.ao.core.geometry import ensure_points_xyz
from robot_follow.ao.core.types import PointCloudFrame


def rotation_matrix_from_rpy_deg(*, roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_points(points, *, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    pts = ensure_points_xyz(points)
    rot = np.asarray(rotation, dtype=np.float64)
    trans = np.asarray(translation, dtype=np.float64)
    if rot.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    if trans.shape != (3,):
        raise ValueError("translation must have shape (3,)")
    return (pts @ rot.T) + trans


def transform_point_cloud_frame(frame: PointCloudFrame, config: TransformConfig) -> PointCloudFrame:
    if not frame.valid:
        return PointCloudFrame.invalid(
            timestamp=frame.timestamp,
            frame_id=config.target_frame,
            reason=frame.reason,
        )
    points = ensure_points_xyz(frame.points_xyz)
    if config.apply_static_transform:
        rotation = rotation_matrix_from_rpy_deg(
            roll_deg=config.roll_deg,
            pitch_deg=config.pitch_deg,
            yaw_deg=config.yaw_deg,
        )
        translation = np.array([config.x_m, config.y_m, config.z_m], dtype=np.float64)
        points = transform_points(points, rotation=rotation, translation=translation)
    return PointCloudFrame(
        timestamp=frame.timestamp,
        frame_id=config.target_frame,
        points_xyz=points,
        valid=True,
        empty_is_clear=frame.empty_is_clear,
        reason="",
    )
