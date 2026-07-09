from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def ensure_points_xyz(points) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    return arr


@dataclass(frozen=True)
class AxisAlignedBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def contains(self, points, *, include_max=False) -> np.ndarray:
        pts = ensure_points_xyz(points)
        if include_max:
            return (
                (pts[:, 0] >= self.x_min)
                & (pts[:, 0] <= self.x_max)
                & (pts[:, 1] >= self.y_min)
                & (pts[:, 1] <= self.y_max)
                & (pts[:, 2] >= self.z_min)
                & (pts[:, 2] <= self.z_max)
            )
        return (
            (pts[:, 0] >= self.x_min)
            & (pts[:, 0] < self.x_max)
            & (pts[:, 1] >= self.y_min)
            & (pts[:, 1] < self.y_max)
            & (pts[:, 2] >= self.z_min)
            & (pts[:, 2] < self.z_max)
        )

    def distance_xy(self, points) -> np.ndarray:
        pts = ensure_points_xyz(points)
        return np.sqrt((pts[:, 0] * pts[:, 0]) + (pts[:, 1] * pts[:, 1]))
