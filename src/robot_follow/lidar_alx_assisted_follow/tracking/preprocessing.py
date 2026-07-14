"""Point cloud preprocessing for target tracking."""

from __future__ import annotations

from typing import Tuple

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    PrototypeConfig,
    TargetConfig,
    CalibrationConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain import PreprocessedCloud


def preprocess_tracking_cloud(
    points_xyz: np.ndarray,
    config: TargetConfig,
    calibration_config: CalibrationConfig,
) -> PreprocessedCloud:
    """Preprocess raw LiDAR points for target tracking.

    Steps:
    1. Filter non-finite points
    2. Filter by radial distance [radial_min_m, radial_max_m]
    3. Filter by physical height [physical_height_min_m, physical_height_max_m]
    4. Exclude self box [self_x_min_m, self_x_max_m] x [-self_abs_y_max_m, self_abs_y_max_m]
    5. Deterministic 5cm voxel downsample
    6. Record source indices (lowest raw index in each voxel)

    Parameters
    ----------
    points_xyz : np.ndarray
        Raw LiDAR points (N, 3) in body frame.
    config : TargetConfig
        Target filtering configuration.
    calibration_config : CalibrationConfig
        Calibration configuration for height filtering.

    Returns
    -------
    PreprocessedCloud
        Downsampled points and source indices.
    """
    if points_xyz.size == 0:
        return PreprocessedCloud(
            points_xyz=np.empty((0, 3), dtype=np.float64),
            source_indices=np.array([], dtype=np.int64),
        )

    # Step 1: Filter non-finite points
    finite_mask = np.all(np.isfinite(points_xyz), axis=1)
    finite_indices = np.where(finite_mask)[0]

    if finite_indices.size == 0:
        return PreprocessedCloud(
            points_xyz=np.empty((0, 3), dtype=np.float64),
            source_indices=np.array([], dtype=np.int64),
        )

    points = points_xyz[finite_indices]

    # Step 2: Filter by radial distance
    radii = np.hypot(points[:, 0], points[:, 1])
    radial_mask = (radii >= config.radial_min_m) & (radii <= config.radial_max_m)

    # Step 3: Filter by physical height
    # Physical height = z + lidar_height_m
    physical_heights = points[:, 2] + calibration_config.lidar_height_m
    height_mask = (physical_heights >= config.physical_height_min_m) & (
        physical_heights <= config.physical_height_max_m
    )

    # Step 4: Exclude self box
    # Self box: [self_x_min_m, self_x_max_m] x [-self_abs_y_max_m, self_abs_y_max_m]
    in_self_x = (points[:, 0] >= config.self_x_min_m) & (points[:, 0] <= config.self_x_max_m)
    in_self_y = np.abs(points[:, 1]) <= config.self_abs_y_max_m
    in_self_box = in_self_x & in_self_y
    not_self_mask = ~in_self_box

    # Combine all filters
    combined_mask = radial_mask & height_mask & not_self_mask
    filtered_indices = np.where(combined_mask)[0]

    if filtered_indices.size == 0:
        return PreprocessedCloud(
            points_xyz=np.empty((0, 3), dtype=np.float64),
            source_indices=np.array([], dtype=np.int64),
        )

    filtered_points = points[filtered_indices]

    # Step 5: Deterministic voxel downsample
    # Voxel grid: compute voxel coordinates
    voxel_size = config.voxel_size_m
    voxel_coords = np.floor(filtered_points / voxel_size).astype(np.int64)

    # Create unique voxel keys
    # Key = (vx, vy, vz) packed into a single 64-bit integer for sorting
    # Use bit shifting: vx in bits 0-20, vy in bits 21-41, vz in bits 42-62
    # This supports coordinates up to ~1M voxels in each dimension
    vx = voxel_coords[:, 0]
    vy = voxel_coords[:, 1]
    vz = voxel_coords[:, 2]

    # Shift to non-negative range for stable packing
    vx_shifted = vx - vx.min()
    vy_shifted = vy - vy.min()
    vz_shifted = vz - vz.min()

    # Pack into 64-bit keys (21 bits per dimension)
    packed_keys = (
        (vx_shifted & 0x1FFFFF)
        | ((vy_shifted & 0x1FFFFF) << 21)
        | ((vz_shifted & 0x1FFFFF) << 42)
    )

    # Find unique voxels - sort by key and then by original index
    # We want the LOWEST raw index in each voxel
    # Sort by (packed_key, original_index)
    sort_indices = np.lexsort((filtered_indices, packed_keys))

    # Walk through sorted indices and pick first occurrence of each voxel
    unique_mask = np.ones(len(packed_keys), dtype=bool)
    unique_mask[1:] = packed_keys[sort_indices[1:]] != packed_keys[sort_indices[:-1]]

    # Select downsampled points and source indices
    downsampled_indices = sort_indices[unique_mask]

    # Map back to original indices
    source_indices = finite_indices[filtered_indices[downsampled_indices]]
    downsampled_points = filtered_points[downsampled_indices]

    # Make read-only
    downsampled_points = downsampled_points.copy()
    downsampled_points.flags.writeable = False

    source_indices = source_indices.copy()
    source_indices.flags.writeable = False

    return PreprocessedCloud(
        points_xyz=downsampled_points,
        source_indices=source_indices,
    )