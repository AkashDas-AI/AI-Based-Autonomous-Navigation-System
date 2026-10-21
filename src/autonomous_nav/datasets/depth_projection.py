"""Depth image parsing, 3D point cloud projection, and 2D occupancy grid mapping.

Implements pinhole camera back-projection for 16-bit PNG depth maps,
spatial coordinate transformations (optical frame -> body frame -> world frame),
and obstacle height slicing into the existing discrete GridMap representation.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple
import cv2
import numpy as np

from autonomous_nav.datasets.config import CameraExtrinsics, CameraIntrinsics
from autonomous_nav.datasets.groundtruth import GroundTruthPose
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotState


def load_depth_image(filepath: Path | str) -> np.ndarray:
    """Load a 16-bit single-channel depth PNG image.

    Args:
        filepath: Path to the .png depth image.

    Returns:
        np.ndarray: 2D array of uint16 depth values.

    Raises:
        FileNotFoundError: If image file does not exist.
        ValueError: If file is corrupted or not a 16-bit single-channel depth image.
    """
    path = Path(filepath)
    if not path.is_file():
        raise FileNotFoundError(f"Depth image file not found: {path.resolve()}")

    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f"Failed to decode image file (corrupted or invalid format): {path.resolve()}")

    if img.ndim != 2:
        raise ValueError(f"Depth image must be single-channel 2D array, got shape {img.shape}")

    if img.dtype != np.uint16:
        raise ValueError(f"Depth image must be uint16 (16-bit integer), got dtype {img.dtype}")

    return img


def depth_to_pointcloud_camera(
    depth_img: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 5000.0,
    min_depth: float = 0.4,
    max_depth: float = 5.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project a 2D depth image into a 3D point cloud in the camera optical frame.

    Pinhole projection model:
        Z_c = depth_val / depth_scale (meters)
        X_c = (u - cx) * Z_c / fx
        Y_c = (v - cy) * Z_c / fy

    Args:
        depth_img: 2D uint16 array of depth measurements.
        intrinsics: Pinhole camera intrinsic calibration parameters.
        depth_scale: Divisor converting raw integer depth to meters (default 5000.0).
        min_depth: Minimum valid depth in meters (discard near-field noise).
        max_depth: Maximum valid depth in meters (discard far-field uncertainty).

    Returns:
        Tuple[np.ndarray, np.ndarray]:
            - points_c: (N, 3) float64 array of 3D points [X_c, Y_c, Z_c] in camera optical coordinates.
            - valid_mask: (H, W) boolean mask of valid pixels.
    """
    if depth_img.ndim != 2:
        raise ValueError(f"Expected 2D depth image, got shape {depth_img.shape}")
    if depth_scale <= 0.0:
        raise ValueError(f"depth_scale must be positive, got {depth_scale}")

    h, w = depth_img.shape
    # Convert raw uint16 to metric depth in meters
    z_metric = depth_img.astype(np.float64) / depth_scale

    # Valid mask: non-zero, within range, and finite
    valid_mask = (z_metric >= min_depth) & (z_metric <= max_depth) & np.isfinite(z_metric)

    if not np.any(valid_mask):
        return np.empty((0, 3), dtype=np.float64), valid_mask

    # Pixel coordinate grid
    u_grid, v_grid = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))

    u_valid = u_grid[valid_mask]
    v_valid = v_grid[valid_mask]
    z_valid = z_metric[valid_mask]

    x_c = (u_valid - intrinsics.cx) * z_valid / intrinsics.fx
    y_c = (v_valid - intrinsics.cy) * z_valid / intrinsics.fy

    points_c = np.column_stack((x_c, y_c, z_valid))
    return points_c, valid_mask


def transform_camera_to_body(
    points_camera: np.ndarray,
    extrinsics: CameraExtrinsics,
) -> np.ndarray:
    """Transform 3D points from optical camera coordinates into robot body coordinates (base_link).

    Coordinate convention mappings:
        Optical Frame: +X Right, +Y Down, +Z Forward (viewing axis)
        Body Frame (REP-105): +X Forward, +Y Left, +Z Up

    Canonical baseline mapping without tilt:
        X_body = +Z_camera + dx
        Y_body = -X_camera + dy
        Z_body = -Y_camera + dz

    Args:
        points_camera: (N, 3) array of [X_c, Y_c, Z_c] points.
        extrinsics: Camera mounting offset and orientation angles.

    Returns:
        np.ndarray: (N, 3) array of [X_b, Y_b, Z_b] points in robot body frame.
    """
    if len(points_camera) == 0:
        return np.empty((0, 3), dtype=np.float64)

    xc = points_camera[:, 0]
    yc = points_camera[:, 1]
    zc = points_camera[:, 2]

    # Base transformation: optical to body
    xb = zc
    yb = -xc
    zb = -yc

    # Apply camera pitch rotation if specified (tilt up/down about lateral Y_b axis)
    if abs(extrinsics.pitch_deg) > 1e-4:
        pitch_rad = math.radians(extrinsics.pitch_deg)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        xb_rot = xb * cos_p + zb * sin_p
        zb_rot = -xb * sin_p + zb * cos_p
        xb, zb = xb_rot, zb_rot

    # Apply camera roll rotation if specified (banking left/right about longitudinal X_b axis)
    if abs(extrinsics.roll_deg) > 1e-4:
        roll_rad = math.radians(extrinsics.roll_deg)
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        yb_rot = yb * cos_r - zb * sin_r
        zb_rot = yb * sin_r + zb * cos_r
        yb, zb = yb_rot, zb_rot

    # Apply camera yaw rotation if specified (panning left/right about vertical Z_b axis)
    if abs(extrinsics.yaw_deg) > 1e-4:
        yaw_rad = math.radians(extrinsics.yaw_deg)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        xb_rot = xb * cos_y - yb * sin_y
        yb_rot = xb * sin_y + yb * cos_y
        xb, yb = xb_rot, yb_rot

    # Add translation offsets
    xb = xb + extrinsics.dx
    yb = yb + extrinsics.dy
    zb = zb + extrinsics.dz

    return np.column_stack((xb, yb, zb))


def filter_obstacle_height(
    points_body: np.ndarray,
    min_height: float = 0.05,
    max_height: float = 1.20,
) -> np.ndarray:
    """Filter 3D body points to retain only returns corresponding to obstacle vertical heights.

    Discards the floor/ground plane (Z_body < min_height) and ceiling/overhead fixtures (Z_body > max_height).

    Args:
        points_body: (N, 3) array of [X_b, Y_b, Z_b] points in robot body frame.
        min_height: Floor elevation threshold in meters (default 0.05m).
        max_height: Ceiling/overhang threshold in meters (default 1.20m).

    Returns:
        np.ndarray: (M, 3) array of obstacle points within the specified vertical window.
    """
    if len(points_body) == 0:
        return np.empty((0, 3), dtype=np.float64)

    z_vals = points_body[:, 2]
    mask = (z_vals >= min_height) & (z_vals <= max_height) & np.isfinite(z_vals)
    return points_body[mask]


def project_body_to_world(
    points_body: np.ndarray,
    robot_pose: GroundTruthPose | RobotState,
) -> np.ndarray:
    """Project 3D body points into 2D world plane (X_world, Y_world) using robot pose.

    Planar transform:
        X_w = robot_x + X_b * cos(yaw) - Y_b * sin(yaw)
        Y_w = robot_y + X_b * sin(yaw) + Y_b * cos(yaw)

    Args:
        points_body: (N, 3) or (N, 2) array of obstacle points in body frame.
        robot_pose: GroundTruthPose or RobotState supplying (x, y, yaw).

    Returns:
        np.ndarray: (N, 2) array of [X_w, Y_w] points in world coordinates.
    """
    if len(points_body) == 0:
        return np.empty((0, 2), dtype=np.float64)

    if isinstance(robot_pose, GroundTruthPose):
        rx, ry = robot_pose.tx, robot_pose.ty
        yaw = robot_pose.yaw
    elif isinstance(robot_pose, RobotState):
        rx, ry = robot_pose.x, robot_pose.y
        yaw = robot_pose.theta
    else:
        raise TypeError(f"Unsupported pose type: {type(robot_pose)}")

    xb = points_body[:, 0]
    yb = points_body[:, 1]

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    xw = rx + xb * cos_yaw - yb * sin_yaw
    yw = ry + xb * sin_yaw + yb * cos_yaw

    return np.column_stack((xw, yw))


def insert_points_into_grid_map(
    points_world: np.ndarray,
    grid_map: GridMap,
    return_diagnostics: bool = False,
) -> int | Tuple[int, int, int]:
    """Rasterize 2D world obstacle points (point hits) into discrete GridMap occupancy cells.

    GRID SEMANTICS DISCLOSURE:
    This function performs discrete obstacle endpoint rasterization (point hits).
    It marks cells containing obstacle points as occupied (`grid == 1`).
    It does NOT perform line-of-sight ray tracing or free-space clearing between the
    sensor origin and the obstacle hit, nor does it update probabilistic log-odds.
    Unpopulated cells (`grid == 0`) represent unmarked/unobserved cells, NOT confirmed
    traversable free space.

    Args:
        points_world: (N, 2) array of world coordinate [X_w, Y_w] obstacle hits.
        grid_map: Destination GridMap to update.
        return_diagnostics: If True, return (new_cells, points_in_bounds, points_clipped).

    Returns:
        int | Tuple[int, int, int]: Count of newly marked occupied cells,
        or tuple of (new_cells, points_in_bounds, points_clipped) if return_diagnostics is True.
    """
    if len(points_world) == 0:
        return (0, 0, 0) if return_diagnostics else 0

    new_cells = 0
    points_in_bounds = 0
    points_clipped = 0

    for i in range(len(points_world)):
        xw = float(points_world[i, 0])
        yw = float(points_world[i, 1])

        col, row = grid_map.world_to_grid(xw, yw)
        if grid_map.is_valid(col, row):
            points_in_bounds += 1
            if not grid_map.is_occupied(col, row):
                grid_map.set_occupied(col, row, occupied=True)
                new_cells += 1
        else:
            points_clipped += 1

    if return_diagnostics:
        return new_cells, points_in_bounds, points_clipped
    return new_cells

