"""Real-world robotics benchmark dataset ingestion and evaluation subsystem.

Provides modular interfaces for standard RGB-D benchmark datasets (TUM RGB-D),
including camera intrinsic/extrinsic models, high-frequency motion-capture ground truth,
pinhole point cloud back-projection, obstacle height slicing, and GridMap rasterization.
"""

from __future__ import annotations

from autonomous_nav.datasets.adapter import TUMRGBDAdapter
from autonomous_nav.datasets.config import CameraExtrinsics, CameraIntrinsics, DatasetConfig
from autonomous_nav.datasets.depth_projection import (
    depth_to_pointcloud_camera,
    filter_obstacle_height,
    insert_points_into_grid_map,
    load_depth_image,
    project_body_to_world,
    transform_camera_to_body,
)
from autonomous_nav.datasets.groundtruth import GroundTruthPose, GroundTruthTrajectory

__all__ = [
    "CameraExtrinsics",
    "CameraIntrinsics",
    "DatasetConfig",
    "GroundTruthPose",
    "GroundTruthTrajectory",
    "TUMRGBDAdapter",
    "depth_to_pointcloud_camera",
    "filter_obstacle_height",
    "insert_points_into_grid_map",
    "load_depth_image",
    "project_body_to_world",
    "transform_camera_to_body",
]

