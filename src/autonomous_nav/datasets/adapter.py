"""Modular dataset adapter for TUM RGB-D sequences.

Orchestrates directory inspection, ground truth parsing, depth map loading,
pinhole back-projection, spatial transformations, and occupancy grid mapping.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

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
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotState


class TUMRGBDAdapter:
    """Modular ingestion and processing adapter for TUM RGB-D dataset sequences.

    Provides end-to-end evaluation:
    1. Ground truth trajectory validation and interpolation.
    2. Synchronized depth image frame extraction.
    3. Depth-to-pointcloud back-projection using calibrated intrinsics.
    4. Obstacle height slicing and 2D GridMap rasterization.
    5. Quantitative trajectory comparison against ground truth.
    """

    def __init__(self, config: Optional[DatasetConfig] = None) -> None:
        self.config: DatasetConfig = config if config is not None else DatasetConfig()
        self._trajectory: Optional[GroundTruthTrajectory] = None
        self._depth_index: List[Tuple[float, str]] = []
        self._rgb_index: List[Tuple[float, str]] = []
        self._initialized: bool = False

    @property
    def dataset_dir(self) -> Path:
        return self.config.dataset_dir

    def discover_and_validate(self) -> Dict[str, Any]:
        """Inspect the dataset directory and validate available files.

        Returns:
            Dict[str, Any]: Status manifest detailing presence and paths of files.
        """
        root = self.dataset_dir
        manifest = {
            "dataset_dir": str(root.resolve()),
            "sequence_name": self.config.sequence_name,
            "has_dataset_dir": root.is_dir(),
            "has_groundtruth": False,
            "has_depth_txt": False,
            "has_rgb_txt": False,
            "has_depth_dir": False,
            "has_rgb_dir": False,
            "groundtruth_file": None,
            "depth_txt_file": None,
            "rgb_txt_file": None,
            "depth_images_count": 0,
            "rgb_images_count": 0,
        }

        if not root.is_dir():
            return manifest

        # Look for ground truth file (either groundtruth.txt or sequence-groundtruth.txt)
        gt_candidates = [
            root / "groundtruth.txt",
            root / f"{self.config.sequence_name}-groundtruth.txt",
            root / f"rgbd_dataset_{self.config.sequence_name}-groundtruth.txt",
        ]
        for candidate in gt_candidates:
            if candidate.is_file():
                manifest["has_groundtruth"] = True
                manifest["groundtruth_file"] = str(candidate.resolve())
                break

        # Check index files
        depth_txt = root / "depth.txt"
        if depth_txt.is_file():
            manifest["has_depth_txt"] = True
            manifest["depth_txt_file"] = str(depth_txt.resolve())

        rgb_txt = root / "rgb.txt"
        if rgb_txt.is_file():
            manifest["has_rgb_txt"] = True
            manifest["rgb_txt_file"] = str(rgb_txt.resolve())

        # Check image directories
        depth_dir = root / "depth"
        if depth_dir.is_dir():
            manifest["has_depth_dir"] = True
            manifest["depth_images_count"] = len(list(depth_dir.glob("*.png")))

        rgb_dir = root / "rgb"
        if rgb_dir.is_dir():
            manifest["has_rgb_dir"] = True
            manifest["rgb_images_count"] = len(list(rgb_dir.glob("*.png")))

        return manifest

    def load_groundtruth(self, filepath: Optional[Path | str] = None) -> GroundTruthTrajectory:
        """Load and cache the ground truth trajectory."""
        if filepath is not None:
            target = Path(filepath)
        else:
            manifest = self.discover_and_validate()
            if manifest["groundtruth_file"]:
                target = Path(manifest["groundtruth_file"])
            else:
                raise FileNotFoundError(
                    f"No groundtruth.txt found in dataset directory: {self.dataset_dir.resolve()}"
                )

        self._trajectory = GroundTruthTrajectory.load_from_file(target)
        return self._trajectory

    def get_trajectory(self) -> GroundTruthTrajectory:
        """Retrieve the cached ground truth trajectory or load default."""
        if self._trajectory is None:
            return self.load_groundtruth()
        return self._trajectory

    def load_index_file(self, filepath: Path | str) -> List[Tuple[float, str]]:
        """Parse a timestamp-to-filename index file (e.g., depth.txt or rgb.txt)."""
        path = Path(filepath)
        if not path.is_file():
            raise FileNotFoundError(f"Index file not found: {path.resolve()}")

        entries: List[Tuple[float, str]] = []
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                clean = line.strip()
                if not clean or clean.startswith("#"):
                    continue
                parts = clean.split()
                if len(parts) >= 2:
                    t = float(parts[0])
                    rel_path = parts[1]
                    entries.append((t, rel_path))

        return entries

    def process_depth_frame(
        self,
        depth_img: np.ndarray,
        robot_pose: GroundTruthPose | RobotState,
        grid_map: GridMap,
    ) -> Dict[str, Any]:
        """Project a single depth frame and insert obstacle returns into the GridMap.

        Args:
            depth_img: 16-bit uint16 depth image.
            robot_pose: Corresponding robot pose in world coordinates.
            grid_map: Destination GridMap to update.

        Returns:
            Dict[str, Any]: Diagnostics detailing points projected, filtered, and cells updated.
        """
        cfg = self.config
        total_pixels = int(depth_img.size)
        zero_depth_pixels = int(np.count_nonzero(depth_img == 0))
        z_metric = depth_img.astype(np.float64) / cfg.depth_scale
        near_field_pixels = int(np.count_nonzero((depth_img > 0) & (z_metric < cfg.min_depth)))
        far_field_pixels = int(np.count_nonzero((depth_img > 0) & (z_metric > cfg.max_depth)))
        out_of_range_pixels = near_field_pixels + far_field_pixels
        invalid_depth_pixels = zero_depth_pixels + out_of_range_pixels
        valid_depth_pixels = total_pixels - invalid_depth_pixels

        # 1. Back-project depth into 3D camera coordinates
        pts_c, valid_mask = depth_to_pointcloud_camera(
            depth_img=depth_img,
            intrinsics=cfg.intrinsics,
            depth_scale=cfg.depth_scale,
            min_depth=cfg.min_depth,
            max_depth=cfg.max_depth,
        )

        total_valid_pixels = len(pts_c)
        if total_valid_pixels == 0:
            return {
                "total_pixels": total_pixels,
                "zero_depth_pixels": zero_depth_pixels,
                "near_field_pixels": near_field_pixels,
                "far_field_pixels": far_field_pixels,
                "out_of_range_pixels": out_of_range_pixels,
                "invalid_depth_pixels": invalid_depth_pixels,
                "valid_depth_pixels": 0,
                "floor_rejected_points": 0,
                "ceiling_rejected_points": 0,
                "obstacle_points": 0,
                "points_in_bounds": 0,
                "points_clipped": 0,
                "grid_cells_updated": 0,
            }

        # 2. Transform camera optical frame -> robot body frame
        pts_body = transform_camera_to_body(points_camera=pts_c, extrinsics=cfg.extrinsics)

        # Height filtering breakdown
        zb = pts_body[:, 2]
        floor_rejected = int(np.count_nonzero((zb < cfg.min_obstacle_height) & np.isfinite(zb)))
        ceiling_rejected = int(np.count_nonzero((zb > cfg.max_obstacle_height) & np.isfinite(zb)))

        # 3. Filter obstacle heights
        pts_obs = filter_obstacle_height(
            points_body=pts_body,
            min_height=cfg.min_obstacle_height,
            max_height=cfg.max_obstacle_height,
        )

        total_obs_points = len(pts_obs)
        if total_obs_points == 0:
            return {
                "total_pixels": total_pixels,
                "zero_depth_pixels": zero_depth_pixels,
                "near_field_pixels": near_field_pixels,
                "far_field_pixels": far_field_pixels,
                "out_of_range_pixels": out_of_range_pixels,
                "invalid_depth_pixels": invalid_depth_pixels,
                "valid_depth_pixels": total_valid_pixels,
                "floor_rejected_points": floor_rejected,
                "ceiling_rejected_points": ceiling_rejected,
                "obstacle_points": 0,
                "points_in_bounds": 0,
                "points_clipped": 0,
                "grid_cells_updated": 0,
            }

        # 4. Project body obstacle points into world frame
        pts_w = project_body_to_world(points_body=pts_obs, robot_pose=robot_pose)

        # 5. Insert obstacle hits into GridMap with boundary clipping diagnostics
        cells_updated, pts_in_bounds, pts_clipped = insert_points_into_grid_map(
            points_world=pts_w, grid_map=grid_map, return_diagnostics=True
        )

        return {
            "total_pixels": total_pixels,
            "zero_depth_pixels": zero_depth_pixels,
            "near_field_pixels": near_field_pixels,
            "far_field_pixels": far_field_pixels,
            "out_of_range_pixels": out_of_range_pixels,
            "invalid_depth_pixels": invalid_depth_pixels,
            "valid_depth_pixels": total_valid_pixels,
            "floor_rejected_points": floor_rejected,
            "ceiling_rejected_points": ceiling_rejected,
            "obstacle_points": total_obs_points,
            "points_in_bounds": pts_in_bounds,
            "points_clipped": pts_clipped,
            "grid_cells_updated": cells_updated,
        }

    def evaluate_trajectory_tracking(
        self,
        tested_trajectory: List[RobotState],
        tolerance: float = 0.05,
    ) -> Dict[str, Any]:
        """Evaluate a test or estimated trajectory against the ground truth trajectory.

        Computes point-to-point Euclidean Absolute Position Error (APE) RMSE and MAE,
        Max Position Error, and Heading Error at synchronized timestamps.
        """
        gt_traj = self.get_trajectory()

        pos_errors = []
        heading_errors = []
        aligned_count = 0
        skipped_count = 0

        for state in tested_trajectory:
            t = state.timestamp
            gt_pose = gt_traj.get_pose_nearest(timestamp=t, tolerance=tolerance)
            if gt_pose is None:
                skipped_count += 1
                continue

            dx = state.x - gt_pose.tx
            dy = state.y - gt_pose.ty
            dist = math.hypot(dx, dy)
            pos_errors.append(dist)

            # Angular difference
            d_yaw = abs(math.atan2(math.sin(state.theta - gt_pose.yaw), math.cos(state.theta - gt_pose.yaw)))
            heading_errors.append(d_yaw)
            aligned_count += 1

        if not pos_errors:
            return {
                "aligned_samples": 0,
                "skipped_samples": skipped_count,
                "position_rmse_m": 0.0,
                "position_mae_m": 0.0,
                "max_position_error_m": 0.0,
                "heading_rmse_deg": 0.0,
            }

        pos_arr = np.array(pos_errors, dtype=np.float64)
        head_arr = np.array(heading_errors, dtype=np.float64)

        return {
            "aligned_samples": aligned_count,
            "skipped_samples": skipped_count,
            "position_rmse_m": round(float(np.sqrt(np.mean(pos_arr ** 2))), 4),
            "position_mae_m": round(float(np.mean(pos_arr)), 4),
            "max_position_error_m": round(float(np.max(pos_arr)), 4),
            "heading_rmse_rad": round(float(np.sqrt(np.mean(head_arr ** 2))), 4),
            "heading_rmse_deg": round(float(math.degrees(np.sqrt(np.mean(head_arr ** 2)))), 2),
        }

