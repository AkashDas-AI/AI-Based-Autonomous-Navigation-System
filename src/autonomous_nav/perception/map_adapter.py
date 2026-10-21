"""Integration adapter converting vision detections into GridMap occupancy updates."""

from __future__ import annotations

from typing import List, Tuple
import cv2
import numpy as np
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.perception.obstacle_detection import DetectedObstacle
from autonomous_nav.perception.vision import OrthographicCamera


class MapAdapter:
    """Bridges perception detections and GridMap representation with explicit coordinate transforms.

    Coordinate Mapping Pipeline:
        1. Image Pixel (u, v) in [0, W_img] x [0, H_img]
        2. Metric World (x, y) = camera.pixel_to_world(u, v)
        3. Discrete Grid (gx, gy) = grid_map.world_to_grid(x, y)
    """

    def __init__(self, camera: OrthographicCamera) -> None:
        self.camera = camera

    def update_from_detections(
        self,
        obstacles: List[DetectedObstacle],
        grid_map: GridMap,
        use_full_contour: bool = True,
    ) -> int:
        """Rasterize detected obstacle contours onto the target GridMap.

        Args:
            obstacles: List of DetectedObstacle instances from ObstacleDetector.
            grid_map: The GridMap instance to update in-place.
            use_full_contour: If True, uses the complete contour points to maximize
                footprint fidelity (e.g. for circular or irregular shapes).
                If False, uses polygon approximation.

        Returns:
            Number of newly occupied grid cells marked.
        """
        if not obstacles:
            return 0

        # Create temporary grid-sized mask
        temp_grid_mask = np.zeros((grid_map.height, grid_map.width), dtype=np.uint8)

        for obs in obstacles:
            raw_pts = obs.contour if use_full_contour else obs.polygon_approx
            pts_px = raw_pts.reshape(-1, 2)
            grid_pts: List[Tuple[int, int]] = []

            for u, v in pts_px:
                wx, wy = self.camera.pixel_to_world(float(u), float(v))
                gx, gy = grid_map.world_to_grid(wx, wy)
                # Keep exact integer coordinates. Do not clamp to boundary cells,
                # as artificial clamping causes off-grid obstacles to collapse onto edge cells.
                grid_pts.append((int(gx), int(gy)))

            if len(grid_pts) >= 3:
                poly_arr = np.array(grid_pts, dtype=np.int32).reshape((-1, 1, 2))
                # cv2.fillPoly naturally clips polygons to the destination raster boundaries
                cv2.fillPoly(temp_grid_mask, [poly_arr], 1)
            elif len(grid_pts) in (1, 2):
                # Mark sparse points only if strictly inside grid bounds
                for gx, gy in grid_pts:
                    if grid_map.is_in_bounds(gx, gy):
                        temp_grid_mask[gy, gx] = 1

        # Calculate newly marked cells
        new_cells = int(np.sum((grid_map.grid == 0) & (temp_grid_mask == 1)))

        # Update grid_map in-place
        grid_map.grid = np.bitwise_or(grid_map.grid, temp_grid_mask)

        return new_cells

    def update_from_binary_mask(
        self,
        binary_mask: np.ndarray,
        grid_map: GridMap,
        threshold: int = 127,
    ) -> int:
        """Project an entire binary obstacle mask onto the GridMap via spatial affine warp.

        This projection maps pixel coordinates (u, v) into continuous metric world coordinates
        and subsequently into discrete grid coordinates (gx, gy), properly handling arbitrary
        camera positions, non-identical extents, scale discrepancies, and vertical flips.
        """
        if binary_mask.shape != (self.camera.image_height, self.camera.image_width):
            raise ValueError(
                f"Binary mask shape {binary_mask.shape} does not match camera dimensions "
                f"({self.camera.image_height}, {self.camera.image_width})"
            )

        delta = grid_map.resolution
        sx = self.camera.scale_x
        sy = self.camera.scale_y
        m02 = (self.camera.world_x_min - grid_map.origin[0]) / delta
        m12 = (self.camera.world_y_max - grid_map.origin[1]) / delta

        M = np.array([
            [1.0 / (sx * delta), 0.0, m02],
            [0.0, -1.0 / (sy * delta), m12],
        ], dtype=np.float32)

        bin_thresh = (binary_mask >= threshold).astype(np.uint8)
        warped = cv2.warpAffine(
            bin_thresh,
            M,
            (grid_map.width, grid_map.height),
            flags=cv2.INTER_NEAREST,
        )

        new_cells = int(np.sum((grid_map.grid == 0) & (warped == 1)))
        grid_map.grid = np.bitwise_or(grid_map.grid, warped)
        return new_cells

