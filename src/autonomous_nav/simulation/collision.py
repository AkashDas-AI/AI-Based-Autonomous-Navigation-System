"""Collision detection algorithms for 2D robotic simulation."""

from __future__ import annotations

import math
from typing import List, Tuple
import numpy as np
from autonomous_nav.environment.grid_map import GridMap


class CollisionChecker:
    """Evaluates geometric collisions between robot footprints and occupancy grids."""

    @staticmethod
    def check_point_collision(x: float, y: float, grid_map: GridMap) -> bool:
        """Check if point (x, y) in world coordinates is inside an obstacle or out of bounds."""
        return grid_map.is_occupied_world(x, y)

    @staticmethod
    def check_disc_collision(
        x: float,
        y: float,
        radius: float,
        grid_map: GridMap,
    ) -> bool:
        """Check if a circular disc robot footprint of given radius centered at (x, y) collides with an obstacle."""
        if radius <= 0.0:
            return CollisionChecker.check_point_collision(x, y, grid_map)

        res = grid_map.resolution
        origin_x, origin_y = grid_map.origin
        cgx = int((x - origin_x) / res)
        cgy = int((y - origin_y) / res)
        radius_cells = int(math.ceil(radius / res))

        gx_start = cgx - radius_cells
        gx_end = cgx + radius_cells
        gy_start = cgy - radius_cells
        gy_end = cgy + radius_cells

        # Boundary check: disc extending outside map boundaries is in collision
        if gx_start < 0 or gx_end >= grid_map.width or gy_start < 0 or gy_end >= grid_map.height:
            return True

        # Fast vector slice check: if bounding box contains zero occupied cells, early return False
        sub_grid = grid_map.grid[gy_start : gy_end + 1, gx_start : gx_end + 1]
        if not np.any(sub_grid):
            return False

        radius_sq = radius ** 2
        # Check exact circle-to-AABB distance on occupied cells in the sub-grid
        for gy in range(gy_start, gy_end + 1):
            for gx in range(gx_start, gx_end + 1):
                if grid_map.grid[gy, gx] == 1:
                    x_cell_min = origin_x + gx * res
                    x_cell_max = x_cell_min + res
                    y_cell_min = origin_y + gy * res
                    y_cell_max = y_cell_min + res

                    closest_x = max(x_cell_min, min(x, x_cell_max))
                    closest_y = max(y_cell_min, min(y, y_cell_max))

                    dist_sq = (x - closest_x) ** 2 + (y - closest_y) ** 2
                    if dist_sq <= radius_sq:
                        return True

        return False

    @staticmethod
    def check_path_collision(
        path: List[Tuple[float, float]],
        radius: float,
        grid_map: GridMap,
    ) -> bool:
        """Check if any point along a sequence of world waypoints collides with obstacles."""
        for wx, wy in path:
            if CollisionChecker.check_disc_collision(wx, wy, radius, grid_map):
                return True
        return False

    @staticmethod
    def distance_to_nearest_obstacle(
        x: float,
        y: float,
        grid_map: GridMap,
        search_radius: float = 5.0,
    ) -> float:
        """Calculate metric distance from (x, y) to closest occupied grid cell within search_radius."""
        res = grid_map.resolution
        origin_x, origin_y = grid_map.origin
        cgx = int((x - origin_x) / res)
        cgy = int((y - origin_y) / res)
        radius_cells = int(math.ceil(search_radius / res))

        gx_start = max(0, cgx - radius_cells)
        gx_end = min(grid_map.width - 1, cgx + radius_cells)
        gy_start = max(0, cgy - radius_cells)
        gy_end = min(grid_map.height - 1, cgy + radius_cells)

        if gx_start > gx_end or gy_start > gy_end:
            return search_radius

        sub_grid = grid_map.grid[gy_start : gy_end + 1, gx_start : gx_end + 1]
        if not np.any(sub_grid):
            return search_radius

        min_dist_sq = search_radius ** 2
        found = False

        for gy in range(gy_start, gy_end + 1):
            for gx in range(gx_start, gx_end + 1):
                if grid_map.grid[gy, gx] == 1:
                    x_cell_min = origin_x + gx * res
                    x_cell_max = x_cell_min + res
                    y_cell_min = origin_y + gy * res
                    y_cell_max = y_cell_min + res

                    closest_x = max(x_cell_min, min(x, x_cell_max))
                    closest_y = max(y_cell_min, min(y, y_cell_max))

                    d_sq = (x - closest_x) ** 2 + (y - closest_y) ** 2
                    if d_sq < min_dist_sq:
                        min_dist_sq = d_sq
                        found = True

        return math.sqrt(min_dist_sq) if found else search_radius

    @staticmethod
    def relative_continuous_distance(
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        o0: Tuple[float, float],
        o1: Tuple[float, float],
    ) -> float:
        """Calculate exact minimum Euclidean distance between two moving discs over a time step interval.

        Converts relative motion into line segment in difference coordinates and computes
        orthogonal projection from the origin (0, 0) to eliminate temporal tunneling.
        """
        ax = p0[0] - o0[0]
        ay = p0[1] - o0[1]
        bx = p1[0] - o1[0]
        by = p1[1] - o1[1]
        vx = bx - ax
        vy = by - ay
        l2 = vx * vx + vy * vy
        if l2 < 1e-12:
            return math.hypot(ax, ay)
        t = -(ax * vx + ay * vy) / l2
        t = max(0.0, min(1.0, t))
        cx = ax + t * vx
        cy = ay + t * vy
        return math.hypot(cx, cy)
