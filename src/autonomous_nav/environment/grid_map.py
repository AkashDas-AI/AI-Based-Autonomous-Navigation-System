"""2D Occupancy Grid Map implementation for autonomous navigation simulation."""

from __future__ import annotations

import math
from typing import Tuple, Optional
import numpy as np


class GridMap:
    """Represents a 2D discrete occupancy grid map.

    Attributes:
        width: Number of horizontal cells (columns).
        height: Number of vertical cells (rows).
        resolution: Physical dimension of each cell in meters.
        origin: World coordinates (x, y) corresponding to cell (0, 0).
        grid: 2D NumPy array of shape (height, width), where 0 = free, 1 = occupied.
    """

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float = 0.5,
        origin: Tuple[float, float] = (0.0, 0.0),
        data: Optional[np.ndarray] = None,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError(f"Width and height must be positive integers, got ({width}, {height})")
        if resolution <= 0.0:
            raise ValueError(f"Resolution must be positive, got {resolution}")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

        if data is not None:
            if data.shape != (self.height, self.width):
                raise ValueError(
                    f"Provided data shape {data.shape} does not match (height, width) "
                    f"({self.height}, {self.width})"
                )
            self.grid = (data > 0).astype(np.uint8)
        else:
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous metric world coordinates (x, y) to discrete grid indices (gx, gy)."""
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert discrete grid cell indices (gx, gy) to continuous world coordinates at cell center."""
        x = self.origin[0] + (gx + 0.5) * self.resolution
        y = self.origin[1] + (gy + 0.5) * self.resolution
        return x, y

    def is_valid(self, gx: int, gy: int) -> bool:
        """Check whether discrete grid cell (gx, gy) lies within map bounds."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx: int, gy: int) -> bool:
        """Return True if cell (gx, gy) is out of bounds or occupied."""
        if not self.is_valid(gx, gy):
            return True
        return bool(self.grid[gy, gx] == 1)

    def is_occupied_world(self, x: float, y: float) -> bool:
        """Return True if continuous world position (x, y) falls inside an obstacle or out of bounds."""
        gx, gy = self.world_to_grid(x, y)
        return self.is_occupied(gx, gy)

    def set_obstacle(self, gx: int, gy: int, occupied: bool = True) -> None:
        """Set occupancy status of grid cell (gx, gy)."""
        if self.is_valid(gx, gy):
            self.grid[gy, gx] = 1 if occupied else 0

    def set_occupied(self, gx: int, gy: int, occupied: bool = True) -> None:
        """Alias for set_obstacle for semantic consistency with is_occupied."""
        self.set_obstacle(gx, gy, occupied)

    def set_obstacle_world(self, x: float, y: float, occupied: bool = True) -> None:
        """Set occupancy status of grid cell containing world coordinate (x, y)."""
        gx, gy = self.world_to_grid(x, y)
        self.set_obstacle(gx, gy, occupied)

    def add_rectangular_obstacle(
        self,
        x_min: float,
        y_min: float,
        x_max: float,
        y_max: float,
    ) -> None:
        """Add an axis-aligned rectangular obstacle in world coordinates."""
        gx_min, gy_min = self.world_to_grid(min(x_min, x_max), min(y_min, y_max))
        gx_max, gy_max = self.world_to_grid(max(x_min, x_max), max(y_min, y_max))

        gx_start = max(0, gx_min)
        gx_end = min(self.width - 1, gx_max)
        gy_start = max(0, gy_min)
        gy_end = min(self.height - 1, gy_max)

        self.grid[gy_start : gy_end + 1, gx_start : gx_end + 1] = 1

    def add_circular_obstacle(self, center_x: float, center_y: float, radius: float) -> None:
        """Add a solid circular obstacle in world coordinates."""
        radius_cells = int(math.ceil(radius / self.resolution))
        cgx, cgy = self.world_to_grid(center_x, center_y)

        gx_start = max(0, cgx - radius_cells)
        gx_end = min(self.width - 1, cgx + radius_cells)
        gy_start = max(0, cgy - radius_cells)
        gy_end = min(self.height - 1, cgy + radius_cells)

        for gy in range(gy_start, gy_end + 1):
            for gx in range(gx_start, gx_end + 1):
                wx, wy = self.grid_to_world(gx, gy)
                if (wx - center_x) ** 2 + (wy - center_y) ** 2 <= radius ** 2:
                    self.grid[gy, gx] = 1

    def inflate_obstacles(self, radius_meters: float) -> GridMap:
        """Return a new GridMap with obstacles dilated by the specified safety clearance radius."""
        if radius_meters <= 0.0:
            return GridMap(self.width, self.height, self.resolution, self.origin, self.grid.copy())

        inflated_grid = self.grid.copy()
        radius_cells = int(math.ceil(radius_meters / self.resolution))

        occupied_coords = np.argwhere(self.grid == 1)
        # occupied_coords has elements [gy, gx]
        for gy, gx in occupied_coords:
            y_start = max(0, gy - radius_cells)
            y_end = min(self.height, gy + radius_cells + 1)
            x_start = max(0, gx - radius_cells)
            x_end = min(self.width, gx + radius_cells + 1)

            for ny in range(y_start, y_end):
                for nx in range(x_start, x_end):
                    dist_sq = ((nx - gx) ** 2 + (ny - gy) ** 2) * (self.resolution ** 2)
                    if dist_sq <= radius_meters ** 2:
                        inflated_grid[ny, nx] = 1

        return GridMap(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin=self.origin,
            data=inflated_grid,
        )

    def to_numpy(self) -> np.ndarray:
        """Return a copy of the underlying binary occupancy array."""
        return self.grid.copy()

    @classmethod
    def from_numpy(
        cls,
        array: np.ndarray,
        resolution: float = 0.5,
        origin: Tuple[float, float] = (0.0, 0.0),
    ) -> GridMap:
        """Construct a GridMap instance from a 2D NumPy array."""
        height, width = array.shape
        return cls(width=width, height=height, resolution=resolution, origin=origin, data=array)

