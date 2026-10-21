"""A* path planning implementation on 2D occupancy grid."""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple
from autonomous_nav.environment.grid_map import GridMap


class AStarPlanner:
    """A* Path Planning algorithm on 2D grid maps.

    Supports 4-connected and 8-connected neighbor expansion, Euclidean and
    Manhattan heuristics, and corner-cutting avoidance for diagonal transitions.
    """

    def __init__(
        self,
        allow_diagonal: bool = True,
        heuristic_type: str = "euclidean",
    ) -> None:
        self.allow_diagonal = allow_diagonal
        self.heuristic_type = heuristic_type.lower()

        # Motion primitives: (dx, dy, cost)
        if self.allow_diagonal:
            diag_cost = math.sqrt(2.0)
            self.motions = [
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (0, 1, 1.0),
                (0, -1, 1.0),
                (1, 1, diag_cost),
                (1, -1, diag_cost),
                (-1, 1, diag_cost),
                (-1, -1, diag_cost),
            ]
        else:
            self.motions = [
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (0, 1, 1.0),
                (0, -1, 1.0),
            ]

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic distance estimate from cell a to cell b."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        if self.heuristic_type == "manhattan":
            return float(dx + dy)
        elif self.heuristic_type == "diagonal":
            # Octile distance
            return float(dx + dy + (math.sqrt(2.0) - 2.0) * min(dx, dy))
        else:
            # Default Euclidean distance
            return math.hypot(dx, dy)

    def plan_grid(
        self,
        start_grid: Tuple[int, int],
        goal_grid: Tuple[int, int],
        grid_map: GridMap,
    ) -> Optional[List[Tuple[int, int]]]:
        """Find the shortest collision-free grid path from start_grid to goal_grid.

        Returns:
            List of (gx, gy) tuples from start to goal, or None if no valid path exists.
        """
        sx, sy = start_grid
        gx, gy = goal_grid

        # Check bounds and occupancy of start and goal
        if not grid_map.is_valid(sx, sy) or grid_map.is_occupied(sx, sy):
            return None
        if not grid_map.is_valid(gx, gy) or grid_map.is_occupied(gx, gy):
            return None

        if start_grid == goal_grid:
            return [start_grid]

        # Priority queue stores tuples of: (f_score, tie_breaker, (x, y))
        open_set: List[Tuple[float, int, Tuple[int, int]]] = []
        counter = 0

        g_score: Dict[Tuple[int, int], float] = {start_grid: 0.0}
        f_score: Dict[Tuple[int, int], float] = {start_grid: self.heuristic(start_grid, goal_grid)}

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        closed_set: Set[Tuple[int, int]] = set()

        heapq.heappush(open_set, (f_score[start_grid], counter, start_grid))

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_grid:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed_set:
                continue
            closed_set.add(current)

            cx, cy = current
            current_g = g_score[current]

            for dx, dy, move_cost in self.motions:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)

                if neighbor in closed_set:
                    continue

                if not grid_map.is_valid(nx, ny) or grid_map.is_occupied(nx, ny):
                    continue

                # Corner cutting prevention: if moving diagonally, ensure neither adjacent cardinal cell is blocked
                if dx != 0 and dy != 0:
                    if grid_map.is_occupied(cx + dx, cy) or grid_map.is_occupied(cx, cy + dy):
                        continue

                tentative_g = current_g + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal_grid)
                    f_score[neighbor] = f
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))

        return None

    def plan_world(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        grid_map: GridMap,
    ) -> Optional[List[Tuple[float, float]]]:
        """Plan path between metric world coordinates and return list of world waypoints."""
        start_grid = grid_map.world_to_grid(start_world[0], start_world[1])
        goal_grid = grid_map.world_to_grid(goal_world[0], goal_world[1])

        grid_path = self.plan_grid(start_grid, goal_grid, grid_map)
        if grid_path is None:
            return None

        world_path = [grid_map.grid_to_world(gx, gy) for gx, gy in grid_path]
        return world_path

