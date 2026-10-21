"""Waypoint tracking and sequential navigation management."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple


class WaypointManager:
    """Manages sequential waypoint progression and lookahead target calculation."""

    def __init__(
        self,
        waypoints: Optional[List[Tuple[float, float]]] = None,
        acceptance_radius: float = 0.8,
    ) -> None:
        self.waypoints: List[Tuple[float, float]] = waypoints if waypoints is not None else []
        self.current_index: int = 0
        self.acceptance_radius: float = float(acceptance_radius)

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Assign a new sequence of metric waypoints and reset active index."""
        self.waypoints = [(float(x), float(y)) for x, y in waypoints]
        self.current_index = 0

    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Return the current active target waypoint, or None if empty/finished."""
        if not self.waypoints or self.current_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_index]

    def get_goal_waypoint(self) -> Optional[Tuple[float, float]]:
        """Return final destination waypoint."""
        if not self.waypoints:
            return None
        return self.waypoints[-1]

    def update(self, robot_x: float, robot_y: float) -> bool:
        """Advance waypoint index if the robot has reached within acceptance radius.

        Returns:
            True if final goal waypoint has been reached, False otherwise.
        """
        if not self.waypoints:
            return True

        while self.current_index < len(self.waypoints):
            wx, wy = self.waypoints[self.current_index]
            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist <= self.acceptance_radius and self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
            else:
                break

        return self.current_index >= len(self.waypoints) - 1 and (
            math.hypot(
                self.waypoints[-1][0] - robot_x,
                self.waypoints[-1][1] - robot_y,
            )
            <= self.acceptance_radius
        )

    def get_lookahead_point(
        self,
        robot_x: float,
        robot_y: float,
        lookahead_distance: float,
    ) -> Optional[Tuple[float, float]]:
        """Find the path point at approximately lookahead_distance ahead of the robot."""
        if not self.waypoints:
            return None

        # Search forward starting from current_index
        for i in range(self.current_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist >= lookahead_distance:
                return wx, wy

        # If all remaining points are closer than lookahead_distance, return final goal
        return self.waypoints[-1]

    def distance_to_goal(self, robot_x: float, robot_y: float) -> float:
        """Compute Euclidean distance from robot to final goal waypoint."""
        if not self.waypoints:
            return 0.0
        gx, gy = self.waypoints[-1]
        return math.hypot(gx - robot_x, gy - robot_y)

    def is_goal_reached(
        self,
        robot_x: float,
        robot_y: float,
        tolerance: float = 0.5,
    ) -> bool:
        """Check if robot is within goal tolerance distance of the final waypoint."""
        return self.distance_to_goal(robot_x, robot_y) <= tolerance

