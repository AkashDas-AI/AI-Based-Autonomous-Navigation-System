"""Dynamic moving obstacle modeling and spatiotemporal trajectory prediction."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


class DynamicObstacle:
    """A moving obstacle with deterministic kinematic updates and future trajectory projection.

    Supported Motion Patterns:
        - 'linear_ping_pong': Moves linearly between two endpoints, reversing velocity on reach.
        - 'circular': Moves along a circular orbit around a designated center.
        - 'waypoints': Navigates sequentially through an ordered set of metric waypoints.
        - 'constant_velocity': Moves with constant (vx, vy), with optional bouncing off metric bounds.
    """

    def __init__(
        self,
        obstacle_id: int,
        initial_position: Tuple[float, float],
        radius: float = 0.5,
        speed: float = 0.8,
        pattern: str = "linear_ping_pong",
        end_position: Optional[Tuple[float, float]] = None,
        center: Optional[Tuple[float, float]] = None,
        orbit_radius: float = 2.0,
        angular_rate: float = 0.5,
        waypoints: Optional[List[Tuple[float, float]]] = None,
        velocity: Optional[Tuple[float, float]] = None,
        bounds: Optional[Tuple[float, float, float, float]] = None,  # (xmin, ymin, xmax, ymax)
    ) -> None:
        self.obstacle_id = int(obstacle_id)
        self.initial_position = (float(initial_position[0]), float(initial_position[1]))
        self.radius = float(radius)
        self.speed = float(speed)
        self.pattern = pattern.lower()

        # State variables
        self.x = self.initial_position[0]
        self.y = self.initial_position[1]
        self.simulated_time = 0.0

        # Pattern parameters
        self.end_position = (float(end_position[0]), float(end_position[1])) if end_position else None
        self.center = (float(center[0]), float(center[1])) if center else None
        self.orbit_radius = float(orbit_radius)
        self.angular_rate = float(angular_rate)
        self.waypoints = [tuple(map(float, wp)) for wp in waypoints] if waypoints else None
        self.bounds = bounds

        # Velocity state
        if velocity is not None:
            self.vx, self.vy = float(velocity[0]), float(velocity[1])
        else:
            self.vx, self.vy = self._compute_initial_velocity()

        # Ping-pong / waypoint progress state
        self._ping_pong_forward = True
        self._current_wp_idx = 0

    def _compute_initial_velocity(self) -> Tuple[float, float]:
        """Compute initial linear velocity components based on pattern."""
        if self.pattern == "linear_ping_pong" and self.end_position:
            dx = self.end_position[0] - self.initial_position[0]
            dy = self.end_position[1] - self.initial_position[1]
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                return (dx / dist) * self.speed, (dy / dist) * self.speed
            return 0.0, 0.0

        elif self.pattern == "circular" and self.center:
            dx = self.initial_position[0] - self.center[0]
            dy = self.initial_position[1] - self.center[1]
            angle = math.atan2(dy, dx)
            # Tangential velocity vector
            vx = -self.angular_rate * self.orbit_radius * math.sin(angle)
            vy = self.angular_rate * self.orbit_radius * math.cos(angle)
            return vx, vy

        elif self.pattern == "waypoints" and self.waypoints and len(self.waypoints) > 1:
            target = self.waypoints[1]
            dx = target[0] - self.initial_position[0]
            dy = target[1] - self.initial_position[1]
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                return (dx / dist) * self.speed, (dy / dist) * self.speed
            return 0.0, 0.0

        return 0.0, 0.0

    @property
    def position(self) -> Tuple[float, float]:
        """Current 2D position tuple."""
        return self.x, self.y

    @property
    def velocity(self) -> Tuple[float, float]:
        """Current 2D velocity vector."""
        return self.vx, self.vy

    def step(self, dt: float) -> Tuple[float, float]:
        """Advance obstacle motion deterministically by dt seconds."""
        dt = float(dt)
        self.simulated_time += dt

        if self.pattern == "linear_ping_pong" and self.end_position:
            p_start = self.initial_position
            p_end = self.end_position
            seg_dx = p_end[0] - p_start[0]
            seg_dy = p_end[1] - p_start[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len > 1e-6:
                ux, uy = seg_dx / seg_len, seg_dy / seg_len
                # Step forward or backward
                direction = 1.0 if self._ping_pong_forward else -1.0
                step_dist = self.speed * dt * direction
                self.x += ux * step_dist
                self.y += uy * step_dist

                # Check endpoints
                proj = (self.x - p_start[0]) * ux + (self.y - p_start[1]) * uy
                if proj >= seg_len:
                    self._ping_pong_forward = False
                    self.x = p_end[0]
                    self.y = p_end[1]
                    self.vx, self.vy = -ux * self.speed, -uy * self.speed
                elif proj <= 0.0:
                    self._ping_pong_forward = True
                    self.x = p_start[0]
                    self.y = p_start[1]
                    self.vx, self.vy = ux * self.speed, uy * self.speed
                else:
                    self.vx, self.vy = ux * self.speed * direction, uy * self.speed * direction

        elif self.pattern == "circular" and self.center:
            angle = math.atan2(self.initial_position[1] - self.center[1], self.initial_position[0] - self.center[0])
            cur_angle = angle + self.angular_rate * self.simulated_time
            self.x = self.center[0] + self.orbit_radius * math.cos(cur_angle)
            self.y = self.center[1] + self.orbit_radius * math.sin(cur_angle)
            self.vx = -self.angular_rate * self.orbit_radius * math.sin(cur_angle)
            self.vy = self.angular_rate * self.orbit_radius * math.cos(cur_angle)

        elif self.pattern == "waypoints" and self.waypoints:
            target = self.waypoints[self._current_wp_idx]
            dx = target[0] - self.x
            dy = target[1] - self.y
            dist = math.hypot(dx, dy)
            if dist <= self.speed * dt:
                self.x, self.y = target
                self._current_wp_idx = (self._current_wp_idx + 1) % len(self.waypoints)
            else:
                self.x += (dx / dist) * self.speed * dt
                self.y += (dy / dist) * self.speed * dt
                self.vx = (dx / dist) * self.speed
                self.vy = (dy / dist) * self.speed

        elif self.pattern == "constant_velocity":
            self.x += self.vx * dt
            self.y += self.vy * dt
            if self.bounds:
                xmin, ymin, xmax, ymax = self.bounds
                if self.x - self.radius < xmin or self.x + self.radius > xmax:
                    self.vx = -self.vx
                    self.x = max(xmin + self.radius, min(self.x, xmax - self.radius))
                if self.y - self.radius < ymin or self.y + self.radius > ymax:
                    self.vy = -self.vy
                    self.y = max(ymin + self.radius, min(self.y, ymax - self.radius))

        return self.x, self.y

    def predict_position(self, dt_ahead: float) -> Tuple[float, float]:
        """Analytically predict the obstacle position at t + dt_ahead without mutating current state.

        Args:
            dt_ahead: Future time offset in seconds.

        Returns:
            Projected (x, y) coordinates.
        """
        if dt_ahead <= 0.0:
            return self.x, self.y

        if self.pattern == "circular" and self.center:
            angle = math.atan2(self.initial_position[1] - self.center[1], self.initial_position[0] - self.center[0])
            future_angle = angle + self.angular_rate * (self.simulated_time + dt_ahead)
            px = self.center[0] + self.orbit_radius * math.cos(future_angle)
            py = self.center[1] + self.orbit_radius * math.sin(future_angle)
            return px, py

        elif self.pattern == "linear_ping_pong" and self.end_position:
            p_start = self.initial_position
            p_end = self.end_position
            seg_dx = p_end[0] - p_start[0]
            seg_dy = p_end[1] - p_start[1]
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len < 1e-6:
                return self.x, self.y

            ux, uy = seg_dx / seg_len, seg_dy / seg_len
            current_proj = (self.x - p_start[0]) * ux + (self.y - p_start[1]) * uy
            direction = 1.0 if self._ping_pong_forward else -1.0

            # Ping-pong distance calculation via periodic folding
            total_travel = current_proj + direction * (self.speed * dt_ahead)
            cycle = 2.0 * seg_len
            mod_travel = total_travel % cycle
            if mod_travel < 0:
                mod_travel += cycle

            if mod_travel <= seg_len:
                final_proj = mod_travel
            else:
                final_proj = cycle - mod_travel

            px = p_start[0] + ux * final_proj
            py = p_start[1] + uy * final_proj
            return px, py

        # Default constant velocity linear extrapolation
        return self.x + self.vx * dt_ahead, self.y + self.vy * dt_ahead

    def predict_trajectory(self, horizon: float, dt: float = 0.1) -> List[Tuple[float, float]]:
        """Compute a list of discrete future positions over a prediction horizon."""
        steps = max(1, int(round(horizon / dt)))
        return [self.predict_position(k * dt) for k in range(1, steps + 1)]

    def reset(self) -> None:
        """Reset obstacle state back to initial configuration."""
        self.x = self.initial_position[0]
        self.y = self.initial_position[1]
        self.simulated_time = 0.0
        self._ping_pong_forward = True
        self._current_wp_idx = 0
        self.vx, self.vy = self._compute_initial_velocity()


class DynamicObstacleManager:
    """Manages an aggregate collection of dynamic obstacles within a simulation episode."""

    def __init__(self, obstacles: Optional[List[DynamicObstacle]] = None) -> None:
        self.obstacles: List[DynamicObstacle] = list(obstacles) if obstacles else []

    def add_obstacle(self, obstacle: DynamicObstacle) -> None:
        """Register a new dynamic obstacle."""
        self.obstacles.append(obstacle)

    def step(self, dt: float) -> None:
        """Advance all dynamic obstacles forward by dt seconds."""
        for obs in self.obstacles:
            obs.step(dt)

    def reset(self) -> None:
        """Reset all dynamic obstacles to their starting states."""
        for obs in self.obstacles:
            obs.reset()

    def check_collision(self, x: float, y: float, robot_radius: float) -> Tuple[bool, Optional[DynamicObstacle]]:
        """Check if disc at (x, y) with robot_radius collides with any moving obstacle."""
        for obs in self.obstacles:
            dist = math.hypot(x - obs.x, y - obs.y)
            if dist <= (robot_radius + obs.radius):
                return True, obs
        return False, None

    def min_distance(self, x: float, y: float) -> float:
        """Calculate minimum Euclidean surface clearance to any dynamic obstacle."""
        if not self.obstacles:
            return float("inf")
        min_d = float("inf")
        for obs in self.obstacles:
            dist = math.hypot(x - obs.x, y - obs.y) - obs.radius
            if dist < min_d:
                min_d = dist
        return max(0.0, min_d)

    def predict_all(self, dt_ahead: float) -> List[Tuple[float, float, float]]:
        """Return projected (x, y, radius) tuples for all obstacles at future time offset."""
        return [(obs.predict_position(dt_ahead)[0], obs.predict_position(dt_ahead)[1], obs.radius) for obs in self.obstacles]

    def __len__(self) -> int:
        return len(self.obstacles)

    def __iter__(self):
        return iter(self.obstacles)

