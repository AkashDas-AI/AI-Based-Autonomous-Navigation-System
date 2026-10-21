"""Trajectory tracking controllers for autonomous mobile navigation."""

from __future__ import annotations

import math
from typing import Tuple
from autonomous_nav.config import ControllerConfig
from autonomous_nav.simulation.robot import RobotState


class PurePursuitController:
    """Pure Pursuit tracking controller with adaptive deceleration for sharp curvatures."""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config if config is not None else ControllerConfig()

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute_control(
        self,
        state: RobotState,
        target_point: Tuple[float, float],
        distance_to_goal: float = 10.0,
    ) -> Tuple[float, float]:
        """Compute linear velocity (v_cmd) and angular velocity (omega_cmd).

        Args:
            state: Current kinematic state of the robot.
            target_point: Metric coordinates (tx, ty) of lookahead waypoint.
            distance_to_goal: Remaining distance to ultimate destination in meters.

        Returns:
            Tuple of (v_cmd, omega_cmd).
        """
        tx, ty = target_point
        dx = tx - state.x
        dy = ty - state.y
        dist = math.hypot(dx, dy)

        if dist < 1e-4:
            return 0.0, 0.0

        # Angle from robot heading to target point
        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - state.theta)

        # Proportional angular velocity control based on heading error
        omega_cmd = self.config.kp_angular * alpha

        # Base forward velocity
        v_base = self.config.target_velocity

        # Decelerate if heading error is sharp (e.g. > 30 degrees)
        turn_slowdown = max(0.2, math.cos(alpha))
        v_cmd = v_base * turn_slowdown

        # Decelerate smoothly when approaching final goal
        if distance_to_goal < 2.0:
            approach_factor = max(0.1, distance_to_goal / 2.0)
            v_cmd *= approach_factor

        return float(v_cmd), float(omega_cmd)

