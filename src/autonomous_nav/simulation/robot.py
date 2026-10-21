"""Robot kinematic modeling and simulation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from autonomous_nav.config import RobotConfig


@dataclass
class RobotState:
    """Current state of the simulated mobile robot."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # heading angle in radians
    v: float = 0.0      # linear forward velocity (m/s)
    omega: float = 0.0  # angular velocity (rad/s)
    timestamp: float = 0.0  # simulation epoch timestamp in seconds

    def to_tuple(self) -> tuple[float, float, float, float, float]:
        """Return state as a tuple (x, y, theta, v, omega)."""
        return self.x, self.y, self.theta, self.v, self.omega


class RobotModel:
    """Kinematic unicycle vehicle model with velocity and acceleration constraints."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self.config = config if config is not None else RobotConfig()
        self.state = RobotState()

    def reset(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        v: float = 0.0,
        omega: float = 0.0,
        timestamp: float = 0.0,
    ) -> RobotState:
        """Reset robot to specified initial state."""
        self.state = RobotState(
            x=float(x),
            y=float(y),
            theta=self.normalize_angle(float(theta)),
            v=float(v),
            omega=float(omega),
            timestamp=float(timestamp),
        )
        return self.state

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def step(self, v_cmd: float, omega_cmd: float, dt: float) -> RobotState:
        """Advance robot state by time step dt using commanded linear and angular velocities.

        Applies acceleration limits and kinematic integration.
        """
        if dt <= 0.0:
            return self.state

        # Bound commanded linear velocity
        v_target = max(self.config.min_velocity, min(self.config.max_velocity, float(v_cmd)))

        # Bound commanded angular velocity
        omega_target = max(
            -self.config.max_angular_velocity,
            min(self.config.max_angular_velocity, float(omega_cmd)),
        )

        # Apply maximum linear acceleration limit
        max_delta_v = self.config.max_acceleration * dt
        v_diff = v_target - self.state.v
        if abs(v_diff) > max_delta_v:
            v_actual = self.state.v + math.copysign(max_delta_v, v_diff)
        else:
            v_actual = v_target

        omega_actual = omega_target

        # Kinematic integration (Runge-Kutta 2nd Order / Midpoint Method)
        theta_mid = self.state.theta + 0.5 * omega_actual * dt
        new_x = self.state.x + v_actual * math.cos(theta_mid) * dt
        new_y = self.state.y + v_actual * math.sin(theta_mid) * dt
        new_theta = self.normalize_angle(self.state.theta + omega_actual * dt)
        new_timestamp = round(self.state.timestamp + dt, 6)

        self.state = RobotState(
            x=new_x,
            y=new_y,
            theta=new_theta,
            v=v_actual,
            omega=omega_actual,
            timestamp=new_timestamp,
        )

        return self.state

