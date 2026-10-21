"""Extended Kalman Filter (EKF) state estimation for 2D mobile robot navigation.

Implements an analytical Extended Kalman Filter fusing:
- Dead-reckoning kinematic prediction (IMU yaw rate and forward velocity)
- Wheel odometry forward velocity measurement updates
- Low-frequency GNSS 2D position fixes

State Vector:
    x = [x, y, theta, v, b_g]^T
Where:
    x, y   : 2D Cartesian world position (meters)
    theta  : Robot heading angle (radians, normalized to [-pi, pi))
    v      : Forward linear velocity (m/s)
    b_g    : IMU rate gyroscope bias (rad/s)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional, Tuple
import numpy as np

if TYPE_CHECKING:
    from autonomous_nav.simulation.robot import RobotState


@dataclass
class EKFState:
    """Estimated state vector and covariance of the vehicle."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    bias_gyro: float = 0.0
    covariance: np.ndarray = field(default_factory=lambda: np.eye(5) * 0.1)
    timestamp: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        """Return estimated 2D position (x, y)."""
        return self.x, self.y


class ExtendedKalmanFilter:
    """Extended Kalman Filter with analytical Jacobians and Joseph-form covariance updates."""

    def __init__(
        self,
        q_pos: float = 0.05,
        q_theta: float = 0.02,
        q_v: float = 0.1,
        q_bias: float = 0.001,
        r_odom: float = 0.05,
        r_gnss: float = 0.3,
        initial_pos_std: float = 0.5,
        initial_theta_std: float = 0.1,
        initial_v_std: float = 0.2,
        initial_bias_std: float = 0.05,
    ) -> None:
        self.q_pos = float(q_pos)
        self.q_theta = float(q_theta)
        self.q_v = float(q_v)
        self.q_bias = float(q_bias)
        self.r_odom = float(r_odom)
        self.r_gnss = float(r_gnss)

        self.initial_pos_std = float(initial_pos_std)
        self.initial_theta_std = float(initial_theta_std)
        self.initial_v_std = float(initial_v_std)
        self.initial_bias_std = float(initial_bias_std)

        # 5-element state vector: [x, y, theta, v, b_g]
        self.x = np.zeros(5, dtype=np.float64)
        # 5x5 error covariance matrix
        self.P = np.eye(5, dtype=np.float64)
        # Process noise covariance base diagonal
        self._q_diag = np.array(
            [self.q_pos**2, self.q_pos**2, self.q_theta**2, self.q_v**2, self.q_bias**2],
            dtype=np.float64,
        )

        self.last_omega_cmd: float = 0.0
        self.reset()

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi) interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        v: float = 0.0,
        bias_gyro: float = 0.0,
    ) -> None:
        """Reset state vector and initialize error covariance."""
        self.x = np.array(
            [float(x), float(y), self.normalize_angle(float(theta)), float(v), float(bias_gyro)],
            dtype=np.float64,
        )
        self.P = np.diag([
            self.initial_pos_std**2,
            self.initial_pos_std**2,
            self.initial_theta_std**2,
            self.initial_v_std**2,
            self.initial_bias_std**2,
        ]).astype(np.float64)
        self.last_omega_cmd = 0.0

    def compute_jacobian_f(self, v: float, theta: float, dt: float) -> np.ndarray:
        """Compute analytical state transition Jacobian matrix F = df/dx.

        f(x) = [
            x + v * cos(theta) * dt,
            y + v * sin(theta) * dt,
            theta + (omega_imu - b_g) * dt,
            v,
            b_g
        ]
        """
        f_mat = np.eye(5, dtype=np.float64)
        f_mat[0, 2] = -v * math.sin(theta) * dt
        f_mat[0, 3] = math.cos(theta) * dt
        f_mat[1, 2] = v * math.cos(theta) * dt
        f_mat[1, 3] = math.sin(theta) * dt
        f_mat[2, 4] = -dt
        return f_mat

    def predict(self, omega_imu: float, dt: float) -> np.ndarray:
        """Execute state and covariance prediction step.

        Args:
            omega_imu: Measured raw angular velocity from IMU rate gyro (rad/s).
            dt: Time interval in seconds.

        Returns:
            np.ndarray: Predicted 5-element state vector.
        """
        if dt <= 0.0:
            return self.x.copy()

        x, y, theta, v, b_g = self.x

        # Unbiased angular velocity rate
        unbiased_omega = omega_imu - b_g
        self.last_omega_cmd = unbiased_omega

        # Predict state forward
        new_x = x + v * math.cos(theta) * dt
        new_y = y + v * math.sin(theta) * dt
        new_theta = self.normalize_angle(theta + unbiased_omega * dt)
        new_v = v
        new_bg = b_g

        self.x = np.array([new_x, new_y, new_theta, new_v, new_bg], dtype=np.float64)

        # Compute state transition Jacobian F
        f_mat = self.compute_jacobian_f(v=v, theta=theta, dt=dt)

        # Scaled process noise covariance Q
        q_mat = np.diag(self._q_diag * dt)

        # Predict covariance: P = F * P * F^T + Q
        self.P = f_mat @ self.P @ f_mat.T + q_mat

        # Symmetrize covariance to suppress numerical precision drift
        self.P = 0.5 * (self.P + self.P.T)
        return self.x.copy()

    def update_odometry(self, v_odom: float, r_odom: Optional[float] = None) -> np.ndarray:
        """Update filter with wheel odometry linear velocity measurement.

        Measurement model:
            h(x) = v  =>  H = [0, 0, 0, 1, 0]
        """
        r_val = float(r_odom**2 if r_odom is not None else self.r_odom**2)
        h_mat = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]], dtype=np.float64)

        # Innovation
        y = float(v_odom - self.x[3])

        # Innovation covariance S = H * P * H^T + R
        s = float(self.P[3, 3] + r_val)
        if s <= 1e-12:
            return self.x.copy()

        # Kalman gain K = P * H^T / S  (5x1 vector)
        k = self.P[:, 3:4] / s  # shape (5, 1)

        # State update
        self.x = self.x + (k * y).flatten()
        self.x[2] = self.normalize_angle(self.x[2])

        # Joseph-form covariance update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        eye5 = np.eye(5, dtype=np.float64)
        ikh = eye5 - k @ h_mat
        self.P = ikh @ self.P @ ikh.T + (k * r_val) @ k.T
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy()

    def update_gnss(
        self,
        x_gnss: float,
        y_gnss: float,
        r_gnss: Optional[float] = None,
    ) -> np.ndarray:
        """Update filter with 2D GNSS position fix (x_meas, y_meas).

        Measurement model:
            h(x) = [x, y]^T  =>  H = [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]
            ]
        """
        r_val = float(r_gnss**2 if r_gnss is not None else self.r_gnss**2)
        h_mat = np.zeros((2, 5), dtype=np.float64)
        h_mat[0, 0] = 1.0
        h_mat[1, 1] = 1.0

        r_mat = np.array([[r_val, 0.0], [0.0, r_val]], dtype=np.float64)

        # Innovation
        y = np.array([x_gnss - self.x[0], y_gnss - self.x[1]], dtype=np.float64)

        # Innovation covariance S = H * P * H^T + R
        s_mat = self.P[0:2, 0:2] + r_mat

        try:
            s_inv = np.linalg.inv(s_mat)
        except np.linalg.LinAlgError:
            return self.x.copy()

        # Kalman gain K = P * H^T * S^-1  (shape 5x2)
        k_mat = self.P[:, 0:2] @ s_inv

        # State update
        self.x = self.x + k_mat @ y
        self.x[2] = self.normalize_angle(self.x[2])

        # Joseph-form covariance update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        eye5 = np.eye(5, dtype=np.float64)
        ikh = eye5 - k_mat @ h_mat
        self.P = ikh @ self.P @ ikh.T + k_mat @ r_mat @ k_mat.T
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy()

    def get_estimated_robot_state(self, timestamp: float = 0.0) -> RobotState:
        """Convert current filter estimate into standard RobotState for controller feedback."""
        from autonomous_nav.simulation.robot import RobotState

        return RobotState(
            x=float(self.x[0]),
            y=float(self.x[1]),
            theta=float(self.x[2]),
            v=float(self.x[3]),
            omega=float(self.last_omega_cmd),
            timestamp=float(timestamp),
        )

    def get_position_covariance_ellipse(
        self, conf_level: float = 0.95
    ) -> Tuple[float, float, float]:
        """Compute the parameters of the 2D positional uncertainty ellipse.

        Args:
            conf_level: Desired confidence level (default 0.95 => Chi-square critical val 5.991).

        Returns:
            Tuple[float, float, float]: (semi_major_axis, semi_minor_axis, orientation_radians)
        """
        # Marginal 2x2 position covariance
        p_pos = self.P[0:2, 0:2]

        # Eigenvalues and eigenvectors
        eigvals, eigvecs = np.linalg.eigh(p_pos)

        # Sort eigenvalues descending
        order = eigvals.argsort()[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]

        # Chi-square critical multiplier for 2 DOF
        # For 0.95 confidence: -2 * ln(1 - 0.95) = -2 * ln(0.05) ~= 5.99146
        chi2_val = -2.0 * math.log(max(1e-6, 1.0 - conf_level))
        scale = math.sqrt(chi2_val)

        semi_major = scale * math.sqrt(max(0.0, float(eigvals[0])))
        semi_minor = scale * math.sqrt(max(0.0, float(eigvals[1])))

        # Heading of the semi-major eigenvector
        orientation = math.atan2(float(eigvecs[1, 0]), float(eigvecs[0, 0]))

        return semi_major, semi_minor, orientation

