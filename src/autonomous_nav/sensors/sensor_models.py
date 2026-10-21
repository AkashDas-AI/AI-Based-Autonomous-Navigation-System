"""Sensor noise models and synthetic measurement generators.

Provides realistic simulation models for:
- Wheel odometry (velocity noise and scale factor errors)
- IMU rate gyroscope (angular rate white noise and slowly drifting bias)
- GNSS position receiver (low-frequency position noise and GPS outage simulation)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

from autonomous_nav.simulation.robot import RobotState


@dataclass
class SensorMeasurements:
    """Synchronous packet of simulated sensor measurements at time t."""
    timestamp: float
    v_odom: float                      # Measured forward velocity (m/s)
    omega_imu: float                   # Measured angular yaw rate (rad/s)
    gnss_pos: Optional[Tuple[float, float]] = None  # (x_meas, y_meas) if fix available
    gnss_valid: bool = False           # True if GNSS measurement update is available at this step


class WheelOdometrySensor:
    """Simulated wheel odometry measuring forward linear velocity.

    Includes zero-mean Gaussian measurement noise and optional systematic wheel scale factor error.
    """

    def __init__(
        self,
        noise_std: float = 0.05,
        scale_factor_error: float = 1.0,
    ) -> None:
        self.noise_std = float(noise_std)
        self.scale_factor_error = float(scale_factor_error)

    def sample(self, true_v: float, rng: np.random.RandomState) -> float:
        """Sample forward linear velocity with scale error and Gaussian noise."""
        noise = rng.normal(0.0, self.noise_std) if self.noise_std > 0.0 else 0.0
        return float(true_v * self.scale_factor_error + noise)


class IMUSensor:
    """Simulated inertial measurement unit (IMU) single-axis rate gyroscope.

    Measures angular yaw rate with additive white Gaussian noise and a time-varying
    random-walk bias drift:
        b_k = b_{k-1} + N(0, sigma_drift^2 * dt)
        omega_meas = omega_true + b_k + N(0, sigma_rate^2)
    """

    def __init__(
        self,
        rate_noise_std: float = 0.02,
        bias_init: float = 0.01,
        bias_drift_std: float = 0.001,
    ) -> None:
        self.rate_noise_std = float(rate_noise_std)
        self.bias_init = float(bias_init)
        self.bias_drift_std = float(bias_drift_std)
        self.bias: float = self.bias_init

    def reset(self, bias_init: Optional[float] = None) -> None:
        """Reset internal gyro bias state."""
        self.bias = float(bias_init) if bias_init is not None else self.bias_init

    def step_bias(self, dt: float, rng: np.random.RandomState) -> None:
        """Evolve random-walk bias drift over time step dt."""
        if dt > 0.0 and self.bias_drift_std > 0.0:
            drift = rng.normal(0.0, self.bias_drift_std * math.sqrt(dt))
            self.bias += float(drift)

    def sample(self, true_omega: float, dt: float, rng: np.random.RandomState) -> float:
        """Advance bias and sample noisy angular velocity."""
        self.step_bias(dt, rng)
        noise = rng.normal(0.0, self.rate_noise_std) if self.rate_noise_std > 0.0 else 0.0
        return float(true_omega + self.bias + noise)


class GNSSReceiver:
    """Simulated Global Navigation Satellite System (GNSS) position sensor.

    Emits 2D metric position fixes at a lower update frequency (e.g. 1 Hz) than the
    simulation rate (e.g. 10 Hz), corrupted by zero-mean Gaussian noise, and simulates
    signal outages (e.g., urban canyon or tunnel occlusion).
    """

    def __init__(
        self,
        pos_noise_std: float = 0.3,
        update_interval: float = 1.0,
        outages: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        self.pos_noise_std = float(pos_noise_std)
        self.update_interval = float(update_interval)
        self.outages: List[Tuple[float, float]] = list(outages) if outages is not None else []
        self.last_fix_time: float = -float("inf")
        self.last_slot: int = -1

    def reset(self) -> None:
        """Reset GNSS internal state and fix schedule."""
        self.last_fix_time = -float("inf")
        self.last_slot = -1

    def is_in_outage(self, current_time: float) -> bool:
        """Check if current timestamp falls within any designated GNSS outage window."""
        for t_start, t_end in self.outages:
            if t_start <= current_time <= t_end:
                return True
        return False

    def should_update(self, current_time: float) -> bool:
        """Check if current time coincides with a periodic GNSS measurement epoch."""
        if self.update_interval <= 0:
            return True
        current_slot = int(math.floor((current_time + 1e-5) / self.update_interval))
        return current_slot > self.last_slot

    def sample(
        self,
        true_x: float,
        true_y: float,
        current_time: float,
        rng: np.random.RandomState,
    ) -> Tuple[Optional[Tuple[float, float]], bool]:
        """Sample GNSS 2D position fix if scheduled and not in outage.

        Returns:
            Tuple[Optional[Tuple[float, float]], bool]:
                ((x_meas, y_meas), True) if a valid fix occurred; (None, False) otherwise.
        """
        if self.update_interval <= 0:
            current_slot = 0
        else:
            current_slot = int(math.floor((current_time + 1e-5) / self.update_interval))

        if not self.should_update(current_time):
            return None, False

        # Advance slot epoch tracking
        self.last_slot = current_slot
        self.last_fix_time = current_time

        if self.is_in_outage(current_time):
            return None, False

        noise_x = rng.normal(0.0, self.pos_noise_std) if self.pos_noise_std > 0.0 else 0.0
        noise_y = rng.normal(0.0, self.pos_noise_std) if self.pos_noise_std > 0.0 else 0.0
        meas_x = float(true_x + noise_x)
        meas_y = float(true_y + noise_y)
        return (meas_x, meas_y), True


class SensorSuite:
    """Integrated sensor suite combining odometry, IMU, and GNSS with deterministic seeding."""

    def __init__(
        self,
        odometry_noise_std: float = 0.05,
        odometry_scale_factor: float = 1.0,
        imu_rate_noise_std: float = 0.02,
        imu_bias_init: float = 0.01,
        imu_bias_drift_std: float = 0.001,
        gnss_pos_noise_std: float = 0.3,
        gnss_update_interval: float = 1.0,
        gnss_outages: Optional[List[Tuple[float, float]]] = None,
        seed: Optional[int] = 42,
    ) -> None:
        self.seed = seed
        self.rng = np.random.RandomState(seed)
        self.odometry = WheelOdometrySensor(
            noise_std=odometry_noise_std,
            scale_factor_error=odometry_scale_factor,
        )
        self.imu = IMUSensor(
            rate_noise_std=imu_rate_noise_std,
            bias_init=imu_bias_init,
            bias_drift_std=imu_bias_drift_std,
        )
        self.gnss = GNSSReceiver(
            pos_noise_std=gnss_pos_noise_std,
            update_interval=gnss_update_interval,
            outages=gnss_outages,
        )

    def reset(self, seed: Optional[int] = None) -> None:
        """Reset sensors and reseed random number generator for deterministic repeatability."""
        if seed is not None:
            self.seed = seed
        self.rng = np.random.RandomState(self.seed)
        self.imu.reset()
        self.gnss.reset()

    def sample(
        self,
        true_state: RobotState,
        dt: float,
        current_time: float,
    ) -> SensorMeasurements:
        """Generate synchronized sensor readings at time current_time."""
        v_meas = self.odometry.sample(true_state.v, self.rng)
        omega_meas = self.imu.sample(true_state.omega, dt, self.rng)
        gnss_pos, gnss_valid = self.gnss.sample(
            true_state.x, true_state.y, current_time, self.rng
        )

        return SensorMeasurements(
            timestamp=float(current_time),
            v_odom=v_meas,
            omega_imu=omega_meas,
            gnss_pos=gnss_pos,
            gnss_valid=gnss_valid,
        )
