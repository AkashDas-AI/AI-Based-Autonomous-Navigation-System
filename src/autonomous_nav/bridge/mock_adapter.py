"""Offline mock test adapter for automated testing and CI.

NOTICE: THIS IS AN OFFLINE TEST ADAPTER DESIGNED EXCLUSIVELY FOR MOCK TESTING.
IT DOES NOT CONNECT TO A RUNNING 3D SIMULATOR PROCESS (ROS 2 / Gazebo / CARLA).
It simulates deterministic kinematic vehicle physics, raycast LiDAR returns,
and noisy odometry/IMU/GNSS observations in-process for automated test validation.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np

from autonomous_nav.bridge.config import BridgeConfig
from autonomous_nav.bridge.contracts import (
    GnssObservation,
    ImuObservation,
    LaserScanObservation,
    NavigationCommand,
    OdometryObservation,
    SimulationLifecycleState,
    SimulatorCapabilities,
    SimulatorSensorPacket,
    TimestampedRobotState,
)
from autonomous_nav.bridge.interface import SimulationBridge
from autonomous_nav.environment.grid_map import GridMap


class OfflineTestAdapter(SimulationBridge):
    """Deterministic in-process test adapter implementing the SimulationBridge contract.

    Explicitly labeled as an offline test adapter. Used for testing message schemas,
    coordinate transforms, command-limit enforcements, failure handling, and lifecycle states.
    """

    def __init__(
        self,
        config: Optional[BridgeConfig] = None,
        grid_map: Optional[GridMap] = None,
        initial_state: Optional[TimestampedRobotState] = None,
        seed: int = 42,
    ) -> None:
        self.config = config if config is not None else BridgeConfig()
        self.grid_map = grid_map
        self.seed = seed
        self._rng = np.random.default_rng(self.seed)

        self._lifecycle: SimulationLifecycleState = SimulationLifecycleState.UNINITIALIZED
        self._connected: bool = False
        self._healthy: bool = True
        self._step_count: int = 0
        self._current_time: float = 0.0
        self._last_gnss_time: float = -1e9

        self._default_start_state = (
            initial_state
            if initial_state is not None
            else TimestampedRobotState(x=0.0, y=0.0, z=0.0, yaw=0.0, timestamp=0.0)
        )
        self._ground_truth_state: TimestampedRobotState = self._default_start_state
        self._latest_command: NavigationCommand = NavigationCommand(
            v_cmd=0.0, omega_cmd=0.0, timestamp=0.0, frame_id="base_link"
        )
        self._latest_sensor_packet: Optional[SimulatorSensorPacket] = None

    def connect(self, timeout_sec: Optional[float] = None) -> bool:
        """Establish simulated in-process connection."""
        timeout = timeout_sec if timeout_sec is not None else self.config.timeout_sec
        if timeout <= 0.0:
            self._healthy = False
            self._lifecycle = SimulationLifecycleState.ERROR
            return False

        self._connected = True
        self._healthy = True
        self._lifecycle = SimulationLifecycleState.READY
        return True

    def disconnect(self) -> None:
        """Gracefully terminate mock session."""
        self._connected = False
        self._lifecycle = SimulationLifecycleState.STOPPED

    def initialize(self, initial_state: Optional[TimestampedRobotState] = None) -> bool:
        """Initialize mock environment and seed positions."""
        if not self._connected:
            self._lifecycle = SimulationLifecycleState.ERROR
            raise ConnectionError("Cannot initialize: adapter is disconnected. Call connect() first.")

        if initial_state is not None:
            if initial_state.frame_id not in self.config.allowed_frames:
                raise ValueError(f"Unsupported frame_id: {initial_state.frame_id}")
            self._default_start_state = initial_state
            self._ground_truth_state = initial_state
        else:
            self._ground_truth_state = self._default_start_state

        self._current_time = self._ground_truth_state.timestamp
        self._step_count = 0
        self._last_gnss_time = -1e9
        self._latest_command = NavigationCommand(v_cmd=0.0, omega_cmd=0.0, timestamp=self._current_time)
        self._latest_sensor_packet = self._generate_sensor_packet(self._ground_truth_state)
        self._lifecycle = SimulationLifecycleState.READY
        return True

    def reset(self, initial_state: Optional[TimestampedRobotState] = None) -> bool:
        """Deterministic reset of mock runtime and RNG streams."""
        if not self._connected:
            raise ConnectionError("Cannot reset: adapter is disconnected.")

        self._rng = np.random.default_rng(self.seed)
        return self.initialize(initial_state)

    def send_command(self, command: NavigationCommand) -> bool:
        """Accept, validate, and store navigation command."""
        if not self._connected:
            raise ConnectionError("Cannot send command: adapter is disconnected.")
        if self._lifecycle in (SimulationLifecycleState.UNINITIALIZED, SimulationLifecycleState.STOPPED, SimulationLifecycleState.ERROR):
            raise RuntimeError(f"Cannot accept command in lifecycle state: {self._lifecycle}")

        # Validate coordinate frame
        if command.frame_id not in self.config.allowed_frames:
            raise ValueError(
                f"Unsupported command frame_id '{command.frame_id}'. Allowed frames: {self.config.allowed_frames}"
            )

        # Validate timestamp monotonicity
        if self.config.strict_timestamp_monotonicity and command.timestamp < (self._current_time - 1e-4):
            raise ValueError(
                f"Command timestamp {command.timestamp:.4f}s is earlier than current simulation time {self._current_time:.4f}s."
            )

        # Enforce physical velocity limits
        validated_cmd = command.validated_or_clamped(
            min_v=self.config.min_velocity,
            max_v=self.config.max_velocity,
            max_omega=self.config.max_angular_velocity,
            clamp=self.config.clamp_commands,
        )
        self._latest_command = validated_cmd
        return True

    def step(self, dt: float) -> bool:
        """Advance physical unicycle model and synthesize noisy observations."""
        if not self._connected:
            raise ConnectionError("Cannot step: adapter is disconnected.")
        if self._lifecycle not in (SimulationLifecycleState.READY, SimulationLifecycleState.RUNNING):
            raise RuntimeError(f"Cannot step in lifecycle state: {self._lifecycle}")
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"Timestep dt must be positive and finite, got: {dt}")

        v = self._latest_command.v_cmd
        omega = self._latest_command.omega_cmd

        # 2nd-order Runge-Kutta (RK2) unicycle integration for ground truth physics
        gt = self._ground_truth_state
        theta_mid = gt.yaw + 0.5 * omega * dt
        new_x = gt.x + v * math.cos(theta_mid) * dt
        new_y = gt.y + v * math.sin(theta_mid) * dt
        new_yaw = math.atan2(math.sin(gt.yaw + omega * dt), math.cos(gt.yaw + omega * dt))
        new_time = round(self._current_time + dt, 6)

        self._ground_truth_state = TimestampedRobotState(
            x=new_x,
            y=new_y,
            z=gt.z,
            roll=0.0,
            pitch=0.0,
            yaw=new_yaw,
            vx=v * math.cos(new_yaw),
            vy=v * math.sin(new_yaw),
            vz=0.0,
            v_linear=v,
            omega_yaw=omega,
            timestamp=new_time,
            frame_id="world",
            child_frame_id="base_link",
        )

        self._current_time = new_time
        self._step_count += 1
        self._lifecycle = SimulationLifecycleState.RUNNING

        # Synthesize noisy observations for the new timestep
        self._latest_sensor_packet = self._generate_sensor_packet(self._ground_truth_state)
        return True

    def get_ground_truth_state(self) -> TimestampedRobotState:
        """Return the physical ground truth state (evaluation only)."""
        return self._ground_truth_state

    def get_sensor_observations(self) -> SimulatorSensorPacket:
        """Return the latest noisy sensor packet."""
        if self._latest_sensor_packet is None:
            self._latest_sensor_packet = self._generate_sensor_packet(self._ground_truth_state)
        return self._latest_sensor_packet

    def get_capabilities(self) -> SimulatorCapabilities:
        """Query capability profile."""
        return SimulatorCapabilities(
            simulator_name="OfflineTestAdapter (Mock Simulation Engine for CI/Testing)",
            version="1.0.0-mock",
            clock_source="simulated_step",
            supported_frames=self.config.allowed_frames,
            supported_sensors=("odometry", "imu", "gnss", "laser_scan"),
            supports_lockstep_stepping=True,
            supports_dynamic_obstacles=False,
            supports_ground_truth_pose=True,
            is_connected=self._connected,
            connection_endpoint="in_process_test_harness",
        )

    def get_lifecycle_state(self) -> SimulationLifecycleState:
        return self._lifecycle

    def is_healthy(self) -> bool:
        return self._healthy and self._connected

    # ---------------------------------------------------------------------------
    # Internal Synthetic Sensor Generators
    # ---------------------------------------------------------------------------

    def _generate_sensor_packet(self, gt: TimestampedRobotState) -> SimulatorSensorPacket:
        """Synthesize noisy sensor readings corresponding to true robot state."""
        t = gt.timestamp

        # Odometry (measured speed with scale error and additive Gaussian noise)
        v_noisy = max(0.0, gt.v_linear * 1.01 + float(self._rng.normal(0.0, 0.02)))
        omega_noisy = gt.omega_yaw + float(self._rng.normal(0.0, 0.01))
        odom_obs = OdometryObservation(
            timestamp=t,
            frame_id="odom",
            linear_velocity=v_noisy,
            angular_velocity=omega_noisy,
            pose_xy=(gt.x, gt.y),
            heading=gt.yaw,
            status=True,
        )

        # IMU (angular rate with noise, linear acceleration)
        imu_obs = ImuObservation(
            timestamp=t,
            frame_id="base_imu",
            angular_velocity_rpy=(0.0, 0.0, omega_noisy),
            linear_acceleration_xyz=(0.0, 0.0, 9.81),
            orientation_rpy=(0.0, 0.0, gt.yaw),
            status=True,
        )

        # GNSS (decimated 1 Hz updates with 0.25m Gaussian noise)
        gnss_obs = None
        if (t - self._last_gnss_time) >= (self.config.gnss_update_interval - 1e-4):
            self._last_gnss_time = t
            x_gnss = gt.x + float(self._rng.normal(0.0, 0.25))
            y_gnss = gt.y + float(self._rng.normal(0.0, 0.25))
            gnss_obs = GnssObservation(
                timestamp=t,
                frame_id="world",
                easting=x_gnss,
                northing=y_gnss,
                altitude=gt.z,
                hdop=1.0,
                status=True,
            )

        # LiDAR Planar Scan
        laser_obs = self._simulate_laser_scan(gt)

        return SimulatorSensorPacket(
            timestamp=t,
            odometry=odom_obs,
            imu=imu_obs,
            gnss=gnss_obs,
            laser_scan=laser_obs,
        )

    def _simulate_laser_scan(self, gt: TimestampedRobotState) -> LaserScanObservation:
        """Raytrace mock planar laser range returns against the grid map if provided."""
        cfg = self.config
        angles = np.linspace(cfg.laser_angle_min, cfg.laser_angle_max, cfg.laser_num_rays)
        ranges = []

        step_dist = 0.1  # raymarch step in meters

        for rel_angle in angles:
            abs_angle = gt.yaw + rel_angle
            cos_a = math.cos(abs_angle)
            sin_a = math.sin(abs_angle)
            hit_dist = cfg.laser_range_max

            if self.grid_map is not None:
                # Raymarch outward from robot position
                dist = cfg.laser_range_min
                while dist < cfg.laser_range_max:
                    rx = gt.x + dist * cos_a
                    ry = gt.y + dist * sin_a
                    if self.grid_map.is_occupied_world(rx, ry):
                        hit_dist = dist
                        break
                    dist += step_dist

            # Add minor Gaussian sensor noise to range measurement
            if hit_dist < cfg.laser_range_max:
                hit_dist = max(cfg.laser_range_min, hit_dist + float(self._rng.normal(0.0, cfg.laser_noise_std)))

            ranges.append(round(hit_dist, 3))

        angle_inc = (cfg.laser_angle_max - cfg.laser_angle_min) / max(1, cfg.laser_num_rays - 1)

        return LaserScanObservation(
            timestamp=gt.timestamp,
            frame_id="base_laser",
            angle_min=cfg.laser_angle_min,
            angle_max=cfg.laser_angle_max,
            angle_increment=angle_inc,
            range_min=cfg.laser_range_min,
            range_max=cfg.laser_range_max,
            ranges=tuple(ranges),
            status=True,
        )
