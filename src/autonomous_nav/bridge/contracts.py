"""Data contracts, message specifications, and enums for the simulation bridge.

Defines standardized, typed, timestamped data contracts conforming to robotics
coordinate frame conventions (REP-103, REP-105) and rigorous timing rules:

1. Coordinate Frame Conventions:
   - World / Map Frame: East-North-Up (ENU) right-handed Cartesian coordinate system:
     * +X axis points East.
     * +Y axis points North.
     * +Z axis points Up (gravity is -Z).
     * Heading theta or yaw angle is measured in radians [-pi, +pi], where 0 rad points
       East (+X) and positive rotations are counter-clockwise toward North (+Y).
   - Robot Body Frame (base_link): Forward-Left-Up (FLU) right-handed coordinate system:
     * +X axis points Forward (longitudinal vehicle axis).
     * +Y axis points Left (lateral vehicle axis).
     * +Z axis points Up.
     * Commanded linear velocity v_cmd is along body +X.
     * Commanded rotational velocity omega_cmd is positive counter-clockwise around body +Z.

2. Timing & Timestamp Semantics:
   - Simulation Clock: Advances strictly monotonically (dt > 0, math.isfinite(dt)).
   - Actuator Commands (NavigationCommand): Dispatched with non-decreasing timestamps
     (Delta t >= 0). Commands with timestamps in the past (t_cmd < t_sim - epsilon) are
     stale and rejected.
   - Synchronous Sensor Observations (SimulatorSensorPacket): Individual observation
     streams within a packet must share the identical step timestamp within 1ms tolerance.
     Consecutive packets across simulation steps are strictly monotonic (t_{k+1} > t_k).
   - Reset & Replay: reset() rewinds the clock to initial epoch t_0 (t_0 >= 0.0), returning
     the adapter to READY and resuming monotonic progression.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from autonomous_nav.simulation.robot import RobotState


class SimulationLifecycleState(str, Enum):
    """Lifecycle states of the external simulation runtime."""
    UNINITIALIZED = "uninitialized"
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    ERROR = "error"


class CoordinateFrame(str, Enum):
    """Standardized coordinate frame identifiers (REP-105)."""
    MAP = "map"
    ODOM = "odom"
    WORLD = "world"
    BASE_LINK = "base_link"
    BASE_LASER = "base_laser"
    BASE_IMU = "base_imu"
    BASE_GNSS = "base_gnss"


@dataclass(frozen=True)
class TimestampedRobotState:
    """Timestamped 3D kinematic robot state contract.

    Attributes:
        x: World-frame X coordinate in meters (East).
        y: World-frame Y coordinate in meters (North).
        z: World-frame Z coordinate in meters (Up).
        roll: Rotation around X axis in radians.
        pitch: Rotation around Y axis in radians.
        yaw: Rotation around Z axis (heading) in radians, [-pi, +pi].
        vx: World-frame or body-frame linear X velocity in m/s.
        vy: World-frame or body-frame linear Y velocity in m/s.
        vz: World-frame or body-frame linear Z velocity in m/s.
        v_linear: Scalar forward ground speed in m/s.
        omega_yaw: Yaw angular velocity in rad/s.
        timestamp: Monotonic simulation epoch timestamp in seconds.
        frame_id: Coordinate frame of pose measurements (e.g., 'world', 'map', 'odom').
        child_frame_id: Coordinate frame of the body (default 'base_link').
    """
    x: float
    y: float
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    v_linear: float = 0.0
    omega_yaw: float = 0.0
    timestamp: float = 0.0
    frame_id: str = "world"
    child_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        """Validate numeric bounds and finite value invariants."""
        floats_to_check = (
            ("x", self.x), ("y", self.y), ("z", self.z),
            ("roll", self.roll), ("pitch", self.pitch), ("yaw", self.yaw),
            ("vx", self.vx), ("vy", self.vy), ("vz", self.vz),
            ("v_linear", self.v_linear), ("omega_yaw", self.omega_yaw),
            ("timestamp", self.timestamp),
        )
        for name, val in floats_to_check:
            if not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f"State field '{name}' must be a finite float, got: {val}")

        if self.timestamp < 0.0:
            raise ValueError(f"State timestamp must be non-negative, got: {self.timestamp}")

        if not self.frame_id or not isinstance(self.frame_id, str):
            raise ValueError(f"Invalid frame_id: {self.frame_id}")

    def to_2d_robot_state(self) -> RobotState:
        """Project 3D pose and velocities onto the 2D planar RobotState."""
        return RobotState(
            x=self.x,
            y=self.y,
            theta=self.yaw,
            v=self.v_linear,
            omega=self.omega_yaw,
            timestamp=self.timestamp,
        )

    def as_quaternion(self) -> Tuple[float, float, float, float]:
        """Convert orientation Euler angles (roll, pitch, yaw) to unit quaternion (qx, qy, qz, qw)."""
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    @classmethod
    def from_quaternion(
        cls,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        v_linear: float = 0.0,
        omega_yaw: float = 0.0,
        timestamp: float = 0.0,
        frame_id: str = "world",
        child_frame_id: str = "base_link",
    ) -> TimestampedRobotState:
        """Construct a TimestampedRobotState from Cartesian positions and a quaternion (qx, qy, qz, qw)."""
        from autonomous_nav.bridge.mapping import quaternion_to_euler
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        return cls(
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            vx=vx,
            vy=vy,
            vz=vz,
            v_linear=v_linear,
            omega_yaw=omega_yaw,
            timestamp=timestamp,
            frame_id=frame_id,
            child_frame_id=child_frame_id,
        )


@dataclass(frozen=True)
class NavigationCommand:
    """Actuator command sent from navigation controller to simulation bridge.

    Attributes:
        v_cmd: Commanded longitudinal linear velocity in m/s.
        omega_cmd: Commanded rotational yaw velocity in rad/s.
        timestamp: Monotonic timestamp of the command dispatch in seconds.
        frame_id: Target body frame for velocity commands (default 'base_link').
        command_id: Monotonically increasing sequence identifier.
    """
    v_cmd: float
    omega_cmd: float
    timestamp: float = 0.0
    frame_id: str = "base_link"
    command_id: int = 0

    def __post_init__(self) -> None:
        """Validate numeric finiteness and non-negative timestamp."""
        if not math.isfinite(self.v_cmd) or not math.isfinite(self.omega_cmd):
            raise ValueError(
                f"NavigationCommand contains non-finite velocity: v_cmd={self.v_cmd}, omega_cmd={self.omega_cmd}"
            )
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError(f"NavigationCommand timestamp must be non-negative and finite, got: {self.timestamp}")
        if not self.frame_id or not isinstance(self.frame_id, str):
            raise ValueError(f"Invalid frame_id: {self.frame_id}")

    def validated_or_clamped(
        self,
        min_v: float = 0.0,
        max_v: float = 2.0,
        max_omega: float = 1.5,
        clamp: bool = True,
    ) -> NavigationCommand:
        """Enforce velocity limits via clamping or exception."""
        is_in_bounds = (min_v <= self.v_cmd <= max_v) and (-max_omega <= self.omega_cmd <= max_omega)
        if is_in_bounds:
            return self

        if not clamp:
            raise ValueError(
                f"Command ({self.v_cmd:.3f} m/s, {self.omega_cmd:.3f} rad/s) violates limits "
                f"[{min_v}, {max_v}] m/s, [{-max_omega}, {max_omega}] rad/s"
            )

        clamped_v = max(min_v, min(max_v, self.v_cmd))
        clamped_omega = max(-max_omega, min(max_omega, self.omega_cmd))
        return NavigationCommand(
            v_cmd=clamped_v,
            omega_cmd=clamped_omega,
            timestamp=self.timestamp,
            frame_id=self.frame_id,
            command_id=self.command_id,
        )


@dataclass(frozen=True)
class OdometryObservation:
    """Wheel/drivetrain odometry observation packet."""
    timestamp: float
    frame_id: str
    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s
    pose_xy: Tuple[float, float] = (0.0, 0.0)  # dead-reckoned odometry pose (meters)
    heading: float = 0.0  # dead-reckoned odometry yaw (rad)
    status: bool = True

    def __post_init__(self) -> None:
        for val, name in (
            (self.timestamp, "timestamp"),
            (self.linear_velocity, "linear_velocity"),
            (self.angular_velocity, "angular_velocity"),
            (self.pose_xy[0], "pose_xy[0]"),
            (self.pose_xy[1], "pose_xy[1]"),
            (self.heading, "heading"),
        ):
            if not math.isfinite(val):
                raise ValueError(f"OdometryObservation '{name}' must be finite, got: {val}")
        if self.timestamp < 0.0:
            raise ValueError(f"Odometry timestamp must be non-negative, got: {self.timestamp}")


@dataclass(frozen=True)
class ImuObservation:
    """Inertial Measurement Unit (IMU) observation packet."""
    timestamp: float
    frame_id: str
    angular_velocity_rpy: Tuple[float, float, float]  # rad/s (roll, pitch, yaw rates)
    linear_acceleration_xyz: Tuple[float, float, float]  # m/s^2 (X, Y, Z accelerations)
    orientation_rpy: Optional[Tuple[float, float, float]] = None  # rad (optional orientation)
    status: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError(f"IMU timestamp must be non-negative and finite, got: {self.timestamp}")
        for v in self.angular_velocity_rpy + self.linear_acceleration_xyz:
            if not math.isfinite(v):
                raise ValueError(f"IMU rates/accelerations must be finite, got: {v}")


@dataclass(frozen=True)
class GnssObservation:
    """Global Navigation Satellite System (GNSS) fix observation packet."""
    timestamp: float
    frame_id: str
    easting: float  # meters in local ENU Cartesian tangent plane
    northing: float  # meters in local ENU Cartesian tangent plane
    altitude: float = 0.0  # meters
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    hdop: float = 1.0  # horizontal dilution of precision
    status: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError(f"GNSS timestamp must be non-negative and finite, got: {self.timestamp}")
        for v, name in ((self.easting, "easting"), (self.northing, "northing"), (self.altitude, "altitude")):
            if not math.isfinite(v):
                raise ValueError(f"GNSS '{name}' must be finite, got: {v}")


@dataclass(frozen=True)
class LaserScanObservation:
    """Planar LiDAR laser scan observation packet (compatible with ROS 2 sensor_msgs/LaserScan)."""
    timestamp: float
    frame_id: str
    angle_min: float  # Start angle of scan [rad]
    angle_max: float  # End angle of scan [rad]
    angle_increment: float  # Angular distance between measurements [rad]
    range_min: float  # Minimum valid range [m]
    range_max: float  # Maximum valid range [m]
    ranges: Tuple[float, ...]  # Range data [m]
    intensities: Tuple[float, ...] = ()
    status: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError(f"LaserScan timestamp must be non-negative and finite, got: {self.timestamp}")
        for v, name in (
            (self.angle_min, "angle_min"),
            (self.angle_max, "angle_max"),
            (self.angle_increment, "angle_increment"),
            (self.range_min, "range_min"),
            (self.range_max, "range_max"),
        ):
            if not math.isfinite(v):
                raise ValueError(f"LaserScan '{name}' must be finite, got: {v}")

        if self.angle_increment <= 0.0:
            raise ValueError(f"LaserScan angle_increment must be positive, got: {self.angle_increment}")
        if self.range_min < 0.0 or self.range_max <= self.range_min:
            raise ValueError(f"Invalid range bounds: [{self.range_min}, {self.range_max}]")


@dataclass(frozen=True)
class SimulatorSensorPacket:
    """Synchronous aggregation of all sensor observations generated at a simulation step."""
    timestamp: float
    odometry: Optional[OdometryObservation] = None
    imu: Optional[ImuObservation] = None
    gnss: Optional[GnssObservation] = None
    laser_scan: Optional[LaserScanObservation] = None
    custom_telemetry: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError(f"SensorPacket timestamp must be non-negative and finite, got: {self.timestamp}")

        # Enforce intra-packet synchronization (all included observations must match packet timestamp)
        for obs, name in (
            (self.odometry, "odometry"),
            (self.imu, "imu"),
            (self.gnss, "gnss"),
            (self.laser_scan, "laser_scan"),
        ):
            if obs is not None and abs(obs.timestamp - self.timestamp) > 1e-3:
                raise ValueError(
                    f"SensorPacket timestamp mismatch for '{name}': packet has {self.timestamp}s, observation has {obs.timestamp}s"
                )


@dataclass(frozen=True)
class SimulatorCapabilities:
    """Static capability profile and dynamic connection health of the simulator backend."""
    simulator_name: str
    version: str
    clock_source: str  # "simulated_step", "real_time_clock", "wall_clock"
    supported_frames: Tuple[str, ...]
    supported_sensors: Tuple[str, ...]
    supports_lockstep_stepping: bool
    supports_dynamic_obstacles: bool
    supports_ground_truth_pose: bool
    is_connected: bool
    connection_endpoint: str = "in_process_mock"

