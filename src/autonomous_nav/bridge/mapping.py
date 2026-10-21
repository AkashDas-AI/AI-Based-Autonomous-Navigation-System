"""Mapping and transformation utilities connecting bridge data contracts with navigation components."""

from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np

from autonomous_nav.bridge.contracts import (
    LaserScanObservation,
    NavigationCommand,
    SimulatorSensorPacket,
    TimestampedRobotState,
)
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.sensors.sensor_models import SensorMeasurements
from autonomous_nav.simulation.robot import RobotState


def state_3d_to_2d_robot_state(state_3d: TimestampedRobotState) -> RobotState:
    """Project a 3D timestamped simulator robot state into a 2D planar RobotState."""
    return state_3d.to_2d_robot_state()


def robot_state_to_3d_state(
    state_2d: RobotState,
    z: float = 0.0,
    frame_id: str = "world",
    child_frame_id: str = "base_link",
) -> TimestampedRobotState:
    """Embed a 2D planar RobotState into a full 3D TimestampedRobotState contract."""
    vx = state_2d.v * math.cos(state_2d.theta)
    vy = state_2d.v * math.sin(state_2d.theta)
    return TimestampedRobotState(
        x=state_2d.x,
        y=state_2d.y,
        z=z,
        roll=0.0,
        pitch=0.0,
        yaw=state_2d.theta,
        vx=vx,
        vy=vy,
        vz=0.0,
        v_linear=state_2d.v,
        omega_yaw=state_2d.omega,
        timestamp=state_2d.timestamp,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
    )


def packet_to_sensor_measurements(packet: SimulatorSensorPacket) -> SensorMeasurements:
    """Translate a high-fidelity SimulatorSensorPacket into the navigation stack's SensorMeasurements.

    Extracts wheel odometry speed, IMU yaw rate, and GNSS 2D fix for direct EKF ingestion.
    """
    v_odom = packet.odometry.linear_velocity if packet.odometry is not None else 0.0
    omega_imu = packet.imu.angular_velocity_rpy[2] if packet.imu is not None else 0.0

    gnss_pos = None
    gnss_valid = False
    if packet.gnss is not None and packet.gnss.status:
        gnss_pos = (packet.gnss.easting, packet.gnss.northing)
        gnss_valid = True

    return SensorMeasurements(
        timestamp=packet.timestamp,
        v_odom=v_odom,
        omega_imu=omega_imu,
        gnss_pos=gnss_pos,
        gnss_valid=gnss_valid,
    )


def laser_scan_to_grid_map(
    scan: LaserScanObservation,
    robot_pose: RobotState,
    grid_map: GridMap,
    max_range_cutoff: Optional[float] = None,
) -> int:
    """Project a 2D planar LiDAR laser scan into an occupancy GridMap.

    Args:
        scan: LaserScanObservation containing radial range returns and angles.
        robot_pose: Current estimated or physical robot pose in world coordinates.
        grid_map: Destination GridMap to update.
        max_range_cutoff: Optional range ceiling beyond which returns are ignored.

    Returns:
        int: Number of discrete grid cells newly marked as occupied.
    """
    if not scan.status or not scan.ranges:
        return 0

    cutoff = max_range_cutoff if max_range_cutoff is not None else (scan.range_max - 0.05)
    cells_updated = 0

    for idx, r in enumerate(scan.ranges):
        # Discard invalid returns or maximum range free-space returns
        if r < scan.range_min or r >= cutoff:
            continue

        rel_angle = scan.angle_min + idx * scan.angle_increment
        abs_angle = robot_pose.theta + rel_angle

        hit_x = robot_pose.x + r * math.cos(abs_angle)
        hit_y = robot_pose.y + r * math.sin(abs_angle)

        grid_c, grid_r = grid_map.world_to_grid(hit_x, hit_y)
        if grid_map.is_valid(grid_c, grid_r):
            if not grid_map.is_occupied(grid_c, grid_r):
                grid_map.set_occupied(grid_c, grid_r, occupied=True)
                cells_updated += 1

    return cells_updated


def control_to_nav_command(
    v_cmd: float,
    omega_cmd: float,
    timestamp: float,
    frame_id: str = "base_link",
    command_id: int = 0,
) -> NavigationCommand:
    """Package raw linear and angular control outputs into a validated NavigationCommand."""
    return NavigationCommand(
        v_cmd=v_cmd,
        omega_cmd=omega_cmd,
        timestamp=timestamp,
        frame_id=frame_id,
        command_id=command_id,
    )


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert orientation quaternion (qx, qy, qz, qw) to Euler angles (roll, pitch, yaw) in radians.

    Validates numeric finiteness, rejects degenerate or zero-norm quaternions, and
    normalizes the quaternion prior to trigonometric projection. Follows ROS/REP-103
    (qx, qy, qz, qw) ordering where qw is the scalar real component.
    """
    for val, name in ((qx, "qx"), (qy, "qy"), (qz, "qz"), (qw, "qw")):
        if not math.isfinite(val):
            raise ValueError(f"Quaternion component '{name}' must be a finite float, got: {val}")

    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    if norm_sq < 1e-12:
        raise ValueError(f"Degenerate quaternion with near-zero norm: ({qx}, {qy}, {qz}, {qw})")

    norm = math.sqrt(norm_sq)
    qx_n = qx / norm
    qy_n = qy / norm
    qz_n = qz / norm
    qw_n = qw / norm

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw_n * qx_n + qy_n * qz_n)
    cosr_cosp = 1.0 - 2.0 * (qx_n * qx_n + qy_n * qy_n)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw_n * qy_n - qz_n * qx_n)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # gimbal lock
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw_n * qz_n + qx_n * qy_n)
    cosy_cosp = 1.0 - 2.0 * (qy_n * qy_n + qz_n * qz_n)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (roll, pitch, yaw) in radians to orientation quaternion (qx, qy, qz, qw)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw
