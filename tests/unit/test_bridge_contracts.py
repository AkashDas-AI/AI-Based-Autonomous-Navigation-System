"""Unit tests for simulation bridge data contracts and message validation."""

from __future__ import annotations

import math
import pytest

from autonomous_nav.bridge.contracts import (
    CoordinateFrame,
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


def test_timestamped_robot_state_valid_and_projection():
    """Verify valid 3D state instantiation and projection to 2D RobotState."""
    state = TimestampedRobotState(
        x=5.2,
        y=3.4,
        z=0.1,
        roll=0.01,
        pitch=-0.02,
        yaw=1.25,
        vx=1.5,
        vy=0.1,
        vz=0.0,
        v_linear=1.503,
        omega_yaw=0.35,
        timestamp=4.5,
        frame_id="world",
        child_frame_id="base_link",
    )

    assert state.x == 5.2
    assert state.y == 3.4
    assert state.z == 0.1
    assert state.yaw == 1.25
    assert state.timestamp == 4.5
    assert state.frame_id == "world"

    state_2d = state.to_2d_robot_state()
    assert state_2d.x == 5.2
    assert state_2d.y == 3.4
    assert state_2d.theta == 1.25
    assert state_2d.v == 1.503
    assert state_2d.omega == 0.35
    assert state_2d.timestamp == 4.5


@pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
def test_timestamped_robot_state_non_finite_rejected(bad_val: float):
    """Verify non-finite numeric floats raise ValueError."""
    with pytest.raises(ValueError, match="must be a finite float"):
        TimestampedRobotState(x=bad_val, y=0.0, yaw=0.0)

    with pytest.raises(ValueError, match="must be a finite float"):
        TimestampedRobotState(x=0.0, y=bad_val, yaw=0.0)

    with pytest.raises(ValueError, match="must be a finite float"):
        TimestampedRobotState(x=0.0, y=0.0, yaw=bad_val)

    with pytest.raises(ValueError, match="must be a finite float"):
        TimestampedRobotState(x=0.0, y=0.0, yaw=0.0, timestamp=bad_val)


def test_timestamped_robot_state_negative_timestamp_rejected():
    """Verify negative timestamps raise ValueError."""
    with pytest.raises(ValueError, match="timestamp must be non-negative"):
        TimestampedRobotState(x=0.0, y=0.0, yaw=0.0, timestamp=-0.001)


def test_navigation_command_validation_and_clamping():
    """Verify NavigationCommand limits enforcement (clamping vs error raising)."""
    # Valid command within default limits [0, 2.0] m/s and [-1.5, +1.5] rad/s
    cmd = NavigationCommand(v_cmd=1.2, omega_cmd=-0.8, timestamp=1.0)
    validated = cmd.validated_or_clamped(min_v=0.0, max_v=2.0, max_omega=1.5, clamp=True)
    assert validated.v_cmd == 1.2
    assert validated.omega_cmd == -0.8

    # Over-speed command clamped
    over_cmd = NavigationCommand(v_cmd=3.5, omega_cmd=-2.5, timestamp=1.0)
    clamped = over_cmd.validated_or_clamped(min_v=0.0, max_v=2.0, max_omega=1.5, clamp=True)
    assert clamped.v_cmd == 2.0
    assert clamped.omega_cmd == -1.5

    # Negative velocity clamped to min_v
    reverse_cmd = NavigationCommand(v_cmd=-0.5, omega_cmd=0.5, timestamp=1.0)
    clamped_rev = reverse_cmd.validated_or_clamped(min_v=0.0, max_v=2.0, max_omega=1.5, clamp=True)
    assert clamped_rev.v_cmd == 0.0

    # Over-speed command raises when clamp=False
    with pytest.raises(ValueError, match="violates limits"):
        over_cmd.validated_or_clamped(min_v=0.0, max_v=2.0, max_omega=1.5, clamp=False)


@pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
def test_navigation_command_non_finite_rejected(bad_val: float):
    """Verify NaN/Inf velocity commands are rejected."""
    with pytest.raises(ValueError, match="non-finite velocity"):
        NavigationCommand(v_cmd=bad_val, omega_cmd=0.0)

    with pytest.raises(ValueError, match="non-finite velocity"):
        NavigationCommand(v_cmd=1.0, omega_cmd=bad_val)


def test_sensor_observation_contracts():
    """Verify sensor observation contracts validate numeric constraints and frame IDs."""
    odom = OdometryObservation(
        timestamp=2.5,
        frame_id="odom",
        linear_velocity=1.2,
        angular_velocity=-0.3,
        pose_xy=(3.1, 4.2),
        heading=0.75,
    )
    assert odom.timestamp == 2.5
    assert odom.frame_id == "odom"

    imu = ImuObservation(
        timestamp=2.5,
        frame_id="base_imu",
        angular_velocity_rpy=(0.01, -0.01, -0.3),
        linear_acceleration_xyz=(0.1, 0.0, 9.81),
    )
    assert imu.linear_acceleration_xyz[2] == 9.81

    gnss = GnssObservation(
        timestamp=2.5,
        frame_id="world",
        easting=102.5,
        northing=50.3,
        altitude=15.0,
        hdop=0.9,
    )
    assert gnss.easting == 102.5

    laser = LaserScanObservation(
        timestamp=2.5,
        frame_id="base_laser",
        angle_min=-math.pi / 2,
        angle_max=math.pi / 2,
        angle_increment=math.pi / 180,
        range_min=0.1,
        range_max=10.0,
        ranges=(1.2, 2.5, 3.4),
    )
    assert len(laser.ranges) == 3

    packet = SimulatorSensorPacket(
        timestamp=2.5,
        odometry=odom,
        imu=imu,
        gnss=gnss,
        laser_scan=laser,
    )
    assert packet.timestamp == 2.5
    assert packet.laser_scan is not None


def test_laser_scan_invalid_parameters():
    """Verify LaserScanObservation raises on invalid range or angular step bounds."""
    # Negative angle increment
    with pytest.raises(ValueError, match="angle_increment must be positive"):
        LaserScanObservation(
            timestamp=1.0,
            frame_id="base_laser",
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=-0.1,
            range_min=0.1,
            range_max=10.0,
            ranges=(1.0,),
        )

    # Inverted range bounds
    with pytest.raises(ValueError, match="Invalid range bounds"):
        LaserScanObservation(
            timestamp=1.0,
            frame_id="base_laser",
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=0.1,
            range_min=10.0,
            range_max=5.0,
            ranges=(1.0,),
        )


def test_sensor_packet_intra_packet_timestamp_desynchronization_rejected():
    """Verify SimulatorSensorPacket raises ValueError when child observations are desynchronized."""
    odom_synced = OdometryObservation(
        timestamp=1.0, frame_id="odom", linear_velocity=1.0, angular_velocity=0.0
    )
    # Valid packet when timestamps match
    pkt = SimulatorSensorPacket(timestamp=1.0, odometry=odom_synced)
    assert pkt.timestamp == 1.0

    # Desynchronized observation timestamp (e.g. odom at 1.0s, packet at 1.05s)
    with pytest.raises(ValueError, match="SensorPacket timestamp mismatch for 'odometry'"):
        SimulatorSensorPacket(timestamp=1.05, odometry=odom_synced)

    # Desynchronized IMU
    imu_desync = ImuObservation(
        timestamp=2.0, frame_id="base_imu", angular_velocity_rpy=(0, 0, 0), linear_acceleration_xyz=(0, 0, 9.81)
    )
    with pytest.raises(ValueError, match="SensorPacket timestamp mismatch for 'imu'"):
        SimulatorSensorPacket(timestamp=1.0, imu=imu_desync)


def test_timestamped_robot_state_quaternion_conversions():
    """Verify TimestampedRobotState quaternion extraction and classmethod reconstruction."""
    state = TimestampedRobotState(
        x=3.0,
        y=4.0,
        z=0.5,
        roll=0.0,
        pitch=0.0,
        yaw=math.pi / 2,
        v_linear=1.0,
        omega_yaw=0.1,
        timestamp=2.0,
    )
    qx, qy, qz, qw = state.as_quaternion()
    # For yaw=pi/2, roll=0, pitch=0: qz = sin(pi/4) ~= 0.7071, qw = cos(pi/4) ~= 0.7071
    assert math.isclose(qx, 0.0, abs_tol=1e-5)
    assert math.isclose(qy, 0.0, abs_tol=1e-5)
    assert math.isclose(qz, math.sin(math.pi / 4), abs_tol=1e-5)
    assert math.isclose(qw, math.cos(math.pi / 4), abs_tol=1e-5)

    # Reconstruct from quaternion
    recon = TimestampedRobotState.from_quaternion(
        x=3.0,
        y=4.0,
        z=0.5,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        v_linear=1.0,
        omega_yaw=0.1,
        timestamp=2.0,
    )
    assert math.isclose(recon.x, 3.0)
    assert math.isclose(recon.y, 4.0)
    assert math.isclose(recon.z, 0.5)
    assert math.isclose(recon.yaw, math.pi / 2, abs_tol=1e-5)
    assert math.isclose(recon.v_linear, 1.0)


def test_quaternion_normalization_and_degenerate_rejection():
    """Verify quaternion_to_euler normalizes non-unit quaternions and rejects degenerate inputs."""
    from autonomous_nav.bridge.mapping import quaternion_to_euler

    # Scaled quaternion (norm != 1.0) should be normalized automatically
    scale = 3.5
    r, p, y = quaternion_to_euler(0.0 * scale, 0.0 * scale, math.sin(math.pi / 4) * scale, math.cos(math.pi / 4) * scale)
    assert math.isclose(r, 0.0, abs_tol=1e-5)
    assert math.isclose(p, 0.0, abs_tol=1e-5)
    assert math.isclose(y, math.pi / 2, abs_tol=1e-5)

    # Zero norm quaternion raises ValueError
    with pytest.raises(ValueError, match="Degenerate quaternion with near-zero norm"):
        quaternion_to_euler(0.0, 0.0, 0.0, 0.0)

    # NaN / Inf quaternion raises ValueError
    with pytest.raises(ValueError, match="must be a finite float"):
        quaternion_to_euler(float("nan"), 0.0, 0.0, 1.0)


