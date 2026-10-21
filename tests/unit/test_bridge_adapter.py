"""Unit tests for OfflineTestAdapter lifecycle, deterministic physics, and failure modes."""

from __future__ import annotations

import math
import pytest
import numpy as np

from autonomous_nav.bridge.config import BridgeConfig
from autonomous_nav.bridge.contracts import (
    NavigationCommand,
    SimulationLifecycleState,
    SimulatorSensorPacket,
    TimestampedRobotState,
)
from autonomous_nav.bridge.mock_adapter import OfflineTestAdapter
from autonomous_nav.environment.grid_map import GridMap


def test_offline_test_adapter_lifecycle_state_transitions():
    """Verify clean lifecycle state machine transitions across connect, initialize, step, reset, disconnect."""
    adapter = OfflineTestAdapter()
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.UNINITIALIZED
    assert not adapter.is_healthy()

    # Connect
    ok = adapter.connect()
    assert ok
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.READY
    assert adapter.is_healthy()

    # Initialize
    init_state = TimestampedRobotState(x=2.0, y=3.0, yaw=0.5, timestamp=0.0)
    ok = adapter.initialize(init_state)
    assert ok
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.READY
    gt = adapter.get_ground_truth_state()
    assert gt.x == 2.0 and gt.y == 3.0 and gt.yaw == 0.5

    # Step
    adapter.send_command(NavigationCommand(v_cmd=1.0, omega_cmd=0.0, timestamp=0.0))
    ok = adapter.step(dt=0.1)
    assert ok
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.RUNNING

    # Reset
    ok = adapter.reset()
    assert ok
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.READY
    assert adapter.get_ground_truth_state().x == 2.0  # reseeded to initial state

    # Disconnect
    adapter.disconnect()
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.STOPPED
    assert not adapter.is_healthy()


def test_offline_test_adapter_unconnected_operations_raise_error():
    """Verify operations prior to connect() raise ConnectionError."""
    adapter = OfflineTestAdapter()

    with pytest.raises(ConnectionError, match="disconnected"):
        adapter.initialize()

    with pytest.raises(ConnectionError, match="disconnected"):
        adapter.step(dt=0.1)

    with pytest.raises(ConnectionError, match="disconnected"):
        adapter.reset()

    with pytest.raises(ConnectionError, match="disconnected"):
        adapter.send_command(NavigationCommand(v_cmd=1.0, omega_cmd=0.0))


def test_offline_test_adapter_connection_timeout_failure():
    """Verify non-positive timeout triggers connection failure and ERROR state."""
    adapter = OfflineTestAdapter()
    ok = adapter.connect(timeout_sec=-1.0)
    assert not ok
    assert adapter.get_lifecycle_state() == SimulationLifecycleState.ERROR
    assert not adapter.is_healthy()


def test_offline_test_adapter_deterministic_replay():
    """Verify identical RNG seed produces byte-for-byte identical state trajectories and sensor streams."""
    def run_simulation(seed: int):
        adapter = OfflineTestAdapter(seed=seed)
        adapter.connect()
        adapter.initialize(TimestampedRobotState(x=0.0, y=0.0, yaw=0.0, timestamp=0.0))

        traj = []
        odom_speeds = []
        laser_first_rays = []

        for i in range(20):
            cmd = NavigationCommand(v_cmd=1.0, omega_cmd=0.2, timestamp=i * 0.1)
            adapter.send_command(cmd)
            adapter.step(dt=0.1)

            gt = adapter.get_ground_truth_state()
            traj.append((gt.x, gt.y, gt.yaw))

            obs = adapter.get_sensor_observations()
            odom_speeds.append(obs.odometry.linear_velocity)
            laser_first_rays.append(obs.laser_scan.ranges[0])

        adapter.disconnect()
        return traj, odom_speeds, laser_first_rays

    traj1, odom1, laser1 = run_simulation(seed=123)
    traj2, odom2, laser2 = run_simulation(seed=123)
    traj_diff, odom_diff, laser_diff = run_simulation(seed=999)

    # Identical seed -> identical outputs
    assert np.allclose(traj1, traj2, atol=1e-12)
    assert np.allclose(odom1, odom2, atol=1e-12)
    assert np.allclose(laser1, laser2, atol=1e-12)

    # Different seed -> identical kinematic trajectory (deterministic control) but different sensor noise
    assert np.allclose(traj1, traj_diff, atol=1e-12)
    assert not np.allclose(odom1, odom_diff, atol=1e-5)


def test_offline_test_adapter_command_validation_rules():
    """Verify adapter enforces frame whitelist, timestamp monotonicity, and limits."""
    adapter = OfflineTestAdapter(config=BridgeConfig(clamp_commands=False))
    adapter.connect()
    adapter.initialize()

    # Unsupported coordinate frame
    bad_frame_cmd = NavigationCommand(v_cmd=1.0, omega_cmd=0.0, frame_id="unsupported_custom_frame")
    with pytest.raises(ValueError, match="Unsupported command frame_id"):
        adapter.send_command(bad_frame_cmd)

    # Velocity exceeding limits when clamp_commands=False
    excess_cmd = NavigationCommand(v_cmd=5.0, omega_cmd=0.0, frame_id="base_link")
    with pytest.raises(ValueError, match="violates limits"):
        adapter.send_command(excess_cmd)

    # Step forward in time
    adapter.send_command(NavigationCommand(v_cmd=1.0, omega_cmd=0.0, timestamp=0.0))
    adapter.step(0.5)

    # Time-travel command (timestamp in past)
    stale_cmd = NavigationCommand(v_cmd=1.0, omega_cmd=0.0, timestamp=0.1)
    with pytest.raises(ValueError, match="earlier than current simulation time"):
        adapter.send_command(stale_cmd)


def test_offline_test_adapter_laser_raycasting_against_map():
    """Verify simulated planar LiDAR detects obstacles in GridMap at correct distances."""
    grid = GridMap(width=40, height=40, resolution=0.5, origin=(0.0, 0.0))
    # Place a 2m x 2m obstacle wall at X=5.0m, Y=0.0m to 2.0m
    for x_m in np.arange(5.0, 6.0, 0.25):
        for y_m in np.arange(0.0, 2.0, 0.25):
            c, r = grid.world_to_grid(x_m, y_m)
            grid.set_occupied(c, r, occupied=True)

    adapter = OfflineTestAdapter(
        grid_map=grid,
        config=BridgeConfig(laser_angle_min=-math.pi / 2, laser_angle_max=math.pi / 2, laser_num_rays=181),
    )
    adapter.connect()
    # Robot at (0.0, 1.0), pointing East (yaw=0.0) -> obstacle at (5.0, 1.0) is directly ahead at 5.0m
    adapter.initialize(TimestampedRobotState(x=0.0, y=1.0, yaw=0.0, timestamp=0.0))

    obs = adapter.get_sensor_observations()
    laser = obs.laser_scan
    assert laser is not None

    # Middle ray (index 90 for 181 rays) points directly forward along yaw=0.0
    mid_idx = 90
    forward_dist = laser.ranges[mid_idx]
    # Expected distance is ~5.0m (+- resolution/noise margin)
    assert 4.7 <= forward_dist <= 5.3


def test_offline_test_adapter_ground_truth_isolation():
    """Verify strict separation between physical truth and noisy sensor observations."""
    adapter = OfflineTestAdapter(seed=42)
    adapter.connect()
    adapter.initialize(TimestampedRobotState(x=10.0, y=20.0, yaw=1.0, timestamp=0.0))

    for _ in range(5):
        adapter.send_command(NavigationCommand(v_cmd=1.5, omega_cmd=0.1, timestamp=adapter._current_time))
        adapter.step(0.1)

    gt = adapter.get_ground_truth_state()
    sensors = adapter.get_sensor_observations()

    # Ground truth is distinct object from sensor packet
    assert isinstance(gt, TimestampedRobotState)
    assert isinstance(sensors, SimulatorSensorPacket)

    # Measured odometry velocity differs from ground truth velocity due to scale and noise
    assert sensors.odometry is not None
    assert sensors.odometry.linear_velocity != gt.v_linear

    # Measured IMU yaw rate differs from ground truth
    assert sensors.imu is not None
    assert sensors.imu.angular_velocity_rpy[2] != gt.omega_yaw
