"""Integration tests demonstrating the bridge decoupled from simulation with existing navigation stack."""

from __future__ import annotations

import math
import pytest
import numpy as np

from autonomous_nav.bridge import (
    BridgeConfig,
    NavigationCommand,
    OfflineTestAdapter,
    TimestampedRobotState,
    control_to_nav_command,
    euler_to_quaternion,
    laser_scan_to_grid_map,
    packet_to_sensor_measurements,
    quaternion_to_euler,
    state_3d_to_2d_robot_state,
)
from autonomous_nav.config import ControllerConfig
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.estimation.ekf import ExtendedKalmanFilter
from autonomous_nav.navigation.controller import PurePursuitController
from autonomous_nav.navigation.waypoint_manager import WaypointManager


def test_bridge_closed_loop_navigation_pure_pursuit():
    """Verify autonomous vehicle tracks waypoints in closed-loop through the simulation bridge."""
    waypoints = [(0.0, 0.0), (4.0, 0.0), (8.0, 4.0), (12.0, 4.0)]
    waypoint_mgr = WaypointManager(waypoints=waypoints, acceptance_radius=0.8)
    controller = PurePursuitController(
        config=ControllerConfig(lookahead_distance=1.0, min_lookahead_distance=0.5, target_velocity=1.5)
    )

    adapter = OfflineTestAdapter(seed=42)
    adapter.connect()
    adapter.initialize(TimestampedRobotState(x=0.0, y=0.0, yaw=0.0, timestamp=0.0))

    dt = 0.1
    max_steps = 250
    steps = 0
    reached = False

    while steps < max_steps:
        gt_3d = adapter.get_ground_truth_state()
        robot_2d = state_3d_to_2d_robot_state(gt_3d)

        if waypoint_mgr.is_goal_reached(robot_2d.x, robot_2d.y, tolerance=0.5):
            reached = True
            break

        waypoint_mgr.update(robot_2d.x, robot_2d.y)
        target = waypoint_mgr.get_lookahead_point(robot_2d.x, robot_2d.y, lookahead_distance=1.0)
        dist_to_goal = waypoint_mgr.distance_to_goal(robot_2d.x, robot_2d.y)

        v_cmd, omega_cmd = controller.compute_control(
            state=robot_2d,
            target_point=target,
            distance_to_goal=dist_to_goal,
        )

        nav_cmd = control_to_nav_command(
            v_cmd=v_cmd,
            omega_cmd=omega_cmd,
            timestamp=gt_3d.timestamp,
            command_id=steps,
        )
        adapter.send_command(nav_cmd)
        adapter.step(dt=dt)
        steps += 1

    adapter.disconnect()

    assert reached, f"Goal was not reached within {max_steps} steps. Final pose: ({robot_2d.x:.2f}, {robot_2d.y:.2f})"
    assert steps < 200


def test_bridge_sensor_packet_to_ekf_fusion():
    """Verify streaming bridge sensor observations into EKF state estimator converges pose."""
    adapter = OfflineTestAdapter(seed=42)
    adapter.connect()
    adapter.initialize(TimestampedRobotState(x=2.0, y=5.0, yaw=0.0, timestamp=0.0))

    ekf = ExtendedKalmanFilter()
    ekf.reset(x=2.0, y=5.0, theta=0.0, v=0.0)

    dt = 0.1
    for step_idx in range(60):
        # Accelerate forward with slight turning rate
        cmd = NavigationCommand(v_cmd=1.2, omega_cmd=0.1, timestamp=step_idx * dt)
        adapter.send_command(cmd)
        adapter.step(dt)

        packet = adapter.get_sensor_observations()
        # Translate bridge packet to EKF-consumable SensorMeasurements
        meas = packet_to_sensor_measurements(packet)

        # Ingest into EKF
        ekf.predict(omega_imu=meas.omega_imu, dt=dt)
        ekf.update_odometry(v_odom=meas.v_odom)
        if meas.gnss_valid and meas.gnss_pos is not None:
            ekf.update_gnss(x_gnss=meas.gnss_pos[0], y_gnss=meas.gnss_pos[1])

    gt = adapter.get_ground_truth_state()
    est = ekf.get_estimated_robot_state()

    pos_error = math.hypot(gt.x - est.x, gt.y - est.y)
    adapter.disconnect()

    # Bounded error consistent with sensor noise floor (< 0.45m)
    assert pos_error < 0.45, f"EKF estimation error too large: {pos_error:.3f}m"


def test_laser_scan_projection_to_grid_map():
    """Verify projecting LiDAR range returns from bridge into GridMap marks occupied cells."""
    # Obstacle map with a vertical wall at X=6.0m
    env_map = GridMap(width=50, height=40, resolution=0.5, origin=(0.0, 0.0))
    for y_m in np.arange(2.0, 8.0, 0.25):
        c, r = env_map.world_to_grid(6.0, y_m)
        env_map.set_occupied(c, r, occupied=True)

    adapter = OfflineTestAdapter(
        grid_map=env_map,
        config=BridgeConfig(laser_angle_min=-math.pi / 2, laser_angle_max=math.pi / 2, laser_num_rays=181),
    )
    adapter.connect()
    # Robot at (2.0, 5.0) pointing East (0.0 rad) directly toward the wall 4m away
    adapter.initialize(TimestampedRobotState(x=2.0, y=5.0, yaw=0.0, timestamp=0.0))

    obs = adapter.get_sensor_observations()
    laser = obs.laser_scan
    assert laser is not None

    # Empty perceived map
    perceived_map = GridMap(width=50, height=40, resolution=0.5, origin=(0.0, 0.0))
    gt_2d = state_3d_to_2d_robot_state(adapter.get_ground_truth_state())

    new_cells = laser_scan_to_grid_map(scan=laser, robot_pose=gt_2d, grid_map=perceived_map)
    adapter.disconnect()

    assert new_cells > 0
    # Check that perceived map has marked occupied cells along X=6.0m
    wall_col, _ = perceived_map.world_to_grid(6.0, 5.0)
    col_has_occupied = any(perceived_map.is_occupied(wall_col, r) for r in range(40))
    assert col_has_occupied


@pytest.mark.parametrize(
    "roll,pitch,yaw",
    [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2),
        (0.0, 0.0, -math.pi / 2),
        (0.1, -0.2, 1.35),
        (-0.3, 0.25, -2.1),
    ],
)
def test_quaternion_and_euler_roundtrip_transforms(roll: float, pitch: float, yaw: float):
    """Verify bidirectional mathematical consistency of quaternion and Euler conversions."""
    qx, qy, qz, qw = euler_to_quaternion(roll=roll, pitch=pitch, yaw=yaw)

    # Quaternion norm must equal 1.0
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    assert abs(norm - 1.0) < 1e-6

    rec_roll, rec_pitch, rec_yaw = quaternion_to_euler(qx, qy, qz, qw)
    assert abs(roll - rec_roll) < 1e-5
    assert abs(pitch - rec_pitch) < 1e-5
    assert abs(yaw - rec_yaw) < 1e-5


def test_bridge_closed_loop_ekf_feedback_isolated_navigation():
    """Verify closed-loop navigation through the bridge where the controller uses ONLY EKF feedback.

    Architectural Invariant: Ground truth is never supplied to the controller or filter.
    Navigation controller operates strictly on EKF estimated states updated via bridge sensor packets.
    """
    waypoints = [(0.0, 0.0), (3.0, 0.0), (6.0, 3.0), (9.0, 3.0)]
    waypoint_mgr = WaypointManager(waypoints=waypoints, acceptance_radius=0.8)
    controller = PurePursuitController(
        config=ControllerConfig(lookahead_distance=1.0, min_lookahead_distance=0.5, target_velocity=1.2)
    )

    adapter = OfflineTestAdapter(seed=42)
    adapter.connect()
    adapter.initialize(TimestampedRobotState(x=0.0, y=0.0, yaw=0.0, timestamp=0.0))

    ekf = ExtendedKalmanFilter()
    ekf.reset(x=0.0, y=0.0, theta=0.0, v=0.0)

    dt = 0.1
    max_steps = 250
    steps = 0
    reached = False

    while steps < max_steps:
        sim_time = round(steps * dt, 4)
        # Controller consumes ONLY the EKF estimated state
        est_state = ekf.get_estimated_robot_state(timestamp=sim_time)

        if waypoint_mgr.is_goal_reached(est_state.x, est_state.y, tolerance=0.5):
            reached = True
            break

        waypoint_mgr.update(est_state.x, est_state.y)
        target = waypoint_mgr.get_lookahead_point(est_state.x, est_state.y, lookahead_distance=1.0)
        dist_to_goal = waypoint_mgr.distance_to_goal(est_state.x, est_state.y)

        v_cmd, omega_cmd = controller.compute_control(
            state=est_state,
            target_point=target,
            distance_to_goal=dist_to_goal,
        )

        nav_cmd = control_to_nav_command(
            v_cmd=v_cmd,
            omega_cmd=omega_cmd,
            timestamp=sim_time,
            command_id=steps,
        )
        adapter.send_command(nav_cmd)
        adapter.step(dt=dt)

        # Ingest sensor packet into EKF
        packet = adapter.get_sensor_observations()
        meas = packet_to_sensor_measurements(packet)
        ekf.predict(omega_imu=meas.omega_imu, dt=dt)
        ekf.update_odometry(v_odom=meas.v_odom)
        if meas.gnss_valid and meas.gnss_pos is not None:
            ekf.update_gnss(x_gnss=meas.gnss_pos[0], y_gnss=meas.gnss_pos[1])

        steps += 1

    # At the end, query ground truth for evaluation only
    gt_final = adapter.get_ground_truth_state()
    adapter.disconnect()

    assert reached, f"Goal was not reached within {max_steps} steps."
    # Physical robot must also be at the goal (within goal acceptance radius + noise margin)
    final_gt_dist = math.hypot(gt_final.x - 9.0, gt_final.y - 3.0)
    assert final_gt_dist < 0.8, f"Physical robot did not reach goal: dist={final_gt_dist:.3f}m"

