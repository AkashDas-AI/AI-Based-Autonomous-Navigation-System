"""Unit tests for RobotModel kinematics and PurePursuitController."""

import math
import pytest
from autonomous_nav.config import ControllerConfig, RobotConfig
from autonomous_nav.navigation.controller import PurePursuitController
from autonomous_nav.navigation.waypoint_manager import WaypointManager
from autonomous_nav.simulation.robot import RobotModel, RobotState


def test_robot_kinematics_straight():
    """Verify robot moves purely forward when omega is zero."""
    model = RobotModel(RobotConfig(max_acceleration=10.0, max_velocity=2.0))
    model.reset(x=0.0, y=0.0, theta=0.0)

    # Step for 1 second with v=1.0, omega=0.0
    state = model.step(v_cmd=1.0, omega_cmd=0.0, dt=1.0)
    assert pytest.approx(state.x, abs=0.01) == 1.0
    assert pytest.approx(state.y, abs=0.01) == 0.0
    assert pytest.approx(state.theta, abs=0.01) == 0.0


def test_robot_kinematics_turn():
    """Verify robot heading updates according to commanded angular velocity."""
    model = RobotModel(RobotConfig(max_acceleration=10.0, max_angular_velocity=2.0))
    model.reset(x=0.0, y=0.0, theta=0.0)

    # Step with angular rate pi/2 for 1 second
    state = model.step(v_cmd=0.0, omega_cmd=math.pi / 2.0, dt=1.0)
    assert pytest.approx(state.theta, abs=0.01) == math.pi / 2.0


def test_pure_pursuit_heading_correction():
    """Verify controller commands positive angular velocity when target is to the left."""
    controller = PurePursuitController(ControllerConfig(kp_angular=2.0, target_velocity=1.0))
    # Robot facing along +X (theta = 0)
    state = RobotState(x=0.0, y=0.0, theta=0.0, v=1.0, omega=0.0)
    # Target is at (5.0, 5.0) -> 45 degrees left
    target = (5.0, 5.0)

    v_cmd, omega_cmd = controller.compute_control(state, target, distance_to_goal=7.0)
    assert v_cmd > 0.0
    assert omega_cmd > 0.0  # Steer left towards target


def test_waypoint_manager_progression():
    """Verify waypoint manager tracks and advances waypoints as robot progresses."""
    waypoints = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    wm = WaypointManager(waypoints=waypoints, acceptance_radius=0.5)

    assert wm.current_index == 0
    # Robot near first waypoint (0.1, 0.0) should advance to index 1
    wm.update(0.1, 0.0)
    assert wm.current_index == 1

    # Robot near second waypoint (2.1, 0.0) should advance to final index 2
    wm.update(2.1, 0.0)
    assert wm.current_index == 2

    # Goal reached check
    assert not wm.is_goal_reached(2.1, 0.0, tolerance=0.5)
    assert wm.is_goal_reached(3.9, 0.0, tolerance=0.5)

