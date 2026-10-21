"""Unit tests for Dynamic Window Approach (DWA) local planner."""

import math
import pytest
from autonomous_nav.config import DWAConfig, RobotConfig
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.navigation.dwa import DWAPlanner
from autonomous_nav.simulation.robot import RobotState


def test_dynamic_window_bounds() -> None:
    """Verify dynamic window adheres to acceleration limits and velocity limits."""
    robot_cfg = RobotConfig(
        max_velocity=2.0,
        min_velocity=0.0,
        max_angular_velocity=1.0,
        max_acceleration=1.0,
        max_angular_acceleration=2.0,
    )
    dwa_cfg = DWAConfig()
    planner = DWAPlanner(dwa_cfg, robot_cfg)

    # Robot at rest: v=0, omega=0, dt=0.1
    # a_max * dt = 0.1
    v_low, v_high, om_low, om_high = planner.compute_dynamic_window(current_v=0.0, current_omega=0.0, dt=0.1)
    assert pytest.approx(v_low, abs=1e-3) == 0.0
    assert pytest.approx(v_high, abs=1e-3) == 0.1
    assert pytest.approx(om_low, abs=1e-3) == -0.2
    assert pytest.approx(om_high, abs=1e-3) == 0.2

    # Robot at high speed: v=1.9, dt=0.2 -> v_high capped at 2.0
    v_low, v_high, _, _ = planner.compute_dynamic_window(current_v=1.9, current_omega=0.0, dt=0.2)
    assert pytest.approx(v_high, abs=1e-3) == 2.0


def test_simulate_trajectory_straight() -> None:
    """Verify trajectory simulation for straight motion (omega=0)."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    traj = planner.simulate_trajectory(
        x=0.0, y=0.0, theta=0.0, v=1.0, omega=0.0, predict_time=1.0, dt=0.1
    )
    assert len(traj) == 10
    final_x, final_y, final_theta = traj[-1]
    assert pytest.approx(final_x, abs=0.05) == 1.0
    assert pytest.approx(final_y, abs=1e-3) == 0.0
    assert pytest.approx(final_theta, abs=1e-3) == 0.0


def test_trajectory_static_collision_filtering() -> None:
    """Verify trajectory candidate crossing a wall is flagged as in collision."""
    planner = DWAPlanner(DWAConfig(safety_margin=0.1), RobotConfig())
    grid_map = GridMap(width=20, height=20, resolution=0.5)
    # Add a wall at x in [4.0, 5.0], y in [0.0, 10.0]
    grid_map.add_rectangular_obstacle(4.0, 0.0, 5.0, 10.0)

    # Straight trajectory from (2.0, 5.0) heading in +x direction towards wall at x=4.0
    traj_blocked = planner.simulate_trajectory(2.0, 5.0, 0.0, v=2.0, omega=0.0, predict_time=2.0, dt=0.1)
    collides, _ = planner.check_trajectory_collision(
        traj_blocked, grid_map, dynamic_obstacles=[], robot_radius=0.4, safety_margin=0.1, dt_step=0.1
    )
    assert collides is True

    # Upward trajectory away from wall (robot at (2.0, 5.0) moving along +y to y=7.0)
    traj_clear = planner.simulate_trajectory(2.0, 5.0, math.pi / 2, v=1.0, omega=0.0, predict_time=1.0, dt=0.1)
    collides_clear, clearance = planner.check_trajectory_collision(
        traj_clear, grid_map, dynamic_obstacles=[], robot_radius=0.4, safety_margin=0.1, dt_step=0.1
    )
    assert collides_clear is False
    assert clearance > 0.0


def test_trajectory_dynamic_collision_detection() -> None:
    """Verify DWA detects future collision with moving obstacle along predicted trajectory."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    # Obstacle moving vertically across (3.0, 5.0) at t=1.0s
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(3.0, 3.0),
        end_position=(3.0, 7.0),
        radius=0.5,
        speed=2.0,
        pattern="linear_ping_pong",
    )
    # Robot at (1.0, 5.0) heading towards (5.0, 5.0) at v=2.0 m/s
    traj = planner.simulate_trajectory(1.0, 5.0, 0.0, v=2.0, omega=0.0, predict_time=2.0, dt=0.1)
    collides, _ = planner.check_trajectory_collision(
        traj, grid_map=None, dynamic_obstacles=[obs], robot_radius=0.4, safety_margin=0.1, dt_step=0.1
    )
    assert collides is True


def test_emergency_stop_when_fully_trapped() -> None:
    """Verify emergency stop (v=0, omega=0, is_safe=False) when all directions are blocked."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    grid_map = GridMap(width=20, height=20, resolution=0.5)
    # Surround robot at (5.0, 5.0) with walls
    grid_map.add_rectangular_obstacle(4.0, 3.5, 6.0, 4.2)
    grid_map.add_rectangular_obstacle(4.0, 5.8, 6.0, 6.5)
    grid_map.add_rectangular_obstacle(3.5, 4.0, 4.2, 6.0)
    grid_map.add_rectangular_obstacle(5.8, 4.0, 6.5, 6.0)

    state = RobotState(x=5.0, y=5.0, theta=0.0, v=0.0, omega=0.0)
    sol = planner.compute_command(
        state=state,
        target_point=(10.0, 5.0),
        grid_map=grid_map,
        dynamic_obstacles=[],
        robot_radius=0.4,
        dt=0.1,
    )
    assert sol.is_safe is False
    assert sol.v_cmd == 0.0
    assert sol.omega_cmd == 0.0


def test_dwa_determinism() -> None:
    """Verify identical inputs yield bit-exact commands and trajectories."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    grid_map = GridMap(width=30, height=30, resolution=0.5)
    grid_map.add_circular_obstacle(10.0, 10.0, 1.5)
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(10.0, 8.0),
        end_position=(10.0, 12.0),
        radius=0.5,
        speed=1.0,
    )
    state = RobotState(x=5.0, y=10.0, theta=0.0, v=1.0, omega=0.0)

    sol1 = planner.compute_command(state, (15.0, 10.0), grid_map, [obs])
    sol2 = planner.compute_command(state, (15.0, 10.0), grid_map, [obs])

    assert sol1.v_cmd == sol2.v_cmd
    assert sol1.omega_cmd == sol2.omega_cmd
    assert sol1.is_safe == sol2.is_safe
    assert sol1.score == sol2.score
    assert sol1.best_trajectory == sol2.best_trajectory
