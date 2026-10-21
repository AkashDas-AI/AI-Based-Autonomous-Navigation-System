"""Unit tests for temporal collision checking, continuous swept distance, and DWA safety bounds."""

import math
import pytest
from autonomous_nav.config import DWAConfig, RobotConfig, SimulationConfig
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.navigation.dwa import DWAPlanner
from autonomous_nav.simulation.collision import CollisionChecker
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.simulation.robot import RobotState


def test_temporal_tunneling_continuous_relative_distance() -> None:
    """Verify that continuous relative swept distance detects tunneling missed by discrete sampling."""
    # Robot moves from (0, 0) to (1.0, 0) in dt=0.1s
    p0 = (0.0, 0.0)
    p1 = (1.0, 0.0)
    # Fast obstacle crosses vertically from (0.5, -0.8) to (0.5, 0.8) in dt=0.1s (speed = 16 m/s)
    o0 = (0.5, -0.8)
    o1 = (0.5, 0.8)

    robot_radius = 0.4
    obs_radius = 0.4
    threshold = robot_radius + obs_radius  # 0.8m

    # 1. Discrete endpoint distances
    d_start = math.hypot(p0[0] - o0[0], p0[1] - o0[1])  # hypot(-0.5, 0.8) = 0.943m
    d_end = math.hypot(p1[0] - o1[0], p1[1] - o1[1])    # hypot(0.5, -0.8) = 0.943m
    assert d_start > threshold  # Discrete check at t=0 misses collision!
    assert d_end > threshold    # Discrete check at t=0.1 misses collision!

    # 2. Continuous relative swept distance
    d_swept = CollisionChecker.relative_continuous_distance(p0, p1, o0, o1)
    # At t=0.05s, both are at (0.5, 0.0) -> exact distance is 0.0m
    assert pytest.approx(d_swept, abs=1e-5) == 0.0
    assert d_swept <= threshold  # Swept check successfully detects collision!


def test_close_pass_and_near_tangency() -> None:
    """Verify continuous relative distance calculation on near-tangent passing paths."""
    # Robot moves straight along y=0 from x=0 to x=2
    p0 = (0.0, 0.0)
    p1 = (2.0, 0.0)
    # Obstacle moves along y=1.5 from x=2 to x=0 (opposing lane, closest distance = 1.5m at midpoint)
    o0 = (2.0, 1.5)
    o1 = (0.0, 1.5)

    d_swept = CollisionChecker.relative_continuous_distance(p0, p1, o0, o1)
    assert pytest.approx(d_swept, abs=1e-3) == 1.5


def test_obstacle_moving_toward_and_away() -> None:
    """Verify swept distance when obstacle moves directly head-on vs away."""
    # Head-on: Robot at (0, 0) -> (1, 0), Obstacle at (5, 0) -> (3, 0)
    d_head_on = CollisionChecker.relative_continuous_distance(
        (0.0, 0.0), (1.0, 0.0), (5.0, 0.0), (3.0, 0.0)
    )
    # Closest approach is at endpoint: (3 - 1) = 2.0m
    assert pytest.approx(d_head_on, abs=1e-3) == 2.0

    # Moving away: Robot at (0, 0) -> (1, 0), Obstacle at (3, 0) -> (5, 0)
    d_away = CollisionChecker.relative_continuous_distance(
        (0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (5.0, 0.0)
    )
    # Closest approach is at start: (3 - 0) = 3.0m
    assert pytest.approx(d_away, abs=1e-3) == 3.0


def test_fast_crossing_obstacle_swept_rejection_in_dwa() -> None:
    """Verify DWA trajectory checker rejects a fast-crossing candidate that collides between steps."""
    planner = DWAPlanner(DWAConfig(safety_margin=0.1), RobotConfig())
    # Obstacle crossing at x=2.0 from y=0 to y=6.0 at high speed
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(2.0, 0.0),
        end_position=(2.0, 6.0),
        radius=0.5,
        speed=6.0,
        pattern="constant_velocity",
        velocity=(0.0, 6.0),
    )
    # Robot moving straight towards x=4.0 at v=2.0 m/s
    traj = planner.simulate_trajectory(0.0, 3.0, 0.0, v=2.0, omega=0.0, predict_time=2.0, dt=0.1)

    collides, _ = planner.check_trajectory_collision(
        trajectory=traj,
        grid_map=None,
        dynamic_obstacles=[obs],
        robot_radius=0.4,
        safety_margin=0.1,
        dt_step=0.1,
        robot_start_pos=(0.0, 3.0),
        obs_initial_pos=[(obs.x, obs.y, obs.radius)],
    )
    assert collides is True


def test_stopping_when_all_candidates_unsafe() -> None:
    """Verify DWA returns emergency stop (v=0, omega=0, is_safe=False) when completely trapped."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    # Trap robot at (5.0, 5.0) with four close surrounding static walls
    grid_map = GridMap(width=20, height=20, resolution=0.5)
    grid_map.add_rectangular_obstacle(4.0, 3.5, 6.0, 4.2)
    grid_map.add_rectangular_obstacle(4.0, 5.8, 6.0, 6.5)
    grid_map.add_rectangular_obstacle(3.5, 4.0, 4.2, 6.0)
    grid_map.add_rectangular_obstacle(5.8, 4.0, 6.5, 6.0)

    sol = planner.compute_command(
        state=RobotState(x=5.0, y=5.0, theta=0.0, v=0.5, omega=0.0),
        target_point=(10.0, 5.0),
        grid_map=grid_map,
        dynamic_obstacles=[],
        robot_radius=0.4,
        dt=0.1,
    )
    assert sol.is_safe is False
    assert sol.v_cmd == 0.0
    assert sol.omega_cmd == 0.0


def test_dwa_velocity_and_acceleration_limits() -> None:
    """Verify commanded (v, omega) strictly respect acceleration and velocity limits."""
    robot_cfg = RobotConfig(
        max_velocity=1.5,
        min_velocity=0.0,
        max_angular_velocity=0.8,
        max_acceleration=1.0,
        max_angular_acceleration=1.5,
    )
    planner = DWAPlanner(DWAConfig(), robot_cfg)
    grid_map = GridMap(width=20, height=20, resolution=0.5)

    current_v = 0.5
    current_omega = 0.1
    state = RobotState(x=2.0, y=2.0, theta=0.0, v=current_v, omega=current_omega)
    dt = 0.1

    sol = planner.compute_command(state, (10.0, 2.0), grid_map=grid_map, dt=dt)

    # Velocity limits
    assert 0.0 <= sol.v_cmd <= robot_cfg.max_velocity
    assert -robot_cfg.max_angular_velocity <= sol.omega_cmd <= robot_cfg.max_angular_velocity

    # Acceleration limits
    max_dv = robot_cfg.max_acceleration * dt + 1e-6
    max_domega = robot_cfg.max_angular_acceleration * dt + 1e-6
    assert abs(sol.v_cmd - current_v) <= max_dv
    assert abs(sol.omega_cmd - current_omega) <= max_domega


def test_dwa_determinism_identical_inputs() -> None:
    """Verify bit-exact determinism across repeated compute_command invocations."""
    planner = DWAPlanner(DWAConfig(), RobotConfig())
    grid_map = GridMap(width=30, height=30, resolution=0.5)
    grid_map.add_circular_obstacle(8.0, 8.0, 1.0)
    obs = DynamicObstacle(1, (10.0, 5.0), end_position=(10.0, 15.0), radius=0.5, speed=1.0)
    state = RobotState(x=4.0, y=8.0, theta=0.2, v=1.0, omega=-0.1)

    sol_a = planner.compute_command(state, (16.0, 8.0), grid_map, [obs])
    sol_b = planner.compute_command(state, (16.0, 8.0), grid_map, [obs])

    assert sol_a.v_cmd == sol_b.v_cmd
    assert sol_a.omega_cmd == sol_b.omega_cmd
    assert sol_a.score == sol_b.score
    assert sol_a.is_safe == sol_b.is_safe
    assert sol_a.best_trajectory == sol_b.best_trajectory


def test_obstacle_and_engine_telemetry_reset() -> None:
    """Verify engine reset restores initial obstacle positions, clocks, and telemetry counters."""
    obs = DynamicObstacle(1, (5.0, 5.0), end_position=(10.0, 5.0), radius=0.5, speed=1.0)
    grid_map = GridMap(width=50, height=50, resolution=0.5)

    cfg = SimulationConfig(nav_mode="dwa", max_steps=50)
    engine = SimulationEngine(cfg)
    ok = engine.reset(grid_map, (1.0, 1.0, 0.0), (15.0, 15.0), dynamic_obstacles=[obs])
    assert ok is True

    # Run for 20 steps
    for _ in range(20):
        engine.step()

    assert engine.steps_taken == 20
    assert obs.x != 5.0

    # Reset
    engine.reset(grid_map, (1.0, 1.0, 0.0), (15.0, 15.0), dynamic_obstacles=[obs])

    assert engine.steps_taken == 0
    assert engine.simulated_time == 0.0
    assert engine.dwa_failures == 0
    assert engine.collision_occurred is False
    assert engine.collided_with_dynamic is False
    assert pytest.approx(obs.x, abs=1e-5) == 5.0
    assert pytest.approx(obs.y, abs=1e-5) == 5.0
