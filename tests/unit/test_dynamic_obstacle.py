"""Unit tests for dynamic obstacle kinematics, trajectory prediction, and manager."""

import math
import pytest
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle, DynamicObstacleManager


def test_dynamic_obstacle_initialization() -> None:
    """Verify obstacle initialization attributes and initial velocity vector."""
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(5.0, 10.0),
        end_position=(15.0, 10.0),
        radius=0.5,
        speed=1.0,
        pattern="linear_ping_pong",
    )
    assert obs.obstacle_id == 1
    assert obs.x == 5.0
    assert obs.y == 10.0
    assert obs.radius == 0.5
    assert obs.speed == 1.0
    assert obs.vx == 1.0
    assert obs.vy == 0.0


def test_linear_ping_pong_kinematics_and_reversal() -> None:
    """Verify that linear_ping_pong moves forward and reverses at endpoint."""
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(0.0, 0.0),
        end_position=(2.0, 0.0),
        radius=0.5,
        speed=1.0,
        pattern="linear_ping_pong",
    )
    # Step 1.0s -> position should be (1.0, 0.0)
    obs.step(dt=1.0)
    assert pytest.approx(obs.x, abs=1e-3) == 1.0
    assert pytest.approx(obs.y, abs=1e-3) == 0.0

    # Step another 1.0s -> position reaches end (2.0, 0.0)
    obs.step(dt=1.0)
    assert pytest.approx(obs.x, abs=1e-3) == 2.0

    # Step another 0.5s -> should reverse direction and be at (1.5, 0.0)
    obs.step(dt=0.5)
    assert pytest.approx(obs.x, abs=1e-3) == 1.5
    assert obs.vx < 0.0  # Heading backwards toward start


def test_analytical_future_prediction_determinism() -> None:
    """Verify analytical prediction matches actual simulation stepping."""
    obs = DynamicObstacle(
        obstacle_id=2,
        initial_position=(10.0, 5.0),
        end_position=(10.0, 15.0),
        radius=0.4,
        speed=2.0,
        pattern="linear_ping_pong",
    )
    # Predict position at dt=1.5s
    pred_x, pred_y = obs.predict_position(dt_ahead=1.5)

    # Step by 1.5s in 15 increments of 0.1s
    for _ in range(15):
        obs.step(dt=0.1)

    assert pytest.approx(pred_x, abs=0.05) == obs.x
    assert pytest.approx(pred_y, abs=0.05) == obs.y


def test_predict_trajectory_horizon() -> None:
    """Verify predict_trajectory returns correct number of horizon points."""
    obs = DynamicObstacle(
        obstacle_id=3,
        initial_position=(0.0, 0.0),
        end_position=(10.0, 0.0),
        radius=0.5,
        speed=1.0,
    )
    traj = obs.predict_trajectory(horizon=2.0, dt=0.5)
    assert len(traj) == 4  # 0.5, 1.0, 1.5, 2.0
    assert pytest.approx(traj[0][0], abs=1e-3) == 0.5
    assert pytest.approx(traj[3][0], abs=1e-3) == 2.0


def test_circular_motion_pattern() -> None:
    """Verify circular pattern orbits around center."""
    obs = DynamicObstacle(
        obstacle_id=4,
        initial_position=(5.0, 0.0),
        radius=0.5,
        speed=1.0,
        pattern="circular",
        center=(0.0, 0.0),
        orbit_radius=5.0,
        angular_rate=0.2,
    )
    assert pytest.approx(math.hypot(obs.x, obs.y), abs=0.1) == 5.0
    for _ in range(20):
        obs.step(dt=0.1)
        assert pytest.approx(math.hypot(obs.x, obs.y), abs=0.1) == 5.0


def test_reset_restores_initial_state() -> None:
    """Verify that calling reset() restores initial state identically."""
    obs = DynamicObstacle(
        obstacle_id=5,
        initial_position=(3.0, 4.0),
        end_position=(10.0, 4.0),
        radius=0.5,
        speed=1.5,
        pattern="linear_ping_pong",
    )
    # Advance state
    for _ in range(50):
        obs.step(dt=0.1)
    assert obs.x != 3.0

    # Reset
    obs.reset()
    assert obs.x == 3.0
    assert obs.y == 4.0
    assert obs.speed == 1.5
    assert obs.vx > 0.0


def test_dynamic_obstacle_manager() -> None:
    """Verify DynamicObstacleManager collision checks and distance queries."""
    obs1 = DynamicObstacle(
        obstacle_id=1,
        initial_position=(5.0, 5.0),
        radius=0.5,
        speed=0.0,
    )
    obs2 = DynamicObstacle(
        obstacle_id=2,
        initial_position=(10.0, 10.0),
        radius=0.5,
        speed=0.0,
    )
    manager = DynamicObstacleManager([obs1, obs2])

    # Collision test: robot disc of radius 0.4 at (5.5, 5.0) -> distance is 0.5 <= 0.4 + 0.5
    is_coll, hit_obs = manager.check_collision(5.5, 5.0, robot_radius=0.4)
    assert is_coll is True
    assert hit_obs is not None and hit_obs.obstacle_id == 1

    # No collision test: robot disc at (0.0, 0.0)
    is_coll_none, _ = manager.check_collision(0.0, 0.0, robot_radius=0.4)
    assert is_coll_none is False

    # Clearance query
    dist = manager.min_distance(5.0, 0.0)
    # dist to obs1 center is 5.0, surface dist = 5.0 - 0.5 = 4.5
    assert pytest.approx(dist, abs=1e-2) == 4.5

    # Prediction test
    preds = manager.predict_all(dt_ahead=1.0)
    assert len(preds) == 2
