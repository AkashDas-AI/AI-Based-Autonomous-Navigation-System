"""Unit tests for CollisionChecker module."""

import pytest
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.collision import CollisionChecker


def test_point_collision(obstacle_grid_map):
    """Verify point inside obstacle triggers collision, free space does not."""
    # The fixture has an obstacle from x=[4.0, 6.0], y=[2.5, 7.0]
    assert CollisionChecker.check_point_collision(5.0, 5.0, obstacle_grid_map)
    assert not CollisionChecker.check_point_collision(1.0, 1.0, obstacle_grid_map)


def test_disc_collision(obstacle_grid_map):
    """Verify disc footprint detects collision even when center is slightly outside obstacle."""
    # Center at (3.8, 5.0), obstacle boundary begins at x=4.0 (distance 0.2m)
    # A robot with radius 0.3m must collide
    assert CollisionChecker.check_disc_collision(3.8, 5.0, radius=0.3, grid_map=obstacle_grid_map)

    # A robot with small radius 0.1m does not penetrate
    assert not CollisionChecker.check_disc_collision(3.8, 5.0, radius=0.1, grid_map=obstacle_grid_map)


def test_path_collision(obstacle_grid_map):
    """Verify path crossing obstacle triggers collision detection."""
    # Path passing directly through the obstacle at x=5.0
    colliding_path = [(2.0, 5.0), (4.5, 5.0), (8.0, 5.0)]
    assert CollisionChecker.check_path_collision(colliding_path, radius=0.2, grid_map=obstacle_grid_map)

    # Path cleanly navigating around obstacle
    clear_path = [(2.0, 1.0), (5.0, 1.0), (8.0, 1.0)]
    assert not CollisionChecker.check_path_collision(clear_path, radius=0.2, grid_map=obstacle_grid_map)


def test_distance_to_nearest_obstacle(obstacle_grid_map):
    """Verify distance calculation to closest occupied cell."""
    # Obstacle is at x in [4.0, 6.0], y in [2.5, 7.0]
    # Point at (1.0, 5.0) has obstacle ~3.0m away
    dist = CollisionChecker.distance_to_nearest_obstacle(1.0, 5.0, obstacle_grid_map, search_radius=5.0)
    assert pytest.approx(dist, abs=0.6) == 3.0

