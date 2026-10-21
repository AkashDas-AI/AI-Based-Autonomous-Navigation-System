"""Unit tests for GridMap representation and operations."""

import numpy as np
import pytest
from autonomous_nav.environment.grid_map import GridMap


def test_grid_map_initialization():
    """Verify grid map creates proper dimensions and defaults to free space."""
    grid = GridMap(width=30, height=20, resolution=0.5, origin=(2.0, 3.0))
    assert grid.width == 30
    assert grid.height == 20
    assert grid.resolution == 0.5
    assert grid.origin == (2.0, 3.0)
    assert grid.grid.shape == (20, 30)
    assert np.all(grid.grid == 0)


def test_invalid_dimensions():
    """Verify ValueError is raised on non-positive dimensions or resolution."""
    with pytest.raises(ValueError):
        GridMap(width=-5, height=10)
    with pytest.raises(ValueError):
        GridMap(width=10, height=0)
    with pytest.raises(ValueError):
        GridMap(width=10, height=10, resolution=-0.1)


def test_coordinate_transforms():
    """Verify metric world coordinates map accurately to grid cells and centers."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))

    # Cell 0, 0 should cover [0, 0.5) x [0, 0.5)
    gx, gy = grid.world_to_grid(0.2, 0.2)
    assert gx == 0 and gy == 0

    # Cell (2, 4) world center should be (2 * 0.5 + 0.25, 4 * 0.5 + 0.25) = (1.25, 2.25)
    wx, wy = grid.grid_to_world(2, 4)
    assert pytest.approx(wx) == 1.25
    assert pytest.approx(wy) == 2.25

    # Reverse mapping
    rgx, rgy = grid.world_to_grid(wx, wy)
    assert (rgx, rgy) == (2, 4)


def test_obstacle_setting_and_queries(empty_grid_map):
    """Verify setting obstacles and querying cell states."""
    assert not empty_grid_map.is_occupied(5, 5)
    empty_grid_map.set_obstacle(5, 5, True)
    assert empty_grid_map.is_occupied(5, 5)

    # Clearing obstacle
    empty_grid_map.set_obstacle(5, 5, False)
    assert not empty_grid_map.is_occupied(5, 5)

    # Out-of-bounds queries should report occupied
    assert empty_grid_map.is_occupied(-1, 0)
    assert empty_grid_map.is_occupied(25, 5)


def test_rectangular_and_circular_obstacles(empty_grid_map):
    """Verify adding shapes marks the appropriate grid cells."""
    empty_grid_map.add_rectangular_obstacle(x_min=1.0, y_min=1.0, x_max=2.0, y_max=2.0)
    # Check cells covered by [1.0, 2.0] at resolution 0.5 -> cells 2, 3, 4
    assert empty_grid_map.is_occupied(2, 2)
    assert empty_grid_map.is_occupied(4, 4)

    empty_grid_map.add_circular_obstacle(center_x=5.0, center_y=5.0, radius=0.8)
    gx, gy = empty_grid_map.world_to_grid(5.0, 5.0)
    assert empty_grid_map.is_occupied(gx, gy)


def test_obstacle_inflation():
    """Verify obstacle inflation expands occupied region by safety margin."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))
    grid.set_obstacle(10, 10, True)

    inflated = grid.inflate_obstacles(radius_meters=0.5)

    # Original cell must remain occupied
    assert inflated.is_occupied(10, 10)
    # Direct neighbors (0.5m away) should now be occupied
    assert inflated.is_occupied(11, 10)
    assert inflated.is_occupied(9, 10)
    assert inflated.is_occupied(10, 11)
    assert inflated.is_occupied(10, 9)
    # Distant cells should remain free
    assert not inflated.is_occupied(5, 5)

