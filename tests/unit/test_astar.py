"""Unit tests for A* path planning algorithm."""

import pytest
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.planning.astar import AStarPlanner


def test_astar_empty_field(empty_grid_map):
    """Verify A* finds shortest diagonal path across open grid."""
    planner = AStarPlanner(allow_diagonal=True)
    start = (1, 1)
    goal = (10, 10)

    path = planner.plan_grid(start, goal, empty_grid_map)
    assert path is not None
    assert len(path) > 0
    assert path[0] == start
    assert path[-1] == goal
    # With 8-connectivity, diagonal path from (1,1) to (10,10) is 10 steps (length 10)
    assert len(path) == 10


def test_astar_around_obstacle():
    """Verify A* generates detour avoiding obstacle wall."""
    grid = GridMap(width=20, height=20, resolution=0.5)
    # Create wall from y=2 to y=15 at x=10
    for y in range(2, 16):
        grid.set_obstacle(10, y, True)

    planner = AStarPlanner(allow_diagonal=True)
    start = (5, 8)
    goal = (15, 8)

    path = planner.plan_grid(start, goal, grid)
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal

    # Verify no path point intersects wall
    for gx, gy in path:
        assert not grid.is_occupied(gx, gy)


def test_astar_unreachable_goal():
    """Verify A* returns None when goal is completely enclosed by obstacles."""
    grid = GridMap(width=20, height=20, resolution=0.5)
    # Box enclosing (10, 10)
    for x in range(8, 13):
        for y in range(8, 13):
            if x in (8, 12) or y in (8, 12):
                grid.set_obstacle(x, y, True)

    planner = AStarPlanner()
    path = planner.plan_grid((2, 2), (10, 10), grid)
    assert path is None


def test_astar_start_equals_goal(empty_grid_map):
    """Verify A* returns single-element list when start is identical to goal."""
    planner = AStarPlanner()
    path = planner.plan_grid((5, 5), (5, 5), empty_grid_map)
    assert path == [(5, 5)]


def test_astar_world_planning(empty_grid_map):
    """Verify world coordinate planning translates accurately."""
    planner = AStarPlanner()
    start_w = (1.0, 1.0)
    goal_w = (5.0, 5.0)

    world_path = planner.plan_world(start_w, goal_w, empty_grid_map)
    assert world_path is not None
    assert len(world_path) > 1
    # Check that waypoints are in reasonable proximity to requested endpoints
    assert pytest.approx(world_path[0][0], abs=0.5) == start_w[0]
    assert pytest.approx(world_path[-1][0], abs=0.5) == goal_w[0]


def test_astar_strictly_forbids_diagonal_cutting_single_blocked_corner():
    """Verify that a diagonal move cannot cut through a single adjacent blocked corner cell."""
    grid = GridMap(width=10, height=10, resolution=0.5)
    # Start at (1, 1), Goal at (3, 2).
    # Obstacle is placed at cardinal neighbor (2, 1).
    grid.set_obstacle(2, 1, True)

    planner = AStarPlanner(allow_diagonal=True)
    path = planner.plan_grid((1, 1), (3, 2), grid)

    assert path is not None
    # Transition from (1, 1) directly to (2, 2) is forbidden because (2, 1) is blocked.
    assert (1, 1) in path
    if (2, 2) in path:
        # If (2, 2) is visited, it must NOT transition directly from (1, 1)
        idx_22 = path.index((2, 2))
        prev_node = path[idx_22 - 1]
        assert prev_node != (1, 1), "A* illegally cut the corner of blocked cell (2, 1)"

