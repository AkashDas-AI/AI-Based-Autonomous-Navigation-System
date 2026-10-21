"""Scenario validation tests ensuring all benchmark environments are well-formed."""

import pytest
from autonomous_nav.environment.scenarios import get_all_scenarios
from autonomous_nav.planning.astar import AStarPlanner


@pytest.mark.parametrize("scenario_name, scenario", get_all_scenarios().items())
def test_scenario_validity(scenario_name, scenario):
    """Ensure each benchmark scenario has valid dimensions, collision-free endpoints, and a feasible path."""
    grid = scenario.grid_map
    assert grid.width > 0
    assert grid.height > 0
    assert grid.resolution > 0.0

    start_x, start_y = scenario.start_world[0], scenario.start_world[1]
    goal_x, goal_y = scenario.goal_world[0], scenario.goal_world[1]

    # Start position must be inside map bounds and not inside an obstacle
    assert not grid.is_occupied_world(start_x, start_y), f"Start position in '{scenario_name}' is occupied"

    # Goal position must be inside map bounds and not inside an obstacle
    assert not grid.is_occupied_world(goal_x, goal_y), f"Goal position in '{scenario_name}' is occupied"

    # A valid path must exist from start to goal
    planner = AStarPlanner()
    path = planner.plan_world((start_x, start_y), (goal_x, goal_y), grid)
    assert path is not None, f"No feasible path found for scenario '{scenario_name}'"
    assert len(path) >= 2, f"Path for '{scenario_name}' is trivially short"

