"""Integration tests for closed-loop autonomous navigation pipeline."""

import pytest
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.scenarios import create_empty_scenario, create_simple_obstacle_scenario
from autonomous_nav.simulation.engine import SimulationEngine


def test_closed_loop_navigation_empty_field():
    """Verify end-to-end autonomous navigation on open field scenario."""
    scenario = create_empty_scenario()
    config = SimulationConfig(max_steps=500, dt=0.1)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    assert result.success, "Simulation should complete successfully on empty field"
    assert not result.collision, "No collision should occur on open field"
    assert result.goal_reached, "Goal must be reached"
    assert result.steps_taken > 0
    assert result.final_distance_to_goal <= config.goal_tolerance


def test_closed_loop_navigation_obstacle_avoidance():
    """Verify autonomous vehicle navigates around obstacles without collision."""
    scenario = create_simple_obstacle_scenario()
    config = SimulationConfig(max_steps=600, dt=0.1)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    assert result.success, "Vehicle should successfully reach goal around obstacles"
    assert not result.collision, "Collision check must pass with zero obstacle intersections"
    assert result.goal_reached
    assert result.termination_reason == "goal_reached"
    assert len(result.trajectory) > 10


def test_navigation_planning_failure():
    """Verify that an impossible/enclosed goal produces planning_failed=True and success=False."""
    from autonomous_nav.environment.grid_map import GridMap
    grid = GridMap(width=20, height=20, resolution=0.5)
    # Completely enclose goal (10, 10) in obstacles
    for x in range(8, 13):
        for y in range(8, 13):
            grid.set_obstacle(x, y, True)

    engine = SimulationEngine()
    result = engine.run(grid_map=grid, start_pose=(2.0, 2.0, 0.0), goal_position=(5.0, 5.0))

    assert not result.success
    assert result.planning_failed
    assert result.termination_reason == "planning_failure"
    assert not result.goal_reached
    assert not result.collision


def test_navigation_timeout_failure():
    """Verify that insufficient max_steps triggers timeout=True and success=False."""
    scenario = create_empty_scenario()
    # Configure tiny step budget
    config = SimulationConfig(max_steps=5, dt=0.1)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    assert not result.success
    assert result.timeout
    assert not result.goal_reached
    assert result.termination_reason == "timeout"
    assert not result.collision


def test_navigation_collision_detection():
    """Verify that a path intersecting an obstacle aborts with collision=True and success=False."""
    scenario = create_empty_scenario()
    # Force an un-cleared path straight through an obstacle
    scenario.grid_map.set_obstacle(5, 4, True)
    straight_path = [(2.0, 2.0), (2.5, 2.0), (3.0, 2.0)]
    scenario.grid_map.set_obstacle_world(2.5, 2.0, True)

    config = SimulationConfig(max_steps=50, dt=0.1)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=(2.0, 2.0, 0.0),
        goal_position=(3.0, 2.0),
        precomputed_path=straight_path,
    )

    assert not result.success
    assert result.collision
    assert result.termination_reason == "collision"
    assert not result.goal_reached

