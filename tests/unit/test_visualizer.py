"""Unit and integration tests for the interactive Pygame visualizer and step interface."""

import math
import os
from pathlib import Path
import numpy as np
import pytest

# Ensure headless video driver for SDL in test environments
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.environment.scenarios import get_scenario_by_name
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.visualization.animate import InteractiveVisualizer


# ---------------------------------------------------------------------------
# 1. Simulation Engine Step Interface Tests
# ---------------------------------------------------------------------------

def test_engine_step_interface():
    """Verify that SimulationEngine supports incremental stepping and state queries."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("empty_field")

    # 1. Reset initializes episode
    plan_ok = engine.reset(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )
    assert plan_ok is True
    assert engine.steps_taken == 0
    assert engine.simulated_time == 0.0
    assert engine.is_done is False
    assert len(engine.planned_path) > 0
    assert len(engine.trajectory) == 1

    # 2. Step advances by dt
    state, term_reason = engine.step()
    assert engine.steps_taken == 1
    assert pytest.approx(engine.simulated_time, abs=1e-4) == engine.config.dt
    assert len(engine.trajectory) == 2
    assert term_reason is None  # Episode still ongoing

    # 3. get_result returns valid in-progress score
    result = engine.get_result()
    assert result.steps_taken == 1
    assert result.simulated_time == engine.simulated_time
    assert result.success is False
    assert result.termination_reason == "in_progress"


def test_engine_replan_on_obstacle_mutation():
    """Verify that engine.replan() re-routes the global path when map occupancy changes."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("empty_field")
    grid = scenario.grid_map

    engine.reset(grid, scenario.start_world, scenario.goal_world)
    original_path = list(engine.planned_path)
    assert len(original_path) > 0

    # Place a wall directly across the middle of the field: x in [8, 12], y in [0, 15]
    for gx in range(16, 24):
        for gy in range(0, 30):
            grid.set_occupied(gx, gy, True)

    replan_ok = engine.replan()
    assert replan_ok is True
    assert engine.planned_path != original_path
    assert not engine.planning_failed


# ---------------------------------------------------------------------------
# 2. Interactive Visualizer Headless Initialization & Teardown
# ---------------------------------------------------------------------------

def test_visualizer_headless_initialization():
    """Verify visualizer initializes cleanly with headless driver and closes without errors."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        window_width=800,
        window_height=600,
        fps=30,
        headless=True,
    )
    try:
        assert visualizer.window_width == 800
        assert visualizer.window_height == 600
        assert visualizer.screen.get_size() == (800, 600)
        assert visualizer.running is True
        assert visualizer.paused is False
        assert visualizer.scenario_name == "simple_obstacles"
        assert visualizer.engine.grid_map is not None
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 3. Coordinate Transformations
# ---------------------------------------------------------------------------

def test_world_to_screen_and_inverse_conversion():
    """Verify bidirectional world-to-screen coordinate mapping consistency."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        window_width=1000,
        window_height=700,
        headless=True,
    )
    try:
        grid = visualizer.engine.grid_map
        assert grid is not None

        # Test arbitrary interior world point (10.0, 7.5)
        wx_orig, wy_orig = 10.0, 7.5
        u, v = visualizer.world_to_screen(wx_orig, wy_orig)
        assert visualizer.map_rect.left <= u <= visualizer.map_rect.right
        assert visualizer.map_rect.top <= v <= visualizer.map_rect.bottom

        # Roundtrip back to world
        wx_rec, wy_rec = visualizer.screen_to_world(u, v)
        # Quantization to integer pixel coordinate gives sub-centimeter tolerance
        assert pytest.approx(wx_rec, abs=0.08) == wx_orig
        assert pytest.approx(wy_rec, abs=0.08) == wy_orig

        # Test vertical axis inversion: y=15.0 must have smaller v than y=5.0
        _, v_high = visualizer.world_to_screen(10.0, 15.0)
        _, v_low = visualizer.world_to_screen(10.0, 5.0)
        assert v_high < v_low, "Higher world Y must correspond to smaller screen V (higher on screen)"
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 4. Rendering Purity: Rendering Does Not Mutate Simulation State
# ---------------------------------------------------------------------------

def test_rendering_does_not_mutate_state():
    """Verify that render() draws the scene without altering robot pose, path, or steps."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        headless=True,
    )
    try:
        # Step simulation 3 times
        for _ in range(3):
            visualizer.engine.step()

        state_before = (
            visualizer.engine.robot.state.x,
            visualizer.engine.robot.state.y,
            visualizer.engine.robot.state.theta,
            visualizer.engine.steps_taken,
            len(visualizer.engine.trajectory),
            len(visualizer.engine.planned_path),
        )

        # Call render multiple times
        visualizer.render()
        visualizer.render()

        state_after = (
            visualizer.engine.robot.state.x,
            visualizer.engine.robot.state.y,
            visualizer.engine.robot.state.theta,
            visualizer.engine.steps_taken,
            len(visualizer.engine.trajectory),
            len(visualizer.engine.planned_path),
        )

        assert state_before == state_after
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 5. Interactive State Controls: Pause, Step, Reset, Switch
# ---------------------------------------------------------------------------

def test_visualizer_pause_step_and_reset_logic():
    """Verify pause toggle, single-step advancing, and scenario reset."""
    visualizer = InteractiveVisualizer(
        scenario_name="empty_field",
        headless=True,
    )
    try:
        # 1. Normal running updates simulation
        assert not visualizer.paused
        visualizer.update_simulation()
        assert visualizer.engine.steps_taken == 1

        # 2. Paused suppresses automatic step
        visualizer.paused = True
        visualizer.update_simulation()
        assert visualizer.engine.steps_taken == 1

        # 3. Manual step advances one step while paused
        visualizer.engine.step()
        assert visualizer.engine.steps_taken == 2

        # 4. Reset returns to initial step 0 and start pose
        visualizer.load_scenario("empty_field")
        assert visualizer.engine.steps_taken == 0
        assert len(visualizer.engine.trajectory) == 1
        assert visualizer.engine.is_done is False
    finally:
        visualizer.close()


def test_visualizer_scenario_switching():
    """Verify switching scenarios re-initializes map, path, and robot pose."""
    visualizer = InteractiveVisualizer(
        scenario_name="empty_field",
        headless=True,
    )
    try:
        assert visualizer.scenario_name == "empty_field"
        start_empty = (visualizer.engine.start_pose[0], visualizer.engine.start_pose[1])

        # Switch to corridor_s_bend
        visualizer.load_scenario("corridor_s_bend")
        assert visualizer.scenario_name == "corridor_s_bend"
        assert visualizer.engine.grid_map is not None
        assert visualizer.engine.grid_map.grid.sum() > 0  # Has obstacles

        # Next scenario cycling
        visualizer.next_scenario()
        assert visualizer.scenario_name == "urban_grid"
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 6. Status Labels & Error States Display
# ---------------------------------------------------------------------------

def test_visualizer_planning_failure_and_collision_status():
    """Verify visualizer displays proper status labels on planning failure and collision."""
    visualizer = InteractiveVisualizer(
        scenario_name="empty_field",
        headless=True,
    )
    try:
        # Normal running
        assert visualizer._get_status_label() == "RUNNING"

        # Paused
        visualizer.paused = True
        assert visualizer._get_status_label() == "PAUSED"
        visualizer.paused = False

        # Planning failure simulation: goal position directly blocked by an obstacle
        grid = visualizer.engine.grid_map
        assert grid is not None
        goal_gx, goal_gy = grid.world_to_grid(*visualizer.engine.goal_position)
        grid.set_occupied(goal_gx, goal_gy, True)
        replan_ok = visualizer.engine.replan()
        assert replan_ok is False
        assert visualizer.engine.planning_failed is True
        assert visualizer._get_status_label() == "PLANNING_FAILURE"

        # Render should safely handle planning failure without crashing
        visualizer.render()

        # Collision simulation: reset and set obstacle at robot pose
        visualizer.load_scenario("empty_field")
        active_grid = visualizer.engine.grid_map
        assert active_grid is not None
        # Place obstacle in front of robot along its path at (2.5, 2.0)
        active_grid.set_occupied(*active_grid.world_to_grid(2.5, 2.0), True)
        # Step engine until collision occurs
        for _ in range(10):
            visualizer.engine.step()
            if visualizer.engine.collision_occurred:
                break

        assert visualizer.engine.collision_occurred is True
        assert visualizer._get_status_label() == "COLLISION"
        visualizer.render()
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 7. Frame Budget Run & Screenshot Export
# ---------------------------------------------------------------------------

def test_visualizer_run_and_save_screenshot(tmp_path: Path):
    """Verify running for a frame budget and capturing a real image screenshot."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        headless=True,
    )
    try:
        # Run for 10 frames
        visualizer.run(max_frames=10)
        assert visualizer.engine.steps_taken == 10

        # Save screenshot
        shot_path = tmp_path / "screenshot_test.png"
        saved = visualizer.save_screenshot(shot_path)
        assert saved.exists()
        assert saved.stat().st_size > 1000
    finally:
        visualizer.close()


# ---------------------------------------------------------------------------
# 8. Dynamic Replanning, Edge Cases & Robustness (Milestone 3.1 Hardening)
# ---------------------------------------------------------------------------

def test_replan_obstacle_forces_valid_detour():
    """Verify that placing an obstacle across the nominal path forces a valid detour."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("empty_field")
    grid = scenario.grid_map
    engine.reset(grid, scenario.start_world, scenario.goal_world)

    orig_path = list(engine.planned_path)
    assert len(orig_path) > 0

    # Place an obstacle block right in the direct diagonal path: x in [10, 14], y in [8, 12]
    grid.add_rectangular_obstacle(x_min=10.0, y_min=8.0, x_max=14.0, y_max=12.0)

    replan_ok = engine.replan()
    assert replan_ok is True
    assert engine.planning_failed is False
    assert engine.is_done is False
    assert engine.planned_path != orig_path

    # Verify no waypoint in the new plan falls inside the obstacle
    for wx, wy in engine.planned_path:
        assert not (10.0 <= wx <= 14.0 and 8.0 <= wy <= 12.0)


def test_replan_blocking_only_available_route():
    """Verify that completely blocking the only route triggers planning failure cleanly."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("corridor_s_bend")
    grid = scenario.grid_map
    engine.reset(grid, scenario.start_world, scenario.goal_world)

    # Place a solid vertical wall from y=0 to y=15 at x=10 in the corridor
    for gy in range(grid.height):
        grid.set_occupied(20, gy, True)  # x = 10.0m

    replan_ok = engine.replan()
    assert replan_ok is False
    assert engine.planning_failed is True
    assert engine.is_done is True
    assert engine.termination_reason == "planning_failure"
    assert engine.planned_path == []  # Stale route is cleared


def test_replan_obstacle_on_goal_cell():
    """Verify that placing an obstacle directly on the goal cell fails safely and clears path."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("simple_obstacles")
    grid = scenario.grid_map
    engine.reset(grid, scenario.start_world, scenario.goal_world)

    # Place obstacle directly at goal
    gx, gy = grid.world_to_grid(*scenario.goal_world)
    grid.set_occupied(gx, gy, True)

    replan_ok = engine.replan()
    assert replan_ok is False
    assert engine.planning_failed is True
    assert engine.is_done is True
    assert engine.termination_reason == "planning_failure"
    assert engine.planned_path == []


def test_replan_and_collision_at_robot_location():
    """Verify that placing an obstacle at robot's current pose triggers collision and can recover."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("empty_field")
    grid = scenario.grid_map
    engine.reset(grid, scenario.start_world, scenario.goal_world)

    # Place obstacle on the cell containing the robot
    rx, ry = engine.robot.state.x, engine.robot.state.y
    rgx, rgy = grid.world_to_grid(rx, ry)
    grid.set_occupied(rgx, rgy, True)

    replan_ok = engine.replan()
    assert replan_ok is False
    assert engine.collision_occurred is True
    assert engine.is_done is True
    assert engine.termination_reason == "collision"
    assert engine.planned_path == []

    # Toggle obstacle off (recovery)
    grid.set_occupied(rgx, rgy, False)
    replan_recovered = engine.replan()
    assert replan_recovered is True
    assert engine.collision_occurred is False
    assert engine.is_done is False
    assert engine.termination_reason is None
    assert len(engine.planned_path) > 0


def test_replan_recovery_from_planning_failure_on_obstacle_removal():
    """Verify removing a blocking obstacle allows recovering from planning failure."""
    engine = SimulationEngine()
    scenario = get_scenario_by_name("corridor_s_bend")
    grid = scenario.grid_map
    engine.reset(grid, scenario.start_world, scenario.goal_world)

    # Block corridor
    blocked_cells = [(20, gy) for gy in range(grid.height)]
    for gx, gy in blocked_cells:
        grid.set_occupied(gx, gy, True)

    assert engine.replan() is False
    assert engine.planning_failed is True
    assert engine.is_done is True
    assert engine.termination_reason == "planning_failure"

    # Remove the wall
    for gx, gy in blocked_cells:
        # Restore only the corridor interior (y in [3, 12])
        if 6 <= gy <= 24:
            grid.set_occupied(gx, gy, False)

    replan_ok = engine.replan()
    assert replan_ok is True
    assert engine.planning_failed is False
    assert engine.is_done is False
    assert engine.termination_reason is None
    assert len(engine.planned_path) > 0

    # Step engine to ensure it advances normally after recovery
    state, term = engine.step()
    assert engine.steps_taken == 1
    assert term is None


def test_failed_replan_clears_stale_route_and_renders_safely():
    """Verify that visualizer handles failed replan without displaying stale routes or crashing."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        headless=True,
    )
    try:
        assert len(visualizer.engine.planned_path) > 0

        # Block the goal cell
        grid = visualizer.engine.grid_map
        gx, gy = grid.world_to_grid(*visualizer.engine.goal_position)
        grid.set_occupied(gx, gy, True)

        replan_ok = visualizer.engine.replan()
        assert replan_ok is False
        assert visualizer.engine.planned_path == []
        assert visualizer._get_status_label() == "PLANNING_FAILURE"

        # Rendering must not crash with empty path
        visualizer.render()
    finally:
        visualizer.close()


def test_reported_termination_state_agreement():
    """Verify complete agreement between simulation engine termination reasons and GUI badges."""
    visualizer = InteractiveVisualizer(
        scenario_name="empty_field",
        headless=True,
    )
    try:
        # Running
        assert visualizer.engine.termination_reason is None
        assert visualizer._get_status_label() == "RUNNING"

        # Paused
        visualizer.paused = True
        assert visualizer._get_status_label() == "PAUSED"
        visualizer.paused = False

        # Collision
        visualizer.engine.collision_occurred = True
        visualizer.engine.is_done = True
        visualizer.engine.termination_reason = "collision"
        assert visualizer._get_status_label() == "COLLISION"
        assert visualizer.engine.get_result().termination_reason == "collision"

        # Goal Reached
        visualizer.engine.collision_occurred = False
        visualizer.engine.goal_reached = True
        visualizer.engine.termination_reason = "goal_reached"
        assert visualizer._get_status_label() == "GOAL_REACHED"
        assert visualizer.engine.get_result().termination_reason == "goal_reached"

        # Timeout
        visualizer.engine.goal_reached = False
        visualizer.engine.timeout_occurred = True
        visualizer.engine.termination_reason = "timeout"
        assert visualizer._get_status_label() == "TIMEOUT"
        assert visualizer.engine.get_result().termination_reason == "timeout"

        # Planning Failure
        visualizer.engine.timeout_occurred = False
        visualizer.engine.planning_failed = True
        visualizer.engine.termination_reason = "planning_failure"
        assert visualizer._get_status_label() == "PLANNING_FAILURE"
        assert visualizer.engine.get_result().termination_reason == "planning_failure"
    finally:
        visualizer.close()


def test_controls_prevent_unintended_advancement():
    """Verify pause, step, reset, and scenario switching prevent accidental steps."""
    visualizer = InteractiveVisualizer(
        scenario_name="empty_field",
        headless=True,
    )
    try:
        # 1. Paused state: repeated update_simulation calls do not advance
        visualizer.paused = True
        for _ in range(5):
            visualizer.update_simulation()
        assert visualizer.engine.steps_taken == 0
        assert visualizer.engine.simulated_time == 0.0

        # 2. Key step while paused: advances exactly one step
        visualizer.engine.step()
        assert visualizer.engine.steps_taken == 1
        assert pytest.approx(visualizer.engine.simulated_time, abs=1e-4) == visualizer.engine.config.dt

        # 3. Reset: completely returns to step 0
        visualizer.load_scenario("empty_field")
        assert visualizer.engine.steps_taken == 0
        assert visualizer.engine.simulated_time == 0.0
        assert len(visualizer.engine.trajectory) == 1

        # 4. Scenario switching: does not advance steps
        visualizer.load_scenario("corridor_s_bend")
        assert visualizer.engine.steps_taken == 0
        assert visualizer.engine.simulated_time == 0.0
    finally:
        visualizer.close()


def test_rendering_invariance_across_all_metrics(tmp_path: Path):
    """Verify rendering multiple frames and exporting snapshots leaves state exactly unchanged."""
    visualizer = InteractiveVisualizer(
        scenario_name="simple_obstacles",
        headless=True,
    )
    try:
        # Step twice
        visualizer.engine.step()
        visualizer.engine.step()

        state_before = (
            visualizer.engine.robot.state.x,
            visualizer.engine.robot.state.y,
            visualizer.engine.robot.state.theta,
            visualizer.engine.robot.state.v,
            visualizer.engine.simulated_time,
            visualizer.engine.steps_taken,
            list(visualizer.engine.planned_path),
            list(visualizer.engine.trajectory),
            visualizer.engine.termination_reason,
        )

        # Repeated renders and screenshot capture
        for _ in range(5):
            visualizer.render()
        visualizer.save_screenshot(tmp_path / "invariance_test.png")

        state_after = (
            visualizer.engine.robot.state.x,
            visualizer.engine.robot.state.y,
            visualizer.engine.robot.state.theta,
            visualizer.engine.robot.state.v,
            visualizer.engine.simulated_time,
            visualizer.engine.steps_taken,
            list(visualizer.engine.planned_path),
            list(visualizer.engine.trajectory),
            visualizer.engine.termination_reason,
        )

        assert state_before == state_after
    finally:
        visualizer.close()
