"""Integration tests for dynamic obstacle navigation: Pure Pursuit baseline vs DWA local planner."""

import pytest
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.scenarios import get_scenario_by_name
from autonomous_nav.simulation.engine import SimulationEngine


def test_dynamic_crossing_pure_pursuit_collides() -> None:
    """Verify that pure pursuit baseline fails on dynamic crossing due to blind trajectory following."""
    scenario = get_scenario_by_name("dynamic_crossing")
    config = SimulationConfig(nav_mode="pure_pursuit", max_steps=500)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    assert result.collision is True
    assert result.collided_with_dynamic is True
    assert result.goal_reached is False


def test_dynamic_crossing_dwa_succeeds() -> None:
    """Verify that DWA local planner safely navigates dynamic crossing and reaches goal."""
    scenario = get_scenario_by_name("dynamic_crossing")
    config = SimulationConfig(nav_mode="dwa", max_steps=500)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    assert result.success is True
    assert result.goal_reached is True
    assert result.collision is False
    assert result.collided_with_dynamic is False
    assert result.dwa_failures == 0


def test_dynamic_chokepoint_pure_pursuit_vs_dwa() -> None:
    """Verify pure pursuit collides at doorway while DWA navigates through safely."""
    scenario = get_scenario_by_name("dynamic_chokepoint")

    # 1. Pure pursuit run -> collision
    cfg_pp = SimulationConfig(nav_mode="pure_pursuit", max_steps=500)
    engine_pp = SimulationEngine(cfg_pp)
    res_pp = engine_pp.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    assert res_pp.collision is True
    assert res_pp.collided_with_dynamic is True

    # 2. DWA run -> goal reached
    cfg_dwa = SimulationConfig(nav_mode="dwa", max_steps=500)
    engine_dwa = SimulationEngine(cfg_dwa)
    res_dwa = engine_dwa.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    assert res_dwa.success is True
    assert res_dwa.goal_reached is True
    assert res_dwa.collision is False


def test_dynamic_corridor_dwa_success() -> None:
    """Verify DWA safely negotiates diagonal crossing dynamic obstacle in corridor."""
    scenario = get_scenario_by_name("dynamic_corridor")
    config = SimulationConfig(nav_mode="dwa", max_steps=500)
    engine = SimulationEngine(config)

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    assert result.success is True
    assert result.goal_reached is True
    assert result.collided_with_dynamic is False


def test_nav_mode_engine_telemetry() -> None:
    """Verify that engine exposes DWA telemetry when running in DWA mode."""
    scenario = get_scenario_by_name("dynamic_crossing")
    config = SimulationConfig(nav_mode="dwa", max_steps=50)
    engine = SimulationEngine(config)

    engine.reset(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    state, term = engine.step()

    assert engine.dwa_safe is True
    assert len(engine.current_candidate_trajectory) > 0
    assert engine.current_cmd[0] >= 0.0
