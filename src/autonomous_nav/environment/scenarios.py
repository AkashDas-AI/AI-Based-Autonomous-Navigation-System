"""Predefined benchmark scenarios for autonomous navigation testing and evaluation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.grid_map import GridMap


@dataclass
class Scenario:
    """Encapsulates a benchmark scenario configuration."""
    name: str
    description: str
    grid_map: GridMap
    start_world: Tuple[float, float, float]  # (x, y, theta_rad)
    goal_world: Tuple[float, float]  # (x, y)
    dynamic_obstacles: List[DynamicObstacle] = field(default_factory=list)


def create_empty_scenario(
    width: int = 50,
    height: int = 40,
    resolution: float = 0.5,
) -> Scenario:
    """An obstacle-free open environment for baseline kinematic verification."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)
    return Scenario(
        name="empty_field",
        description="Open unobstructed field for testing straight-line tracking and velocity profiles",
        grid_map=grid_map,
        start_world=(2.0, 2.0, 0.0),
        goal_world=(22.0, 17.0),
    )


def create_simple_obstacle_scenario(
    width: int = 60,
    height: int = 40,
    resolution: float = 0.5,
) -> Scenario:
    """A scenario with isolated rectangular and circular obstacles requiring avoidance."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # Add rectangular obstacle blocks
    grid_map.add_rectangular_obstacle(x_min=10.0, y_min=6.0, x_max=14.0, y_max=14.0)
    grid_map.add_rectangular_obstacle(x_min=18.0, y_min=2.0, x_max=22.0, y_max=10.0)

    # Add circular obstacle
    grid_map.add_circular_obstacle(center_x=16.0, center_y=15.0, radius=2.0)

    return Scenario(
        name="simple_obstacles",
        description="Environment with discrete geometric obstacles requiring lateral avoidance",
        grid_map=grid_map,
        start_world=(3.0, 3.0, 0.0),
        goal_world=(26.0, 16.0),
    )


def create_corridor_scenario(
    width: int = 70,
    height: int = 30,
    resolution: float = 0.5,
) -> Scenario:
    """A narrow corridor with sharp corners forcing constrained heading changes."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # Upper and lower boundary walls creating a corridor
    grid_map.add_rectangular_obstacle(x_min=0.0, y_min=0.0, x_max=35.0, y_max=3.0)
    grid_map.add_rectangular_obstacle(x_min=0.0, y_min=12.0, x_max=35.0, y_max=15.0)

    # Baffle wall creating an S-bend
    grid_map.add_rectangular_obstacle(x_min=14.0, y_min=3.0, x_max=16.0, y_max=9.0)
    grid_map.add_rectangular_obstacle(x_min=22.0, y_min=6.0, x_max=24.0, y_max=12.0)

    return Scenario(
        name="corridor_s_bend",
        description="Constrained corridor requiring multi-waypoint navigation through narrow passage",
        grid_map=grid_map,
        start_world=(4.0, 7.5, 0.0),
        goal_world=(30.0, 7.5),
    )


def create_urban_grid_scenario(
    width: int = 80,
    height: int = 60,
    resolution: float = 0.5,
) -> Scenario:
    """An urban road-like grid with city blocks and cross intersections."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # 4 distinct city blocks with traversable streets between them
    # Block 1 (SW)
    grid_map.add_rectangular_obstacle(x_min=5.0, y_min=5.0, x_max=16.0, y_max=12.0)
    # Block 2 (SE)
    grid_map.add_rectangular_obstacle(x_min=22.0, y_min=5.0, x_max=35.0, y_max=12.0)
    # Block 3 (NW)
    grid_map.add_rectangular_obstacle(x_min=5.0, y_min=17.0, x_max=16.0, y_max=25.0)
    # Block 4 (NE)
    grid_map.add_rectangular_obstacle(x_min=22.0, y_min=17.0, x_max=35.0, y_max=25.0)

    return Scenario(
        name="urban_grid",
        description="Structured urban layout with intersecting roadways and rectangular city blocks",
        grid_map=grid_map,
        start_world=(2.0, 14.5, 0.0),
        goal_world=(37.0, 14.5),
    )


def create_dynamic_crossing_scenario(
    width: int = 60,
    height: int = 40,
    resolution: float = 0.5,
) -> Scenario:
    """An environment with moving obstacles crossing the robot's nominal trajectory."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # Some static guide barriers
    grid_map.add_rectangular_obstacle(x_min=8.0, y_min=0.0, x_max=10.0, y_max=6.0)
    grid_map.add_rectangular_obstacle(x_min=8.0, y_min=14.0, x_max=10.0, y_max=20.0)

    # Moving obstacles crossing vertically across the robot path (y=10)
    obs1 = DynamicObstacle(
        obstacle_id=1,
        initial_position=(14.0, 3.0),
        end_position=(14.0, 17.0),
        radius=0.6,
        speed=0.8,
        pattern="linear_ping_pong",
    )
    obs2 = DynamicObstacle(
        obstacle_id=2,
        initial_position=(22.0, 17.0),
        end_position=(22.0, 3.0),
        radius=0.5,
        speed=0.7,
        pattern="linear_ping_pong",
    )

    return Scenario(
        name="dynamic_crossing",
        description="Open corridor with two independent moving obstacles crossing the path perpendicularly",
        grid_map=grid_map,
        start_world=(2.0, 10.0, 0.0),
        goal_world=(27.0, 10.0),
        dynamic_obstacles=[obs1, obs2],
    )


def create_dynamic_corridor_scenario(
    width: int = 70,
    height: int = 30,
    resolution: float = 0.5,
) -> Scenario:
    """A corridor scenario with an obstacle crossing diagonally between corridor edges requiring local dynamic avoidance."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # Upper and lower corridor walls defining an 8m wide corridor (y in [4.0, 12.0])
    grid_map.add_rectangular_obstacle(x_min=0.0, y_min=0.0, x_max=35.0, y_max=4.0)
    grid_map.add_rectangular_obstacle(x_min=0.0, y_min=12.0, x_max=35.0, y_max=15.0)

    # Dynamic obstacle traversing diagonally across the corridor between y=5.0 and y=11.0
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(14.0, 5.0),
        end_position=(20.0, 11.0),
        radius=0.6,
        speed=0.4,
        pattern="linear_ping_pong",
    )

    return Scenario(
        name="dynamic_corridor",
        description="Corridor with a dynamic obstacle crossing diagonally requiring local trajectory avoidance",
        grid_map=grid_map,
        start_world=(3.0, 7.0, 0.0),
        goal_world=(32.0, 7.0),
        dynamic_obstacles=[obs],
    )


def create_dynamic_chokepoint_scenario(
    width: int = 60,
    height: int = 40,
    resolution: float = 0.5,
) -> Scenario:
    """A scenario where a dynamic obstacle periodically blocks and opens a central doorway."""
    grid_map = GridMap(width=width, height=height, resolution=resolution)

    # Dividing barrier at x in [14.0, 16.0] with a central doorway at y in [7.5, 12.5]
    grid_map.add_rectangular_obstacle(x_min=14.0, y_min=0.0, x_max=16.0, y_max=7.5)
    grid_map.add_rectangular_obstacle(x_min=14.0, y_min=12.5, x_max=16.0, y_max=20.0)

    # Dynamic obstacle patrolling back and forth across the doorway (y in [5.5, 14.5])
    obs = DynamicObstacle(
        obstacle_id=1,
        initial_position=(15.0, 6.0),
        end_position=(15.0, 14.0),
        radius=0.65,
        speed=0.6,
        pattern="linear_ping_pong",
    )

    return Scenario(
        name="dynamic_chokepoint",
        description="Divided rooms with a central doorway periodically obstructed by a patrolling obstacle",
        grid_map=grid_map,
        start_world=(4.0, 10.0, 0.0),
        goal_world=(26.0, 10.0),
        dynamic_obstacles=[obs],
    )


def get_all_scenarios() -> Dict[str, Scenario]:
    """Retrieve all standard and dynamic benchmark scenarios indexed by identifier."""
    return {
        "empty_field": create_empty_scenario(),
        "simple_obstacles": create_simple_obstacle_scenario(),
        "corridor_s_bend": create_corridor_scenario(),
        "urban_grid": create_urban_grid_scenario(),
        "dynamic_crossing": create_dynamic_crossing_scenario(),
        "dynamic_corridor": create_dynamic_corridor_scenario(),
        "dynamic_chokepoint": create_dynamic_chokepoint_scenario(),
    }


def get_dynamic_scenarios() -> Dict[str, Scenario]:
    """Retrieve only scenarios that contain moving obstacles."""
    scenarios = get_all_scenarios()
    return {k: v for k, v in scenarios.items() if len(v.dynamic_obstacles) > 0}


def get_scenario_by_name(name: str) -> Scenario:
    """Fetch scenario by name, raising KeyError if not found."""
    scenarios = get_all_scenarios()
    if name not in scenarios:
        raise KeyError(
            f"Scenario '{name}' not found. Available scenarios: {list(scenarios.keys())}"
        )
    return scenarios[name]

