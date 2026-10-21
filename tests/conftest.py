"""Pytest configuration and shared test fixtures."""

import sys
from pathlib import Path
import pytest

# Ensure src/ directory is accessible to all tests
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotModel, RobotState


@pytest.fixture
def default_config() -> SimulationConfig:
    """Fixture providing a fresh default SimulationConfig."""
    return SimulationConfig()


@pytest.fixture
def empty_grid_map() -> GridMap:
    """Fixture providing a 20x20 cell empty grid map with 0.5m resolution."""
    return GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))


@pytest.fixture
def obstacle_grid_map() -> GridMap:
    """Fixture providing a grid map with an obstacle block in the center."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))
    # Add rectangular obstacle from cell (8, 5) to (12, 14)
    grid.add_rectangular_obstacle(x_min=4.0, y_min=2.5, x_max=6.0, y_max=7.0)
    return grid


@pytest.fixture
def robot_model() -> RobotModel:
    """Fixture providing an initialized robot model at the origin."""
    model = RobotModel()
    model.reset(x=0.0, y=0.0, theta=0.0)
    return model

