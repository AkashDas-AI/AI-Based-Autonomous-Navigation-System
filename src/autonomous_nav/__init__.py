"""AI-Based Autonomous Navigation System.

A simulation-first autonomous navigation framework featuring 2D occupancy grid
representation, A* path planning, kinematic robot motion simulation,
waypoint tracking, collision detection, and performance evaluation.
"""

__version__ = "0.1.0"
__author__ = "Autonomous Systems Team"

from autonomous_nav.config import (
    SimulationConfig,
    MapConfig,
    RobotConfig,
    PlannerConfig,
    ControllerConfig,
    load_config,
)
from autonomous_nav.datasets import TUMRGBDAdapter, DatasetConfig, GroundTruthTrajectory

__all__ = [
    "__version__",
    "SimulationConfig",
    "MapConfig",
    "RobotConfig",
    "PlannerConfig",
    "ControllerConfig",
    "load_config",
    "TUMRGBDAdapter",
    "DatasetConfig",
    "GroundTruthTrajectory",
]

