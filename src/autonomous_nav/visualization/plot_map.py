"""Matplotlib-based 2D grid map and trajectory visualization."""

import os
from pathlib import Path
from typing import List, Optional, Tuple

# Route Matplotlib configuration and font cache into project-local .matplotlib
project_root = Path(__file__).resolve().parent.parent.parent.parent
os.environ.setdefault("MPLCONFIGDIR", str(project_root / ".matplotlib"))

import matplotlib.pyplot as plt
import numpy as np
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotState


def plot_simulation_run(
    grid_map: GridMap,
    planned_path: Optional[List[Tuple[float, float]]] = None,
    trajectory: Optional[List[RobotState]] = None,
    start_point: Optional[Tuple[float, float]] = None,
    goal_point: Optional[Tuple[float, float]] = None,
    title: str = "Autonomous Navigation Simulation",
    save_path: Optional[str | Path] = None,
    show: bool = False,
) -> plt.Figure:
    """Render the occupancy grid, planned path, and executed trajectory.

    Args:
        grid_map: GridMap instance to visualize.
        planned_path: List of (x, y) world coordinates representing planned route.
        trajectory: List of RobotState instances representing actual vehicle path.
        start_point: (x, y) coordinates of initial location.
        goal_point: (x, y) coordinates of target location.
        title: Plot title.
        save_path: Optional path to save figure image.
        show: If True, calls plt.show().

    Returns:
        Matplotlib Figure object.
    """
    fig, ax = plt.subplots(figsize=(10, 7), dpi=100)

    # Convert binary grid to display matrix
    # Note: grid[gy, gx] maps to extent [x_min, x_max, y_min, y_max]
    x_min = grid_map.origin[0]
    x_max = grid_map.origin[0] + grid_map.width * grid_map.resolution
    y_min = grid_map.origin[1]
    y_max = grid_map.origin[1] + grid_map.height * grid_map.resolution

    ax.imshow(
        grid_map.grid,
        cmap="binary",
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        interpolation="nearest",
        alpha=0.8,
    )

    # Plot planned path
    if planned_path and len(planned_path) > 1:
        px = [p[0] for p in planned_path]
        py = [p[1] for p in planned_path]
        ax.plot(px, py, "b--", linewidth=2.0, label="Planned A* Path", alpha=0.85)

    # Plot executed trajectory
    if trajectory and len(trajectory) > 1:
        tx = [s.x for s in trajectory]
        ty = [s.y for s in trajectory]
        ax.plot(tx, ty, "g-", linewidth=2.5, label="Driven Trajectory")

        # Draw robot heading arrows along sample steps
        step_interval = max(1, len(trajectory) // 15)
        for s in trajectory[::step_interval]:
            dx = 0.6 * np.cos(s.theta)
            dy = 0.6 * np.sin(s.theta)
            ax.arrow(
                s.x, s.y, dx, dy,
                head_width=0.3, head_length=0.2,
                fc="orange", ec="darkorange", alpha=0.7
            )

    # Plot Start
    if start_point:
        ax.plot(start_point[0], start_point[1], "go", markersize=10, label="Start", markeredgecolor="black")

    # Plot Goal
    if goal_point:
        ax.plot(goal_point[0], goal_point[1], "r*", markersize=14, label="Goal", markeredgecolor="black")

    ax.set_title(title, fontsize=13, fontweight="bold", pad=12)
    ax.set_xlabel("X [meters]", fontsize=11)
    ax.set_ylabel("Y [meters]", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.legend(loc="upper right", framealpha=0.9)
    plt.tight_layout()

    if save_path:
        out_file = Path(save_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_file, dpi=150)

    if show:
        plt.show()

    return fig

