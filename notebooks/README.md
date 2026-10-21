# Notebooks Directory (`notebooks/`)

This directory is reserved for Jupyter Notebooks (`.ipynb`) used for research exploration, algorithmic prototyping, and visual analysis.

## Typical Workflows

- **`01_grid_and_astar_exploration.ipynb`**: Interactive prototyping of A* heuristic tuning and grid discretization.
- **`02_controller_tuning.ipynb`**: Pure Pursuit lookahead distance and PID gain parameter sweeps.
- **`03_benchmark_analysis.ipynb`**: Visualizing performance metrics, trajectory comparisons, and cross-track errors across test scenarios.

## Guidelines

- Notebooks should import reusable functionality directly from `autonomous_nav` package modules rather than reimplementing algorithms inline.
- Clear cell outputs or keep committed notebook outputs minimal to avoid repository bloat.

