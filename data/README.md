# Data Directory (`data/`)

This directory houses static grid maps, scenario definitions, benchmark environments, and offline test data.

## Subdirectories

- **`maps/`**: Static 2D occupancy grid representations stored in structured formats (e.g., CSV, JSON, PNG binary masks, or NumPy `.npy` arrays). Contains predefined floor plans, urban grid layouts, and obstacle mazes.
- **`samples/`**: Sample sensor recordings, recorded trajectories, and telemetry captures used for offline replay and unit testing.

## Data Retention & Hygiene

- All temporary scratch files (`*.tmp`) are excluded by `.gitignore`.
- Raw or heavy experimental logs should be directed to `outputs/logs/` rather than stored here.

