# Outputs Directory (`outputs/`)

This directory contains artifacts generated during simulation runs, test executions, and benchmark experiments.

## Subdirectories

- **`figures/`**: Generated Matplotlib plots, trajectory visualizations, and occupancy grid visual exports.
- **`logs/`**: Raw execution logs, time-series telemetry dumps, and simulation diagnostic text files.
- **`metrics/`**: Quantitative benchmark evaluation reports, JSON/CSV summaries containing path lengths, clearance margins, tracking errors, and execution times.

## Git Tracking Policy

By default, `.gitignore` ignores all generated files within `outputs/figures/`, `outputs/logs/`, and `outputs/metrics/` to prevent committing ephemeral run data, while preserving the directory structure.

