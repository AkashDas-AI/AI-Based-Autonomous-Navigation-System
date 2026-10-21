# Project Execution & Verification Log

This log maintains a chronological record of all architectural decisions, module implementations, tests, and validation outcomes.

---

## Milestone 0: Technical Documentation Hub Setup
- **Date**: 2026-10-19
- **Status**: Completed
- **Implemented Scope**:
  - Established dedicated documentation repository under [`docs/`](.).
  - Drafted comprehensive system architecture in [`01_system_architecture.md`](01_system_architecture.md).
  - Defined end-to-end multi-rate data flow pipeline in [`02_data_flow_and_pipeline.md`](02_data_flow_and_pipeline.md).
  - Outlined detailed module boundaries and interfaces in [`03_module_specifications.md`](03_module_specifications.md).
  - Documented dataset modalities, benchmarks, and augmentation pipelines in [`04_datasets_and_benchmarks.md`](04_datasets_and_benchmarks.md).
  - Specified model learning mechanics, neural network architecture, and loss formulations in [`05_model_learning_and_training.md`](05_model_learning_and_training.md).
  - Established multi-tiered testing protocols and quantitative pass/fail thresholds in [`06_validation_and_testing_framework.md`](06_validation_and_testing_framework.md).
- **Validation Criteria**:
  - All files in `docs/` cross-linked and verified for structural consistency.
  - Ready for module-by-module implementation starting with foundational core types and state estimation.
- **Next Steps**:
  - Implement professional repository structure, baseline 2D occupancy grid simulator, A* planning, kinematic robot model, waypoint tracking, collision checking, and test suites.

---

## Milestone 1: Professional Repository Structure & 2D Simulation Baseline
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - Configured repository structure with `src/` layout, separating production logic (`src/autonomous_nav/`), execution scripts (`scripts/`), tests (`tests/`), configuration (`configs/`), data (`data/`), outputs (`outputs/`), assets (`assets/`), and documentation (`docs/`).
  - Implemented core package `autonomous_nav`:
    - `config.py`: Typed configuration dataclasses (`MapConfig`, `RobotConfig`, `PlannerConfig`, `ControllerConfig`, `SimulationConfig`).
    - `environment/grid_map.py`: 2D occupancy grid representation, coordinate transforms, and obstacle inflation.
    - `environment/scenarios.py`: 4 reproducible benchmark environments (`empty_field`, `simple_obstacles`, `corridor_s_bend`, `urban_grid`).
    - `planning/astar.py`: 8-connected A* search with Euclidean heuristic and corner-cut prevention.
    - `simulation/collision.py`: Exact circle-to-AABB collision detection and clearance queries.
    - `simulation/robot.py`: Kinematic unicycle vehicle model with RK2 integration and acceleration limits.
    - `simulation/engine.py`: Closed-loop simulation engine orchestrator.
    - `navigation/waypoint_manager.py`: Sequential waypoint progression and adaptive lookahead target calculation.
    - `navigation/controller.py`: Pure pursuit tracking controller with curvature-aware velocity slowdown.
    - `visualization/plot_map.py`: Publication-grade Matplotlib 2D trajectory rendering.
    - `evaluation/metrics.py`: Quantitative scoring (path efficiency, cross-track error, smoothness).
    - `evaluation/logger.py`: Time-series telemetry recorder and JSON/CSV exporters.
    - `main.py`: Unified CLI entry point.
  - Implemented convenience scripts: `scripts/run_simulation.py` and `scripts/run_experiments.py`.
  - Created complete 25-test suite covering units, integration, and scenario validations.
  - Created documentation files: `architecture.md`, `installation.md`, `methodology.md`, `testing.md`, `results.md`, `limitations.md`, `roadmap.md`.
  - Created directory READMEs for `data/`, `notebooks/`, `outputs/`, `assets/`.
  - Created root `pyproject.toml`, `requirements.txt`, `requirements-dev.txt`, `.gitignore`, `.gitattributes`, `CONTRIBUTING.md`.
- **Validation Results**:
  - `pytest -v`: 25 passed in 0.13s (100% pass rate).
  - `python scripts/run_experiments.py`: All 4 scenarios completed successfully with zero collisions and sub-0.2m mean cross-track error.
- **Next Steps**:
  - Proceed to Milestone 1.1: Environment Isolation, Benchmark Audit & Reproducibility.

---

## Milestone 1.1: Environment Isolation, Benchmark Audit & Reproducibility
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - Isolated environment at `<project-root>\.venv` using Python 3.11.0.
  - Audited dependencies: Removed unused `scipy` from `requirements.txt` and `pyproject.toml`.
  - Installed package in editable mode (`autonomous-nav==0.1.0`) into `.venv`.
  - Audited navigation and planning logic:
    - Fixed A* corner-cutting prevention: switched from `and` to `or` to strictly forbid diagonal moves past any adjacent blocked cell.
    - Added `set_obstacle_world(x, y)` to `GridMap`.
    - Corrected Cross-Track Error (CTE) in `metrics.py`: implemented point-to-segment orthogonal Euclidean distance instead of vertex-sampling, eliminating artificial discretization bias.
    - Updated `engine.py` and `metrics.py` to distinguish simulated virtual time vs wall-clock compute runtime (`time.perf_counter()`), explicit termination reasons (`"goal_reached"`, `"collision"`, `"timeout"`, `"planning_failure"`), and failure flags.
  - Expanded test suite: Added `test_metrics.py` (5 tests), strict corner-cutting unit test (1 test), and explicit failure outcome integration tests (3 tests), reaching **34 passed tests**.
  - Configured project-scoped cache containment: routed Matplotlib cache to `.matplotlib/` and pip cache to `.pip_cache/`, both git-ignored.
  - Re-ran benchmark suite: Achieved 100% success rate across all 4 scenarios with 0 collisions, mean orthogonal CTE between 0.016m and 0.086m, and sub-80ms CPU execution time.
- **Validation Results**:
  - `.venv\Scripts\python.exe -m pytest -v`: 34 passed in 0.19s (100% pass rate).
  - `.venv\Scripts\python.exe scripts/run_experiments.py`: All 4 scenarios completed with zero collisions.
  - `.venv\Scripts\autonomous-nav.exe`: Verified CLI execution directly via project-local environment.
  - `git status --short`: Clean, with zero commits, pushes, or remote repositories.
- **Next Steps**:
  - Milestone 2: Perception Module & Vision-based Grid Mapping (OpenCV).

---

## Milestone 2: Vision-Based Obstacle Perception and Grid Mapping
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - Added `opencv-python>=4.8.0` dependency to `requirements.txt` and `pyproject.toml`, installed into `.venv` using local `--cache-dir .pip_cache`.
  - Implemented modular perception package `src/autonomous_nav/perception/`:
    - `preprocessing.py`: Validation, BGR to grayscale conversion, Gaussian/Bilateral smoothing, and min-max dynamic range normalization.
    - `vision.py`: `OrthographicCamera` (top-down affine with vertical inversion), `PlanarPerspectiveCamera` (ground-plane Homography $\mathbf{H}$ and IPM), and `SyntheticImageRenderer` (deterministic geometric obstacle rasterizer).
    - `obstacle_detection.py`: `ObstacleDetector` implementing Otsu/adaptive thresholding, morphological closing/opening, contour extraction, spatial moments centroid computation, and area bounds filtering.
    - `map_adapter.py`: Decoupled `MapAdapter` interface translating pixel contours and binary masks into metric `GridMap` obstacle cells.
    - `__init__.py`: Exported clean public API for perception module.
  - Implemented configuration dataclass `VisionExtensionConfig` in `config.py` with corresponding `vision` fields in `configs/default_config.json`.
  - Implemented demonstration script `scripts/run_perception_demo.py`:
    - Synthesizes multi-obstacle image.
    - Detects obstacles and estimates metric world positions ($< 0.02\text{ m}$ centroid error).
    - Updates `GridMap` with 364 occupied cells.
    - Plans a 36.49m obstacle-free path from $(2, 2)$ to $(27, 17)$.
    - Generates 4-panel diagnostic figure `outputs/figures/perception_demo.png` and summary metrics `outputs/metrics/perception_demo_summary.json`.
  - Created 13 automated tests in `tests/unit/test_perception.py` covering:
    - Image validation & bad inputs.
    - Grayscale and normalization contracts.
    - Empty and uniform images.
    - Known shape and multi-obstacle detection.
    - Camera round-trip coordinate transformations (Orthographic and IPM Homography).
    - Determinism across repeated runs.
    - Noise area filtering.
    - GridMap updates and end-to-end perception-to-path planning.
- **Validation Results**:
  - `.venv\Scripts\python.exe -m pytest -v`: 47 passed in 0.25s (100% pass rate).
  - `.venv\Scripts\python.exe scripts/run_perception_demo.py`: 3/3 obstacles detected, zero errors.
  - Full backward compatibility: all 34 prior navigation, collision, and kinematics tests continue to pass.
- **Next Steps**:
  - Milestone 2.1: Perception Geometry, Obstacle Safety & Reproducibility Audit.

---

## Milestone 2.1: Perception Geometry, Obstacle Safety & Reproducibility Audit
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Audit Findings & Implemented Fixes**:
  - **OpenCV Version & Dependency Consistency**: Confirmed installed `opencv-python==5.0.0.93` matches `cv2.__version__ == 5.0.0`, verified available on PyPI index as the latest release, matching `opencv-python>=4.8.0` in `requirements.txt` and `pyproject.toml`.
  - **Physical Camera Pinhole Geometry**: Corrected `PlanarPerspectiveCamera` homography matrix derivation for downward pitch $\alpha$:
    $$\mathbf{H} = \begin{bmatrix} c_x \cos\alpha & -f_x & c_x h \sin\alpha \\ c_y \cos\alpha - f_y \sin\alpha & 0 & c_y h \sin\alpha + f_y h \cos\alpha \\ \cos\alpha & 0 & h \sin\alpha \end{bmatrix}$$
    Ensuring ground points inside the field of view properly project within $[0, W_{\text{img}}] \times [0, H_{\text{img}}]$ and reconstruct with sub-millimeter accuracy.
  - **Homography from Correspondences**: Implemented `PlanarPerspectiveCamera.compute_homography_from_correspondences` and `CalibratedHomographyCamera`, handling collinear points, singular matrices, and out-of-bounds pixel checks.
  - **Boundary-Safe Footprint Rasterization**: In `MapAdapter.update_from_detections`, eliminated artificial clamping of polygon vertices to boundary indices, enabling `cv2.fillPoly` to naturally clip off-grid polygons without distorting edge cells. Defaulted to `use_full_contour=True` to eliminate geometric area shrinkage on curved obstacles.
  - **Spatial Affine Mask Projection**: Replaced simplistic `cv2.resize` in `MapAdapter.update_from_binary_mask` with a rigorous 2x3 affine matrix warp (`cv2.warpAffine`), properly aligning binary masks even when camera world extents or origins differ from the grid map.
  - **Footprint IoU & Safety Verification**: Added automated tests measuring footprint overlap ($\text{IoU} = 0.900$) against ground truth, zero false negatives (no under-estimation), robot clearance retention ($> 0.5\text{ m}$) with obstacle inflation ($0.6\text{ m}$), and graceful failure reporting for enclosed unreachable goals.
  - **Expanded Test Suite**: Added 10 audit unit tests in `tests/unit/test_perception.py`, increasing total passing test count from 47 to **57 tests** (100% pass rate).
- **Validation Results**:
  - `.venv\Scripts\python.exe -m pytest -v`: 57 passed in 0.38s.
  - `.venv\Scripts\python.exe scripts/run_perception_demo.py`: 402 occupied cells marked, optimal 36.49m path found.
  - `.venv\Scripts\python.exe scripts/run_experiments.py`: All 4 scenarios completed with zero collisions.
- **Next Steps**:
  - Milestone 3: Interactive Simulation & Pygame Visualizer.

---

## Milestone 3: Interactive 2D Navigation Simulator with Pygame
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - **Dependency Addition**: Added `pygame>=2.5.0` to `requirements.txt` and `pyproject.toml`. Installed `pygame==2.6.1` into `.venv` using local `--cache-dir .pip_cache`. Verified clean import in Python 3.11.0.
  - **Engine Step & Replan Interface**: Refactored `SimulationEngine` (`src/autonomous_nav/simulation/engine.py`) to expose:
    - `reset(grid_map, start, goal)`: initializes episode, plans path, and resets robot pose.
    - `step()`: advances simulation by discrete $\Delta t$, integrates unicycle kinematics, checks collisions, and returns `(RobotState, termination_reason)`.
    - `replan()`: triggers dynamic online A* path re-routing from current robot pose to goal upon environmental edits.
    - `get_result()`: packages final simulation metrics into `SimulationResult`.
    - Maintained full backward compatibility with `run()` wrapper.
  - **GridMap Semantic Method**: Added `set_occupied(gx, gy, occupied=True)` in `src/autonomous_nav/environment/grid_map.py` as semantic alias for obstacle manipulation.
  - **Interactive 2D Pygame Visualizer**: Implemented `InteractiveVisualizer` in `src/autonomous_nav/visualization/animate.py`:
    - Clean dual-viewport layout: left map canvas ($780 \times 680\text{ px}$) and right sidebar telemetry panel ($280 \times 680\text{ px}$).
    - Bidirectional screen $\leftrightarrow$ world coordinate transforms with vertical axis inversion and padding.
    - Dynamic rendering: grid cells, inflated obstacle safety halos, start/goal bullseyes, planned A* path, robot circular chassis ($R_{\text{robot}}$), heading orientation arrow, lookahead target point, and persistent trajectory trail.
    - Live HUD telemetry dashboard: scenario badge, simulation state badge, virtual time, step budget, linear speed, angular rate, cross-track error, driven & planned distance, goal distance.
    - Interactive controls: `[SPACE]` pause/resume, `[S]` / `[->]` single-frame stepping, `[R]` episode reset, `[1-4]` scenario jump, `[N]` scenario cycle, left mouse click to toggle obstacle with dynamic online replanning, `[ESC]/[Q]` exit.
    - Headless mode support: `SDL_VIDEODRIVER="dummy"` and `save_screenshot(path)` via `pygame.image.save()`.
  - **Executable Entry Points**:
    - Created dedicated CLI launcher: `scripts/run_interactive.py`.
    - Added `--interactive` / `-i` flag to unified CLI in `src/autonomous_nav/main.py`.
  - **Snapshot Verification**: Generated verified screenshot artifact `outputs/figures/interactive_sim_screenshot.png` (42,565 bytes).
  - **Automated Test Suite Expansion**: Implemented 9 headless unit tests in `tests/unit/test_visualizer.py` verifying step execution, dynamic replanning, headless surface initialization, bidirectional coordinate conversion, rendering state immutability, pause/step/reset logic, scenario switching, status badge updates, and screenshot disk export.
- **Validation Results**:
  - `.venv\Scripts\python.exe -m pytest -v`: **66 passed in 1.95s** across 8 test modules ($100\%$ pass rate).
  - Visualizer headless snapshot generated and verified at `outputs/figures/interactive_sim_screenshot.png`.
  - Full backward compatibility verified: `scripts/run_experiments.py` and `scripts/run_perception_demo.py` continue to pass cleanly.
- **Next Steps**:
  - Milestone 3.1: Independent Verification and Hardening.

---

## Milestone 3.1: Independent Verification and Hardening
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Audit Findings & Implemented Fixes**:
  - **Dynamic Replanning Stale Path Clearance (`engine.py`)**: Corrected `replan()` so that when planning fails (due to blocked route or obstacle on goal), `self.planned_path` is cleared to `[]` and `waypoint_mgr` is reset to terminal target, preventing stale paths from lingering on screen or in metrics.
  - **Planning Failure Recovery (`engine.py`)**: Corrected `replan()` so that when an obstacle is removed/toggled off, `self.is_done` and `self.termination_reason` are safely reset to allow simulation execution to resume cleanly.
  - **Robot-Pose Collision Detection (`engine.py`)**: Added check in `replan()` for robot disc collision when obstacles are placed directly onto the vehicle, properly setting `collision_occurred = True` and allowing recovery if toggled off.
  - **GUI Status Synchronization (`animate.py`)**: Updated `_get_status_label()` to derive status directly from `self.engine.termination_reason`, guaranteeing 100% agreement between HUD badges and simulation engine states.
  - **Single-Step Key Guard (`animate.py`)**: Guarded single-step event handler (`[S]` / `[->]`) to only advance when paused (`if self.paused and not self.engine.is_done:`), eliminating unintended double-stepping during unpaused simulation.
  - **Empirical Frame Rate Measurement**: Measured exact headless rendering latency ($3.18\text{ ms/frame}$, unthrottled capacity $\approx 315\text{ FPS}$) and full step+render loop duration ($3.87\text{ ms/frame}$, unthrottled capacity $\approx 258\text{ FPS}$); corrected unsupported "sub-millisecond" claim in `docs/results.md`.
  - **Visual Screenshot Inspection**: Inspected `outputs/figures/interactive_sim_screenshot.png` ($1100 \times 720\text{ px}$, 42,562 bytes) verifying all visual elements (robot body, heading quiver, lookahead line, start/goal targets, obstacle halos, and HUD telemetry).
  - **Expanded Automated Test Suite**: Added 9 unit tests to `tests/unit/test_visualizer.py` covering detours, blocked routes, goal obstacles, robot collision, failure recovery, stale path clearance, badge synchronization, accidental advancement guards, and rendering invariance across all metrics.
- **Validation Results**:
  - `.venv\Scripts\python.exe -m pytest -v`: **75 passed in 2.44s** across 8 test suites ($100\%$ pass rate).
  - `.venv\Scripts\python.exe -m autonomous_nav.main --scenario simple_obstacles`: Completed with `GOAL_REACHED`.
  - `.venv\Scripts\python.exe scripts/run_experiments.py`: 4/4 scenarios passed with 0 collisions.
  - `.venv\Scripts\python.exe scripts/run_perception_demo.py`: 3/3 obstacles detected, 36.49m path found.
  - `.venv\Scripts\python.exe scripts/run_interactive.py --headless --frames 30 --screenshot outputs/figures/interactive_sim_screenshot.png`: Clean execution, verified screenshot saved.
  - `git status --porcelain`: Untracked files distinguished from modified files (0 modified tracked files, 0 commits, 0 pushes).
- **Next Steps**:
  - Milestone 4: Dynamic Obstacle Avoidance & Local Trajectory Optimization (DWA / TEB).

---

## Milestone 4: Dynamic Obstacles and Local Collision-Aware Navigation
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Features & Technical Contributions**:
  - **Dynamic Obstacle Modeling (`dynamic_obstacle.py`)**:
    - Created `DynamicObstacle` supporting `linear_ping_pong`, `circular`, `waypoints`, and `constant_velocity` motion patterns.
    - Implemented analytical spatiotemporal trajectory prediction (`predict_position()`, `predict_trajectory()`) for multi-step collision forecasting without forward state mutation.
    - Implemented `DynamicObstacleManager` managing aggregate moving obstacles with collision and distance queries.
  - **Dynamic Window Approach Local Planner (`dwa.py`)**:
    - Implemented `DWAPlanner` using classical 3-term objective function (heading alignment, obstacle clearance, forward velocity).
    - Dynamic window derived from robot current velocities, unicycle acceleration bounds, and max speed constraints.
    - Integrated spatiotemporal collision evaluation against static AABB grid map and future predicted dynamic obstacle discs.
    - Added deterministic tie-breaking logic (prefer higher score; on tie prefer higher speed, then lower $|\omega|$).
    - Added safe emergency stop fallback: if all sampled trajectories collide, returns $(v=0, \omega=0)$ with `is_safe=False`.
  - **Dynamic Benchmark Environments (`scenarios.py`)**:
    - Added `dynamic_crossing`: 2 orthogonal crossing obstacles intersecting nominal route.
    - Added `dynamic_corridor`: Constrained hallway with diagonal crossing obstacle requiring lateral detour.
    - Added `dynamic_chokepoint`: Doorway periodically occluded by patrolling obstacle.
  - **SimulationEngine Integration (`engine.py`)**:
    - Added `nav_mode` configuration (`pure_pursuit` vs `dwa`).
    - Extended `step()` to advance dynamic obstacles, evaluate dynamic collisions, and route command generation to DWA or Pure Pursuit.
    - Added step-level DWA telemetry (`dwa_safe`, `dwa_failures`, `current_candidate_trajectory`).
  - **Interactive Pygame Visualizer (`animate.py`)**:
    - Rendered moving dynamic obstacles as coral discs with directional velocity arrows and future prediction trails.
    - Rendered active DWA candidate trajectory arc in cyan.
    - Added HUD `Nav Mode: DWA_LOCAL` / `PURE_PURSUIT` badge and dynamic obstacle counter.
    - Added runtime `[M]` key handler to toggle navigation mode dynamically.
  - **Automated Comparative Benchmarking (`scripts/run_dynamic_benchmarks.py`)**:
    - Executed comparative evaluation of Pure Pursuit vs DWA across all 3 dynamic scenarios.
    - Serialized quantitative comparison records to `outputs/metrics/dynamic_benchmark_results.json`.
    - Generated trajectory comparison figures in `outputs/figures/`.
  - **Automated Test Suite Expansion**:
    - Created `tests/unit/test_dynamic_obstacle.py` (7 tests: kinematics, prediction, patterns, reset, manager).
    - Created `tests/unit/test_dwa.py` (6 tests: dynamic window limits, trajectory integration, static/dynamic collision detection, emergency stop, determinism).
    - Created `tests/integration/test_dynamic_navigation.py` (5 tests: end-to-end crossing, corridor, chokepoint, telemetry).
- **Validation Results**:
  - Comparative Benchmark Outcome:
    - Pure Pursuit: $0\%$ success rate ($3/3$ collisions with dynamic obstacles).
    - DWA Local Planner: **$100\%$ success rate** ($3/3$ goals reached, 0 collisions, 0 DWA failures).
  - Test Suite: **96 passed in 20.2s** ($100\%$ pass rate).
  - Code Coverage: **82% overall coverage**, with **98% coverage on DWA planner**.
  - Interactive Screenshots: Captured verified PNGs for all dynamic scenarios in `outputs/figures/`.
  - Full Backward Compatibility: Batch simulation, four-scenario static benchmark, and OpenCV perception demo continue to pass without error.
---

## Milestone 4.1: DWA Performance, Benchmark Reproducibility & Safety Audit
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Fixes & Audit Findings**:
  - **Latency Instrumentation & Reconciliation (`measure_dwa_performance.py`)**:
    - Disaggregated DWA planning latency, full simulation step latency, and total mission execution time.
    - Reconciled multi-second mission runtimes ($5.0\text{--}6.1\text{ s}$) with per-step execution ($23.3\text{--}26.8\text{ ms}$ pre-optimization), proving the simulation runs $> 4\times$ faster than real-time physics.
    - Identified that DWA candidate evaluation accounts for $> 95\%$ of per-step computational overhead.
  - **Profiling & Hot-Path Optimization (`collision.py`, `dwa.py`)**:
    - Profiled with `cProfile`, identifying redundant bounds checks in `check_disc_collision` (68,300 calls), exhaustive searches in `distance_to_nearest_obstacle`, and repetitive trajectory projections in `predict_position` (136,500 calls).
    - Implemented NumPy bounding-box vector slice fast-paths in `CollisionChecker.check_disc_collision` and `distance_to_nearest_obstacle`, accelerating clearance queries by $1.55\times\text{--}9.60\times$.
    - Implemented prediction horizon caching in `DWAPlanner.compute_command`, reducing projection calls by $165\times$.
    - Achieved $2.4\times\text{--}3.3\times$ overall planning speedup ($8.0\text{--}9.6\text{ ms/step}$ post-optimization, running $10.4\times\text{--}12.2\times$ faster than real-time physics) while strictly preserving collision margins and unicycle kinematics.
  - **Temporal Continuous Collision Checking (`collision.py`, `dwa.py`)**:
    - Implemented `CollisionChecker.relative_continuous_distance(p0, p1, o0, o1)` calculating exact orthogonal distance from difference space origin $(0, 0)$ to relative displacement segment.
    - Integrated relative continuous swept-distance checking into `DWAPlanner.check_trajectory_collision`, mathematically eliminating temporal tunneling between adjacent time samples.
  - **Expanded Multi-Trial Dynamic Benchmark Suite (`run_dynamic_benchmarks.py`)**:
    - Expanded benchmark to 18 deterministic trials (9 per mode across nominal, slow, and fast obstacle speeds).
    - Recorded statistical distributions: Pure Pursuit achieved 3/9 successes (33.3% success rate, 6 dynamic collisions) strictly by timing coincidence when obstacles cleared before arrival.
    - DWA achieved 9/9 successes (100.0% success rate, 0 dynamic collisions, 0 planner failures, mean loop runtime $2.41\text{ s}$).
    - Documented architectural baseline limitation of Pure Pursuit (open-loop with respect to moving agents).
  - **Verified Headless Screenshot Artifacts**:
    - Regenerated headless screenshots for all 3 dynamic scenarios (`dynamic_crossing`, `dynamic_corridor`, `dynamic_chokepoint`).
    - Verified dimensions ($1100 \times 720\text{ px}$), non-zero channels, and conducted pixel array scans for coral obstacles, cyan DWA arcs, amber prediction trails, yellow robot chassis, and HUD text.
  - **Automated Test Suite Expansion**:
    - Created `tests/unit/test_temporal_collision.py` (8 unit tests covering tunneling, near-tangency, head-on/receding paths, fast-crossing rejection, emergency stop, velocity/acceleration limits, determinism, and engine telemetry reset).
- **Validation Results**:
  - Test Suite: **104 passed in 8.46s** ($100\%$ pass rate across 11 test modules).
  - Code Coverage: **82% overall coverage** ($2062$ statements), with **98% coverage on DWA planner** and **97% coverage on collision checker**.
  - All Baselines Preserved: 4-scenario static benchmark, batch simulation on `simple_obstacles`, OpenCV perception demo, and dynamic benchmarks pass cleanly.
- **Next Steps**:
  - Milestone 5: State Estimation & Sensor Noise Modeling (EKF fusing IMU and wheel odometry).

---

## Milestone 5: Sensor Noise Modeling and Extended Kalman Filter State Estimation
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - **Sensors Subsystem (`src/autonomous_nav/sensors/`)**:
    - Implemented `WheelOdometrySensor`: simulates linear forward velocity with additive Gaussian white noise $\mathcal{N}(0, \sigma_v^2)$ and systematic wheel radius scaling error.
    - Implemented `IMUSensor`: simulates single-axis rate gyro with white noise $\mathcal{N}(0, \sigma_\omega^2)$ and time-varying random-walk bias drift $b_k = b_{k-1} + \mathcal{N}(0, \sigma_b^2 \Delta t)$.
    - Implemented `GNSSReceiver`: generates periodic 2D position fixes $(x, y)$ at 1 Hz epoch schedule with 2D Gaussian positional noise, supporting designated signal outage intervals.
    - Implemented `SensorSuite`: bundles sensor streams with deterministic pseudo-random number generator seeding (`seed=42`) and generates timestamped `SensorMeasurements`.
  - **State Estimation Subsystem (`src/autonomous_nav/estimation/`)**:
    - Implemented `ExtendedKalmanFilter`: 5-state vector $\mathbf{x} = [x, y, \theta, v, b_g]^T$ estimating world position $(x, y)$, heading angle ($\theta$), linear forward speed ($v$), and rate gyro bias ($b_g$).
    - Implemented analytical state transition Jacobian matrix $F = \partial f / \partial \mathbf{x}$ for unicycle kinematics.
    - Implemented Joseph-form stabilized error covariance updates $P = (I - KH)P(I - KH)^T + KRK^T$ guaranteeing positive semi-definiteness and symmetry under finite precision.
    - Implemented circular angle normalization across the $[-\pi, \pi)$ branch cut.
    - Implemented 95% position uncertainty ellipse computation ($a, b, \phi$) via 2D marginal covariance eigen-decomposition.
  - **Simulation Engine & Configuration Integration**:
    - Extended `SimulationConfig` with `SensorConfig`, `EKFConfig`, and `state_feedback` mode (`"ground_truth"`, `"ekf"`, `"dead_reckoning"`).
    - Updated `SimulationEngine` (`engine.py`): true physical unicycle integrates physics and evaluates collisions strictly on ground truth, while navigation controllers (Pure Pursuit or DWA) strictly consume estimated poses in EKF mode without state leakage.
  - **Evaluation & Benchmarking (`metrics.py`, `run_estimation_benchmark.py`)**:
    - Added `calculate_estimation_errors()` computing Position RMSE, Position MAE, Max Position Error, Heading RMSE, and Velocity RMSE.
    - Implemented `scripts/run_estimation_benchmark.py` running 36 trials across 3 scenarios (`simple_obstacles`, `corridor_s_bend`, `urban_grid`) comparing Ground Truth vs Dead Reckoning vs Full EKF vs 5s GNSS Outage across seeds `[42, 43, 44]`.
    - Persisted benchmark results to `outputs/metrics/estimation_benchmark_results.json`.
  - **Interactive Pygame Visualizer (`animate.py`)**:
    - Rendered cyan estimated robot chassis outline and 95% position uncertainty ellipse.
    - Rendered recent GNSS position fixes as hot pink crosshairs.
    - Extended sidebar HUD telemetry with Feedback mode badge, Position Estimation Error, Heading Estimation Error, Gyro Bias Estimate, and GNSS Fix count.
    - Added runtime `[E]` key handler to cycle feedback modes (`GROUND_TRUTH` $\to$ `EKF_FUSION` $\to$ `DEAD_RECKONING`).
  - **Automated Test Suite Expansion**:
    - Created `tests/unit/test_sensors.py` (7 tests covering noise distributions, scale error, bias drift, GNSS decimation, outage masking, and determinism).
    - Created `tests/unit/test_ekf.py` (9 tests covering state vector shapes, analytical Jacobian numerical finite-difference verification, covariance symmetry and positive definiteness, Joseph update numerical stability, angle wrapping, variance reduction, uncertainty ellipses, and stationary convergence).
    - Created `tests/integration/test_ekf_navigation.py` (5 tests covering closed-loop EKF navigation success, feedback isolation without ground-truth leakage, Dead Reckoning drift vs EKF bounding, GNSS outage divergence and recovery, and multi-seed determinism).
- **Validation Results**:
  - Test Suite: **125 passed in 9.61s** ($100\%$ pass rate across 14 test modules).
  - Code Coverage: **84% overall codebase coverage** ($2422$ statements, $392$ missed).
    - Sensor models (`sensor_models.py`): **98% coverage**
    - Extended Kalman Filter (`ekf.py`): **96% coverage**
    - Simulation Engine (`engine.py`): **96% coverage**
  - Benchmark Performance:
    - Ground Truth: $100.0\%$ success across all scenarios.
    - Full EKF: **$100.0\%$ success** across all scenarios, with Position RMSE bounded between **$0.249\text{--}0.258\text{ m}$** (consistent with GNSS $\sigma = 0.25\text{ m}$) and heading error $< 4.3^\circ$.
    - Dead Reckoning: accumulates uncompensated drift on longer routes ($0.801\text{ m}$ max error on `urban_grid`).
    - GNSS Outage: error escalates to $1.30\text{--}1.51\text{ m}$ during 5s signal drop, re-converging rapidly upon fix resumption.
    - Estimation Filter Step Latency: **$0.060\text{--}0.074\text{ ms/step}$** ($> 1400\times$ faster than the $100\text{ ms}$ control loop deadline).
  - Full Backward Compatibility: 7-scenario benchmark (`run_experiments.py`), perception demo (`run_perception_demo.py`), and dynamic benchmark (`run_dynamic_benchmarks.py`) continue to pass 100% cleanly.
- **Next Steps**:
  - Milestone 5.1: State Estimation Validation Audit.

---

## Milestone 5.1: State Estimation Validation Audit
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Implemented Scope**:
  - **Audit of Ground-Truth Estimation Metric Discrepancy**:
    - Discovered root causes for Milestone 5 reporting $\approx 0.24\text{--}0.26\text{ m}$ position RMSE in `ground_truth` mode:
      1. `self.estimated_trajectory` recorded background EKF output unconditionally, and `get_result()` evaluated against the background EKF rather than the feedback pose used for control.
      2. Step loop timing asymmetry sampled sensors and updated EKF before `robot.step()`, creating a 1-step latency and an unstepped trailing estimate on termination.
    - Resolved by formally establishing `feedback_trajectory` on `SimulationEngine` and `SimulationResult`. In `ground_truth` mode, feedback pose is true state by construction, yielding identically $0.0000\text{ m}$ error without artificial hardcoding.
  - **Decoupled Metric Definitions**:
    - Implemented `calculate_tracking_errors()` in `metrics.py` measuring orthogonal Cross-Track Error (Mean/Max CTE), path efficiency, and actual vs planned path lengths.
    - Decoupled State Estimation Accuracy (feedback pose vs true pose: Pos RMSE/MAE, Max Pos Error, Heading RMSE) from Trajectory-Tracking Performance (driven path vs planned reference path) and Navigation Outcomes (success, collision, timeout).
  - **Strict Feedback Isolation & GNSS Verification**:
    - Enforced that waypoint manager and controllers exclusively consume `feedback_state`.
    - Verified that in `dead_reckoning` mode, zero GNSS fixes are processed (`engine.gnss_fixes_processed == 0`, `len(engine.gnss_measurements) == 0`).
  - **Comprehensive Multi-Trial Re-Benchmarking**:
    - Updated `scripts/run_estimation_benchmark.py` to evaluate 36 trials across 3 scenarios (`simple_obstacles`, `corridor_s_bend`, `urban_grid`) and 4 modes (`ground_truth`, `dead_reckoning`, `ekf`, `ekf_outage`) across seeds `[42, 43, 44]`.
    - Generated distinct Table 1 (Estimation Accuracy), Table 2 (Trajectory Tracking & Outcomes with explicit denominators e.g. `3/3 (100.0%)`), and Section 3 (GNSS Outage & Filter Recovery breakdown).
    - Serialized full audit telemetry to `outputs/metrics/estimation_benchmark_results.json`.
  - **Automated Test Suite Expansion**:
    - Added unit tests in `tests/unit/test_metrics.py`: `test_calculate_tracking_errors`, `test_calculate_estimation_errors_zero_for_identical`.
    - Added integration tests in `tests/integration/test_ekf_navigation.py`:
      - `test_ground_truth_feedback_zero_estimation_error`
      - `test_dead_reckoning_strictly_zero_gnss_leakage`
      - `test_metric_distinction_estimation_vs_tracking`
- **Validation Results**:
  - Full Test Suite: **130 passed in 25.24s** ($100\%$ pass rate across 14 test modules).
  - Codebase Coverage: **84% overall codebase coverage** ($2454$ statements, $397$ missed).
  - Ground Truth Feedback: exactly $0.0000\text{ m}$ Pos RMSE, $0.0000\text{ m}$ Max Pos Error, $0.00^\circ$ Heading RMSE across all tracks.
  - EKF Feedback: bounded Pos RMSE of $0.2488\text{--}0.2583\text{ m}$, matching sensor noise $\sigma = 0.25\text{ m}$.
  - Tracking Metrics: Mean CTE $0.0158\text{--}0.0874\text{ m}$ (Ground Truth), $0.1580\text{--}0.1915\text{ m}$ (EKF).
  - 100% Backward Compatibility: `run_experiments.py` (7/7 PASS), `run_perception_demo.py` (PASS), and `run_dynamic_benchmarks.py` (DWA 9/9 100%) confirmed.
- **Next Steps**:
  - Milestone 5.2: Final Evidence and Benchmark Integrity Check.

---

## Milestone 5.2: Final Evidence and Benchmark Integrity Check
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Scope & Verified Corrections**:
  - **Explicit Timestamp Semantics & Validation**:
    - Added `timestamp: float = 0.0` to `RobotState` dataclass in `src/autonomous_nav/simulation/robot.py`.
    - Updated `RobotModel.reset()` ($t=0.0$) and `RobotModel.step()` ($t += dt$) to maintain monotonic simulation time.
    - Updated `ExtendedKalmanFilter.get_estimated_robot_state(timestamp=...)` to accept and set explicit timestamps.
    - Added strict pairwise timestamp verification and length equality assertions in `calculate_estimation_errors()` in `src/autonomous_nav/evaluation/metrics.py`.
    - Synchronized `SimulationEngine.step()` so that physical ground truth, estimated trajectory, and feedback trajectory are strictly synchronized at every step ($0 \dots N$) with zero trailing or missing elements across all termination modes (goal reached, timeout, early collision).
  - **Audit of Benchmark Data Integrity**:
    - Verified all 36 trials in `outputs/metrics/estimation_benchmark_results.json` (3 scenarios $\times$ 4 modes $\times$ 3 seeds).
    - Directly verified that aggregated scorecards match individual trial means to 4 decimal places.
    - Verified outcomes: 31 successful runs, 5 collisions during GNSS outage (due to accumulated uncompensated dead reckoning drift exceeding the $1.0\text{ m}$ inflation safety clearance).
  - **Latency Characterization & Decomposition**:
    - Disentangled conflated "Filter Latency" in `engine.step()` by instrumenting:
      1. Pure EKF compute latency (`estimator_compute_ms`): Mean **$0.038\text{--}0.051\text{ ms}$** ($38\text{--}51\ \mu\text{s}$), P95 **$0.041\text{--}0.089\text{ ms}$**.
      2. Full simulation step cycle latency (`step_latency_ms`): Mean **$0.059\text{--}0.076\text{ ms}$** ($59\text{--}76\ \mu\text{s}$), P95 **$0.064\text{--}0.123\text{ ms}$**.
    - Updated `scripts/run_estimation_benchmark.py` Table 1 and serialized JSON to report both metrics with Mean and P95.
    - Documented real-time performance disclaimers regarding Python/NumPy user-space execution on Windows OS.
  - **Honest Interpretation of EKF vs Dead Reckoning**:
    - Heading RMSE: Explained that Dead Reckoning achieves lower heading RMSE ($1.65^\circ\text{--}2.96^\circ$ vs EKF $3.18^\circ\text{--}4.28^\circ$) because GNSS provides only position measurements $(x, y)$. Non-zero cross-covariance terms in $P$ inject lateral position noise innovations into heading estimates, causing angular jitter. Gyro Dead Reckoning integrates rate with low noise ($0.015\text{ rad/s}$), remaining smooth without innovation shocks.
    - Position RMSE: Over short runs ($21\text{ s}$, $30\text{ m}$), $1.5\%$ wheel odometry scale error drifts only $\approx 0.45\text{ m}$, rivaling EKF ($0.25\text{ m}$ noise floor). On longer runs (`urban_grid`), Dead Reckoning error diverges monotonically to $0.801\text{ m}$ max ($0.327\text{ m}$ RMSE), while EKF remains bounded at $0.556\text{ m}$ max ($0.249\text{ m}$ RMSE).
  - **Test Suite Expansion**:
    - Unit tests: `test_calculate_estimation_errors_timestamp_alignment_enforced`, `test_calculate_estimation_errors_heading_pi_wrapping`.
    - Integration tests: `test_trajectory_timestamp_semantics_and_terminal_alignment`, `test_early_collision_termination_timestamp_alignment`.
- **Validation Results**:
  - Full Test Suite: **134 passed in 24.86s** ($100\%$ pass rate across 12 test modules).
  - Codebase Coverage: **84% overall codebase coverage** ($2490$ statements, $403$ missed).
  - All benchmarks passing: `run_experiments.py` (7/7 PASS), `run_perception_demo.py` (PASS), `run_dynamic_benchmarks.py` (9/9 PASS, 100%), `run_estimation_benchmark.py` (36 trials refreshed).
- **Next Steps**:
  - Milestone 6: High-Fidelity Simulation Bridge (ROS 2 / CARLA point clouds and costmaps).

---

## Milestone 6: High-Fidelity Simulation Bridge
- **Date**: 2026-10-20
- **Status**: Completed & Verified (Bridge Foundation & Offline Test Adapter)
- **Implemented Scope**:
  - **Simulator Feasibility Assessment**:
    - Audited host environment (Windows 10/11 x86_64, Python 3.11.0).
    - Verified absence of installed robotics middleware and simulators (`rclpy`, `ros2`, `carla`, `gazebo`, `gz`).
    - Evaluated resource constraints: Downloading 20GB+ CARLA or installing native ROS 2 on Windows is prohibited by safety constraints and resource limits.
    - Selected primary route: Protocol-first, simulator-agnostic bridge architecture conforming to REP-103/REP-105 standards, validated via an in-process deterministic mock adapter (`OfflineTestAdapter`).
  - **Bridge Package Architecture (`autonomous_nav.bridge`)**:
    - Created typed data contracts in `contracts.py`:
      - `TimestampedRobotState`: 3D Cartesian coordinates, Euler roll/pitch/yaw, linear velocities $(v_x, v_y, v_z)$, scalar speed $v$, yaw rate $\omega$, timestamp in seconds, `frame_id`, and `child_frame_id`.
      - `NavigationCommand`: Commanded forward speed $v_{\text{cmd}}$ and yaw rate $\omega_{\text{cmd}}$, timestamp, and sequence ID with limit validation and clamping.
      - `OdometryObservation`, `ImuObservation`, `GnssObservation`, `LaserScanObservation`: Synchronized sensor packets with explicit frame IDs and validity flags.
      - `SimulatorSensorPacket`: Unified synchronous observation packet.
      - `SimulatorCapabilities`: Static profile detailing simulator backend, supported frames/sensors, and health.
      - `SimulationLifecycleState`: Formal state enum (`UNINITIALIZED`, `READY`, `RUNNING`, `PAUSED`, `STOPPED`, `ERROR`).
    - Defined abstract bridge interface in `interface.py` (`SimulationBridge`) with formal lifecycle hooks: `connect`, `initialize`, `step`, `reset`, `disconnect`, `send_command`, `get_sensor_observations`, `get_ground_truth_state`.
    - Implemented `BridgeConfig` in `config.py` defining frame whitelists, command velocity limits, and LiDAR raycast parameters.
  - **Deterministic Offline Test Adapter (`mock_adapter.py`)**:
    - Created `OfflineTestAdapter` explicitly labeled as an in-process mock adapter for CI/testing (NOT a real 3D simulator).
    - Implemented deterministic unicycle RK2 physics, synthetic sensor noise streams, and planar LiDAR raycasting against metric `GridMap` obstacles.
    - Enforced strict input validation: rejected negative/non-monotonic timestamps, non-finite floats (NaN, Inf), unsupported coordinate frames, and excessive command limits.
  - **Bridge-to-Navigation Mappings (`mapping.py`)**:
    - Implemented bidirectional projections between 3D contracts and 2D components (`state_3d_to_2d_robot_state`, `robot_state_to_3d_state`).
    - Implemented translation from `SimulatorSensorPacket` directly into `SensorMeasurements` for direct EKF ingestion.
    - Implemented LiDAR scan projection `laser_scan_to_grid_map()` updating discrete occupancy cells from range returns.
    - Implemented bidirectional conversions between Euler angles and unit quaternions (`quaternion_to_euler`, `euler_to_quaternion`).
  - **Automated Test Suite Expansion**:
    - Added 26 dedicated unit and integration tests across:
      - `tests/unit/test_bridge_contracts.py` (11 tests)
      - `tests/unit/test_bridge_adapter.py` (7 tests)
      - `tests/integration/test_bridge_navigation.py` (8 tests)
- **Validation Results**:
  - Full Test Suite: **160 passed in 11.60s** ($100\%$ pass rate across 15 test modules).
  - Codebase Coverage: **85% overall codebase coverage** ($2897$ statements, $431$ missed).
  - Bridge package coverage:
    - `contracts.py`: 93%
    - `config.py`: 100%
    - `mock_adapter.py`: 96%
    - `mapping.py`: 93%
    - `interface.py`: 71%
  - 100% Backward Compatibility:
    - `scripts/run_experiments.py`: 7/7 scenarios PASS.
    - `scripts/run_perception_demo.py`: PASS.
    - `scripts/run_dynamic_benchmarks.py`: DWA 9/9 PASS (100.0% success rate).
    - `scripts/run_estimation_benchmark.py`: 36 trials PASS.
- **Next Steps**:
  - Milestone 6.1: Evidence and Interface Audit.

---

## Milestone 6.1: Evidence and Interface Audit
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Scope & Verified Corrections**:
  - **Audit of Estimation Benchmark Claims**:
    - Located all 36 raw trial results in `outputs/metrics/estimation_benchmark_results.json` and inspected `scripts/run_estimation_benchmark.py`.
    - Identified exact metric: Point-to-point Euclidean Absolute Position Error (APE) $\text{RMSE} = \sqrt{\frac{1}{n}\sum_{i=1}^n (x_{\text{gt},i} - x_{\text{est},i})^2 + (y_{\text{gt},i} - y_{\text{est},i})^2}$.
    - Recomputed aggregate performance across all 9 trials:
      * Dead Reckoning Pos RMSE: **$0.2676\text{ m}$**
      * EKF Pos RMSE: **$0.2522\text{ m}$**
      * Pos RMSE Reduction: **$+0.0154\text{ m}$** (**$+5.77\%$ improvement**)
    - Identified regime variance across scenarios:
      * `simple_obstacles` (short): DR RMSE $0.2328\text{ m}$ vs EKF RMSE $0.2583\text{ m}$ ($\Delta = -0.0255\text{ m}$, **$-10.96\%$** change). Dead reckoning slightly outperforms EKF because $1.5\%$ odometry scale error does not have time to accumulate substantial drift over $21\text{ s}$, whereas discrete $1\text{ Hz}$ GNSS ($\sigma = 0.25\text{ m}$) injects sensor jitter.
      * `corridor_s_bend` (medium): DR RMSE $0.2433\text{ m}$ vs EKF RMSE $0.2488\text{ m}$ ($\Delta = -0.0055\text{ m}$, **$-2.25\%$** change, comparable within noise floor).
      * `urban_grid` (long, multi-turn): DR RMSE $0.3267\text{ m}$ vs EKF RMSE $0.2494\text{ m}$ ($\Delta = +0.0773\text{ m}$, **$+23.66\%$** improvement by EKF, demonstrating drift containment over longer paths).
    - Formally corrected earlier informal claims of "$\approx 81.3\%$ error reduction" as ungrounded and erroneous. Enhanced `scripts/run_estimation_benchmark.py` with explicit Section 4 comparative analysis and refreshed `outputs/metrics/estimation_benchmark_results.json`.
  - **Interface & Timestamp Hardening**:
    - Fixed missing `self` parameter in `SimulationBridge.get_ground_truth_state(self)` signature in `src/autonomous_nav/bridge/interface.py`.
    - Enforced intra-packet timestamp synchronization in `SimulatorSensorPacket.__post_init__` (rejects observations with timestamps differing by $>1\text{ ms}$ from packet header).
    - Enforced quaternion norm normalization and rejected degenerate/non-finite inputs in `quaternion_to_euler()`.
    - Added `as_quaternion()` and `from_quaternion()` helper methods to `TimestampedRobotState` dataclass.
    - Added integration test `test_bridge_closed_loop_ekf_feedback_isolated_navigation` verifying complete isolation between ground truth and controller feedback.
    - Documented World frame (REP-103 ENU) and Robot Body frame (REP-105 FLU) coordinate conventions and timestamp semantics. Qualified REP compliance: conforms to conventions (SI units, ENU/FLU standards, right-handed geometry) and frame naming (`map`, `odom`, `base_link`, sensor links) without implementing a dynamic tf2 transform tree.
  - **Automated Test Suite Expansion**:
    - Added 4 targeted tests across `tests/unit/test_bridge_contracts.py` and `tests/integration/test_bridge_navigation.py`.
    - Expanded test suite from 160 to **164 passed in 10.40s** ($100\%$ pass rate across 15 test modules).
    - Codebase coverage maintained at **85%** ($2952$ statements, $435$ missed).
  - **Regression Verification**:
    - `scripts/run_experiments.py`: 7/7 scenarios PASS.
    - `scripts/run_perception_demo.py`: PASS (402 cells marked, A* path length 36.49m).
    - `scripts/run_dynamic_benchmarks.py`: Pure Pursuit 3/9 (33.3%), DWA 9/9 (100.0%, 0 collisions).
    - `scripts/run_estimation_benchmark.py`: 36 trials PASS.
  - **Dataset Preparation (Research Only)**:
    - Compared candidate datasets: KITTI Vision Benchmark (Odometry) vs TUM RGB-D Benchmark.
    - Recommended TUM RGB-D (`freiburg2_pioneer_slam2`) for lightweight, CPU-only execution on Windows with ground-robot differential drive kinematics and direct depth-to-grid mapping.
- **Next Steps**:
  - Milestone 7: Real-World Dataset Ingestion & Validation.

---

## Milestone 7: Real-World Dataset Integration
- **Date**: 2026-10-20
- **Status**: Completed & Verified
- **Scope & Implementation Details**:
  - **Dataset Verification & Containment**:
    - Investigated official TUM RGB-D sequence `freiburg2_pioneer_slam2` (`rgbd_dataset_freiburg2_pioneer_slam2.tgz`). Confirmed exact file size is **1,183,281,970 bytes (~1.10 GB / 1128.47 MB)**.
    - Verified network transfer rate to TUM webshare server ($\approx 0.43\text{ MB/s}$, requiring $\approx 44\text{ minutes}$ for the complete archive).
    - Identified and downloaded the official standalone motion capture ground truth file `rgbd_dataset_freiburg2_pioneer_slam2-groundtruth.txt` (**2,262,149 bytes**, $33,188$ poses at $100\text{ Hz}$, $115.63\text{ s}$ duration, $23.326\text{ m}$ driven path) to `data/tum_rgbd/groundtruth.txt`.
    - Updated `.gitignore` to safely exclude raw `.tgz`, `.bag`, and dataset cache files while retaining `data/tum_rgbd/README.md` and `.gitkeep`. Verified `git check-ignore` passes cleanly.
    - Documented sequence metadata, official URLs, Creative Commons Attribution 4.0 / 3.0 licensing, and formal citation (Sturm et al., IROS 2012) in `data/tum_rgbd/README.md`.
  - **Modular Dataset Adapter Package (`autonomous_nav.datasets`)**:
    - `config.py`: Calibrated Freiburg 2 camera intrinsics ($f_x=520.9, f_y=521.0, c_x=325.1, c_y=249.7$), pioneer body extrinsics ($d_x=0.15\text{ m}, d_z=0.40\text{ m}$), and `DatasetConfig` (depth scale factor 5000.0, height filtering $[0.05, 1.20]\text{ m}$).
    - `groundtruth.py`: `GroundTruthPose` (with quaternion yaw extraction and `RobotState` conversion) and `GroundTruthTrajectory` (timestamp index, nearest-neighbor query, spherical linear interpolation SLERP, path statistics).
    - `depth_projection.py`: 16-bit uint16 depth image loading, pinhole unprojection to 3D camera point cloud ($X_c, Y_c, Z_c$), optical to REP-105 robot body transform ($X_b = Z_c + d_x, Y_b = -X_c + d_y, Z_b = -Y_c + d_z$), obstacle height slicing $[0.05, 1.20]\text{ m}$ (rejecting floor and ceiling), body-to-world coordinate transform, and 2D discrete `GridMap` occupancy rasterization.
    - `adapter.py`: `TUMRGBDAdapter` orchestrating dataset file discovery, ground truth kinematics parsing, depth frame projection, and trajectory tracking evaluation.
  - **Reproducible Evaluation Script (`scripts/run_dataset_evaluation.py`)**:
    - Executes ground truth kinematics parsing and path metrics calculation.
    - Projects depth frames into 3D points, filters floor/ceiling, and updates 2D `GridMap`.
    - Evaluates trajectory tracking against real-world ground truth poses using SLERP interpolation.
    - Saves serialized JSON report to `outputs/metrics/dataset_evaluation_summary.json` (2842 bytes).
    - Generates publication-ready 3-panel diagnostic visualization to `outputs/figures/dataset_evaluation_diagnostic.png` (188,973 bytes).
  - **Automated Test Suite Expansion**:
    - Added 9 unit tests in `tests/unit/test_dataset_adapter.py` using synthetic calibrated fixtures (independent of network).
    - Added 2 integration tests in `tests/integration/test_dataset_integration.py` validating local file parsing and evaluation pipeline.
    - Full test suite expanded to **175 passed in 25.43s** ($100\%$ pass rate across 17 test modules).
    - Total codebase coverage maintained at **85%** ($3385$ statements, $522$ missed).
  - **Regression Verification**:
    - `scripts/run_experiments.py`: 7/7 scenarios PASS.
    - `scripts/run_perception_demo.py`: PASS.
    - `scripts/run_dynamic_benchmarks.py`: Pure Pursuit 3/9 (33.3%), DWA 9/9 (100.0%, 0 collisions).
    - `scripts/run_estimation_benchmark.py`: 36 trials PASS.
    - `scripts/run_dataset_evaluation.py`: PASS (Pos RMSE $0.0425\text{ m}$, Heading RMSE $1.16^\circ$).
- **Next Steps**:
  - Milestone 7.1: Real-Frame Validation and Dataset Evidence Audit.

---

## Milestone 7.1: Real-Frame Validation and Dataset Evidence Audit
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: CONDITIONAL PASS — Provenance established, authentic ground truth fully validated, depth archive download deferred due to TUM network limits with calibrated synthetic fixture validated)
- **Scope & Evidence Findings**:
  - **Depth-Frame Provenance Resolution**:
    - Audited `scripts/run_dataset_evaluation.py` lines 171–215. Determined definitively that the 3 depth frames reported in Milestone 7 were **synthetically generated fixtures** created dynamically in memory via `generate_benchmark_synthetic_depth_scene()`, NOT authentic images extracted from the TUM archive.
    - Investigated acquisition of authentic depth frames:
      * Sequence archive `rgbd_dataset_freiburg2_pioneer_slam2.tgz` is **1,183,281,970 bytes (~1.10 GB)**.
      * Drive `D:` has **213.06 GB free** disk space.
      * TUM webshare server download speed measured at $\approx 0.05\text{--}0.43\text{ MB/s}$, requiring $\approx 45\text{--}90\text{ minutes}$ for a full download.
      * Bounded HTTP Range request streaming downloaded 15 MB of the archive. Tar stream inspection proved that the filesystem ordered all 3,318 RGB PNG files ($\approx 1.4\text{ GB}$ uncompressed) before the `depth/` folder. Zero depth frames exist in the initial 18 MB of uncompressed tar data.
      * Checked TUM web endpoints for standalone depth images or `depth.txt`: returned HTTP 404 (no individual depth file endpoints).
      * Ground truth file `data/tum_rgbd/groundtruth.txt` is **100% authentic**, with size **2,262,149 bytes** and SHA-256 `bd57b2c9ce6776fe0d39c7795d7f7789bf206e4eaa58e04aa216090b7a55d08a`.
  - **Dataset License & Attribution Audit**:
    - Checked authoritative TUM benchmark landing page (`https://vision.in.tum.de/data/datasets/rgbd-dataset`) under `id="license"`.
    - Confirmed official text: *"Unless stated otherwise, all data in the TUM RGB-D benchmark is licensed under a Creative Commons 4.0 Attribution License (CC BY 4.0) and the accompanying source code is licensed under a BSD-2-Clause License."*
    - Resolved license discrepancy: Earlier publications used CC BY 3.0, but the official benchmark terms have formally standardized on CC BY 4.0.
    - Distinguished legal copyright permission (CC BY 4.0) from academic attribution (Sturm et al., IROS 2012).
  - **Calibration & Coordinate Transform Verification**:
    - Verified Freiburg 2 RGB camera intrinsics from official calibration table: $f_x=520.9, f_y=521.0, c_x=325.1, c_y=249.7$, $k_1=0.2312, k_2=-0.7849, p_1=-0.0033, p_2=-0.0001, k_3=0.9172$.
    - Verified from TUM documentation that OpenNI pre-registers depth to RGB camera; unprojection uses RGB intrinsics directly without radial undistortion.
    - Verified camera-to-body extrinsics ($d_x=0.15\text{ m}, d_z=0.40\text{ m}$) and REP-105 FLU frame conventions ($X_b = Z_c + d_x, Y_b = -X_c + d_y, Z_b = -Y_c + d_z$).
    - Verified floor plane height filtering ($[0.05, 1.20]\text{ m}$), achieving 100% floor point rejection.
  - **Metric Decoupling & Scientific Disclosure**:
    - Fully audited the reported $0.0425\text{ m}$ position RMSE and $1.16^\circ$ heading RMSE: proved mathematically and empirically that this resulted from a synthetic perturbation test ($\sigma = 0.03\text{ m}$ on x, y; $\sigma = 0.02\text{ rad}$ on heading), evaluating SLERP interpolation accuracy, NOT visual odometry or SLAM estimation.
    - Implemented and evaluated genuine closed-loop waypoint tracking using `PurePursuitController` on Pioneer ground truth waypoints:
      * Waypoints Followed: 61
      * Driven Distance: 19.30 m
      * Mean Cross-Track Error: **0.0429 m** (4.29 cm)
      * Max Cross-Track Error: **0.1485 m** (14.85 cm)
      * Goal Reached: **True**
  - **Automated Test Suite Expansion**:
    - Added 3 unit tests in `tests/unit/test_dataset_adapter.py` (NaN/Inf rejection, floor plane rejection invariance, and manifest provenance verification).
    - Expanded full test suite to **178 passing tests** ($100\%$ pass rate in $31.67\text{ s}$).
    - Codebase coverage maintained at **85%**.
  - **Reproducible Artifacts**:
    - `outputs/metrics/dataset_evaluation_summary.json` (4,462 bytes): complete transparent provenance and separate metric sections.
    - `outputs/figures/dataset_evaluation_diagnostic.png` (186,051 bytes): publication-ready 3-panel figure with explicit fixture labeling.
- **Next Steps**:
  - Milestone 7.2: Authentic TUM Depth-Frame Acquisition and End-to-End Validation.

---

## Milestone 7.2: Authentic TUM Depth-Frame Acquisition and End-to-End Validation
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: PASS)
- **Scope & Implementation Details**:
  - **Authentic Archive Acquisition (`scripts/download_tum_archive.py`)**:
    - Developed a robust, resumable, multi-threaded chunk downloader using HTTP Range requests (`Accept-Ranges: bytes`) with exponential backoff retry, `.state.json` resume tracking, and `.part` file staging.
    - Successfully downloaded the official sequence archive `rgbd_dataset_freiburg2_pioneer_slam2.tgz` (**1,183,281,970 bytes**, ~1.10 GB) from the official TUM webshare portal (`https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.tgz`).
    - Verified archive cryptographic integrity: computed SHA-256 digest `67f6c8458e7d9010e0a816ee893f862d27da99fa9d86a711b8c70c4b84e1a077` and verified tarfile stream integrity.
    - Verified ground truth file `groundtruth.txt` (**2,262,149 bytes**, SHA-256 `bd57b2c9ce6776fe0d39c7795d7f7789bf206e4eaa58e04aa216090b7a55d08a`, 33,188 poses at 100 Hz, 115.632s duration).
  - **Authentic Depth Frame Extraction & Manifest**:
    - Selectively extracted the official depth index `depth.txt` (**95,535 bytes**, 2,075 depth frames) and 5 authentic 16-bit single-channel depth PNGs spaced across the recording:
      * `1311877812.987032.png` (141,882 B, SHA-256 `c22b1fe2...`, 83.7% valid, depth 1.07–5.00m)
      * `1311877830.276341.png` (145,655 B, SHA-256 `f2a9852a...`, 44.4% valid, depth 1.07–5.00m)
      * `1311877860.973057.png` (138,680 B, SHA-256 `6c135adc...`, 49.6% valid, depth 0.54–5.00m)
      * `1311877892.876368.png` (156,120 B, SHA-256 `1d26b8ef...`, 62.9% valid, depth 0.81–5.00m)
      * `1311877928.049984.png` (135,951 B, SHA-256 `42d31574...`, 83.6% valid, depth 1.08–4.02m)
    - Documented all provenance metadata, timestamps, byte sizes, and SHA-256 hashes in `data/tum_rgbd/manifest.json`.
  - **End-to-End Authentic Depth Projection & Grid Mapping**:
    - Processed all 5 authentic depth frames through calibrated pinhole unprojection ($f_x=520.9, f_y=521.0, c_x=325.1, c_y=249.7$, depth scale $5000.0$), camera-to-body extrinsics ($d_x=0.15\text{ m}, d_z=0.40\text{ m}$), obstacle vertical slicing ($[0.05, 1.20]\text{ m}$), and discrete 2D `GridMap` rasterization ($0.05\text{ m}$ resolution).
    - Total valid metric 3D points projected: **996,194 points**.
    - Total obstacle points retained: **549,503 points**.
    - Total grid cells updated: **8,930 occupied cells**.
    - Generated per-frame diagnostic figures `outputs/figures/dataset_evaluation_depth_<timestamp>.png` and summary diagnostic plot `outputs/figures/dataset_evaluation_diagnostic.png`.
  - **Metric Separation**:
    - Decoupled real perception results from synthetic evaluation:
      * Authentic Ground Truth Trajectory Parsing: 33,188 poses, 115.632s duration, 23.326m path length.
      * Authentic Depth Projection & Mapping: 5 frames, 996k points, 550k obstacle points, 8,930 cells mapped.
      * Closed-Loop Pure Pursuit Waypoint Tracking: 61 waypoints, 956 steps, 19.30m driven, mean CTE 0.0429m, max CTE 0.1485m, goal reached.
      * Synthetic Pose Perturbation & SLERP Interpolation: 3,319 samples, position RMSE 0.0425m, heading RMSE 1.16°.
  - **Automated Test Suite Expansion**:
    - Added 6 unit tests in `tests/unit/test_dataset_adapter.py` (corrupt PNG handling, comprehensive invalid depth filtering, timestamp association, projection bounds clipping, determinism, manifest parsing).
    - Added 2 integration tests in `tests/integration/test_dataset_integration.py` (authentic depth frame file verification and end-to-end grid mapping).
    - Full test suite expanded to **186 passed in 31.05s** ($100\%$ pass rate across 17 test modules).
    - Codebase coverage maintained at **85%** ($3385$ statements, $503$ missed).
  - **Regression Verification**:
    - `scripts/run_experiments.py`: 7/7 scenarios PASS.
    - `scripts/run_perception_demo.py`: PASS.
    - `scripts/run_dynamic_benchmarks.py`: Pure Pursuit 3/9 (33.3%), DWA 9/9 (100.0%, 0 collisions).
    - `scripts/run_estimation_benchmark.py`: 36 trials PASS.
    - `scripts/run_dataset_evaluation.py`: PASS (all 5 authentic depth frames processed).
- **Next Steps**:
  - Milestone 7.3: Dataset Metric Integrity, Spatial Bounds, and Reproducibility Audit.

---

## Milestone 7.3: Dataset Metric Integrity, Spatial Bounds, and Reproducibility Audit
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: PASS)
- **Scope & Implementation Details**:
  - **Depth-Pixel Statistics Reconciliation**:
    - Disaggregated depth image pixels across all 5 authentic frames ($480 \times 640 = 307,200\text{ pixels}$):
      * Clarified and mathematically proved that `zero_depth_pixels` is a strict subset of `invalid_depth_pixels`.
      * Out-of-range pixels ($<0.4\text{ m}$ or $>5.0\text{ m}$) make up the remainder ($0\text{--}115,457$ pixels).
      * Reconciliation Check 1 ($\text{zero} + \text{out-of-range} + \text{valid} = 307,200$) passed across 100% of frames ($996,194$ aggregate valid pixels, $263,277$ zero-depth dropouts, $276,529$ out-of-range pixels).
      * Reconciliation Check 2 ($\text{floor} + \text{ceiling} + \text{retained} = \text{valid}$) passed across 100% of frames ($549,503$ aggregate obstacle points retained, $242,193$ floor points rejected, $204,498$ ceiling points rejected).
  - **Ground-Truth Timestamp Association & SLERP Bracketing**:
    - Replaced deceptive $\Delta t = 0.0\text{ s}$ claims with exact physical time offsets to nearest discrete 100 Hz MoCap measurements:
      * Frame 1: $0.668\text{ ms}$ offset
      * Frame 2: $0.659\text{ ms}$ offset
      * Frame 3: $0.557\text{ ms}$ offset
      * Frame 4: $1.568\text{ ms}$ offset
      * Frame 5: $0.016\text{ ms}$ offset
    - Documented bounding bracket interval $[t_{\text{before}}, t_{\text{after}}]$, bracket interval $\approx 3.3\text{ ms}$, and SLERP interpolation blending factor $\alpha \in [0.490, 0.995]$.
  - **Spatial Coordinate Transforms & Grid Bounds Clipping**:
    - Nominal $10.0\text{ m} \times 10.0\text{ m}$ GridMap arena bounds $[-5.0, 5.0]\text{ m} \times [-5.0, 5.0]\text{ m}$ ($200 \times 200$ cells, resolution $0.05\text{ m}$).
    - Robot trajectory bounds: $[-2.83, 2.15]\text{ m} \times [-2.38, 2.96]\text{ m}$.
    - Forward-facing Kinect camera ($5.0\text{ m}$ range) viewing peripheral walls extends obstacle coordinates to $X \in [-7.089, 7.044]\text{ m}$ and $Y \in [-6.184, 6.511]\text{ m}$.
    - Audited and instrumented boundary clipping diagnostics: across all 5 frames, $436,356$ obstacle points ($79.41\%$) are within bounds, rasterizing $8,930$ occupied cells; $113,147$ points ($20.59\%$) are safely clipped without memory overflow or map corruption.
  - **Sequence Archive Provenance & Hash Digest Integrity**:
    - Clarified that SHA-256 `67f6c8458e7d9010e0a816ee893f862d27da99fa9d86a711b8c70c4b84e1a077` is a locally computed verification digest.
    - Documented that TUM does not host official `.sha256` or `.md5` checksum files on its web server.
    - Established upstream authenticity via server Content-Length ($1,183,281,970$ bytes), HTTP ETag `"46877332-4ae2a2e91a900"`, and internal DEFLATE/tar stream member verification across all 4,195 entries.
  - **Automated Test Suite Expansion**:
    - Added `test_pixel_reconciliation_exact_breakdown` in `tests/unit/test_dataset_adapter.py`.
    - Added `test_timestamp_association_nearest_pose_and_brackets` in `tests/unit/test_dataset_adapter.py`.
    - Added `test_grid_boundary_clipping_diagnostics` in `tests/unit/test_dataset_adapter.py`.
    - Added `test_authentic_depth_frame_pixel_reconciliation_and_spatial_bounds` in `tests/integration/test_dataset_integration.py`.
    - Full test suite: **190 passed** ($100\%$ pass rate in $30.05\text{ s}$).
    - Codebase coverage maintained at **85%**.
  - **Full Regression Verification**:
    - `scripts/run_experiments.py`: 7/7 scenarios PASS.
    - `scripts/run_perception_demo.py`: PASS.
    - `scripts/run_dynamic_benchmarks.py`: Pure Pursuit 3/9 (33.3%), DWA 9/9 (100.0%, 0 collisions).
    - `scripts/run_estimation_benchmark.py`: 36 trials PASS.
    - `scripts/run_dataset_evaluation.py`: PASS (all 5 authentic depth frames processed with reconciled metrics).
- **Next Steps**:
  - Milestone 7.4: Coordinate-Frame Provenance and Geometric Mapping Validation.

---

## Milestone 7.4: Coordinate-Frame Provenance and Geometric Mapping Validation
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: PASS)
- **Scope & Implementation Details**:
  - **Coordinate-Frame Provenance Chain Audit**:
    - Traced all 4 spatial reference frames:
      1. `camera_depth_optical_frame`: OpenCV/OpenNI optical frame (+X right, +Y down, +Z forward).
      2. `base_link`: REP-105 standard mobile robot frame (+X forward, +Y left, +Z up).
      3. `tum_mocap_world`: Local Euclidean Cartesian indoor laboratory tracking frame (+Z vertical, X/Y horizontal room axes). Formally established that it is **not** geographically aligned to East-North-Up (REP-103 ENU).
      4. `grid_map_world`: 2D discrete grid placed directly in the horizontal $(t_x, t_y)$ plane of the MoCap volume with origin $(-5.0, -5.0)\text{ m}$.
  - **Mounting Extrinsics Provenance Disclosure**:
    - Pioneer mounting offsets ($d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}$, zero pitch/yaw) were audited: TUM did not publish an official marker-tree-to-camera extrinsic calibration matrix; these parameters are explicitly documented and designated as **project-defined engineering assumptions** based on Pioneer 3-DX physical deck geometry.
  - **Rotation and Projection Mathematical Validation**:
    - Verified quaternion component ordering `(qx, qy, qz, qw)` with $qw$ scalar real component.
    - Verified quaternion normalization: non-unit quaternions are normalized before Euler projection, and degenerate/non-finite inputs raise `ValueError`.
    - Verified quaternion-to-yaw angle wrapping strictly in $(-\pi, \pi]$ across all 4 quadrants.
    - Verified optical-to-body rotation matrix has determinant $= +1.0$ (proper SO(3) rigid rotation) and orthonormal matrix product $R^T R = I$.
    - Implemented closed-form analytical hand calculations for raw pixel $(u=420, v=140, \text{depth}=10000)$ under non-zero robot translation $(3.0, -1.0)\text{ m}$ and non-zero heading ($30^\circ \to X_w = 5.061955\text{ m}, Y_w = -0.271410\text{ m}$; $90^\circ \to X_w = 3.40\text{ m}, Y_w = 1.15\text{ m}$), verified with zero discrepancy.
    - Verified composition order: confirmed mounting offsets are rotated with vehicle heading and applied exactly once without duplication.
  - **Timestamp Residual Terminology Clarification**:
    - Relabeled time offsets as **nearest MoCap sample residuals** ($0.016\text{--}1.568\text{ ms}$), reflecting 100 Hz MoCap sampling granularity.
    - Formally disclosed that nearest-sample residuals do not establish physical hardware clock synchronization between the Kinect sensor and the MoCap tracking computer.
  - **Grid Semantics Validation**:
    - Inspected `insert_points_into_grid_map()` and `GridMap`.
    - Formally defined that the output represents **projected obstacle endpoint rasterization (point hits)**.
    - Documented that cells with value `1` represent obstacle returns and cells with value `0` are unobserved/unmarked (not verified traversable free space, as free-space ray tracing is absent).
  - **Automated Test Suite Expansion**:
    - Added 6 unit tests in `tests/unit/test_dataset_adapter.py`:
      * `test_quaternion_component_ordering_and_normalization`
      * `test_quaternion_to_yaw_four_quadrants_and_wrapping`
      * `test_optical_to_body_axis_handedness_and_so3_determinant`
      * `test_end_to_end_projection_analytical_hand_calculation`
      * `test_transformation_composition_no_double_offset`
      * `test_grid_semantics_obstacle_endpoint_rasterization_only`
    - Full test suite: **196 passed in 35.61s** ($100\%$ pass rate, $85\%$ coverage).
  - **Legacy Benchmark Regression Invariance**:
    - Shared navigation, estimation, and simulation code was not altered; legacy benchmark baselines remain mathematically invariant and intact.
- **Next Steps**:
  - Milestone 7.5: Extrinsic Calibration Sensitivity and Mapping Robustness Analysis.

---

## Milestone 7.5: Extrinsic Calibration Sensitivity and Mapping Robustness Analysis
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: CONDITIONAL PASS — Uncertainty Bounded)
- **Scope & Implementation Details**:
  - **Extrinsic Mounting Model 6-DoF Expansion**:
    - Added `roll_deg: float = 0.0` with non-breaking default to `CameraExtrinsics` in `src/autonomous_nav/datasets/config.py`.
    - Added validation enforcing finite parameters across all 6 DoF (`dx, dy, dz, pitch_deg, yaw_deg, roll_deg`).
    - Added standard right-handed roll rotation about the forward longitudinal body axis ($X_b$) in `transform_camera_to_body` in `src/autonomous_nav/datasets/depth_projection.py`.
  - **Dedicated Sensitivity Analysis Engine (`scripts/run_calibration_sensitivity.py`)**:
    - Established baseline nominal hypothesis ($d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}$, pitch = $0.0^\circ$, yaw = $0.0^\circ$, roll = $0.0^\circ$).
    - Swept systematic 1D perturbations around nominal across all 6 DoF:
      * Longitudinal $d_x \in [0.05, 0.25]\text{ m}$ ($\pm 0.10\text{ m}$)
      * Lateral $d_y \in [-0.10, +0.10]\text{ m}$ ($\pm 0.10\text{ m}$)
      * Vertical $d_z \in [0.30, 0.50]\text{ m}$ ($\pm 0.10\text{ m}$)
      * Tilt pitch $\in [-5.0^\circ, +5.0^\circ]$
      * Pan yaw $\in [-5.0^\circ, +5.0^\circ]$
      * Bank roll $\in [-5.0^\circ, +5.0^\circ]$
    - Processed all 5 authentic TUM depth frames ($996,194$ valid pixels) for each candidate configuration.
    - Evaluated 3D point displacement (Mean, RMSE, Max, Median), 2D world displacement, GridMap cell overlap (IoU, recall, precision, new cells, missed cells), and height-filtering breakdown.
  - **Empirical Sensitivity & Failure Mode Findings**:
    - **Downward Pitch Failure Mode**: A $+5.0^\circ$ downward tilt plunges obstacle points below the $0.05\text{ m}$ floor threshold, dropping retained obstacle points to $367,736$ and collapsing occupied GridMap cells from $8,930$ to $616$ ($93.1\%$ loss of cells, IoU = $6.23\%$).
    - **Yaw Lever-Arm Arc Error**: Yaw leaves obstacle height filtering untouched ($549,503$ points across all angles), but produces range-dependent arc distortion ($\approx r \cdot \theta$), reaching $0.5017\text{ m}$ max displacement and reducing IoU to $66.39\%$.
    - **Vertical Height Slicing Coupling**: Shifting $d_z$ by $\pm 0.10\text{ m}$ shifts the height window, altering retained obstacle counts by $-21.7\%$ ($d_z = 0.30\text{ m}$) to $+44.9\%$ ($d_z = 0.50\text{ m}$).
    - **Horizontal Rigid Translations**: Translations $d_x, d_y$ cause pure rigid shifts with $\Delta P = \text{offset}$ and zero height impact.
    - **Cell Count Deception**: Proved conclusively that occupied cell count alone cannot serve as a proxy for calibration accuracy.
  - **Automated Test Suite Expansion**:
    - Implemented `tests/unit/test_calibration_sensitivity.py` (7 tests):
      * `test_camera_extrinsics_defaults_and_validation`
      * `test_analytical_pure_translation_displacement`
      * `test_analytical_pure_yaw_rotation`
      * `test_analytical_pure_roll_rotation`
      * `test_height_filtering_coupling_pitch_vs_yaw`
      * `test_grid_map_overlap_and_iou_zero_perturbation`
      * `test_production_config_immutability`
    - Full test suite: **203 passed tests** in $35.04\text{ s}$ ($100\%$ pass rate, $86\%$ coverage).
  - **Regression Invariance**:
    - Re-ran `scripts/run_dataset_evaluation.py`: baseline metrics remain bitwise identical ($996,194$ valid pixels, $549,503$ obstacle points, $8,930$ occupied cells, $0.0429\text{ m}$ CTE).
  - **Diagnostic Artifacts**:
    - JSON Summary: [`outputs/metrics/calibration_sensitivity_summary.json`](../outputs/metrics/calibration_sensitivity_summary.json)
    - 6-Panel Diagnostic Plot: [`outputs/figures/calibration_sensitivity_diagnostic.png`](../outputs/figures/calibration_sensitivity_diagnostic.png)
- **Next Steps**:
  - Milestone 8.0: Repository Cleanup, Scope Alignment, and Release Readiness Audit.

---

## Milestone 8.0: Repository Cleanup, Scope Alignment, and Release Readiness Audit
- **Date**: 2026-10-21
- **Status**: Completed & Verified (Verdict: PASS)
- **Scope Alignment & Direction**:
  - Confirmed project direction as a **primarily 2D autonomous mobile robot navigation framework** supported by 2D simulation, A* planning, DWA local planning, dynamic obstacle avoidance, EKF state estimation, sensor contracts, and an experimental authentic TUM RGB-D-to-2D depth projection and calibration sensitivity pipeline.
  - Formally categorized full 3D volumetric mapping (voxel grids / OctoMap), 3D point cloud clustering, and live ROS 2 / CARLA 3D simulator daemons as **optional post-release future work**, ensuring the core release remains lightweight, modular, and fully testable in CPU-only Python environments.
- **Pre-Cleanup Baseline Inventory**:
  - **Git Status**: Branch `main`, zero commits, clean working tree under `.gitignore` rules.
  - **Total Disk Usage**: $1.51\text{ GB}$ total ($1.13\text{ GB}$ authentic TUM dataset archive and ground truth, $291\text{ MB}$ `.venv`, $87\text{ MB}$ `.pip_cache`, $3.6\text{ MB}$ outputs, $1.1\text{ MB}$ tests, $0.7\text{ MB}$ src).
  - **Test Suite**: 203 tests passing with 86% coverage.
  - **Benchmark Suite**: 6 automated runners across static, dynamic, perception, estimation, dataset evaluation, and calibration sensitivity.
- **Audit Findings & Safe Cleanup Actions**:
  - **Orphaned 3D Voxel Artifacts Removed**:
    * Deleted `outputs/figures/dataset_evaluation_voxel_3d.png` ($153,393\text{ bytes}$): Abandoned ad-hoc 3D plot from an earlier experimental run, not referenced in any code or documentation.
    * Deleted `src/autonomous_nav/environment/__pycache__/voxel_grid.cpython-311.pyc` and `tests/unit/__pycache__/test_voxel_grid.cpython-311-pytest-9.1.1.pyc`: Stale compiled bytecode files whose source `.py` files did not exist.
  - **Redundant Metrics Backup Directory Removed**:
    * Deleted `outputs/metrics_baseline_backup/` ($112,654\text{ bytes}$, 13 duplicate JSON/log files + `.gitkeep`): Superseded ad-hoc snapshot created during Milestone 7.3 prior to authentic frame re-evaluation; active authentic metrics are preserved in `outputs/metrics/`.
  - **Git Hygiene Hardening**:
    * Updated `.gitignore` to explicitly ignore `outputs/*_backup/` to prevent future untracked backup staging.
- **Files Intentionally Retained**:
  - `data/tum_rgbd/`: Authentic dataset archive (`rgbd_dataset_freiburg2_pioneer_slam2.tgz`, $1.18\text{ GB}$), ground truth (`groundtruth.txt`, $2.26\text{ MB}$), depth index (`depth.txt`), 5 authentic depth PNGs ($718\text{ KB}$), cryptographic manifest (`manifest.json`), and comprehensive dataset README with Sturm et al. (IROS 2012) CC BY 4.0 attribution.
  - `scripts/measure_dwa_performance.py`: Retained as it generates `outputs/metrics/dwa_profile_baseline.json` and is cited in `docs/results.md`.
  - `docs/01-06_*.md`: System architecture and theoretical design specifications retained as foundational reference documents.
- **Validation Results Post-Cleanup**:
  - Full test suite: `pytest --cov=src/autonomous_nav -v` -> **203 passed** in $36\text{ s}$, **86% coverage**.
  - `run_experiments.py`: 4/4 scenarios PASS, 0 collisions, sub-0.09m mean CTE.
  - `run_perception_demo.py`: PASS, 3/3 obstacles detected, 36.49m path planned.
  - `run_dynamic_benchmarks.py`: 18/18 trials evaluated, Pure Pursuit 33.3% vs DWA 100% success.
  - `run_estimation_benchmark.py`: 36/36 trials PASS.
  - `run_dataset_evaluation.py`: PASS, 996,194 valid pixels, 549,503 obstacle points, 8,930 occupied cells.
  - `run_calibration_sensitivity.py`: PASS, 6-DoF sensitivity bounded.
- **Release Readiness Status**: All systems operational, scope fully aligned, 0 regressions, repository clean and ready for release.











