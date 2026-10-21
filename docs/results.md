# Benchmark Results & Performance Evaluation

## 1. Verified Benchmark Scorecard

The baseline autonomous navigation pipeline was audited and benchmarked across four standard environments using the project-local virtual environment ([`.venv\Scripts\python.exe scripts/run_experiments.py`](../scripts/run_experiments.py)).

All four scenarios achieved a **$100\%$ task completion rate** with **zero collisions**, strict corner-clearance enforcement, and sub-$0.09\text{ m}$ average orthogonal cross-track error.

### Quantitative Summary Table

| Benchmark Scenario | Outcome | Planned Path ($m$) | Driven Trajectory ($m$) | Path Efficiency | Mean CTE ($m$) | Max CTE ($m$) | Sim Duration ($s$) | Wall-Clock Compute ($s$) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`empty_field`** | **PASS** | $26.21$ | $25.79$ | $1.016$ | $0.052$ | $0.354$ | $18.90$ | $0.0049$ |
| **`simple_obstacles`** | **PASS** | $30.73$ | $29.50$ | $1.042$ | $0.075$ | $0.354$ | $21.60$ | $0.0293$ |
| **`corridor_s_bend`** | **PASS** | $30.73$ | $29.27$ | $1.050$ | $0.086$ | $0.354$ | $21.50$ | $0.0783$ |
| **`urban_grid`** | **PASS** | $35.00$ | $34.79$ | $1.006$ | $0.016$ | $0.355$ | $24.70$ | $0.0791$ |

*Artifact Reference: Raw serialized JSON records are persisted in [`outputs/metrics/benchmark_results.json`](../outputs/metrics/benchmark_results.json).*

---

## 2. Rigorous Metric Definitions

1. **Planned Path Length ($m$)**:
   $$\sum_{i=1}^{N-1} \sqrt{(w_{x, i} - w_{x, i-1})^2 + (w_{y, i} - w_{y, i-1})^2}$$
   Cumulative length connecting discrete waypoints produced by the 8-connected A* planner.
2. **Driven Trajectory Length ($m$)**:
   $$\sum_{k=1}^{M-1} \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$
   Actual physical distance traveled by the kinematic robot model across all simulation steps.
3. **Simulated Duration ($s$)**:
   $$T_{\text{sim}} = N_{\text{steps}} \times \Delta t \quad (\Delta t = 0.1\text{ s})$$
   Represents virtual vehicle physics time, distinct from CPU execution time.
4. **Wall-Clock Compute Runtime ($s$)**:
   High-resolution CPU time measured via `time.perf_counter()`, demonstrating that all scenarios solve and simulate in under $80\text{ ms}$ of actual computation.
5. **Orthogonal Cross-Track Error (CTE, $m$)**:
   Minimum Euclidean distance from the vehicle position to the closest continuous line segment of the planned reference route:
   $$\text{CTE}(P) = \min_{i \in [0, N-2]} \text{dist}(P, \text{segment}(W_i, W_{i+1}))$$
   This eliminates vertex-discretization sampling bias, accurately reflecting lateral path-following deviation.
6. **Path Efficiency Ratio**:
   $$\eta = \frac{\text{Planned Path Length}}{\text{Driven Trajectory Length}}$$
   Ratios $\ge 1.0$ indicate smooth trajectory shortcuts around discrete grid corners.

---

## 3. Analysis & Verification of Discrepancies

### 3.1 Why Driven Distance is Shorter Than Planned Path
In the audited runs, the driven trajectory is systematically shorter than the planned grid path by $0.21\text{--}1.46\text{ m}$. Two physical factors account for this discrepancy:
1. **Terminal Goal Tolerance ($0.5\text{ m}$)**: The simulation successfully terminates as soon as the vehicle reaches within the configured goal acceptance radius ($0.5\text{ m}$) of the destination. Stopping $\approx 0.48\text{ m}$ before the exact cell center saves $\approx 0.48\text{ m}$ of travel.
2. **Kinematic Corner Smoothing (Triangle Inequality)**: A* plans piecewise orthogonal and diagonal grid segments with sharp $45^\circ$ or $90^\circ$ corners. The Pure Pursuit controller tracks a lookahead point $L_{\text{fd}} = 0.8\text{ m}$ ahead, creating a smooth arc that cuts inside the sharp vertex. By the triangle inequality ($d(A, C) \le d(A, B) + d(B, C)$), the smooth chord is strictly shorter than the discrete zigzag path.

### 3.2 Corner-Cutting Prevention Audit
Following the Milestone 1.1 audit, A* diagonal neighbor expansion enforces strict corner clearance:
$$\text{Blocked if } \mathbf{M}(cx + dx, cy) = 1 \lor \mathbf{M}(cx, cy + dy) = 1$$
This ensures diagonal paths never clip the corners of obstacle cells, maintaining safety margins for vehicles with non-zero radius.

---

## 4. Vision-Based Perception & Closed-Loop Planning Evaluation

Milestone 2 introduced the OpenCV classical vision pipeline, evaluated using [`scripts/run_perception_demo.py`](../scripts/run_perception_demo.py).

The demonstration generates a synthetic multi-obstacle overhead observation ($640 \times 480\text{ px}$), extracts obstacle contours, projects detections into continuous metric world coordinates via `OrthographicCamera`, updates the discrete `GridMap` using `MapAdapter`, and plans a collision-free A* trajectory from $(2.0, 2.0)\text{ m}$ to $(27.0, 17.0)\text{ m}$.

### 4.1 Obstacle Localization Accuracy

| Obstacle ID | Ground Truth Shape | Ground Truth Center ($m$) | Detected Center ($m$) | Centroid Error ($m$) | Area ($\text{px}$) | Bounding Box $(u, v, w, h)$ |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **#1** | Rectangle ($4.0 \times 6.0\text{ m}$) | $(22.0, 6.0)$ | $(22.01, 6.00)$ | **$0.01\text{ m}$** | $12,238$ | $(427, 264, 86, 145)$ |
| **#2** | Circle ($R = 2.5\text{ m}$) | $(16.0, 11.0)$ | $(15.98, 11.00)$ | **$0.02\text{ m}$** | $10,024$ | $(285, 160, 113, 113)$ |
| **#3** | Rectangle ($4.0 \times 10.0\text{ m}$) | $(10.0, 10.0)$ | $(10.01, 10.00)$ | **$0.01\text{ m}$** | $20,398$ | $(171, 120, 86, 241)$ |

### 4.2 End-to-End Perception-to-Planning Summary

- **Total Detected Obstacles**: $3 / 3$ ($100\%$ detection rate, $0$ false positives, $0$ false negatives).
- **Occupancy Grid Mapping**: $402$ discrete cells ($0.5\text{ m}$ resolution) marked occupied in the $60 \times 40$ grid map using full-contour footprint rasterization.
- **Path Search Outcome**: A* found optimal obstacle-avoiding trajectory of length **$36.49\text{ m}$**.
- **Execution Performance**: Image preprocessing, Otsu thresholding, contour extraction, coordinate projection, and map update solved in **$< 12\text{ ms}$**.
- **Visual Artifact**: Saved publication 4-panel diagnostic plot to [`outputs/figures/perception_demo.png`](../outputs/figures/perception_demo.png) displaying Raw Observation, Preprocessed Grayscale, Detected Contours/Centroids, and Perceived Grid Map with A* Planned Path.
- **Metrics Artifact**: Serialized evaluation summary to [`outputs/metrics/perception_demo_summary.json`](../outputs/metrics/perception_demo_summary.json).

---

## 5. Milestone 2.1 Perception Geometry & Safety Audit

The Milestone 2.1 audit thoroughly verified coordinate transformations, footprint fidelity, boundary integrity, and collision avoidance safety:

### 5.1 Footprint Overlap & Coverage (Synthetic Fixture)

| Metric | Measured Value | Requirement / Benchmark | Status |
| :--- | :--- | :--- | :--- |
| **Footprint Intersection-over-Union (IoU)** | **$0.900$** ($90.0\%$) | $\ge 0.88$ | **PASS** |
| **False Negatives (Under-estimated Danger)** | **$0\text{ cells}$** | $0\text{ cells}$ | **PASS** |
| **False Positives (Safety Buffer at Bounds)** | $9\text{ cells}$ | Discretization margin ($\le 12$) | **PASS** |
| **Centroid Localization Error** | **$\le 0.02\text{ m}$** | $< 0.05\text{ m}$ | **PASS** |
| **Detection Precision & Recall** | **$100\% / 100\%$** | $100\%$ on synthetic fixtures | **PASS** |

### 5.2 Geometric Transformation & Boundary Safety Audit

- **Orthographic Inversion & Scale**: Verified 4 image corners, vertical axis inversion ($v=0 \to y=y_{\max}$), and metric scale preservation ($s_x = 21.33\text{ px/m}, s_y = 24.00\text{ px/m}$).
- **Planar Perspective Homography**: Verified analytical derivation against 4-point empirical DLT estimation ($< 4\times 10^{-4}$ max matrix discrepancy). Handled degenerate/collinear inputs, singular matrices, and horizon clipping safely.
- **Off-Grid Obstacles**: Tested obstacles located partially or completely outside the grid map boundaries. Confirmed $0$ artificial boundary walls and $0$ false occupied cells on internal routes.
- **Clearance Invariant**: Perceived obstacles inflated with $0.6\text{ m}$ robot buffer. All planned A* waypoints verified to maintain continuous Euclidean clearance $\ge 0.5\text{ m}$ from raw obstacle boundaries.
- **Unreachable Goal Reporting**: Enclosed goals safely trigger `path is None` without hanging or throwing unhandled exceptions.
- **Total Automated Test Suite**: Expanded from 47 to **57 passing tests** ($100\%$ pass rate in $0.38\text{ s}$).

---

## 6. Milestone 3: Interactive 2D Pygame Simulation Evaluation

Milestone 3 introduced the interactive 2D simulation environment implemented in [`src/autonomous_nav/visualization/animate.py`](../src/autonomous_nav/visualization/animate.py), launched via [`scripts/run_interactive.py`](../scripts/run_interactive.py) or `autonomous-nav --interactive`.

### 6.1 Interactive Capabilities & Verification

| Feature | Implementation Mechanism | Validation Status |
| :--- | :--- | :--- |
| **Real-Time 2D Rendering** | Pygame double-buffered display surface ($1100 \times 720\text{ px}$) measured at $3.18\text{ ms}$ render overhead per frame (unthrottled capacity $\approx 315\text{ FPS}$). | **VERIFIED** |
| **Dual Viewport Layout** | Dedicated $780 \times 680\text{ px}$ map canvas (left) and $280 \times 680\text{ px}$ telemetry sidebar (right). | **VERIFIED** |
| **Coordinate Transforms** | Bidirectional metric $(x, y) \leftrightarrow$ screen $(u, v)$ with vertical inversion ($v = 0 \to y = y_{\max}$) and aspect-ratio preservation. | **VERIFIED** |
| **Telemetry HUD Dashboard** | Live telemetry displaying Simulation State, Virtual Sim Time ($T_{\text{sim}}$), Step Budget ($k / N_{\max}$), Linear Velocity ($v$), Angular Rate ($\omega$), CTE ($m$), Driven & Planned Distance ($m$), and Goal Distance ($m$). | **VERIFIED** |
| **Dynamic Obstacle Mutation** | Left-click on map toggles grid cell occupancy; immediately triggers `AStarPlanner.replan()` from current robot pose to goal without halting simulation loop. | **VERIFIED** |
| **Interactive Controls** | `[SPACE]` pause/resume, `[S]` / `[->]` single-frame step, `[R]` episode reset, `[1-4]` scenario jump, `[N]` next scenario cycle, `[ESC]/[Q]` exit. | **VERIFIED** |
| **Headless CI/CD Support** | Fully automated headless execution with `SDL_VIDEODRIVER="dummy"` and `SDL_AUDIODRIVER="dummy"`; supports programmatic frame bounds and off-screen snapshot export via `save_screenshot(path)`. | **VERIFIED** |

### 6.2 Visual Snapshot Artifact

- **Saved Figure**: [`outputs/figures/interactive_sim_screenshot.png`](../outputs/figures/interactive_sim_screenshot.png) (42.5 KB, $1100 \times 720\text{ px}$, 3 RGB channels).
- **Inspection Summary**: Verified high-contrast dark theme (#1e1e24 background), crisp grid rendering, obstacle cells with dilation halos (204,258 obstacle pixels), start (124 green pixels) and goal (136 crimson pixels) bullseye markers, planned A* path (315,277 path/marker pixels), real-time robot body ($R=0.4\text{ m}$, 172 yellow pixels) with heading arrow and lookahead target, driven trajectory (golden trail), and right-hand telemetry card with active KPI metrics (12,947 text pixels).

### 6.3 Empirical Performance Measurements

Frame rendering and step loop performance were measured using project-local interpreter `.venv\Scripts\python.exe` with high-resolution `time.perf_counter()` over 200 consecutive benchmark iterations on `simple_obstacles`:

| Performance Metric | Measured Value | Budget @ 30 FPS | Budget @ 60 FPS | Frame-budget usage |
| :--- | :--- | :--- | :--- | :--- |
| **Pure Render Time ($T_{\text{render}}$)** | **$3.18\text{ ms/frame}$** | $33.3\text{ ms}$ | $16.7\text{ ms}$ | $9.5\% / 19.0\%$ |
| **Full Loop Time (Physics + Render)** | **$3.87\text{ ms/frame}$** | $33.3\text{ ms}$ | $16.7\text{ ms}$ | $11.6\% / 23.2\%$ |
| **Unthrottled FPS Capacity** | **$258.5\text{ FPS}$** | $\ge 30\text{ FPS}$ | $\ge 60\text{ FPS}$ | Large headroom |

*Measurement Methodology & Limitations*: Benchmarks were executed headlessly using `SDL_VIDEODRIVER="dummy"`. The recorded times measure CPU surface rasterization, kinematics integration, and collision checking. Physical display VSync latencies, OS window compositing, and GPU blitting overhead in physical monitor environments are not modeled by the dummy SDL driver.

---

## 7. Milestone 3.1: Independent Verification & Hardening Results

Milestone 3.1 audited the simulation engine, interactive visualizer, and dynamic replanning failure modes, expanding coverage with 9 targeted edge-case unit tests.

### 7.1 Dynamic Replanning & Failure Mode Matrix

| Edge Case Test | Condition / Invariant Tested | Outcome | Status |
| :--- | :--- | :--- | :--- |
| **Obstacle Forces Detour** | Rectangular obstacle placed across direct line-of-sight. | Detour path found; no waypoints intersect obstacle. | **PASS** |
| **Route Blocked Completely** | Solid wall placed across entire corridor. | `replan()` returns `False`; path cleared (`[]`); status `PLANNING_FAILURE`. | **PASS** |
| **Obstacle on Goal Cell** | Goal cell toggled to occupied. | Planning safely aborts; stale path cleared; status `PLANNING_FAILURE`. | **PASS** |
| **Obstacle at Robot Pose** | Cell containing robot toggled to occupied. | Collision detected; status `COLLISION`; clearing obstacle recovers safe state. | **PASS** |
| **Failure Recovery** | Wall blocking corridor removed. | `replan()` restores valid path; `is_done` reset to `False`; stepping advances. | **PASS** |
| **Stale Route Prevention** | Planning failure triggered while path displayed. | `planned_path` cleared to `[]`; renderer draws empty path without error. | **PASS** |
| **Status Badge Synchronization** | Engine termination reasons compared to HUD badge. | 100% agreement across `RUNNING`, `PAUSED`, `COLLISION`, `GOAL_REACHED`, `TIMEOUT`, `PLANNING_FAILURE`. | **PASS** |
| **Accidental Step Prevention** | Repeated pause updates, scenario switching, reset. | 0 unintended steps taken; reset returns step counter to 0. | **PASS** |
| **Rendering Invariance** | 5 repeated `render()` calls + screenshot export. | Exact equality across $x, y, \theta, v, T_{\text{sim}}, \text{steps}$, path, and trajectory. | **PASS** |

### 7.2 Total Test Suite Status

- Total passing automated tests expanded from 66 to **75 passing tests** across 8 test suites in $2.44\text{ s}$ ($100\%$ pass rate).

---

## 8. Milestone 4: Dynamic Obstacles & Local Collision-Aware Navigation Results

Milestone 4 evaluated local collision avoidance around dynamic moving obstacles using the Dynamic Window Approach (DWA local planner) compared against the baseline Pure Pursuit trajectory tracker.

The benchmark suite was executed via [`.venv\Scripts\python.exe scripts/run_dynamic_benchmarks.py`](../scripts/run_dynamic_benchmarks.py) across three dynamic environments:
1. **`dynamic_crossing`**: Two orthogonal moving obstacles crossing the vehicle's nominal path.
2. **`dynamic_corridor`**: Constrained 8-meter corridor with an obstacle crossing diagonally between work areas.
3. **`dynamic_chokepoint`**: Central doorway periodically blocked and reopened by a patrolling obstacle.

### 8.1 Comparative Benchmark Matrix: Pure Pursuit vs DWA Local Planner

| Scenario | Navigation Mode | Outcome | Dynamic Collision | Driven Dist ($m$) | Planned Path ($m$) | Mean CTE ($m$) | Sim Duration ($s$) | Wall-Clock Compute ($s$) | DWA Failures |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`dynamic_crossing`** | **Pure Pursuit** | **COLLISION** | **YES** (Step 83) | $11.40$ | $25.00$ | $0.047$ | $8.3$ | $0.0074$ | N/A |
| **`dynamic_crossing`** | **DWA Local** | **SUCCESS** | **NO** | $24.77$ | $25.00$ | $0.063$ | $21.7$ | $5.8087$ | **0** |
| **`dynamic_corridor`** | **Pure Pursuit** | **COLLISION** | **YES** (Step 92) | $12.75$ | $29.00$ | $0.042$ | $9.2$ | $0.0519$ | N/A |
| **`dynamic_corridor`** | **DWA Local** | **SUCCESS** | **NO** | $28.77$ | $29.00$ | $0.048$ | $25.7$ | $7.7958$ | **0** |
| **`dynamic_chokepoint`** | **Pure Pursuit** | **COLLISION** | **YES** (Step 74) | $10.05$ | $22.00$ | $0.052$ | $7.4$ | $0.0150$ | N/A |
| **`dynamic_chokepoint`** | **DWA Local** | **SUCCESS** | **NO** | $21.77$ | $22.00$ | $0.061$ | $19.8$ | $5.2961$ | **0** |

*Artifact Reference: Benchmark results serialized in [`outputs/metrics/dynamic_benchmark_results.json`](../outputs/metrics/dynamic_benchmark_results.json).*

### 8.2 Key Findings & Analysis

1. **Failure Mode of Baseline Pure Pursuit**:
   In all dynamic scenarios, Pure Pursuit blindly tracks the static global path without evaluating moving obstacles. This causes guaranteed collisions within $7.4\text{--}9.2\text{ s}$ ($0\%$ success rate on dynamic scenarios).
2. **Safety and Adaptability of DWA**:
   DWA achieves **$100\%$ task completion rate** ($3/3$ goals reached) with **zero collisions** and **zero emergency stops/failures** across all three dynamic scenarios.
3. **Behavioral Characteristics**:
   - In `dynamic_crossing`, DWA detects impending trajectory intersections and decelerates to yield to the first moving obstacle, then accelerates through the open gap.
   - In `dynamic_corridor`, DWA executes a smooth lateral detour into the open lane while the dynamic entity traverses diagonally, returning to the nominal corridor centerline once clear.
   - In `dynamic_chokepoint`, DWA approaches the doorway, pauses safely outside the doorway collision envelope while the obstacle occludes the portal, and passes through immediately upon clearance.
4. **Computational Performance**:
   DWA step evaluation takes $\approx 18\text{--}25\text{ ms/step}$ ($\ge 40\text{ FPS}$), well within real-time robotics control loop deadlines.

### 8.3 Visual & Metric Artifacts

- **Dynamic Interactive Visualizer Screenshots**:
  - [`outputs/figures/dynamic_crossing_dwa_interactive.png`](../outputs/figures/dynamic_crossing_dwa_interactive.png)
  - [`outputs/figures/dynamic_corridor_dwa_interactive.png`](../outputs/figures/dynamic_corridor_dwa_interactive.png)
  - [`outputs/figures/dynamic_chokepoint_dwa_interactive.png`](../outputs/figures/dynamic_chokepoint_dwa_interactive.png)
- **Trajectory Plots**:
  - [`outputs/figures/dynamic_dynamic_crossing_dwa.png`](../outputs/figures/dynamic_dynamic_crossing_dwa.png)
  - [`outputs/figures/dynamic_dynamic_corridor_dwa.png`](../outputs/figures/dynamic_dynamic_corridor_dwa.png)
  - [`outputs/figures/dynamic_dynamic_chokepoint_dwa.png`](../outputs/figures/dynamic_dynamic_chokepoint_dwa.png)
- **Test Suite Expansion**:
  Expanded to **96 automated unit and integration tests** ($100\%$ passing in $20.2\text{ s}$), achieving **$82\%$ overall codebase coverage** and **$98\%$ coverage** on the DWA planner module.

---

## 9. Milestone 4.1: DWA Performance, Benchmark Reproducibility & Safety Audit

Milestone 4.1 reconciled multi-second DWA simulation times with per-step latency, eliminated temporal collision tunneling via continuous swept-volume checking, optimized the inner collision-checking loops with NumPy bounding-box fast-paths, and expanded the comparative evaluation to multi-trial speed variations.

### 9.1 Timing Methodology & Reconciliation

A comprehensive profiling audit was conducted to reconcile the multi-second episode runtimes with per-step latency:

1. **Timing Boundaries**:
   - **`dwa_latency`**: High-resolution `time.perf_counter()` duration measuring exclusively the execution of [`DWAPlanner.compute_command()`](../src/autonomous_nav/navigation/dwa.py).
   - **`step_latency`**: Complete duration of [`SimulationEngine.step()`](../src/autonomous_nav/simulation/engine.py), comprising target extraction, DWA planning, unicycle RK2 integration, dynamic obstacle stepping, and static/dynamic collision checks.
   - **`sim_loop_time`**: Cumulative wall-clock time spent actively iterating the simulation physics loop.
   - **`total_wall_clock`**: Complete end-to-end execution including `engine.reset()` and A* global planning.
   - **Excluded from Simulation Loop**: Matplotlib visualization and figure disk export (`plot_simulation_run()`), which takes $\approx 1.5\text{--}2.0\text{ s}$ separately.
2. **Reconciliation Analysis**:
   In Milestone 4, a run of 217 steps taking $\approx 5.8\text{ s}$ represents an average of $26.8\text{ ms/step}$ over $21.7\text{ s}$ of virtual simulated time. Rather than running slower than real-time, the simulation executes at $\approx 4\times$ faster than real-time physics. DWA planning accounts for **$> 95\%$** of the per-step computation.

### 9.2 Before-and-After Optimization Benchmark

Profiling revealed that $71\%$ of DWA execution time was consumed by redundant bounds checking inside `check_disc_collision`, repetitive dynamic obstacle trajectory prediction across candidate trajectories, and exhaustive cell searches in `distance_to_nearest_obstacle`.

Two mathematically rigorous optimizations were implemented without altering collision margins or kinematics:
1. **NumPy Bounding-Box Slice Fast-Paths**: Early-exiting disc collision and distance queries in open regions when `not np.any(sub_grid)` is True, preserving exact circle-to-AABB distance when obstacles exist.
2. **Prediction Horizon Caching**: Precomputing dynamic obstacle positions once per control cycle rather than re-evaluating for every candidate trajectory ($165\times$ fewer projection calls).

#### Empirical Before-and-After Measurements

| Dynamic Scenario | Metric / Phase | Pre-Optimization Baseline | Post-Optimization Audited | Speedup Factor |
| :--- | :--- | :--- | :--- | :--- |
| **`dynamic_crossing`** | DWA Latency (Mean / Med / p95 / Max) | $23.36 / 23.02 / 25.42 / 32.94\text{ ms}$ | **$9.59 / 9.20 / 12.08 / 14.16\text{ ms}$** | **$2.43\times$** |
| | Complete Step Latency (Mean / p95 / Max) | $23.29 / 25.45 / 33.02\text{ ms}$ | **$9.57 / 12.12 / 14.21\text{ ms}$** | **$2.43\times$** |
| | Sim Loop Duration ($21.9\text{ s}$ sim) | $5.084\text{ s}$ | **$2.111\text{ s}$** | **$2.41\times$** |
| | Wall-to-Sim Duration Ratio ($T_{\text{wall}} / T_{\text{sim}}$) | $0.234\times$ ($4.3\times$ real-time) | **$0.096\times$ ($10.4\times$ real-time)** | — |
| **`dynamic_corridor`** | DWA Latency (Mean / Med / p95 / Max) | $23.49 / 22.28 / 28.73 / 56.57\text{ ms}$ | **$8.00 / 7.93 / 8.47 / 10.84\text{ ms}$** | **$2.94\times$** |
| | Complete Step Latency (Mean / p95 / Max) | $23.44 / 28.78 / 56.63\text{ ms}$ | **$7.99 / 8.50 / 10.88\text{ ms}$** | **$2.93\times$** |
| | Sim Loop Duration ($25.7\text{ s}$ sim) | $6.089\text{ s}$ | **$2.102\text{ s}$** | **$2.90\times$** |
| | Wall-to-Sim Duration Ratio ($T_{\text{wall}} / T_{\text{sim}}$) | $0.237\times$ ($4.2\times$ real-time) | **$0.082\times$ ($12.2\times$ real-time)** | — |
| **`dynamic_chokepoint`**| DWA Latency (Mean / Med / p95 / Max) | $26.85 / 23.23 / 46.88 / 135.08\text{ ms}$| **$8.21 / 7.95 / 9.74 / 10.92\text{ ms}$** | **$3.27\times$** |
| | Complete Step Latency (Mean / p95 / Max) | $26.76 / 46.86 / 135.14\text{ ms}$ | **$8.19 / 9.76 / 10.96\text{ ms}$** | **$3.27\times$** |
| | Sim Loop Duration ($19.8\text{ s}$ sim) | $5.336\text{ s}$ | **$1.637\text{ s}$** | **$3.26\times$** |
| | Wall-to-Sim Duration Ratio ($T_{\text{wall}} / T_{\text{sim}}$) | $0.269\times$ ($3.7\times$ real-time) | **$0.083\times$ ($12.0\times$ real-time)** | — |

*Measurement Methodology*: Measured using `.venv\Scripts\python.exe scripts/measure_dwa_performance.py` on the project root platform using `time.perf_counter()`.

### 9.3 Temporal Collision Checking & Tunneling Elimination

To eliminate temporal aliasing where moving obstacles could penetrate the robot footprint between discrete time samples ($t_{k-1} \to t_k$), [`CollisionChecker.relative_continuous_distance()`](../src/autonomous_nav/simulation/collision.py#L98-L123) was implemented.
- **Formulation**: Converts relative motion into a linear displacement segment $D(t) = P(t) - O(t)$ from $D_{k-1}$ to $D_k$. Computes exact orthogonal projection from the difference space origin $(0, 0)$ to the segment.
- **Verification**: Verified via [`tests/unit/test_temporal_collision.py`](../tests/unit/test_temporal_collision.py) (8 unit tests):
  - Fast-crossing tunneling detection ($v_{\text{rel}} = 16\text{ m/s}$) where discrete endpoints miss collision ($d > 0.94\text{ m}$) while swept check detects direct hit ($d = 0.0\text{ m}$).
  - Close-pass and near-tangent clearance accuracy.
  - Head-on vs. receding obstacle distance queries.
  - Emergency braking when all candidate arcs are blocked.

### 9.4 Reproducible Multi-Trial Dynamic Benchmark Suite

To avoid claiming universal performance from a single trial, the benchmark suite in [`scripts/run_dynamic_benchmarks.py`](../scripts/run_dynamic_benchmarks.py) was expanded across **18 total deterministic trials** (9 trials per mode across nominal, $-20\%$ slower, and $+20\%$ faster obstacle velocities):

| Scenario | Variation | Pure Pursuit Outcome | Pure Pursuit Dyn Coll | DWA Outcome | DWA Dyn Coll | DWA Failures | DWA Step Latency | DWA Sim Loop |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`dynamic_crossing`** | Nominal ($1.0\times$) | **COLLISION** ($8.3\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $10.3\text{ ms}$ | $2.270\text{ s}$ |
| | Slower ($0.8\times$) | **GOAL_REACHED** ($18.0\text{ s}$)* | **NO** | **GOAL_REACHED** | **NO** | 0 | $10.2\text{ ms}$ | $2.180\text{ s}$ |
| | Faster ($1.2\times$) | **COLLISION** ($8.2\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $10.0\text{ ms}$ | $2.026\text{ s}$ |
| **`dynamic_corridor`** | Nominal ($1.0\times$) | **COLLISION** ($9.2\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $9.0\text{ ms}$ | $2.325\text{ s}$ |
| | Slower ($0.8\times$) | **COLLISION** ($8.8\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $8.9\text{ ms}$ | $2.663\text{ s}$ |
| | Faster ($1.2\times$) | **GOAL_REACHED** ($20.7\text{ s}$)* | **NO** | **GOAL_REACHED** | **NO** | 0 | $16.7\text{ ms}$ | $3.902\text{ s}$ |
| **`dynamic_chokepoint`**| Nominal ($1.0\times$) | **COLLISION** ($7.4\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $11.9\text{ ms}$ | $2.362\text{ s}$ |
| | Slower ($0.8\times$) | **COLLISION** ($7.5\text{ s}$) | **YES** | **GOAL_REACHED** | **NO** | 0 | $9.9\text{ ms}$ | $2.223\text{ s}$ |
| | Faster ($1.2\times$) | **GOAL_REACHED** ($16.0\text{ s}$)* | **NO** | **GOAL_REACHED** | **NO** | 0 | $9.6\text{ ms}$ | $1.741\text{ s}$ |

*\*Contextual Explanation*: Pure Pursuit succeeded in 3 of 9 trials ($33.3\%$) strictly by coincidence when obstacle speed caused the moving agent to have already cleared the crossing zone before the blind robot arrived. In the other 6 trials ($66.7\%$), Pure Pursuit suffered dynamic collisions. DWA achieved **$9 / 9$ successes ($100.0\%$)** across all variations with zero dynamic collisions and zero planner failures.

### 9.5 Verified Screenshot Artifact Pixel Inspection

Regenerated headless screenshot artifacts were verified on disk and inspected via analytical pixel array scans:

| Screenshot Artifact | Resolution | Size (Bytes) | Coral Dynamic Obs Px | Cyan DWA Arc Px | Amber Trail Px | Yellow Robot Px | Sidebar HUD Text Px |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`dynamic_crossing_dwa_interactive.png`** | $1100 \times 720$ | $47,576$ | $986\text{ px}$ | $129\text{ px}$ | $168\text{ px}$ | $168\text{ px}$ | $2,550\text{ px}$ |
| **`dynamic_corridor_dwa_interactive.png`** | $1100 \times 720$ | $47,224$ | $496\text{ px}$ | $109\text{ px}$ | $127\text{ px}$ | $127\text{ px}$ | $2,544\text{ px}$ |
| **`dynamic_chokepoint_dwa_interactive.png`** | $1100 \times 720$ | $47,483$ | $714\text{ px}$ | $129\text{ px}$ | $168\text{ px}$ | $168\text{ px}$ | $2,534\text{ px}$ |

*Rendering Invariance Confirmation*: Confirmed via `tests/unit/test_visualizer.py::test_rendering_invariance_across_all_metrics` that rendering and saving screenshots performs read-only operations that do not mutate robot pose $(x, y, \theta)$, velocities $(v, \omega)$, obstacle positions, simulation time, or step counters.

### 9.6 Automated Test Suite Scorecard

- **Total Passing Automated Tests**: Expanded from 96 to **104 tests** across 11 test modules ($100\%$ pass rate in $8.46\text{ s}$).
- **Code Coverage**: **$82\%$ overall codebase coverage** ($2062$ statements, $365$ missed).
  - DWA Local Planner (`dwa.py`): **$98\%$ coverage**
  - Collision Detection (`collision.py`): **$97\%$ coverage**
  - Kinematic Controller (`controller.py`): **$96\%$ coverage**
  - Simulation Engine (`engine.py`): **$95\%$ coverage**
  - Robot Model (`robot.py`): **$95\%$ coverage**
  - A* Planner (`astar.py`): **$95\%$ coverage**
  - Scenario Loader (`scenarios.py`): **$95\%$ coverage**
  - Grid Map (`grid_map.py`): **$94\%$ coverage**
  - Interactive Visualizer (`animate.py`): **$81\%$ coverage**
  - Dynamic Obstacle Modeling (`dynamic_obstacle.py`): **$72\%$ coverage**

---

## 10. Milestone 5 & 5.1 — State Estimation Validation & Tracking Audit

Milestone 5 introduced synthetic sensor simulation (wheel odometry, rate gyro with bias drift, and decimated GNSS fixes with outage windows) and an analytical 5-state Extended Kalman Filter ($\mathbf{x} = [x, y, \theta, v, b_g]^T$).

Milestone 5.1 audited the estimation evaluation pipeline, resolved the feedback versus estimator decoupling, synchronized simulation stepping timestamps, and distinctly separated **State Estimation Accuracy** from **Trajectory-Tracking Error**.

### 10.1 Root Cause Analysis of Milestone 5 Ground-Truth Discrepancy

In Milestone 5, benchmark scorecards erroneously reported $\approx 0.24\text{--}0.26\text{ m}$ position RMSE even for `ground_truth` feedback mode. The Milestone 5.1 audit identified two root causes:
1. **Decoupled Evaluated Trajectory**: In `SimulationEngine.step()`, `self.estimated_trajectory` unconditionally recorded the background EKF state estimate (`self.ekf.get_estimated_robot_state()`), regardless of whether `state_feedback` was set to `"ekf"`, `"dead_reckoning"`, or `"ground_truth"`. The evaluation function `calculate_estimation_errors(self.trajectory, self.estimated_trajectory)` evaluated true poses against the noisy background EKF rather than evaluating the feedback pose actually supplied to the robot's navigation controller.
2. **Stepping Loop Timing Offset & Trailing Estimate**: In `engine.step()`, sensors were sampled and the estimator was updated *before* checking goal termination and *before* executing `robot.step()`. This produced a 1-step latency ($0.1\text{ s}$, corresponding to $\approx 0.08\text{ m}$ displacement) and appended a trailing 218th estimate on the goal-reaching step that had no counterpart in the 217-element ground-truth trajectory.

**Architectural Fix**: In Milestone 5.1, `feedback_trajectory` was formally established. In `ground_truth` mode, the feedback trajectory records the exact ground-truth states supplied to the controller at each discrete timestamp. When `calculate_estimation_errors` evaluates `self.trajectory` against `self.feedback_trajectory`, all errors ($\text{RMSE}, \text{MAE}, \text{Max Error}, \Delta\theta, \Delta v$) are identically $0.0\text{ m}$ by construction, without any hardcoded if-statements.

---

### 10.2 Table 1: State Estimation Accuracy (Feedback Pose vs Ground Truth)

Evaluated across 36 total trials (3 scenarios $\times$ 4 feedback modes $\times$ 3 seeds `[42, 43, 44]`) using [`scripts/run_estimation_benchmark.py`](../scripts/run_estimation_benchmark.py):

| Scenario | Mode | Feedback Pos RMSE ($m$) | Feedback Pos MAE ($m$) | Max Pos Error ($m$) | Heading RMSE ($^\circ$) | EKF Compute (Mean / P95) | Step Latency (Mean / P95) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`simple_obstacles`** | Ground Truth | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.00^\circ$** | $0.0511\text{ ms}$ / $0.0891\text{ ms}$ | $0.0764\text{ ms}$ / $0.1226\text{ ms}$ |
| | Dead Reckoning | $0.2328\text{ m}$ | $0.2098\text{ m}$ | $0.5134\text{ m}$ | $2.69^\circ$ | $0.0437\text{ ms}$ / $0.0586\text{ ms}$ | $0.0675\text{ ms}$ / $0.0941\text{ ms}$ |
| | **EKF Fusion** | **$0.2583\text{ m}$** | **$0.2230\text{ m}$** | **$0.5992\text{ m}$** | **$4.28^\circ$** | **$0.0493\text{ ms}$ / $0.0813\text{ ms}$** | **$0.0741\text{ ms}$ / $0.1110\text{ ms}$** |
| | EKF (5s Outage) | $0.5479\text{ m}$ | $0.4005\text{ m}$ | $1.4355\text{ m}$ | $6.15^\circ$ | $0.0484\text{ ms}$ / $0.0800\text{ ms}$ | $0.0724\text{ ms}$ / $0.1118\text{ ms}$ |
| **`corridor_s_bend`** | Ground Truth | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.00^\circ$** | $0.0455\text{ ms}$ / $0.0738\text{ ms}$ | $0.0683\text{ ms}$ / $0.0996\text{ ms}$ |
| | Dead Reckoning | $0.2433\text{ m}$ | $0.2161\text{ m}$ | $0.5117\text{ m}$ | $2.96^\circ$ | $0.0386\text{ ms}$ / $0.0414\text{ ms}$ | $0.0595\text{ ms}$ / $0.0641\text{ ms}$ |
| | **EKF Fusion** | **$0.2488\text{ m}$** | **$0.2159\text{ m}$** | **$0.5175\text{ m}$** | **$4.05^\circ$** | **$0.0422\text{ ms}$ / $0.0681\text{ ms}$** | **$0.0633\text{ ms}$ / $0.0909\text{ ms}$** |
| | EKF (5s Outage) | $0.6176\text{ m}$ | $0.5264\text{ m}$ | $1.3039\text{ m}$ | $7.54^\circ$ | $0.0478\text{ ms}$ / $0.0615\text{ ms}$ | $0.0725\text{ ms}$ / $0.0951\text{ ms}$ |
| **`urban_grid`** | Ground Truth | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | **$0.00^\circ$** | $0.0455\text{ ms}$ / $0.0734\text{ ms}$ | $0.0686\text{ ms}$ / $0.1034\text{ ms}$ |
| | Dead Reckoning | $0.3267\text{ m}$ | $0.2627\text{ m}$ | $0.8013\text{ m}$ | $1.65^\circ$ | $0.0420\text{ ms}$ / $0.0576\text{ ms}$ | $0.0655\text{ ms}$ / $0.0920\text{ ms}$ |
| | **EKF Fusion** | **$0.2494\text{ m}$** | **$0.2189\text{ m}$** | **$0.5563\text{ m}$** | **$3.18^\circ$** | **$0.0452\text{ ms}$ / $0.0721\text{ ms}$** | **$0.0678\text{ ms}$ / $0.1021\text{ ms}$** |
| | EKF (5s Outage) | $0.7015\text{ m}$ | $0.5909\text{ m}$ | $1.5091\text{ m}$ | $7.74^\circ$ | $0.0488\text{ ms}$ / $0.0804\text{ ms}$ | $0.0731\text{ ms}$ / $0.1171\text{ ms}$ |

*Telemetry Artifact: Serialized JSON dataset available at [`outputs/metrics/estimation_benchmark_results.json`](../outputs/metrics/estimation_benchmark_results.json).*

#### 10.2.1 Estimator Latency Decomposition & Execution Semantics
In Milestone 5.1, timing was measured around `engine.step()` and labeled as "Filter Latency", which conflated estimator compute with sensor noise sampling, waypoint lookahead progression, controller trajectory evaluation, unicycle RK2 integration, dynamic obstacle progression, and collision checking. 

In Milestone 5.2, telemetry was partitioned into two distinct metrics:
1. **Pure EKF Compute Latency (`estimator_compute_ms`)**:
   - Isolates the mathematical execution of `predict()` (kinematic propagation with gyro rate and gyro bias drift), `update_odometry()` (Kalman gain update with forward wheel speed), `update_gnss()` (when fixes arrive), and state extraction.
   - **Empirical Measurement**: Mean latency is **$0.038\text{--}0.051\text{ ms}$** ($38\text{--}51\ \mu\text{s}$) with P95 latency of **$0.041\text{--}0.089\text{ ms}$**.
2. **Full Simulation Step Latency (`step_latency_ms`)**:
   - Measures the complete simulation loop tick including sensor generation, EKF computation, Pure Pursuit / DWA control generation, kinematic step, obstacle animation, and spatial collision queries.
   - **Empirical Measurement**: Mean latency is **$0.059\text{--}0.076\text{ ms}$** ($59\text{--}76\ \mu\text{s}$) with P95 latency of **$0.064\text{--}0.123\text{ ms}$**.
3. **Background EKF Execution**:
   - In `ground_truth` and `dead_reckoning` modes, the EKF estimator still executes in `engine.step()` in the background. In `ground_truth`, it provides parallel estimation metrics against truth without affecting navigation. In `dead_reckoning`, GNSS measurement updates are skipped while odometry and gyro propagation continue.
4. **Real-Time Determinism Disclaimer**:
   - While the pure filter latency consumes $< 0.1\%$ of the $100\text{ ms}$ ($10\text{ Hz}$) control period, execution runs in user-space Python/NumPy on a non-real-time Windows host. Garbage collection, process scheduling, and OS thread preemption cause tail jitter. Hard real-time determinism cannot be claimed, but soft real-time execution headroom exceeds $1000\times$.

#### 10.2.2 Theoretical & Empirical Interpretation: EKF Fusion vs Dead Reckoning
The benchmark results reveal two essential estimation phenomena that must be understood honestly:
1. **Heading RMSE Anomaly (Dead Reckoning vs EKF)**:
   - Notice that Dead Reckoning yields **lower heading RMSE** ($1.65^\circ\text{--}2.96^\circ$) than EKF Fusion ($3.18^\circ\text{--}4.28^\circ$).
   - *Physical Mechanism*: GNSS provides only position measurements $(z_x, z_y)$; it does not provide direct heading observations. In the Extended Kalman Filter, heading is coupled to position via off-diagonal covariance terms in $P$ and the non-linear unicycle Jacobian $F_x$. When noisy GNSS fixes ($\sigma_{\text{gnss}} = 0.25\text{ m}$) are assimilated, the innovation vector $[z_x - \hat{x},\, z_y - \hat{y}]^T$ is multiplied by Kalman gain elements $K_{\theta x}, K_{\theta y}$. This injects lateral position noise into the heading estimate $\hat{\theta}$, causing high-frequency angular jitter. In Dead Reckoning, the gyro rate noise is only $\sigma_{\text{gyro}} = 0.015\text{ rad/s}$ ($0.86^\circ/\text{s}$), which integrates smoothly without position innovation noise.
2. **Position Error Evolution (Short Routes vs Long Routes)**:
   - On short trajectories (`simple_obstacles` and `corridor_s_bend`, $\approx 21\text{ s}$ duration, $\approx 30\text{ m}$ distance), Dead Reckoning position RMSE ($0.233\text{ m}$ and $0.243\text{ m}$) appears comparable to or slightly lower than EKF position RMSE ($0.258\text{ m}$ and $0.249\text{ m}$).
   - *Physical Mechanism*: A $1.5\%$ wheel scale error over $30\text{ m}$ accumulates only $\approx 0.45\text{ m}$ total distance error. Because each GNSS fix has an independent $0.25\text{ m}$ noise distribution, the EKF position RMSE is fundamentally bounded by the sensor noise floor ($\approx 0.25\text{ m}$). Over short intervals, the uncompensated Dead Reckoning error has not yet surpassed the GNSS noise floor.
   - However, on longer, multi-turn trajectories (`urban_grid`), Dead Reckoning error diverges monotonically, reaching a **maximum position error of $0.8013\text{ m}$** and RMSE of **$0.3267\text{ m}$**. In contrast, EKF position error remains strictly bounded with a maximum error of **$0.5563\text{ m}$** and RMSE of **$0.2494\text{ m}$**, proving long-term stability and drift prevention.

#### 10.2.3 Timestamp Alignment & Terminal Lifecycle Invariants
In Milestone 5.2, `RobotState` was enhanced with an explicit `timestamp: float = 0.0` attribute. The evaluation engine enforces:
1. **Temporal Agreement**: `calculate_estimation_errors` asserts identical length and pairwise timestamp equivalence ($|t_{\text{gt}} - t_{\text{est}}| < 10^{-6}\text{ s}$) across all states.
2. **Lifecycle Synchronization**: Terminal conditions (goal reached, timeout, or early collision) are handled without creating unstepped trailing estimates or dropping physical states, ensuring identical trajectory lengths across ground truth, estimated, and feedback histories.

---

### 10.3 Table 2: Trajectory Tracking & Navigation Outcomes (Physical Motion vs Planned Path)

Trajectory-tracking metrics measure lateral deviation from the planned global A* route (cross-track error) and route length efficiency, completely distinct from state estimation accuracy:

| Scenario | Mode | Navigation Success (Trials) | Mean CTE ($m$) | Max CTE ($m$) | Driven Trajectory ($m$) | Planned Path ($m$) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`simple_obstacles`** | Ground Truth | **$3 / 3$ ($100.0\%$)** | $0.0713\text{ m}$ | $0.3536\text{ m}$ | $29.53\text{ m}$ | $30.73\text{ m}$ |
| | Dead Reckoning | **$3 / 3$ ($100.0\%$)** | $0.1304\text{ m}$ | $0.4807\text{ m}$ | $29.24\text{ m}$ | $30.73\text{ m}$ |
| | **EKF Fusion** | **$3 / 3$ ($100.0\%$)** | $0.1580\text{ m}$ | $0.4845\text{ m}$ | $29.21\text{ m}$ | $30.73\text{ m}$ |
| | EKF (5s Outage) | $2 / 3$ ($66.7\%$) | $0.3942\text{ m}$ | $1.3099\text{ m}$ | $24.14\text{ m}$ | $30.73\text{ m}$ |
| **`corridor_s_bend`** | Ground Truth | **$3 / 3$ ($100.0\%$)** | $0.0874\text{ m}$ | $0.3536\text{ m}$ | $30.32\text{ m}$ | $31.56\text{ m}$ |
| | Dead Reckoning | **$3 / 3$ ($100.0\%$)** | $0.1395\text{ m}$ | $0.4986\text{ m}$ | $30.09\text{ m}$ | $31.56\text{ m}$ |
| | **EKF Fusion** | **$3 / 3$ ($100.0\%$)** | $0.1705\text{ m}$ | $0.5594\text{ m}$ | $30.44\text{ m}$ | $31.56\text{ m}$ |
| | EKF (5s Outage) | $1 / 3$ ($33.3\%$) | $0.4140\text{ m}$ | $1.1918\text{ m}$ | $16.74\text{ m}$ | $31.56\text{ m}$ |
| **`urban_grid`** | Ground Truth | **$3 / 3$ ($100.0\%$)** | $0.0158\text{ m}$ | $0.3536\text{ m}$ | $34.79\text{ m}$ | $35.00\text{ m}$ |
| | Dead Reckoning | **$3 / 3$ ($100.0\%$)** | $0.1964\text{ m}$ | $0.6712\text{ m}$ | $34.33\text{ m}$ | $35.00\text{ m}$ |
| | **EKF Fusion** | **$3 / 3$ ($100.0\%$)** | $0.1915\text{ m}$ | $0.5403\text{ m}$ | $35.00\text{ m}$ | $35.00\text{ m}$ |
| | EKF (5s Outage) | $1 / 3$ ($33.3\%$) | $0.5068\text{ m}$ | $1.4932\text{ m}$ | $19.48\text{ m}$ | $35.00\text{ m}$ |

---

### 10.4 Empirical GNSS Outage Drift & Filter Re-Convergence Breakdown

For all trials subject to a 5.0-second mid-route GNSS signal loss (from $t=4.0\text{ s}$ to $9.0\text{ s}$):

| Scenario | Seed | Pre-Outage Mean Error ($m$) | Peak Error During Outage ($m$) | Post-Outage Re-Converged Error ($m$) | Outage Outcome |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`simple_obstacles`** | Seed 42 | $0.2024\text{ m}$ | $0.8883\text{ m}$ | $0.0000\text{ m}$ (goal reached) | **SUCCESS** |
| | Seed 43 | $0.2212\text{ m}$ | $0.3245\text{ m}$ | $0.1565\text{ m}$ | **SUCCESS** |
| | Seed 44 | $0.5261\text{ m}$ | $2.6588\text{ m}$ | $0.1691\text{ m}$ | Collision (wall clip) |
| **`corridor_s_bend`** | Seed 42 | $0.2175\text{ m}$ | $1.2066\text{ m}$ | $0.0000\text{ m}$ (goal reached) | **SUCCESS** |
| | Seed 43 | $0.1979\text{ m}$ | $0.6824\text{ m}$ | $0.1655\text{ m}$ | Collision (corner clip) |
| | Seed 44 | $0.4843\text{ m}$ | $1.8236\text{ m}$ | $0.0000\text{ m}$ | Collision (wall clip) |
| **`urban_grid`** | Seed 42 | $0.2156\text{ m}$ | $1.6974\text{ m}$ | $0.0000\text{ m}$ (goal reached) | **SUCCESS** |
| | Seed 43 | $0.1940\text{ m}$ | $0.6160\text{ m}$ | $0.1712\text{ m}$ | Collision (junction clip) |
| | Seed 44 | $0.4946\text{ m}$ | $1.8775\text{ m}$ | $0.0000\text{ m}$ | Collision (turn clip) |

*Key Takeaway*: During satellite denial, pure dead reckoning integrates uncompensated gyro drift and wheel slippage, causing peak errors to spike up to $2.66\text{ m}$. In tight corridors with $1.0\text{ m}$ inflation margin, drifts $> 1.0\text{ m}$ lead to boundary collisions. In open runs where the robot survives until $t > 9.0\text{ s}$, the resumption of $1\text{ Hz}$ GNSS fixes immediately contracts the covariance matrix $P$ and re-converges position errors to sub-$0.17\text{ m}$.

---

### 10.5 Feedback Isolation & Audit Invariants

1. **Strict Feedback Isolation**:
   - Navigation controller and waypoint manager consume exclusively `feedback_state`.
   - In `ekf` and `dead_reckoning` modes, true states are never exposed to control or waypoint logic.
   - In `dead_reckoning` mode, `engine.gnss_fixes_processed == 0` and `len(engine.gnss_measurements) == 0`.
2. **Automated Test Suite Expansion (Milestone 5.2 Verification)**:
   - Total passing automated tests: **134 tests passing** across 12 test modules ($100\%$ pass rate in $24.86\text{ s}$).
   - Overall codebase coverage: **$84\%$** ($2490$ statements, $403$ missed).
   - Metrics module (`metrics.py`): **$91\%$ coverage** ($111$ statements, $10$ missed).
   - Simulation Engine (`engine.py`): **$94\%$ coverage** ($296$ statements, $17$ missed).
   - Extended Kalman Filter (`ekf.py`): **$96\%$ coverage** ($116$ statements, $5$ missed).
   - Robot Model (`robot.py`): **$95\%$ coverage** ($42$ statements, $2$ missed).
3. **Temporal Invariants & Telemetry Separation**:
   - `RobotState.timestamp` guarantees monotonic, pairwise physical and estimated trajectory alignment.
   - Separation of pure EKF computation latency ($0.038\text{--}0.051\text{ ms}$) from full simulation step latency ($0.059\text{--}0.076\text{ ms}$) resolved all telemetry ambiguities.

---

## 11. Real-World Dataset Evaluation (Milestones 7, 7.1, 7.2, 7.3 & 7.4: TUM RGB-D Benchmark)

Milestone 7 integrates and rigorously audits real-world robotics benchmark data from the official **TUM RGB-D Benchmark** sequence `freiburg2_pioneer_slam2` (Pioneer 3-DX differential drive mobile robot with Microsoft Kinect RGB-D sensor and external high-accuracy motion capture system at 100 Hz, Sturm et al., IROS 2012).

### 11.1 Real-World Sequence Kinematic Verification

The standalone ground truth file `rgbd_dataset_freiburg2_pioneer_slam2-groundtruth.txt` (2,262,149 bytes) was ingested and parsed:

| Metric | Measured Value | Unit / Description |
| :--- | :--- | :--- |
| **Total Recorded Poses** | $33,188$ | Motion capture 6-DOF poses at $100\text{ Hz}$ |
| **Sequence Duration** | $115.632$ | Seconds ($1.93\text{ minutes}$) |
| **Trajectory Arc Length** | $23.326$ | Total driven metric distance ($m$) |
| **Spatial Span ($X \times Y$)** | $4.984 \times 5.339$ | Metric bounding envelope ($m \times m$) |
| **Average Translational Velocity** | $0.209$ | Meters per second ($m/s$) |
| **Total Cumulative Heading Rotation** | $36.61\text{ rad}$ ($2097.41^\circ$) | Total angular turning across trajectory |
| **Height Bounds ($Z$)** | $[0.523, 0.591]\text{ m}$ | Pioneer robot optical marker height |

### 11.2 Sequence Archive Provenance & Cryptographic Verification
- **Sequence Archive**: `rgbd_dataset_freiburg2_pioneer_slam2.tgz` ($1,183,281,970\text{ bytes}$).
- **SHA-256 Digest**: `67f6c8458e7d9010e0a816ee893f862d27da99fa9d86a711b8c70c4b84e1a077` (locally computed verification digest).
- **Upstream Integrity Note**: TUM does not host external official `.sha256` or `.md5` checksum files on the sequence webpage. Upstream authenticity is established by server Content-Length ($1,183,281,970\text{ bytes}$), HTTP ETag `"46877332-4ae2a2e91a900"`, and internal DEFLATE/tar stream member verification across all 4,195 entries.
- **Ground Truth**: `bd57b2c9ce6776fe0d39c7795d7f7789bf206e4eaa58e04aa216090b7a55d08a` ($2,262,149\text{ bytes}$).
- **Depth Index**: `e09cfeb53e70cf271a94cb2db3ae06d203fcf93ff181bfb8ecde08b8b7d9df48` ($95,535\text{ bytes}$, 2,075 records).

### 11.3 Coordinate-Frame Provenance & Calibration Disclosures
1. **Camera Optical Frame (`camera_depth_optical_frame`)**: Standard OpenCV/OpenNI optical frame (+X right, +Y down, +Z forward).
2. **Robot Base Link (`base_link`)**: REP-105 vehicle frame (+X forward, +Y left, +Z up normal to floor).
3. **Camera Extrinsics Provenance**:
   - Offsets: $d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}$, zero pitch, zero yaw.
   - **Disclosure**: These offsets are **project-defined engineering assumptions** based on the nominal chassis geometry of the Pioneer 3-DX top deck. TUM did **not** publish an official rigid-body calibration transform between the reflective marker tree and the Kinect camera optical center.
4. **TUM Motion Capture World Frame (`tum_mocap_world`)**: Local Euclidean indoor laboratory coordinate frame (+Z vertical, X/Y horizontal room axes). It is **not** geographically aligned to East-North-Up (REP-103 ENU).
5. **Navigation & GridMap Frame (`grid_map_world`)**: 2D discrete grid placed directly in the horizontal $(t_x, t_y)$ plane of the TUM MoCap tracking volume with origin $(-5.0, -5.0)\text{ m}$.

### 11.4 Analytical Hand Calculation Verification
To verify the composition order and prevent sign errors or double-applied mounting offsets, an analytical hand-calculation fixture was evaluated:
- **Intrinsics**: $f_x=f_y=500.0, c_x=320.0, c_y=240.0$, scale $5000.0$.
- **Raw Input**: Pixel $(u=420, v=140)$ with depth $10,000$ ($2.00\text{ m}$).
- **Hand Calculation**:
  - Optical: $X_c = (420 - 320) \cdot 2.0 / 500 = +0.40\text{ m}$, $Y_c = (140 - 240) \cdot 2.0 / 500 = -0.40\text{ m}$, $Z_c = 2.00\text{ m}$.
  - Body: $X_b = 2.00 + 0.15 = 2.15\text{ m}$, $Y_b = -0.40 + 0.00 = -0.40\text{ m}$, $Z_b = -(-0.40) + 0.40 = 0.80\text{ m}$.
  - World at Robot Pose $(r_x=3.0, r_y=-1.0, \theta=30^\circ)$:
    $$X_w = 3.00 + 2.15 \cos(30^\circ) - (-0.40) \sin(30^\circ) = 5.061955\text{ m}$$
    $$Y_w = -1.00 + 2.15 \sin(30^\circ) + (-0.40) \cos(30^\circ) = -0.271410\text{ m}$$
  - World at Robot Pose $(r_x=3.0, r_y=-1.0, \theta=90^\circ)$:
    $$X_w = 3.00 + 2.15(0) - (-0.40)(1) = 3.400000\text{ m}$$
    $$Y_w = -1.00 + 2.15(1) + (-0.40)(0) = 1.150000\text{ m}$$
Automated tests confirmed agreement within $10^{-5}\text{ m}$.

### 11.5 Audited Depth-Pixel Statistics Reconciliation
Every authentic 16-bit depth image has exactly $480 \times 640 = 307,200\text{ pixels}$.
- **Reconciliation Check 1**: $\text{zero\_depth\_pixels} + \text{out\_of\_range\_pixels} + \text{valid\_depth\_pixels} = 307,200$.
- **Reconciliation Check 2**: $\text{floor\_rejected\_points} (Z_b < 0.05\text{m}) + \text{ceiling\_rejected\_points} (Z_b > 1.20\text{m}) + \text{retained\_obstacle\_points} = \text{valid\_depth\_pixels}$.

| Frame Filename | File Size | Valid Pixels ($[0.4, 5.0]\text{m}$) | Zero Depth ($0\text{m}$) | Out-of-Range ($<0.4\text{m}$ / $>5.0\text{m}$) | Invalid Total | Floor Rej. ($<0.05\text{m}$) | Ceiling Rej. ($>1.20\text{m}$) | Retained Obstacles | Check 1 / Check 2 |
|---|---|---|---|---|---|---|---|---|---|
| `1311877812.987032.png` | $141,882\text{ B}$ | 257,151 (83.71%) | 49,783 (16.21%) | 266 (0.09%) | 50,049 (16.29%) | 70,696 (27.49%) | 62,220 (24.20%) | 124,235 (48.31%) | **PASS / PASS** |
| `1311877830.276341.png` | $145,655\text{ B}$ | 136,385 (44.40%) | 55,358 (18.02%) | 115,457 (37.58%) | 170,815 (55.60%) | 28,683 (21.03%) | 39,635 (29.06%) | 68,067 (49.91%) | **PASS / PASS** |
| `1311877860.973057.png` | $138,680\text{ B}$ | 152,383 (49.60%) | 52,246 (17.01%) | 102,571 (33.39%) | 154,817 (50.40%) | 26,242 (17.22%) | 36,122 (23.71%) | 90,019 (59.07%) | **PASS / PASS** |
| `1311877892.876368.png` | $156,120\text{ B}$ | 193,370 (62.95%) | 55,595 (18.10%) | 58,235 (18.96%) | 113,830 (37.05%) | 45,407 (23.48%) | 10,418 (5.39%) | 137,545 (71.13%) | **PASS / PASS** |
| `1311877928.049984.png` | $135,951\text{ B}$ | 256,905 (83.63%) | 50,295 (16.37%) | 0 (0.00%) | 50,295 (16.37%) | 71,165 (27.70%) | 56,103 (21.84%) | 129,637 (50.46%) | **PASS / PASS** |
| **Total / Aggregate** | **718,288 B** | **996,194** | **263,277** | **276,529** | **539,806** | **242,193** | **204,498** | **549,503** | **PASS / PASS** |

### 11.6 Ground-Truth Timestamp Association & Nearest MoCap Residuals
Poses used for 3D-to-world projection are evaluated via SLERP quaternion interpolation at the exact depth timestamp $t_{\text{depth}}$ between bounding 100 Hz MoCap bracket measurements. The time offset to the nearest raw discrete MoCap sample is recorded as the **nearest MoCap sample residual**:

| Frame Timestamp $t_d$ | Nearest MoCap Sample $t_{\text{mocap}}$ | Nearest MoCap Residual $|t_d - t_{\text{mocap}}|$ | Bracket Interval $[t_{\text{before}}, t_{\text{after}}]$ | Interval Duration $\Delta t$ | SLERP Factor $\alpha$ | Interpolated Pose $(x, y, \theta)$ |
|---|---|---|---|---|---|---|
| `1311877812.987032` | `1311877812.987700` | **$0.668\text{ ms}$** ($0.000668\text{ s}$) | `[1311877812.9843, 1311877812.9877]` | $3.4\text{ ms}$ | 0.8035 | $(-2.258\text{m}, -2.380\text{m}, 151.08^\circ)$ |
| `1311877830.276341` | `1311877830.277000` | **$0.659\text{ ms}$** ($0.000659\text{ s}$) | `[1311877830.2737, 1311877830.2770]` | $3.3\text{ ms}$ | 0.8003 | $(-2.694\text{m}, -0.211\text{m}, -7.88^\circ)$ |
| `1311877860.973057` | `1311877860.973600` | **$0.557\text{ ms}$** ($0.000557\text{ s}$) | `[1311877860.9703, 1311877860.9736]` | $3.3\text{ ms}$ | 0.8355 | $(0.457\text{m}, -1.745\text{m}, 92.48^\circ)$ |
| `1311877892.876368` | `1311877892.874800` | **$1.568\text{ ms}$** ($0.001568\text{ s}$) | `[1311877892.8748, 1311877892.8780]` | $3.2\text{ ms}$ | 0.4900 | $(1.900\text{m}, 0.943\text{m}, 38.26^\circ)$ |
| `1311877928.049984` | `1311877928.050000` | **$0.016\text{ ms}$** ($0.000016\text{ s}$) | `[1311877928.0467, 1311877928.0500]` | $3.3\text{ ms}$ | 0.9952 | $(-1.828\text{m}, 2.953\text{m}, 20.64^\circ)$ |

> [!WARNING]
> **Synchronization Disclosure**: Nearest MoCap sample residuals ($0.016\text{--}1.568\text{ ms}$) measure temporal closeness to discrete 100 Hz tracking samples; they do NOT demonstrate physical hardware clock synchronization between the sensor and tracking computers.

### 11.7 Spatial Grid Bounds, Clipping & Grid Semantics Disclosure
- **Grid Semantics**: `GridMap` output represents **projected obstacle endpoint rasterization (point hits)**. Cells with value `1` contain obstacle returns; cells with value `0` are **unobserved / unmarked**, NOT confirmed navigable free space (no ray-marching line-of-sight clearing is performed).
- **GridMap Arena Configuration**: $10.0\text{ m} \times 10.0\text{ m}$ local world frame ($200 \times 200$ cells, resolution $0.05\text{ m}$, origin $(-5.0, -5.0)\text{ m}$).
- **Robot Trajectory Bounds**: $X \in [-2.829, 2.154]\text{ m}$ (span: $4.98\text{ m}$), $Y \in [-2.381, 2.958]\text{ m}$ (span: $5.34\text{ m}$).
- **Obstacle Observation Extents**: Forward-facing Kinect camera ($5.0\text{ m}$ range) viewing peripheral hall walls extends obstacle coordinates to $X \in [-7.089, 7.044]\text{ m}$, $Y \in [-6.184, 6.511]\text{ m}$.
- **Boundary Handling**: Obstacle points exceeding $[-5.0, 5.0]\text{ m}$ are safely clipped without memory overflow or map corruption.

| Frame Filename | World Envelope $[X_{\min}, X_{\max}] \times [Y_{\min}, Y_{\max}]\text{ m}$ | Retained Obstacles | Points In-Bounds | Points Clipped | Clipping % | New Grid Cells Marked |
|---|---|---|---|---|---|---|
| `1311877812.987032.png` | $[-7.089, -3.475] \times [-2.353, 1.571]\text{ m}$ | 124,235 | 44,700 | 79,535 | 64.02% | +1,502 |
| `1311877830.276341.png` | $[-2.637, 2.290] \times [-1.905, 0.490]\text{ m}$ | 68,067 | 68,067 | 0 | 0.00% | +3,857 |
| `1311877860.973057.png` | $[-0.963, 1.834] \times [-1.687, 3.238]\text{ m}$ | 90,019 | 85,292 | 4,727 | 5.25% | +1,378 |
| `1311877892.876368.png` | $[2.818, 7.044] \times [1.194, 4.850]\text{ m}$ | 137,545 | 136,138 | 1,407 | 1.02% | +685 |
| `1311877928.049984.png` | $[-0.296, 1.738] \times [2.521, 6.511]\text{ m}$ | 129,637 | 102,159 | 27,478 | 21.20% | +1,508 |
| **Total / Aggregate** | **Full Span: $[-7.089, 7.044] \times [-6.184, 6.511]\text{ m}$** | **549,503** | **436,356 (79.41%)** | **113,147 (20.59%)** | **20.59%** | **8,930 occupied cells** |

### 11.8 Trajectory Tracking & Alignment Benchmarks

Evaluation was partitioned into two distinct benchmarks to separate control tracking from state interpolation:

#### 11.6.1 Closed-Loop Pure Pursuit Waypoint Tracking
A simulated unicycle vehicle tracked $61$ reference waypoints extracted from the Pioneer ground truth trajectory (spaced by $\ge 0.35\text{ m}$) using `PurePursuitController`:
- **Simulated Steps**: $956$ steps ($95.6\text{ s}$ simulated physics time).
- **Driven Trajectory Length**: $19.30\text{ m}$ (following the $23.33\text{ m}$ path with smooth kinematic corner chords).
- **Mean Orthogonal Cross-Track Error (CTE)**: **$0.0429\text{ m}$** ($4.29\text{ cm}$).
- **Maximum Cross-Track Error**: **$0.1485\text{ m}$** ($14.85\text{ cm}$).
- **Task Outcome**: **GOAL REACHED** (successful tracking with zero divergence).

#### 11.6.2 Synthetic Pose Perturbation & SLERP Interpolation Benchmark
Decimated poses ($10\text{ Hz}$) with added calibrated Gaussian perturbation ($\sigma_p = 0.03\text{ m}, \sigma_\theta = 0.02\text{ rad}$) were evaluated against ground truth poses using spherical linear interpolation (SLERP):
- **Aligned Samples**: $3,319$ evaluation steps ($10\text{ Hz}$ sampling over $115.63\text{ s}$).
- **Position RMSE**: **$0.0425\text{ m}$** ($4.25\text{ cm}$, mathematically consistent with $\sqrt{0.03^2 + 0.03^2} \approx 0.0424\text{ m}$).
- **Position MAE**: **$0.0375\text{ m}$** ($3.75\text{ cm}$).
- **Max Position Error**: **$0.1225\text{ m}$** ($12.25\text{ cm}$).
- **Heading RMSE**: **$0.0203\text{ rad}$** (**$1.16^\circ$**).
- **Scientific Disclosure**: This benchmark measures SLERP interpolation accuracy under calibrated perturbation; it does NOT measure visual odometry, EKF state estimation, or real-time RGB-D localization accuracy.

- **Execution Output**: Metrics serialized to [`outputs/metrics/dataset_evaluation_summary.json`](../outputs/metrics/dataset_evaluation_summary.json) and diagnostic figure saved to [`outputs/figures/dataset_evaluation_diagnostic.png`](../outputs/figures/dataset_evaluation_diagnostic.png).

---

## 12. Milestone 7.5: Extrinsic Calibration Sensitivity & Mapping Robustness Analysis

Milestone 7.5 establishes an empirical sensitivity study quantifying the impact of 6-DoF camera mounting tolerances ($d_x, d_y, d_z, \text{pitch}, \text{yaw}, \text{roll}$) on authentic TUM RGB-D projection, height slicing, and GridMap rasterization.

### 12.1 Calibration Provenance & Engineering Hypothesis Disclosure
> [!IMPORTANT]
> **Extrinsic Calibration Provenance Disclosure**:
> The baseline camera mounting parameters ($d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}, \text{pitch} = 0.0^\circ, \text{yaw} = 0.0^\circ, \text{roll} = 0.0^\circ$) represent a **project-defined engineering hypothesis** derived from the physical chassis layout of the Pioneer 3-DX robot top deck.
> TUM provides intrinsic camera calibration, but does **not** publish an official rigid-body extrinsic transformation linking the 100 Hz motion-capture reflective marker frame to the Kinect optical center.
> This sensitivity analysis establishes empirical uncertainty bounds; it does **not** claim unverified physical real-world calibration accuracy.

### 12.2 Systematic 6-DoF Sensitivity Scorecard

Evaluated on all 5 authentic TUM depth frames ($996,194$ valid depth pixels, $549,503$ baseline obstacle points, $8,930$ baseline occupied cells):

| Parameter Sweep | Parameter Value | Offset from Baseline | Mean 3D Disp ($m$) | Max 3D Disp ($m$) | GridMap IoU (%) | Retained Obstacle Points | Occupied Grid Cells | Primary Effect / Failure Mode |
|---|---|---|---|---|---|---|---|---|
| **$d_x$ (Forward)** | $0.05\text{ m}$ | $-0.10\text{ m}$ | $0.1000$ | $0.1000$ | $82.40\%$ | $549,503$ | $9,128$ | Rigid forward/backward shift; zero height impact |
| | $0.10\text{ m}$ | $-0.05\text{ m}$ | $0.0500$ | $0.0500$ | $88.34\%$ | $549,503$ | $9,023$ | Rigid forward/backward shift; zero height impact |
| | **$0.15\text{ m}$ (Base)**| **$0.00\text{ m}$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $0.20\text{ m}$ | $+0.05\text{ m}$ | $0.0500$ | $0.0500$ | $88.53\%$ | $549,503$ | $8,828$ | Rigid forward/backward shift; zero height impact |
| | $0.25\text{ m}$ | $+0.10\text{ m}$ | $0.1000$ | $0.1000$ | $82.54\%$ | $549,503$ | $8,700$ | Rigid forward/backward shift; zero height impact |
| **$d_y$ (Lateral)** | $-0.10\text{ m}$ | $-0.10\text{ m}$ | $0.1000$ | $0.1000$ | $82.14\%$ | $549,503$ | $9,042$ | Rigid lateral translation; zero height impact |
| | $-0.05\text{ m}$ | $-0.05\text{ m}$ | $0.0500$ | $0.0500$ | $88.62\%$ | $549,503$ | $8,978$ | Rigid lateral translation; zero height impact |
| | **$0.00\text{ m}$ (Base)**| **$0.00\text{ m}$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $+0.05\text{ m}$ | $+0.05\text{ m}$ | $0.0500$ | $0.0500$ | $88.39\%$ | $549,503$ | $8,865$ | Rigid lateral translation; zero height impact |
| | $+0.10\text{ m}$ | $+0.10\text{ m}$ | $0.1000$ | $0.1000$ | $81.87\%$ | $549,503$ | $8,790$ | Rigid lateral translation; zero height impact |
| **$d_z$ (Height)** | $0.30\text{ m}$ | $-0.10\text{ m}$ | $0.1000$ | $0.1000$ | $68.36\%$ | $430,344$ | $6,111$ | Vertical window drops; $-21.7\%$ obstacles |
| | $0.35\text{ m}$ | $-0.05\text{ m}$ | $0.0500$ | $0.0500$ | $84.59\%$ | $478,980$ | $7,554$ | Vertical window drops; $-12.8\%$ obstacles |
| | **$0.40\text{ m}$ (Base)**| **$0.00\text{ m}$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $0.45\text{ m}$ | $+0.05\text{ m}$ | $0.0500$ | $0.0500$ | $89.31\%$ | $643,451$ | $9,993$ | Vertical window rises; $+17.1\%$ obstacles |
| | $0.50\text{ m}$ | $+0.10\text{ m}$ | $0.1000$ | $0.1000$ | $81.88\%$ | $796,383$ | $10,895$ | Vertical window rises; $+44.9\%$ obstacles |
| **Pitch (Tilt)** | $-5.00^\circ$ | $-5.00^\circ$ | $0.2002$ | $0.4753$ | $79.24\%$ | $768,450$ | $10,756$ | Upward tilt; floor points rise into obstacle band |
| | $-2.50^\circ$ | $-2.50^\circ$ | $0.1001$ | $0.2377$ | $85.67\%$ | $656,120$ | $10,138$ | Upward tilt; moderate floor point intrusion |
| | **$0.00^\circ$ (Base)** | **$0.00^\circ$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $+2.50^\circ$ | $+2.50^\circ$ | $0.1001$ | $0.2377$ | $61.89\%$ | $440,799$ | $5,661$ | Downward tilt; obstacles drop below floor cutoff |
| | $+5.00^\circ$ | $+5.00^\circ$ | $0.2002$ | $0.4753$ | **$6.23\%$** | $367,736$ | **$616$** | **Catastrophic slicing collapse ($93\%$ cell loss)** |
| **Yaw (Pan)** | $-5.00^\circ$ | $-5.00^\circ$ | $0.2050$ | $0.5017$ | $67.32\%$ | $549,503$ | $9,391$ | Lateral arc error ($\approx r \theta$); 0% height impact |
| | $-2.50^\circ$ | $-2.50^\circ$ | $0.1025$ | $0.2509$ | $77.60\%$ | $549,503$ | $9,134$ | Lateral arc error ($\approx r \theta$); 0% height impact |
| | **$0.00^\circ$ (Base)** | **$0.00^\circ$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $+2.50^\circ$ | $+2.50^\circ$ | $0.1025$ | $0.2509$ | $77.24\%$ | $549,503$ | $8,666$ | Lateral arc error ($\approx r \theta$); 0% height impact |
| | $+5.00^\circ$ | $+5.00^\circ$ | $0.2050$ | $0.5017$ | $66.39\%$ | $549,503$ | $8,474$ | Lateral arc error ($\approx r \theta$); 0% height impact |
| **Roll (Bank)** | $-5.00^\circ$ | $-5.00^\circ$ | $0.0721$ | $0.3111$ | $85.16\%$ | $549,979$ | $8,610$ | Asymmetric lateral height tilt across sensor FOV |
| | $-2.50^\circ$ | $-2.50^\circ$ | $0.0361$ | $0.1556$ | $92.39\%$ | $548,559$ | $8,814$ | Minor lateral tilt across sensor FOV |
| | **$0.00^\circ$ (Base)** | **$0.00^\circ$** | **$0.0000$** | **$0.0000$** | **$100.00\%$** | **$549,503$** | **$8,930$** | Baseline Nominal Hypothesis |
| | $+2.50^\circ$ | $+2.50^\circ$ | $0.0361$ | $0.1556$ | $92.18\%$ | $551,252$ | $8,862$ | Minor lateral tilt across sensor FOV |
| | $+5.00^\circ$ | $+5.00^\circ$ | $0.0721$ | $0.3111$ | $83.94\%$ | $554,749$ | $8,669$ | Asymmetric lateral height tilt across sensor FOV |

### 12.3 Robustness & Failure Mode Analysis

1. **Catastrophic Failure Mode in Downward Pitch ($+5.0^\circ$)**:
   - When the camera is pitched downward by $+5.0^\circ$, forward body coordinates rotate downward:
     $$Z_b' = -X_b \sin(5^\circ) + Z_b \cos(5^\circ) \approx -0.0871 X_b + 0.996 Z_b$$
   - At typical obstacle ranges ($X_b \in [2.0, 4.0]\text{ m}$), this introduces a negative vertical shift of $-0.17\text{ to } -0.35\text{ m}$.
   - Obstacles previously positioned inside the $[0.05, 1.20]\text{ m}$ window plunge below the $0.05\text{ m}$ floor rejection cutoff and are discarded as ground plane!
   - Consequently, occupied GridMap cells collapse from $8,930$ to just $616$ (a $93.1\%$ loss of obstacle structure), causing IoU to drop to **$6.23\%$**.
2. **Yaw Lever-Arm Arc Distortion**:
   - Yaw rotation leaves $Z_b$ completely unaltered ($Z_b' = Z_b$), preserving obstacle count at exactly $549,503$.
   - However, horizontal coordinates rotate by $\psi$, creating range-dependent lateral displacement:
     $$\Delta \approx 2 R \sin(\psi / 2) \approx R \psi$$
   - At the $5.0\text{ m}$ sensor cutoff, a $5.0^\circ$ yaw misalignment displaces endpoints by $0.436\text{ m}$ (over 8 grid cells of $5\text{ cm}$), causing IoU to drop to $66.39\%$.
3. **Vertical Offset ($d_z$) vs Horizontal Offsets ($d_x, d_y$)**:
   - $d_x$ and $d_y$ produce pure rigid translations where point displacement equals offset magnitude ($\Delta P = \text{offset}$). IoU remains above $81\%$.
   - $d_z$ translations directly shift the height filtering window. Raising $d_z$ to $0.50\text{ m}$ causes floor points ($Z_b \in [0.00, 0.05]\text{ m}$) to enter the obstacle filter, inflating retained points by $+44.9\%$ to $796,383$.
4. **Deceptive Nature of Raw Cell Count**:
   - Symmetrical parameter offsets produce asymmetric cell counts (e.g. $-5^\circ$ pitch yields $10,756$ cells while $+5^\circ$ pitch yields $616$ cells). An uncalibrated sensor can create an apparently "dense" map that actually consists of spurious floor returns.

- **Generated Artifacts**:
  - Machine-readable JSON summary: [`outputs/metrics/calibration_sensitivity_summary.json`](../../outputs/metrics/calibration_sensitivity_summary.json)
  - Comprehensive 6-panel diagnostic plot: [`outputs/figures/calibration_sensitivity_diagnostic.png`](../../outputs/figures/calibration_sensitivity_diagnostic.png)

