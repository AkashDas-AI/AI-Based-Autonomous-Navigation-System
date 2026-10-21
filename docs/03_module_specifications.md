# 03 - Module Specifications

## 1. Overview of Codebase Modules

The system is structured into high-cohesion, low-coupling modules designed for testing, simulation injection, and real-time execution.

```
AI-Based Autonomous Navigation System/
├── docs/                      # Technical specifications, architecture, execution logs
├── config/                    # YAML / JSON configuration for vehicles, sensors & models
├── data/                      # Dataset caches, offline logs, and evaluation routes
├── models/                    # Model weights, checkpoint files, and ONNX exports
├── src/                       # Production source code
│   ├── core/                  # Common primitives, math utilities, spatial types
│   ├── sensors/               # Camera, LiDAR, IMU, GPS drivers & mock simulators
│   ├── localization/          # EKF state estimator, odometry fusion, pose graph
│   ├── perception/            # Neural detection, segmentation, and occupancy mapper
│   ├── planning/              # Global pathfinder (A*), local spline/lattice planner
│   ├── control/               # PID, Pure Pursuit, MPC trajectory followers
│   ├── safety/                # Fault detector, collision watchdog, emergency braking
│   └── pipeline/              # Main autonomous loop orchestrator
└── tests/                     # Unit, integration, and closed-loop regression test suites
```

---

## 2. Detailed Subsystem Specifications

### 2.1 Core Utilities (`src/core/`)
- **`types.py`**: Immutable dataclasses representing core entities:
  - `Vector3D(x: float, y: float, z: float)`
  - `Pose(position: Vector3D, orientation: Vector3D)`
  - `VehicleState(x: float, y: float, v: float, yaw: float, steer: float)`
  - `TrajectoryPoint(x: float, y: float, v: float, target_time: float, curvature: float)`
- **`transformations.py`**: Homogeneous rotation matrices, SE(3) and SO(3) coordinate transforms, Euler-Quaternion converters.

### 2.2 Localization & State Estimation (`src/localization/`)
- **Responsibility**: Estimating high-precision ego position, heading, and speed without latency gaps.
- **Key Class**: `ExtendedKalmanFilter`
  - **State Vector**: $\mathbf{x} = [x, y, v, \psi, \dot{\psi}]^T$
  - **Inputs**: IMU accelerometer ($a_x, a_y$), gyro ($\dot{\psi}$), GNSS coordinates $(x_{\text{gps}}, y_{\text{gps}})$, and wheel speed ($v_{\text{wheel}}$).
  - **Outputs**: `VehicleState` at $100\text{ Hz}$.

### 2.3 Perception & World Representation (`src/perception/`)
- **Responsibility**: Identifying obstacles, road geometry, and free traversable corridors.
- **Key Classes**:
  - `ObstacleDetector`: Deep neural network taking RGB/LiDAR tensors, returning 2D/3D bounding boxes `List[BoundingBox3D]`.
  - `LaneSegmenter`: Computes lane polynomial coefficients $y(x) = c_0 + c_1 x + c_2 x^2 + c_3 x^3$ and lane center offset.
  - `OccupancyGridMapper`: Maintains a 2D probabilistic top-down occupancy grid (Log-Odds representation).

### 2.4 Planning Subsystem (`src/planning/`)
- **Responsibility**: Generating optimal, kinematically feasible, collision-free paths.
- **Key Classes**:
  - `GlobalRoutePlanner`: Calculates high-level waypoints along a graph or topological road network using $A^*$ or Dijkstra.
  - `LocalTrajectoryPlanner`: Generates smooth quintic polynomial trajectories / Frenet frame paths optimizing:
    $$J = w_j \int j(t)^2 dt + w_t T + w_d (d - d_{\text{center}})^2 + w_v (v - v_{\text{target}})^2$$
  - **Collision Checker**: Evaluates swept volume of the ego vehicle against predicted obstacle positions over the planning horizon.

### 2.5 Control Subsystem (`src/control/`)
- **Responsibility**: Accurate trajectory execution via vehicle dynamics.
- **Key Classes**:
  - `PurePursuitController`: Geometric steering tracker calculating:
    $$\delta = \arctan\left(\frac{2 L \sin \alpha}{L_{fd}}\right)$$
    where $L$ is wheelbase, $L_{fd}$ is lookahead distance, $\alpha$ is angle between vehicle heading and lookahead target.
  - `PIDVelocityController`: Longitudinal throttle/brake regulator minimizing speed error $e_v = v_{\text{target}} - v_{\text{current}}$.
  - `ModelPredictiveController (MPC)`: Receding horizon optimization over kinematic bicycle model enforcing actuation bounds.

### 2.6 Safety & Interceptor (`src/safety/`)
- **Responsibility**: Hard real-time safety envelope enforcement.
- **Key Classes**:
  - `SafetyWatchdog`: Monitors sensor freshness (drops if timeout $> 100\text{ ms}$).
  - `CollisionAvoider (AEB)`: Asserts Time-To-Collision condition:
    $$\text{TTC} = \frac{d_{\text{rel}}}{v_{\text{rel}}} < \text{TTC}_{\text{critical}} \implies \text{Trigger Full Emergency Braking}$$

