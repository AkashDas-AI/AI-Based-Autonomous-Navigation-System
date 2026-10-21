# Technical Documentation Hub: AI-Based Autonomous Navigation System

Welcome to the centralized technical documentation repository for the **AI-Based Autonomous Navigation System**. This folder (`docs/`) serves as the single source of truth for system architecture, mathematical formulations, quality assurance protocols, benchmark results, technical constraints, and development milestones.

---

## Technical Documentation Suite

| Document | Focus Area | Description |
| :--- | :--- | :--- |
| **[`architecture.md`](architecture.md)** | Subsystem Architecture & Interfaces | Layered component topology, data boundaries, simulation bridge contracts (REP-103/105), and state machines. |
| **[`installation.md`](installation.md)** | Setup & Verification Guide | Step-by-step Windows/POSIX Python environment setup, virtualenv activation, package installation, and sanity checks. |
| **[`methodology.md`](methodology.md)** | Mathematical Formulations & Control | Mathematical models for 2D GridMap inflation, 8-connected A* search, unicycle kinematics (RK2), Pure Pursuit, DWA, and EKF sensor fusion. |
| **[`testing.md`](testing.md)** | Quality Assurance & 4-Tier Test Suite | Comprehensive testing architecture, invariant assertions, continuous collision verification, and pytest coverage reports across 203 automated tests. |
| **[`results.md`](results.md)** | Benchmark Performance & Evaluation | Quantitative scorecard and metric analysis across static scenarios, dynamic obstacle avoidance, state estimation, and authentic TUM RGB-D dataset mapping. |
| **[`limitations.md`](limitations.md)** | Scope, Assumptions & Boundaries | Engineering assumptions, 2D planar navigation boundaries, endpoint rasterization semantics, unverified CAD extrinsics, and kinematic limits. |
| **[`roadmap.md`](roadmap.md)** | Phased Milestones & Future Work | Progressive development milestones from baseline planning to OpenCV perception, Pygame interactive GUI, DWA local planner, and future ROS 2 integration. |
| **[`execution_log.md`](execution_log.md)** | Chronological Engineering Diary | Detailed audit journal tracking every milestone implementation step, benchmark execution, and verification evidence. |

---

## Core System Architecture Summary

The repository implements a simulation-first 2D mobile robot navigation stack designed for reproducible workstation execution:

1. **Perception**: Classical OpenCV image segmentation and authentic TUM RGB-D depth frame unprojection into metric 2D GridMap representations.
2. **State Estimation**: 5-state Extended Kalman Filter fusing wheel odometry, drifting IMU gyro, and decimated GNSS fixes with Joseph-form updates.
3. **Global Planning**: 8-connected A* search with octile distance heuristics and strict diagonal corner-cutting prevention.
4. **Local Reactive Control**: Dynamic Window Approach (DWA) evaluating dynamic velocity search spaces for dynamic obstacle avoidance, coupled with an adaptive Pure Pursuit trajectory tracker.
5. **Continuous Simulation & Safety**: Unicycle kinematic integration (RK2) with swept-disc relative continuous collision checking to eliminate temporal tunneling.
