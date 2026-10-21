# Technical Documentation Hub: AI-Based Autonomous Navigation System

Welcome to the centralized technical documentation repository for the **AI-Based Autonomous Navigation System**. This folder (`docs/`) serves as the single source of truth for all technical specifications, system architecture, data flow pipelines, module implementations, model learning mechanics, datasets, validation criteria, and execution logs.

---

## 1. Simulation-First Baseline Documentation (Current Milestone)

| Document | Focus Area | Description |
| :--- | :--- | :--- |
| **[`architecture.md`](architecture.md)** | 2D Baseline Architecture | Component interaction flow, subsystem responsibilities, and interface contracts. |
| **[`installation.md`](installation.md)** | Setup & Dependencies | Python environment setup, virtualenv activation, package installation, and verification steps. |
| **[`methodology.md`](methodology.md)** | Mathematical Formulations | Mathematical models for 2D GridMap, 8-connected A*, unicycle kinematics, Pure Pursuit, and Circle-AABB collisions. |
| **[`testing.md`](testing.md)** | Quality Assurance & Tests | Test architecture, suite descriptions, invariant assertions, and pytest execution commands. |
| **[`results.md`](results.md)** | Benchmark Performance | Quantitative evaluation across 4 standard benchmark scenarios with error analysis. |
| **[`limitations.md`](limitations.md)** | Scope & Constraints | Technical assumptions, planar 2D bounds, unicycle limits, and mitigation strategies. |
| **[`roadmap.md`](roadmap.md)** | Phased Milestones | Progressive milestones from 2D baseline to OpenCV perception, Pygame GUI, DWA, and 3D simulation. |

---

## 2. Advanced System Architecture & Theory

| Document | Focus Area | Description |
| :--- | :--- | :--- |
| **[`01_system_architecture.md`](01_system_architecture.md)** | High-Level System Design | Sensor layers, perception pipelines, MPC control, and safety supervisor topology. |
| **[`02_data_flow_and_pipeline.md`](02_data_flow_and_pipeline.md)** | Multi-Rate Streaming | End-to-end data flow, execution timing, and ISO 8855 coordinate transforms ($S_{\text{cam}} \to V_{\text{ego}} \to W_{\text{map}}$). |
| **[`03_module_specifications.md`](03_module_specifications.md)** | Module Contracts & APIs | In-depth technical breakdown of state estimation, perception, planning, and control modules. |
| **[`04_datasets_and_benchmarks.md`](04_datasets_and_benchmarks.md)** | Datasets & Sensor Modalities | Synthetic & real-world benchmarks (nuScenes, KITTI, CARLA), data schemas, and noise modeling. |
| **[`05_model_learning_and_training.md`](05_model_learning_and_training.md)** | Model Training Mechanics | CTTN deep neural architecture, loss formulations (Huber + Smoothness), and RL reward design. |
| **[`06_validation_and_testing_framework.md`](06_validation_and_testing_framework.md)** | Verification Protocol | 4-tier validation pipeline, safety thresholds, and quantitative pass/fail boundaries. |
| **[`execution_log.md`](execution_log.md)** | Progress Journal | Chronological implementation record tracking every milestone, test result, and validation artifact. |
