# 04 - Datasets and Benchmarks

## 1. Selected Datasets & Simulation Environments

To train, evaluate, and benchmark the autonomous navigation system across diverse scenarios (urban, highway, adverse weather, emergency maneuvers), we leverage a dual strategy combining high-fidelity synthetic simulation with established real-world open benchmarks.

| Source / Benchmark | Modality | Primary Use Case | Scale / Characteristics |
| :--- | :--- | :--- | :--- |
| **CARLA Autonomous Driving Leaderboard (Synthetic)** | Multi-camera (RGB + Depth), LiDAR, GNSS, IMU, CAN bus | End-to-end closed-loop navigation, edge cases, closed-loop driving benchmarks | $1000\text{+ km}$ simulated urban/highway miles, multiple weather conditions, dynamic pedestrians/vehicles |
| **nuScenes Dataset (Real-World)** | 6 Cameras, 1 LiDAR (32 beams), 5 Radars, IMU/GPS | 3D object detection, tracking, map extraction, trajectory forecasting | 1000 scenes of $20\text{ s}$ each, diverse urban conditions (Boston & Singapore) |
| **KITTI Vision Benchmark (Real-World)** | Stereo RGB, Velodyne HDL-64E, RTK-GPS/IMU | Optical flow, visual odometry, stereo depth, lane estimation | Classic benchmark for baseline calibration and odometry precision |
| **Synthesized Closed-Loop Scenario Suite** | Direct kinematic telemetry and synthetic range sensors | Algorithmic unit verification, controller stress-testing, automated CI/CD regression | Rapid automated integration tests running without heavyweight GPU simulators |

---

## 2. Sensor Modalities & Ingestion Schema

Data frames are recorded and processed using structured hierarchical schemas:

### 2.1 Sensor Stream Specification
- **Front RGB Camera**:
  - Resolution: $1920 \times 1080$ downscaled to $640 \times 360$ for inference.
  - Field of View (FOV): $90^\circ$ horizontal.
  - Frame Rate: $20\text{--}30\text{ FPS}$.
- **LiDAR**:
  - Type: 32/64 Channel Rotating LiDAR.
  - Range: $0.5\text{ m} \text{ to } 100\text{ m}$.
  - Points per second: $\sim 700,000 \text{ pts/s}$.
- **Inertial Measurement Unit (IMU)**:
  - 3-axis Accelerometer ($\text{m/s}^2$).
  - 3-axis Gyroscope ($\text{rad/s}$).
  - Frequency: $100\text{ Hz}$.
- **Vehicle Telemetry (CAN Bus)**:
  - Ground truth speed ($v$), steering angle ($\delta$), throttle, brake, wheel speeds.

---

## 3. Data Storage & Serialization Format

Data logs and offline training sequences are serialized using standard high-throughput formats:
- **`HDF5` / `Zarr`**: Dense numerical tensors (synchronized camera images, LiDAR range matrices, state trajectories).
- **`MCAP` / `Parquet`**: Time-stamped telemetry events, bounding box annotations, and ground-truth odometry.

```
data/
├── raw/                      # Downloaded/captured dataset runs
│   ├── episode_0001/
│   │   ├── camera_front/     # Timestamped PNGs
│   │   ├── lidar/            # .bin or .pcd files
│   │   └── telemetry.csv     # GPS, IMU, Control actions
├── processed/                # Normalized numpy/pt cache files
│   ├── train_shards/
│   └── val_shards/
└── maps/                     # Vector maps and waypoint graphs
```

---

## 4. Data Augmentation & Noise Modeling

To bridge the sim-to-real gap and enhance neural model robustness, inputs undergo online stochastic transformations during training:

1. **Photometric Augmentations**:
   - Random brightness, contrast, and color temperature adjustment.
   - Gaussian blur and simulated rain/lens-droplet occlusions.
2. **Sensor Noise Injection**:
   - Zero-mean Gaussian noise on IMU acceleration: $\mathbf{a}_{\text{meas}} = \mathbf{a}_{\text{true}} + \mathcal{N}(0, \sigma_a^2) + \mathbf{b}_a$.
   - Gaussian drift on gyroscope measurements: $\boldsymbol{\omega}_{\text{meas}} = \boldsymbol{\omega}_{\text{true}} + \mathcal{N}(0, \sigma_\omega^2) + \mathbf{b}_\omega$.
   - GNSS multipath error simulation with occasional signal loss.
3. **Trajectory Perturbations**:
   - Synthetic lateral drift injection to force imitation models to learn recovery trajectories (DAgger-style perturbation).

