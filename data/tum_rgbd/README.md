# TUM RGB-D Benchmark Dataset (`freiburg2_pioneer_slam2`)

This directory houses real-world robotics dataset assets for the **AI-Based Autonomous Navigation System**.

## 1. Sequence Overview
- **Sequence Name:** `freiburg2_pioneer_slam2`
- **Platform:** Mobile Pioneer 3-DX differential-drive ground robot equipped with a forward-facing Microsoft Kinect RGB-D camera.
- **Recording Environment:** Large robotics testing hall with obstacles, driven via joystick.
- **Official Host:** Computer Vision Group, Technical University of Munich (TUM).
- **Official Portal:** [https://vision.in.tum.de/data/datasets/rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- **Direct Ground Truth URL:** [https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2-groundtruth.txt](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2-groundtruth.txt)
- **Full Sequence Archive URL:** [https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.tgz](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.tgz) (Approx. 1.10 GB / 1,183,281,970 bytes).

---

## 2. License & Attribution Requirements
- **Authoritative License**: Creative Commons Attribution 4.0 International (CC BY 4.0).
  - *Official Notice from TUM Landing Page (`https://vision.in.tum.de/data/datasets/rgbd-dataset`)*:
    > "Unless stated otherwise, all data in the TUM RGB-D benchmark is licensed under a Creative Commons 4.0 Attribution License (CC BY 4.0) and the accompanying source code is licensed under a BSD-2-Clause License."
  - *License Permissions*: Permitted to share, copy, redistribute, and adapt in any medium or format for any purpose with appropriate credit.
- **Academic Attribution / Citation (Distinguished from Legal License)**:
  - While CC BY 4.0 provides the legal copyright permission, scientific integrity requires formal academic citation:
    > J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers,
    > *"A Benchmark for the Evaluation of RGB-D SLAM Systems,"*
    > in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), October 2012.
- **Repository Policy**: Raw dataset files are ignored via `.gitignore` and are not committed to Git.

---

## 3. Sensor & Calibration Parameters

### Camera Intrinsics (Freiburg 2 RGB-D)
- **Resolution:** 640 $\times$ 480 pixels ($307,200\text{ px}$)
- **Focal Length:** $f_x = 520.9\text{ px}, f_y = 521.0\text{ px}$
- **Principal Point:** $c_x = 325.1\text{ px}, c_y = 249.7\text{ px}$
- **Distortion Coefficients:** $k_1 = 0.2312, k_2 = -0.7849, p_1 = -0.0033, p_2 = -0.0001, k_3 = 0.9172$
- **Distortion Handling**: Per official TUM documentation, depth images are pre-registered to the color camera using the OpenNI PrimeSense driver; pinhole back-projection uses $(f_x, f_y, c_x, c_y)$ without separate radial lens undistortion.
- **Depth Scale Factor:** `5000.0`
  $$\text{Depth (meters)} = \frac{\text{raw\_uint16}}{5000.0}$$
- **Valid Depth Range:** $[0.40, 5.00]\text{ m}$ (near-field $<0.4\text{ m}$ and far-field $>5.0\text{ m}$ rejected).
- **Obstacle Height Filter:** $Z_b \in [0.05, 1.20]\text{ m}$ (floor $<0.05\text{ m}$ and ceiling $>1.20\text{ m}$ rejected).

### Camera Extrinsics (Mounting on Pioneer 3-DX)
- **Longitudinal Offset ($d_x$):** $+0.15\text{ m}$ (forward of robot rotation center `base_link`)
- **Lateral Offset ($d_y$):** $0.00\text{ m}$ (centered)
- **Height above Ground ($d_z$):** $+0.40\text{ m}$ (elevation above ground surface)
- **Orientation:** $0^\circ$ pitch / $0^\circ$ yaw (aligned horizontally with vehicle forward direction).
- **Calibration Provenance Disclosure**:
  > [!IMPORTANT]
  > Mounting offsets ($d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}$, zero pitch/yaw) are **project-defined engineering assumptions** derived from the physical chassis dimensions of the Pioneer 3-DX top deck. TUM does **not** publish an official rigid-body extrinsic transform connecting the MoCap reflective marker tree to the Kinect camera optical center. Distinguishing externally verified parameters from engineering assumptions is required for scientific integrity.

---

## 4. File Structure & Verified Cryptographic Provenance
Local repository layout:
```
data/tum_rgbd/
├── groundtruth.txt       # High-frequency (100 Hz) MoCap ground truth poses (33,188 poses, 2,262,149 bytes)
├── depth.txt             # Official depth frame timestamp index (2,075 depth frames, 95,535 bytes)
├── manifest.json         # Cryptographic manifest of selectively extracted authentic depth frames
├── depth/                # Authentic 16-bit single-channel depth PNG images extracted from official archive
│   ├── 1311877812.987032.png
│   ├── 1311877830.276341.png
│   ├── 1311877860.973057.png
│   ├── 1311877892.876368.png
│   └── 1311877928.049984.png
└── rgbd_dataset_freiburg2_pioneer_slam2.tgz  # Full official sequence archive (1,183,281,970 bytes)
```

### Verified File Provenance & Digest Integrity
- **Official Sequence Archive (`rgbd_dataset_freiburg2_pioneer_slam2.tgz`)**:
  - **Source URL**: `https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.tgz`
  - **Status**: AUTHENTIC (Acquired via resumable multi-threaded chunk downloader).
  - **File Size**: $1,183,281,970\text{ bytes}$ ($\approx 1.10\text{ GB}$).
  - **SHA-256 Digest**: `67f6c8458e7d9010e0a816ee893f862d27da99fa9d86a711b8c70c4b84e1a077` (locally computed verification digest).
  - **Upstream Integrity Note**: TUM does not publish official `.sha256` or `.md5` checksum files on the sequence webpage. Upstream authenticity is established by server Content-Length ($1,183,281,970\text{ bytes}$), HTTP ETag `"46877332-4ae2a2e91a900"`, and internal DEFLATE/tar stream member verification across all 4,195 entries.
- **Ground Truth Trajectory (`groundtruth.txt`)**:
  - **Status**: AUTHENTIC (Downloaded directly from official TUM webshare server).
  - **File Size**: $2,262,149\text{ bytes}$.
  - **SHA-256**: `bd57b2c9ce6776fe0d39c7795d7f7789bf206e4eaa58e04aa216090b7a55d08a`.
  - **Kinematics**: 33,188 poses at 100 Hz MoCap, duration 115.632 s, driven path 23.326 m.
- **Official Depth Index (`depth.txt`)**:
  - **Status**: AUTHENTIC (Extracted from official archive).
  - **File Size**: $95,535\text{ bytes}$.
  - **SHA-256**: `e09cfeb53e70cf271a94cb2db3ae06d203fcf93ff181bfb8ecde08b8b7d9df48`.
  - **Records**: 2,075 timestamped depth frame records.

---

## 5. Audited Depth-Pixel Statistics Reconciliation
Every 16-bit depth image has exactly $480 \times 640 = 307,200\text{ pixels}$.
- **Reconciliation Check 1**: $\text{zero\_depth\_pixels} + \text{out\_of\_range\_pixels} + \text{valid\_depth\_pixels} = 307,200$.
- **Reconciliation Check 2**: $\text{floor\_rejected\_points} + \text{ceiling\_rejected\_points} + \text{retained\_obstacle\_points} = \text{valid\_depth\_pixels}$.

| Frame Filename | File Size | Valid Pixels ($[0.4, 5.0]\text{m}$) | Zero Depth ($0\text{m}$) | Out-of-Range ($<0.4\text{m}$ / $>5.0\text{m}$) | Invalid Total | Floor Rej. ($<0.05\text{m}$) | Ceiling Rej. ($>1.20\text{m}$) | Retained Obstacles | Check 1 / Check 2 |
|---|---|---|---|---|---|---|---|---|---|
| `1311877812.987032.png` | $141,882\text{ B}$ | 257,151 (83.71%) | 49,783 (16.21%) | 266 (0.09%) | 50,049 (16.29%) | 70,696 (27.49%) | 62,220 (24.20%) | 124,235 (48.31%) | **PASS / PASS** |
| `1311877830.276341.png` | $145,655\text{ B}$ | 136,385 (44.40%) | 55,358 (18.02%) | 115,457 (37.58%) | 170,815 (55.60%) | 28,683 (21.03%) | 39,635 (29.06%) | 68,067 (49.91%) | **PASS / PASS** |
| `1311877860.973057.png` | $138,680\text{ B}$ | 152,383 (49.60%) | 52,246 (17.01%) | 102,571 (33.39%) | 154,817 (50.40%) | 26,242 (17.22%) | 36,122 (23.71%) | 90,019 (59.07%) | **PASS / PASS** |
| `1311877892.876368.png` | $156,120\text{ B}$ | 193,370 (62.95%) | 55,595 (18.10%) | 58,235 (18.96%) | 113,830 (37.05%) | 45,407 (23.48%) | 10,418 (5.39%) | 137,545 (71.13%) | **PASS / PASS** |
| `1311877928.049984.png` | $135,951\text{ B}$ | 256,905 (83.63%) | 50,295 (16.37%) | 0 (0.00%) | 50,295 (16.37%) | 71,165 (27.70%) | 56,103 (21.84%) | 129,637 (50.46%) | **PASS / PASS** |
| **Total / Aggregate** | **718,288 B** | **996,194** | **263,277** | **276,529** | **539,806** | **242,193** | **204,498** | **549,503** | **PASS / PASS** |

---

## 6. Ground-Truth Timestamp Association & Nearest MoCap Residuals
Projection poses are evaluated via SLERP quaternion interpolation at the exact query depth timestamp $t_{\text{depth}}$ between the two enclosing 100 Hz MoCap bracket measurements. The time offset to the nearest raw discrete MoCap sample is recorded as the **nearest MoCap sample residual**:

| Frame Timestamp $t_d$ | Nearest MoCap Sample $t_{\text{mocap}}$ | Nearest MoCap Residual $|t_d - t_{\text{mocap}}|$ | Bracket Interval $[t_{\text{before}}, t_{\text{after}}]$ | Interval Duration $\Delta t$ | SLERP Factor $\alpha$ | Interpolated Pose $(x, y, \theta)$ |
|---|---|---|---|---|---|---|
| `1311877812.987032` | `1311877812.987700` | **$0.668\text{ ms}$** ($0.000668\text{ s}$) | `[1311877812.9843, 1311877812.9877]` | $3.4\text{ ms}$ | 0.8035 | $(-2.258\text{m}, -2.380\text{m}, 151.08^\circ)$ |
| `1311877830.276341` | `1311877830.277000` | **$0.659\text{ ms}$** ($0.000659\text{ s}$) | `[1311877830.2737, 1311877830.2770]` | $3.3\text{ ms}$ | 0.8003 | $(-2.694\text{m}, -0.211\text{m}, -7.88^\circ)$ |
| `1311877860.973057` | `1311877860.973600` | **$0.557\text{ ms}$** ($0.000557\text{ s}$) | `[1311877860.9703, 1311877860.9736]` | $3.3\text{ ms}$ | 0.8355 | $(0.457\text{m}, -1.745\text{m}, 92.48^\circ)$ |
| `1311877892.876368` | `1311877892.874800` | **$1.568\text{ ms}$** ($0.001568\text{ s}$) | `[1311877892.8748, 1311877892.8780]` | $3.2\text{ ms}$ | 0.4900 | $(1.900\text{m}, 0.943\text{m}, 38.26^\circ)$ |
| `1311877928.049984` | `1311877928.050000` | **$0.016\text{ ms}$** ($0.000016\text{ s}$) | `[1311877928.0467, 1311877928.0500]` | $3.3\text{ ms}$ | 0.9952 | $(-1.828\text{m}, 2.953\text{m}, 20.64^\circ)$ |

> [!WARNING]
> **Timestamp & Synchronization Disclosure**:
> The nearest MoCap sample residuals ($0.016\text{--}1.568\text{ ms}$) measure the temporal distance from the query optical frame to the closest discrete 100 Hz MoCap trajectory entry. These residuals reflect the sampling granularity of the 100 Hz motion capture system ($\Delta t_{\text{sample}} \approx 10\text{ ms}$); they do **NOT** prove physical hardware clock synchronization between the Kinect sensor and the MoCap tracking computer. Poses used for spatial mapping are synthesized via SLERP interpolation across the bounding interval $[t_{\text{before}}, t_{\text{after}}]$.

---

## 7. Spatial Coordinates, Coordinate-Frame Provenance & Grid Semantics

### 7.1 Coordinate Frames & Provenance Chain
1. **Camera Optical Frame (`camera_depth_optical_frame`)**: Standard OpenCV/OpenNI frame (+X right, +Y down, +Z forward). Right-handed, origin at Kinect IR camera center.
2. **Robot Base Link (`base_link`)**: REP-105 standard ground robot frame (+X forward, +Y left, +Z up). Right-handed, origin at vehicle rotation center on ground.
   $$\begin{bmatrix} X_b \\ Y_b \\ Z_b \end{bmatrix} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} + \begin{bmatrix} +0.15 \\ 0.00 \\ +0.40 \end{bmatrix}$$
   Rotation matrix has determinant $\det(\mathbf{R}) = +1$ (orthonormal SO(3)).
3. **TUM Motion Capture World Frame (`tum_mocap_world`)**: Laboratory Euclidean coordinate frame (+Z up normal to floor, X and Y horizontal along room calibration axes). Origin set by TUM during 10-camera infrared system calibration.
   > [!IMPORTANT]
   > The TUM MoCap world frame is an **indoor laboratory coordinate system**, NOT a geographic East-North-Up (REP-103 ENU) frame. There is no evidence of geographic compass alignment.
4. **Navigation & GridMap Frame (`grid_map_world`)**: 2D discrete grid placed directly in the horizontal $(t_x, t_y)$ plane of the TUM MoCap tracking volume with origin at continuous metric coordinate $(-5.0, -5.0)\text{ m}$.

### 7.2 Grid Semantics Disclosure: Endpoint Rasterization vs Occupancy Mapping
> [!CAUTION]
> **Grid Semantics Disclosure**:
> The `GridMap` output generated by `insert_points_into_grid_map()` represents **projected obstacle endpoint rasterization (point hits)**:
> - Cells with value `1` contain one or more projected obstacle 3D points.
> - Cells with value `0` are **unobserved / unmarked**, NOT confirmed navigable free space.
> - The current pipeline does **NOT** perform line-of-sight ray tracing or ray marching (Bresenham clearing) to clear free space between the sensor origin and the obstacle hit, nor does it maintain a probabilistic log-odds occupancy state.

### 7.3 Spatial Bounds & Boundary Clipping
- **GridMap Arena Configuration**: $10.0\text{ m} \times 10.0\text{ m}$ ($200 \times 200$ cells, resolution $0.05\text{ m}$, origin $(-5.0, -5.0)\text{ m}$, bounds $[-5.0, 5.0]\text{ m} \times [-5.0, 5.0]\text{ m}$).
- **Robot Trajectory Extents**: $X \in [-2.829, 2.154]\text{ m}$ (span: $4.98\text{ m}$), $Y \in [-2.381, 2.958]\text{ m}$ (span: $5.34\text{ m}$).
- **Obstacle Observation Extents**: Forward-facing Kinect camera ($5.0\text{ m}$ range) viewing peripheral walls extends obstacle coordinates to $X \in [-7.089, 7.044]\text{ m}$, $Y \in [-6.184, 6.511]\text{ m}$.
- **Boundary Handling**: Obstacle points exceeding $[-5.0, 5.0]\text{ m}$ are safely clipped without memory overflow or map corruption.

| Frame Filename | World Envelope $[X_{\min}, X_{\max}] \times [Y_{\min}, Y_{\max}]\text{ m}$ | Retained Obstacles | Points In-Bounds | Points Clipped | Clipping % | New Grid Cells Marked |
|---|---|---|---|---|---|---|
| `1311877812.987032.png` | $[-7.089, -3.475] \times [-2.353, 1.571]\text{ m}$ | 124,235 | 44,700 | 79,535 | 64.02% | +1,502 |
| `1311877830.276341.png` | $[-2.637, 2.290] \times [-1.905, 0.490]\text{ m}$ | 68,067 | 68,067 | 0 | 0.00% | +3,857 |
| `1311877860.973057.png` | $[-0.963, 1.834] \times [-1.687, 3.238]\text{ m}$ | 90,019 | 85,292 | 4,727 | 5.25% | +1,378 |
| `1311877892.876368.png` | $[2.818, 7.044] \times [1.194, 4.850]\text{ m}$ | 137,545 | 136,138 | 1,407 | 1.02% | +685 |
| `1311877928.049984.png` | $[-0.296, 1.738] \times [2.521, 6.511]\text{ m}$ | 129,637 | 102,159 | 27,478 | 21.20% | +1,508 |
| **Total / Aggregate** | **Full Span: $[-7.089, 7.044] \times [-6.184, 6.511]\text{ m}$** | **549,503** | **436,356 (79.41%)** | **113,147 (20.59%)** | **20.59%** | **8,930 occupied cells** |

---

## 8. Acquisition & Evaluation Commands
To download archive and extract authentic frames:
```powershell
.\.venv\Scripts\python.exe scripts/download_tum_archive.py --workers 8 --num-frames 5
```
To evaluate end-to-end dataset projection, mapping, and tracking:
```powershell
.\.venv\Scripts\python.exe scripts/run_dataset_evaluation.py
```
To evaluate 6-DoF extrinsic calibration sensitivity and mapping robustness:
```powershell
.\.venv\Scripts\python.exe scripts/run_calibration_sensitivity.py
```
To execute automated unit and integration tests:
```powershell
.\.venv\Scripts\python.exe -m pytest tests/unit/test_dataset_adapter.py tests/unit/test_calibration_sensitivity.py tests/integration/test_dataset_integration.py -v
```

---

## 9. Milestone 7.5: Extrinsic Calibration Sensitivity & Mapping Robustness Analysis

### 9.1 Provenance Status & Baseline Hypothesis
The baseline camera-to-body extrinsics ($d_x = +0.15\text{ m}, d_y = 0.00\text{ m}, d_z = +0.40\text{ m}$, pitch = $0.0^\circ$, yaw = $0.0^\circ$, roll = $0.0^\circ$) represent an **engineering hypothesis** based on the Pioneer 3-DX chassis structure. Because TUM does not publish a physical rigid-body marker-to-sensor extrinsic calibration, Milestone 7.5 conducts an empirical 6-DoF sensitivity analysis around this baseline to bound mapping uncertainty.

### 9.2 Systematic 6-DoF Perturbation Results Summary
Evaluated across all 5 authentic TUM depth frames ($996,194$ valid depth pixels):

| Parameter | Baseline Value | Sweep Range | Max 3D Disp ($m$) | GridMap IoU Range (%) | Retained Points Range | Primary Failure Mode / Distortion |
|---|---|---|---|---|---|---|
| **$d_x$ (Forward)** | $+0.15\text{ m}$ | $[0.05, 0.25]\text{ m}$ ($\pm 0.10\text{ m}$) | $0.1000\text{ m}$ | $82.40\%\text{--}100.0\%$ | $549,503$ (constant) | Rigid longitudinal shift; 0% height impact |
| **$d_y$ (Lateral)** | $0.00\text{ m}$ | $[-0.10, +0.10]\text{ m}$ ($\pm 0.10\text{ m}$) | $0.1000\text{ m}$ | $81.87\%\text{--}100.0\%$ | $549,503$ (constant) | Rigid lateral shift; 0% height impact |
| **$d_z$ (Height)** | $+0.40\text{ m}$ | $[0.30, 0.50]\text{ m}$ ($\pm 0.10\text{ m}$) | $0.1000\text{ m}$ | $68.36\%\text{--}100.0\%$ | $430,344\text{--}796,383$ | Vertical window shift; $\pm 45\%$ obstacle count variation |
| **Pitch (Tilt)** | $0.0^\circ$ | $[-5.0^\circ, +5.0^\circ]$ | $0.4753\text{ m}$ | $6.23\%\text{--}100.0\%$ | $367,736\text{--}768,450$ | Severe height slicing failure ($+5^\circ$ collapses IoU to $6.23\%$) |
| **Yaw (Pan)** | $0.0^\circ$ | $[-5.0^\circ, +5.0^\circ]$ | $0.5017\text{ m}$ | $66.39\%\text{--}100.0\%$ | $549,503$ (constant) | Range-dependent lateral arc error ($\approx r \cdot \theta$) |
| **Roll (Bank)** | $0.0^\circ$ | $[-5.0^\circ, +5.0^\circ]$ | $0.3111\text{ m}$ | $83.94\%\text{--}100.0\%$ | $548,559\text{--}554,749$ | Asymmetric lateral height tilt across sensor FOV |

### 9.3 Key Engineering Insights
1. **Separation of Effects**:
   - Translations $d_x, d_y$ and rotation $\text{yaw}$ have **zero impact** on height slicing (retaining exactly $549,503$ points).
   - Pitch and $d_z$ directly displace points relative to the $[0.05, 1.20]\text{ m}$ height boundaries, causing severe variations in obstacle point counts.
2. **Deceptive Nature of Cell Counts**:
   - Cell count alone is **not** an indicator of calibration accuracy. A $+5.0^\circ$ pitch reduces occupied cells by $93\%$ because obstacles plunge below the floor cutoff, while a $-5.0^\circ$ pitch increases occupied cells by $20\%$ by elevating floor points into the obstacle band.
3. **Artifact Outputs**:
   - Metrics Report: [`outputs/metrics/calibration_sensitivity_summary.json`](../../outputs/metrics/calibration_sensitivity_summary.json)
   - Diagnostic Figure: [`outputs/figures/calibration_sensitivity_diagnostic.png`](../../outputs/figures/calibration_sensitivity_diagnostic.png)

