# 05 - Model Learning and Training

## 1. Learning Paradigms Overview

The autonomous navigation system supports two complementary learning paradigms:
1. **Supervised & Imitation Learning (Primary Policy & Perception)**: Learns robust representations and expert trajectories from human/expert demonstrations (Behavioral Cloning with trajectory regularization).
2. **Deep Reinforcement Learning (Exploration & Edge-Case Policy)**: Refines collision avoidance and aggressive cornering policies via closed-loop rewards in simulated environments.

```mermaid
flowchart LR
    subgraph DataEngine["Data Generation & Collection"]
        EXPERT["Expert Demonstrations (Auto/Human)"]
        SIM["Simulation Rollouts (CARLA/Gym)"]
    end

    subgraph TrainingPipeline["Learning & Optimization Loop"]
        ENC["Vision & State Encoder (Backbone)"]
        FUS["Temporal Fusion (GRU / Transformer)"]
        HEADS["Multi-Head Predictor (Waypoints / Affordance)"]
        LOSS["Loss Formulation (Huber + Collision + Regularization)"]
    end

    subgraph PolicyOutput["Policy Execution"]
        TRAJ["Predicted Trajectory Horizon w_{1:H}"]
        CONTROLLER["Kinematic Tracking Controller"]
    end

    DataEngine --> ENC
    ENC --> FUS
    FUS --> HEADS
    HEADS --> LOSS
    LOSS -.->|Gradient Backprop (AdamW)| ENC
    HEADS --> TRAJ
    TRAJ --> CONTROLLER
```

---

## 2. Neural Architecture

The primary learning model is a **Conditional Temporal Trajectory Network (CTTN)**:

### 2.1 Sensory Encoder
- **Visual Stream**: Convolutional Backbone (ResNet-34 / MobileNetV3 / EfficientNet) producing a spatial feature map $\mathbf{F}_{\text{vis}} \in \mathbb{R}^{C \times H' \times W'}$.
- **Kinematic State Stream**: Fully connected projection of current velocity, acceleration, and navigational command (e.g., *Turn Left*, *Turn Right*, *Straight*, *Follow Lane*):
  $$\mathbf{e}_{\text{state}} = \text{MLP}([v, \dot{v}, \omega, \mathbf{c}_{\text{cmd}}])$$

### 2.2 Temporal Fusion & Aggregation
- Features are flattened and processed by a temporal sequence model (Bidirectional GRU or Transformer Encoder) across a historical window $k \in [t - K, t]$ to capture object motion and velocity derivatives:
  $$\mathbf{h}_t = \text{GRU}(\mathbf{z}_t, \mathbf{h}_{t-1})$$

### 2.3 Output Heads
The network does not directly output raw unconstrained steering/throttle commands (which causes catastrophic compounding drift). Instead, it predicts a **spatiotemporal waypoint horizon**:
$$\hat{\mathbf{W}} = \{(\hat{x}_i, \hat{y}_i, \hat{v}_i)\}_{i=1}^H \quad \text{for future time steps } t_i = t + i \Delta t$$
These waypoints are then stably tracked by the deterministic Controller and vetted by the Safety Interceptor.

---

## 3. Loss Functions & Objective Formulations

### 3.1 Waypoint Prediction Loss (Smooth Huber Loss)
To prevent sensitivity to outlier trajectory noise in expert demonstrations:
$$\mathcal{L}_{\text{wp}} = \frac{1}{H} \sum_{i=1}^H \text{Huber}_\delta \left(\hat{\mathbf{w}}_i - \mathbf{w}_i^{\text{gt}}\right)$$
$$\text{Huber}_\delta(e) = \begin{cases} \frac{1}{2} e^2 & \text{if } |e| \le \delta \\ \delta (|e| - \frac{1}{2}\delta) & \text{otherwise} \end{cases}$$

### 3.2 Curvature & Acceleration Regularization
To ensure kinematically feasible trajectories without sharp jerks:
$$\mathcal{L}_{\text{smooth}} = \sum_{i=2}^{H-1} \|\hat{\mathbf{w}}_{i+1} - 2\hat{\mathbf{w}}_i + \hat{\mathbf{w}}_{i-1}\|_2^2$$

### 3.3 Total Loss Function
$$\mathcal{L}_{\text{total}} = \mathcal{L}_{\text{wp}} + \lambda_1 \mathcal{L}_{\text{smooth}} + \lambda_2 \mathcal{L}_{\text{velocity}} + \lambda_3 \mathcal{L}_{\text{collision}}$$
where $\mathcal{L}_{\text{collision}}$ penalizes waypoints intersecting occupied cells in the predicted occupancy grid.

---

## 4. Reinforcement Learning Reward Mechanics (Optional Sub-policy)

When training policies via Model-Free RL (PPO or SAC), the reward function at each step $t$ is formulated as:
$$R_t = R_{\text{progress}} + R_{\text{center}} - R_{\text{jerk}} - R_{\text{violation}} - R_{\text{collision}}$$

- **Progress Reward**: $R_{\text{progress}} = v_t \cos(\Delta \theta)$, incentivizing forward velocity aligned with the road.
- **Lane Centering**: $R_{\text{center}} = - |d_{\text{center}}|^2$, penalizing deviations from lane centerline.
- **Smoothness Penalty**: $R_{\text{jerk}} = \gamma_s |\delta_t - \delta_{t-1}|^2 + \gamma_a |a_t - a_{t-1}|^2$.
- **Collision Penalty**: $R_{\text{collision}} = -100$ upon impact or leaving traversable roadway, triggering early episode termination.

---

## 5. Training Strategy & Optimization

- **Optimizer**: AdamW with weight decay $\eta = 10^{-4}$.
- **Learning Rate Schedule**: Cosine Annealing with Warm Restarts ($\text{lr}_{\text{max}} = 3 \times 10^{-4}$, $\text{lr}_{\text{min}} = 10^{-6}$).
- **Batch Size**: 64 to 128 multi-frame sequences.
- **Hardware Acceleration**: Mixed precision (`torch.cuda.amp.autocast`) with gradient clipping ($\|\mathbf{g}\| \le 1.0$) to stabilize recurrent units.

