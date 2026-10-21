"""Configuration management for autonomous navigation system."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple


@dataclass
class MapConfig:
    """Configuration for 2D occupancy grid map."""
    width: int = 60
    height: int = 40
    resolution: float = 0.5  # meters per grid cell
    inflation_radius: float = 0.8  # obstacle safety clearance in meters


@dataclass
class RobotConfig:
    """Kinematic parameters and physical limits of the robot."""
    max_velocity: float = 2.0  # m/s
    min_velocity: float = 0.0  # m/s
    max_angular_velocity: float = 1.5  # rad/s
    max_acceleration: float = 1.0  # m/s^2
    max_angular_acceleration: float = 2.0  # rad/s^2
    wheelbase: float = 0.5  # meters
    track_width: float = 0.4  # meters


@dataclass
class PlannerConfig:
    """A* path planning parameters."""
    algorithm: str = "astar"
    heuristic: str = "euclidean"
    allow_diagonal: bool = True
    diagonal_cost: float = 1.41421356
    straight_cost: float = 1.0


@dataclass
class ControllerConfig:
    """Trajectory tracking controller parameters."""
    type: str = "pure_pursuit"
    lookahead_distance: float = 0.8  # meters
    min_lookahead_distance: float = 0.5
    max_lookahead_distance: float = 2.0
    kp_linear: float = 1.0
    kp_angular: float = 2.0
    target_velocity: float = 1.5  # m/s


@dataclass
class VisionExtensionConfig:
    """Perception and vision pipeline configuration settings."""
    enabled: bool = False
    camera_type: str = "orthographic"
    image_width: int = 640
    image_height: int = 480
    color_format: str = "RGB"
    blur_kernel_size: int = 5
    blur_sigma: float = 1.0
    threshold_method: str = "otsu"  # "otsu", "binary", "adaptive"
    binary_threshold: int = 127
    morphology_kernel_size: int = 3
    min_obstacle_area_pixels: float = 20.0
    max_obstacle_area_pixels: float = 250000.0
    camera_fov_degrees: float = 90.0
    sensor_range_meters: float = 10.0


@dataclass
class DWAConfig:
    """Dynamic Window Approach local planner parameters."""
    enabled: bool = False
    predict_time: float = 2.0  # seconds forward trajectory prediction horizon
    dt: float = 0.15  # trajectory simulation integration time step
    v_samples: int = 7  # velocity discretization samples
    omega_samples: int = 15  # angular velocity discretization samples
    alpha_heading: float = 0.25  # weight for goal heading alignment
    beta_clearance: float = 0.50  # weight for obstacle clearance
    gamma_velocity: float = 0.25  # weight for forward speed
    safety_margin: float = 0.15  # additional obstacle clearance buffer in meters
    max_lookahead_dist: float = 3.5  # local sub-goal lookahead clamp


@dataclass
class SensorConfig:
    """Configuration for simulated noisy sensors (odometry, IMU, GNSS)."""
    odometry_noise_std: float = 0.05       # standard deviation of linear velocity noise (m/s)
    odometry_scale_factor: float = 1.0     # systematic wheel radius/scaling error
    imu_rate_noise_std: float = 0.02       # gyro angular rate noise standard deviation (rad/s)
    imu_bias_init: float = 0.01            # initial gyro bias (rad/s)
    imu_bias_drift_std: float = 0.001      # gyro bias drift standard deviation (rad/s/sqrt(s))
    gnss_pos_noise_std: float = 0.3        # GNSS position noise standard deviation (meters)
    gnss_update_interval: float = 1.0      # interval between GNSS position fixes (seconds, e.g. 1 Hz)
    gnss_outages: List[Tuple[float, float]] = field(default_factory=list)  # list of (t_start, t_end) outage intervals
    seed: Optional[int] = 42               # deterministic RNG seed


@dataclass
class EKFConfig:
    """Extended Kalman Filter process and measurement noise covariance parameters."""
    q_pos: float = 0.05                    # process noise standard deviation for position (m)
    q_theta: float = 0.02                  # process noise standard deviation for heading (rad)
    q_v: float = 0.1                       # process noise standard deviation for linear velocity (m/s)
    q_bias: float = 0.001                  # process noise standard deviation for gyro bias (rad/s)
    r_odom: float = 0.05                   # measurement noise standard deviation for odometry (m/s)
    r_gnss: float = 0.3                    # measurement noise standard deviation for GNSS position (m)
    initial_pos_std: float = 0.5           # initial standard deviation for position states (m)
    initial_theta_std: float = 0.1         # initial standard deviation for heading (rad)
    initial_v_std: float = 0.2             # initial standard deviation for velocity (m/s)
    initial_bias_std: float = 0.05         # initial standard deviation for gyro bias (rad/s)


@dataclass
class SimulationConfig:
    """Master simulation configuration aggregating all subsystem settings."""
    dt: float = 0.1  # simulation step time in seconds
    max_steps: int = 1000
    collision_radius: float = 0.4  # meters
    goal_tolerance: float = 0.5  # meters
    start_pose: Tuple[float, float, float] = (5.0, 5.0, 0.0)  # (x, y, theta)
    goal_position: Tuple[float, float] = (25.0, 15.0)  # (x, y)
    nav_mode: str = "pure_pursuit"  # "pure_pursuit" (baseline) or "dwa" (local avoidance)
    state_feedback: str = "ground_truth"  # "ground_truth" (baseline), "ekf" (estimated), or "dead_reckoning"
    map: MapConfig = field(default_factory=MapConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    dwa: DWAConfig = field(default_factory=DWAConfig)
    vision_extension: VisionExtensionConfig = field(default_factory=VisionExtensionConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    ekf: EKFConfig = field(default_factory=EKFConfig)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> SimulationConfig:
        """Create SimulationConfig from dictionary."""
        sim_data = data.get("simulation", {})
        map_data = data.get("map", {})
        robot_data = data.get("robot", {})
        planner_data = data.get("planner", {})
        controller_data = data.get("controller", {})
        dwa_data = data.get("dwa", {})
        vision_data = data.get("vision_extension", {})
        sensors_data = data.get("sensors", {})
        ekf_data = data.get("ekf", {})

        nav_mode = sim_data.get("nav_mode", "pure_pursuit")
        if "nav_mode" in data:
            nav_mode = data["nav_mode"]

        state_feedback = sim_data.get("state_feedback", "ground_truth")
        if "state_feedback" in data:
            state_feedback = data["state_feedback"]

        # Parse gnss_outages if present
        if "gnss_outages" in sensors_data and sensors_data["gnss_outages"]:
            sensors_data["gnss_outages"] = [tuple(pair) for pair in sensors_data["gnss_outages"]]

        return cls(
            dt=sim_data.get("dt", 0.1),
            max_steps=sim_data.get("max_steps", 1000),
            collision_radius=sim_data.get("collision_radius", 0.4),
            goal_tolerance=sim_data.get("goal_tolerance", 0.5),
            start_pose=tuple(sim_data.get("start_pose", (5.0, 5.0, 0.0))),
            goal_position=tuple(sim_data.get("goal_position", (25.0, 15.0))),
            nav_mode=nav_mode,
            state_feedback=state_feedback,
            map=MapConfig(**map_data),
            robot=RobotConfig(**robot_data),
            planner=PlannerConfig(**planner_data),
            controller=ControllerConfig(**controller_data),
            dwa=DWAConfig(**dwa_data),
            vision_extension=VisionExtensionConfig(**vision_data),
            sensors=SensorConfig(**sensors_data),
            ekf=EKFConfig(**ekf_data),
        )


def load_config(config_path: str | Path | None = None) -> SimulationConfig:
    """Load configuration from a JSON file, or return default configuration."""
    if config_path is None:
        return SimulationConfig()

    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file not found at: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    return SimulationConfig.from_dict(data)
