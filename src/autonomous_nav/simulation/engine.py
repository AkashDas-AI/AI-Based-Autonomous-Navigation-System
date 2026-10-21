"""Closed-loop 2D autonomous navigation simulation engine."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.navigation.controller import PurePursuitController
from autonomous_nav.navigation.dwa import DWAPlanner, DWASolution
from autonomous_nav.navigation.waypoint_manager import WaypointManager
from autonomous_nav.planning.astar import AStarPlanner
from autonomous_nav.sensors.sensor_models import SensorSuite, SensorMeasurements
from autonomous_nav.estimation.ekf import ExtendedKalmanFilter
from autonomous_nav.evaluation.metrics import (
    calculate_estimation_errors,
    calculate_tracking_errors,
)
from autonomous_nav.simulation.collision import CollisionChecker
from autonomous_nav.simulation.robot import RobotModel, RobotState


@dataclass
class SimulationResult:
    """Outcome and recorded trajectory of a simulation run."""
    success: bool
    collision: bool
    timeout: bool
    goal_reached: bool
    planning_failed: bool
    termination_reason: str  # "goal_reached", "collision", "timeout", "planning_failure"
    steps_taken: int
    simulated_time: float
    wall_clock_time: float
    trajectory: List[RobotState] = field(default_factory=list)
    planned_path: List[Tuple[float, float]] = field(default_factory=list)
    final_distance_to_goal: float = 0.0
    dwa_failures: int = 0
    collided_with_dynamic: bool = False
    estimated_trajectory: List[RobotState] = field(default_factory=list)
    feedback_trajectory: List[RobotState] = field(default_factory=list)
    estimation_metrics: dict = field(default_factory=dict)
    tracking_metrics: dict = field(default_factory=dict)
    state_feedback: str = "ground_truth"
    estimator_latencies_ms: List[float] = field(default_factory=list)
    step_latencies_ms: List[float] = field(default_factory=list)

    @property
    def estimator_compute_mean_ms(self) -> float:
        return float(np.mean(self.estimator_latencies_ms)) if self.estimator_latencies_ms else 0.0

    @property
    def estimator_compute_p95_ms(self) -> float:
        return float(np.percentile(self.estimator_latencies_ms, 95)) if self.estimator_latencies_ms else 0.0

    @property
    def step_latency_mean_ms(self) -> float:
        return float(np.mean(self.step_latencies_ms)) if self.step_latencies_ms else 0.0

    @property
    def step_latency_p95_ms(self) -> float:
        return float(np.percentile(self.step_latencies_ms, 95)) if self.step_latencies_ms else 0.0

    @property
    def time_taken(self) -> float:
        """Backward-compatible alias for simulated_time."""
        return self.simulated_time


class SimulationEngine:
    """Orchestrates closed-loop autonomous navigation simulation runs."""

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config if config is not None else SimulationConfig()
        self.robot = RobotModel(self.config.robot)
        self.planner = AStarPlanner(
            allow_diagonal=self.config.planner.allow_diagonal,
            heuristic_type=self.config.planner.heuristic,
        )
        self.controller = PurePursuitController(self.config.controller)
        self.dwa_planner = DWAPlanner(self.config.dwa, self.config.robot)

        # Stateful episode variables (accessible for interactive visualization and step interface)
        self.grid_map: Optional[GridMap] = None
        self.inflated_map: Optional[GridMap] = None
        self.dynamic_obstacles: List[DynamicObstacle] = []
        self.start_pose: Tuple[float, float, float] = self.config.start_pose
        self.goal_position: Tuple[float, float] = self.config.goal_position
        self.planned_path: List[Tuple[float, float]] = []
        self.waypoint_mgr: Optional[WaypointManager] = None
        self.trajectory: List[RobotState] = []
        self.steps_taken: int = 0
        self.simulated_time: float = 0.0
        self.start_clock: float = 0.0
        self.elapsed_clock: float = 0.0
        self.is_done: bool = False
        self.goal_reached: bool = False
        self.collision_occurred: bool = False
        self.collided_with_dynamic: bool = False
        self.timeout_occurred: bool = False
        self.planning_failed: bool = False
        self.termination_reason: Optional[str] = None
        self.current_target: Optional[Tuple[float, float]] = None
        self.current_cmd: Tuple[float, float] = (0.0, 0.0)
        self.current_candidate_trajectory: List[Tuple[float, float]] = []
        self.dwa_safe: bool = True
        self.dwa_failures: int = 0

        # Sensors and Extended Kalman Filter state estimator
        self.sensors = SensorSuite(
            odometry_noise_std=self.config.sensors.odometry_noise_std,
            odometry_scale_factor=self.config.sensors.odometry_scale_factor,
            imu_rate_noise_std=self.config.sensors.imu_rate_noise_std,
            imu_bias_init=self.config.sensors.imu_bias_init,
            imu_bias_drift_std=self.config.sensors.imu_bias_drift_std,
            gnss_pos_noise_std=self.config.sensors.gnss_pos_noise_std,
            gnss_update_interval=self.config.sensors.gnss_update_interval,
            gnss_outages=self.config.sensors.gnss_outages,
            seed=self.config.sensors.seed,
        )
        self.ekf = ExtendedKalmanFilter(
            q_pos=self.config.ekf.q_pos,
            q_theta=self.config.ekf.q_theta,
            q_v=self.config.ekf.q_v,
            q_bias=self.config.ekf.q_bias,
            r_odom=self.config.ekf.r_odom,
            r_gnss=self.config.ekf.r_gnss,
            initial_pos_std=self.config.ekf.initial_pos_std,
            initial_theta_std=self.config.ekf.initial_theta_std,
            initial_v_std=self.config.ekf.initial_v_std,
            initial_bias_std=self.config.ekf.initial_bias_std,
        )
        self.estimated_trajectory: List[RobotState] = []
        self.feedback_trajectory: List[RobotState] = []
        self.gnss_measurements: List[Tuple[float, float]] = []
        self.gnss_fixes_processed: int = 0
        self.last_sensor_measurements: Optional[SensorMeasurements] = None
        self.estimator_latencies_ms: List[float] = []
        self.step_latencies_ms: List[float] = []

    def reset(
        self,
        grid_map: GridMap,
        start_pose: Tuple[float, float, float] | None = None,
        goal_position: Tuple[float, float] | None = None,
        precomputed_path: Optional[List[Tuple[float, float]]] = None,
        dynamic_obstacles: Optional[List[DynamicObstacle]] = None,
    ) -> bool:
        """Initialize an episode and compute the global planned path.

        Returns:
            bool: True if path was successfully found or provided; False on planning failure.
        """
        self.start_clock = time.perf_counter()
        self.grid_map = grid_map
        self.start_pose = start_pose if start_pose is not None else self.config.start_pose
        self.goal_position = goal_position if goal_position is not None else self.config.goal_position

        # Initialize dynamic moving obstacles
        if dynamic_obstacles is not None:
            self.dynamic_obstacles = list(dynamic_obstacles)
        else:
            self.dynamic_obstacles = []
        for obs in self.dynamic_obstacles:
            obs.reset()

        # Reset robot state
        self.robot.reset(x=self.start_pose[0], y=self.start_pose[1], theta=self.start_pose[2])
        self.trajectory = [self.robot.state]

        # Reset sensors and state estimator
        self.sensors.reset(self.config.sensors.seed)
        self.ekf.reset(
            x=self.start_pose[0],
            y=self.start_pose[1],
            theta=self.start_pose[2],
            v=0.0,
            bias_gyro=self.config.sensors.imu_bias_init,
        )
        initial_est = self.ekf.get_estimated_robot_state(timestamp=0.0)
        self.estimated_trajectory = [initial_est]
        if self.config.state_feedback == "ground_truth":
            self.feedback_trajectory = [self.robot.state]
        else:
            self.feedback_trajectory = [initial_est]
        self.gnss_measurements = []
        self.gnss_fixes_processed = 0
        self.last_sensor_measurements = None
        self.estimator_latencies_ms = []
        self.step_latencies_ms = []

        self.steps_taken = 0
        self.simulated_time = 0.0
        self.is_done = False
        self.goal_reached = False
        self.collision_occurred = False
        self.collided_with_dynamic = False
        self.timeout_occurred = False
        self.planning_failed = False
        self.termination_reason = None
        self.current_target = None
        self.current_cmd = (0.0, 0.0)
        self.current_candidate_trajectory = []
        self.dwa_safe = True
        self.dwa_failures = 0

        # Inflate map for planning clearance
        self.inflated_map = self.grid_map.inflate_obstacles(self.config.map.inflation_radius)

        # Plan trajectory
        if precomputed_path is not None:
            self.planned_path = precomputed_path
        else:
            self.planned_path = self.planner.plan_world(
                start_world=(self.start_pose[0], self.start_pose[1]),
                goal_world=(self.goal_position[0], self.goal_position[1]),
                grid_map=self.inflated_map,
            )
            # If inflated planning fails, attempt on raw grid map
            if self.planned_path is None:
                self.planned_path = self.planner.plan_world(
                    start_world=(self.start_pose[0], self.start_pose[1]),
                    goal_world=(self.goal_position[0], self.goal_position[1]),
                    grid_map=self.grid_map,
                )

        if not self.planned_path:
            self.planned_path = []
            self.planning_failed = True
            self.is_done = True
            self.termination_reason = "planning_failure"
            self.elapsed_clock = time.perf_counter() - self.start_clock
            self.waypoint_mgr = WaypointManager([self.goal_position])
            return False

        self.waypoint_mgr = WaypointManager(
            waypoints=self.planned_path,
            acceptance_radius=self.config.goal_tolerance * 1.5,
        )
        return True

    def replan(self) -> bool:
        """Replan trajectory from current robot state to goal (e.g. after interactive obstacle edits).

        Returns:
            bool: True if new path was found, False otherwise.
        """
        if self.grid_map is None:
            return False

        start_w = (self.robot.state.x, self.robot.state.y)
        goal_w = (self.goal_position[0], self.goal_position[1])

        # Check if the robot is currently in collision with the (possibly mutated) environment
        in_collision = CollisionChecker.check_disc_collision(
            start_w[0], start_w[1], self.config.collision_radius, self.grid_map
        )
        if not in_collision:
            for obs in self.dynamic_obstacles:
                if math.hypot(start_w[0] - obs.x, start_w[1] - obs.y) <= (self.config.collision_radius + obs.radius):
                    in_collision = True
                    self.collided_with_dynamic = True
                    break

        if in_collision:
            self.collision_occurred = True
            self.is_done = True
            self.termination_reason = "collision"
            self.planned_path = []
            self.waypoint_mgr = WaypointManager([self.goal_position])
            return False

        # If previously marked as colliding and obstacle was toggled off, clear collision flag
        if self.collision_occurred and not in_collision:
            self.collision_occurred = False
            self.collided_with_dynamic = False

        self.inflated_map = self.grid_map.inflate_obstacles(self.config.map.inflation_radius)

        new_path = self.planner.plan_world(start_w, goal_w, self.inflated_map)
        if new_path is None:
            new_path = self.planner.plan_world(start_w, goal_w, self.grid_map)

        if new_path:
            self.planned_path = new_path
            self.waypoint_mgr = WaypointManager(
                waypoints=self.planned_path,
                acceptance_radius=self.config.goal_tolerance * 1.5,
            )
            self.planning_failed = False
            # If the episode was previously halted due to planning failure or cleared collision, recover
            if self.termination_reason in ("planning_failure", "collision") and not self.collision_occurred:
                self.is_done = False
                self.termination_reason = None
            return True
        else:
            self.planned_path = []
            self.waypoint_mgr = WaypointManager([self.goal_position])
            self.planning_failed = True
            self.is_done = True
            self.termination_reason = "planning_failure"
            return False

    def step(self) -> Tuple[RobotState, Optional[str]]:
        """Advance the simulation by one discrete time step dt.

        Returns:
            Tuple[RobotState, Optional[str]]:
                Current robot state, and termination reason if finished (e.g. 'goal_reached', 'collision', 'timeout') or None if ongoing.
        """
        if self.is_done:
            return self.robot.state, self.termination_reason

        t_step_start = time.perf_counter()
        current_state = self.robot.state

        # Generate noisy simulated sensor measurements from true robot state
        meas = self.sensors.sample(
            true_state=current_state,
            dt=self.config.dt,
            current_time=self.simulated_time,
        )
        self.last_sensor_measurements = meas

        # Pure state estimator computation
        t_est_start = time.perf_counter()
        # Advance state estimator prediction (kinematic model with gyro measurement)
        self.ekf.predict(omega_imu=meas.omega_imu, dt=self.config.dt)

        # Odometry forward velocity measurement update
        self.ekf.update_odometry(meas.v_odom)

        # GNSS position measurement update (unless purely in dead-reckoning mode)
        if meas.gnss_valid and self.config.state_feedback != "dead_reckoning":
            self.ekf.update_gnss(meas.gnss_pos[0], meas.gnss_pos[1])
            self.gnss_measurements.append(meas.gnss_pos)
            self.gnss_fixes_processed += 1

        current_step_time = round((self.steps_taken + 1) * self.config.dt, 4)
        est_state = self.ekf.get_estimated_robot_state(timestamp=current_step_time)
        t_est_end = time.perf_counter()
        self.estimator_latencies_ms.append((t_est_end - t_est_start) * 1000.0)

        # Select state feedback: ground truth vs EKF estimated pose
        if self.config.state_feedback in ("ekf", "dead_reckoning"):
            feedback_state = est_state
        else:
            feedback_state = current_state

        # Check if destination reached
        if self.waypoint_mgr.is_goal_reached(
            feedback_state.x, feedback_state.y, tolerance=self.config.goal_tolerance
        ) or self.waypoint_mgr.is_goal_reached(
            current_state.x, current_state.y, tolerance=self.config.goal_tolerance
        ):
            self.goal_reached = True
            self.is_done = True
            self.termination_reason = "goal_reached"
            self.elapsed_clock = round(time.perf_counter() - self.start_clock, 5)
            self.step_latencies_ms.append((time.perf_counter() - t_step_start) * 1000.0)
            return current_state, self.termination_reason

        # Check if max steps exceeded (timeout)
        if self.steps_taken >= self.config.max_steps:
            self.timeout_occurred = True
            self.is_done = True
            self.termination_reason = "timeout"
            self.elapsed_clock = round(time.perf_counter() - self.start_clock, 5)
            self.step_latencies_ms.append((time.perf_counter() - t_step_start) * 1000.0)
            return current_state, self.termination_reason

        # Update active waypoint index based on feedback state
        self.waypoint_mgr.update(feedback_state.x, feedback_state.y)

        # Query lookahead target along global route
        lookahead_dist = (
            getattr(self.config.dwa, "max_lookahead_dist", 2.0)
            if self.config.nav_mode == "dwa"
            else self.config.controller.lookahead_distance
        )
        target = self.waypoint_mgr.get_lookahead_point(
            feedback_state.x,
            feedback_state.y,
            lookahead_distance=lookahead_dist,
        )
        if target is None:
            target = self.goal_position
        self.current_target = target

        dist_to_goal = self.waypoint_mgr.distance_to_goal(feedback_state.x, feedback_state.y)

        # Compute control command based on active navigation mode and feedback state
        if self.config.nav_mode == "dwa":
            dwa_sol = self.dwa_planner.compute_command(
                state=feedback_state,
                target_point=target,
                grid_map=self.grid_map,
                dynamic_obstacles=self.dynamic_obstacles,
                robot_radius=self.config.collision_radius,
                dt=self.config.dt,
            )
            v_cmd, omega_cmd = dwa_sol.v_cmd, dwa_sol.omega_cmd
            self.current_candidate_trajectory = dwa_sol.best_trajectory
            self.dwa_safe = dwa_sol.is_safe
            if not dwa_sol.is_safe:
                self.dwa_failures += 1
        else:
            v_cmd, omega_cmd = self.controller.compute_control(
                state=feedback_state,
                target_point=target,
                distance_to_goal=dist_to_goal,
            )
            self.current_candidate_trajectory = []
            self.dwa_safe = True

        self.current_cmd = (v_cmd, omega_cmd)

        # Step robot kinematics (ground truth physics integration)
        dt = self.config.dt
        new_state = self.robot.step(v_cmd, omega_cmd, dt)
        self.trajectory.append(new_state)
        self.steps_taken += 1
        self.simulated_time = round(self.steps_taken * dt, 4)

        self.estimated_trajectory.append(est_state)
        if self.config.state_feedback == "ground_truth":
            self.feedback_trajectory.append(new_state)
        else:
            self.feedback_trajectory.append(est_state)

        # Advance moving dynamic obstacles
        for obs in self.dynamic_obstacles:
            obs.step(dt)

        # Check static obstacle collision (using ground truth)
        if CollisionChecker.check_disc_collision(
            new_state.x, new_state.y, self.config.collision_radius, self.grid_map
        ):
            self.collision_occurred = True
            self.is_done = True
            self.termination_reason = "collision"
            self.elapsed_clock = round(time.perf_counter() - self.start_clock, 5)
            self.step_latencies_ms.append((time.perf_counter() - t_step_start) * 1000.0)
            return new_state, self.termination_reason

        # Check moving dynamic obstacle collision (using ground truth)
        for obs in self.dynamic_obstacles:
            dist_to_obs = math.hypot(new_state.x - obs.x, new_state.y - obs.y)
            if dist_to_obs <= (self.config.collision_radius + obs.radius):
                self.collision_occurred = True
                self.collided_with_dynamic = True
                self.is_done = True
                self.termination_reason = "collision"
                self.elapsed_clock = round(time.perf_counter() - self.start_clock, 5)
                self.step_latencies_ms.append((time.perf_counter() - t_step_start) * 1000.0)
                return new_state, self.termination_reason

        self.step_latencies_ms.append((time.perf_counter() - t_step_start) * 1000.0)
        return new_state, None

    def get_result(self) -> SimulationResult:
        """Return the current or final SimulationResult."""
        if self.start_clock > 0:
            elapsed = time.perf_counter() - self.start_clock if not self.is_done else self.elapsed_clock
        else:
            elapsed = 0.0

        if self.waypoint_mgr and self.trajectory:
            final_dist = self.waypoint_mgr.distance_to_goal(self.robot.state.x, self.robot.state.y)
        else:
            final_dist = 0.0

        success = self.goal_reached and not self.collision_occurred
        fb_traj = self.feedback_trajectory if self.feedback_trajectory else (
            list(self.trajectory) if self.config.state_feedback == "ground_truth" else list(self.estimated_trajectory)
        )
        est_metrics = calculate_estimation_errors(
            ground_truth_trajectory=self.trajectory,
            estimated_trajectory=fb_traj,
        )
        tracking_metrics = calculate_tracking_errors(
            trajectory=self.trajectory,
            planned_path=self.planned_path,
        )
        return SimulationResult(
            success=success,
            collision=self.collision_occurred,
            timeout=self.timeout_occurred,
            goal_reached=self.goal_reached,
            planning_failed=self.planning_failed,
            termination_reason=self.termination_reason or "in_progress",
            steps_taken=self.steps_taken,
            simulated_time=self.simulated_time,
            wall_clock_time=round(elapsed, 5),
            trajectory=list(self.trajectory),
            planned_path=list(self.planned_path),
            final_distance_to_goal=final_dist,
            dwa_failures=self.dwa_failures,
            collided_with_dynamic=self.collided_with_dynamic,
            estimated_trajectory=list(self.estimated_trajectory),
            feedback_trajectory=list(fb_traj),
            estimation_metrics=est_metrics,
            tracking_metrics=tracking_metrics,
            state_feedback=self.config.state_feedback,
            estimator_latencies_ms=list(self.estimator_latencies_ms),
            step_latencies_ms=list(self.step_latencies_ms),
        )

    def run(
        self,
        grid_map: GridMap,
        start_pose: Tuple[float, float, float] | None = None,
        goal_position: Tuple[float, float] | None = None,
        precomputed_path: Optional[List[Tuple[float, float]]] = None,
        dynamic_obstacles: Optional[List[DynamicObstacle]] = None,
    ) -> SimulationResult:
        """Execute a full closed-loop navigation episode synchronously."""
        plan_ok = self.reset(grid_map, start_pose, goal_position, precomputed_path, dynamic_obstacles)
        if not plan_ok:
            return self.get_result()

        while not self.is_done:
            self.step()

        return self.get_result()

