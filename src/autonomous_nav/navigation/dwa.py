"""Dynamic Window Approach (DWA) local collision-aware trajectory planner."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple
import numpy as np

from autonomous_nav.config import DWAConfig, RobotConfig
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.collision import CollisionChecker
from autonomous_nav.simulation.robot import RobotState


@dataclass
class DWASolution:
    """Outcome of a DWA local planning cycle."""
    v_cmd: float
    omega_cmd: float
    best_trajectory: List[Tuple[float, float]] = field(default_factory=list)
    is_safe: bool = True
    score: float = 0.0
    evaluated_trajectories: int = 0
    collision_free_candidates: int = 0


class DWAPlanner:
    """Dynamic Window Approach (DWA) local collision-avoidance planner for unicycle robots.

    Samples feasible (v, omega) within dynamic velocity windows, simulates forward
    candidate trajectories over a prediction horizon, eliminates trajectories colliding
    with static grid obstacles or dynamic moving obstacles, and scores candidates
    using heading alignment, obstacle clearance, and forward velocity.
    """

    def __init__(
        self,
        config: Optional[DWAConfig] = None,
        robot_config: Optional[RobotConfig] = None,
    ) -> None:
        self.config = config if config is not None else DWAConfig()
        self.robot_config = robot_config if robot_config is not None else RobotConfig()

    def compute_dynamic_window(
        self,
        current_v: float,
        current_omega: float,
        dt: float,
    ) -> Tuple[float, float, float, float]:
        """Compute the dynamic velocity window [v_min, v_max, omega_min, omega_max].

        Combines physical velocity bounds with instantaneous acceleration limits.
        """
        # Linear velocity bounds
        v_low = max(self.robot_config.min_velocity, current_v - self.robot_config.max_acceleration * dt)
        v_high = min(self.robot_config.max_velocity, current_v + self.robot_config.max_acceleration * dt)

        # Angular velocity bounds
        max_omega_accel = getattr(self.robot_config, "max_angular_acceleration", 2.0)
        omega_low = max(-self.robot_config.max_angular_velocity, current_omega - max_omega_accel * dt)
        omega_high = min(self.robot_config.max_angular_velocity, current_omega + max_omega_accel * dt)

        return v_low, v_high, omega_low, omega_high

    def simulate_trajectory(
        self,
        x: float,
        y: float,
        theta: float,
        v: float,
        omega: float,
        predict_time: float,
        dt: float,
    ) -> List[Tuple[float, float, float]]:
        """Simulate unicycle forward trajectory: returns list of (x, y, theta) states."""
        trajectory: List[Tuple[float, float, float]] = []
        curr_x, curr_y, curr_theta = x, y, theta
        steps = max(1, int(round(predict_time / dt)))

        for _ in range(steps):
            curr_theta += omega * dt
            # Normalize angle to [-pi, pi]
            curr_theta = math.atan2(math.sin(curr_theta), math.cos(curr_theta))
            curr_x += v * math.cos(curr_theta) * dt
            curr_y += v * math.sin(curr_theta) * dt
            trajectory.append((curr_x, curr_y, curr_theta))

        return trajectory

    def check_trajectory_collision(
        self,
        trajectory: List[Tuple[float, float, float]],
        grid_map: Optional[GridMap],
        dynamic_obstacles: Sequence[DynamicObstacle],
        robot_radius: float,
        safety_margin: float,
        dt_step: float,
        precomputed_predictions: Optional[List[List[Tuple[float, float, float]]]] = None,
        robot_start_pos: Optional[Tuple[float, float]] = None,
        obs_initial_pos: Optional[List[Tuple[float, float, float]]] = None,
    ) -> Tuple[bool, float]:
        """Evaluate if trajectory is collision-free and return minimum clearance distance.

        Performs both discrete disc checks and continuous relative segment swept-distance
        checks to eliminate temporal tunneling between adjacent time samples.

        Returns:
            Tuple[bool, float]: (is_colliding, min_clearance_meters)
        """
        total_radius = robot_radius + safety_margin
        min_clearance = float("inf")

        prev_robot_p = robot_start_pos
        prev_obs_positions = obs_initial_pos

        for step_idx, (tx, ty, _) in enumerate(trajectory):
            curr_robot_p = (tx, ty)
            time_offset = (step_idx + 1) * dt_step

            # 1. Static occupancy grid collision check (fast circle-to-AABB)
            if grid_map is not None:
                if CollisionChecker.check_disc_collision(tx, ty, total_radius, grid_map):
                    return True, 0.0

            # 2. Dynamic moving obstacle collision check (discrete + swept segment)
            if precomputed_predictions is not None and step_idx < len(precomputed_predictions):
                curr_obs_list = precomputed_predictions[step_idx]
            else:
                curr_obs_list = [
                    (obs.predict_position(time_offset)[0], obs.predict_position(time_offset)[1], obs.radius)
                    for obs in dynamic_obstacles
                ]

            for obs_idx, (ox, oy, obs_rad) in enumerate(curr_obs_list):
                obs_thresh = total_radius + obs_rad

                # Discrete point check at t_k
                dist_to_obs = math.hypot(tx - ox, ty - oy)
                if dist_to_obs <= obs_thresh:
                    return True, 0.0

                # Continuous relative swept segment check over [t_{k-1}, t_k] to eliminate tunneling
                if prev_robot_p is not None and prev_obs_positions is not None and obs_idx < len(prev_obs_positions):
                    prev_ox, prev_oy, _ = prev_obs_positions[obs_idx]
                    swept_dist = CollisionChecker.relative_continuous_distance(
                        prev_robot_p, curr_robot_p, (prev_ox, prev_oy), (ox, oy)
                    )
                    if swept_dist <= obs_thresh:
                        return True, 0.0
                    effective_dist = min(dist_to_obs, swept_dist)
                else:
                    effective_dist = dist_to_obs

                obs_surf_dist = effective_dist - (robot_radius + obs_rad)
                if obs_surf_dist < min_clearance:
                    min_clearance = max(0.0, obs_surf_dist)

            prev_robot_p = curr_robot_p
            prev_obs_positions = curr_obs_list

        # 3. Static obstacle clearance estimate at trajectory endpoint
        if grid_map is not None and trajectory:
            end_x, end_y, _ = trajectory[-1]
            static_dist = CollisionChecker.distance_to_nearest_obstacle(end_x, end_y, grid_map, search_radius=2.0)
            surf_dist = max(0.0, static_dist - robot_radius)
            if surf_dist < min_clearance:
                min_clearance = surf_dist

        return False, min_clearance

    def score_trajectory(
        self,
        trajectory: List[Tuple[float, float, float]],
        v: float,
        target_point: Tuple[float, float],
        min_clearance: float,
        start_x: float = 0.0,
        start_y: float = 0.0,
    ) -> float:
        """Calculate classical 3-term DWA objective score (Fox et al., 1997).

        Terms:
            1. Heading alignment: alignment between robot endpoint heading and vector to lookahead sub-goal.
            2. Clearance: normalized distance to closest static or dynamic obstacle.
            3. Velocity: forward speed reward.
        """
        end_x, end_y, end_theta = trajectory[-1]
        gx, gy = target_point

        # 1. Heading Alignment Score (classical DWA)
        target_angle = math.atan2(gy - end_y, gx - end_x)
        angle_diff = abs(math.atan2(math.sin(target_angle - end_theta), math.cos(target_angle - end_theta)))
        score_heading = (math.pi - angle_diff) / math.pi  # Normalized to [0, 1]

        # 2. Clearance Score (penalize close proximity)
        if min_clearance <= 0.15:
            score_clearance = 0.0
        else:
            score_clearance = min(min_clearance / 2.0, 1.0)

        # 3. Forward Velocity Score (prefer moving over standing still when obstacles are nearby)
        max_v = max(0.1, self.robot_config.max_velocity)
        score_velocity = max(0.0, v) / max_v

        # Weighted combination
        total_score = (
            self.config.alpha_heading * score_heading
            + self.config.beta_clearance * score_clearance
            + self.config.gamma_velocity * score_velocity
        )
        return total_score

    def compute_command(
        self,
        state: RobotState,
        target_point: Tuple[float, float],
        grid_map: Optional[GridMap] = None,
        dynamic_obstacles: Sequence[DynamicObstacle] = (),
        robot_radius: float = 0.4,
        dt: float = 0.1,
    ) -> DWASolution:
        """Plan optimal (v, omega) command using Dynamic Window Approach.

        Args:
            state: Current robot kinematic state.
            target_point: Local sub-goal (e.g. from A* path lookahead).
            grid_map: Static occupancy grid map.
            dynamic_obstacles: Sequence of active moving obstacles.
            robot_radius: Chassis collision radius.
            dt: Simulation time step.

        Returns:
            DWASolution containing best (v_cmd, omega_cmd), planned arc, and safety flag.
        """
        current_v = state.v
        # Estimate current angular velocity from state or assume previous cmd
        current_omega = getattr(state, "omega", 0.0)

        v_low, v_high, omega_low, omega_high = self.compute_dynamic_window(current_v, current_omega, dt)

        # Generate sample grid deterministically
        v_samples = np.linspace(v_low, v_high, max(3, self.config.v_samples))
        omega_samples = np.linspace(omega_low, omega_high, max(3, self.config.omega_samples))

        # Always include zero velocities (stopping / in-place rotation)
        if 0.0 not in v_samples and v_low <= 0.0 <= v_high:
            v_samples = np.sort(np.append(v_samples, 0.0))
        if 0.0 not in omega_samples and omega_low <= 0.0 <= omega_high:
            omega_samples = np.sort(np.append(omega_samples, 0.0))

        best_score = -float("inf")
        best_v = 0.0
        best_omega = 0.0
        best_traj_pts: List[Tuple[float, float]] = []
        safe_candidates = 0
        total_evaluated = 0

        predict_dt = self.config.dt
        predict_time = self.config.predict_time
        horizon_steps = max(1, int(round(predict_time / predict_dt)))

        # Precompute projected dynamic obstacle positions once per command cycle
        obs_initial_pos = [(obs.x, obs.y, obs.radius) for obs in dynamic_obstacles]
        obs_predictions = [
            [
                (
                    obs.predict_position((k + 1) * predict_dt)[0],
                    obs.predict_position((k + 1) * predict_dt)[1],
                    obs.radius,
                )
                for obs in dynamic_obstacles
            ]
            for k in range(horizon_steps)
        ]
        robot_start_pos = (state.x, state.y)

        # Deterministic evaluation loop
        for v in v_samples:
            for omega in omega_samples:
                total_evaluated += 1
                traj = self.simulate_trajectory(
                    x=state.x,
                    y=state.y,
                    theta=state.theta,
                    v=float(v),
                    omega=float(omega),
                    predict_time=predict_time,
                    dt=predict_dt,
                )

                collides, clearance = self.check_trajectory_collision(
                    trajectory=traj,
                    grid_map=grid_map,
                    dynamic_obstacles=dynamic_obstacles,
                    robot_radius=robot_radius,
                    safety_margin=self.config.safety_margin,
                    dt_step=predict_dt,
                    precomputed_predictions=obs_predictions,
                    robot_start_pos=robot_start_pos,
                    obs_initial_pos=obs_initial_pos,
                )

                if collides:
                    continue

                safe_candidates += 1
                score = self.score_trajectory(
                    trajectory=traj,
                    v=float(v),
                    target_point=target_point,
                    min_clearance=clearance,
                    start_x=state.x,
                    start_y=state.y,
                )

                # Deterministic tie-breaking: prefer higher score; on tie prefer higher speed, then lower |omega|
                if score > best_score + 1e-6:
                    best_score = score
                    best_v = float(v)
                    best_omega = float(omega)
                    best_traj_pts = [(p[0], p[1]) for p in traj]
                elif abs(score - best_score) <= 1e-6:
                    if float(v) > best_v + 1e-6:
                        best_score = score
                        best_v = float(v)
                        best_omega = float(omega)
                        best_traj_pts = [(p[0], p[1]) for p in traj]
                    elif abs(float(v) - best_v) <= 1e-6 and abs(float(omega)) < abs(best_omega):
                        best_score = score
                        best_v = float(v)
                        best_omega = float(omega)
                        best_traj_pts = [(p[0], p[1]) for p in traj]

        if safe_candidates == 0:
            # Emergency stop: no collision-free command found
            return DWASolution(
                v_cmd=0.0,
                omega_cmd=0.0,
                best_trajectory=[],
                is_safe=False,
                score=0.0,
                evaluated_trajectories=total_evaluated,
                collision_free_candidates=0,
            )

        return DWASolution(
            v_cmd=best_v,
            omega_cmd=best_omega,
            best_trajectory=best_traj_pts,
            is_safe=True,
            score=best_score,
            evaluated_trajectories=total_evaluated,
            collision_free_candidates=safe_candidates,
        )
