"""Performance evaluation metrics for autonomous navigation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Dict, List, Tuple

if TYPE_CHECKING:
    from autonomous_nav.simulation.robot import RobotState


def calculate_path_length(path: List[Tuple[float, float]]) -> float:
    """Compute total cumulative Euclidean path length in meters."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return total


def calculate_trajectory_length(trajectory: List[RobotState]) -> float:
    """Compute driven distance from robot trajectory states."""
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(trajectory)):
        total += math.hypot(
            trajectory[i].x - trajectory[i - 1].x,
            trajectory[i].y - trajectory[i - 1].y,
        )
    return total


def calculate_angular_smoothness(trajectory: List[RobotState]) -> float:
    """Compute sum of absolute changes in angular velocity (proxy for steering jerk)."""
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(trajectory)):
        total += abs(trajectory[i].omega - trajectory[i - 1].omega)
    return total


def point_to_segment_distance(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """Compute minimum Euclidean distance from point (px, py) to line segment [(ax, ay), (bx, by)]."""
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    # Projection factor clamped to segment bounds [0.0, 1.0]
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def calculate_cross_track_error(
    trajectory: List[RobotState],
    reference_path: List[Tuple[float, float]],
) -> Tuple[float, float]:
    """Calculate mean and maximum orthogonal cross-track error (CTE) from driven trajectory to reference path.

    Returns:
        Tuple of (mean_cte, max_cte) in meters.
    """
    if not trajectory or not reference_path:
        return 0.0, 0.0

    if len(reference_path) == 1:
        rx, ry = reference_path[0]
        errors = [math.hypot(s.x - rx, s.y - ry) for s in trajectory]
        return float(sum(errors) / len(errors)), float(max(errors))

    errors = []
    for state in trajectory:
        # Distance to closest line segment of reference path
        min_d = min(
            point_to_segment_distance(
                state.x, state.y,
                reference_path[i][0], reference_path[i][1],
                reference_path[i + 1][0], reference_path[i + 1][1],
            )
            for i in range(len(reference_path) - 1)
        )
        errors.append(min_d)

    mean_cte = float(sum(errors) / len(errors))
    max_cte = float(max(errors))
    return mean_cte, max_cte


def calculate_tracking_errors(
    trajectory: List[RobotState],
    planned_path: List[Tuple[float, float]],
) -> Dict[str, float]:
    """Calculate trajectory-tracking metrics comparing actual driven trajectory against planned reference path.

    Returns:
        Dict[str, float]:
            - mean_cross_track_error (meters)
            - max_cross_track_error (meters)
            - planned_path_length (meters)
            - actual_trajectory_length (meters)
            - path_efficiency_ratio (dimensionless: planned / actual)
    """
    ref_length = calculate_path_length(planned_path)
    actual_length = calculate_trajectory_length(trajectory)
    mean_cte, max_cte = calculate_cross_track_error(trajectory, planned_path)
    path_efficiency = (ref_length / actual_length) if actual_length > 0 else 0.0

    return {
        "mean_cross_track_error": round(float(mean_cte), 4),
        "max_cross_track_error": round(float(max_cte), 4),
        "planned_path_length": round(float(ref_length), 3),
        "actual_trajectory_length": round(float(actual_length), 3),
        "path_efficiency_ratio": round(float(path_efficiency), 3),
    }


def evaluate_run(
    success: bool,
    collision: bool,
    timeout: bool,
    goal_reached: bool,
    steps_taken: int,
    time_taken: float,
    trajectory: List[RobotState],
    planned_path: List[Tuple[float, float]],
    final_distance_to_goal: float,
    planning_failed: bool = False,
    termination_reason: str | None = None,
    wall_clock_time_seconds: float | None = None,
) -> Dict[str, Any]:
    """Synthesize complete performance scorecard for a navigation run."""
    ref_length = calculate_path_length(planned_path)
    actual_length = calculate_trajectory_length(trajectory)
    smoothness = calculate_angular_smoothness(trajectory)
    mean_cte, max_cte = calculate_cross_track_error(trajectory, planned_path)

    path_efficiency = (ref_length / actual_length) if actual_length > 0 else 0.0

    velocities = [s.v for s in trajectory]
    mean_v = float(sum(velocities) / len(velocities)) if velocities else 0.0
    max_v = float(max(velocities)) if velocities else 0.0

    if termination_reason is None:
        if collision:
            termination_reason = "collision"
        elif goal_reached and not collision:
            termination_reason = "goal_reached"
        elif planning_failed:
            termination_reason = "planning_failure"
        else:
            termination_reason = "timeout"

    return {
        "success": bool(success),
        "collision": bool(collision),
        "timeout": bool(timeout),
        "goal_reached": bool(goal_reached),
        "planning_failed": bool(planning_failed),
        "termination_reason": str(termination_reason),
        "steps_taken": int(steps_taken),
        "simulated_time_seconds": round(float(time_taken), 2),
        "wall_clock_time_seconds": round(float(wall_clock_time_seconds), 4) if wall_clock_time_seconds is not None else None,
        "time_taken_seconds": round(float(time_taken), 2),  # Backward-compatible alias for simulated time
        "planned_path_length_meters": round(float(ref_length), 2),
        "actual_trajectory_length_meters": round(float(actual_length), 2),
        "path_efficiency_ratio": round(float(path_efficiency), 3),
        "mean_cross_track_error_meters": round(float(mean_cte), 3),
        "max_cross_track_error_meters": round(float(max_cte), 3),
        "mean_velocity_mps": round(float(mean_v), 2),
        "max_velocity_mps": round(float(max_v), 2),
        "angular_smoothness_metric": round(float(smoothness), 3),
        "final_distance_to_goal_meters": round(float(final_distance_to_goal), 3),
    }


def calculate_estimation_errors(
    ground_truth_trajectory: List[RobotState],
    estimated_trajectory: List[RobotState],
) -> Dict[str, float]:
    """Calculate trajectory-wide error metrics comparing state estimates to true ground-truth poses.

    Args:
        ground_truth_trajectory: Time-ordered list of true robot states.
        estimated_trajectory: Time-ordered list of estimated robot states.

    Returns:
        Dict[str, float]: Comprehensive dictionary containing:
            - position_rmse (meters)
            - position_mae (meters)
            - max_position_error (meters)
            - final_position_error (meters)
            - heading_rmse (radians)
            - heading_mae (radians)
            - max_heading_error (radians)
            - velocity_rmse (m/s)
            - velocity_mae (m/s)
    """
    n = min(len(ground_truth_trajectory), len(estimated_trajectory))
    if n == 0:
        return {
            "position_rmse": 0.0,
            "position_mae": 0.0,
            "max_position_error": 0.0,
            "final_position_error": 0.0,
            "heading_rmse": 0.0,
            "heading_mae": 0.0,
            "max_heading_error": 0.0,
            "velocity_rmse": 0.0,
            "velocity_mae": 0.0,
        }

    pos_errors_sq = []
    pos_errors = []
    heading_errors_sq = []
    heading_errors = []
    v_errors_sq = []
    v_errors = []

    for i in range(n):
        gt = ground_truth_trajectory[i]
        est = estimated_trajectory[i]

        # Verify timestamp alignment when timestamps are populated
        if hasattr(gt, "timestamp") and hasattr(est, "timestamp"):
            if gt.timestamp > 0.0 and est.timestamp > 0.0:
                if abs(gt.timestamp - est.timestamp) > 1e-3:
                    raise ValueError(
                        f"Trajectory timestamp mismatch at sample index {i}: "
                        f"ground_truth={gt.timestamp}s, estimated={est.timestamp}s"
                    )

        # Positional Euclidean error
        dx = gt.x - est.x
        dy = gt.y - est.y
        dist = math.hypot(dx, dy)
        pos_errors.append(dist)
        pos_errors_sq.append(dist * dist)

        # Angular heading error wrapped to [-pi, pi]
        d_theta = math.atan2(math.sin(gt.theta - est.theta), math.cos(gt.theta - est.theta))
        abs_d_theta = abs(d_theta)
        heading_errors.append(abs_d_theta)
        heading_errors_sq.append(d_theta * d_theta)

        # Linear velocity error
        dv = abs(gt.v - est.v)
        v_errors.append(dv)
        v_errors_sq.append(dv * dv)

    pos_rmse = math.sqrt(sum(pos_errors_sq) / n)
    pos_mae = sum(pos_errors) / n
    max_pos = max(pos_errors)
    final_pos = pos_errors[-1]

    heading_rmse = math.sqrt(sum(heading_errors_sq) / n)
    heading_mae = sum(heading_errors) / n
    max_heading = max(heading_errors)

    v_rmse = math.sqrt(sum(v_errors_sq) / n)
    v_mae = sum(v_errors) / n

    return {
        "position_rmse": round(float(pos_rmse), 4),
        "position_mae": round(float(pos_mae), 4),
        "max_position_error": round(float(max_pos), 4),
        "final_position_error": round(float(final_pos), 4),
        "heading_rmse": round(float(heading_rmse), 4),
        "heading_mae": round(float(heading_mae), 4),
        "max_heading_error": round(float(max_heading), 4),
        "velocity_rmse": round(float(v_rmse), 4),
        "velocity_mae": round(float(v_mae), 4),
    }

