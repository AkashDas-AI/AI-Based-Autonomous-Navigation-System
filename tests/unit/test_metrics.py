"""Unit tests for performance metrics and simulation logger."""

import json
from pathlib import Path
import pytest
from autonomous_nav.evaluation.logger import SimulationLogger
from autonomous_nav.evaluation.metrics import (
    calculate_angular_smoothness,
    calculate_cross_track_error,
    calculate_path_length,
    calculate_trajectory_length,
    calculate_tracking_errors,
    calculate_estimation_errors,
    evaluate_run,
    point_to_segment_distance,
)
from autonomous_nav.simulation.robot import RobotState


def test_point_to_segment_distance():
    """Verify distance from point to segment handles collinear, perpendicular, and endpoint cases."""
    # Segment from (0, 0) to (2, 0)
    # Point at (1, 1): perpendicular projection falls at (1, 0), distance = 1.0
    assert pytest.approx(point_to_segment_distance(1.0, 1.0, 0.0, 0.0, 2.0, 0.0)) == 1.0

    # Point at (3, 0): beyond segment end, closest point is (2, 0), distance = 1.0
    assert pytest.approx(point_to_segment_distance(3.0, 0.0, 0.0, 0.0, 2.0, 0.0)) == 1.0

    # Point on segment at (1, 0): distance = 0.0
    assert pytest.approx(point_to_segment_distance(1.0, 0.0, 0.0, 0.0, 2.0, 0.0)) == 0.0


def test_calculate_path_and_trajectory_length():
    """Verify cumulative Euclidean distance for path and trajectory."""
    path = [(0.0, 0.0), (3.0, 4.0), (6.0, 8.0)]
    # First segment: 5.0, second segment: 5.0 -> total 10.0
    assert pytest.approx(calculate_path_length(path)) == 10.0

    trajectory = [
        RobotState(x=0.0, y=0.0),
        RobotState(x=1.0, y=0.0),
        RobotState(x=1.0, y=2.0),
    ]
    # Segments: 1.0 + 2.0 = 3.0
    assert pytest.approx(calculate_trajectory_length(trajectory)) == 3.0


def test_calculate_cross_track_error_orthogonal():
    """Verify CTE returns zero when trajectory points fall precisely on reference line segment."""
    reference_path = [(0.0, 0.0), (10.0, 0.0)]
    # Robot drives along y=0.0 at various x
    trajectory = [
        RobotState(x=0.0, y=0.0),
        RobotState(x=2.5, y=0.0),
        RobotState(x=5.0, y=0.0),
        RobotState(x=10.0, y=0.0),
    ]
    mean_cte, max_cte = calculate_cross_track_error(trajectory, reference_path)
    assert pytest.approx(mean_cte, abs=1e-6) == 0.0
    assert pytest.approx(max_cte, abs=1e-6) == 0.0


def test_evaluate_run_metrics_dictionary():
    """Verify evaluate_run properly classifies outcomes and computes scorecard."""
    trajectory = [
        RobotState(x=0.0, y=0.0, v=1.0, omega=0.0),
        RobotState(x=1.0, y=0.0, v=1.0, omega=0.1),
    ]
    planned_path = [(0.0, 0.0), (1.0, 0.0)]

    scorecard = evaluate_run(
        success=True,
        collision=False,
        timeout=False,
        goal_reached=True,
        steps_taken=10,
        time_taken=1.0,
        trajectory=trajectory,
        planned_path=planned_path,
        final_distance_to_goal=0.1,
        wall_clock_time_seconds=0.02,
    )

    assert scorecard["success"] is True
    assert scorecard["termination_reason"] == "goal_reached"
    assert scorecard["simulated_time_seconds"] == 1.0
    assert scorecard["wall_clock_time_seconds"] == 0.02
    assert scorecard["planned_path_length_meters"] == 1.0
    assert scorecard["actual_trajectory_length_meters"] == 1.0
    assert scorecard["path_efficiency_ratio"] == 1.0


def test_simulation_logger(tmp_path: Path):
    """Verify SimulationLogger records steps and writes valid CSV and JSON."""
    logger = SimulationLogger(run_id="test_run")
    state = RobotState(x=1.0, y=2.0, theta=0.5, v=1.5, omega=0.1)
    logger.log_step(step=1, timestamp=0.1, state=state, v_cmd=1.5, omega_cmd=0.1, dist_to_goal=5.0)

    csv_file = tmp_path / "test_log.csv"
    json_file = tmp_path / "test_summary.json"

    logger.save_csv(csv_file)
    logger.save_summary_json({"success": True}, json_file)

    assert csv_file.exists()
    assert json_file.exists()

    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert data["success"] is True


def test_calculate_tracking_errors():
    """Verify trajectory-tracking metrics synthesis."""
    reference_path = [(0.0, 0.0), (10.0, 0.0)]
    trajectory = [
        RobotState(x=0.0, y=0.0),
        RobotState(x=5.0, y=0.2),
        RobotState(x=10.0, y=0.0),
    ]
    res = calculate_tracking_errors(trajectory, reference_path)
    assert "mean_cross_track_error" in res
    assert "max_cross_track_error" in res
    assert "planned_path_length" in res
    assert "actual_trajectory_length" in res
    assert "path_efficiency_ratio" in res
    assert res["planned_path_length"] == 10.0
    assert res["max_cross_track_error"] == 0.2


def test_calculate_estimation_errors_zero_for_identical():
    """Verify calculate_estimation_errors returns exact 0.0 for identical trajectories."""
    trajectory = [
        RobotState(x=0.0, y=0.0, theta=0.0, v=0.5),
        RobotState(x=1.0, y=1.0, theta=0.785, v=0.8),
    ]
    res = calculate_estimation_errors(trajectory, trajectory)
    assert res["position_rmse"] == 0.0
    assert res["position_mae"] == 0.0
    assert res["max_position_error"] == 0.0
    assert res["heading_rmse"] == 0.0
    assert res["velocity_rmse"] == 0.0


def test_calculate_estimation_errors_timestamp_alignment_enforced():
    """Verify calculate_estimation_errors validates matching sample timestamps and rejects misaligned inputs."""
    s0 = RobotState(x=0.0, y=0.0, timestamp=0.0)
    s1_gt = RobotState(x=1.0, y=0.0, timestamp=0.1)
    s2_gt = RobotState(x=2.0, y=0.0, timestamp=0.2)

    s1_est_aligned = RobotState(x=1.05, y=0.02, timestamp=0.1)
    s2_est_aligned = RobotState(x=2.08, y=0.01, timestamp=0.2)

    # Aligned timestamps succeed
    res = calculate_estimation_errors([s0, s1_gt, s2_gt], [s0, s1_est_aligned, s2_est_aligned])
    assert res["position_rmse"] > 0.0

    # Misaligned timestamps (1-step offset) raise ValueError
    s1_est_lagged = RobotState(x=1.05, y=0.02, timestamp=0.0)
    s2_est_lagged = RobotState(x=2.08, y=0.01, timestamp=0.1)
    with pytest.raises(ValueError, match="Trajectory timestamp mismatch"):
        calculate_estimation_errors([s0, s1_gt, s2_gt], [s0, s1_est_lagged, s2_est_lagged])


def test_calculate_estimation_errors_heading_pi_wrapping():
    """Verify angular heading error wraps properly across [-pi, pi] branch cut."""
    # Near +pi (+3.14 rad) vs near -pi (-3.14 rad): true angular difference is ~0.00318 rad
    gt = [RobotState(x=0.0, y=0.0, theta=3.14, timestamp=0.1)]
    est = [RobotState(x=0.0, y=0.0, theta=-3.14, timestamp=0.1)]

    res = calculate_estimation_errors(gt, est)
    # Without angle wrapping, error would be 6.28 rad. With wrapping, it is ~0.0032 rad
    assert res["heading_rmse"] == pytest.approx(abs(3.14 - (-3.14) - 2 * 3.1415926535), abs=1e-3)
    assert res["heading_rmse"] < 0.01



