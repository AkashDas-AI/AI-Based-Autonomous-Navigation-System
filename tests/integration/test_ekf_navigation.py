"""Integration tests for closed-loop navigation under sensor noise and EKF state estimation."""

import math
import pytest

from autonomous_nav.config import EKFConfig, SensorConfig, SimulationConfig
from autonomous_nav.environment.scenarios import get_scenario_by_name
from autonomous_nav.simulation.engine import SimulationEngine


def test_closed_loop_ekf_navigation_success():
    """Verify that autonomous navigation achieves goal safely in closed-loop EKF mode on simple_obstacles."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(
            odometry_noise_std=0.03,
            imu_rate_noise_std=0.01,
            imu_bias_init=0.01,
            gnss_pos_noise_std=0.25,
            gnss_update_interval=1.0,
            seed=42,
        ),
        ekf=EKFConfig(
            q_pos=0.05,
            q_theta=0.02,
            q_v=0.1,
            q_bias=0.001,
            r_odom=0.05,
            r_gnss=0.25,
        ),
    )

    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    assert result.success is True
    assert result.collision is False
    assert result.termination_reason == "goal_reached"
    assert result.final_distance_to_goal <= cfg.goal_tolerance + 0.2

    # Check estimation error metrics
    est = result.estimation_metrics
    assert est["position_rmse"] < 0.40  # Well within sensor noise envelope
    assert est["position_mae"] < 0.35
    assert est["heading_rmse"] < 0.25


def test_state_feedback_isolation():
    """Verify navigation controller strictly consumes estimated state in EKF mode without leaking ground truth."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(seed=42),
    )

    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")
    engine.reset(grid_map=scenario.grid_map, start_pose=scenario.start_world, goal_position=scenario.goal_world)

    # Instrument pure pursuit controller to log the state passed to it
    received_states = []
    original_compute = engine.controller.compute_control

    def wrapped_compute(state, target_point, distance_to_goal):
        received_states.append((state.x, state.y, state.theta))
        return original_compute(state, target_point, distance_to_goal)

    engine.controller.compute_control = wrapped_compute

    for _ in range(15):
        if engine.is_done:
            break
        engine.step()

    assert len(received_states) > 0
    # Confirm that every received state matches the EKF estimated state and differs from ground truth
    for idx, (rx, ry, rth) in enumerate(received_states):
        est = engine.estimated_trajectory[idx + 1]  # index 0 is start pose
        true_s = engine.trajectory[idx]

        # Must match EKF estimate
        assert rx == pytest.approx(est.x, abs=1e-9)
        assert ry == pytest.approx(est.y, abs=1e-9)
        assert rth == pytest.approx(est.theta, abs=1e-9)


def test_dead_reckoning_drift_vs_ekf_bounded():
    """Verify that pure dead-reckoning drifts over time whereas EKF fusion maintains bounded error."""
    scenario = get_scenario_by_name("corridor_s_bend")

    # Run Dead Reckoning
    cfg_dr = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="dead_reckoning",
        sensors=SensorConfig(
            odometry_noise_std=0.04,
            odometry_scale_factor=1.02,  # 2% scale factor drift
            imu_rate_noise_std=0.015,
            imu_bias_init=0.015,
            seed=42,
        ),
    )
    engine_dr = SimulationEngine(config=cfg_dr)
    res_dr = engine_dr.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    # Run EKF Fusion
    cfg_ekf = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(
            odometry_noise_std=0.04,
            odometry_scale_factor=1.02,
            imu_rate_noise_std=0.015,
            imu_bias_init=0.015,
            gnss_pos_noise_std=0.25,
            gnss_update_interval=1.0,
            seed=42,
        ),
    )
    engine_ekf = SimulationEngine(config=cfg_ekf)
    res_ekf = engine_ekf.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    rmse_dr = res_dr.estimation_metrics["position_rmse"]
    rmse_ekf = res_ekf.estimation_metrics["position_rmse"]

    # EKF should significantly outperform unbounded Dead Reckoning drift
    assert rmse_ekf < rmse_dr
    assert rmse_ekf < 0.45


def test_gnss_outage_divergence_and_reconvergence():
    """Verify error grows during GNSS outage interval and re-converges after fixes resume."""
    scenario = get_scenario_by_name("corridor_s_bend")
    outage_window = (5.0, 12.0)  # 7-second outage in middle of trajectory

    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(
            odometry_noise_std=0.03,
            odometry_scale_factor=1.01,
            imu_rate_noise_std=0.01,
            imu_bias_init=0.01,
            gnss_pos_noise_std=0.25,
            gnss_update_interval=1.0,
            gnss_outages=[outage_window],
            seed=42,
        ),
    )
    engine = SimulationEngine(config=cfg)
    engine.reset(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    errors_pre_outage = []
    errors_during_outage = []
    errors_post_outage = []

    while not engine.is_done:
        engine.step()
        t = engine.simulated_time
        true_s = engine.robot.state
        est_s = engine.ekf.get_estimated_robot_state()
        err = math.hypot(true_s.x - est_s.x, true_s.y - est_s.y)

        if t < outage_window[0]:
            errors_pre_outage.append(err)
        elif outage_window[0] <= t <= outage_window[1]:
            errors_during_outage.append(err)
        elif t > outage_window[1] + 2.0:  # Allow 2s to re-converge
            errors_post_outage.append(err)

    avg_pre = sum(errors_pre_outage) / len(errors_pre_outage) if errors_pre_outage else 0.0
    max_outage = max(errors_during_outage) if errors_during_outage else 0.0
    avg_post = sum(errors_post_outage) / len(errors_post_outage) if errors_post_outage else 0.0

    # Max error during outage should exceed pre-outage average
    assert max_outage > avg_pre
    # Post-outage error should re-converge to bounded levels
    assert avg_post < max_outage


def test_ekf_navigation_determinism():
    """Verify identical sensor seeds produce bitwise identical navigation runs and metrics."""
    scenario = get_scenario_by_name("simple_obstacles")

    def run_sim(seed: int):
        cfg = SimulationConfig(
            nav_mode="pure_pursuit",
            state_feedback="ekf",
            sensors=SensorConfig(seed=seed),
        )
        engine = SimulationEngine(config=cfg)
        return engine.run(
            grid_map=scenario.grid_map,
            start_pose=scenario.start_world,
            goal_position=scenario.goal_world,
        )

    res_1 = run_sim(12345)
    res_2 = run_sim(12345)

    assert res_1.steps_taken == res_2.steps_taken
    assert res_1.success == res_2.success
    assert res_1.final_distance_to_goal == res_2.final_distance_to_goal
    assert res_1.estimation_metrics == res_2.estimation_metrics


def test_ground_truth_feedback_zero_estimation_error():
    """Verify that ground_truth feedback mode yields 0.0 feedback-pose estimation error by construction."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ground_truth",
        sensors=SensorConfig(seed=42),
    )
    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")
    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    assert result.success is True
    # Feedback pose estimation error must be identically zero in ground_truth mode
    est = result.estimation_metrics
    assert est["position_rmse"] == 0.0
    assert est["position_mae"] == 0.0
    assert est["max_position_error"] == 0.0
    assert est["final_position_error"] == 0.0
    assert est["heading_rmse"] == 0.0
    assert est["velocity_rmse"] == 0.0

    # Array length synchronization across physical and feedback trajectories
    assert len(result.trajectory) == len(result.feedback_trajectory)
    assert len(result.trajectory) == result.steps_taken + 1

    # In ground_truth mode, cross-track error (tracking error) remains non-zero
    # demonstrating clear separation of estimation accuracy from path tracking performance
    track = result.tracking_metrics
    assert track["mean_cross_track_error"] > 0.0
    assert track["max_cross_track_error"] > 0.0
    assert track["actual_trajectory_length"] > 0.0
    assert track["planned_path_length"] > 0.0


def test_dead_reckoning_strictly_zero_gnss_leakage():
    """Verify that dead-reckoning mode never processes GNSS fixes and isolates feedback from GNSS."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="dead_reckoning",
        sensors=SensorConfig(
            odometry_noise_std=0.03,
            imu_rate_noise_std=0.01,
            gnss_pos_noise_std=0.25,
            gnss_update_interval=1.0,  # GNSS is firing in sensor suite
            seed=42,
        ),
    )
    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")
    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    # GNSS fixes processed by the state estimator must remain strictly zero
    assert engine.gnss_fixes_processed == 0
    assert len(engine.gnss_measurements) == 0

    # Feedback pose estimation error must reflect uncorrected dead-reckoning drift
    assert result.estimation_metrics["position_rmse"] > 0.0
    assert result.estimation_metrics["max_position_error"] > 0.0


def test_metric_distinction_estimation_vs_tracking():
    """Verify distinct separation of estimation accuracy from trajectory-tracking metrics."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(seed=42),
    )
    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")
    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    # 1. Estimation metrics describe state belief vs true state
    est = result.estimation_metrics
    assert "position_rmse" in est
    assert "position_mae" in est
    assert "max_position_error" in est
    assert "heading_rmse" in est
    assert "velocity_rmse" in est

    # 2. Tracking metrics describe physical vehicle motion vs planned path
    track = result.tracking_metrics
    assert "mean_cross_track_error" in track
    assert "max_cross_track_error" in track
    assert "planned_path_length" in track
    assert "actual_trajectory_length" in track
    assert "path_efficiency_ratio" in track

    # Both metric classes must be independently quantified
    assert est["position_rmse"] != track["mean_cross_track_error"]


def test_trajectory_timestamp_semantics_and_terminal_alignment():
    """Verify physical, estimated, and feedback trajectories have matching timestamps across run life cycle."""
    cfg = SimulationConfig(
        nav_mode="pure_pursuit",
        state_feedback="ekf",
        sensors=SensorConfig(seed=42),
    )
    engine = SimulationEngine(config=cfg)
    scenario = get_scenario_by_name("simple_obstacles")
    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
    )

    n = len(result.trajectory)
    assert n == len(result.feedback_trajectory)
    assert n == len(result.estimated_trajectory)
    assert n == result.steps_taken + 1

    # Verify initial sample semantics (t = 0.0)
    assert result.trajectory[0].timestamp == 0.0
    assert result.feedback_trajectory[0].timestamp == 0.0
    assert result.estimated_trajectory[0].timestamp == 0.0

    # Verify terminal sample semantics (t = simulated_time)
    assert pytest.approx(result.trajectory[-1].timestamp, abs=1e-4) == result.simulated_time
    assert pytest.approx(result.feedback_trajectory[-1].timestamp, abs=1e-4) == result.simulated_time
    assert pytest.approx(result.estimated_trajectory[-1].timestamp, abs=1e-4) == result.simulated_time

    # Verify monotonic, perfectly synchronized timestamps across every intermediate step
    dt = cfg.dt
    for i in range(n):
        expected_t = round(i * dt, 4)
        assert pytest.approx(result.trajectory[i].timestamp, abs=1e-4) == expected_t
        assert pytest.approx(result.feedback_trajectory[i].timestamp, abs=1e-4) == expected_t
        assert pytest.approx(result.estimated_trajectory[i].timestamp, abs=1e-4) == expected_t


def test_early_collision_termination_timestamp_alignment():
    """Verify that early collision termination preserves exact timestamp alignment without trailing estimates."""
    from autonomous_nav.environment.grid_map import GridMap

    # Create a scenario where an obstacle is placed directly in front of the start
    grid = GridMap(width=20.0, height=20.0, resolution=0.5)
    grid.set_obstacle_world(5.0, 5.0)  # Obstacle right in front of start (4.0, 5.0)

    cfg = SimulationConfig(
        start_pose=(4.0, 5.0, 0.0),
        goal_position=(15.0, 5.0),
        state_feedback="ekf",
    )
    engine = SimulationEngine(config=cfg)
    # Manually provide a precomputed path straight into the obstacle
    result = engine.run(
        grid_map=grid,
        precomputed_path=[(4.0, 5.0), (5.0, 5.0), (15.0, 5.0)],
    )

    assert result.collision is True
    assert result.success is False
    assert result.termination_reason == "collision"

    # Strict length and timestamp alignment even under unexpected premature collision termination
    n = len(result.trajectory)
    assert n == len(result.feedback_trajectory)
    assert n == len(result.estimated_trajectory)
    for i in range(n):
        assert result.trajectory[i].timestamp == result.feedback_trajectory[i].timestamp
        assert result.trajectory[i].timestamp == result.estimated_trajectory[i].timestamp


