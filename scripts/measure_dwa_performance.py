#!/usr/bin/env python3
"""Instrument and measure DWA local planning latency vs complete simulation step latency.

Measures fine-grained timing breakdowns (mean, median, p95, max) for DWA planning operations,
total step durations, trajectory evaluations, and full closed-loop episodes across all dynamic scenarios.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

# Ensure src/ is on sys.path
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import numpy as np
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.scenarios import get_dynamic_scenarios
from autonomous_nav.simulation.engine import SimulationEngine


def percentile(data: List[float], p: float) -> float:
    """Calculate percentile p in [0, 100]."""
    if not data:
        return 0.0
    return float(np.percentile(data, p))


def measure_scenario_performance(scenario_name: str, config: SimulationConfig | None = None) -> Dict[str, Any]:
    """Run an instrumented episode measuring DWA planning latency and step latency separately."""
    scenarios = get_dynamic_scenarios()
    scenario = scenarios[scenario_name]

    cfg = config if config is not None else SimulationConfig(nav_mode="dwa", max_steps=500)
    engine = SimulationEngine(cfg)

    # 1. Setup / Reset timing
    t_setup_start = time.perf_counter()
    ok = engine.reset(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
    )
    t_setup = time.perf_counter() - t_setup_start
    if not ok:
        raise RuntimeError(f"Global planning failed for scenario {scenario_name}")

    dwa_latencies: List[float] = []
    step_latencies: List[float] = []
    candidates_evaluated: List[int] = []
    safe_candidates_list: List[int] = []

    # 2. Main simulation stepping loop
    t_loop_start = time.perf_counter()

    while not engine.is_done:
        t_step_start = time.perf_counter()

        # Check termination conditions (same as in engine.step())
        current_state = engine.robot.state
        if engine.waypoint_mgr.is_goal_reached(
            current_state.x, current_state.y, tolerance=engine.config.goal_tolerance
        ):
            engine.goal_reached = True
            engine.is_done = True
            engine.termination_reason = "goal_reached"
            t_step_end = time.perf_counter()
            step_latencies.append(t_step_end - t_step_start)
            break

        if engine.steps_taken >= engine.config.max_steps:
            engine.timeout_occurred = True
            engine.is_done = True
            engine.termination_reason = "timeout"
            t_step_end = time.perf_counter()
            step_latencies.append(t_step_end - t_step_start)
            break

        engine.waypoint_mgr.update(current_state.x, current_state.y)
        target = engine.waypoint_mgr.get_lookahead_point(
            current_state.x,
            current_state.y,
            lookahead_distance=engine.config.dwa.max_lookahead_dist,
        )
        if target is None:
            target = engine.goal_position
        engine.current_target = target

        # Instrument DWA planning specifically
        t_dwa_start = time.perf_counter()
        dwa_sol = engine.dwa_planner.compute_command(
            state=current_state,
            target_point=target,
            grid_map=engine.grid_map,
            dynamic_obstacles=engine.dynamic_obstacles,
            robot_radius=engine.config.collision_radius,
            dt=engine.config.dt,
        )
        t_dwa_end = time.perf_counter()
        dwa_lat = t_dwa_end - t_dwa_start
        dwa_latencies.append(dwa_lat)
        candidates_evaluated.append(dwa_sol.evaluated_trajectories)
        safe_candidates_list.append(dwa_sol.collision_free_candidates)

        v_cmd, omega_cmd = dwa_sol.v_cmd, dwa_sol.omega_cmd
        engine.current_candidate_trajectory = dwa_sol.best_trajectory
        engine.dwa_safe = dwa_sol.is_safe
        if not dwa_sol.is_safe:
            engine.dwa_failures += 1

        engine.current_cmd = (v_cmd, omega_cmd)

        # Kinematics and obstacles update
        dt = engine.config.dt
        new_state = engine.robot.step(v_cmd, omega_cmd, dt)
        engine.trajectory.append(new_state)
        engine.steps_taken += 1
        engine.simulated_time = round(engine.steps_taken * dt, 4)

        for obs in engine.dynamic_obstacles:
            obs.step(dt)

        # Collisions
        from autonomous_nav.simulation.collision import CollisionChecker

        if CollisionChecker.check_disc_collision(
            new_state.x, new_state.y, engine.config.collision_radius, engine.grid_map
        ):
            engine.collision_occurred = True
            engine.is_done = True
            engine.termination_reason = "collision"

        for obs in engine.dynamic_obstacles:
            if math.hypot(new_state.x - obs.x, new_state.y - obs.y) <= (engine.config.collision_radius + obs.radius):
                engine.collision_occurred = True
                engine.collided_with_dynamic = True
                engine.is_done = True
                engine.termination_reason = "collision"
                break

        t_step_end = time.perf_counter()
        step_latencies.append(t_step_end - t_step_start)

    t_loop_total = time.perf_counter() - t_loop_start
    total_wall_clock = t_setup + t_loop_total

    result = engine.get_result()

    metrics = {
        "scenario": scenario_name,
        "nav_mode": "dwa",
        "outcome": result.termination_reason,
        "success": result.success,
        "collision": result.collision,
        "collided_with_dynamic": result.collided_with_dynamic,
        "dwa_failures": result.dwa_failures,
        "steps_taken": len(dwa_latencies),
        "simulated_time_seconds": result.simulated_time,
        "setup_wall_clock_seconds": t_setup,
        "loop_wall_clock_seconds": t_loop_total,
        "total_wall_clock_seconds": total_wall_clock,
        "wall_to_sim_ratio": total_wall_clock / result.simulated_time if result.simulated_time > 0 else 0.0,
        "total_candidates_evaluated": sum(candidates_evaluated),
        "mean_candidates_per_step": float(np.mean(candidates_evaluated)) if candidates_evaluated else 0.0,
        "mean_safe_candidates_per_step": float(np.mean(safe_candidates_list)) if safe_candidates_list else 0.0,
        # DWA Planner Latency
        "dwa_latency_ms": {
            "mean": float(np.mean(dwa_latencies) * 1000.0) if dwa_latencies else 0.0,
            "median": float(np.median(dwa_latencies) * 1000.0) if dwa_latencies else 0.0,
            "p95": float(percentile(dwa_latencies, 95) * 1000.0) if dwa_latencies else 0.0,
            "max": float(np.max(dwa_latencies) * 1000.0) if dwa_latencies else 0.0,
            "total_seconds": float(sum(dwa_latencies)),
        },
        # Complete Step Latency (including DWA + kinematics + dynamic obstacles + collision checks)
        "complete_step_latency_ms": {
            "mean": float(np.mean(step_latencies) * 1000.0) if step_latencies else 0.0,
            "median": float(np.median(step_latencies) * 1000.0) if step_latencies else 0.0,
            "p95": float(percentile(step_latencies, 95) * 1000.0) if step_latencies else 0.0,
            "max": float(np.max(step_latencies) * 1000.0) if step_latencies else 0.0,
        },
    }
    return metrics


def run_all_profiles() -> Dict[str, Any]:
    print("================================================================================")
    print(" Detailed DWA & Simulation Step Latency Instrumentation Benchmark")
    print("================================================================================")
    records = {}
    for sc_name in ["dynamic_crossing", "dynamic_corridor", "dynamic_chokepoint"]:
        print(f"Profiling {sc_name} ...", flush=True)
        rec = measure_scenario_performance(sc_name)
        records[sc_name] = rec
        dwa_m = rec["dwa_latency_ms"]
        step_m = rec["complete_step_latency_ms"]
        print(
            f"  Steps: {rec['steps_taken']} | Outcome: {rec['outcome']} | Failures: {rec['dwa_failures']}\n"
            f"  DWA Latency (ms): Mean={dwa_m['mean']:.2f}, Med={dwa_m['median']:.2f}, p95={dwa_m['p95']:.2f}, Max={dwa_m['max']:.2f}\n"
            f"  Step Latency (ms): Mean={step_m['mean']:.2f}, p95={step_m['p95']:.2f}, Max={step_m['max']:.2f}\n"
            f"  SimTime: {rec['simulated_time_seconds']:.1f}s | WallClock: {rec['total_wall_clock_seconds']:.3f}s | Ratio: {rec['wall_to_sim_ratio']:.3f}x\n"
            f"  Total Candidates Evaluated: {rec['total_candidates_evaluated']} (~{rec['mean_candidates_per_step']:.1f}/step)\n"
        )
    return records


if __name__ == "__main__":
    records = run_all_profiles()
    out_path = Path("outputs/metrics/dwa_profile_baseline.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(records, f, indent=2)
    print(f"Saved profile records to: {out_path}")

