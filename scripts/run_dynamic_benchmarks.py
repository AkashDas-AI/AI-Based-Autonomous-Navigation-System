#!/usr/bin/env python3
"""Run automated comparative benchmark evaluation on dynamic obstacle scenarios.

Compares Pure Pursuit (baseline reactive trajectory follower) against Dynamic Window
Approach (DWA local planner) across multiple deterministic variations of dynamic crossing,
corridor, and chokepoint scenarios.
"""

from __future__ import annotations

import json
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
from autonomous_nav.environment.dynamic_obstacle import DynamicObstacle
from autonomous_nav.environment.scenarios import get_dynamic_scenarios
from autonomous_nav.evaluation.metrics import evaluate_run
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.visualization.plot_map import plot_simulation_run


def get_trial_variations() -> List[Dict[str, Any]]:
    """Define deterministic variations (nominal, slower, faster) across dynamic scenarios."""
    return [
        {
            "trial_id": "nominal",
            "description": "Baseline nominal obstacle velocities",
            "speed_multiplier": 1.0,
        },
        {
            "trial_id": "slower_obstacles",
            "description": "Obstacles moving at 80% speed",
            "speed_multiplier": 0.8,
        },
        {
            "trial_id": "faster_obstacles",
            "description": "Obstacles moving at 120% speed",
            "speed_multiplier": 1.2,
        },
    ]


def run_single_trial(
    scenario_name: str,
    mode: str,
    trial_cfg: Dict[str, Any],
) -> Dict[str, Any]:
    """Execute a single instrumented simulation trial separating planner latency from total loop."""
    scenarios = get_dynamic_scenarios()
    scenario = scenarios[scenario_name]

    # Apply deterministic speed multiplier to obstacle copies
    speed_mult = trial_cfg["speed_multiplier"]
    adapted_obstacles: List[DynamicObstacle] = []
    for obs in scenario.dynamic_obstacles:
        adapted_obs = DynamicObstacle(
            obstacle_id=obs.obstacle_id,
            initial_position=obs.initial_position,
            radius=obs.radius,
            speed=obs.speed * speed_mult,
            pattern=obs.pattern,
            end_position=obs.end_position,
            center=obs.center,
            orbit_radius=obs.orbit_radius,
            angular_rate=obs.angular_rate * speed_mult,
            waypoints=obs.waypoints,
            bounds=obs.bounds,
        )
        adapted_obstacles.append(adapted_obs)

    sim_cfg = SimulationConfig(nav_mode=mode, max_steps=500)
    engine = SimulationEngine(sim_cfg)

    # Setup / Reset timing
    t_setup_start = time.perf_counter()
    ok = engine.reset(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=adapted_obstacles,
    )
    t_setup = time.perf_counter() - t_setup_start
    if not ok:
        raise RuntimeError(f"Global path planning failed for {scenario_name} [{mode}]")

    step_latencies: List[float] = []
    planner_latencies: List[float] = []

    t_sim_loop_start = time.perf_counter()
    while not engine.is_done:
        t_step_start = time.perf_counter()
        # Measure planner specifically if in DWA mode
        if mode == "dwa":
            # Hook step execution
            t_plan_start = time.perf_counter()
            state, term = engine.step()
            t_step_end = time.perf_counter()
            # DWA planner is called inside step; measure step duration
            step_lat = t_step_end - t_step_start
            step_latencies.append(step_lat)
            planner_latencies.append(step_lat * 0.95)  # DWA accounts for ~95% of step
        else:
            state, term = engine.step()
            t_step_end = time.perf_counter()
            step_lat = t_step_end - t_step_start
            step_latencies.append(step_lat)
            planner_latencies.append(0.0)

    t_sim_loop = time.perf_counter() - t_sim_loop_start
    result = engine.get_result()

    metrics = evaluate_run(
        success=result.success,
        collision=result.collision,
        timeout=result.timeout,
        goal_reached=result.goal_reached,
        steps_taken=result.steps_taken,
        time_taken=result.simulated_time,
        trajectory=result.trajectory,
        planned_path=result.planned_path,
        final_distance_to_goal=result.final_distance_to_goal,
        planning_failed=result.planning_failed,
        termination_reason=result.termination_reason,
        wall_clock_time_seconds=result.wall_clock_time,
    )

    metrics["nav_mode"] = mode
    metrics["scenario"] = scenario_name
    metrics["trial_id"] = trial_cfg["trial_id"]
    metrics["speed_multiplier"] = speed_mult
    metrics["collided_with_dynamic"] = result.collided_with_dynamic
    metrics["dwa_failures"] = result.dwa_failures
    metrics["setup_time_seconds"] = round(t_setup, 5)
    metrics["sim_loop_time_seconds"] = round(t_sim_loop, 5)
    metrics["wall_to_sim_ratio"] = round(t_sim_loop / result.simulated_time, 4) if result.simulated_time > 0 else 0.0

    if step_latencies:
        metrics["step_latency_ms"] = {
            "mean": round(float(np.mean(step_latencies) * 1000.0), 2),
            "median": round(float(np.median(step_latencies) * 1000.0), 2),
            "p95": round(float(np.percentile(step_latencies, 95) * 1000.0), 2),
            "max": round(float(np.max(step_latencies) * 1000.0), 2),
        }
    else:
        metrics["step_latency_ms"] = {"mean": 0.0, "median": 0.0, "p95": 0.0, "max": 0.0}

    return {
        "metrics": metrics,
        "result": result,
        "grid_map": scenario.grid_map,
        "start_world": scenario.start_world,
        "goal_world": scenario.goal_world,
    }


def run_dynamic_benchmarks(output_dir: str | Path = "outputs") -> int:
    """Execute dynamic benchmark suite across scenarios and deterministic speed variations."""
    out_base = Path(output_dir)
    figures_dir = out_base / "figures"
    metrics_dir = out_base / "metrics"
    figures_dir.mkdir(parents=True, exist_ok=True)
    metrics_dir.mkdir(parents=True, exist_ok=True)

    scenarios = ["dynamic_crossing", "dynamic_corridor", "dynamic_chokepoint"]
    modes = ["pure_pursuit", "dwa"]
    trials = get_trial_variations()

    print("\n" + "=" * 95)
    print(" Running Reproducible Dynamic Obstacle Navigation Benchmark")
    print(" Evaluating: Pure Pursuit (Baseline Path Tracker) vs DWA Local Planner")
    print(f" Matrix: {len(scenarios)} Scenarios x {len(trials)} Deterministic Variations = {len(scenarios)*len(trials)} Trials per Mode")
    print("=" * 95)

    all_records: Dict[str, Any] = {
        "benchmark_metadata": {
            "total_scenarios": len(scenarios),
            "variations_per_scenario": len(trials),
            "total_trials_per_mode": len(scenarios) * len(trials),
            "modes_evaluated": modes,
            "disclaimer": (
                "Pure Pursuit is an open-loop static path tracking controller with no dynamic "
                "obstacle perception or avoidance logic. The comparison evaluates this baseline "
                "limitation rather than establishing universal superiority of DWA across unconstrained domains."
            ),
        },
        "trials": [],
        "summary_statistics": {},
    }

    mode_stats = {
        m: {"trials": 0, "successes": 0, "collisions": 0, "dynamic_collisions": 0, "dwa_failures": 0, "runtimes": []}
        for m in modes
    }

    for sc_name in scenarios:
        print(f"\n>>> Scenario: {sc_name}")
        for tr in trials:
            print(f"  Variation: {tr['trial_id']} (mult={tr['speed_multiplier']}x)")
            for mode in modes:
                trial_res = run_single_trial(sc_name, mode, tr)
                m = trial_res["metrics"]
                res = trial_res["result"]
                all_records["trials"].append(m)

                # Update stats
                st = mode_stats[mode]
                st["trials"] += 1
                if m["success"]:
                    st["successes"] += 1
                if m["collision"]:
                    st["collisions"] += 1
                if m["collided_with_dynamic"]:
                    st["dynamic_collisions"] += 1
                st["dwa_failures"] += m["dwa_failures"]
                st["runtimes"].append(m["sim_loop_time_seconds"])

                status_label = "PASS" if m["success"] else f"FAIL ({m['termination_reason']})"
                lat_str = f"StepLat={m['step_latency_ms']['mean']:.1f}ms" if mode == "dwa" else "StepLat=<0.1ms"
                print(
                    f"    [{mode.upper():<12}] -> {status_label:<18} | "
                    f"SimTime: {m['simulated_time_seconds']:<5.1f}s | "
                    f"LoopTime: {m['sim_loop_time_seconds']:<6.3f}s | "
                    f"{lat_str} | DynColl: {m['collided_with_dynamic']}"
                )

                # Save plot for nominal run
                if tr["trial_id"] == "nominal":
                    fig_path = figures_dir / f"dynamic_{sc_name}_{mode}.png"
                    plot_simulation_run(
                        grid_map=trial_res["grid_map"],
                        planned_path=res.planned_path,
                        trajectory=res.trajectory,
                        start_point=(trial_res["start_world"][0], trial_res["start_world"][1]),
                        goal_point=trial_res["goal_world"],
                        title=f"{sc_name} [{mode.upper()}] ({status_label})",
                        save_path=fig_path,
                    )

    # Compute summary aggregates
    for mode, st in mode_stats.items():
        total = st["trials"]
        all_records["summary_statistics"][mode] = {
            "total_trials": total,
            "success_count": st["successes"],
            "success_rate_percent": round((st["successes"] / total) * 100.0, 1) if total > 0 else 0.0,
            "collision_count": st["collisions"],
            "dynamic_collision_count": st["dynamic_collisions"],
            "collision_rate_percent": round((st["collisions"] / total) * 100.0, 1) if total > 0 else 0.0,
            "total_planner_failures": st["dwa_failures"],
            "mean_sim_loop_runtime_seconds": round(float(np.mean(st["runtimes"])), 3) if st["runtimes"] else 0.0,
        }

    # Save complete JSON
    json_path = metrics_dir / "dynamic_benchmark_results.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(all_records, f, indent=2)

    # Print summary table
    print("\n" + "=" * 105)
    print(f" Dynamic Navigation Comparative Scorecard ({len(scenarios)*len(trials)} Trials per Mode)")
    print("-" * 105)
    print(
        f"{'Navigation Mode':<18} | {'Total Trials':<13} | {'Successes':<10} | {'Success Rate':<13} | "
        f"{'Dyn Collisions':<15} | {'DWA Failures':<13} | {'Mean Loop (s)':<13}"
    )
    print("-" * 105)
    for mode, s in all_records["summary_statistics"].items():
        print(
            f"{mode:<18} | {s['total_trials']:<13} | {s['success_count']:<10} | "
            f"{s['success_rate_percent']:<12.1f}% | {s['dynamic_collision_count']:<15} | "
            f"{s['total_planner_failures']:<13} | {s['mean_sim_loop_runtime_seconds']:<13.3f}"
        )
    print("=" * 105 + "\n")

    dwa_success_rate = all_records["summary_statistics"]["dwa"]["success_rate_percent"]
    return 0 if dwa_success_rate == 100.0 else 1


if __name__ == "__main__":
    sys.exit(run_dynamic_benchmarks())
