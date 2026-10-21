#!/usr/bin/env python3
"""Run automated experiments and benchmark evaluation across all predefined scenarios."""

from __future__ import annotations

import json
import sys
from pathlib import Path

# Add src/ to path
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.scenarios import get_all_scenarios
from autonomous_nav.evaluation.metrics import evaluate_run
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.visualization.plot_map import plot_simulation_run


def run_all_benchmarks(output_dir: str | Path = "outputs") -> int:
    """Run simulation on all registered scenarios and collect aggregate metrics."""
    out_base = Path(output_dir)
    figures_dir = out_base / "figures"
    metrics_dir = out_base / "metrics"
    figures_dir.mkdir(parents=True, exist_ok=True)
    metrics_dir.mkdir(parents=True, exist_ok=True)

    scenarios = get_all_scenarios()
    config = SimulationConfig()
    engine = SimulationEngine(config)

    print("\n" + "=" * 70)
    print(" Running Autonomous Navigation Multi-Scenario Benchmark Suite")
    print("=" * 70)

    summary_records = {}
    all_passed = True

    for name, scenario in scenarios.items():
        print(f"Testing Scenario: {name} ...", end=" ", flush=True)

        result = engine.run(
            grid_map=scenario.grid_map,
            start_pose=scenario.start_world,
            goal_position=scenario.goal_world,
        )

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

        summary_records[name] = metrics
        status_str = "PASS" if result.success else "FAIL"
        if not result.success:
            all_passed = False
        print(f"[{status_str}] (Sim Time: {metrics['simulated_time_seconds']}s, Compute: {metrics['wall_clock_time_seconds']}s, Mean CTE: {metrics['mean_cross_track_error_meters']}m)")

        # Save plot
        plot_simulation_run(
            grid_map=scenario.grid_map,
            planned_path=result.planned_path,
            trajectory=result.trajectory,
            start_point=(scenario.start_world[0], scenario.start_world[1]),
            goal_point=scenario.goal_world,
            title=f"Benchmark: {scenario.name} ({status_str})",
            save_path=figures_dir / f"benchmark_{name}.png",
        )

    # Save aggregated benchmark results
    benchmark_file = metrics_dir / "benchmark_results.json"
    with open(benchmark_file, "w", encoding="utf-8") as f:
        json.dump(summary_records, f, indent=2)

    print("\n" + "=" * 80)
    print(f" Aggregate Benchmark Summary (Saved to {benchmark_file})")
    print("-" * 80)
    print(f"{'Scenario':<18} | {'Success':<8} | {'Path (m)':<9} | {'Driven (m)':<10} | {'Mean CTE (m)':<12} | {'Sim (s)':<8} | {'Compute (s)':<11}")
    print("-" * 80)
    for name, m in summary_records.items():
        status = "YES" if m["success"] else "NO"
        print(f"{name:<18} | {status:<8} | {m['planned_path_length_meters']:<9.2f} | {m['actual_trajectory_length_meters']:<10.2f} | {m['mean_cross_track_error_meters']:<12.3f} | {m['simulated_time_seconds']:<8.2f} | {m['wall_clock_time_seconds']:<11.4f}")
    print("=" * 80 + "\n")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(run_all_benchmarks())

