"""Main command-line interface entry point for AI-Based Autonomous Navigation System."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from autonomous_nav.config import load_config
from autonomous_nav.environment.scenarios import get_scenario_by_name, get_all_scenarios
from autonomous_nav.evaluation.metrics import evaluate_run
from autonomous_nav.evaluation.logger import SimulationLogger
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.visualization.plot_map import plot_simulation_run


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Autonomous Navigation Simulation CLI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="simple_obstacles",
        choices=list(get_all_scenarios().keys()),
        help="Benchmark scenario identifier",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to custom JSON configuration file",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate and save Matplotlib plot of executed trajectory",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Launch interactive 2D Pygame visualizer",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="pure_pursuit",
        choices=["pure_pursuit", "dwa"],
        help="Navigation controller / local planner mode ('pure_pursuit' or 'dwa')",
    )
    parser.add_argument(
        "--feedback",
        type=str,
        default="ground_truth",
        choices=["ground_truth", "ekf", "dead_reckoning"],
        help="State estimation feedback source ('ground_truth', 'ekf', or 'dead_reckoning')",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs",
        help="Directory to save generated outputs and metrics",
    )
    return parser.parse_args(args)


def main(args: list[str] | None = None) -> int:
    """Execute autonomous navigation simulation and print results."""
    opts = parse_args(args)
    print(f"\n=======================================================")
    print(f" AI-Based Autonomous Navigation System - Simulation Run")
    print(f"=======================================================")
    print(f"Scenario:    {opts.scenario}")
    print(f"Config:      {opts.config or 'Default'}")
    print(f"Nav Mode:    {opts.mode.upper()}")
    print(f"Feedback:    {opts.feedback.upper()}")

    try:
        config = load_config(opts.config)
        config.nav_mode = opts.mode
        config.state_feedback = opts.feedback
        scenario = get_scenario_by_name(opts.scenario)
    except Exception as e:
        print(f"Initialization Error: {e}", file=sys.stderr)
        return 1

    engine = SimulationEngine(config)

    if opts.interactive:
        from autonomous_nav.visualization.animate import InteractiveVisualizer
        print("Launching interactive 2D Pygame visualizer...")
        visualizer = InteractiveVisualizer(
            engine=engine,
            scenario_name=opts.scenario,
        )
        try:
            visualizer.run()
        finally:
            visualizer.close()
        return 0

    print("Running closed-loop simulation episode...")

    result = engine.run(
        grid_map=scenario.grid_map,
        start_pose=scenario.start_world,
        goal_position=scenario.goal_world,
        dynamic_obstacles=scenario.dynamic_obstacles,
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

    status_str = result.termination_reason.upper()
    print(f"\nSimulation Outcome: {status_str}")
    print(f" - Steps Taken:            {metrics['steps_taken']}")
    print(f" - Simulated Duration:     {metrics['simulated_time_seconds']} s (simulated)")
    print(f" - Wall-Clock Runtime:     {metrics['wall_clock_time_seconds']} s (compute)")
    print(f" - Planned Path Length:    {metrics['planned_path_length_meters']} m")
    print(f" - Driven Trajectory:      {metrics['actual_trajectory_length_meters']} m")
    print(f" - Mean Cross-Track Error: {metrics['mean_cross_track_error_meters']} m")
    print(f" - Max Cross-Track Error:  {metrics['max_cross_track_error_meters']} m")
    print(f" - Final Distance to Goal: {metrics['final_distance_to_goal_meters']} m")
    if result.estimation_metrics:
        print(f" - Pos Estimation RMSE:   {result.estimation_metrics.get('position_rmse', 0.0)} m")
        print(f" - Max Pos Error:         {result.estimation_metrics.get('max_position_error', 0.0)} m")
        print(f" - Heading Est RMSE:      {result.estimation_metrics.get('heading_rmse', 0.0)} rad")
        metrics["estimation_metrics"] = result.estimation_metrics

    out_base = Path(opts.output_dir)
    logger = SimulationLogger(run_id=opts.scenario)
    metrics_path = out_base / "metrics" / f"{opts.scenario}_summary.json"
    logger.save_summary_json(metrics, metrics_path)
    print(f"\nSaved metrics summary to: {metrics_path}")

    if opts.plot:
        fig_path = out_base / "figures" / f"{opts.scenario}_trajectory.png"
        plot_simulation_run(
            grid_map=scenario.grid_map,
            planned_path=result.planned_path,
            trajectory=result.trajectory,
            start_point=(scenario.start_world[0], scenario.start_world[1]),
            goal_point=scenario.goal_world,
            title=f"Autonomous Navigation - {scenario.name} ({status_str})",
            save_path=fig_path,
        )
        print(f"Saved trajectory visualization to: {fig_path}")

    print("=======================================================\n")
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())

