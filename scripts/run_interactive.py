#!/usr/bin/env python3
"""Launch script for the interactive 2D Pygame autonomous navigation simulator."""

import argparse
import sys
from pathlib import Path

# Add src/ to Python path
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from autonomous_nav.config import load_config
from autonomous_nav.environment.scenarios import get_all_scenarios
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.visualization.animate import InteractiveVisualizer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch Interactive Pygame Navigation Simulator",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="simple_obstacles",
        choices=list(get_all_scenarios().keys()),
        help="Initial benchmark scenario",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to custom JSON configuration",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Simulation rendering frame rate",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1100,
        help="Window width in pixels",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Window height in pixels",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without displaying a physical window (headless mode)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="pure_pursuit",
        choices=["pure_pursuit", "dwa"],
        help="Navigation mode: 'pure_pursuit' (baseline) or 'dwa' (local collision avoidance)",
    )
    parser.add_argument(
        "--screenshot",
        type=str,
        default=None,
        help="Optional path to save a visual screenshot artifact",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Optional frame budget before exiting (useful for testing and benchmarks)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("==================================================================")
    print(" Launching Interactive 2D Pygame Autonomous Navigation Simulator")
    print("==================================================================")
    print(f"Scenario:    {args.scenario}")
    print(f"Config:      {args.config or 'Default'}")
    print(f"Nav Mode:    {args.mode.upper()}")
    print(f"Resolution:  {args.width}x{args.height} @ {args.fps} FPS")
    print(f"Headless:    {args.headless}")
    print("------------------------------------------------------------------")
    print("Controls:")
    print("  [SPACE]      Pause / Resume simulation")
    print("  [M]          Toggle Nav Mode (pure_pursuit <-> dwa)")
    print("  [S] / [->]   Step forward one frame (when paused)")
    print("  [R]          Reset current scenario")
    print("  [1 - 7]      Select specific benchmark scenario")
    print("  [N]          Cycle to next scenario")
    print("  [Click]      Add or toggle obstacle cell (auto-replanning)")
    print("  [ESC / Q]    Exit simulator")
    print("==================================================================")

    config = load_config(args.config)
    config.nav_mode = args.mode
    engine = SimulationEngine(config)

    visualizer = InteractiveVisualizer(
        engine=engine,
        scenario_name=args.scenario,
        window_width=args.width,
        window_height=args.height,
        fps=args.fps,
        headless=args.headless,
    )

    try:
        if args.screenshot:
            # Advance simulation a few steps and capture screenshot
            if args.frames:
                visualizer.run(max_frames=args.frames)
            else:
                for _ in range(15):
                    visualizer.engine.step()
            visualizer.render()
            saved_path = visualizer.save_screenshot(args.screenshot)
            print(f"Saved visualizer screenshot to: {saved_path}")
        else:
            visualizer.run(max_frames=args.frames)
    finally:
        visualizer.close()

    print("Interactive simulation session ended.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

