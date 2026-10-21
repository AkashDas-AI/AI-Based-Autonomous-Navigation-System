"""Structured logging and telemetry serialization for simulation runs."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional
from autonomous_nav.simulation.robot import RobotState


class SimulationLogger:
    """Records time-series vehicle states and dumps summary reports."""

    def __init__(self, run_id: Optional[str] = None) -> None:
        self.run_id = run_id or "run_001"
        self.records: List[Dict[str, Any]] = []

    def log_step(
        self,
        step: int,
        timestamp: float,
        state: RobotState,
        v_cmd: float,
        omega_cmd: float,
        dist_to_goal: float,
    ) -> None:
        """Record telemetry for a single simulation time step."""
        self.records.append({
            "step": step,
            "timestamp": round(timestamp, 3),
            "x": round(state.x, 3),
            "y": round(state.y, 3),
            "theta": round(state.theta, 4),
            "v": round(state.v, 3),
            "omega": round(state.omega, 4),
            "v_cmd": round(v_cmd, 3),
            "omega_cmd": round(omega_cmd, 4),
            "dist_to_goal": round(dist_to_goal, 3),
        })

    def save_csv(self, file_path: str | Path) -> Path:
        """Export logged records to CSV file."""
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        if not self.records:
            path.touch()
            return path

        fieldnames = list(self.records[0].keys())
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        return path

    def save_summary_json(self, summary_data: Dict[str, Any], file_path: str | Path) -> Path:
        """Write benchmark/metric summary dictionary to a formatted JSON file."""
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w", encoding="utf-8") as f:
            json.dump(summary_data, f, indent=2)

        return path

