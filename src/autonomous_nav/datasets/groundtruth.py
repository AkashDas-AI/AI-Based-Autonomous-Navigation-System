"""Parser and evaluator for high-frequency motion-capture ground truth trajectories.

TUM RGB-D groundtruth format:
# timestamp tx ty tz qx qy qz qw
Each line contains a float timestamp in seconds, 3D translation (tx, ty, tz) in meters,
and unit orientation quaternion (qx, qy, qz, qw).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from autonomous_nav.bridge.contracts import TimestampedRobotState
from autonomous_nav.bridge.mapping import quaternion_to_euler
from autonomous_nav.simulation.robot import RobotState


@dataclass(frozen=True)
class GroundTruthPose:
    """Individual 6-DoF timestamped ground truth pose record."""
    timestamp: float
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float

    def __post_init__(self) -> None:
        for val, name in (
            (self.timestamp, "timestamp"),
            (self.tx, "tx"), (self.ty, "ty"), (self.tz, "tz"),
            (self.qx, "qx"), (self.qy, "qy"), (self.qz, "qz"), (self.qw, "qw"),
        ):
            if not math.isfinite(val):
                raise ValueError(f"GroundTruthPose '{name}' must be finite, got: {val}")
        if self.timestamp < 0.0:
            raise ValueError(f"GroundTruthPose timestamp must be non-negative, got: {self.timestamp}")

    @property
    def yaw(self) -> float:
        """Extract planar heading angle (yaw) in radians from orientation quaternion."""
        _, _, yaw = quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)
        return yaw

    def to_robot_state(self, v: float = 0.0, omega: float = 0.0) -> RobotState:
        """Project 6-DoF pose into 2D planar RobotState."""
        return RobotState(
            x=self.tx,
            y=self.ty,
            theta=self.yaw,
            v=v,
            omega=omega,
            timestamp=self.timestamp,
        )

    def to_timestamped_robot_state(
        self,
        v_linear: float = 0.0,
        omega_yaw: float = 0.0,
        frame_id: str = "world",
        child_frame_id: str = "base_link",
    ) -> TimestampedRobotState:
        """Convert into canonical high-fidelity TimestampedRobotState contract."""
        roll, pitch, yaw = quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)
        return TimestampedRobotState(
            x=self.tx,
            y=self.ty,
            z=self.tz,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            vx=v_linear * math.cos(yaw),
            vy=v_linear * math.sin(yaw),
            vz=0.0,
            v_linear=v_linear,
            omega_yaw=omega_yaw,
            timestamp=self.timestamp,
            frame_id=frame_id,
            child_frame_id=child_frame_id,
        )


class GroundTruthTrajectory:
    """Time-indexed sequence of ground truth poses with interpolation and metric queries."""

    def __init__(self, poses: List[GroundTruthPose]) -> None:
        if not poses:
            raise ValueError("GroundTruthTrajectory cannot be empty.")
        # Ensure chronological ordering
        self.poses: Tuple[GroundTruthPose, ...] = tuple(sorted(poses, key=lambda p: p.timestamp))
        self._timestamps: np.ndarray = np.array([p.timestamp for p in self.poses], dtype=np.float64)

    def __len__(self) -> int:
        return len(self.poses)

    @classmethod
    def load_from_file(cls, filepath: Path | str) -> GroundTruthTrajectory:
        """Load and parse a TUM-format groundtruth.txt trajectory file."""
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"Ground truth file not found: {path.resolve()}")

        poses: List[GroundTruthPose] = []
        with open(path, "r", encoding="utf-8") as f:
            for line_idx, line in enumerate(f, 1):
                clean = line.strip()
                if not clean or clean.startswith("#"):
                    continue
                tokens = clean.split()
                if len(tokens) < 8:
                    raise ValueError(
                        f"Malformed ground truth entry at {path.name}:{line_idx}: expected 8 fields, got {len(tokens)}"
                    )
                try:
                    t = float(tokens[0])
                    tx, ty, tz = float(tokens[1]), float(tokens[2]), float(tokens[3])
                    qx, qy, qz, qw = float(tokens[4]), float(tokens[5]), float(tokens[6]), float(tokens[7])
                    poses.append(GroundTruthPose(timestamp=t, tx=tx, ty=ty, tz=tz, qx=qx, qy=qy, qz=qz, qw=qw))
                except ValueError as err:
                    raise ValueError(f"Numeric parse error at {path.name}:{line_idx}: {err}") from err

        if not poses:
            raise ValueError(f"No valid pose records found in: {path.resolve()}")

        return cls(poses)

    @property
    def start_time(self) -> float:
        return self.poses[0].timestamp

    @property
    def end_time(self) -> float:
        return self.poses[-1].timestamp

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    def get_pose_nearest(self, timestamp: float, tolerance: float = 0.05) -> Optional[GroundTruthPose]:
        """Find the nearest pose record within a specified time tolerance window in seconds."""
        if not math.isfinite(timestamp):
            raise ValueError(f"Query timestamp must be finite, got {timestamp}")

        idx = int(np.searchsorted(self._timestamps, timestamp))
        best_candidate: Optional[GroundTruthPose] = None
        best_diff = float("inf")

        # Check adjacent candidates
        for candidate_idx in (idx - 1, idx, idx + 1):
            if 0 <= candidate_idx < len(self.poses):
                diff = abs(self.poses[candidate_idx].timestamp - timestamp)
                if diff < best_diff:
                    best_diff = diff
                    best_candidate = self.poses[candidate_idx]

        if best_diff <= tolerance:
            return best_candidate
        return None

    def get_pose_interpolated(self, timestamp: float, tolerance: float = 0.05) -> Optional[GroundTruthPose]:
        """Interpolate 3D translation and spherical quaternion rotation at timestamp."""
        if timestamp < self.start_time - tolerance or timestamp > self.end_time + tolerance:
            return None

        # Clamp within trajectory range if within tolerance
        t = max(self.start_time, min(self.end_time, timestamp))
        idx = int(np.searchsorted(self._timestamps, t))

        if idx == 0:
            return self.poses[0] if abs(self.start_time - t) <= tolerance else None
        if idx >= len(self.poses):
            return self.poses[-1] if abs(self.end_time - t) <= tolerance else None

        p0 = self.poses[idx - 1]
        p1 = self.poses[idx]
        dt = p1.timestamp - p0.timestamp

        if dt <= 1e-6:
            return p0

        alpha = (t - p0.timestamp) / dt

        # Linear translation interpolation
        interp_tx = p0.tx + alpha * (p1.tx - p0.tx)
        interp_ty = p0.ty + alpha * (p1.ty - p0.ty)
        interp_tz = p0.tz + alpha * (p1.tz - p0.tz)

        # SLERP for orientation quaternion
        q0 = np.array([p0.qx, p0.qy, p0.qz, p0.qw], dtype=np.float64)
        q1 = np.array([p1.qx, p1.qy, p1.qz, p1.qw], dtype=np.float64)

        # Ensure shortest path on 4D hypersphere
        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        if dot > 0.9995:
            # Linear interpolation for very close orientations
            q_interp = q0 + alpha * (q1 - q0)
        else:
            theta_0 = math.acos(min(1.0, max(-1.0, dot)))
            sin_theta_0 = math.sin(theta_0)
            theta = theta_0 * alpha
            sin_theta = math.sin(theta)
            s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0
            q_interp = (s0 * q0) + (s1 * q1)

        norm = float(np.linalg.norm(q_interp))
        if norm > 1e-12:
            q_interp /= norm

        return GroundTruthPose(
            timestamp=t,
            tx=float(interp_tx),
            ty=float(interp_ty),
            tz=float(interp_tz),
            qx=float(q_interp[0]),
            qy=float(q_interp[1]),
            qz=float(q_interp[2]),
            qw=float(q_interp[3]),
        )

    def compute_summary_statistics(self) -> Dict[str, Any]:
        """Compute aggregate trajectory metrics and kinematics."""
        xs = [p.tx for p in self.poses]
        ys = [p.ty for p in self.poses]
        zs = [p.tz for p in self.poses]

        path_len = 0.0
        angular_dist = 0.0
        speeds = []

        for i in range(1, len(self.poses)):
            p_prev = self.poses[i - 1]
            p_curr = self.poses[i]
            d_pos = math.hypot(p_curr.tx - p_prev.tx, p_curr.ty - p_prev.ty)
            path_len += d_pos
            dt = p_curr.timestamp - p_prev.timestamp
            if dt > 1e-4:
                speeds.append(d_pos / dt)
            # Angular distance
            d_yaw = abs(math.atan2(math.sin(p_curr.yaw - p_prev.yaw), math.cos(p_curr.yaw - p_prev.yaw)))
            angular_dist += d_yaw

        avg_speed = float(np.mean(speeds)) if speeds else 0.0
        max_speed = float(np.max(speeds)) if speeds else 0.0
        duration = self.duration

        return {
            "pose_count": len(self.poses),
            "start_time_s": round(self.start_time, 4),
            "end_time_s": round(self.end_time, 4),
            "duration_s": round(duration, 3),
            "trajectory_length_m": round(path_len, 3),
            "average_speed_mps": round(avg_speed, 4),
            "max_speed_mps": round(max_speed, 4),
            "angular_distance_rad": round(angular_dist, 3),
            "angular_distance_deg": round(math.degrees(angular_dist), 2),
            "bounds_x": (round(float(min(xs)), 3), round(float(max(xs)), 3)),
            "bounds_y": (round(float(min(ys)), 3), round(float(max(ys)), 3)),
            "bounds_z": (round(float(min(zs)), 3), round(float(max(zs)), 3)),
            "span_x_m": round(float(max(xs) - min(xs)), 3),
            "span_y_m": round(float(max(ys) - min(ys)), 3),
        }

