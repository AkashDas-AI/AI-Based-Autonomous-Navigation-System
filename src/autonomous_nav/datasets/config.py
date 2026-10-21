"""Configuration specifications and camera calibration models for real-world datasets.

Authoritative calibration parameters for TUM RGB-D Freiburg 2 (fr2) sensor:
- Camera: Microsoft Kinect RGB-D (PrimeSense 1.08)
- Focal length: fx = 520.9 px, fy = 521.0 px
- Principal point: cx = 325.1 px, cy = 249.7 px
- Radial/Tangential distortion: k1=0.2312, k2=-0.7849, p1=-0.0033, p2=-0.0001, k3=0.9172
- Depth scaling factor: 5000.0 (1 meter = 5000 integer units in 16-bit PNG)
- Missing/Invalid depth: 0
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsic calibration model.

    Attributes:
        fx: Focal length along horizontal axis in pixels.
        fy: Focal length along vertical axis in pixels.
        cx: Optical center (principal point) X coordinate in pixels.
        cy: Optical center (principal point) Y coordinate in pixels.
        width: Image sensor width in pixels.
        height: Image sensor height in pixels.
        k1, k2, p1, p2, k3: Lens radial and tangential distortion coefficients.
    """
    fx: float = 520.9
    fy: float = 521.0
    cx: float = 325.1
    cy: float = 249.7
    width: int = 640
    height: int = 480
    k1: float = 0.2312
    k2: float = -0.7849
    p1: float = -0.0033
    p2: float = -0.0001
    k3: float = 0.9172

    def __post_init__(self) -> None:
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(f"Focal lengths must be positive, got fx={self.fx}, fy={self.fy}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"Image dimensions must be positive, got {self.width}x{self.height}")
        for val, name in ((self.cx, "cx"), (self.cy, "cy")):
            if not math.isfinite(val):
                raise ValueError(f"Principal point '{name}' must be finite, got {val}")


@dataclass(frozen=True)
class CameraExtrinsics:
    """Extrinsic mounting transformation from camera optical frame to robot body frame (base_link).

    Body frame convention (REP-105):
    - +X: Forward along vehicle longitudinal axis.
    - +Y: Left along vehicle lateral axis.
    - +Z: Upward normal to ground plane.

    Optical camera frame convention:
    - +X: Right across image sensor.
    - +Y: Downward across image sensor.
    - +Z: Forward along optical viewing axis.

    Mounting offsets and calibration provenance:
    - dx: Forward translation from robot center in meters (+0.15m nominal).
    - dy: Lateral translation from robot center in meters (0.00m centered).
    - dz: Vertical translation (height) from robot center/ground in meters (+0.40m nominal).
    - pitch_deg: Downward/upward tilt in degrees (default 0.0, horizontal).
    - yaw_deg: Horizontal rotation relative to robot heading in degrees (default 0.0).
    - roll_deg: Rotation about forward body axis in degrees (default 0.0).

    PROVENANCE & CALIBRATION DISCLOSURE:
    The mounting offsets (dx = +0.15m, dy = 0.00m, dz = +0.40m, pitch = 0.0 deg, yaw = 0.0 deg, roll = 0.0 deg)
    are project-defined engineering assumptions derived from the physical chassis dimensions
    of the Pioneer 3-DX mobile robot top deck. TUM does NOT publish an official rigid-body
    extrinsic calibration file connecting the motion-capture reflective marker tree to the
    Kinect optical center.
    """
    dx: float = 0.15   # 15 cm forward of Pioneer robot center (project assumption)
    dy: float = 0.0    # centered laterally (project assumption)
    dz: float = 0.40   # 40 cm above ground (project assumption)
    pitch_deg: float = 0.0  # nominal horizontal alignment (project assumption)
    yaw_deg: float = 0.0    # nominal forward alignment (project assumption)
    roll_deg: float = 0.0   # nominal roll alignment (project assumption)

    def __post_init__(self) -> None:
        for val, name in (
            (self.dx, "dx"), (self.dy, "dy"), (self.dz, "dz"),
            (self.pitch_deg, "pitch_deg"), (self.yaw_deg, "yaw_deg"),
            (self.roll_deg, "roll_deg"),
        ):
            if not math.isfinite(val):
                raise ValueError(f"Extrinsic parameter '{name}' must be finite, got {val}")


@dataclass
class DatasetConfig:
    """Configuration for real-world dataset ingestion and evaluation.

    Attributes:
        sequence_name: Identifier of the sequence (e.g., 'freiburg2_pioneer_slam2').
        dataset_dir: Directory containing extracted dataset files or ground truth.
        intrinsics: Pinhole camera intrinsic calibration parameters.
        extrinsics: Extrinsic camera mount geometry relative to robot body.
        depth_scale: Conversion divisor from raw 16-bit PNG depth value to meters.
        min_depth: Minimum valid depth in meters (discard near-field noise).
        max_depth: Maximum valid depth in meters (discard far-field uncertainty).
        min_obstacle_height: Minimum body-frame Z height for obstacle classification in meters.
        max_obstacle_height: Maximum body-frame Z height for obstacle classification in meters.
        timestamp_tolerance: Maximum acceptable time difference for pose/image association in seconds.
        grid_resolution: Resolution of destination GridMap in meters/cell.
    """
    sequence_name: str = "freiburg2_pioneer_slam2"
    dataset_dir: Path = field(default_factory=lambda: Path("data/tum_rgbd"))
    intrinsics: CameraIntrinsics = field(default_factory=CameraIntrinsics)
    extrinsics: CameraExtrinsics = field(default_factory=CameraExtrinsics)
    depth_scale: float = 5000.0
    min_depth: float = 0.4
    max_depth: float = 5.0
    min_obstacle_height: float = 0.05   # 5 cm (ignore floor)
    max_obstacle_height: float = 1.20   # 1.20 m (ignore ceiling/overhead)
    timestamp_tolerance: float = 0.02   # 20 ms matching window (100 Hz MoCap vs 30 Hz camera)
    grid_resolution: float = 0.05       # 5 cm per grid cell

    def __post_init__(self) -> None:
        if isinstance(self.dataset_dir, str):
            self.dataset_dir = Path(self.dataset_dir)
        if self.depth_scale <= 0.0:
            raise ValueError(f"depth_scale must be positive, got {self.depth_scale}")
        if self.min_depth < 0.0 or self.max_depth <= self.min_depth:
            raise ValueError(f"Invalid depth range: [{self.min_depth}, {self.max_depth}]")
        if self.min_obstacle_height >= self.max_obstacle_height:
            raise ValueError(
                f"Invalid obstacle height range: [{self.min_obstacle_height}, {self.max_obstacle_height}]"
            )

