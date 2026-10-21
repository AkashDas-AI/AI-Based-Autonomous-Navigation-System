"""Unit tests for Camera Extrinsic Calibration Sensitivity and Robustness (Milestone 7.5).

Verifies mathematical invariants:
1. Zero-perturbation identity: zero delta extrinsics produce zero displacement and 1.0 IoU.
2. Analytical pure translations produce exact predictable displacements.
3. Analytical pure rotations produce predictable arc-length displacements.
4. Height-filtering coupling: pitch and dz modify height retention, yaw does not.
5. Roll rotation mathematical correctness around longitudinal body axis.
6. Extrinsic parameter validation and non-finite rejection.
7. Immutability of default production configuration instances.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import numpy as np
import pytest

from autonomous_nav.datasets.config import CameraExtrinsics, CameraIntrinsics, DatasetConfig
from autonomous_nav.datasets.depth_projection import (
    filter_obstacle_height,
    insert_points_into_grid_map,
    project_body_to_world,
    transform_camera_to_body,
)
from autonomous_nav.datasets.groundtruth import GroundTruthPose
from autonomous_nav.environment.grid_map import GridMap


def test_camera_extrinsics_defaults_and_validation():
    """Verify default nominal hypothesis and validation rules for CameraExtrinsics."""
    nominal = CameraExtrinsics()
    assert nominal.dx == 0.15
    assert nominal.dy == 0.0
    assert nominal.dz == 0.40
    assert nominal.pitch_deg == 0.0
    assert nominal.yaw_deg == 0.0
    assert nominal.roll_deg == 0.0

    # Non-finite parameter rejection
    for param in ("dx", "dy", "dz", "pitch_deg", "yaw_deg", "roll_deg"):
        kwargs = {p: 0.0 for p in ("dx", "dy", "dz", "pitch_deg", "yaw_deg", "roll_deg")}
        kwargs[param] = float("nan")
        with pytest.raises(ValueError, match="must be finite"):
            CameraExtrinsics(**kwargs)

        kwargs[param] = float("inf")
        with pytest.raises(ValueError, match="must be finite"):
            CameraExtrinsics(**kwargs)


def test_analytical_pure_translation_displacement():
    """Verify that pure translation offsets yield exact, uniform 3D point displacements."""
    pts_c = np.array([
        [0.0, 0.0, 2.0],
        [0.5, -0.2, 3.0],
        [-0.4, 0.3, 1.5],
        [1.2, -0.8, 4.0],
    ], dtype=np.float64)

    base_ext = CameraExtrinsics(dx=0.15, dy=0.00, dz=0.40)
    pts_base = transform_camera_to_body(pts_c, base_ext)

    # Pure dx shift (+0.05m)
    dx_ext = CameraExtrinsics(dx=0.20, dy=0.00, dz=0.40)
    pts_dx = transform_camera_to_body(pts_c, dx_ext)
    disp_dx = np.linalg.norm(pts_dx - pts_base, axis=1)
    assert np.allclose(disp_dx, 0.05, atol=1e-10)

    # Pure dy shift (-0.08m)
    dy_ext = CameraExtrinsics(dx=0.15, dy=-0.08, dz=0.40)
    pts_dy = transform_camera_to_body(pts_c, dy_ext)
    disp_dy = np.linalg.norm(pts_dy - pts_base, axis=1)
    assert np.allclose(disp_dy, 0.08, atol=1e-10)

    # Pure dz shift (+0.10m)
    dz_ext = CameraExtrinsics(dx=0.15, dy=0.00, dz=0.50)
    pts_dz = transform_camera_to_body(pts_c, dz_ext)
    disp_dz = np.linalg.norm(pts_dz - pts_base, axis=1)
    assert np.allclose(disp_dz, 0.10, atol=1e-10)


def test_analytical_pure_yaw_rotation():
    """Verify that yaw rotation produces exact arc-length displacement."""
    # Point along viewing axis at distance R = 3.0m (Xc=0, Yc=0, Zc=3.0)
    # Optical to body: Xb = 3.0, Yb = 0.0, Zb = 0.0 (ignoring translation for angular test)
    pts_c = np.array([[0.0, 0.0, 3.0]], dtype=np.float64)

    base_ext = CameraExtrinsics(dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0)
    pts_base = transform_camera_to_body(pts_c, base_ext)

    yaw_angle_deg = 5.0
    yaw_rad = math.radians(yaw_angle_deg)
    yaw_ext = CameraExtrinsics(dx=0.0, dy=0.0, dz=0.0, yaw_deg=yaw_angle_deg)
    pts_yaw = transform_camera_to_body(pts_c, yaw_ext)

    # Expected analytical chord displacement: 2 * R * sin(theta / 2)
    expected_chord = 2.0 * 3.0 * math.sin(yaw_rad / 2.0)
    actual_disp = float(np.linalg.norm(pts_yaw - pts_base))

    assert math.isclose(actual_disp, expected_chord, rel_tol=1e-7)
    # Also verify small angle approximation: chord ~ R * theta
    assert math.isclose(actual_disp, 3.0 * yaw_rad, rel_tol=1e-3)


def test_analytical_pure_roll_rotation():
    """Verify roll rotation about forward longitudinal axis Xb."""
    # Point with Yb = 1.0 (left), Zb = 0.0
    # In camera coords: Xc = -1.0, Yc = 0.0, Zc = 2.0
    pts_c = np.array([[-1.0, 0.0, 2.0]], dtype=np.float64)

    # 90-degree roll banking right: Yb -> 0, Zb -> 1
    roll_90_ext = CameraExtrinsics(dx=0.0, dy=0.0, dz=0.0, roll_deg=90.0)
    pts_roll_90 = transform_camera_to_body(pts_c, roll_90_ext)

    # Xb should be untouched (= Zc = 2.0)
    assert math.isclose(pts_roll_90[0, 0], 2.0, abs_tol=1e-6)
    # Yb' = Yb * cos(90) - Zb * sin(90) = 0.0
    assert math.isclose(pts_roll_90[0, 1], 0.0, abs_tol=1e-6)
    # Zb' = Yb * sin(90) + Zb * cos(90) = 1.0
    assert math.isclose(pts_roll_90[0, 2], 1.0, abs_tol=1e-6)


def test_height_filtering_coupling_pitch_vs_yaw():
    """Verify that pitch and dz alter obstacle height retention, while yaw leaves it invariant."""
    # Create points at various depths and body heights
    # In camera frame:
    # Point 1: close to floor threshold (Zb = 0.06m)
    # Point 2: middle of obstacle band (Zb = 0.50m)
    # Point 3: near ceiling threshold (Zb = 1.15m)
    # Using camera base: Xb = Zc + 0.15, Yb = -Xc + 0.0, Zb = -Yc + 0.40
    # To get Zb = 0.06: -Yc + 0.40 = 0.06 -> Yc = 0.34
    # To get Zb = 0.50: -Yc + 0.40 = 0.50 -> Yc = -0.10
    # To get Zb = 1.15: -Yc + 0.40 = 1.15 -> Yc = -0.75
    pts_c = np.array([
        [0.0, 0.34, 2.0],    # Base Zb = 0.06 (barely retained obstacle)
        [0.0, -0.10, 2.0],   # Base Zb = 0.50 (well inside obstacle band)
        [0.0, -0.75, 2.0],   # Base Zb = 1.15 (barely retained obstacle)
    ], dtype=np.float64)

    base_ext = CameraExtrinsics(dx=0.15, dy=0.0, dz=0.40)
    pts_body_base = transform_camera_to_body(pts_c, base_ext)
    obs_base = filter_obstacle_height(pts_body_base, min_height=0.05, max_height=1.20)
    assert len(obs_base) == 3

    # Yaw perturbation (+5 deg) does NOT affect Zb
    yaw_ext = CameraExtrinsics(dx=0.15, dy=0.0, dz=0.40, yaw_deg=5.0)
    pts_body_yaw = transform_camera_to_body(pts_c, yaw_ext)
    obs_yaw = filter_obstacle_height(pts_body_yaw, min_height=0.05, max_height=1.20)
    assert len(obs_yaw) == 3
    assert np.allclose(pts_body_yaw[:, 2], pts_body_base[:, 2], atol=1e-8)

    # Downward pitch (+5 deg) rotates Xb into -Zb:
    # Point 1 (Zb=0.06, Xb~2.15) plunges downward:
    # zb_rot = -xb * sin(5°) + zb * cos(5°) ~ -2.15 * 0.087 + 0.06 = -0.187 + 0.06 = -0.127m < 0.05m -> REJECTED!
    pitch_ext = CameraExtrinsics(dx=0.15, dy=0.0, dz=0.40, pitch_deg=5.0)
    pts_body_pitch = transform_camera_to_body(pts_c, pitch_ext)
    obs_pitch = filter_obstacle_height(pts_body_pitch, min_height=0.05, max_height=1.20)
    # The low point plunged below the floor cutoff
    assert len(obs_pitch) < 3
    assert pts_body_pitch[0, 2] < 0.05


def test_grid_map_overlap_and_iou_zero_perturbation():
    """Verify that zero perturbation yields exact 1.0 IoU and 100% recall."""
    grid_base = GridMap(width=50, height=50, resolution=0.1, origin=(-2.5, -2.5))
    grid_cand = GridMap(width=50, height=50, resolution=0.1, origin=(-2.5, -2.5))

    # Populate matching cells
    grid_base.set_occupied(10, 10, True)
    grid_base.set_occupied(20, 25, True)
    grid_base.set_occupied(35, 40, True)

    grid_cand.set_occupied(10, 10, True)
    grid_cand.set_occupied(20, 25, True)
    grid_cand.set_occupied(35, 40, True)

    from scripts.run_calibration_sensitivity import compute_grid_map_overlap
    overlap = compute_grid_map_overlap(grid_base, grid_cand)

    assert overlap["iou"] == 1.0
    assert overlap["base_recall"] == 1.0
    assert overlap["candidate_precision"] == 1.0
    assert overlap["new_cells_added"] == 0
    assert overlap["baseline_cells_missed"] == 0


def test_production_config_immutability():
    """Verify default DatasetConfig and CameraExtrinsics instances remain immutable."""
    d_cfg = DatasetConfig()
    c_ext = CameraExtrinsics()

    assert c_ext.dx == 0.15
    assert c_ext.dy == 0.00
    assert c_ext.dz == 0.40
    assert c_ext.pitch_deg == 0.0
    assert c_ext.yaw_deg == 0.0
    assert c_ext.roll_deg == 0.0

    # Frozen dataclass should reject direct attribute assignments
    with pytest.raises(Exception):
        c_ext.dx = 0.25  # type: ignore

    with pytest.raises(Exception):
        c_ext.yaw_deg = 5.0  # type: ignore

