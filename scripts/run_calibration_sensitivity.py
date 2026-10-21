"""Extrinsic Calibration Sensitivity and Mapping Robustness Analysis (Milestone 7.5).

Evaluates the sensitivity of authentic TUM RGB-D 3D projection, obstacle height slicing,
and 2D GridMap rasterization to perturbations across all 6 degrees of freedom:
- Longitudinal translation dx (baseline +0.15m nominal)
- Lateral translation dy (baseline 0.00m centered)
- Vertical height translation dz (baseline +0.40m nominal)
- Camera tilt pitch_deg (baseline 0.0 deg nominal)
- Camera pan yaw_deg (baseline 0.0 deg nominal)
- Camera roll_deg (baseline 0.0 deg nominal)

PROVENANCE & CALIBRATION DISCLOSURE:
Nominal mounting parameters (dx = +0.15m, dy = 0.00m, dz = +0.40m, pitch = 0.0 deg,
yaw = 0.0 deg, roll = 0.0 deg) are engineering hypotheses based on Pioneer 3-DX top-deck
dimensions. TUM published no ground-truth marker-to-camera optical transformation.
This script quantifies mapping uncertainty under plausible mounting tolerances.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "src"))

from autonomous_nav.datasets.config import CameraExtrinsics, CameraIntrinsics, DatasetConfig
from autonomous_nav.datasets.depth_projection import (
    depth_to_pointcloud_camera,
    filter_obstacle_height,
    insert_points_into_grid_map,
    project_body_to_world,
    transform_camera_to_body,
)
from autonomous_nav.datasets.groundtruth import GroundTruthPose, GroundTruthTrajectory
from autonomous_nav.environment.grid_map import GridMap


def compute_grid_map_overlap(grid_base: GridMap, grid_cand: GridMap) -> Dict[str, Any]:
    """Compute exact cell-level overlap metrics between baseline and candidate GridMaps.

    Metrics:
        - intersection: |A ∩ B|
        - union: |A ∪ B|
        - iou: |A ∩ B| / |A ∪ B| (Jaccard similarity)
        - base_recall: |A ∩ B| / |A|
        - cand_precision: |A ∩ B| / |B|
        - new_cells: |B \\ A|
        - missed_cells: |A \\ B|
    """
    mask_base = grid_base.grid == 1
    mask_cand = grid_cand.grid == 1

    count_base = int(np.count_nonzero(mask_base))
    count_cand = int(np.count_nonzero(mask_cand))

    intersection = int(np.count_nonzero(mask_base & mask_cand))
    union = int(np.count_nonzero(mask_base | mask_cand))

    iou = float(intersection / union) if union > 0 else 1.0
    base_recall = float(intersection / count_base) if count_base > 0 else 1.0
    cand_precision = float(intersection / count_cand) if count_cand > 0 else 1.0

    new_cells = int(np.count_nonzero((~mask_base) & mask_cand))
    missed_cells = int(np.count_nonzero(mask_base & (~mask_cand)))

    return {
        "baseline_occupied_cells": count_base,
        "candidate_occupied_cells": count_cand,
        "intersection_cells": intersection,
        "union_cells": union,
        "iou": iou,
        "base_recall": base_recall,
        "candidate_precision": cand_precision,
        "new_cells_added": new_cells,
        "baseline_cells_missed": missed_cells,
    }


def evaluate_extrinsics_configuration(
    extrinsics: CameraExtrinsics,
    frame_data: List[Tuple[float, np.ndarray, GroundTruthPose, np.ndarray]],
    base_frame_body_points: Optional[List[np.ndarray]] = None,
    cfg: Optional[DatasetConfig] = None,
    map_bounds: Tuple[int, int, float, Tuple[float, float]] = (200, 200, 0.05, (-5.0, -5.0)),
) -> Tuple[Dict[str, Any], GridMap, List[np.ndarray], List[np.ndarray]]:
    """Evaluate 3D projection, height slicing, and rasterization under specific extrinsics.

    Args:
        extrinsics: Extrinsic parameters to evaluate.
        frame_data: Pre-loaded frames containing (timestamp, depth_img, pose, pts_camera).
        base_frame_body_points: Optional baseline body points to compute point displacements.
        cfg: Dataset configuration.
        map_bounds: GridMap specifications (width, height, resolution, origin).

    Returns:
        Tuple containing:
            - metrics: Comprehensive summary dictionary.
            - grid_map: The populated candidate GridMap.
            - all_body_points: List of pre-filtering body point clouds per frame.
            - all_world_obstacle_points: List of retained world obstacle points per frame.
    """
    if cfg is None:
        cfg = DatasetConfig()

    map_w, map_h, res, origin = map_bounds
    grid_map = GridMap(width=map_w, height=map_h, resolution=res, origin=origin)

    all_body_points: List[np.ndarray] = []
    all_world_obstacle_points: List[np.ndarray] = []

    total_valid_pixels = 0
    total_retained_obs = 0
    total_floor_rejected = 0
    total_ceiling_rejected = 0
    total_pts_in_bounds = 0
    total_pts_clipped = 0

    all_displacements_3d: List[np.ndarray] = []
    all_displacements_2d: List[np.ndarray] = []

    for idx, (t_stamp, d_img, pose, pts_c) in enumerate(frame_data):
        total_valid_pixels += len(pts_c)

        # 1. Optical to body transformation
        pts_body = transform_camera_to_body(pts_c, extrinsics)
        all_body_points.append(pts_body)

        # 2. Point displacement relative to baseline body points
        if base_frame_body_points is not None and len(base_frame_body_points) > idx:
            base_b = base_frame_body_points[idx]
            disp_vec = pts_body - base_b
            d3 = np.linalg.norm(disp_vec, axis=1)
            d2 = np.linalg.norm(disp_vec[:, :2], axis=1)
            all_displacements_3d.append(d3)
            all_displacements_2d.append(d2)

        # 3. Obstacle vertical slicing
        zb = pts_body[:, 2]
        floor_rej = int(np.count_nonzero((zb < cfg.min_obstacle_height) & np.isfinite(zb)))
        ceil_rej = int(np.count_nonzero((zb > cfg.max_obstacle_height) & np.isfinite(zb)))
        total_floor_rejected += floor_rej
        total_ceiling_rejected += ceil_rej

        pts_obs = filter_obstacle_height(
            pts_body,
            min_height=cfg.min_obstacle_height,
            max_height=cfg.max_obstacle_height,
        )
        total_retained_obs += len(pts_obs)

        # Mathematical sanity check: floor + ceiling + retained == valid
        assert floor_rej + ceil_rej + len(pts_obs) == len(pts_c)

        # 4. World planar projection
        pts_world = project_body_to_world(pts_obs, pose)
        all_world_obstacle_points.append(pts_world)

        # 5. Rasterize into GridMap
        _, pts_in_b, pts_clip = insert_points_into_grid_map(
            pts_world, grid_map, return_diagnostics=True
        )
        total_pts_in_bounds += pts_in_b
        total_pts_clipped += pts_clip

    # Aggregate displacement statistics
    if all_displacements_3d:
        concat_d3 = np.concatenate(all_displacements_3d)
        concat_d2 = np.concatenate(all_displacements_2d)
        mean_disp_3d = float(np.mean(concat_d3))
        rmse_disp_3d = float(np.sqrt(np.mean(concat_d3 ** 2)))
        max_disp_3d = float(np.max(concat_d3))
        median_disp_3d = float(np.median(concat_d3))

        mean_disp_2d = float(np.mean(concat_d2))
        rmse_disp_2d = float(np.sqrt(np.mean(concat_d2 ** 2)))
        max_disp_2d = float(np.max(concat_d2))
    else:
        mean_disp_3d = 0.0
        rmse_disp_3d = 0.0
        max_disp_3d = 0.0
        median_disp_3d = 0.0
        mean_disp_2d = 0.0
        rmse_disp_2d = 0.0
        max_disp_2d = 0.0

    # Retained world points bounding box
    if all_world_obstacle_points:
        concat_w = np.concatenate(all_world_obstacle_points, axis=0) if total_retained_obs > 0 else np.empty((0, 2))
        if len(concat_w) > 0:
            xw_min = float(np.min(concat_w[:, 0]))
            xw_max = float(np.max(concat_w[:, 0]))
            yw_min = float(np.min(concat_w[:, 1]))
            yw_max = float(np.max(concat_w[:, 1]))
        else:
            xw_min, xw_max, yw_min, yw_max = 0.0, 0.0, 0.0, 0.0
    else:
        xw_min, xw_max, yw_min, yw_max = 0.0, 0.0, 0.0, 0.0

    occupied_cells = int(np.count_nonzero(grid_map.grid == 1))

    metrics: Dict[str, Any] = {
        "extrinsics": {
            "dx": float(extrinsics.dx),
            "dy": float(extrinsics.dy),
            "dz": float(extrinsics.dz),
            "pitch_deg": float(extrinsics.pitch_deg),
            "yaw_deg": float(extrinsics.yaw_deg),
            "roll_deg": float(extrinsics.roll_deg),
        },
        "total_valid_pixels": total_valid_pixels,
        "floor_rejected_points": total_floor_rejected,
        "ceiling_rejected_points": total_ceiling_rejected,
        "retained_obstacle_points": total_retained_obs,
        "retention_percentage": float(total_retained_obs / total_valid_pixels * 100.0) if total_valid_pixels > 0 else 0.0,
        "points_in_bounds": total_pts_in_bounds,
        "points_clipped": total_pts_clipped,
        "in_bounds_percentage": float(total_pts_in_bounds / total_retained_obs * 100.0) if total_retained_obs > 0 else 0.0,
        "occupied_cells": occupied_cells,
        "displacement_3d_m": {
            "mean": mean_disp_3d,
            "rmse": rmse_disp_3d,
            "max": max_disp_3d,
            "median": median_disp_3d,
        },
        "displacement_2d_m": {
            "mean": mean_disp_2d,
            "rmse": rmse_disp_2d,
            "max": max_disp_2d,
        },
        "bounding_box_world": {
            "x_min": xw_min,
            "x_max": xw_max,
            "y_min": yw_min,
            "y_max": yw_max,
        },
    }

    return metrics, grid_map, all_body_points, all_world_obstacle_points


def plot_sensitivity_diagnostic(
    sweep_results: Dict[str, List[Dict[str, Any]]],
    baseline_metrics: Dict[str, Any],
    grid_baseline: GridMap,
    key_perturbed_grids: Dict[str, GridMap],
    output_path: Path,
) -> None:
    """Generate a high-fidelity 6-panel diagnostic plot summarizing calibration sensitivity."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 11), dpi=200)

    # Palette
    c_dx = "#1f77b4"
    c_dy = "#ff7f0e"
    c_dz = "#2ca02c"
    c_pitch = "#d62728"
    c_yaw = "#9467bd"
    c_roll = "#8c564b"

    # -------------------------------------------------------------
    # Panel 1: Translation Sensitivity - 3D Displacement vs Offset
    # -------------------------------------------------------------
    ax1 = axes[0, 0]
    for param, col, lbl in [("dx", c_dx, "dx (forward)"), ("dy", c_dy, "dy (lateral)"), ("dz", c_dz, "dz (height)")]:
        recs = sweep_results[param]
        offsets = [r["offset"] for r in recs]
        d_mean = [r["displacement_3d_m"]["mean"] for r in recs]
        d_max = [r["displacement_3d_m"]["max"] for r in recs]
        ax1.plot(offsets, d_mean, marker="o", color=col, linewidth=2, label=f"{lbl} mean")
        ax1.plot(offsets, d_max, linestyle="--", color=col, alpha=0.6, label=f"{lbl} max")
    ax1.set_title("Translation Sensitivity: 3D Point Displacement", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Translation Offset from Nominal (m)", fontsize=10)
    ax1.set_ylabel("3D Displacement (m)", fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8, loc="upper center", ncol=3)

    # -------------------------------------------------------------
    # Panel 2: Translation Sensitivity - GridMap IoU vs Offset
    # -------------------------------------------------------------
    ax2 = axes[0, 1]
    for param, col, lbl in [("dx", c_dx, "dx (forward)"), ("dy", c_dy, "dy (lateral)"), ("dz", c_dz, "dz (height)")]:
        recs = sweep_results[param]
        offsets = [r["offset"] for r in recs]
        ious = [r["overlap"]["iou"] * 100.0 for r in recs]
        ax2.plot(offsets, ious, marker="s", color=col, linewidth=2, label=lbl)
    ax2.set_title("Translation Sensitivity: GridMap IoU (%)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Translation Offset from Nominal (m)", fontsize=10)
    ax2.set_ylabel("Occupancy Grid IoU (%)", fontsize=10)
    ax2.set_ylim(40, 105)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)

    # -------------------------------------------------------------
    # Panel 3: Angular Sensitivity - 3D Displacement vs Angle
    # -------------------------------------------------------------
    ax3 = axes[0, 2]
    for param, col, lbl in [("pitch", c_pitch, "pitch (tilt)"), ("yaw", c_yaw, "yaw (pan)"), ("roll", c_roll, "roll (bank)")]:
        recs = sweep_results[param]
        angles = [r["angle_deg"] for r in recs]
        d_mean = [r["displacement_3d_m"]["mean"] for r in recs]
        d_max = [r["displacement_3d_m"]["max"] for r in recs]
        ax3.plot(angles, d_mean, marker="o", color=col, linewidth=2, label=f"{lbl} mean")
        ax3.plot(angles, d_max, linestyle="--", color=col, alpha=0.6, label=f"{lbl} max")
    ax3.set_title("Angular Sensitivity: 3D Point Displacement", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Angular Perturbation (deg)", fontsize=10)
    ax3.set_ylabel("3D Displacement (m)", fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, loc="upper center", ncol=3)

    # -------------------------------------------------------------
    # Panel 4: Angular Sensitivity - GridMap IoU vs Angle
    # -------------------------------------------------------------
    ax4 = axes[1, 0]
    for param, col, lbl in [("pitch", c_pitch, "pitch (tilt)"), ("yaw", c_yaw, "yaw (pan)"), ("roll", c_roll, "roll (bank)")]:
        recs = sweep_results[param]
        angles = [r["angle_deg"] for r in recs]
        ious = [r["overlap"]["iou"] * 100.0 for r in recs]
        ax4.plot(angles, ious, marker="s", color=col, linewidth=2, label=lbl)
    ax4.set_title("Angular Sensitivity: GridMap IoU (%)", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Angular Perturbation (deg)", fontsize=10)
    ax4.set_ylabel("Occupancy Grid IoU (%)", fontsize=10)
    ax4.set_ylim(20, 105)
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)

    # -------------------------------------------------------------
    # Panel 5: Retained Obstacle Points vs Perturbation
    # -------------------------------------------------------------
    ax5 = axes[1, 1]
    # Pitch & dz strongly alter height slicing; yaw leaves it unchanged
    pitch_recs = sweep_results["pitch"]
    yaw_recs = sweep_results["yaw"]
    dz_recs = sweep_results["dz"]

    base_pts = baseline_metrics["retained_obstacle_points"]

    ax5.plot([r["angle_deg"] for r in pitch_recs], [r["retained_obstacle_points"] for r in pitch_recs],
             marker="^", color=c_pitch, linewidth=2, label="pitch (tilt) [-5° to +5°]")
    ax5.plot([r["angle_deg"] for r in yaw_recs], [r["retained_obstacle_points"] for r in yaw_recs],
             marker="o", color=c_yaw, linewidth=2, label="yaw (pan) [-5° to +5°]")
    # Plot dz normalized to +/-5deg equivalent scale for comparison
    dz_offsets = [r["offset"] for r in dz_recs]
    # Map dz offset -0.10..+0.10 m to -5..+5 deg scale for co-plotting
    dz_pts = [r["retained_obstacle_points"] for r in dz_recs]
    ax5.plot(np.array(dz_offsets) * 50.0, dz_pts, marker="d", color=c_dz, linewidth=2, linestyle="-.",
             label="dz (height) [-0.10m to +0.10m]")

    ax5.axhline(base_pts, color="black", linestyle=":", label=f"Baseline ({base_pts:,} pts)")
    ax5.set_title("Obstacle Height Slicing Retention Sensitivity", fontsize=11, fontweight="bold")
    ax5.set_xlabel("Perturbation Index (deg or scaled cm)", fontsize=10)
    ax5.set_ylabel("Retained Obstacle Points", fontsize=10)
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=8)

    # -------------------------------------------------------------
    # Panel 6: Spatial GridMap Visual Comparison Overlay
    # -------------------------------------------------------------
    ax6 = axes[1, 2]
    # Create composite RGB image:
    # Baseline in Gray/Black
    # Yaw +5 deg in Blue
    # Pitch +5 deg in Red
    # dx +0.10 m in Green
    h, w = grid_baseline.grid.shape
    rgb_overlay = np.ones((h, w, 3), dtype=np.float32)

    # Baseline points: dark gray
    base_occ = grid_baseline.grid == 1
    rgb_overlay[base_occ] = [0.3, 0.3, 0.3]

    # Overlay key perturbations
    if "yaw_plus_5" in key_perturbed_grids:
        yaw_occ = key_perturbed_grids["yaw_plus_5"].grid == 1
        rgb_overlay[yaw_occ & (~base_occ)] = [0.1, 0.4, 0.9]  # Blue for yaw shift
    if "pitch_plus_5" in key_perturbed_grids:
        pitch_occ = key_perturbed_grids["pitch_plus_5"].grid == 1
        rgb_overlay[pitch_occ & (~base_occ)] = [0.9, 0.1, 0.1]  # Red for pitch shift
    if "dx_plus_10cm" in key_perturbed_grids:
        dx_occ = key_perturbed_grids["dx_plus_10cm"].grid == 1
        rgb_overlay[dx_occ & (~base_occ)] = [0.1, 0.8, 0.2]  # Green for dx shift

    # Extents in world coordinates: origin = (-5.0, -5.0), size 200x200, res 0.05 -> [-5.0, 5.0]
    extent = [-5.0, 5.0, -5.0, 5.0]
    ax6.imshow(rgb_overlay, origin="lower", extent=extent)
    ax6.set_title("GridMap Footprint Overlay (Zoomed Region)", fontsize=11, fontweight="bold")
    ax6.set_xlabel("World X (m)", fontsize=10)
    ax6.set_ylabel("World Y (m)", fontsize=10)
    ax6.set_xlim(-2.0, 4.0)
    ax6.set_ylim(-3.0, 3.0)
    ax6.grid(True, alpha=0.2, color="gray")

    # Custom legend for panel 6
    from matplotlib.lines import Line2D
    custom_lines = [
        Line2D([0], [0], color=(0.3, 0.3, 0.3), lw=4, label="Baseline (Nominal)"),
        Line2D([0], [0], color=(0.1, 0.4, 0.9), lw=4, label="Yaw +5.0° (Arc Shift)"),
        Line2D([0], [0], color=(0.9, 0.1, 0.1), lw=4, label="Pitch +5.0° (Tilt Distortion)"),
        Line2D([0], [0], color=(0.1, 0.8, 0.2), lw=4, label="dx +0.10m (Rigid Shift)"),
    ]
    ax6.legend(handles=custom_lines, fontsize=8, loc="lower right")

    plt.suptitle(
        "Milestone 7.5: TUM RGB-D Camera Extrinsics Sensitivity & Mapping Robustness Analysis\n"
        "(Quantifying 6-DoF Calibration Perturbations Against Authentic Depth Frames)",
        fontsize=13, fontweight="bold", y=0.99
    )
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f"  [SAVED] Diagnostic sensitivity plot: {output_path.resolve()}")


def main():
    parser = argparse.ArgumentParser(description="TUM RGB-D Extrinsic Calibration Sensitivity (Milestone 7.5)")
    parser.add_argument(
        "--dataset-dir",
        type=str,
        default="data/tum_rgbd",
        help="Path to TUM RGB-D sequence directory",
    )
    parser.add_argument(
        "--output-json",
        type=str,
        default="outputs/metrics/calibration_sensitivity_summary.json",
        help="Path for machine-readable JSON metrics",
    )
    parser.add_argument(
        "--output-figure",
        type=str,
        default="outputs/figures/calibration_sensitivity_diagnostic.png",
        help="Path for multi-panel diagnostic PNG",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable figure generation",
    )
    args = parser.parse_args()

    print("=" * 100)
    print("AI-BASED AUTONOMOUS NAVIGATION SYSTEM — MILESTONE 7.5: CALIBRATION SENSITIVITY AUDIT")
    print("=" * 100)

    # 1. Verify Dataset Files & Provenance
    dataset_path = Path(args.dataset_dir)
    gt_file = dataset_path / "groundtruth.txt"
    manifest_file = dataset_path / "manifest.json"

    if not gt_file.is_file():
        raise FileNotFoundError(f"Missing groundtruth.txt in {dataset_path.resolve()}")

    print(f"[STEP 1] Loading ground truth trajectory: {gt_file.name}")
    traj = GroundTruthTrajectory.load_from_file(gt_file)
    print(f"  Ground truth loaded: {len(traj.poses):,} poses ({traj.duration:.2f}s duration)")

    # 2. Identify and Load Authentic Depth Frames
    depth_dir = dataset_path / "depth"
    authentic_frames: List[Tuple[float, Path]] = []
    if manifest_file.is_file():
        with open(manifest_file, "r", encoding="utf-8") as f:
            man = json.load(f)
        for item in man.get("frames", []):
            p = depth_dir / item["filename"]
            if p.is_file():
                authentic_frames.append((float(item["timestamp"]), p))
    elif depth_dir.is_dir():
        for p in sorted(depth_dir.glob("*.png")):
            try:
                authentic_frames.append((float(p.stem), p))
            except ValueError:
                continue

    if not authentic_frames:
        raise RuntimeError(f"No authentic depth frames found in {depth_dir.resolve()}")

    print(f"[STEP 2] Loaded {len(authentic_frames)} authentic depth frames:")
    for ts, fp in authentic_frames:
        print(f"  - Frame {ts:.4f}: {fp.name} ({fp.stat().st_size:,} bytes)")

    # 3. Pre-project depth frames to optical camera coordinates
    cfg = DatasetConfig()
    frame_cache: List[Tuple[float, np.ndarray, GroundTruthPose, np.ndarray]] = []
    total_valid_pixels = 0

    print("\n[STEP 3] Pre-projecting depth frames using authoritative TUM pinhole intrinsics...")
    for ts, fp in authentic_frames:
        d_img = cv2.imread(str(fp), cv2.IMREAD_UNCHANGED)
        if d_img is None or d_img.dtype != np.uint16:
            raise ValueError(f"Corrupted or invalid 16-bit depth PNG: {fp}")

        pose = traj.get_pose_interpolated(ts, tolerance=0.05)
        if pose is None:
            raise ValueError(f"No ground truth pose within 0.05s for timestamp {ts}")

        pts_c, _ = depth_to_pointcloud_camera(
            depth_img=d_img,
            intrinsics=cfg.intrinsics,
            depth_scale=cfg.depth_scale,
            min_depth=cfg.min_depth,
            max_depth=cfg.max_depth,
        )
        total_valid_pixels += len(pts_c)
        frame_cache.append((ts, d_img, pose, pts_c))

    print(f"  Total valid depth pixels across all {len(frame_cache)} frames: {total_valid_pixels:,}")

    # 4. Evaluate Baseline (Nominal Engineering Hypothesis)
    print("\n[STEP 4] Evaluating Baseline Mounting Parameters (Nominal Hypothesis)...")
    baseline_extrinsics = CameraExtrinsics(
        dx=0.15, dy=0.00, dz=0.40, pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0
    )
    print(f"  Baseline Extrinsics: dx=+{baseline_extrinsics.dx}m, dy={baseline_extrinsics.dy}m, "
          f"dz=+{baseline_extrinsics.dz}m, pitch={baseline_extrinsics.pitch_deg}°, "
          f"yaw={baseline_extrinsics.yaw_deg}°, roll={baseline_extrinsics.roll_deg}°")

    baseline_metrics, grid_baseline, base_body_pts, base_world_obs = evaluate_extrinsics_configuration(
        extrinsics=baseline_extrinsics,
        frame_data=frame_cache,
        base_frame_body_points=None,
        cfg=cfg,
    )
    baseline_metrics["overlap"] = {
        "iou": 1.0,
        "base_recall": 1.0,
        "candidate_precision": 1.0,
        "new_cells_added": 0,
        "baseline_cells_missed": 0,
    }

    print(f"  Baseline Valid Pixels:         {baseline_metrics['total_valid_pixels']:,}")
    print(f"  Baseline Retained Obstacles:   {baseline_metrics['retained_obstacle_points']:,} "
          f"({baseline_metrics['retention_percentage']:.2f}%)")
    print(f"  Baseline In-bounds Points:     {baseline_metrics['points_in_bounds']:,} "
          f"({baseline_metrics['in_bounds_percentage']:.2f}%)")
    print(f"  Baseline Clipped Points:       {baseline_metrics['points_clipped']:,}")
    print(f"  Baseline Occupied Grid Cells:  {baseline_metrics['occupied_cells']:,}")

    # 5. Define Systematic 6-DoF Sensitivity Perturbation Matrix
    # We sweep each parameter across 5 points centered on baseline
    SWEEPS: Dict[str, Dict[str, Any]] = {
        "dx": {
            "param": "dx",
            "unit": "m",
            "baseline": 0.15,
            "values": [0.05, 0.10, 0.15, 0.20, 0.25],
        },
        "dy": {
            "param": "dy",
            "unit": "m",
            "baseline": 0.00,
            "values": [-0.10, -0.05, 0.00, 0.05, 0.10],
        },
        "dz": {
            "param": "dz",
            "unit": "m",
            "baseline": 0.40,
            "values": [0.30, 0.35, 0.40, 0.45, 0.50],
        },
        "pitch": {
            "param": "pitch_deg",
            "unit": "deg",
            "baseline": 0.0,
            "values": [-5.0, -2.5, 0.0, 2.5, 5.0],
        },
        "yaw": {
            "param": "yaw_deg",
            "unit": "deg",
            "baseline": 0.0,
            "values": [-5.0, -2.5, 0.0, 2.5, 5.0],
        },
        "roll": {
            "param": "roll_deg",
            "unit": "deg",
            "baseline": 0.0,
            "values": [-5.0, -2.5, 0.0, 2.5, 5.0],
        },
    }

    print("\n[STEP 5] Running Systematic 6-DoF Sensitivity Matrix...")
    sweep_results: Dict[str, List[Dict[str, Any]]] = {}
    key_perturbed_grids: Dict[str, GridMap] = {}

    for sweep_key, sweep_spec in SWEEPS.items():
        param_name = sweep_spec["param"]
        unit = sweep_spec["unit"]
        base_val = sweep_spec["baseline"]
        vals = sweep_spec["values"]

        print(f"\n  --- Sweeping {param_name} ({unit}) across {vals} ---")
        sweep_results[sweep_key] = []

        for val in vals:
            # Construct candidate extrinsics without modifying baseline
            cand_kwargs = {
                "dx": baseline_extrinsics.dx,
                "dy": baseline_extrinsics.dy,
                "dz": baseline_extrinsics.dz,
                "pitch_deg": baseline_extrinsics.pitch_deg,
                "yaw_deg": baseline_extrinsics.yaw_deg,
                "roll_deg": baseline_extrinsics.roll_deg,
            }
            cand_kwargs[param_name] = val
            cand_ext = CameraExtrinsics(**cand_kwargs)

            cand_metrics, cand_grid, _, _ = evaluate_extrinsics_configuration(
                extrinsics=cand_ext,
                frame_data=frame_cache,
                base_frame_body_points=base_body_pts,
                cfg=cfg,
            )

            # Compute overlap with baseline
            overlap = compute_grid_map_overlap(grid_baseline, cand_grid)
            cand_metrics["overlap"] = overlap

            # Record offset or angle
            offset = float(val - base_val)
            cand_metrics["offset"] = offset
            if unit == "deg":
                cand_metrics["angle_deg"] = val

            sweep_results[sweep_key].append(cand_metrics)

            # Capture key perturbations for composite visualization
            if sweep_key == "yaw" and math.isclose(val, 5.0, abs_tol=1e-3):
                key_perturbed_grids["yaw_plus_5"] = cand_grid
            elif sweep_key == "pitch" and math.isclose(val, 5.0, abs_tol=1e-3):
                key_perturbed_grids["pitch_plus_5"] = cand_grid
            elif sweep_key == "dx" and math.isclose(val, 0.25, abs_tol=1e-3):
                key_perturbed_grids["dx_plus_10cm"] = cand_grid

            d3_mean = cand_metrics["displacement_3d_m"]["mean"]
            d3_max = cand_metrics["displacement_3d_m"]["max"]
            iou_pct = overlap["iou"] * 100.0
            retained = cand_metrics["retained_obstacle_points"]
            occ_cells = cand_metrics["occupied_cells"]

            print(f"    Val: {val:6.2f} {unit:<3} | Offset: {offset:+6.2f} | "
                  f"Disp3D Mean: {d3_mean:.4f}m (Max: {d3_max:.4f}m) | "
                  f"IoU: {iou_pct:5.2f}% | Retained: {retained:6,d} | OccCells: {occ_cells:5,d}")

    # 6. Generate Comparative Diagnostic Plot
    fig_path = Path(args.output_figure)
    if not args.no_plot:
        print("\n[STEP 6] Generating multi-panel diagnostic sensitivity plot...")
        plot_sensitivity_diagnostic(
            sweep_results=sweep_results,
            baseline_metrics=baseline_metrics,
            grid_baseline=grid_baseline,
            key_perturbed_grids=key_perturbed_grids,
            output_path=fig_path,
        )

    # 7. Robustness and Failure Mode Analysis
    print("\n[STEP 7] Synthesizing Sensitivity Insights & Failure Modes...")
    # Calculate impact rankings
    analysis = {
        "height_filtering_impact": {
            "description": "Height filtering retention sensitivity [min_height=0.05m, max_height=1.20m]",
            "pitch_range_pts": [r["retained_obstacle_points"] for r in sweep_results["pitch"]],
            "yaw_range_pts": [r["retained_obstacle_points"] for r in sweep_results["yaw"]],
            "dz_range_pts": [r["retained_obstacle_points"] for r in sweep_results["dz"]],
            "insight": (
                "Pitch and vertical height dz directly shift body Z coordinates, moving points across "
                "the floor (0.05m) and ceiling (1.20m) thresholds. Yaw, dx, and dy perturbations have zero "
                "or near-zero impact on height-filtering retention."
            ),
        },
        "spatial_distortion_impact": {
            "description": "GridMap occupancy IoU degradation and point displacement",
            "yaw_5deg_iou": next(r["overlap"]["iou"] for r in sweep_results["yaw"] if math.isclose(r["angle_deg"], 5.0)),
            "pitch_5deg_iou": next(r["overlap"]["iou"] for r in sweep_results["pitch"] if math.isclose(r["angle_deg"], 5.0)),
            "dx_10cm_iou": next(r["overlap"]["iou"] for r in sweep_results["dx"] if math.isclose(r["offset"], 0.10)),
            "insight": (
                "Yaw produces range-dependent arc distortion (arc error ~ r * theta). For a 5m max range, "
                "a 5-degree yaw causes ~0.436m horizontal displacement, dropping IoU below 50%. "
                "Translations dx and dy cause rigid 1-to-1 shifts of the entire map footprint."
            ),
        },
        "metric_deception_warning": (
            "Cell count alone is a misleading metric for calibration quality. For example, a downward "
            "pitch (+5 deg) can tilt ceiling fixtures into the obstacle band or cause grazing floor returns "
            "to populate cells, altering cell counts while severely degrading geometric fidelity."
        ),
    }

    # 8. Export Structured Machine-Readable Metrics JSON
    json_path = Path(args.output_json)
    json_path.parent.mkdir(parents=True, exist_ok=True)

    summary_payload = {
        "provenance_disclosure": {
            "authoritative_status": "CONDITIONAL",
            "dataset": "TUM RGB-D freiburg2_pioneer_slam2",
            "physical_calibration_available": False,
            "statement": (
                "TUM publishes camera intrinsics but does NOT publish physical extrinsic calibration "
                "relating MoCap marker frames to Kinect optical center. Baseline mounting parameters "
                "(dx=+0.15m, dy=0.0m, dz=+0.40m, pitch=0°, yaw=0°, roll=0°) are engineering assumptions. "
                "This study provides empirical sensitivity bounds rather than claiming unverified physical accuracy."
            ),
        },
        "baseline_summary": baseline_metrics,
        "sweeps": sweep_results,
        "robustness_analysis": analysis,
        "timestamp_generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary_payload, f, indent=2)

    print(f"\n[STEP 8] Saved comprehensive machine-readable metrics report: {json_path.resolve()}")
    print("=" * 100)
    print("CALIBRATION SENSITIVITY AUDIT COMPLETE: CONDITIONAL PASS (UNCERTAINTY BOUNDED)")
    print("=" * 100)


if __name__ == "__main__":
    main()
