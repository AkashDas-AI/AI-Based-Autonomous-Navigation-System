#!/usr/bin/env python3
"""Demonstration of vision-based obstacle perception, grid mapping, and path planning."""

from __future__ import annotations

import json
import sys
from pathlib import Path

# Add src/ to path
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import matplotlib.pyplot as plt
import numpy as np
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.perception.map_adapter import MapAdapter
from autonomous_nav.perception.obstacle_detection import ObstacleDetector
from autonomous_nav.perception.preprocessing import preprocess_image
from autonomous_nav.perception.vision import OrthographicCamera, SyntheticImageRenderer
from autonomous_nav.planning.astar import AStarPlanner


def run_perception_demo(output_dir: str | Path = "outputs") -> int:
    """Execute end-to-end vision perception to grid mapping demonstration."""
    out_base = Path(output_dir)
    figures_dir = out_base / "figures"
    metrics_dir = out_base / "metrics"
    figures_dir.mkdir(parents=True, exist_ok=True)
    metrics_dir.mkdir(parents=True, exist_ok=True)

    print("\n==================================================================")
    print(" Running Vision-Based Obstacle Perception & Grid Mapping Demo")
    print("==================================================================")

    # 1. Setup Camera Model for 30m x 20m workspace
    world_w, world_h = 30.0, 20.0
    img_w, img_h = 640, 480
    camera = OrthographicCamera(
        image_width=img_w,
        image_height=img_h,
        world_x_min=0.0,
        world_y_min=0.0,
        world_x_max=world_w,
        world_y_max=world_h,
    )
    print(f"Camera Model: Orthographic ({img_w}x{img_h} px viewing [0, {world_w}]m x [0, {world_h}]m)")
    print(f" - Scale X: {camera.scale_x:.2f} px/m, Scale Y: {camera.scale_y:.2f} px/m")

    # 2. Render Deterministic Synthetic Scene with Known Geometric Obstacles
    rectangles = [
        (8.0, 5.0, 12.0, 15.0),    # Tall vertical block
        (20.0, 3.0, 24.0, 9.0),    # Lower right barrier
    ]
    circles = [
        (16.0, 11.0, 2.5),         # Center circular obstacle
    ]

    renderer = SyntheticImageRenderer(camera)
    synthetic_raw = renderer.render_obstacles(
        rectangles=rectangles,
        circles=circles,
        background_color=240,
        obstacle_color=30,
    )
    print(f"Rendered synthetic scene with {len(rectangles)} rectangles and {len(circles)} circle.")

    # 3. Image Preprocessing (Color conversion, Gaussian blur)
    prep_result = preprocess_image(
        synthetic_raw,
        color_format="GRAY",
        blur_kernel_size=5,
        blur_sigma=1.2,
    )
    print("Completed image preprocessing (validation, Gaussian smoothing).")

    # 4. Obstacle Detection (Otsu thresholding, morphology, contour analysis)
    detector = ObstacleDetector(
        threshold_method="otsu",
        invert_threshold=True,
        morphology_kernel_size=3,
        min_area_pixels=50.0,
    )
    det_result = detector.detect(prep_result.blurred)
    num_detected = len(det_result.obstacles)
    print(f"Detected {num_detected} obstacle contours in synthetic observation.")

    # 5. Grid Map Update via MapAdapter
    resolution = 0.5  # meters per cell
    grid_width = int(world_w / resolution)   # 60 cells
    grid_height = int(world_h / resolution)  # 40 cells
    perceived_grid = GridMap(width=grid_width, height=grid_height, resolution=resolution)

    adapter = MapAdapter(camera)
    cells_marked = adapter.update_from_detections(det_result.obstacles, perceived_grid)
    print(f"Updated GridMap ({grid_width}x{grid_height}): Marked {cells_marked} occupied cells from vision.")

    # 6. Plan Route on Perceived Map
    start_world = (2.0, 2.0)
    goal_world = (27.0, 17.0)
    inflated_map = perceived_grid.inflate_obstacles(radius_meters=0.8)

    planner = AStarPlanner(allow_diagonal=True)
    planned_path = planner.plan_world(start_world, goal_world, inflated_map)
    path_found = planned_path is not None and len(planned_path) > 0
    path_len = 0.0
    if path_found:
        for i in range(1, len(planned_path)):
            path_len += float(np.hypot(
                planned_path[i][0] - planned_path[i - 1][0],
                planned_path[i][1] - planned_path[i - 1][1],
            ))
    print(f"A* Path Planning on perceived map: Found = {path_found}, Path Length = {path_len:.2f} m")

    # 7. Generate 4-Panel Visualization
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=120)

    # Panel 1: Raw Synthetic Image
    axes[0, 0].imshow(synthetic_raw, cmap="gray", vmin=0, vmax=255)
    axes[0, 0].set_title("1. Synthetic Camera Observation", fontweight="bold")
    axes[0, 0].set_xlabel("Pixel u")
    axes[0, 0].set_ylabel("Pixel v")

    # Panel 2: Binary Obstacle Mask
    axes[0, 1].imshow(det_result.binary_mask, cmap="binary", vmin=0, vmax=255)
    axes[0, 1].set_title("2. Cleaned Binary Mask (Otsu + Morphology)", fontweight="bold")
    axes[0, 1].set_xlabel("Pixel u")
    axes[0, 1].set_ylabel("Pixel v")

    # Panel 3: Annotated Detections
    # Convert BGR to RGB for matplotlib
    annotated_rgb = det_result.annotated_image[:, :, ::-1] if det_result.annotated_image.ndim == 3 else det_result.annotated_image
    axes[1, 0].imshow(annotated_rgb)
    axes[1, 0].set_title(f"3. Detected Contours & Centroids ({num_detected} objects)", fontweight="bold")
    axes[1, 0].set_xlabel("Pixel u")
    axes[1, 0].set_ylabel("Pixel v")

    # Panel 4: Perceived Occupancy Grid Map & Planned Path
    x_min, x_max = 0.0, world_w
    y_min, y_max = 0.0, world_h
    axes[1, 1].imshow(
        perceived_grid.grid,
        cmap="binary",
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        interpolation="nearest",
        alpha=0.75,
    )
    if path_found and planned_path:
        px = [p[0] for p in planned_path]
        py = [p[1] for p in planned_path]
        axes[1, 1].plot(px, py, "b--", linewidth=2.5, label="Planned A* Path")

    axes[1, 1].plot(start_world[0], start_world[1], "go", markersize=10, label="Start")
    axes[1, 1].plot(goal_world[0], goal_world[1], "r*", markersize=14, label="Goal")
    axes[1, 1].set_title("4. Resulting Occupancy Grid & Planned Route", fontweight="bold")
    axes[1, 1].set_xlabel("World X [meters]")
    axes[1, 1].set_ylabel("World Y [meters]")
    axes[1, 1].set_xlim(x_min, x_max)
    axes[1, 1].set_ylim(y_min, y_max)
    axes[1, 1].legend(loc="upper left")
    axes[1, 1].grid(True, linestyle=":", alpha=0.5)

    plt.tight_layout()
    fig_path = figures_dir / "perception_demo.png"
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)
    print(f"Saved 4-panel demonstration figure to: {fig_path}")

    # 8. Save Metrics Summary JSON
    detection_summary = []
    for idx, obs in enumerate(det_result.obstacles):
        u_c, v_c = obs.centroid_pixel
        wx_c, wy_c = camera.pixel_to_world(u_c, v_c)
        detection_summary.append({
            "id": idx + 1,
            "centroid_pixel": [round(u_c, 1), round(v_c, 1)],
            "estimated_world_position_m": [round(wx_c, 2), round(wy_c, 2)],
            "area_pixels": round(obs.area_pixels, 1),
            "bounding_box_uvwh": list(obs.bbox),
        })

    summary_data = {
        "camera": {
            "type": "orthographic",
            "image_dimensions": [img_w, img_h],
            "world_bounds_m": [0.0, 0.0, world_w, world_h],
            "scale_x_px_per_m": round(camera.scale_x, 2),
            "scale_y_px_per_m": round(camera.scale_y, 2),
        },
        "detections_count": num_detected,
        "detections": detection_summary,
        "grid_map": {
            "dimensions_cells": [grid_width, grid_height],
            "resolution_m": resolution,
            "marked_occupied_cells": cells_marked,
        },
        "path_planning": {
            "path_found": path_found,
            "start_world": list(start_world),
            "goal_world": list(goal_world),
            "planned_path_length_m": round(path_len, 2) if path_found else None,
        },
    }

    metrics_path = metrics_dir / "perception_demo_summary.json"
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(summary_data, f, indent=2)
    print(f"Saved perception demo metrics to: {metrics_path}")
    print("==================================================================\n")

    return 0 if path_found and num_detected == (len(rectangles) + len(circles)) else 1


if __name__ == "__main__":
    sys.exit(run_perception_demo())

