"""Integration tests for real-world dataset ingestion and trajectory evaluation."""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import pytest

from autonomous_nav.datasets.adapter import TUMRGBDAdapter
from autonomous_nav.datasets.config import DatasetConfig
from autonomous_nav.datasets.groundtruth import GroundTruthTrajectory
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotState


GT_PATH = Path("data/tum_rgbd/groundtruth.txt")


@pytest.mark.skipif(not GT_PATH.is_file(), reason="Official TUM ground truth not locally downloaded")
def test_tum_rgbd_groundtruth_file_integration():
    """Verify loading and kinematics analysis of the downloaded official Pioneer SLAM2 trajectory."""
    traj = GroundTruthTrajectory.load_from_file(GT_PATH)

    # Sequence properties must match authoritative TUM dataset specifications:
    # 33,188 poses recorded at 100 Hz, ~115.6s duration, ~23.3m total driven path
    assert len(traj) > 30000
    assert 110.0 <= traj.duration <= 120.0

    stats = traj.compute_summary_statistics()
    assert stats["trajectory_length_m"] > 20.0
    assert 4.5 <= stats["span_x_m"] <= 5.5
    assert 4.5 <= stats["span_y_m"] <= 5.5

    # Interpolate at arbitrary mid-trajectory timestamp
    t_query = traj.start_time + 45.0
    pose = traj.get_pose_interpolated(t_query, tolerance=0.05)
    assert pose is not None
    assert math.isfinite(pose.tx)
    assert math.isfinite(pose.ty)
    assert math.isfinite(pose.yaw)


@pytest.mark.skipif(not GT_PATH.is_file(), reason="Official TUM ground truth not locally downloaded")
def test_tum_rgbd_adapter_evaluation_pipeline():
    """Verify end-to-end adapter execution on the downloaded trajectory."""
    cfg = DatasetConfig(dataset_dir=Path("data/tum_rgbd"))
    adapter = TUMRGBDAdapter(config=cfg)

    manifest = adapter.discover_and_validate()
    assert manifest["has_groundtruth"]
    assert manifest["groundtruth_file"] is not None

    traj = adapter.get_trajectory()
    assert len(traj) > 30000

    # Build an occupancy grid encompassing the trajectory
    grid = GridMap(width=100, height=100, resolution=0.1, origin=(-5.0, -5.0))

    # Evaluate trajectory tracking with zero-error synthetic run (ground truth compared against itself)
    sampled_states = [p.to_robot_state() for p in traj.poses[::50]]
    metrics = adapter.evaluate_trajectory_tracking(sampled_states, tolerance=0.01)

    assert metrics["aligned_samples"] == len(sampled_states)
    assert metrics["skipped_samples"] == 0
    # Comparing identical poses must yield zero error
    assert math.isclose(metrics["position_rmse_m"], 0.0, abs_tol=1e-5)
    assert math.isclose(metrics["heading_rmse_deg"], 0.0, abs_tol=1e-3)


MANIFEST_PATH = Path("data/tum_rgbd/manifest.json")
DEPTH_DIR = Path("data/tum_rgbd/depth")


@pytest.mark.skipif(not MANIFEST_PATH.is_file(), reason="Authentic depth frames not yet extracted")
def test_authentic_depth_frame_manifest_and_files():
    """Verify integrity, metadata, and timestamps of extracted authentic depth PNG files."""
    import hashlib
    import json
    from autonomous_nav.datasets.depth_projection import load_depth_image

    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    assert manifest["sequence"] == "freiburg2_pioneer_slam2"
    frames = manifest["frames"]
    assert len(frames) >= 3, f"Expected at least 3 authentic frames, found {len(frames)}"

    traj = GroundTruthTrajectory.load_from_file(GT_PATH)

    for frame in frames:
        fpath = Path(frame["local_path"])
        if not fpath.is_file():
            fpath = DEPTH_DIR / frame["filename"]
        assert fpath.is_file(), f"Authentic depth file missing: {fpath}"

        # 1. Byte size verification
        actual_size = fpath.stat().st_size
        assert actual_size == frame["file_size_bytes"]

        # 2. SHA-256 hash verification
        h = hashlib.sha256(fpath.read_bytes()).hexdigest()
        assert h == frame["sha256"]

        # 3. Residual naming verification
        assert "nearest_mocap_sample_residual_ms" in frame
        assert frame["nearest_mocap_sample_residual_ms"] < 2.0  # sub-2ms

        # 4. Image dimensions and dtype
        img = load_depth_image(fpath)
        assert img.shape == (480, 640)
        assert img.dtype == "uint16"

        # 5. Temporal association with ground truth pose
        ts = frame["timestamp"]
        pose = traj.get_pose_interpolated(ts, tolerance=0.05)
        assert pose is not None, f"Failed to associate timestamp {ts} within 0.05s tolerance"
        assert abs(pose.timestamp - ts) <= 0.05


@pytest.mark.skipif(not MANIFEST_PATH.is_file(), reason="Authentic depth frames not yet extracted")
def test_authentic_depth_frame_grid_mapping():
    """Verify projection of authentic depth frames into 2D GridMap."""
    import json
    from autonomous_nav.datasets.depth_projection import load_depth_image

    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    cfg = DatasetConfig(dataset_dir=Path("data/tum_rgbd"))
    adapter = TUMRGBDAdapter(config=cfg)
    traj = adapter.get_trajectory()

    grid = GridMap(width=200, height=200, resolution=0.05, origin=(-5.0, -5.0))

    frames = manifest["frames"]
    assert len(frames) >= 3

    for frame in frames:
        fpath = Path(frame["local_path"])
        if not fpath.is_file():
            fpath = DEPTH_DIR / frame["filename"]

        img = load_depth_image(fpath)
        pose = traj.get_pose_interpolated(frame["timestamp"], tolerance=0.05)
        assert pose is not None

        diag = adapter.process_depth_frame(depth_img=img, robot_pose=pose, grid_map=grid)

        # Authentic Kinect frames in testing hall have valid returns and obstacles
        assert diag["valid_depth_pixels"] > 0
        assert diag["obstacle_points"] > 0

    assert np.count_nonzero(grid.grid == 1) > 0


@pytest.mark.skipif(not MANIFEST_PATH.is_file(), reason="Authentic depth frames not yet extracted")
def test_authentic_depth_frame_pixel_reconciliation_and_spatial_bounds():
    """Verify that all authentic depth frames satisfy pixel count reconciliation and bounds clipping diagnostics."""
    import json
    from autonomous_nav.datasets.depth_projection import load_depth_image

    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    cfg = DatasetConfig(dataset_dir=Path("data/tum_rgbd"))
    adapter = TUMRGBDAdapter(config=cfg)
    traj = adapter.get_trajectory()

    grid = GridMap(width=200, height=200, resolution=0.05, origin=(-5.0, -5.0))
    frames = manifest["frames"]

    for frame in frames:
        fpath = Path(frame["local_path"])
        if not fpath.is_file():
            fpath = DEPTH_DIR / frame["filename"]

        img = load_depth_image(fpath)
        pose = traj.get_pose_interpolated(frame["timestamp"], tolerance=0.05)
        assert pose is not None

        # Verify nearest MoCap pose timestamp offset is sub-2ms
        nearest = traj.get_pose_nearest(frame["timestamp"], tolerance=0.05)
        assert nearest is not None
        nearest_dt = abs(nearest.timestamp - frame["timestamp"])
        assert nearest_dt < 0.002  # Less than 2 ms

        diag = adapter.process_depth_frame(depth_img=img, robot_pose=pose, grid_map=grid)

        # 1. Image Pixel Count Reconciliation (exact sum = 307,200)
        assert diag["total_pixels"] == 307200
        assert (
            diag["zero_depth_pixels"] + diag["out_of_range_pixels"] + diag["valid_depth_pixels"]
            == diag["total_pixels"]
        )
        assert diag["invalid_depth_pixels"] + diag["valid_depth_pixels"] == diag["total_pixels"]

        # 2. Height Filtering Reconciliation (exact sum = valid_depth_pixels)
        assert (
            diag["floor_rejected_points"] + diag["ceiling_rejected_points"] + diag["obstacle_points"]
            == diag["valid_depth_pixels"]
        )

        # 3. Spatial Bounds Diagnostics (points_in_bounds + points_clipped == obstacle_points)
        assert diag["points_in_bounds"] + diag["points_clipped"] == diag["obstacle_points"]
        assert diag["points_in_bounds"] > 0



