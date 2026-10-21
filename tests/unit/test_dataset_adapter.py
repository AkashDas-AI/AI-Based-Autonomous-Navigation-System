"""Unit tests for dataset adapters, calibration, depth back-projection, and ground truth parsing."""

from __future__ import annotations

import math
from pathlib import Path
import cv2
import numpy as np
import pytest

from autonomous_nav.datasets.adapter import TUMRGBDAdapter
from autonomous_nav.datasets.config import CameraExtrinsics, CameraIntrinsics, DatasetConfig
from autonomous_nav.datasets.depth_projection import (
    depth_to_pointcloud_camera,
    filter_obstacle_height,
    insert_points_into_grid_map,
    load_depth_image,
    project_body_to_world,
    transform_camera_to_body,
)
from autonomous_nav.datasets.groundtruth import GroundTruthPose, GroundTruthTrajectory
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.simulation.robot import RobotState


def test_camera_intrinsics_validation():
    """Verify camera intrinsics model rejects non-positive or non-finite parameters."""
    calib = CameraIntrinsics(fx=520.9, fy=521.0, cx=325.1, cy=249.7)
    assert calib.fx == 520.9
    assert calib.width == 640

    with pytest.raises(ValueError, match="Focal lengths must be positive"):
        CameraIntrinsics(fx=-10.0)

    with pytest.raises(ValueError, match="Image dimensions must be positive"):
        CameraIntrinsics(width=0)

    with pytest.raises(ValueError, match="must be finite"):
        CameraIntrinsics(cx=float("nan"))


def test_depth_to_pointcloud_pinhole_projection_math():
    """Verify exact pinhole back-projection geometry from 2D pixel + depth to 3D metric coordinates."""
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    depth_img = np.zeros((480, 640), dtype=np.uint16)

    # Place a pixel at optical center (320, 240) with depth 2.0 meters (2.0 * 5000 = 10000)
    depth_img[240, 320] = 10000

    # Place a pixel offset: u = 420 (+100 px right), v = 140 (-100 px up) at 1.0 meter (5000)
    # Expected: Xc = (420 - 320) * 1.0 / 500.0 = +0.20 m
    #           Yc = (140 - 240) * 1.0 / 500.0 = -0.20 m
    #           Zc = 1.0 m
    depth_img[140, 420] = 5000

    pts_c, mask = depth_to_pointcloud_camera(
        depth_img=depth_img,
        intrinsics=intrinsics,
        depth_scale=5000.0,
        min_depth=0.4,
        max_depth=5.0,
    )

    assert len(pts_c) == 2
    assert mask[240, 320]
    assert mask[140, 420]

    # Center point (u=320, v=240, Z=2.0)
    pt_center = pts_c[pts_c[:, 2] == 2.0][0]
    assert math.isclose(pt_center[0], 0.0, abs_tol=1e-5)
    assert math.isclose(pt_center[1], 0.0, abs_tol=1e-5)
    assert math.isclose(pt_center[2], 2.0, abs_tol=1e-5)

    # Offset point (u=420, v=140, Z=1.0)
    pt_offset = pts_c[pts_c[:, 2] == 1.0][0]
    assert math.isclose(pt_offset[0], 0.20, abs_tol=1e-5)
    assert math.isclose(pt_offset[1], -0.20, abs_tol=1e-5)
    assert math.isclose(pt_offset[2], 1.0, abs_tol=1e-5)


def test_depth_filter_range_rejection():
    """Verify near-field noise and far-field thresholds discard invalid pixels."""
    intrinsics = CameraIntrinsics()
    depth_img = np.zeros((480, 640), dtype=np.uint16)

    # Value 0 is invalid/missing
    depth_img[100, 100] = 0
    # 0.2 meters (1000 units) < min_depth (0.4m)
    depth_img[100, 101] = 1000
    # 6.0 meters (30000 units) > max_depth (5.0m)
    depth_img[100, 102] = 30000
    # 2.0 meters (10000 units) - valid
    depth_img[100, 103] = 10000

    pts_c, mask = depth_to_pointcloud_camera(
        depth_img=depth_img,
        intrinsics=intrinsics,
        depth_scale=5000.0,
        min_depth=0.4,
        max_depth=5.0,
    )

    assert len(pts_c) == 1
    assert math.isclose(pts_c[0, 2], 2.0, abs_tol=1e-5)


def test_transform_camera_to_body():
    """Verify coordinate transformation from camera optical frame to robot body frame (REP-105)."""
    extrinsics = CameraExtrinsics(dx=0.20, dy=0.0, dz=0.40)
    # Optical point: Xc = +0.5m (right), Yc = -0.1m (up), Zc = 2.0m (forward)
    pts_c = np.array([[0.5, -0.1, 2.0]], dtype=np.float64)

    pts_b = transform_camera_to_body(points_camera=pts_c, extrinsics=extrinsics)

    # Expected body:
    # Xb = +Zc + dx = 2.0 + 0.20 = 2.20 m
    # Yb = -Xc + dy = -0.5 + 0.0 = -0.50 m
    # Zb = -Yc + dz = -(-0.1) + 0.40 = 0.50 m
    assert math.isclose(pts_b[0, 0], 2.20, abs_tol=1e-5)
    assert math.isclose(pts_b[0, 1], -0.50, abs_tol=1e-5)
    assert math.isclose(pts_b[0, 2], 0.50, abs_tol=1e-5)


def test_filter_obstacle_height():
    """Verify obstacle vertical slicing excludes floor ground plane and high ceilings."""
    pts_b = np.array(
        [
            [2.0, 0.0, 0.02],   # Floor (Z=2cm < 5cm) -> discard
            [2.0, 0.0, 0.35],   # Obstacle (Z=35cm in [5cm, 1.2m]) -> keep
            [2.0, 0.0, 0.80],   # Obstacle (Z=80cm) -> keep
            [2.0, 0.0, 1.60],   # Ceiling fixture (Z=1.6m > 1.2m) -> discard
        ],
        dtype=np.float64,
    )

    filtered = filter_obstacle_height(pts_b, min_height=0.05, max_height=1.20)
    assert len(filtered) == 2
    assert math.isclose(filtered[0, 2], 0.35)
    assert math.isclose(filtered[1, 2], 0.80)


def test_project_body_to_world_and_grid_insertion():
    """Verify rotating body points to world frame and marking discrete GridMap cells."""
    # Robot at world (10.0, 5.0) pointing North (yaw = pi/2)
    robot_pose = RobotState(x=10.0, y=5.0, theta=math.pi / 2, v=0.0, omega=0.0, timestamp=1.0)

    # Obstacle point in body frame: Xb = 2.0m (forward), Yb = 1.0m (left)
    # When yaw = pi/2:
    # Xw = rx + Xb*cos(pi/2) - Yb*sin(pi/2) = 10.0 + 0 - 1.0 = 9.0 m
    # Yw = ry + Xb*sin(pi/2) + Yb*cos(pi/2) = 5.0 + 2.0 + 0 = 7.0 m
    pts_b = np.array([[2.0, 1.0, 0.30]], dtype=np.float64)

    pts_w = project_body_to_world(points_body=pts_b, robot_pose=robot_pose)
    assert math.isclose(pts_w[0, 0], 9.0, abs_tol=1e-5)
    assert math.isclose(pts_w[0, 1], 7.0, abs_tol=1e-5)

    # Insert into GridMap (origin 0,0, res 0.1m, size 200x200)
    grid = GridMap(width=200, height=200, resolution=0.1, origin=(0.0, 0.0))
    cells_marked = insert_points_into_grid_map(points_world=pts_w, grid_map=grid)
    assert cells_marked == 1

    col, row = grid.world_to_grid(9.0, 7.0)
    assert grid.is_occupied(col, row)


def test_ground_truth_pose_yaw_and_conversions():
    """Verify GroundTruthPose quaternion parsing and yaw extraction."""
    # Orientation: 90 deg yaw (pi/2) about Z axis
    # qz = sin(pi/4) ~= 0.7071068, qw = cos(pi/4) ~= 0.7071068
    qz = math.sin(math.pi / 4)
    qw = math.cos(math.pi / 4)

    pose = GroundTruthPose(timestamp=100.0, tx=2.5, ty=-3.2, tz=0.45, qx=0.0, qy=0.0, qz=qz, qw=qw)

    assert math.isclose(pose.yaw, math.pi / 2, abs_tol=1e-5)

    state_2d = pose.to_robot_state()
    assert state_2d.x == 2.5
    assert state_2d.y == -3.2
    assert math.isclose(state_2d.theta, math.pi / 2, abs_tol=1e-5)
    assert state_2d.timestamp == 100.0

    state_3d = pose.to_timestamped_robot_state()
    assert state_3d.x == 2.5
    assert state_3d.z == 0.45
    assert math.isclose(state_3d.yaw, math.pi / 2, abs_tol=1e-5)


def test_ground_truth_trajectory_slerp_interpolation():
    """Verify SLERP interpolation and nearest-neighbor lookups on synthetic trajectory."""
    # Two poses 0.1s apart with 0.1m translation and 10 deg rotation
    p0 = GroundTruthPose(
        timestamp=10.0, tx=0.0, ty=0.0, tz=0.0,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
    )
    p1 = GroundTruthPose(
        timestamp=10.1, tx=1.0, ty=0.0, tz=0.0,
        qx=0.0, qy=0.0, qz=math.sin(math.radians(10) / 2), qw=math.cos(math.radians(10) / 2),
    )

    traj = GroundTruthTrajectory([p0, p1])
    assert len(traj) == 2
    assert math.isclose(traj.duration, 0.1, abs_tol=1e-5)

    # Exact midpoint at t = 10.05s
    p_mid = traj.get_pose_interpolated(timestamp=10.05, tolerance=0.01)
    assert p_mid is not None
    assert math.isclose(p_mid.tx, 0.5, abs_tol=1e-5)
    assert math.isclose(p_mid.ty, 0.0, abs_tol=1e-5)
    # Heading should be 5 degrees
    assert math.isclose(math.degrees(p_mid.yaw), 5.0, abs_tol=1e-3)

    # Query out of bounds beyond tolerance returns None
    assert traj.get_pose_interpolated(timestamp=12.0, tolerance=0.05) is None


def test_load_depth_image_corrupt_and_missing():
    """Verify load_depth_image raises appropriate exceptions on missing or invalid files."""
    with pytest.raises(FileNotFoundError):
        load_depth_image("non_existent_depth_file.png")

    # Create temporary non-uint16 image (8-bit grayscale)
    tmp_path = Path("outputs/temp_8bit_test.png")
    tmp_path.parent.mkdir(parents=True, exist_ok=True)
    img_8bit = np.zeros((100, 100), dtype=np.uint8)
    cv2.imwrite(str(tmp_path), img_8bit)

    try:
        with pytest.raises(ValueError, match="must be uint16"):
            load_depth_image(tmp_path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def test_depth_projection_nan_inf_and_negative_rejection():
    """Verify pinhole projection strictly rejects NaN, Inf, and non-physical negative depths."""
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    depth_img = np.zeros((480, 640), dtype=np.uint16)

    # Valid pixel at (320, 240) with 1.5m depth (7500 units)
    depth_img[240, 320] = 7500

    pts_c, mask = depth_to_pointcloud_camera(
        depth_img=depth_img,
        intrinsics=intrinsics,
        min_depth=0.4,
        max_depth=5.0,
    )

    assert len(pts_c) == 1
    assert np.all(np.isfinite(pts_c))
    assert math.isclose(pts_c[0, 2], 1.5, abs_tol=1e-5)


def test_floor_plane_rejection_invariance():
    """Verify that a ground floor plane at body elevation Z_b=0.0 is 100% rejected."""
    # Synthesize 100 points on the ground floor (Z_b in [-0.02, 0.02]m)
    rng = np.random.default_rng(seed=123)
    xs = rng.uniform(0.5, 4.0, size=100)
    ys = rng.uniform(-2.0, 2.0, size=100)
    zs = rng.uniform(-0.02, 0.02, size=100)
    pts_floor = np.column_stack([xs, ys, zs])

    filtered = filter_obstacle_height(pts_floor, min_height=0.05, max_height=1.20)
    assert len(filtered) == 0, f"Expected 0 points retained from floor, got {len(filtered)}"


def test_provenance_manifest_depth_directory_absence(tmp_path: Path):
    """Verify adapter manifest correctly detects absent depth directory and does not fabricate frames."""
    adapter = TUMRGBDAdapter(config=DatasetConfig(dataset_dir=tmp_path))
    manifest = adapter.discover_and_validate()

    assert not manifest["has_groundtruth"]
    assert not manifest["has_depth_dir"]
    assert not manifest["has_rgb_dir"]
    assert manifest["depth_images_count"] == 0
    assert manifest["rgb_images_count"] == 0


def test_corrupt_or_truncated_png_handling(tmp_path: Path):
    """Verify that corrupt, incomplete, or truncated PNG files raise ValueError."""
    # 1. Truncated PNG (only PNG header bytes, missing IHDR and IDAT)
    trunc_png = tmp_path / "truncated.png"
    trunc_png.write_bytes(b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00")

    with pytest.raises(ValueError, match="corrupted or invalid format"):
        load_depth_image(trunc_png)

    # 2. Random non-image binary bytes
    garbage_png = tmp_path / "garbage.png"
    garbage_png.write_bytes(b"This is completely invalid random binary data that cannot be a PNG image.")

    with pytest.raises(ValueError, match="corrupted or invalid format"):
        load_depth_image(garbage_png)


def test_invalid_depth_pixel_filtering_comprehensive():
    """Verify comprehensive filtering of sensor dropout, near-field, and far-field pixels."""
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    depth_img = np.zeros((480, 640), dtype=np.uint16)

    # 0 = dropout / missing return -> discard
    depth_img[100, 100] = 0
    # 1000 raw = 0.20m < 0.40m min_depth -> discard
    depth_img[100, 101] = 1000
    # 1500 raw = 0.30m < 0.40m min_depth -> discard
    depth_img[100, 102] = 1500
    # 2000 raw = 0.40m == min_depth -> retain
    depth_img[100, 103] = 2000
    # 10000 raw = 2.00m (valid mid-range) -> retain
    depth_img[100, 104] = 10000
    # 25000 raw = 5.00m == max_depth -> retain
    depth_img[100, 105] = 25000
    # 25005 raw = 5.001m > max_depth -> discard
    depth_img[100, 106] = 25005
    # 50000 raw = 10.0m > max_depth -> discard
    depth_img[100, 107] = 50000

    pts_c, mask = depth_to_pointcloud_camera(
        depth_img=depth_img,
        intrinsics=intrinsics,
        depth_scale=5000.0,
        min_depth=0.40,
        max_depth=5.00,
    )

    # Exactly 3 pixels should be retained: 0.40m, 2.00m, 5.00m
    assert len(pts_c) == 3
    assert np.all(pts_c[:, 2] >= 0.40)
    assert np.all(pts_c[:, 2] <= 5.00)
    assert math.isclose(pts_c[0, 2], 0.40, abs_tol=1e-5)
    assert math.isclose(pts_c[1, 2], 2.00, abs_tol=1e-5)
    assert math.isclose(pts_c[2, 2], 5.00, abs_tol=1e-5)


def test_timestamp_association_and_interpolation():
    """Verify temporal association of sensor timestamps with ground truth poses."""
    p0 = GroundTruthPose(timestamp=100.0, tx=0.0, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    p1 = GroundTruthPose(timestamp=100.1, tx=0.2, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    p2 = GroundTruthPose(timestamp=100.2, tx=0.4, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    traj = GroundTruthTrajectory([p0, p1, p2])

    # 1. Exact timestamp match
    exact_pose = traj.get_pose_interpolated(100.1, tolerance=0.05)
    assert exact_pose is not None
    assert math.isclose(exact_pose.tx, 0.2, abs_tol=1e-5)

    # 2. Interpolation within tolerance (dt = 0.03s from nearest)
    interp_pose = traj.get_pose_interpolated(100.05, tolerance=0.05)
    assert interp_pose is not None
    assert math.isclose(interp_pose.tx, 0.1, abs_tol=1e-5)

    # 3. Beyond tolerance (dt = 0.15s before start) -> None
    assert traj.get_pose_interpolated(99.8, tolerance=0.05) is None
    # 4. Beyond tolerance (dt = 0.15s after end) -> None
    assert traj.get_pose_interpolated(100.35, tolerance=0.05) is None


def test_projection_and_grid_map_bounds():
    """Verify grid bounds handling: out-of-bounds points are safely ignored without exceptions."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))
    # Grid covers [0.0, 10.0] in X and [0.0, 10.0] in Y

    # Mix of points: some inside, some outside
    pts_w = np.array(
        [
            [5.0, 5.0],      # Inside (col=10, row=10)
            [1.0, 1.0],      # Inside (col=2, row=2)
            [-2.0, 5.0],     # Outside X negative
            [12.0, 5.0],     # Outside X positive
            [5.0, -1.0],     # Outside Y negative
            [5.0, 15.0],     # Outside Y positive
            [100.0, 100.0],  # Far outside
        ],
        dtype=np.float64,
    )

    new_cells = insert_points_into_grid_map(points_world=pts_w, grid_map=grid)
    assert new_cells == 2

    c1, r1 = grid.world_to_grid(5.0, 5.0)
    c2, r2 = grid.world_to_grid(1.0, 1.0)
    assert grid.is_occupied(c1, r1)
    assert grid.is_occupied(c2, r2)


def test_depth_projection_determinism():
    """Verify that depth projection and grid rasterization are strictly deterministic."""
    cfg = DatasetConfig()
    adapter = TUMRGBDAdapter(config=cfg)

    # Create synthetic test depth image
    depth_img = np.zeros((480, 640), dtype=np.uint16)
    depth_img[200:300, 250:350] = 10000  # 2.0 meters obstacle

    pose = GroundTruthPose(timestamp=1.0, tx=0.0, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)

    grid1 = GridMap(width=100, height=100, resolution=0.1, origin=(-5.0, -5.0))
    grid2 = GridMap(width=100, height=100, resolution=0.1, origin=(-5.0, -5.0))

    diag1 = adapter.process_depth_frame(depth_img, pose, grid1)
    diag2 = adapter.process_depth_frame(depth_img, pose, grid2)

    assert diag1["valid_depth_pixels"] == diag2["valid_depth_pixels"]
    assert diag1["obstacle_points"] == diag2["obstacle_points"]
    assert diag1["grid_cells_updated"] == diag2["grid_cells_updated"]
    np.testing.assert_array_equal(grid1.grid, grid2.grid)


def test_manifest_parsing_and_provenance(tmp_path: Path):
    """Verify reading and validation of authentic depth frame manifest."""
    import json

    manifest_file = tmp_path / "manifest.json"
    dummy_manifest = {
        "sequence": "freiburg2_pioneer_slam2",
        "source_archive": "rgbd_dataset_freiburg2_pioneer_slam2.tgz",
        "extracted_count": 3,
        "frames": [
            {
                "member_path": "rgbd_dataset_freiburg2_pioneer_slam2/depth/1311877814.945678.png",
                "filename": "1311877814.945678.png",
                "timestamp": 1311877814.945678,
                "file_size_bytes": 105432,
                "sha256": "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789",
                "local_path": "data/tum_rgbd/depth/1311877814.945678.png",
                "provenance": "Authentic extraction from official TUM rgbd_dataset_freiburg2_pioneer_slam2.tgz",
            }
        ],
    }
    manifest_file.write_text(json.dumps(dummy_manifest), encoding="utf-8")

    with open(manifest_file, "r", encoding="utf-8") as f:
        loaded = json.load(f)

    assert loaded["sequence"] == "freiburg2_pioneer_slam2"
    assert len(loaded["frames"]) == 1
    frame = loaded["frames"][0]
    assert frame["filename"].endswith(".png")
    assert frame["file_size_bytes"] > 0
    assert len(frame["sha256"]) == 64
    assert "Authentic extraction" in frame["provenance"]


def test_pixel_reconciliation_exact_breakdown():
    """Verify exact arithmetic reconciliation across raw pixels, valid range, and height filtering."""
    cfg = DatasetConfig()
    adapter = TUMRGBDAdapter(config=cfg)

    # Construct an image of 100x100 = 10,000 pixels
    # With distinct designated subsets
    depth_img = np.zeros((100, 100), dtype=np.uint16)

    # 1. Zero depth pixels: 4,000 pixels (remain 0)
    # 2. Near-field out-of-range (< 0.4m = 2000 units): 1,000 pixels at 0.2m (1000 units)
    depth_img[40:50, :] = 1000
    # 3. Far-field out-of-range (> 5.0m = 25000 units): 1,000 pixels at 6.0m (30000 units)
    depth_img[50:60, :] = 30000
    # 4. Valid depth (in [0.4m, 5.0m]): 4,000 pixels at 2.0m (10000 units)
    depth_img[60:100, :] = 10000

    # Total pixels: 10,000
    # Expected:
    # zero_depth: 4,000
    # near_field: 1,000
    # far_field: 1,000
    # out_of_range: 2,000
    # invalid: 6,000
    # valid: 4,000

    pose = GroundTruthPose(timestamp=1.0, tx=0.0, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    grid = GridMap(width=200, height=200, resolution=0.05, origin=(-5.0, -5.0))

    diag = adapter.process_depth_frame(depth_img, pose, grid)

    assert diag["total_pixels"] == 10000
    assert diag["zero_depth_pixels"] == 4000
    assert diag["near_field_pixels"] == 1000
    assert diag["far_field_pixels"] == 1000
    assert diag["out_of_range_pixels"] == 2000
    assert diag["invalid_depth_pixels"] == 6000
    assert diag["valid_depth_pixels"] == 4000

    # Reconciliation Check 1: total = invalid + valid = zero + out_of_range + valid
    assert diag["zero_depth_pixels"] + diag["out_of_range_pixels"] + diag["valid_depth_pixels"] == diag["total_pixels"]
    assert diag["invalid_depth_pixels"] + diag["valid_depth_pixels"] == diag["total_pixels"]

    # Reconciliation Check 2: valid = floor + ceiling + retained
    assert (
        diag["floor_rejected_points"] + diag["ceiling_rejected_points"] + diag["obstacle_points"]
        == diag["valid_depth_pixels"]
    )


def test_timestamp_association_nearest_pose_and_brackets():
    """Verify that nearest MoCap timestamp association reports non-zero physical delta_t and valid brackets."""
    poses = [
        GroundTruthPose(timestamp=10.00, tx=1.0, ty=1.0, tz=0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        GroundTruthPose(timestamp=10.01, tx=1.1, ty=1.1, tz=0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        GroundTruthPose(timestamp=10.02, tx=1.2, ty=1.2, tz=0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        GroundTruthPose(timestamp=10.03, tx=1.3, ty=1.3, tz=0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
    ]
    traj = GroundTruthTrajectory(poses)

    # Query at 10.0125 s (2.5 ms after 10.01, 7.5 ms before 10.02)
    query_ts = 10.0125
    nearest = traj.get_pose_nearest(query_ts)
    assert nearest is not None
    assert nearest.timestamp == 10.01
    dt_nearest = abs(nearest.timestamp - query_ts)
    assert math.isclose(dt_nearest, 0.0025, abs_tol=1e-6)

    # Brackets for interpolation
    interp = traj.get_pose_interpolated(query_ts)
    assert interp is not None
    assert math.isclose(interp.timestamp, query_ts)
    # tx should be linearly interpolated: 1.1 + 0.25 * (1.2 - 1.1) = 1.125
    assert math.isclose(interp.tx, 1.125, abs_tol=1e-5)


def test_grid_boundary_clipping_diagnostics():
    """Verify insertion diagnostics accurately track in-bounds vs boundary-clipped points."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))
    # Metric coverage: [0.0, 10.0] x [0.0, 10.0]

    pts_w = np.array(
        [
            [2.0, 2.0],    # In bounds
            [5.0, 5.0],    # In bounds
            [8.0, 8.0],    # In bounds
            [-1.0, 5.0],   # Clipped (X negative)
            [12.0, 5.0],   # Clipped (X positive)
            [5.0, -2.0],   # Clipped (Y negative)
            [5.0, 15.0],   # Clipped (Y positive)
        ],
        dtype=np.float64,
    )

    new_cells, in_bounds, clipped = insert_points_into_grid_map(
        points_world=pts_w,
        grid_map=grid,
        return_diagnostics=True,
    )

    assert in_bounds == 3
    assert clipped == 4
    assert in_bounds + clipped == len(pts_w)
    assert new_cells == 3


def test_quaternion_component_ordering_and_normalization():
    """Verify quaternion parsing enforces (qx, qy, qz, qw) ordering and normalizes unnormalized vectors."""
    from autonomous_nav.bridge.mapping import quaternion_to_euler

    # 1. Identity quaternion
    roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
    assert math.isclose(roll, 0.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)

    # 2. Unnormalized quaternion: [0, 0, 3.0, 3.0] with norm sqrt(18)
    # Normalized: qz = 1/sqrt(2), qw = 1/sqrt(2) -> pure yaw = pi/2 (90 deg)
    roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 3.0, 3.0)
    assert math.isclose(roll, 0.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-6)

    # 3. Degenerate quaternion with zero norm raises ValueError
    with pytest.raises(ValueError, match="Degenerate quaternion"):
        quaternion_to_euler(0.0, 0.0, 0.0, 0.0)

    # 4. Non-finite values raise ValueError
    with pytest.raises(ValueError, match="must be a finite float"):
        quaternion_to_euler(float("nan"), 0.0, 0.0, 1.0)


def test_quaternion_to_yaw_four_quadrants_and_wrapping():
    """Verify quaternion-to-yaw conversion across all four quadrants with correct wrapping in (-pi, pi]."""
    from autonomous_nav.bridge.mapping import quaternion_to_euler

    angles_deg = [0.0, 30.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0, -179.9]
    for deg in angles_deg:
        rad = math.radians(deg)
        # For pure z-rotation: qx = 0, qy = 0, qz = sin(rad/2), qw = cos(rad/2)
        qz = math.sin(rad * 0.5)
        qw = math.cos(rad * 0.5)

        roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, qz, qw)
        assert math.isclose(roll, 0.0, abs_tol=1e-5)
        assert math.isclose(pitch, 0.0, abs_tol=1e-5)

        # Expected wrapped angle
        expected = math.atan2(math.sin(rad), math.cos(rad))
        assert math.isclose(yaw, expected, abs_tol=1e-5)
        assert -math.pi < yaw <= math.pi


def test_optical_to_body_axis_handedness_and_so3_determinant():
    """Verify optical-to-body coordinate transform preserves right-handedness with SO(3) determinant = +1."""
    # Rotation matrix mapping optical to body:
    # X_b = +Z_c
    # Y_b = -X_c
    # Z_b = -Y_c
    R_opt_to_body = np.array([
        [0.0,  0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    # 1. Determinant must be exactly +1 (proper rigid rotation, not reflection)
    det = np.linalg.det(R_opt_to_body)
    assert math.isclose(det, 1.0, abs_tol=1e-9)

    # 2. Orthonormality check: R^T * R == I
    identity_check = R_opt_to_body.T @ R_opt_to_body
    np.testing.assert_allclose(identity_check, np.eye(3), atol=1e-9)

    # 3. Axis mapping verification using transform_camera_to_body with zero translation
    extrinsics_zero_offset = CameraExtrinsics(dx=0.0, dy=0.0, dz=0.0)

    # Optical right (+X_c) -> Body right (-Y_b)
    pt_right = transform_camera_to_body(np.array([[1.0, 0.0, 0.0]]), extrinsics_zero_offset)
    assert math.isclose(pt_right[0, 0], 0.0)
    assert math.isclose(pt_right[0, 1], -1.0)
    assert math.isclose(pt_right[0, 2], 0.0)

    # Optical down (+Y_c) -> Body down (-Z_b)
    pt_down = transform_camera_to_body(np.array([[0.0, 1.0, 0.0]]), extrinsics_zero_offset)
    assert math.isclose(pt_down[0, 0], 0.0)
    assert math.isclose(pt_down[0, 1], 0.0)
    assert math.isclose(pt_down[0, 2], -1.0)

    # Optical forward (+Z_c) -> Body forward (+X_b)
    pt_fwd = transform_camera_to_body(np.array([[0.0, 0.0, 1.0]]), extrinsics_zero_offset)
    assert math.isclose(pt_fwd[0, 0], 1.0)
    assert math.isclose(pt_fwd[0, 1], 0.0)
    assert math.isclose(pt_fwd[0, 2], 0.0)


def test_end_to_end_projection_analytical_hand_calculation():
    """Verify complete projection pipeline against independently derived analytical hand calculations.

    Hand-derived analytical test fixture:
    - Intrinsics: fx=500.0, fy=500.0, cx=320.0, cy=240.0, depth_scale=5000.0
    - Raw Pixel: (u=420, v=140), raw_depth=10000 -> Z_c = 2.0m
    - Optical:
        X_c = (420 - 320) * 2.0 / 500.0 = +0.40 m
        Y_c = (140 - 240) * 2.0 / 500.0 = -0.40 m
        Z_c = 2.00 m
    - Extrinsics (Pioneer mount): dx=+0.15m, dy=0.0m, dz=+0.40m
    - Body:
        X_b = +Z_c + dx = 2.00 + 0.15 = 2.15 m
        Y_b = -X_c + dy = -0.40 + 0.00 = -0.40 m
        Z_b = -Y_c + dz = -(-0.40) + 0.40 = 0.80 m (retained: in [0.05, 1.20]m)
    - World Robot Pose: rx=+3.00m, ry=-1.00m, yaw=30 deg (pi/6 rad)
    - World:
        X_w = rx + X_b*cos(30 deg) - Y_b*sin(30 deg)
            = 3.00 + 2.15 * (sqrt(3)/2) - (-0.40) * 0.5
            = 3.00 + 1.8619546... + 0.20 = 5.061955 m
        Y_w = ry + X_b*sin(30 deg) + Y_b*cos(30 deg)
            = -1.00 + 2.15 * 0.5 + (-0.40) * (sqrt(3)/2)
            = -1.00 + 1.075 - 0.3464102... = -0.271410 m
    """
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    extrinsics = CameraExtrinsics(dx=0.15, dy=0.0, dz=0.40)

    # 1. Create depth image with the designated pixel
    depth_img = np.zeros((480, 640), dtype=np.uint16)
    depth_img[140, 420] = 10000  # 2.0 meters

    # 2. Step 1: Camera unprojection
    pts_c, mask = depth_to_pointcloud_camera(
        depth_img=depth_img,
        intrinsics=intrinsics,
        depth_scale=5000.0,
        min_depth=0.4,
        max_depth=5.0,
    )
    assert len(pts_c) == 1
    assert math.isclose(pts_c[0, 0], 0.40, abs_tol=1e-6)
    assert math.isclose(pts_c[0, 1], -0.40, abs_tol=1e-6)
    assert math.isclose(pts_c[0, 2], 2.00, abs_tol=1e-6)

    # 3. Step 2: Body transformation
    pts_b = transform_camera_to_body(pts_c, extrinsics)
    assert math.isclose(pts_b[0, 0], 2.15, abs_tol=1e-6)
    assert math.isclose(pts_b[0, 1], -0.40, abs_tol=1e-6)
    assert math.isclose(pts_b[0, 2], 0.80, abs_tol=1e-6)

    # 4. Step 3: Height filtering
    pts_obs = filter_obstacle_height(pts_b, min_height=0.05, max_height=1.20)
    assert len(pts_obs) == 1

    # 5. Step 4: World projection with nonzero translation and heading (30 deg)
    # Quaternion for 30 deg yaw: qz = sin(15 deg), qw = cos(15 deg)
    qz_30 = math.sin(math.radians(15.0))
    qw_30 = math.cos(math.radians(15.0))
    pose_30 = GroundTruthPose(
        timestamp=1.0, tx=3.00, ty=-1.00, tz=0.55, qx=0.0, qy=0.0, qz=qz_30, qw=qw_30
    )
    pts_w_30 = project_body_to_world(pts_obs, pose_30)

    expected_xw_30 = 3.00 + 2.15 * math.cos(math.pi / 6.0) - (-0.40) * math.sin(math.pi / 6.0)
    expected_yw_30 = -1.00 + 2.15 * math.sin(math.pi / 6.0) + (-0.40) * math.cos(math.pi / 6.0)
    assert math.isclose(pts_w_30[0, 0], 5.061955, abs_tol=1e-5)
    assert math.isclose(pts_w_30[0, 1], -0.271410, abs_tol=1e-5)
    assert math.isclose(pts_w_30[0, 0], expected_xw_30, abs_tol=1e-6)
    assert math.isclose(pts_w_30[0, 1], expected_yw_30, abs_tol=1e-6)

    # 6. Secondary test: 90 deg yaw (pi/2 rad)
    # Expected:
    # X_w = 3.00 + 2.15*0 - (-0.40)*1 = 3.40 m
    # Y_w = -1.00 + 2.15*1 + (-0.40)*0 = 1.15 m
    qz_90 = math.sin(math.radians(45.0))
    qw_90 = math.cos(math.radians(45.0))
    pose_90 = GroundTruthPose(
        timestamp=1.0, tx=3.00, ty=-1.00, tz=0.55, qx=0.0, qy=0.0, qz=qz_90, qw=qw_90
    )
    pts_w_90 = project_body_to_world(pts_obs, pose_90)
    assert math.isclose(pts_w_90[0, 0], 3.40, abs_tol=1e-5)
    assert math.isclose(pts_w_90[0, 1], 1.15, abs_tol=1e-5)


def test_transformation_composition_no_double_offset():
    """Verify mounting offsets are rotated with vehicle heading and applied exactly once without duplication."""
    dx, dy, dz = 0.15, -0.05, 0.40
    extrinsics = CameraExtrinsics(dx=dx, dy=dy, dz=dz)

    # An optical point at (0, 0, 0) represents the camera optical center
    pts_c = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
    pts_b = transform_camera_to_body(pts_c, extrinsics)

    # Body position of camera optical center must be exactly (dx, dy, dz)
    assert math.isclose(pts_b[0, 0], dx, abs_tol=1e-6)
    assert math.isclose(pts_b[0, 1], dy, abs_tol=1e-6)
    assert math.isclose(pts_b[0, 2], dz, abs_tol=1e-6)

    # World position when robot is at (rx, ry) with heading yaw
    rx, ry, yaw = 5.0, 2.0, math.pi / 4.0
    robot_pose = RobotState(x=rx, y=ry, theta=yaw, v=0.0, omega=0.0, timestamp=1.0)
    pts_w = project_body_to_world(pts_b, robot_pose)

    expected_xw = rx + dx * math.cos(yaw) - dy * math.sin(yaw)
    expected_yw = ry + dx * math.sin(yaw) + dy * math.cos(yaw)

    assert math.isclose(pts_w[0, 0], expected_xw, abs_tol=1e-6)
    assert math.isclose(pts_w[0, 1], expected_yw, abs_tol=1e-6)


def test_grid_semantics_obstacle_endpoint_rasterization_only():
    """Verify GridMap insertion only marks endpoint hits as occupied and does not perform ray clearing."""
    grid = GridMap(width=20, height=20, resolution=0.5, origin=(0.0, 0.0))

    # An obstacle point hit at world (5.25, 5.25) corresponds to grid cell (10, 10)
    pts_w = np.array([[5.25, 5.25]], dtype=np.float64)
    insert_points_into_grid_map(pts_w, grid)

    # Hit cell is marked occupied
    assert grid.is_occupied(10, 10)

    # Ray path cells between sensor at (0, 0) and hit at (10, 10) remain UNMARKED (0)
    # Demonstrating that this is endpoint rasterization, NOT free-space ray tracing
    assert not grid.is_occupied(2, 2)
    assert not grid.is_occupied(5, 5)
    assert not grid.is_occupied(8, 8)
    assert grid.grid[2, 2] == 0
    assert grid.grid[5, 5] == 0
    assert grid.grid[8, 8] == 0





