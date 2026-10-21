"""Unit and integration tests for vision-based perception pipeline."""

import math
import numpy as np
import pytest
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.perception.map_adapter import MapAdapter
from autonomous_nav.perception.obstacle_detection import DetectedObstacle, ObstacleDetector
from autonomous_nav.perception.preprocessing import (
    apply_blur,
    normalize_intensity,
    preprocess_image,
    to_grayscale,
    validate_image,
)
from autonomous_nav.perception.vision import (
    CalibratedHomographyCamera,
    OrthographicCamera,
    PlanarPerspectiveCamera,
    SyntheticImageRenderer,
)
from autonomous_nav.planning.astar import AStarPlanner
from autonomous_nav.simulation.collision import CollisionChecker


# ---------------------------------------------------------------------------
# 1. Image Preprocessing Tests
# ---------------------------------------------------------------------------

def test_valid_image_preprocessing():
    """Verify color conversion, Gaussian blur, and contrast normalization."""
    # Synthetic 3-channel RGB image
    rgb_img = np.zeros((100, 100, 3), dtype=np.uint8)
    rgb_img[20:50, 20:50, :] = [200, 100, 50]

    result = preprocess_image(rgb_img, color_format="RGB", blur_kernel_size=5, blur_sigma=1.0)

    assert result.original.shape == (100, 100, 3)
    assert result.grayscale.shape == (100, 100)
    assert result.grayscale.dtype == np.uint8
    assert result.blurred.shape == (100, 100)
    assert result.blurred.dtype == np.uint8
    assert result.normalized.shape == (100, 100)


def test_color_conversions():
    """Verify BGR and single-channel grayscale inputs are properly handled."""
    bgr_img = np.full((50, 50, 3), fill_value=120, dtype=np.uint8)
    gray_from_bgr = to_grayscale(bgr_img, color_format="BGR")
    assert gray_from_bgr.shape == (50, 50)

    gray_input = np.full((50, 50), fill_value=80, dtype=np.uint8)
    gray_out = to_grayscale(gray_input)
    assert np.array_equal(gray_input, gray_out)


def test_normalization_dynamic_range():
    """Verify normalization expands low-contrast image across full [0, 255] spectrum."""
    low_contrast = np.full((40, 40), fill_value=100, dtype=np.uint8)
    low_contrast[10:20, 10:20] = 110  # Range [100, 110]

    normalized = normalize_intensity(low_contrast)
    assert int(np.min(normalized)) == 0
    assert int(np.max(normalized)) == 255


# ---------------------------------------------------------------------------
# 2. Input Validation & Error Handling Tests
# ---------------------------------------------------------------------------

def test_invalid_image_inputs():
    """Verify TypeError and ValueError are raised for improper image inputs."""
    with pytest.raises(TypeError, match="cannot be None"):
        validate_image(None)

    with pytest.raises(TypeError, match="must be a numpy.ndarray"):
        validate_image([[1, 2], [3, 4]])

    with pytest.raises(ValueError, match="empty"):
        validate_image(np.array([], dtype=np.uint8))

    with pytest.raises(TypeError, match="dtype uint8"):
        validate_image(np.zeros((50, 50), dtype=np.float32))

    with pytest.raises(ValueError, match="dimensions"):
        validate_image(np.zeros((1, 50), dtype=np.uint8))

    with pytest.raises(ValueError, match="channel count"):
        validate_image(np.zeros((50, 50, 5), dtype=np.uint8))


# ---------------------------------------------------------------------------
# 3. Empty & Uniform Image Tests
# ---------------------------------------------------------------------------

def test_empty_and_uniform_images():
    """Verify that images with uniform background produce zero detected obstacles."""
    detector = ObstacleDetector()

    # Uniform white image
    white_img = np.full((100, 100), fill_value=255, dtype=np.uint8)
    res_white = detector.detect(white_img)
    assert len(res_white.obstacles) == 0
    assert np.all(res_white.binary_mask == 0)

    # Uniform black image
    black_img = np.full((100, 100), fill_value=0, dtype=np.uint8)
    res_black = detector.detect(black_img)
    assert len(res_black.obstacles) == 0
    assert np.all(res_black.binary_mask == 0)


# ---------------------------------------------------------------------------
# 4. Known Synthetic Obstacle Shapes
# ---------------------------------------------------------------------------

def test_detect_known_shapes():
    """Verify detection accuracy for known rectangular and circular obstacle primitives."""
    camera = OrthographicCamera(
        image_width=200,
        image_height=200,
        world_x_min=0.0,
        world_y_min=0.0,
        world_x_max=10.0,
        world_y_max=10.0,
    )
    renderer = SyntheticImageRenderer(camera)

    # Single rectangle from world [2.0, 3.0] to [4.0, 7.0] -> center (3.0, 5.0)
    img_rect = renderer.render_obstacles(
        rectangles=[(2.0, 3.0, 4.0, 7.0)],
        background_color=255,
        obstacle_color=0,
    )

    detector = ObstacleDetector(min_area_pixels=50.0)
    res = detector.detect(img_rect)

    assert len(res.obstacles) == 1
    obs = res.obstacles[0]
    u_c, v_c = obs.centroid_pixel
    wx_c, wy_c = camera.pixel_to_world(u_c, v_c)

    # Centroid should match center (3.0, 5.0) within 0.15m
    assert pytest.approx(wx_c, abs=0.15) == 3.0
    assert pytest.approx(wy_c, abs=0.15) == 5.0


def test_detect_multiple_obstacles():
    """Verify detector extracts all disjoint obstacles without merges."""
    camera = OrthographicCamera(
        image_width=300,
        image_height=200,
        world_x_min=0.0,
        world_y_min=0.0,
        world_x_max=15.0,
        world_y_max=10.0,
    )
    renderer = SyntheticImageRenderer(camera)

    rectangles = [
        (2.0, 2.0, 4.0, 4.0),
        (8.0, 6.0, 10.0, 8.0),
    ]
    circles = [
        (12.0, 3.0, 1.0),
    ]
    img = renderer.render_obstacles(rectangles=rectangles, circles=circles)

    detector = ObstacleDetector(min_area_pixels=30.0)
    res = detector.detect(img)
    assert len(res.obstacles) == 3


# ---------------------------------------------------------------------------
# 5. Coordinate Transformations & Calibration Tests
# ---------------------------------------------------------------------------

def test_orthographic_camera_roundtrip_transforms():
    """Verify bidirectional mapping between pixel and world frames is consistent."""
    camera = OrthographicCamera(
        image_width=640,
        image_height=480,
        world_x_min=5.0,
        world_y_min=10.0,
        world_x_max=35.0,
        world_y_max=30.0,
    )

    # Test center point
    u_orig, v_orig = 320.0, 240.0
    wx, wy = camera.pixel_to_world(u_orig, v_orig)
    u_rt, v_rt = camera.world_to_pixel(wx, wy)

    assert pytest.approx(u_rt, abs=1e-5) == u_orig
    assert pytest.approx(v_rt, abs=1e-5) == v_orig

    # Test bounds checking
    assert camera.is_pixel_in_bounds(0.0, 0.0)
    assert camera.is_pixel_in_bounds(639.0, 479.0)
    assert not camera.is_pixel_in_bounds(-1.0, 0.0)
    assert not camera.is_pixel_in_bounds(640.0, 200.0)

    assert camera.is_world_in_bounds(20.0, 20.0)
    assert not camera.is_world_in_bounds(4.0, 20.0)


def test_planar_perspective_homography_transforms():
    """Verify ground-plane homography and inverse IPM mapping for perspective camera."""
    cam = PlanarPerspectiveCamera(
        image_width=640,
        image_height=480,
        focal_length_px=500.0,
        camera_height_m=1.5,
        pitch_angle_rad=math.radians(30.0),
    )

    # Test ground point 10 meters forward, 1 meter to the left
    xg_orig, yg_orig = 10.0, 1.0
    pixel = cam.ground_to_pixel(xg_orig, yg_orig)
    assert pixel is not None
    u, v = pixel

    # Project pixel back to ground
    ground_rt = cam.pixel_to_ground(u, v)
    assert ground_rt is not None
    xg_rt, yg_rt = ground_rt

    assert pytest.approx(xg_rt, abs=1e-3) == xg_orig
    assert pytest.approx(yg_rt, abs=1e-3) == yg_orig

    # Point at or behind camera should return None
    assert cam.ground_to_pixel(-2.0, 0.0) is None


# ---------------------------------------------------------------------------
# 6. Determinism & Noise Filtering Tests
# ---------------------------------------------------------------------------

def test_deterministic_output_for_identical_inputs():
    """Verify detection yields identical numerical outputs on repeated execution."""
    camera = OrthographicCamera(
        image_width=200, image_height=200,
        world_x_min=0.0, world_y_min=0.0,
        world_x_max=10.0, world_y_max=10.0,
    )
    renderer = SyntheticImageRenderer(camera)
    img = renderer.render_obstacles(rectangles=[(3.0, 3.0, 7.0, 7.0)])

    detector = ObstacleDetector()
    run1 = detector.detect(img)
    run2 = detector.detect(img)

    assert len(run1.obstacles) == len(run2.obstacles)
    assert np.array_equal(run1.binary_mask, run2.binary_mask)
    assert run1.obstacles[0].centroid_pixel == run2.obstacles[0].centroid_pixel
    assert run1.obstacles[0].area_pixels == run2.obstacles[0].area_pixels


def test_noise_filtering_min_max_area():
    """Verify tiny noise specks are filtered out by min_area_pixels threshold."""
    img = np.full((100, 100), fill_value=255, dtype=np.uint8)
    # Add 2x2 noise speck (area 4 px)
    img[10:12, 10:12] = 0
    # Add large valid obstacle (20x20 = 400 px)
    img[50:70, 50:70] = 0

    detector = ObstacleDetector(min_area_pixels=50.0)
    res = detector.detect(img)

    # Only the 20x20 obstacle must be detected
    assert len(res.obstacles) == 1
    assert res.obstacles[0].area_pixels > 200.0


# ---------------------------------------------------------------------------
# 7. GridMap Updates & Navigation Integration Tests
# ---------------------------------------------------------------------------

def test_grid_map_updates_from_detections():
    """Verify MapAdapter rasterizes detected obstacles onto a clean GridMap."""
    camera = OrthographicCamera(
        image_width=200, image_height=200,
        world_x_min=0.0, world_y_min=0.0,
        world_x_max=10.0, world_y_max=10.0,
    )
    renderer = SyntheticImageRenderer(camera)
    # Obstacle from [3.0, 3.0] to [7.0, 7.0]
    img = renderer.render_obstacles(rectangles=[(3.0, 3.0, 7.0, 7.0)])

    detector = ObstacleDetector()
    det_res = detector.detect(img)

    # Clean grid map
    grid = GridMap(width=20, height=20, resolution=0.5)
    assert np.all(grid.grid == 0)

    adapter = MapAdapter(camera)
    marked_count = adapter.update_from_detections(det_res.obstacles, grid)

    assert marked_count > 0
    # Center cell (gx=10, gy=10) corresponding to (5.0, 5.0) must be occupied
    gx, gy = grid.world_to_grid(5.0, 5.0)
    assert grid.is_occupied(gx, gy)
    # Origin cell (gx=1, gy=1) corresponding to (0.5, 0.5) must remain free
    gx_free, gy_free = grid.world_to_grid(0.5, 0.5)
    assert not grid.is_occupied(gx_free, gy_free)


def test_complete_perception_to_path_planning_integration():
    """Verify that perceived obstacles successfully block direct A* path and force an avoidance detour."""
    camera = OrthographicCamera(
        image_width=400, image_height=200,
        world_x_min=0.0, world_y_min=0.0,
        world_x_max=20.0, world_y_max=10.0,
    )
    renderer = SyntheticImageRenderer(camera)

    # Place barrier wall directly across the straight path at x = [9.0, 11.0], y = [0.0, 8.0]
    wall_rect = (9.0, 0.0, 11.0, 8.0)
    obs_img = renderer.render_obstacles(rectangles=[wall_rect])

    # Perception
    detector = ObstacleDetector()
    det_res = detector.detect(obs_img)
    assert len(det_res.obstacles) == 1

    # Mapping
    grid = GridMap(width=40, height=20, resolution=0.5)
    adapter = MapAdapter(camera)
    adapter.update_from_detections(det_res.obstacles, grid)

    # Path Planning
    planner = AStarPlanner()
    start = (2.0, 4.0)
    goal = (18.0, 4.0)

    # Before adding obstacle, straight line was free
    # With perceived obstacle, A* must navigate around the top of the wall (y > 8.0)
    path = planner.plan_world(start, goal, grid)
    assert path is not None
    assert len(path) > 0

    # Ensure no waypoint in path intersects the wall
    for wx, wy in path:
        assert not (9.0 <= wx <= 11.0 and 0.0 <= wy <= 8.0), f"Path intersects perceived obstacle at ({wx}, {wy})"


# ---------------------------------------------------------------------------
# 8. Milestone 2.1 Audit: Camera Models & Transformations
# ---------------------------------------------------------------------------

def test_orthographic_camera_boundary_and_corners():
    """Audit all 4 image corners, vertical axis inversion, metric scale, and boundary checks."""
    cam = OrthographicCamera(
        image_width=640,
        image_height=480,
        world_x_min=0.0,
        world_y_min=0.0,
        world_x_max=30.0,
        world_y_max=20.0,
    )

    # 1. Metric scale units
    assert pytest.approx(cam.scale_x, abs=1e-3) == 640.0 / 30.0
    assert pytest.approx(cam.scale_y, abs=1e-3) == 480.0 / 20.0

    # 2. Vertical axis convention: v = 0 is world_y_max (20.0m), v = H is world_y_min (0.0m)
    x_top_left, y_top_left = cam.pixel_to_world(0.0, 0.0)
    assert pytest.approx(x_top_left, abs=1e-4) == 0.0
    assert pytest.approx(y_top_left, abs=1e-4) == 20.0

    x_bot_right, y_bot_right = cam.pixel_to_world(640.0, 480.0)
    assert pytest.approx(x_bot_right, abs=1e-4) == 30.0
    assert pytest.approx(y_bot_right, abs=1e-4) == 0.0

    # 3. All 4 pixel corners forward and inverse round-trip
    corners = [(0.0, 0.0), (639.0, 0.0), (0.0, 479.0), (639.0, 479.0)]
    for u, v in corners:
        wx, wy = cam.pixel_to_world(u, v)
        u_rt, v_rt = cam.world_to_pixel(wx, wy)
        assert pytest.approx(u_rt, abs=1e-4) == u
        assert pytest.approx(v_rt, abs=1e-4) == v

    # 4. Out-of-range checks
    assert not cam.is_pixel_in_bounds(-1.0, 240.0)
    assert not cam.is_pixel_in_bounds(640.0, 240.0)
    assert not cam.is_pixel_in_bounds(320.0, -0.5)
    assert not cam.is_pixel_in_bounds(320.0, 480.0)

    assert not cam.is_world_in_bounds(-0.1, 10.0)
    assert not cam.is_world_in_bounds(30.1, 10.0)
    assert not cam.is_world_in_bounds(15.0, -0.1)
    assert not cam.is_world_in_bounds(15.0, 20.1)


def test_planar_perspective_homography_from_correspondences():
    """Verify homography construction from 4 known coplanar calibration correspondences."""
    # 4 ground plane points (meters)
    gnd_pts = np.array([
        [2.5, -1.0],
        [2.5, 1.0],
        [8.0, 1.5],
        [8.0, -1.5],
    ], dtype=np.float32)

    # Parametric camera producing ground truth projection
    param_cam = PlanarPerspectiveCamera(
        image_width=640,
        image_height=480,
        focal_length_px=500.0,
        camera_height_m=1.5,
        pitch_angle_rad=math.radians(30.0),
    )
    img_pts = np.array([param_cam.ground_to_pixel(x, y) for x, y in gnd_pts], dtype=np.float32)

    # Compute homography directly from correspondences
    H = PlanarPerspectiveCamera.compute_homography_from_correspondences(img_pts, gnd_pts)
    assert H.shape == (3, 3)
    assert abs(np.linalg.det(H)) > 1e-6

    # Test CalibratedHomographyCamera instantiation
    calib_cam = CalibratedHomographyCamera(640, 480, H)

    # Round-trip coplanar ground point (5.0m forward, 0.5m left)
    pix = calib_cam.ground_to_pixel(5.0, 0.5)
    assert pix is not None
    u, v = pix
    assert calib_cam.is_pixel_in_bounds(u, v)

    rec_gnd = calib_cam.pixel_to_ground(u, v)
    assert rec_gnd is not None
    assert pytest.approx(rec_gnd[0], abs=1e-3) == 5.0
    assert pytest.approx(rec_gnd[1], abs=1e-3) == 0.5


def test_planar_perspective_degenerate_and_singular_inputs():
    """Audit degenerate calibration correspondences, collinearity, and singular matrices."""
    # 1. Insufficient points (< 4)
    with pytest.raises(ValueError, match="At least 4"):
        PlanarPerspectiveCamera.compute_homography_from_correspondences(
            np.zeros((3, 2)), np.zeros((3, 2))
        )

    # 2. 3 collinear calibration points in ground array
    collinear_gnd = np.array([[2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [5.0, 2.0]], dtype=np.float32)
    valid_img = np.array([[100.0, 100.0], [200.0, 100.0], [300.0, 100.0], [250.0, 300.0]], dtype=np.float32)
    with pytest.raises(ValueError, match="collinear"):
        PlanarPerspectiveCamera.compute_homography_from_correspondences(valid_img, collinear_gnd)

    # 3. Singular homography matrix
    singular_H = np.zeros((3, 3), dtype=np.float64)
    with pytest.raises(ValueError, match="singular or degenerate"):
        CalibratedHomographyCamera(640, 480, singular_H)

    # 4. Out-of-bounds pixel projection returns None when check_bounds=True
    cam = PlanarPerspectiveCamera(640, 480, 500.0, 1.5, math.radians(30.0))
    assert cam.pixel_to_ground(-10.0, 240.0, check_bounds=True) is None
    assert cam.pixel_to_ground(700.0, 240.0, check_bounds=True) is None

    # 5. Ground point behind horizon (x <= 0) returns None
    assert cam.ground_to_pixel(-1.0, 0.0) is None
    assert cam.ground_to_pixel(0.0, 0.0) is None


# ---------------------------------------------------------------------------
# 9. Milestone 2.1 Audit: Obstacle Footprints & Map Safety
# ---------------------------------------------------------------------------

def test_obstacle_footprint_mapping_modes():
    """Verify MapAdapter rasterizes a filled obstacle footprint rather than only a centroid or outline."""
    cam = OrthographicCamera(200, 200, 0.0, 0.0, 10.0, 10.0)
    renderer = SyntheticImageRenderer(cam)
    # 4x4m square obstacle from [3.0, 3.0] to [7.0, 7.0]
    img = renderer.render_obstacles(rectangles=[(3.0, 3.0, 7.0, 7.0)])
    det = ObstacleDetector().detect(img)

    grid = GridMap(width=20, height=20, resolution=0.5)
    adapter = MapAdapter(cam)
    marked_cells = adapter.update_from_detections(det.obstacles, grid, use_full_contour=True)

    # A 4x4m square on 0.5m grid contains (4/0.5) x (4/0.5) = 8x8 = 64 core cells
    # Footprint rasterization must mark > 50 cells, demonstrating filled interior
    assert marked_cells >= 50
    # Center cell must be occupied
    gx_c, gy_c = grid.world_to_grid(5.0, 5.0)
    assert grid.is_occupied(gx_c, gy_c)


def test_obstacle_detection_precision_recall_and_centroid_accuracy():
    """Verify 100% precision/recall on controlled synthetic fixture and measure centroid error."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    renderer = SyntheticImageRenderer(cam)

    # 3 distinct obstacles: 2 rectangles, 1 circle
    gt_obstacles = [
        {"type": "rect", "geom": (20.0, 4.0, 24.0, 8.0), "center": (22.0, 6.0)},
        {"type": "rect", "geom": (8.0, 8.0, 12.0, 12.0), "center": (10.0, 10.0)},
        {"type": "circle", "geom": (16.0, 15.0, 2.0), "center": (16.0, 15.0)},
    ]

    img = renderer.render_obstacles(
        rectangles=[gt_obstacles[0]["geom"], gt_obstacles[1]["geom"]],
        circles=[gt_obstacles[2]["geom"]],
    )

    det = ObstacleDetector(min_area_pixels=30.0)
    result = det.detect(img)

    # Precision & Recall
    tp = len(result.obstacles)
    assert tp == 3, f"Expected 3 detections, got {tp}"

    # Centroid localization error
    for gt in gt_obstacles:
        gt_x, gt_y = gt["center"]
        # Find closest detected obstacle
        min_err = float("inf")
        for obs in result.obstacles:
            wx, wy = cam.pixel_to_world(*obs.centroid_pixel)
            err = math.hypot(wx - gt_x, wy - gt_y)
            if err < min_err:
                min_err = err
        # Synthetic centroid error must be sub-5cm
        assert min_err < 0.05, f"Centroid error {min_err:.3f}m exceeds 0.05m tolerance for {gt}"


def test_obstacle_footprint_iou_and_coverage():
    """Measure IoU between perceived rasterized obstacle footprint and ground-truth grid map."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    renderer = SyntheticImageRenderer(cam)

    rect = (4.0, 4.0, 8.0, 8.0)  # 4m x 4m rectangle
    img = renderer.render_obstacles(rectangles=[rect])
    det = ObstacleDetector().detect(img)

    # Perceived grid map
    grid_perceived = GridMap(60, 40, 0.5)
    adapter = MapAdapter(cam)
    adapter.update_from_detections(det.obstacles, grid_perceived, use_full_contour=True)

    # Ground-truth grid map
    grid_gt = GridMap(60, 40, 0.5)
    grid_gt.add_rectangular_obstacle(4.0, 4.0, 8.0, 8.0)

    # Intersection-over-Union (IoU)
    intersection = int(np.sum((grid_perceived.grid == 1) & (grid_gt.grid == 1)))
    union = int(np.sum((grid_perceived.grid == 1) | (grid_gt.grid == 1)))
    iou = intersection / float(union)

    # IoU must be >= 0.88, with zero false negatives (no under-estimation of danger)
    assert iou >= 0.88, f"Perceived footprint IoU {iou:.3f} below 0.88 threshold"
    fn = int(np.sum((grid_perceived.grid == 0) & (grid_gt.grid == 1)))
    assert fn == 0, f"Dangerous under-estimation: {fn} ground-truth obstacle cells missed"


def test_obstacles_touching_and_crossing_map_boundaries():
    """Verify obstacles partly or completely outside map boundaries do not corrupt unrelated cells."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    renderer = SyntheticImageRenderer(cam)

    # 1. Obstacle partially crossing map boundary: x in [-3.0, 3.0], y in [4.0, 8.0]
    img_partial = renderer.render_obstacles(rectangles=[(-3.0, 4.0, 3.0, 8.0)])
    det_partial = ObstacleDetector().detect(img_partial)

    grid = GridMap(width=60, height=40, resolution=0.5)  # World [0, 30] x [0, 20]
    adapter = MapAdapter(cam)
    marked = adapter.update_from_detections(det_partial.obstacles, grid)

    assert marked > 0
    # Inside portion must be occupied: (1.5, 6.0)
    gx_in, gy_in = grid.world_to_grid(1.5, 6.0)
    assert grid.is_occupied(gx_in, gy_in)
    # Unrelated cells must remain free: (15.0, 15.0)
    gx_unrelated, gy_unrelated = grid.world_to_grid(15.0, 15.0)
    assert not grid.is_occupied(gx_unrelated, gy_unrelated)

    # 2. Obstacle completely outside map: x in [-10.0, -5.0]
    img_outside = renderer.render_obstacles(rectangles=[(-10.0, 4.0, -5.0, 8.0)])
    det_outside = ObstacleDetector().detect(img_outside)
    grid_outside = GridMap(width=60, height=40, resolution=0.5)
    marked_outside = adapter.update_from_detections(det_outside.obstacles, grid_outside)
    assert marked_outside == 0


def test_binary_mask_affine_warp_projection():
    """Verify MapAdapter.update_from_binary_mask uses affine warp to accurately map occupancy."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    grid = GridMap(60, 40, 0.5)

    # Synthetic binary mask with 100x100 square
    mask = np.zeros((480, 640), dtype=np.uint8)
    # Draw rectangle at pixel [200, 200] to [300, 300]
    mask[200:300, 200:300] = 255

    adapter = MapAdapter(cam)
    marked = adapter.update_from_binary_mask(mask, grid)
    assert marked > 0

    # Calculate expected world center for pixel (250, 250)
    wx, wy = cam.pixel_to_world(250.0, 250.0)
    gx, gy = grid.world_to_grid(wx, wy)
    assert grid.is_occupied(gx, gy)


# ---------------------------------------------------------------------------
# 10. Milestone 2.1 Audit: Robot Clearance & Path Safety
# ---------------------------------------------------------------------------

def test_perceived_obstacle_clearance_and_inflation_safety():
    """Verify that obstacle inflation preserves robot clearance and corner-cutting invariants."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    renderer = SyntheticImageRenderer(cam)

    # Obstacle at [10.0, 5.0, 14.0, 15.0] (4m x 10m barrier)
    img = renderer.render_obstacles(rectangles=[(10.0, 5.0, 14.0, 15.0)])
    det = ObstacleDetector().detect(img)

    grid = GridMap(width=60, height=40, resolution=0.5)
    adapter = MapAdapter(cam)
    adapter.update_from_detections(det.obstacles, grid)

    # Inflate obstacles with robot radius buffer (0.6m) returning a new GridMap
    inflated_grid = grid.inflate_obstacles(radius_meters=0.6)

    # Plan path around inflated obstacle
    planner = AStarPlanner()
    start = (5.0, 10.0)
    goal = (20.0, 10.0)
    path = planner.plan_world(start, goal, inflated_grid)

    assert path is not None
    assert len(path) > 1

    # Verify physical clearance: every waypoint must be > 0.5m away from raw obstacle boundary
    for wx, wy in path:
        # Distance to rectangle [10, 5, 14, 15]
        cx = max(10.0, min(wx, 14.0))
        cy = max(5.0, min(wy, 15.0))
        dist = math.hypot(wx - cx, wy - cy)
        assert dist >= 0.5, f"Waypoint ({wx:.2f}, {wy:.2f}) violates 0.5m clearance: dist={dist:.3f}m"


def test_perceived_obstacle_unreachable_goal_reporting():
    """Verify that an unreachable goal enclosed by perceived obstacles safely returns None."""
    cam = OrthographicCamera(640, 480, 0.0, 0.0, 30.0, 20.0)
    renderer = SyntheticImageRenderer(cam)

    # Complete enclosure around goal (15.0, 10.0)
    enclosure_walls = [
        (12.0, 7.0, 13.0, 13.0),   # Left wall
        (17.0, 7.0, 18.0, 13.0),   # Right wall
        (12.0, 7.0, 18.0, 8.0),    # Bottom wall
        (12.0, 12.0, 18.0, 13.0),  # Top wall
    ]
    img = renderer.render_obstacles(rectangles=enclosure_walls)
    det = ObstacleDetector().detect(img)

    grid = GridMap(width=60, height=40, resolution=0.5)
    adapter = MapAdapter(cam)
    adapter.update_from_detections(det.obstacles, grid)

    planner = AStarPlanner()
    start = (2.0, 2.0)
    goal = (15.0, 10.0)

    path = planner.plan_world(start, goal, grid)
    # Must report planning failure gracefully without throwing exceptions
    assert path is None

