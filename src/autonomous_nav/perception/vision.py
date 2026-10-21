"""Camera models, coordinate transformations, and synthetic scene rendering."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import cv2
import numpy as np
from autonomous_nav.environment.grid_map import GridMap


@dataclass(frozen=True)
class OrthographicCamera:
    """Represents a calibrated top-down orthographic camera observing a planar ground region.

    Coordinate Conventions:
        - Image Frame: (u, v) where u in [0, image_width - 1] (horizontal right),
          v in [0, image_height - 1] (vertical down). Origin (0, 0) is top-left.
        - World Frame: (x, y) in metric meters. x points right (East),
          y points up (North). Origin is at world_x_min, world_y_min.

    Calibration Assumptions:
        - Flat planar terrain (Z = 0).
        - Zero lens distortion.
        - Orthographic parallel projection with known metric extent:
          [world_x_min, world_x_max] x [world_y_min, world_y_max].
    """
    image_width: int
    image_height: int
    world_x_min: float
    world_y_min: float
    world_x_max: float
    world_y_max: float

    def __post_init__(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError(f"Image dimensions must be positive, got {self.image_width}x{self.image_height}")
        if self.world_x_max <= self.world_x_min:
            raise ValueError(f"world_x_max ({self.world_x_max}) must be > world_x_min ({self.world_x_min})")
        if self.world_y_max <= self.world_y_min:
            raise ValueError(f"world_y_max ({self.world_y_max}) must be > world_y_min ({self.world_y_min})")

    @property
    def scale_x(self) -> float:
        """Pixels per meter horizontally."""
        return float(self.image_width) / (self.world_x_max - self.world_x_min)

    @property
    def scale_y(self) -> float:
        """Pixels per meter vertically."""
        return float(self.image_height) / (self.world_y_max - self.world_y_min)

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """Convert continuous metric world coordinates (x, y) to image pixel coordinates (u, v)."""
        u = (float(x) - self.world_x_min) * self.scale_x
        # Invert vertical axis: image v = 0 is world_y_max, image v = H is world_y_min
        v = (self.world_y_max - float(y)) * self.scale_y
        return u, v

    def pixel_to_world(self, u: float, v: float) -> Tuple[float, float]:
        """Convert image pixel coordinates (u, v) to continuous metric world coordinates (x, y)."""
        x = self.world_x_min + (float(u) / self.scale_x)
        y = self.world_y_max - (float(v) / self.scale_y)
        return x, y

    def is_pixel_in_bounds(self, u: float, v: float) -> bool:
        """Check if pixel coordinate lies within image dimensions."""
        return 0.0 <= u < float(self.image_width) and 0.0 <= v < float(self.image_height)

    def is_world_in_bounds(self, x: float, y: float) -> bool:
        """Check if world coordinate lies within observed camera region."""
        return self.world_x_min <= x <= self.world_x_max and self.world_y_min <= y <= self.world_y_max


@dataclass(frozen=True)
class PlanarPerspectiveCamera:
    """Represents a forward-facing perspective camera calibrated for ground-plane Inverse Perspective Mapping (IPM).

    Assumptions & Critical Limitations:
        - Pinhole camera model with zero radial/tangential distortion.
        - Ground plane is strictly flat at world Z = 0.
        - Camera is mounted at height h (meters), tilted downward by pitch angle alpha (radians).
        - Camera optical center is at (image_width / 2, image_height / 2).
        - IMPORTANT SAFETY NOTE: Ground-plane homography assumes all projected pixels lie on Z = 0.
          It does NOT, by itself, reconstruct the actual physical footprint of a raised 3D obstacle.
          Raised surfaces (e.g. tops and faces of obstacles) project to pixels that, when back-projected
          via H^-1, appear artificially elongated/smeared along the optical radial axis away from the camera.
    """
    image_width: int
    image_height: int
    focal_length_px: float
    camera_height_m: float
    pitch_angle_rad: float

    def __post_init__(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("Image dimensions must be positive")
        if self.focal_length_px <= 0.0:
            raise ValueError("Focal length must be positive")
        if self.camera_height_m <= 0.0:
            raise ValueError("Camera height must be positive")
        if not (0.0 < self.pitch_angle_rad < math.pi / 2.0):
            raise ValueError("Pitch angle must be between 0 and pi/2 radians (downward tilt)")

    @property
    def homography_matrix(self) -> np.ndarray:
        """Compute the 3x3 planar homography mapping ground coordinates (X_g, Y_g, 1) to image pixels (u, v, 1).

        Ground frame: X_g forward, Y_g left, Z_g = 0.
        """
        fx = self.focal_length_px
        fy = self.focal_length_px
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        h = self.camera_height_m
        alpha = self.pitch_angle_rad
        cos_a = math.cos(alpha)
        sin_a = math.sin(alpha)

        # Derived from standard downward-tilted pinhole camera model:
        # Camera mounted at (0, 0, h), rotated by pitch alpha about X-axis (downward tilt).
        # Camera optical frame: Z_c forward/down, X_c right (-Y_g), Y_c down/back.
        # Z_c = X_g * cos(alpha) + h * sin(alpha)
        # X_c = -Y_g
        # Y_c = -X_g * sin(alpha) + h * cos(alpha)
        # u = cx + fx * (X_c / Z_c) = cx - fx * (Y_g / Z_c)
        # v = cy + fy * (Y_c / Z_c) = cy + fy * ((-X_g * sin(alpha) + h * cos(alpha)) / Z_c)
        # In homogeneous matrix form: [u', v', w']^T = H * [X_g, Y_g, 1]^T
        H = np.array([
            [cx * cos_a, -fx, cx * h * sin_a],
            [cy * cos_a - fy * sin_a, 0.0, cy * h * sin_a + fy * h * cos_a],
            [cos_a, 0.0, h * sin_a],
        ], dtype=np.float64)

        return H

    @property
    def inverse_homography_matrix(self) -> np.ndarray:
        """Compute the 3x3 Inverse Perspective Mapping (IPM) matrix mapping image (u, v, 1) to ground (X_g, Y_g, 1)."""
        det = np.linalg.det(self.homography_matrix)
        if abs(det) < 1e-12:
            raise ValueError("Homography matrix is singular or degenerate and cannot be inverted.")
        return np.linalg.inv(self.homography_matrix)

    def is_pixel_in_bounds(self, u: float, v: float) -> bool:
        """Check if pixel coordinate lies strictly within camera sensor bounds."""
        return 0.0 <= float(u) < float(self.image_width) and 0.0 <= float(v) < float(self.image_height)

    def pixel_to_ground(self, u: float, v: float, check_bounds: bool = True) -> Optional[Tuple[float, float]]:
        """Project pixel (u, v) onto ground plane (X_g, Y_g).

        Args:
            u: Horizontal pixel coordinate.
            v: Vertical pixel coordinate.
            check_bounds: If True, returns None if (u, v) lies outside the camera image frame.

        Returns:
            (X_forward, Y_lateral) in meters, or None if pixel is above horizon (X <= 0)
            or has invalid homogeneous scale.
        """
        if check_bounds and not self.is_pixel_in_bounds(u, v):
            return None

        p_img = np.array([float(u), float(v), 1.0], dtype=np.float64)
        p_ground = self.inverse_homography_matrix @ p_img

        if abs(p_ground[2]) < 1e-9:
            return None

        x_g = p_ground[0] / p_ground[2]
        y_g = p_ground[1] / p_ground[2]

        if x_g <= 0.0:
            # Point is behind or at camera horizon
            return None

        return float(x_g), float(y_g)

    def ground_to_pixel(self, x_g: float, y_g: float) -> Optional[Tuple[float, float]]:
        """Project ground point (X_g, Y_g) to image pixel coordinates (u, v)."""
        if x_g <= 0.0:
            return None

        p_ground = np.array([float(x_g), float(y_g), 1.0], dtype=np.float64)
        p_img = self.homography_matrix @ p_ground

        if abs(p_img[2]) < 1e-9:
            return None

        u = p_img[0] / p_img[2]
        v = p_img[1] / p_img[2]
        return float(u), float(v)

    @staticmethod
    def compute_homography_from_correspondences(
        image_points: np.ndarray,
        ground_points: np.ndarray,
    ) -> np.ndarray:
        """Compute 3x3 homography matrix mapping ground points (X_g, Y_g) to image pixels (u, v).

        Args:
            image_points: (N, 2) array of pixel coordinates (u, v). Must have at least 4 correspondences.
            ground_points: (N, 2) array of metric ground plane coordinates (X_g, Y_g).

        Returns:
            3x3 homography matrix H.

        Raises:
            ValueError: If correspondences are collinear, degenerate, or rank-deficient.
        """
        img_pts = np.asarray(image_points, dtype=np.float32)
        gnd_pts = np.asarray(ground_points, dtype=np.float32)

        if img_pts.shape[0] < 4 or gnd_pts.shape[0] < 4:
            raise ValueError(f"At least 4 point correspondences required, got {img_pts.shape[0]}")
        if img_pts.shape[1] != 2 or gnd_pts.shape[1] != 2:
            raise ValueError("Point arrays must have shape (N, 2)")

        # Verify collinearity on 4-point sets
        if img_pts.shape[0] == 4:
            for i in range(4):
                idx = [j for j in range(4) if j != i]
                p0, p1, p2 = img_pts[idx[0]], img_pts[idx[1]], img_pts[idx[2]]
                area = 0.5 * abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
                if area < 1e-4:
                    raise ValueError("Degenerate calibration: image points contain 3 collinear points")

            for i in range(4):
                idx = [j for j in range(4) if j != i]
                p0, p1, p2 = gnd_pts[idx[0]], gnd_pts[idx[1]], gnd_pts[idx[2]]
                area = 0.5 * abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
                if area < 1e-4:
                    raise ValueError("Degenerate calibration: ground points contain 3 collinear points")

        if img_pts.shape[0] == 4 and gnd_pts.shape[0] == 4:
            H = cv2.getPerspectiveTransform(gnd_pts, img_pts)
        else:
            H, _ = cv2.findHomography(gnd_pts, img_pts)

        if H is None or abs(np.linalg.det(H)) < 1e-12:
            raise ValueError("Computed homography is singular or degenerate")

        return H


@dataclass(frozen=True)
class CalibratedHomographyCamera:
    """Camera representation using an explicitly calibrated 3x3 planar homography matrix."""
    image_width: int
    image_height: int
    homography_matrix: np.ndarray

    def __post_init__(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("Image dimensions must be positive")
        if self.homography_matrix.shape != (3, 3):
            raise ValueError("Homography matrix must have shape (3, 3)")
        if abs(np.linalg.det(self.homography_matrix)) < 1e-12:
            raise ValueError("Homography matrix is singular or degenerate and cannot be inverted.")

    @property
    def inverse_homography_matrix(self) -> np.ndarray:
        return np.linalg.inv(self.homography_matrix)

    def is_pixel_in_bounds(self, u: float, v: float) -> bool:
        return 0.0 <= float(u) < float(self.image_width) and 0.0 <= float(v) < float(self.image_height)

    def pixel_to_ground(self, u: float, v: float, check_bounds: bool = True) -> Optional[Tuple[float, float]]:
        if check_bounds and not self.is_pixel_in_bounds(u, v):
            return None
        p_img = np.array([float(u), float(v), 1.0], dtype=np.float64)
        p_g = self.inverse_homography_matrix @ p_img
        if abs(p_g[2]) < 1e-9:
            return None
        xg = p_g[0] / p_g[2]
        yg = p_g[1] / p_g[2]
        return float(xg), float(yg)

    def ground_to_pixel(self, xg: float, yg: float) -> Optional[Tuple[float, float]]:
        p_g = np.array([float(xg), float(yg), 1.0], dtype=np.float64)
        p_img = self.homography_matrix @ p_g
        if abs(p_img[2]) < 1e-9:
            return None
        u = p_img[0] / p_img[2]
        v = p_img[1] / p_img[2]
        return float(u), float(v)


class SyntheticImageRenderer:
    """Generates deterministic synthetic image observations from a known GridMap or obstacle set."""

    def __init__(self, camera: OrthographicCamera) -> None:
        self.camera = camera

    def render_from_grid_map(
        self,
        grid_map: GridMap,
        background_color: int = 240,
        obstacle_color: int = 20,
    ) -> np.ndarray:
        """Render a deterministic synthetic grayscale image from a GridMap."""
        img = np.full(
            (self.camera.image_height, self.camera.image_width),
            fill_value=background_color,
            dtype=np.uint8,
        )

        occupied_indices = np.argwhere(grid_map.grid == 1)
        # occupied_indices has [gy, gx]
        for gy, gx in occupied_indices:
            # Get cell bounding box in world coordinates
            x_min = grid_map.origin[0] + gx * grid_map.resolution
            x_max = x_min + grid_map.resolution
            y_min = grid_map.origin[1] + gy * grid_map.resolution
            y_max = y_min + grid_map.resolution

            u_min, v_max = self.camera.world_to_pixel(x_min, y_min)
            u_max, v_min = self.camera.world_to_pixel(x_max, y_max)

            px_start = max(0, int(math.floor(min(u_min, u_max))))
            px_end = min(self.camera.image_width, int(math.ceil(max(u_min, u_max))))
            py_start = max(0, int(math.floor(min(v_min, v_max))))
            py_end = min(self.camera.image_height, int(math.ceil(max(v_min, v_max))))

            img[py_start:py_end, px_start:px_end] = obstacle_color

        return img

    def render_obstacles(
        self,
        rectangles: Optional[List[Tuple[float, float, float, float]]] = None,
        circles: Optional[List[Tuple[float, float, float]]] = None,
        background_color: int = 240,
        obstacle_color: int = 20,
    ) -> np.ndarray:
        """Render discrete geometric shapes in world coordinates onto synthetic image.

        Args:
            rectangles: List of (x_min, y_min, x_max, y_max) in meters.
            circles: List of (center_x, center_y, radius) in meters.
            background_color: Grayscale background intensity [0, 255].
            obstacle_color: Grayscale obstacle intensity [0, 255].
        """
        img = np.full(
            (self.camera.image_height, self.camera.image_width),
            fill_value=background_color,
            dtype=np.uint8,
        )

        if rectangles:
            for x_min, y_min, x_max, y_max in rectangles:
                u1, v1 = self.camera.world_to_pixel(x_min, y_max)
                u2, v2 = self.camera.world_to_pixel(x_max, y_min)
                pt1 = (int(round(min(u1, u2))), int(round(min(v1, v2))))
                pt2 = (int(round(max(u1, u2))), int(round(max(v1, v2))))
                cv2.rectangle(img, pt1, pt2, color=obstacle_color, thickness=-1)

        if circles:
            for cx, cy, radius in circles:
                u_center, v_center = self.camera.world_to_pixel(cx, cy)
                r_px = int(round(radius * (self.camera.scale_x + self.camera.scale_y) / 2.0))
                center = (int(round(u_center)), int(round(v_center)))
                cv2.circle(img, center, r_px, color=obstacle_color, thickness=-1)

        return img

