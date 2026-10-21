"""Classical OpenCV-based obstacle detection on synthetic camera observations."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import cv2
import numpy as np
from autonomous_nav.perception.preprocessing import to_grayscale, validate_image


@dataclass
class DetectedObstacle:
    """Represents an obstacle detected in 2D image coordinates."""
    contour: np.ndarray  # (N, 1, 2) pixel coordinates
    bbox: Tuple[int, int, int, int]  # (u_min, v_min, width, height)
    centroid_pixel: Tuple[float, float]  # (u_bar, v_bar)
    area_pixels: float
    polygon_approx: np.ndarray  # Approximated polygon vertices (M, 1, 2)


@dataclass
class DetectionResult:
    """Aggregated output from the obstacle detection stage."""
    binary_mask: np.ndarray  # uint8 binary mask (0 = free, 255 = obstacle)
    obstacles: List[DetectedObstacle] = field(default_factory=list)
    annotated_image: Optional[np.ndarray] = None


class ObstacleDetector:
    """Detects geometric obstacles in synthetic camera images using classical OpenCV operations."""

    def __init__(
        self,
        threshold_method: str = "otsu",
        binary_threshold: int = 127,
        invert_threshold: bool = True,
        morphology_kernel_size: int = 3,
        min_area_pixels: float = 20.0,
        max_area_pixels: float = 250000.0,
    ) -> None:
        self.threshold_method = threshold_method.lower()
        self.binary_threshold = int(binary_threshold)
        self.invert_threshold = bool(invert_threshold)
        self.morphology_kernel_size = int(morphology_kernel_size)
        self.min_area_pixels = float(min_area_pixels)
        self.max_area_pixels = float(max_area_pixels)

        if self.morphology_kernel_size > 0 and self.morphology_kernel_size % 2 == 0:
            self.morphology_kernel_size += 1

    def threshold(self, gray_image: np.ndarray) -> np.ndarray:
        """Apply configured thresholding technique to produce binary mask."""
        if self.threshold_method == "otsu":
            flag = cv2.THRESH_BINARY_INV if self.invert_threshold else cv2.THRESH_BINARY
            _, mask = cv2.threshold(gray_image, 0, 255, flag | cv2.THRESH_OTSU)
        elif self.threshold_method == "adaptive":
            flag = cv2.THRESH_BINARY_INV if self.invert_threshold else cv2.THRESH_BINARY
            mask = cv2.adaptiveThreshold(
                gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, flag, 11, 2.0
            )
        else:
            # Standard binary threshold
            flag = cv2.THRESH_BINARY_INV if self.invert_threshold else cv2.THRESH_BINARY
            _, mask = cv2.threshold(gray_image, self.binary_threshold, 255, flag)

        return mask

    def clean_morphology(self, binary_mask: np.ndarray) -> np.ndarray:
        """Apply morphological opening and closing to suppress noise and seal small gaps."""
        if self.morphology_kernel_size <= 1:
            return binary_mask.copy()

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.morphology_kernel_size, self.morphology_kernel_size),
        )
        # Opening removes isolated noise pixels; closing bridges small internal voids
        opened = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        return closed

    def detect(self, image: np.ndarray) -> DetectionResult:
        """Execute full detection pipeline on input image.

        Args:
            image: Single-channel grayscale or multi-channel RGB/BGR uint8 array.

        Returns:
            DetectionResult container with binary mask, detected obstacles, and annotated visual.
        """
        validate_image(image)

        # Convert to single-channel grayscale if needed
        gray = to_grayscale(image)

        # Handle flat/uniform images
        if int(np.max(gray)) == int(np.min(gray)):
            empty_mask = np.zeros_like(gray, dtype=np.uint8)
            annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            return DetectionResult(
                binary_mask=empty_mask,
                obstacles=[],
                annotated_image=annotated,
            )

        # Stage 1: Thresholding
        raw_mask = self.threshold(gray)

        # Stage 2: Morphological filtering
        clean_mask = self.clean_morphology(raw_mask)

        # Stage 3: Contour extraction
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles: List[DetectedObstacle] = []
        annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_area_pixels or area > self.max_area_pixels:
                continue

            # Bounding box (u, v, width, height)
            bx, by, bw, bh = cv2.boundingRect(cnt)

            # Spatial moments for centroid
            M = cv2.moments(cnt)
            if M["m00"] > 1e-5:
                cx = float(M["m10"] / M["m00"])
                cy = float(M["m01"] / M["m00"])
            else:
                cx = float(bx + bw / 2.0)
                cy = float(by + bh / 2.0)

            # Polygon approximation
            perimeter = cv2.arcLength(cnt, closed=True)
            approx = cv2.approxPolyDP(cnt, epsilon=0.02 * perimeter, closed=True)

            obs = DetectedObstacle(
                contour=cnt,
                bbox=(bx, by, bw, bh),
                centroid_pixel=(cx, cy),
                area_pixels=area,
                polygon_approx=approx,
            )
            obstacles.append(obs)

            # Draw on annotated image
            cv2.drawContours(annotated, [cnt], -1, (0, 0, 255), 2)  # Red outline
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (255, 100, 0), 1)  # Blue box
            cv2.circle(annotated, (int(round(cx)), int(round(cy))), 4, (0, 255, 0), -1)  # Green centroid

        return DetectionResult(
            binary_mask=clean_mask,
            obstacles=obstacles,
            annotated_image=annotated,
        )

