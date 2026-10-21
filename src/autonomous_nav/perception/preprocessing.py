"""Image preprocessing and input validation for vision-based perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import cv2
import numpy as np


@dataclass(frozen=True)
class PreprocessingResult:
    """Container for the output of each image preprocessing stage."""
    original: np.ndarray
    grayscale: np.ndarray
    blurred: np.ndarray
    normalized: np.ndarray


def validate_image(image: Any) -> None:
    """Validate that an input object is a non-empty, properly formatted uint8 image array.

    Raises:
        TypeError: If image is not a numpy ndarray or has unsupported dtype.
        ValueError: If image is empty or has invalid spatial dimensions.
    """
    if image is None:
        raise TypeError("Input image cannot be None")

    if not isinstance(image, np.ndarray):
        raise TypeError(f"Input image must be a numpy.ndarray, got {type(image).__name__}")

    if image.size == 0:
        raise ValueError("Input image array is empty (size 0)")

    if image.dtype != np.uint8:
        raise TypeError(
            f"Input image must have dtype uint8, got {image.dtype}. "
            "Convert and scale your data to uint8 in range [0, 255]."
        )

    if image.ndim == 2:
        # Grayscale image (H, W)
        h, w = image.shape
        if h < 2 or w < 2:
            raise ValueError(f"Image dimensions must be at least 2x2, got {h}x{w}")
    elif image.ndim == 3:
        # Multi-channel image (H, W, C)
        h, w, c = image.shape
        if h < 2 or w < 2:
            raise ValueError(f"Image dimensions must be at least 2x2, got {h}x{w}")
        if c not in (1, 3, 4):
            raise ValueError(f"Unsupported channel count {c}. Expected 1 (gray), 3 (RGB/BGR), or 4 (RGBA/BGRA)")
    else:
        raise ValueError(f"Unsupported image shape {image.shape}. Expected 2D or 3D array")


def to_grayscale(image: np.ndarray, color_format: str = "RGB") -> np.ndarray:
    """Convert input image to single-channel 8-bit grayscale."""
    validate_image(image)

    if image.ndim == 2:
        return image.copy()

    h, w, c = image.shape
    if c == 1:
        return image[:, :, 0].copy()

    fmt = color_format.upper()
    if fmt == "RGB":
        if c == 3:
            return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        elif c == 4:
            return cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    elif fmt == "BGR":
        if c == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif c == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    else:
        raise ValueError(f"Unsupported color format '{color_format}'. Expected 'RGB' or 'BGR'")

    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def apply_blur(image: np.ndarray, kernel_size: int = 5, sigma: float = 1.0) -> np.ndarray:
    """Apply Gaussian noise reduction filter to 2D image.

    Args:
        image: Single-channel grayscale image.
        kernel_size: Positive odd integer specifying kernel dimensions.
        sigma: Standard deviation for Gaussian kernel.
    """
    if kernel_size <= 0:
        return image.copy()

    if kernel_size % 2 == 0:
        kernel_size += 1

    return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigmaX=float(sigma))


def normalize_intensity(image: np.ndarray) -> np.ndarray:
    """Normalize image dynamic range using min-max contrast stretching."""
    if image.ndim != 2:
        raise ValueError(f"Normalization expects 2D grayscale image, got shape {image.shape}")

    min_val, max_val = float(np.min(image)), float(np.max(image))
    if max_val - min_val < 1e-5:
        # Uniform image; return original copy
        return image.copy()

    normalized = ((image.astype(np.float32) - min_val) / (max_val - min_val) * 255.0)
    return np.clip(normalized, 0, 255).astype(np.uint8)


def preprocess_image(
    image: np.ndarray,
    color_format: str = "RGB",
    blur_kernel_size: int = 5,
    blur_sigma: float = 1.0,
    normalize: bool = False,
) -> PreprocessingResult:
    """Execute complete deterministic image preprocessing pipeline.

    Stages:
        1. Input structure and dtype validation.
        2. Color-to-grayscale conversion.
        3. Gaussian noise filtering.
        4. Optional intensity contrast normalization.
    """
    validate_image(image)

    gray = to_grayscale(image, color_format=color_format)
    blurred = apply_blur(gray, kernel_size=blur_kernel_size, sigma=blur_sigma)
    normalized = normalize_intensity(blurred) if normalize else blurred.copy()

    return PreprocessingResult(
        original=image.copy(),
        grayscale=gray,
        blurred=blurred,
        normalized=normalized,
    )

