"""Vision-based obstacle perception and grid mapping subpackage."""

from autonomous_nav.perception.preprocessing import (
    PreprocessingResult,
    validate_image,
    to_grayscale,
    apply_blur,
    normalize_intensity,
    preprocess_image,
)
from autonomous_nav.perception.vision import (
    CalibratedHomographyCamera,
    OrthographicCamera,
    PlanarPerspectiveCamera,
    SyntheticImageRenderer,
)
from autonomous_nav.perception.obstacle_detection import (
    DetectedObstacle,
    DetectionResult,
    ObstacleDetector,
)
from autonomous_nav.perception.map_adapter import MapAdapter

__all__ = [
    "PreprocessingResult",
    "validate_image",
    "to_grayscale",
    "apply_blur",
    "normalize_intensity",
    "preprocess_image",
    "CalibratedHomographyCamera",
    "OrthographicCamera",
    "PlanarPerspectiveCamera",
    "SyntheticImageRenderer",
    "DetectedObstacle",
    "DetectionResult",
    "ObstacleDetector",
    "MapAdapter",
]
