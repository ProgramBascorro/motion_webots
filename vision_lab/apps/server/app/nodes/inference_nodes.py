from __future__ import annotations

from typing import Any

import numpy as np

from app.inference.detection import DetectionModelRunner
from app.inference.segmentation import SegmentationModelRunner


def run_detection(
    image: np.ndarray,
    params: dict[str, Any],
    detection_runner: DetectionModelRunner,
) -> tuple[list[dict[str, Any]], float]:
    return detection_runner.run(image, params)


def run_segmentation(
    image: np.ndarray,
    params: dict[str, Any],
    segmentation_runner: SegmentationModelRunner,
) -> tuple[np.ndarray, float]:
    return segmentation_runner.run(image, params)
