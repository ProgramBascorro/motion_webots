from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def mock_detections(image: np.ndarray, labels: list[str]) -> list[dict[str, Any]]:
    height, width = image.shape[:2]
    box_w = max(40, width // 5)
    box_h = max(40, height // 5)
    x = max(0, width // 2 - box_w // 2)
    y = max(0, height // 2 - box_h // 2)
    return [
        {
            "box": [x, y, box_w, box_h],
            "class_id": 0,
            "label": labels[0] if labels else "object",
            "score": 0.82,
        }
    ]


def mock_mask(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    center = (width // 2, height // 2)
    axes = (max(20, width // 4), max(20, height // 4))
    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
    return mask
