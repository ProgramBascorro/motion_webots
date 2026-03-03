from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def run_confidence_filter(
    detections: list[dict[str, Any]],
    params: dict[str, Any],
) -> list[dict[str, Any]]:
    min_confidence = float(params.get("min_confidence", 0.5))
    allowed_class_ids = params.get("allowed_class_ids", [])
    allowed_set = {int(item) for item in allowed_class_ids} if allowed_class_ids else None
    filtered: list[dict[str, Any]] = []
    for detection in detections:
        if float(detection.get("score", 0.0)) < min_confidence:
            continue
        class_id = int(detection.get("class_id", -1))
        if allowed_set is not None and class_id not in allowed_set:
            continue
        filtered.append(detection)
    return filtered


def run_draw_detections(
    image: np.ndarray,
    detections: list[dict[str, Any]],
    params: dict[str, Any],
) -> np.ndarray:
    output = image.copy()
    thickness = max(1, int(params.get("line_thickness", 2)))
    font_scale = float(params.get("font_scale", 0.55))
    show_labels = bool(params.get("show_labels", True))
    for detection in detections:
        x, y, width, height = [int(value) for value in detection.get("box", [0, 0, 1, 1])]
        cv2.rectangle(output, (x, y), (x + width, y + height), (0, 255, 255), thickness)
        if show_labels:
            label = f"{detection.get('label', 'object')} {float(detection.get('score', 0.0)):.2f}"
            cv2.putText(
                output,
                label,
                (x, max(20, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (0, 255, 255),
                2,
                lineType=cv2.LINE_AA,
            )
    return output


def run_mask_overlay(image: np.ndarray, mask: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    output = image.copy()
    alpha = float(params.get("alpha", 0.45))
    color_map = str(params.get("color_map", "green")).lower()
    color = {
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
        "red": (0, 0, 255),
        "yellow": (0, 255, 255),
    }.get(color_map, (0, 255, 0))
    color_image = np.zeros_like(output)
    color_image[:] = color
    binary_mask = mask > 0
    output[binary_mask] = cv2.addWeighted(output, 1.0 - alpha, color_image, alpha, 0.0)[binary_mask]
    return output


def run_json_output(payload: Any, params: dict[str, Any]) -> dict[str, Any]:
    include_metrics = bool(params.get("include_metrics", True))
    return {"include_metrics": include_metrics, "data": payload}


def run_metrics(payload: Any, params: dict[str, Any]) -> dict[str, Any]:
    return {
        "window_size": int(params.get("window_size", 30)),
        "summary": payload if isinstance(payload, dict) else {"value": payload},
    }
