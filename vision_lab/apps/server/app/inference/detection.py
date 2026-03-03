from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np

from app.inference.mock_models import mock_detections
from app.inference.model_cache import ModelCache


class DetectionModelRunner:
    def __init__(self, model_cache: ModelCache) -> None:
        self.model_cache = model_cache

    def run(self, image: np.ndarray, params: dict[str, Any]) -> tuple[list[dict[str, Any]], float]:
        labels = [str(label) for label in params.get("labels", ["object"])]
        if params.get("use_mock", False):
            start = time.perf_counter()
            detections = mock_detections(image, labels)
            return detections, (time.perf_counter() - start) * 1000.0

        model_path = str(params.get("model_path", "")).strip()
        if not model_path:
            raise FileNotFoundError("missing model_path")

        session = self.model_cache.get_session(model_path)
        input_name = session.get_inputs()[0].name
        input_width = int(params.get("input_width", 640))
        input_height = int(params.get("input_height", 640))
        resized = cv2.resize(image, (input_width, input_height))
        blob = resized.astype(np.float32) / 255.0
        blob = np.transpose(blob[:, :, ::-1], (2, 0, 1))[None, ...]

        start = time.perf_counter()
        outputs = session.run(None, {input_name: blob})
        latency = (time.perf_counter() - start) * 1000.0
        detections = self._decode(outputs[0], image.shape[1], image.shape[0], labels, params)
        return detections, latency

    def _decode(
        self,
        raw_output: np.ndarray,
        width: int,
        height: int,
        labels: list[str],
        params: dict[str, Any],
    ) -> list[dict[str, Any]]:
        array = raw_output[0] if raw_output.ndim == 3 else raw_output
        if array.ndim == 2 and array.shape[0] < array.shape[1]:
            rows = array.T
        else:
            rows = array

        confidence_threshold = float(params.get("confidence_threshold", 0.25))
        nms_threshold = float(params.get("nms_threshold", 0.45))
        boxes: list[list[int]] = []
        scores: list[float] = []
        items: list[dict[str, Any]] = []

        for row in rows:
            if len(row) < 6:
                continue
            cx, cy, w, h = row[0:4]
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if score < confidence_threshold:
                continue
            x = int(max(0, min(width - 1, cx - w / 2)))
            y = int(max(0, min(height - 1, cy - h / 2)))
            bw = int(max(1, min(width - x, w)))
            bh = int(max(1, min(height - y, h)))
            boxes.append([x, y, bw, bh])
            scores.append(score)
            items.append(
                {
                    "box": [x, y, bw, bh],
                    "class_id": class_id,
                    "label": labels[class_id] if class_id < len(labels) else f"class_{class_id}",
                    "score": score,
                }
            )

        if not items:
            return []
        kept = cv2.dnn.NMSBoxes(boxes, scores, confidence_threshold, nms_threshold)
        if len(kept) == 0:
            return []
        indices = [int(index) for index in np.array(kept).flatten().tolist()]
        return [items[index] for index in indices]
