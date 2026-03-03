from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np

from app.inference.mock_models import mock_mask
from app.inference.model_cache import ModelCache


class SegmentationModelRunner:
    def __init__(self, model_cache: ModelCache) -> None:
        self.model_cache = model_cache

    def run(self, image: np.ndarray, params: dict[str, Any]) -> tuple[np.ndarray, float]:
        if params.get("use_mock", False):
            start = time.perf_counter()
            mask = mock_mask(image)
            return mask, (time.perf_counter() - start) * 1000.0

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
        threshold = float(params.get("mask_threshold", 0.5))
        raw = outputs[0]
        if raw.ndim == 4:
            raw = raw[0, 0]
        elif raw.ndim == 3:
            raw = raw[0]
        mask = (raw > threshold).astype(np.uint8) * 255
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
        return mask, latency
