from __future__ import annotations

import base64
from typing import Any

import cv2
import numpy as np


def ensure_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
      return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
      return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def resize_for_preview(image: np.ndarray, max_width: int) -> np.ndarray:
    image = ensure_bgr(image)
    if image.shape[1] <= max_width:
        return image
    scale = max_width / image.shape[1]
    height = max(1, int(image.shape[0] * scale))
    return cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)


def encode_image_preview(image: np.ndarray, max_width: int = 480, fmt: str = ".jpg") -> dict[str, Any]:
    preview = resize_for_preview(image, max_width=max_width)
    ok, encoded = cv2.imencode(fmt, preview)
    if not ok:
        raise ValueError("Failed to encode preview image")
    mime = "image/png" if fmt == ".png" else "image/jpeg"
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return {
        "kind": "image",
        "imageUrl": f"data:{mime};base64,{payload}",
        "width": int(preview.shape[1]),
        "height": int(preview.shape[0]),
    }
