from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from app.utils.image_codec import encode_image_preview


def build_preview(output_kind: str, payload: Any, preview_max_width: int) -> dict[str, Any] | None:
    if output_kind == "image" and isinstance(payload, np.ndarray):
        return encode_image_preview(payload, max_width=preview_max_width)
    if output_kind == "mask" and isinstance(payload, np.ndarray):
        colored = cv2.applyColorMap(payload, cv2.COLORMAP_JET)
        return encode_image_preview(colored, max_width=preview_max_width, fmt=".png")
    if output_kind in {"detections", "json", "metrics"}:
        return {"kind": "json", "data": payload}
    if output_kind == "text":
        return {"kind": "text", "text": str(payload)}
    return None
