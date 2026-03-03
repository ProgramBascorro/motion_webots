from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def run_resize(image: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    width = int(params.get("width", 640))
    height = int(params.get("height", 640))
    interpolation_name = str(params.get("interpolation", "linear")).lower()
    interpolation = {
        "nearest": cv2.INTER_NEAREST,
        "linear": cv2.INTER_LINEAR,
        "area": cv2.INTER_AREA,
        "cubic": cv2.INTER_CUBIC,
    }.get(interpolation_name, cv2.INTER_LINEAR)
    return cv2.resize(image, (width, height), interpolation=interpolation)


def run_crop(image: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    x = max(0, int(params.get("x", 0)))
    y = max(0, int(params.get("y", 0)))
    width = max(1, int(params.get("width", image.shape[1])))
    height = max(1, int(params.get("height", image.shape[0])))
    return image[y : y + height, x : x + width].copy()


def run_gaussian_blur(image: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    kernel_size = max(1, int(params.get("kernel_size", 5)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    sigma = float(params.get("sigma", 0))
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)


def run_hsv_threshold(image: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [
            int(params.get("lower_h", 0)),
            int(params.get("lower_s", 0)),
            int(params.get("lower_v", 0)),
        ],
        dtype=np.uint8,
    )
    upper = np.array(
        [
            int(params.get("upper_h", 179)),
            int(params.get("upper_s", 255)),
            int(params.get("upper_v", 255)),
        ],
        dtype=np.uint8,
    )
    return cv2.inRange(hsv, lower, upper)
