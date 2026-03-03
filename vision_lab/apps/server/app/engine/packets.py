from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np


@dataclass
class FramePacket:
    frame_id: str
    timestamp_ms: float
    image: np.ndarray
    width: int
    height: int
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class NodeMetrics:
    latency_ms: float
    fps_estimate: float | None = None


@dataclass
class NodeResult:
    node_id: str
    type: str
    success: bool
    output_kind: Literal["image", "detections", "mask", "json", "text", "metrics"]
    payload: Any
    preview: dict[str, Any] | None
    metrics: NodeMetrics
    error: str | None = None
