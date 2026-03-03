from __future__ import annotations

from typing import Any

from app.core.metrics import fps_from_latency
from app.engine.packets import NodeMetrics, NodeResult


def make_result(
    node_id: str,
    node_type: str,
    output_kind: str,
    payload: Any,
    preview: dict[str, Any] | None,
    latency_ms: float,
    success: bool = True,
    error: str | None = None,
) -> NodeResult:
    return NodeResult(
        node_id=node_id,
        type=node_type,
        success=success,
        output_kind=output_kind,  # type: ignore[arg-type]
        payload=payload,
        preview=preview,
        metrics=NodeMetrics(latency_ms=latency_ms, fps_estimate=fps_from_latency(latency_ms)),
        error=error,
    )
