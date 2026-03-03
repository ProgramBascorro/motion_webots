from __future__ import annotations

from typing import Any

from app.engine.packets import NodeResult


def node_preview_event(session_id: str, frame_id: str, result: NodeResult) -> dict[str, Any]:
    preview_type = "json"
    if result.output_kind in {"image", "mask"}:
        preview_type = "image"
    elif result.output_kind == "text":
        preview_type = "text"
    return {
        "type": "node_preview",
        "session_id": session_id,
        "frame_id": frame_id,
        "node_id": result.node_id,
        "preview_type": preview_type,
        "preview_payload": result.preview,
    }


def node_metrics_event(session_id: str, frame_id: str, result: NodeResult) -> dict[str, Any]:
    return {
        "type": "node_metrics",
        "session_id": session_id,
        "frame_id": frame_id,
        "node_id": result.node_id,
        "latency_ms": result.metrics.latency_ms,
        "fps_estimate": result.metrics.fps_estimate,
        "output_kind": result.output_kind,
    }
