from __future__ import annotations


def fps_from_latency(latency_ms: float) -> float | None:
    if latency_ms <= 0:
        return None
    return 1000.0 / latency_ms
