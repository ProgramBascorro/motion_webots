from __future__ import annotations

from collections import defaultdict

from app.core.models import PipelineGraph


def incoming_edges(graph: PipelineGraph) -> dict[str, list[str]]:
    result: dict[str, list[str]] = defaultdict(list)
    for edge in graph.edges:
        result[edge.target].append(edge.source)
    return result
