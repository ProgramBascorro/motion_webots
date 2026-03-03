from __future__ import annotations

from app.core.models import PipelineGraph


GRAPH_VERSION = "1.0"


def ensure_graph_version(graph: PipelineGraph) -> bool:
    return graph.version == GRAPH_VERSION
