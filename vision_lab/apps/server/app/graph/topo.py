from __future__ import annotations

from collections import defaultdict, deque

from app.core.models import PipelineGraph


def topological_sort(graph: PipelineGraph) -> list[str]:
    indegree: dict[str, int] = {node.id: 0 for node in graph.nodes}
    adjacency: dict[str, list[str]] = defaultdict(list)

    for edge in graph.edges:
        adjacency[edge.source].append(edge.target)
        indegree[edge.target] += 1

    queue = deque(sorted(node_id for node_id, degree in indegree.items() if degree == 0))
    result: list[str] = []

    while queue:
        node_id = queue.popleft()
        result.append(node_id)
        for target in sorted(adjacency[node_id]):
            indegree[target] -= 1
            if indegree[target] == 0:
                queue.append(target)

    if len(result) != len(graph.nodes):
        raise ValueError("Graph contains a cycle")
    return result
