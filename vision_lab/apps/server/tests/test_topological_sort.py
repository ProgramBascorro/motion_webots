from app.core.models import GraphEdge, GraphNode, PipelineGraph, Position
from app.graph.topo import topological_sort


def test_topological_sort_returns_stable_order() -> None:
    graph = PipelineGraph(
        id="g1",
        name="order",
        nodes=[
            GraphNode(id="cam", type="webcam_input", label="cam", position=Position(x=0, y=0), params={"device_index": 0}),
            GraphNode(id="resize", type="resize", label="resize", position=Position(x=1, y=0), params={}),
            GraphNode(id="metrics", type="metrics", label="metrics", position=Position(x=2, y=0), params={}),
        ],
        edges=[
            GraphEdge(id="e1", source="cam", target="resize"),
            GraphEdge(id="e2", source="resize", target="metrics"),
        ],
    )
    assert topological_sort(graph) == ["cam", "resize", "metrics"]
