from app.core.models import GraphEdge, GraphNode, PipelineGraph, Position
from app.graph.validator import validate_graph
from app.nodes.registry import NodeRegistry


def test_cycle_is_rejected() -> None:
    graph = PipelineGraph(
        id="g1",
        name="cycle",
        nodes=[
            GraphNode(id="a", type="webcam_input", label="A", position=Position(x=0, y=0), params={"device_index": 0}),
            GraphNode(id="b", type="resize", label="B", position=Position(x=1, y=0), params={"width": 10, "height": 10}),
        ],
        edges=[
            GraphEdge(id="e1", source="a", target="b"),
            GraphEdge(id="e2", source="b", target="a"),
        ],
    )

    result = validate_graph(graph, NodeRegistry())
    assert not result.valid
    assert any(issue.code in {"input_has_incoming", "invalid_dag"} for issue in result.errors)
