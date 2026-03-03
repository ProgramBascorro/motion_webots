from app.nodes.registry import NodeRegistry


def test_registry_contains_required_nodes() -> None:
    registry = NodeRegistry()
    assert registry.get("onnx_detection") is not None
    assert registry.get("draw_detections") is not None
