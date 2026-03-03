from __future__ import annotations

from collections.abc import Iterable

from app.nodes.base import NodeTypeDefinition


def _field(key: str, label: str, field_type: str, **kwargs):
    return {"key": key, "label": label, "type": field_type, **kwargs}


NODE_DEFINITIONS: list[NodeTypeDefinition] = [
    NodeTypeDefinition(
        type="image_input",
        display_name="Image / MP4 Input",
        category="input",
        accepted_input_kinds=[],
        produced_output_kinds=["frame"],
        params_schema=[
            _field(
                "file_id",
                "Uploaded Image Or MP4",
                "text",
                required=True,
                helperText="Accepts single-image uploads and uploaded MP4/video assets.",
            )
        ],
        previewable=True,
        max_inputs=0,
    ),
    NodeTypeDefinition(
        type="video_input",
        display_name="Video Input",
        category="input",
        accepted_input_kinds=[],
        produced_output_kinds=["frame"],
        params_schema=[
            _field("file_id", "Uploaded Video", "text", required=True),
            _field("loop", "Loop", "boolean", defaultValue=False),
        ],
        previewable=True,
        max_inputs=0,
    ),
    NodeTypeDefinition(
        type="webcam_input",
        display_name="Webcam Input",
        category="input",
        accepted_input_kinds=[],
        produced_output_kinds=["frame"],
        params_schema=[
            _field("device_index", "Device Index", "number", defaultValue=0),
            _field("width", "Width", "number", defaultValue=1280),
            _field("height", "Height", "number", defaultValue=720),
        ],
        previewable=True,
        max_inputs=0,
    ),
    NodeTypeDefinition(
        type="folder_replay_input",
        display_name="Folder Replay",
        category="input",
        accepted_input_kinds=[],
        produced_output_kinds=["frame"],
        params_schema=[_field("file_ids", "Image File IDs", "string_array", defaultValue=[])],
        previewable=True,
        max_inputs=0,
    ),
    NodeTypeDefinition(
        type="resize",
        display_name="Resize",
        category="preprocess",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["image"],
        params_schema=[
            _field("width", "Width", "number", defaultValue=640),
            _field("height", "Height", "number", defaultValue=640),
            _field(
                "interpolation",
                "Interpolation",
                "select",
                defaultValue="linear",
                options=[
                    {"label": "Nearest", "value": "nearest"},
                    {"label": "Linear", "value": "linear"},
                    {"label": "Area", "value": "area"},
                    {"label": "Cubic", "value": "cubic"},
                ],
            ),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="crop",
        display_name="Crop",
        category="preprocess",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["image"],
        params_schema=[
            _field("x", "X", "number", defaultValue=0),
            _field("y", "Y", "number", defaultValue=0),
            _field("width", "Width", "number", defaultValue=320),
            _field("height", "Height", "number", defaultValue=320),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="gaussian_blur",
        display_name="Gaussian Blur",
        category="preprocess",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["image"],
        params_schema=[
            _field("kernel_size", "Kernel Size", "number", defaultValue=5),
            _field("sigma", "Sigma", "number", defaultValue=0),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="hsv_threshold",
        display_name="HSV Threshold",
        category="preprocess",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["mask"],
        params_schema=[
            _field("lower_h", "Lower H", "number", defaultValue=25),
            _field("lower_s", "Lower S", "number", defaultValue=40),
            _field("lower_v", "Lower V", "number", defaultValue=40),
            _field("upper_h", "Upper H", "number", defaultValue=95),
            _field("upper_s", "Upper S", "number", defaultValue=255),
            _field("upper_v", "Upper V", "number", defaultValue=255),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="onnx_detection",
        display_name="ONNX Detection",
        category="inference",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["detections"],
        params_schema=[
            _field("model_path", "Model Path", "text", defaultValue=""),
            _field("input_width", "Input Width", "number", defaultValue=640),
            _field("input_height", "Input Height", "number", defaultValue=640),
            _field("confidence_threshold", "Confidence", "number", defaultValue=0.25),
            _field("nms_threshold", "NMS Threshold", "number", defaultValue=0.45),
            _field("labels", "Labels", "string_array", defaultValue=["ball", "goal post", "robot"]),
            _field("use_mock", "Use Mock", "boolean", defaultValue=True),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="onnx_segmentation",
        display_name="ONNX Segmentation",
        category="inference",
        accepted_input_kinds=["frame", "image"],
        produced_output_kinds=["mask"],
        params_schema=[
            _field("model_path", "Model Path", "text", defaultValue=""),
            _field("input_width", "Input Width", "number", defaultValue=640),
            _field("input_height", "Input Height", "number", defaultValue=640),
            _field("mask_threshold", "Mask Threshold", "number", defaultValue=0.5),
            _field("palette", "Palette", "int_array", defaultValue=[0, 255, 0]),
            _field("use_mock", "Use Mock", "boolean", defaultValue=True),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="confidence_filter",
        display_name="Confidence Filter",
        category="postprocess",
        accepted_input_kinds=["detections"],
        produced_output_kinds=["detections"],
        params_schema=[
            _field("min_confidence", "Min Confidence", "number", defaultValue=0.5),
            _field("allowed_class_ids", "Allowed Class IDs", "int_array", defaultValue=[]),
        ],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="draw_detections",
        display_name="Draw Detections",
        category="postprocess",
        accepted_input_kinds=["frame", "image", "detections"],
        produced_output_kinds=["image"],
        params_schema=[
            _field("line_thickness", "Line Thickness", "number", defaultValue=2),
            _field("font_scale", "Font Scale", "number", defaultValue=0.55),
            _field("show_labels", "Show Labels", "boolean", defaultValue=True),
        ],
        previewable=True,
        max_inputs=2,
    ),
    NodeTypeDefinition(
        type="mask_overlay",
        display_name="Mask Overlay",
        category="postprocess",
        accepted_input_kinds=["frame", "image", "mask"],
        produced_output_kinds=["image"],
        params_schema=[
            _field("alpha", "Alpha", "number", defaultValue=0.45),
            _field(
                "color_map",
                "Color Map",
                "select",
                defaultValue="green",
                options=[
                    {"label": "Green", "value": "green"},
                    {"label": "Blue", "value": "blue"},
                    {"label": "Red", "value": "red"},
                    {"label": "Yellow", "value": "yellow"},
                ],
            ),
        ],
        previewable=True,
        max_inputs=2,
    ),
    NodeTypeDefinition(
        type="json_output",
        display_name="JSON Output",
        category="postprocess",
        accepted_input_kinds=["detections", "mask", "json", "metrics", "text", "image", "frame"],
        produced_output_kinds=["json"],
        params_schema=[_field("include_metrics", "Include Metrics", "boolean", defaultValue=True)],
        previewable=True,
        max_inputs=1,
    ),
    NodeTypeDefinition(
        type="metrics",
        display_name="Metrics",
        category="postprocess",
        accepted_input_kinds=["frame", "image", "detections", "mask", "json", "text", "metrics"],
        produced_output_kinds=["metrics"],
        params_schema=[_field("window_size", "Window Size", "number", defaultValue=30)],
        previewable=True,
        max_inputs=1,
    ),
]


class NodeRegistry:
    def __init__(self, definitions: Iterable[NodeTypeDefinition] | None = None) -> None:
        items = list(definitions or NODE_DEFINITIONS)
        self._by_type = {definition.type: definition for definition in items}

    def get(self, node_type: str) -> NodeTypeDefinition | None:
        return self._by_type.get(node_type)

    def require(self, node_type: str) -> NodeTypeDefinition:
        definition = self.get(node_type)
        if definition is None:
            raise KeyError(node_type)
        return definition

    def list(self) -> list[NodeTypeDefinition]:
        return list(self._by_type.values())
