from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np

from app.adapters.folder_replay_input import FolderReplayInputAdapter
from app.adapters.image_input import ImageInputAdapter
from app.adapters.video_input import VideoInputAdapter
from app.adapters.webcam_input import WebcamInputAdapter
from app.core.errors import VisionLabError
from app.core.session_manager import SessionState
from app.engine.context import ExecutionContext
from app.engine.events import node_metrics_event, node_preview_event
from app.engine.packets import FramePacket, NodeResult
from app.engine.results import make_result
from app.graph.slots import incoming_edges
from app.graph.topo import topological_sort
from app.nodes.inference_nodes import run_detection, run_segmentation
from app.nodes.postprocess_nodes import (
    run_confidence_filter,
    run_draw_detections,
    run_json_output,
    run_mask_overlay,
    run_metrics,
)
from app.nodes.preprocess_nodes import run_crop, run_gaussian_blur, run_hsv_threshold, run_resize


INPUT_NODE_TYPES = {"image_input", "video_input", "webcam_input", "folder_replay_input"}


class PipelineExecutor:
    def __init__(self, context: ExecutionContext) -> None:
        self.context = context

    async def execute(self, session: SessionState) -> None:
        graph = session.graph
        session.started = True
        try:
            await self.context.sessions.send(
                session.session_id,
                {"type": "run_started", "session_id": session.session_id, "graph_id": graph.id, "mode": session.run_config.mode},
            )
            await self._send_log(session, "info", "Preparing execution graph")

            order = topological_sort(graph)
            node_map = {node.id: node for node in graph.nodes}
            upstream = incoming_edges(graph)
            input_nodes = [node for node in graph.nodes if node.type in INPUT_NODE_TYPES]
            if not input_nodes:
                raise VisionLabError("missing_input_node", "Pipeline requires at least one input node")
            input_node = input_nodes[0]

            await self._send_log(session, "info", f"Opening input source: {input_node.label}")
            adapter = self._build_adapter(input_node.type, input_node.params)
            await self._send_log(session, "info", "Input source ready, waiting for first frame")

            frame_count = 0
            total_latency_ms = 0.0

            for frame in adapter.frames():
                if session.stop_requested:
                    await self.context.sessions.send(
                        session.session_id,
                        {"type": "run_stopped", "session_id": session.session_id, "reason": "Stopped by user"},
                    )
                    break

                frame_start = time.perf_counter()
                frame_count += 1
                if frame_count == 1:
                    await self._send_log(session, "info", "First frame received, executing pipeline")
                frame_results: dict[str, NodeResult] = {}

                for node_id in order:
                    node = node_map[node_id]
                    result = self._execute_node(node.type, node.id, node.params, frame, upstream.get(node.id, []), frame_results, session)
                    frame_results[node_id] = result
                    if result.preview is not None:
                        await self.context.sessions.send(session.session_id, node_preview_event(session.session_id, frame.frame_id, result))
                    await self.context.sessions.send(session.session_id, node_metrics_event(session.session_id, frame.frame_id, result))
                    if not result.success:
                        await self.context.sessions.send(
                            session.session_id,
                            {
                                "type": "node_error",
                                "session_id": session.session_id,
                                "frame_id": frame.frame_id,
                                "node_id": node.id,
                                "error_code": "execution_failure",
                                "message": result.error or "Node execution failed",
                            },
                        )
                        raise VisionLabError("execution_failure", result.error or "Node execution failed")

                frame_latency_ms = (time.perf_counter() - frame_start) * 1000.0
                total_latency_ms += frame_latency_ms
                await self.context.sessions.send(
                    session.session_id,
                    {
                        "type": "run_progress",
                        "session_id": session.session_id,
                        "frame_id": frame.frame_id,
                        "frame_index": frame_count,
                        "pipeline_fps": 1000.0 / frame_latency_ms if frame_latency_ms > 0 else 0,
                    },
                )

                if session.run_config.max_frames and frame_count >= session.run_config.max_frames:
                    break

            if not session.stop_requested:
                if frame_count == 0:
                    await self._send_log(session, "warning", "Input source produced no frames")
                summary = {
                    "frames": frame_count,
                    "avg_latency_ms": total_latency_ms / frame_count if frame_count else 0.0,
                }
                session.summary = summary
                await self.context.sessions.send(
                    session.session_id,
                    {
                        "type": "run_finished",
                        "session_id": session.session_id,
                        "total_frames": frame_count,
                        "total_latency_ms": total_latency_ms,
                        "avg_fps": (frame_count * 1000.0 / total_latency_ms) if total_latency_ms > 0 else 0.0,
                        "summary": summary,
                    },
                )
        except VisionLabError as exc:
            await self.context.sessions.send(
                session.session_id,
                {
                    "type": "node_error",
                    "session_id": session.session_id,
                    "error_code": exc.code,
                    "message": exc.message,
                },
            )
            await self._send_log(session, "error", exc.message)
        except Exception as exc:
            await self.context.sessions.send(
                session.session_id,
                {
                    "type": "node_error",
                    "session_id": session.session_id,
                    "error_code": "execution_failure",
                    "message": str(exc),
                },
            )
            await self._send_log(session, "error", f"Execution failed: {exc}")
        finally:
            if "adapter" in locals():
                adapter.close()
            session.finished = True

    async def _send_log(self, session: SessionState, level: str, message: str) -> None:
        await self.context.sessions.send(
            session.session_id,
            {
                "type": "run_log",
                "session_id": session.session_id,
                "level": level,
                "message": message,
            },
        )

    def _build_adapter(self, node_type: str, params: dict[str, Any]):
        if node_type == "image_input":
            path = self._resolve_required_file(params.get("file_id"))
            return ImageInputAdapter(path)
        if node_type == "video_input":
            path = self._resolve_required_file(params.get("file_id"))
            return VideoInputAdapter(path, loop=bool(params.get("loop", False)))
        if node_type == "webcam_input":
            return WebcamInputAdapter(
                device_index=int(params.get("device_index", 0)),
                width=int(params.get("width", 1280)),
                height=int(params.get("height", 720)),
            )
        if node_type == "folder_replay_input":
            file_ids = [str(item) for item in params.get("file_ids", [])]
            paths = [self._resolve_required_file(file_id) for file_id in file_ids]
            return FolderReplayInputAdapter(paths)
        raise VisionLabError("unsupported_input", f"Unsupported input node: {node_type}")

    def _resolve_required_file(self, file_id: Any) -> str:
        if not file_id:
            raise VisionLabError("missing_upload", "Input file is missing")
        path = self.context.file_store.resolve(str(file_id))
        if path is None:
            raise VisionLabError("bad_uploaded_file", f"Could not resolve uploaded file {file_id}")
        return str(path)

    def _execute_node(
        self,
        node_type: str,
        node_id: str,
        params: dict[str, Any],
        frame: FramePacket,
        upstream_ids: list[str],
        frame_results: dict[str, NodeResult],
        session: SessionState,
    ) -> NodeResult:
        start = time.perf_counter()
        preview_width = session.run_config.preview_max_width
        try:
            if node_type in INPUT_NODE_TYPES:
                preview = self.context.make_preview("image", frame.image, preview_width)
                return make_result(node_id, node_type, "image", frame.image.copy(), preview, (time.perf_counter() - start) * 1000.0)

            inputs = [frame_results[source_id] for source_id in upstream_ids if source_id in frame_results]
            image_input = self._resolve_image_input(frame, inputs)

            if node_type == "resize":
                output = run_resize(image_input, params)
                return self._image_result(node_id, node_type, output, start, preview_width)
            if node_type == "crop":
                output = run_crop(image_input, params)
                return self._image_result(node_id, node_type, output, start, preview_width)
            if node_type == "gaussian_blur":
                output = run_gaussian_blur(image_input, params)
                return self._image_result(node_id, node_type, output, start, preview_width)
            if node_type == "hsv_threshold":
                output = run_hsv_threshold(image_input, params)
                preview = self.context.make_preview("mask", output, preview_width)
                return make_result(node_id, node_type, "mask", output, preview, (time.perf_counter() - start) * 1000.0)
            if node_type == "onnx_detection":
                detections, latency_ms = run_detection(image_input, params, self.context.detection_runner)
                preview = self.context.make_preview("detections", detections, preview_width)
                return make_result(node_id, node_type, "detections", detections, preview, latency_ms)
            if node_type == "onnx_segmentation":
                mask, latency_ms = run_segmentation(image_input, params, self.context.segmentation_runner)
                preview = self.context.make_preview("mask", mask, preview_width)
                return make_result(node_id, node_type, "mask", mask, preview, latency_ms)
            if node_type == "confidence_filter":
                detections = self._find_payload(inputs, "detections", [])
                filtered = run_confidence_filter(detections, params)
                preview = self.context.make_preview("detections", filtered, preview_width)
                return make_result(node_id, node_type, "detections", filtered, preview, (time.perf_counter() - start) * 1000.0)
            if node_type == "draw_detections":
                detections = self._find_payload(inputs, "detections", [])
                output = run_draw_detections(image_input, detections, params)
                return self._image_result(node_id, node_type, output, start, preview_width)
            if node_type == "mask_overlay":
                mask = self._find_payload(inputs, "mask", np.zeros(image_input.shape[:2], dtype=np.uint8))
                output = run_mask_overlay(image_input, mask, params)
                return self._image_result(node_id, node_type, output, start, preview_width)
            if node_type == "json_output":
                payload = inputs[0].payload if inputs else {}
                output = run_json_output(payload, params)
                preview = self.context.make_preview("json", output, preview_width)
                return make_result(node_id, node_type, "json", output, preview, (time.perf_counter() - start) * 1000.0)
            if node_type == "metrics":
                payload = inputs[0].payload if inputs else {"frame_id": frame.frame_id}
                output = run_metrics(payload, params)
                preview = self.context.make_preview("metrics", output, preview_width)
                return make_result(node_id, node_type, "metrics", output, preview, (time.perf_counter() - start) * 1000.0)
            raise VisionLabError("unsupported_node", f"Unsupported node {node_type}")
        except Exception as exc:
            return make_result(
                node_id,
                node_type,
                "text",
                None,
                {"kind": "text", "text": str(exc)},
                (time.perf_counter() - start) * 1000.0,
                success=False,
                error=str(exc),
            )

    def _resolve_image_input(self, frame: FramePacket, inputs: list[NodeResult]) -> np.ndarray:
        for result in inputs:
            if result.output_kind == "image":
                return result.payload
        for result in inputs:
            if result.output_kind == "mask":
                return cv2.cvtColor(result.payload, cv2.COLOR_GRAY2BGR)
        return frame.image

    def _find_payload(self, inputs: list[NodeResult], output_kind: str, default: Any):
        for result in inputs:
            if result.output_kind == output_kind:
                return result.payload
        return default

    def _image_result(self, node_id: str, node_type: str, image: np.ndarray, start: float, preview_width: int) -> NodeResult:
        preview = self.context.make_preview("image", image, preview_width)
        latency = (time.perf_counter() - start) * 1000.0
        return make_result(node_id, node_type, "image", image, preview, latency)
