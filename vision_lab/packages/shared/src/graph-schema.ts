export type OutputKind = "frame" | "image" | "detections" | "mask" | "json" | "text" | "metrics";

export type NodeTypeId =
  | "image_input"
  | "video_input"
  | "webcam_input"
  | "folder_replay_input"
  | "resize"
  | "crop"
  | "gaussian_blur"
  | "hsv_threshold"
  | "onnx_detection"
  | "onnx_segmentation"
  | "confidence_filter"
  | "draw_detections"
  | "mask_overlay"
  | "json_output"
  | "metrics";

export type GraphNode = {
  id: string;
  type: NodeTypeId;
  label: string;
  position: {
    x: number;
    y: number;
  };
  params: Record<string, unknown>;
};

export type GraphEdge = {
  id: string;
  source: string;
  target: string;
};

export type PipelineGraph = {
  id: string;
  name: string;
  version: "1.0";
  nodes: GraphNode[];
  edges: GraphEdge[];
};

export type ValidationIssue = {
  code: string;
  message: string;
  nodeId?: string;
  edgeId?: string;
};
