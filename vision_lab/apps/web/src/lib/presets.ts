import imagePreset from "../../../../presets/image_detection.json";
import videoPreset from "../../../../presets/video_segmentation.json";
import webcamPreset from "../../../../presets/webcam_hsv.json";

import type { PipelineGraph } from "@vision-lab/shared";

export const presetOptions: Array<{ id: string; label: string; graph: PipelineGraph }> = [
  { id: "image_detection", label: "Image Detection", graph: imagePreset as PipelineGraph },
  { id: "video_segmentation", label: "Video Segmentation", graph: videoPreset as PipelineGraph },
  { id: "webcam_hsv", label: "Webcam HSV Threshold", graph: webcamPreset as PipelineGraph },
];
