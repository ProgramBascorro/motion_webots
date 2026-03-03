import type { NodeTypeDefinition, PreviewPayload } from "@vision-lab/shared";

export type RunState = "idle" | "starting" | "running" | "stopping" | "finished" | "error";

export type PreviewEntry = {
  nodeId: string;
  previewType: "image" | "mask" | "json" | "text";
  previewPayload: PreviewPayload | null;
  frameId?: string;
};

export type MetricEntry = {
  nodeId: string;
  latencyMs: number;
  fpsEstimate?: number;
  outputKind: string;
};

export type UploadEntry = {
  fileId: string;
  filename: string;
  mediaType?: string;
  path: string;
  sizeBytes: number;
};

export type NodeCatalog = Record<string, NodeTypeDefinition>;
