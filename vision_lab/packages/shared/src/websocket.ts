import type { OutputKind } from "./graph-schema";

export type PreviewPayload =
  | { kind: "image"; imageUrl: string; width: number; height: number }
  | { kind: "json"; data: unknown }
  | { kind: "text"; text: string };

export type WsMessage =
  | { type: "run_started"; session_id: string; graph_id: string; mode: string }
  | {
      type: "run_progress";
      session_id: string;
      frame_id: string;
      frame_index: number;
      total_frames?: number;
      pipeline_fps?: number;
    }
  | {
      type: "node_preview";
      session_id: string;
      frame_id: string;
      node_id: string;
      preview_type: "image" | "mask" | "json" | "text";
      preview_payload: PreviewPayload;
    }
  | {
      type: "node_metrics";
      session_id: string;
      frame_id: string;
      node_id: string;
      latency_ms: number;
      fps_estimate?: number;
      output_kind: OutputKind;
    }
  | {
      type: "node_error";
      session_id: string;
      frame_id?: string;
      node_id?: string;
      error_code: string;
      message: string;
    }
  | {
      type: "run_finished";
      session_id: string;
      total_frames: number;
      total_latency_ms: number;
      avg_fps: number;
      summary: Record<string, unknown>;
    }
  | { type: "run_stopped"; session_id: string; reason: string }
  | { type: "run_log"; session_id: string; level: "info" | "warning" | "error"; message: string };
