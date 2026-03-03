import type {
  PipelineGraph,
  ValidationIssue,
  WsMessage,
} from "@vision-lab/shared";
import type { NodeTypeDefinition } from "@vision-lab/shared";
import type { UploadEntry } from "@/types/ui";

const apiBase = process.env.NEXT_PUBLIC_API_BASE_URL ?? "http://localhost:8000";

export const API_BASE = apiBase;

export async function fetchNodeTypes(): Promise<NodeTypeDefinition[]> {
  const response = await fetch(`${apiBase}/api/node-types`);
  if (!response.ok) {
    throw new Error("Failed to load node types");
  }
  const raw = (await response.json()) as Array<Record<string, unknown>>;
  return raw.map((item) => ({
    type: String(item.type),
    displayName: String(item.display_name),
    category: String(item.category) as NodeTypeDefinition["category"],
    acceptedInputKinds: (item.accepted_input_kinds as string[]) ?? [],
    producedOutputKinds: (item.produced_output_kinds as string[]) ?? [],
    paramsSchema: (item.params_schema as NodeTypeDefinition["paramsSchema"]) ?? [],
    previewable: Boolean(item.previewable),
    maxInputs: Number(item.max_inputs ?? 1),
  }));
}

export async function validateGraph(graph: PipelineGraph): Promise<{ valid: boolean; errors: ValidationIssue[]; topoOrder?: string[] }> {
  const response = await fetch(`${apiBase}/api/graphs/validate`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(graph),
  });
  const payload = await response.json();
  return {
    valid: Boolean(payload.valid),
    errors: (payload.errors ?? []) as ValidationIssue[],
    topoOrder: payload.topo_order as string[] | undefined,
  };
}

export async function uploadFiles(files: File[]): Promise<UploadEntry[]> {
  const form = new FormData();
  files.forEach((file) => form.append("files", file));
  const response = await fetch(`${apiBase}/api/upload`, {
    method: "POST",
    body: form,
  });
  if (!response.ok) {
    throw new Error("Upload failed");
  }
  const payload = (await response.json()) as {
    files: Array<{
      file_id: string;
      filename: string;
      media_type?: string;
      path: string;
      size_bytes: number;
    }>;
  };
  return payload.files.map((item) => ({
    fileId: item.file_id,
    filename: item.filename,
    mediaType: item.media_type,
    path: item.path,
    sizeBytes: item.size_bytes,
  }));
}

export async function startRun(graph: PipelineGraph, runConfig: Record<string, unknown>) {
  const response = await fetch(`${apiBase}/api/run/start`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ graph, run_config: runConfig }),
  });
  const payload = await response.json();
  if (!response.ok) {
    const detail = payload?.detail;
    const firstError = detail?.errors?.[0]?.message ?? "Run start failed";
    throw new Error(firstError);
  }
  return payload as { session_id: string; websocket_url: string; accepted: boolean };
}

export async function stopRun(sessionId: string): Promise<void> {
  await fetch(`${apiBase}/api/run/stop`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ session_id: sessionId }),
  });
}

export function buildWebSocketUrl(path: string): string {
  const url = new URL(apiBase);
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
  url.pathname = path;
  return url.toString();
}

export type { WsMessage };
