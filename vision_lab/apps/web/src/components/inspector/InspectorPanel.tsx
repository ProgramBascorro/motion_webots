"use client";

import { useMemo, useState } from "react";

import type { ParameterField } from "@vision-lab/shared";

import { uploadFiles } from "@/lib/api";
import { useEditorStore } from "@/store/editor-store";
import type { UploadEntry } from "@/types/ui";

function renderStringArrayValue(value: unknown): string {
  return Array.isArray(value) ? value.join(", ") : "";
}

function isFileBackedField(field: ParameterField): boolean {
  return field.key === "file_id" || field.key === "file_ids" || field.key === "model_path";
}

function getFieldAccept(field: ParameterField, nodeType: string): string {
  if (field.key === "model_path") {
    return ".onnx,application/octet-stream";
  }
  if (field.key === "file_ids") {
    return "image/*";
  }
  if (nodeType === "video_input") {
    return "video/*";
  }
  if (nodeType === "image_input") {
    return "image/*,video/*";
  }
  return "*/*";
}

function getEligibleUploads(field: ParameterField, nodeType: string, uploads: UploadEntry[]): UploadEntry[] {
  const hasMediaPrefix = (item: UploadEntry, prefix: string) => (item.mediaType ?? "").startsWith(prefix);

  if (field.key === "model_path") {
    return uploads.filter((item) => item.filename.toLowerCase().endsWith(".onnx"));
  }
  if (field.key === "file_ids") {
    return uploads.filter((item) => hasMediaPrefix(item, "image/"));
  }
  if (nodeType === "video_input") {
    return uploads.filter((item) => hasMediaPrefix(item, "video/"));
  }
  if (nodeType === "image_input") {
    return uploads.filter((item) => hasMediaPrefix(item, "image/") || hasMediaPrefix(item, "video/"));
  }
  return uploads;
}

export function InspectorPanel() {
  const [uploadingField, setUploadingField] = useState<string | null>(null);
  const selectedNodeId = useEditorStore((state) => state.selectedNodeId);
  const graph = useEditorStore((state) => state.graph);
  const nodeTypes = useEditorStore((state) => state.nodeTypes);
  const updateNodeParams = useEditorStore((state) => state.updateNodeParams);
  const uploads = useEditorStore((state) => state.uploads);
  const addUploads = useEditorStore((state) => state.addUploads);
  const addLog = useEditorStore((state) => state.addLog);

  const selectedNode = graph.nodes.find((node) => node.id === selectedNodeId) ?? null;
  const nodeType = nodeTypes.find((item) => item.type === selectedNode?.type) ?? null;
  const uploadEntries = useMemo(() => Object.values(uploads), [uploads]);

  if (!selectedNode || !nodeType) {
    return (
      <aside className="panel rounded-3xl p-5">
        <div className="text-xs uppercase tracking-[0.2em] text-slate-400">Inspector</div>
        <div className="mt-6 text-sm text-slate-300">Select a node to edit its parameters and inspect live output.</div>
      </aside>
    );
  }

  const onFieldChange = (field: ParameterField, rawValue: string | boolean) => {
    let value: unknown = rawValue;
    if (field.type === "number") {
      value = Number(rawValue);
    } else if (field.type === "boolean") {
      value = Boolean(rawValue);
    } else if (field.type === "string_array" || field.type === "int_array") {
      const items = String(rawValue)
        .split(",")
        .map((item) => item.trim())
        .filter(Boolean);
      value = field.type === "int_array" ? items.map((item) => Number(item)) : items;
    }
    updateNodeParams(selectedNode.id, { ...selectedNode.params, [field.key]: value });
  };

  const handleFileUpload = async (field: ParameterField, files: FileList | null) => {
    if (!files?.length) {
      return;
    }

    try {
      setUploadingField(field.key);
      const uploaded = await uploadFiles(Array.from(files));
      addUploads(uploaded);

      if (field.key === "model_path") {
        updateNodeParams(selectedNode.id, { ...selectedNode.params, [field.key]: uploaded[0]?.path ?? "" });
      } else if (field.key === "file_ids") {
        updateNodeParams(selectedNode.id, {
          ...selectedNode.params,
          [field.key]: uploaded.map((item) => item.fileId),
        });
      } else {
        updateNodeParams(selectedNode.id, { ...selectedNode.params, [field.key]: uploaded[0]?.fileId ?? "" });
      }

      uploaded.forEach((item) => addLog(`Uploaded ${item.filename}`));
    } catch (error) {
      addLog(error instanceof Error ? error.message : "Upload failed");
    } finally {
      setUploadingField(null);
    }
  };

  return (
    <aside className="panel rounded-3xl p-5">
      <div className="text-xs uppercase tracking-[0.2em] text-sky-300">Inspector</div>
      <h3 className="mt-2 text-lg font-semibold">{selectedNode.label}</h3>
      <p className="mt-1 text-xs uppercase tracking-[0.16em] text-slate-400">{selectedNode.type}</p>
      <div className="mt-5 space-y-4">
        {nodeType.paramsSchema.map((field) => {
          const currentValue = selectedNode.params[field.key];
          const eligibleUploads = getEligibleUploads(field, selectedNode.type, uploadEntries);
          return (
            <label key={field.key} className="block">
              <div className="mb-1 text-sm font-medium text-slate-200">{field.label}</div>
              {field.type === "boolean" ? (
                <input
                  type="checkbox"
                  checked={Boolean(currentValue)}
                  onChange={(event) => onFieldChange(field, event.target.checked)}
                  className="h-4 w-4 rounded border-white/20 bg-slate-950"
                />
              ) : field.type === "select" ? (
                <select
                  value={String(currentValue ?? field.defaultValue ?? "")}
                  onChange={(event) => onFieldChange(field, event.target.value)}
                  className="w-full rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm"
                >
                  {field.options?.map((option) => (
                    <option key={String(option.value)} value={String(option.value)}>
                      {option.label}
                    </option>
                  ))}
                </select>
              ) : isFileBackedField(field) ? (
                <div className="space-y-2">
                  {field.key === "file_ids" ? (
                    <select
                      multiple
                      value={Array.isArray(currentValue) ? currentValue.map(String) : []}
                      onChange={(event) =>
                        updateNodeParams(selectedNode.id, {
                          ...selectedNode.params,
                          [field.key]: Array.from(event.target.selectedOptions).map((option) => option.value),
                        })
                      }
                      className="min-h-28 w-full rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm"
                    >
                      {eligibleUploads.map((item) => (
                        <option key={item.fileId} value={item.fileId}>
                          {item.filename}
                        </option>
                      ))}
                    </select>
                  ) : (
                    <select
                      value={String(currentValue ?? "")}
                      onChange={(event) => onFieldChange(field, event.target.value)}
                      className="w-full rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm"
                    >
                      <option value="">Select uploaded file</option>
                      {eligibleUploads.map((item) => (
                        <option key={item.fileId} value={field.key === "model_path" ? item.path : item.fileId}>
                          {item.filename}
                        </option>
                      ))}
                    </select>
                  )}
                  <label className="inline-flex cursor-pointer items-center gap-2 rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm text-white transition hover:border-sky-400/40 hover:bg-slate-900">
                    <span>{uploadingField === field.key ? "Uploading..." : "Choose File"}</span>
                    <input
                      type="file"
                      className="hidden"
                      multiple={field.key === "file_ids"}
                      accept={getFieldAccept(field, selectedNode.type)}
                      onChange={(event) => {
                        void handleFileUpload(field, event.target.files);
                        event.target.value = "";
                      }}
                    />
                  </label>
                  {eligibleUploads.length > 0 ? (
                    <div className="text-xs text-slate-500">
                      Available: {eligibleUploads.map((item) => item.filename).join(", ")}
                    </div>
                  ) : (
                    <div className="text-xs text-slate-500">No uploaded files match this field yet.</div>
                  )}
                </div>
              ) : (
                <input
                  type={field.type === "number" ? "number" : "text"}
                  value={
                    field.type === "string_array" || field.type === "int_array"
                      ? renderStringArrayValue(currentValue)
                      : String(currentValue ?? field.defaultValue ?? "")
                  }
                  onChange={(event) => onFieldChange(field, event.target.value)}
                  className="w-full rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm"
                />
              )}
              {field.helperText ? <div className="mt-1 text-xs text-slate-500">{field.helperText}</div> : null}
            </label>
          );
        })}
      </div>
    </aside>
  );
}
