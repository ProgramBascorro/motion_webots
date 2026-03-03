"use client";

import { Handle, Position, type NodeProps } from "@xyflow/react";
import { Camera, CircuitBoard, Eye, SlidersHorizontal, Workflow } from "lucide-react";

const categoryIcon = {
  input: Camera,
  preprocess: SlidersHorizontal,
  inference: Eye,
  postprocess: CircuitBoard,
};

export function VisionNode({ data, selected }: NodeProps<{ label: string; nodeType: string; category?: string }>) {
  const Icon = categoryIcon[(data.category ?? "preprocess") as keyof typeof categoryIcon] ?? Workflow;
  return (
    <div
      className={`min-w-[190px] rounded-2xl border px-4 py-3 shadow-lg transition ${
        selected ? "border-sky-300 bg-sky-400/10" : "border-white/10 bg-slate-950/85"
      }`}
    >
      <Handle type="target" position={Position.Left} className="!bg-sky-300" />
      <div className="mb-2 flex items-center gap-2">
        <div className="rounded-xl bg-sky-400/10 p-2 text-sky-300">
          <Icon size={16} />
        </div>
        <div>
          <div className="text-sm font-semibold text-white">{data.label}</div>
          <div className="text-[11px] uppercase tracking-[0.18em] text-slate-400">{data.nodeType}</div>
        </div>
      </div>
      <div className="flex items-center justify-between text-[11px] text-slate-400">
        <span>Vision node</span>
        <span>{selected ? "Selected" : "Ready"}</span>
      </div>
      <Handle type="source" position={Position.Right} className="!bg-emerald-300" />
    </div>
  );
}
