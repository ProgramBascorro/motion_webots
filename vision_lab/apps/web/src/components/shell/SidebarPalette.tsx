"use client";

import type { NodeTypeDefinition } from "@vision-lab/shared";

type Props = {
  nodeTypes: NodeTypeDefinition[];
  onAddNode: (definition: NodeTypeDefinition) => void;
};

const categoryOrder = ["input", "preprocess", "inference", "postprocess"];

export function SidebarPalette({ nodeTypes, onAddNode }: Props) {
  const grouped = categoryOrder.map((category) => ({
    category,
    items: nodeTypes.filter((item) => item.category === category),
  }));

  return (
    <aside className="panel panel-strong flex h-full flex-col rounded-3xl p-4">
      <div className="mb-4">
        <div className="text-xs uppercase tracking-[0.24em] text-sky-300">Node Palette</div>
        <h2 className="mt-2 text-lg font-semibold">Vision building blocks</h2>
      </div>
      <div className="space-y-4 overflow-y-auto pr-1">
        {grouped.map(({ category, items }) => (
          <section key={category}>
            <div className="mb-2 text-xs uppercase tracking-[0.18em] text-slate-400">{category}</div>
            <div className="space-y-2">
              {items.map((item) => (
                <button
                  key={item.type}
                  type="button"
                  onClick={() => onAddNode(item)}
                  className="w-full rounded-2xl border border-white/10 bg-slate-950/70 px-3 py-3 text-left transition hover:border-sky-400/40 hover:bg-slate-900"
                >
                  <div className="text-sm font-medium text-white">{item.displayName}</div>
                  <div className="mt-1 text-xs text-slate-400">{item.producedOutputKinds.join(", ")}</div>
                </button>
              ))}
            </div>
          </section>
        ))}
      </div>
    </aside>
  );
}
