"use client";

import { Download, LayoutPanelTop, Play, Save, Square, Upload } from "lucide-react";

type Props = {
  onRun: () => void;
  onStop: () => void;
  onSave: () => void;
  onLoad: (file: File) => void;
  onAutoLayout: () => void;
  onUploadAssets: (files: File[]) => void;
  runDisabled?: boolean;
  stopDisabled?: boolean;
  children?: React.ReactNode;
};

function ActionButton({
  label,
  icon: Icon,
  onClick,
  disabled,
}: {
  label: string;
  icon: React.ComponentType<{ size?: number }>;
  onClick: () => void;
  disabled?: boolean;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      disabled={disabled}
      className="inline-flex items-center gap-2 rounded-2xl border border-white/10 bg-slate-950/70 px-4 py-2 text-sm font-medium text-white transition hover:border-sky-400/40 hover:bg-slate-900 disabled:cursor-not-allowed disabled:opacity-50"
    >
      <Icon size={16} />
      {label}
    </button>
  );
}

export function Toolbar({
  onRun,
  onStop,
  onSave,
  onLoad,
  onAutoLayout,
  onUploadAssets,
  runDisabled,
  stopDisabled,
  children,
}: Props) {
  return (
    <div className="panel flex flex-wrap items-center justify-between gap-3 rounded-3xl px-4 py-3">
      <div className="flex flex-wrap items-center gap-2">
        <ActionButton label="Run" icon={Play} onClick={onRun} disabled={runDisabled} />
        <ActionButton label="Stop" icon={Square} onClick={onStop} disabled={stopDisabled} />
        <ActionButton label="Save JSON" icon={Save} onClick={onSave} />
        <label className="inline-flex cursor-pointer items-center gap-2 rounded-2xl border border-white/10 bg-slate-950/70 px-4 py-2 text-sm font-medium text-white transition hover:border-sky-400/40 hover:bg-slate-900">
          <Download size={16} />
          Load JSON
          <input type="file" accept="application/json" className="hidden" onChange={(event) => event.target.files?.[0] && onLoad(event.target.files[0])} />
        </label>
        <label className="inline-flex cursor-pointer items-center gap-2 rounded-2xl border border-white/10 bg-slate-950/70 px-4 py-2 text-sm font-medium text-white transition hover:border-sky-400/40 hover:bg-slate-900">
          <Upload size={16} />
          Upload Assets
          <input type="file" multiple className="hidden" onChange={(event) => onUploadAssets(Array.from(event.target.files ?? []))} />
        </label>
        <ActionButton label="Auto Layout" icon={LayoutPanelTop} onClick={onAutoLayout} />
      </div>
      <div>{children}</div>
    </div>
  );
}
