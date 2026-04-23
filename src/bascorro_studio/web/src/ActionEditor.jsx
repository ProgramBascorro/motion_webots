import { useEffect, useMemo, useRef, useState } from "react";
import YAML from "js-yaml";
import ROSLIB from "roslib";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { 
  Play, 
  Square, 
  RotateCw, 
  RotateCcw, 
  Eye, 
  EyeOff, 
  Save, 
  Trash2, 
  Undo, 
  Redo, 
  Send,
  Download,
  Upload,
  RefreshCw,
  Plus,
  Copy
} from "lucide-react";

// --- Constants & Helpers ---

const JOINT_ORDER = [
  "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
  "r_el", "l_el", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
  "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch",
  "r_ank_roll", "l_ank_roll", "head_pan", "head_tilt",
];

// Source of truth: src/ROBOTIS-OP3/op3_manager/config/OP3.robot
const JOINT_META = {
  r_sho_pitch: { id: 1, labelEn: "R Shoulder Pitch", labelId: "Bahu kanan pitch" },
  l_sho_pitch: { id: 2, labelEn: "L Shoulder Pitch", labelId: "Bahu kiri pitch" },
  r_sho_roll: { id: 3, labelEn: "R Shoulder Roll", labelId: "Bahu kanan roll" },
  l_sho_roll: { id: 4, labelEn: "L Shoulder Roll", labelId: "Bahu kiri roll" },
  r_el: { id: 5, labelEn: "R Elbow", labelId: "Siku kanan" },
  l_el: { id: 6, labelEn: "L Elbow", labelId: "Siku kiri" },
  r_hip_yaw: { id: 7, labelEn: "R Hip Yaw", labelId: "Pinggul kanan yaw" },
  l_hip_yaw: { id: 8, labelEn: "L Hip Yaw", labelId: "Pinggul kiri yaw" },
  r_hip_roll: { id: 9, labelEn: "R Hip Roll", labelId: "Pinggul kanan roll" },
  l_hip_roll: { id: 10, labelEn: "L Hip Roll", labelId: "Pinggul kiri roll" },
  r_hip_pitch: { id: 11, labelEn: "R Hip Pitch", labelId: "Pinggul kanan pitch" },
  l_hip_pitch: { id: 12, labelEn: "L Hip Pitch", labelId: "Pinggul kiri pitch" },
  r_knee: { id: 13, labelEn: "R Knee", labelId: "Lutut kanan" },
  l_knee: { id: 14, labelEn: "L Knee", labelId: "Lutut kiri" },
  r_ank_pitch: { id: 15, labelEn: "R Ankle Pitch", labelId: "Pergelangan kaki kanan pitch" },
  l_ank_pitch: { id: 16, labelEn: "L Ankle Pitch", labelId: "Pergelangan kaki kiri pitch" },
  r_ank_roll: { id: 17, labelEn: "R Ankle Roll", labelId: "Pergelangan kaki kanan roll" },
  l_ank_roll: { id: 18, labelEn: "L Ankle Roll", labelId: "Pergelangan kaki kiri roll" },
  head_pan: { id: 19, labelEn: "Head Pan", labelId: "Kepala kiri-kanan" },
  head_tilt: { id: 20, labelEn: "Head Tilt", labelId: "Kepala atas-bawah" },
};

// Joint limits in degrees, based on op3_kinematics_dynamics.cpp.
const JOINT_LIMITS_DEG = {
  head_pan: { min: -90, max: 90 },
  head_tilt: { min: -90, max: 90 },
  r_sho_pitch: { min: -90, max: 90 },
  r_sho_roll: { min: -90, max: 54 },
  r_el: { min: -90, max: 90 },
  l_sho_pitch: { min: -90, max: 90 },
  l_sho_roll: { min: -54, max: 90 },
  l_el: { min: -90, max: 90 },
  r_hip_yaw: { min: -81, max: 81 },
  l_hip_yaw: { min: -81, max: 81 },
  r_hip_roll: { min: -54, max: 54 },
  l_hip_roll: { min: -54, max: 54 },
  r_hip_pitch: { min: -72, max: 72 },
  l_hip_pitch: { min: -72, max: 72 },
  r_knee: { min: -126, max: 18 },
  l_knee: { min: -18, max: 126 },
  r_ank_pitch: { min: -81, max: 81 },
  l_ank_pitch: { min: -81, max: 81 },
  r_ank_roll: { min: -81, max: 81 },
  l_ank_roll: { min: -81, max: 81 },
};

const JOINT_GROUPS = {
  head: ["head_pan", "head_tilt"],
  arm: ["r_sho_pitch", "r_sho_roll", "r_el", "l_sho_pitch", "l_sho_roll", "l_el"],
  leg: [
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
  ],
};

const LEFT_RIGHT_MIRROR_PAIRS = [
  { right: "r_sho_pitch", left: "l_sho_pitch", invert: true },
  { right: "r_sho_roll", left: "l_sho_roll", invert: true },
  { right: "r_el", left: "l_el", invert: true },
  { right: "r_hip_yaw", left: "l_hip_yaw", invert: true },
  { right: "r_hip_roll", left: "l_hip_roll", invert: true },
  { right: "r_hip_pitch", left: "l_hip_pitch", invert: true },
  { right: "r_knee", left: "l_knee", invert: true },
  { right: "r_ank_pitch", left: "l_ank_pitch", invert: true },
  { right: "r_ank_roll", left: "l_ank_roll", invert: true },
];

const JOINT_ID = Object.fromEntries(
  Object.entries(JOINT_META).map(([name, meta]) => [name, meta.id])
);
const JOINT_LABELS = Object.fromEntries(
  Object.entries(JOINT_META).map(([name, meta]) => [name, meta.labelEn])
);
const JOINT_LABELS_ID = Object.fromEntries(
  Object.entries(JOINT_META).map(([name, meta]) => [name, meta.labelId])
);

const RAW_CENTER = 2048;
const RAW_RANGE = 2048;
const ACTION_MAX_STEPS = 7;
const PRESET_STORAGE_KEY = "op3PosePresets";
const ACTION_DRAFT_KEY = "op3ActionEditorDraftYaml";
const ACTION_DRAFT_META_KEY = "op3ActionEditorDraftMeta";
const HISTORY_STORAGE_KEY = "op3ActionEditorHistoryV2";
const SNAPSHOT_STORAGE_KEY = "op3ActionEditorSnapshotsV1";
const DOC_HISTORY_LIMIT = 120;
const SNAPSHOT_LIMIT = 40;
const COALESCE_WINDOW_MS = 300;
const JOINT_SOURCE_TIMEOUT_MS = 1500;
const TICK_SECONDS = 0.008;

const DEFAULT_ASSETS = import.meta.env.VITE_ASSETS_URL || "http://localhost:8001";

function toRadians(raw) { return ((raw - RAW_CENTER) * Math.PI) / RAW_RANGE; }
function toDegrees(raw) { return ((raw - RAW_CENTER) * 180) / RAW_RANGE; }
function clampRaw(value) { return Math.max(0, Math.min(4095, value)); }
function clampNumber(value, min, max) { return Math.max(min, Math.min(max, value)); }
function toRawDegrees(degrees) { return clampRaw(Math.round((degrees * RAW_RANGE) / 180 + RAW_CENTER)); }
function isTorqueOff(value) { return typeof value === "string" && value.toLowerCase() === "torque_off"; }
function isInvalidRaw(value) {
  if (value === null || value === undefined) return true;
  if (typeof value === "string") {
    const lowered = value.toLowerCase();
    return lowered === "torque_off" || lowered === "invalid" || lowered === "none";
  }
  return false;
}
function normalizeRaw(value) {
  if (isInvalidRaw(value)) return null;
  if (typeof value === "number") return Math.round(value);
  if (typeof value === "string") {
    const parsed = Number(value.trim());
    if (Number.isFinite(parsed)) return Math.round(parsed);
  }
  return null;
}
function formatJointLabel(name) { return JOINT_LABELS[name] || name; }
function formatJointLabelId(name) { return JOINT_LABELS_ID[name] || name; }
function formatLimitDeg(value) {
  return Number.isFinite(value) ? value.toFixed(1) : "";
}
function buildLimitPreset(preset) {
  if (preset === "default") return JOINT_LIMITS_DEG;
  const scaled = {};
  Object.entries(JOINT_LIMITS_DEG).forEach(([name, lim]) => {
    scaled[name] = { min: lim.min, max: lim.max };
  });
  const scales = preset === "walking"
    ? { head: 0.6, arm: 0.6, leg: 1 }
    : preset === "gesture"
      ? { head: 1, arm: 1, leg: 0.6 }
      : {};
  Object.entries(scales).forEach(([group, factor]) => {
    (JOINT_GROUPS[group] || []).forEach(name => {
      const lim = scaled[name];
      if (!lim) return;
      scaled[name] = { min: lim.min * factor, max: lim.max * factor };
    });
  });
  return scaled;
}
function stepDurationSeconds(step, header, scheduleMode) {
  const time = Number(step?.time ?? 0);
  const pause = Number(step?.pause ?? 0);
  const timeTicks = Number.isFinite(time) ? time : 0;
  const pauseTicks = Number.isFinite(pause) ? pause : 0;
  const speed = resolveSpeed(header);
  const safeSpeed = Number.isFinite(speed) && speed > 0 ? speed : 32;
  if (scheduleMode === "time") {
    const ticks = timeTicks + pauseTicks;
    return (ticks > 0 ? ticks : 1) * TICK_SECONDS;
  }
  const timeSeconds = timeTicks * TICK_SECONDS * (safeSpeed / 32);
  const pauseSeconds = pauseTicks * TICK_SECONDS * (32 / safeSpeed);
  const total = timeSeconds + pauseSeconds;
  return total > 0 ? total : TICK_SECONDS;
}

function getTimelineSegment(pos, durations, cumulative) {
  if (!durations.length) return { index: 0, t: 0 };
  const total = cumulative[cumulative.length - 1] + durations[durations.length - 1];
  const clamped = clampNumber(Number(pos) || 0, 0, total);
  for (let i = 0; i < durations.length; i += 1) {
    const start = cumulative[i];
    const end = start + durations[i];
    if (clamped <= end || i === durations.length - 1) {
      const t = durations[i] > 0 ? (clamped - start) / durations[i] : 0;
      return { index: i, t: clampNumber(t, 0, 1) };
    }
  }
  return { index: durations.length - 1, t: 1 };
}

function lookupRawPosition(positions, name, idMap) {
  if (!positions) return undefined;
  if (Array.isArray(positions)) {
    const id = idMap[name];
    return id ? positions[id] : undefined;
  }
  if (typeof positions === "object") {
    if (Object.prototype.hasOwnProperty.call(positions, name)) return positions[name];
    const id = idMap[name];
    if (id) {
      const idKey = `id_${id}`;
      if (Object.prototype.hasOwnProperty.call(positions, idKey)) return positions[idKey];
    }
  }
  return undefined;
}

function setRawPosition(positions, name, idMap, value) {
  if (!positions) return;
  const id = idMap[name];
  if (value === undefined) {
    if (Array.isArray(positions)) {
      if (id !== undefined) delete positions[id];
      return;
    }
    if (typeof positions === "object") {
      delete positions[name];
      if (id) delete positions[`id_${id}`];
    }
    return;
  }
  if (Array.isArray(positions)) {
    if (id !== undefined) positions[id] = value;
    return;
  }
  if (typeof positions === "object") positions[name] = value;
}

function mirrorRawValue(rawValue, invertSign) {
  if (rawValue === undefined) return undefined;
  const normalized = normalizeRaw(rawValue);
  if (normalized === null) return rawValue;
  if (!invertSign) return clampRaw(normalized);
  return clampRaw(2 * RAW_CENTER - normalized);
}

function mirrorStepPositions(positions, idMap, availableNames = null) {
  const source = (positions && typeof positions === "object") ? positions : {};
  const target = Array.isArray(source) ? [...source] : { ...source };
  LEFT_RIGHT_MIRROR_PAIRS.forEach(({ right, left, invert }) => {
    if (availableNames && (!availableNames.has(right) || !availableNames.has(left))) return;
    const rightRaw = lookupRawPosition(source, right, idMap);
    const leftRaw = lookupRawPosition(source, left, idMap);
    setRawPosition(target, right, idMap, mirrorRawValue(leftRaw, invert));
    setRawPosition(target, left, idMap, mirrorRawValue(rightRaw, invert));
  });
  return target;
}

const SectionHeader = ({ title, children }) => (
  <div className="flex justify-between items-center mb-4">
    <h2 className="text-lg font-bold font-display text-gray-800">{title}</h2>
    <div className="flex gap-2">{children}</div>
  </div>
);

const resolveSpeed = (header) => {
  const raw = Number(header?.speed);
  return (Number.isFinite(raw) && raw > 0) ? raw : 32;
};

const secondsToTimeTicks = (seconds, speed) => {
  if (!Number.isFinite(seconds)) return 0;
  return Math.max(0, Math.min(255, Math.round((seconds / 0.008) * (32 / speed))));
};

const secondsToPauseTicks = (seconds, speed) => {
  if (!Number.isFinite(seconds)) return 0;
  return Math.max(0, Math.min(255, Math.round((seconds / 0.008) * (speed / 32))));
};

function buildPose(positions, livePose) {
  const pose = {};
  JOINT_ORDER.forEach((name) => {
    const rawValue = lookupRawPosition(positions, name, JOINT_ID);
    const normalized = normalizeRaw(rawValue);
    if (normalized === null) {
      pose[name] = livePose[name] ?? 0;
    } else {
      pose[name] = toRadians(normalized);
    }
  });
  return pose;
}

function normalizePageStepsInPlace(page) {
  if (!page || typeof page !== "object") return;
  if (!Array.isArray(page.steps)) page.steps = [];
  if (page.steps.length > ACTION_MAX_STEPS) {
    page.steps = page.steps.slice(0, ACTION_MAX_STEPS);
  }
  page.steps.forEach((step, idx) => {
    if (!step || typeof step !== "object") page.steps[idx] = { index: idx, pause: 0, time: 0, positions: {} };
    page.steps[idx].index = idx;
  });
  if (!page.header || typeof page.header !== "object") page.header = {};
  page.header.stepnum = page.steps.length;
  const speed = Number(page.header.speed);
  if (!Number.isFinite(speed) || speed <= 0) page.header.speed = 32;
}

function findStepByIndex(page, stepIndex) {
  if (!page || !Array.isArray(page.steps)) return null;
  const byIndex = page.steps.find(step => Number(step?.index) === Number(stepIndex));
  if (byIndex) return byIndex;
  return page.steps[stepIndex] || null;
}

function normalizeSelection(selection) {
  return {
    pageIndex: selection?.pageIndex ?? null,
    stepIndex: selection?.stepIndex ?? null,
  };
}

function sameSelection(a, b) {
  return (a?.pageIndex ?? null) === (b?.pageIndex ?? null)
    && (a?.stepIndex ?? null) === (b?.stepIndex ?? null);
}

function makeHistoryEntry(label, yaml, selection) {
  return {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    ts: Date.now(),
    label,
    yamlText: String(yaml ?? ""),
    selection: normalizeSelection(selection),
  };
}

function parseYamlSafe(text) {
  if (!String(text || "").trim()) return { data: null, error: "" };
  try {
    const parsed = YAML.load(text);
    return { data: parsed || null, error: "" };
  } catch (err) {
    return { data: null, error: err?.message || "Invalid YAML" };
  }
}

// --- Component ---

export default function ActionEditor({ isActive = true, rosUrl = "" }) {
  const [yamlText, setYamlText] = useState("");
  const [yamlData, setYamlData] = useState(null);
  const [parseError, setParseError] = useState("");
  const [selectedPageIndex, setSelectedPageIndex] = useState(null);
  const [selectedStepIndex, setSelectedStepIndex] = useState(null);

  const jointDraftKey = (name) => {
    const pk = selectedPageIndex ?? "n";
    const sk = selectedStepIndex ?? "n";
    return `${pk}:${sk}:${name}`;
  };
  const [showAllJoints, setShowAllJoints] = useState(false);
  const [upright, setUpright] = useState(true);
  const [layFlat, setLayFlat] = useState(false);
  const [mirrorView, setMirrorView] = useState(false);
  const [viewerEnabled, setViewerEnabled] = useState(true);
  const [viewerNote, setViewerNote] = useState("");
  const [sendStepToWebots, setSendStepToWebots] = useState(() => {
    const raw = localStorage.getItem("op3SendStepToWebots");
    return raw === "1" || raw === "true";
  });
  const [recordEnabled, setRecordEnabled] = useState(false);
  const [recordMode, setRecordMode] = useState("time");
  const [recordElapsed, setRecordElapsed] = useState(0);
  const [recordLastDelta, setRecordLastDelta] = useState(null);
  const [recordLastTicks, setRecordLastTicks] = useState(null);
  const [timelinePos, setTimelinePos] = useState(0);
  const [timelinePlaying, setTimelinePlaying] = useState(false);
  const [timelineSpeed, setTimelineSpeed] = useState(1);
  const [timelineLoop, setTimelineLoop] = useState(false);
  const [limitPreset, setLimitPreset] = useState("default");
  const [jointGridColumns, setJointGridColumns] = useState(() => {
    const raw = localStorage.getItem("op3JointGridColumns");
    return raw === "2" ? 2 : 1;
  });
  const [autoEnableAction, setAutoEnableAction] = useState(true);
  const [showYaml, setShowYaml] = useState(true);
  const [scratchPageIndex, setScratchPageIndex] = useState(250);
  const [rangeStart, setRangeStart] = useState("");
  const [rangeEnd, setRangeEnd] = useState("");
  const [copySourcePageIndex, setCopySourcePageIndex] = useState("");
  const [copySourceStepIndex, setCopySourceStepIndex] = useState("");
  const [copyTargetPageIndex, setCopyTargetPageIndex] = useState("");
  const [copyTargetStepIndex, setCopyTargetStepIndex] = useState("");
  const [presetName, setPresetName] = useState("");
  const [posePresets, setPosePresets] = useState(() => {
    try {
      const raw = localStorage.getItem(PRESET_STORAGE_KEY);
      return raw ? JSON.parse(raw) : [];
    } catch { return []; }
  });
  const [showJointGuide, setShowJointGuide] = useState(false);
  const [previewPose, setPreviewPose] = useState(null);
  const [livePose, setLivePose] = useState({});
  const [status, setStatus] = useState("Idle");
  const [statusError, setStatusError] = useState(false);
  const [exportPages, setExportPages] = useState("used");
  const [pageFilter, setPageFilter] = useState("");
  const [jointDrafts, setJointDrafts] = useState({});
  const [manualOffJoints, setManualOffJoints] = useState(() => new Set());
  const [hasLocalDraft, setHasLocalDraft] = useState(false);
  const [historyTick, setHistoryTick] = useState(0);
  const [lastHistoryLabel, setLastHistoryLabel] = useState("");
  const [snapshotName, setSnapshotName] = useState("");
  const [snapshots, setSnapshots] = useState(() => {
    try {
      const raw = localStorage.getItem(SNAPSHOT_STORAGE_KEY);
      const parsed = raw ? JSON.parse(raw) : [];
      return Array.isArray(parsed) ? parsed.slice(0, SNAPSHOT_LIMIT) : [];
    } catch {
      return [];
    }
  });

  const [assetsUrl, setAssetsUrl] = useState(() => localStorage.getItem("op3AssetsUrl") || DEFAULT_ASSETS);
  const [rosState, setRosState] = useState("disconnected");

  const jointPubRef = useRef(null);
  const torquePubRef = useRef(null);
  const actionPagePubRef = useRef(null);
  const enableModulePubRef = useRef(null);
  const requestRef = useRef(null);
  const resultSubRef = useRef(null);
  const jointHardwareSubRef = useRef(null);
  const jointSimSubRef = useRef(null);
  const lastJointSourceRef = useRef({ hardware: 0, sim: 0 });
  const livePoseRef = useRef({});
  const pendingRunRef = useRef(null);
  const historyRef = useRef({
    undo: [],
    redo: [],
    lastCommitted: {
      yamlText: "",
      selection: { pageIndex: null, stepIndex: null },
    },
  });
  const historyCoalesceRef = useRef({ key: "", ts: 0 });
  const suppressYamlHistoryRef = useRef(false);
  const yamlDirtyRef = useRef(false);
  const recordRef = useRef({ lastTime: null, lastStepIndex: null, lastPageIndex: null });
  const timelineRef = useRef({ lastTime: null });

  const viewerRef = useRef(null);
  const robotRef = useRef(null);
  const cameraRef = useRef(null);
  const controlsRef = useRef(null);
  const defaultCameraPosRef = useRef(new THREE.Vector3(0.6, 0.35, 1.4));
  const defaultTargetRef = useRef(new THREE.Vector3(0, 0, 0));

  // --- Logic Hooks (Same as original) ---
  const pages = useMemo(() => yamlData && Array.isArray(yamlData.pages) ? yamlData.pages : [], [yamlData]);
  const jointMeta = useMemo(() => {
    const order = yamlData?.meta?.joint_order || [];
    const names = [], nameToId = {};
    order.forEach(entry => { if(entry?.name) { names.push(entry.name); nameToId[entry.name] = entry.id; }});
    return { names, nameToId };
  }, [yamlData]);
  const jointIdMap = useMemo(() => Object.keys(jointMeta.nameToId).length ? jointMeta.nameToId : JOINT_ID, [jointMeta]);
  const editableJointNames = useMemo(
    () => (jointMeta.names.length ? jointMeta.names : JOINT_ORDER),
    [jointMeta]
  );
  const jointNames = useMemo(() => showAllJoints && jointMeta.names.length ? jointMeta.names : JOINT_ORDER, [showAllJoints, jointMeta]);
  const visiblePages = useMemo(() => {
    const filter = pageFilter.trim().toLowerCase();
    if (!filter) return pages;
    return pages.filter(p => (p.name || "").toLowerCase().includes(filter) || String(p.index).includes(filter));
  }, [pages, pageFilter]);
  const sortedPages = useMemo(
    () => [...pages].sort((a, b) => Number(a.index) - Number(b.index)),
    [pages]
  );
  
  const activePage = useMemo(() => selectedPageIndex === null ? null : pages.find(p => p.index === selectedPageIndex) || null, [pages, selectedPageIndex]);
  
  const activeStep = useMemo(() => {
    if (!activePage || selectedStepIndex === null) return null;
    return activePage.steps?.find(s => Number(s.index) === Number(selectedStepIndex)) || activePage.steps?.[selectedStepIndex] || null;
  }, [activePage, selectedStepIndex]);
  const scheduleMode = useMemo(() => {
    const header = activePage?.header;
    return header?.schedule === 10 || header?.schedule === "time" ? "time" : "speed";
  }, [activePage]);

  const timelineData = useMemo(() => {
    const steps = activePage?.steps || [];
    if (!steps.length) return { total: 0, durations: [], cumulative: [] };
    const durations = steps.map(step => stepDurationSeconds(step, activePage?.header, scheduleMode));
    const cumulative = [];
    let acc = 0;
    durations.forEach((d) => { cumulative.push(acc); acc += d; });
    return { total: acc, durations, cumulative };
  }, [activePage, scheduleMode]);
  const activeLimitMap = useMemo(() => buildLimitPreset(limitPreset), [limitPreset]);

  const editorDisabled = !yamlData || Boolean(parseError);

  // --- Robot Viewer Logic ---
  const applyRobotOrientation = (robot, uprightValue, layFlatValue) => {
    const baseRotationX = -Math.PI / 2;
    const extra = uprightValue ? 0 : Math.PI;
    const flat = layFlatValue ? Math.PI / 2 : 0;
    robot.rotation.set(baseRotationX + extra + flat, 0, 0);
  };

  const applyRobotMirror = (robot, mirrorValue) => {
    robot.scale.x = mirrorValue ? -1 : 1;
    robot.traverse((child) => {
      if (child.isMesh && child.material) {
        if (Array.isArray(child.material)) child.material.forEach(m => { m.side = THREE.DoubleSide; m.needsUpdate = true; });
        else { child.material.side = THREE.DoubleSide; child.material.needsUpdate = true; }
      }
    });
  };

  const resetView = () => {
    if (!cameraRef.current || !controlsRef.current) return;
    cameraRef.current.position.copy(defaultCameraPosRef.current);
    controlsRef.current.target.copy(defaultTargetRef.current);
    cameraRef.current.lookAt(defaultTargetRef.current);
    controlsRef.current.update();
  };

  const rotateView = (degrees) => {
    if (!cameraRef.current || !controlsRef.current) return;
    const target = controlsRef.current.target.clone();
    const offset = cameraRef.current.position.clone().sub(target);
    offset.applyAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(degrees));
    cameraRef.current.position.copy(target.clone().add(offset));
    cameraRef.current.lookAt(target);
    controlsRef.current.update();
  };

  const cloneData = (data) => JSON.parse(JSON.stringify(data));

  const currentSelection = () => normalizeSelection({
    pageIndex: selectedPageIndex,
    stepIndex: selectedStepIndex,
  });

  const updateCommittedState = (yamlValue, selectionValue) => {
    historyRef.current.lastCommitted = {
      yamlText: String(yamlValue ?? ""),
      selection: normalizeSelection(selectionValue),
    };
  };

  const applyDocumentState = (state, options = {}) => {
    const nextYamlText = String(state?.yamlText ?? "");
    const nextSelection = normalizeSelection(state?.selection);
    const parsed = options.parsed || parseYamlSafe(nextYamlText);
    suppressYamlHistoryRef.current = true;
    setYamlText(nextYamlText);
    setYamlData(parsed.data);
    setParseError(parsed.error);
    setSelectedPageIndex(nextSelection.pageIndex);
    setSelectedStepIndex(nextSelection.stepIndex);
    setJointDrafts({});
    updateCommittedState(nextYamlText, nextSelection);
  };

  const pushUndoState = (label, state, options = {}) => {
    const snapshot = {
      yamlText: String(state?.yamlText ?? ""),
      selection: normalizeSelection(state?.selection),
    };
    const top = historyRef.current.undo[historyRef.current.undo.length - 1];
    if (top && top.yamlText === snapshot.yamlText && sameSelection(top.selection, snapshot.selection)) return;

    const now = Date.now();
    const coalesceKey = options.coalesceKey || "";
    const recent = historyCoalesceRef.current;
    const canCoalesce = Boolean(
      coalesceKey
      && recent.key === coalesceKey
      && (now - recent.ts) <= COALESCE_WINDOW_MS
    );

    if (!canCoalesce) {
      historyRef.current.undo.push(makeHistoryEntry(label, snapshot.yamlText, snapshot.selection));
      if (historyRef.current.undo.length > DOC_HISTORY_LIMIT) historyRef.current.undo.shift();
    }

    historyCoalesceRef.current = { key: coalesceKey, ts: now };
  };

  const commitDocumentState = (label, nextState, options = {}) => {
    const nextYamlText = String(nextState?.yamlText ?? "");
    const nextSelection = normalizeSelection(nextState?.selection ?? currentSelection());
    const currSelection = currentSelection();
    const currYamlText = String(yamlText ?? "");
    const unchanged = currYamlText === nextYamlText && sameSelection(currSelection, nextSelection);
    if (unchanged) return false;

    pushUndoState(label, { yamlText: currYamlText, selection: currSelection }, { coalesceKey: options.coalesceKey });
    historyRef.current.redo = [];
    applyDocumentState({ yamlText: nextYamlText, selection: nextSelection }, { parsed: options.parsed });
    setLastHistoryLabel(label);
    setHistoryTick((t) => t + 1);
    return true;
  };

  const mutateYaml = (label, updater, options = {}) => {
    if (!yamlData) return false;
    const next = cloneData(yamlData);
    updater(next);
    if (Array.isArray(next.pages)) next.pages.forEach(normalizePageStepsInPlace);
    const nextYamlText = YAML.dump(next, { sortKeys: false, lineWidth: -1 });
    const nextSelection = options.nextSelection || currentSelection();
    return commitDocumentState(label, { yamlText: nextYamlText, selection: nextSelection }, {
      coalesceKey: options.coalesceKey,
      parsed: { data: next, error: "" },
    });
  };

  // --- Effects ---
  useEffect(() => {
    try {
      const saved = localStorage.getItem(ACTION_DRAFT_KEY);
      if (!saved || !saved.trim()) return;
      setYamlText(saved);
      updateCommittedState(saved, { pageIndex: null, stepIndex: null });
      setHasLocalDraft(true);
      setStatus("Restored local draft");
      setStatusError(false);
    } catch {
      // Ignore localStorage read errors in restrictive browser contexts.
    }
  }, []);

  useEffect(() => {
    try {
      const raw = localStorage.getItem(HISTORY_STORAGE_KEY);
      if (!raw) return;
      const parsed = JSON.parse(raw);
      const sanitizeEntries = (entries) => (
        Array.isArray(entries)
          ? entries
            .filter((entry) => entry && typeof entry.yamlText === "string")
            .map((entry) => ({
              ...entry,
              selection: normalizeSelection(entry.selection),
            }))
            .slice(-DOC_HISTORY_LIMIT)
          : []
      );

      historyRef.current.undo = sanitizeEntries(parsed?.undo);
      historyRef.current.redo = sanitizeEntries(parsed?.redo);
      const current = parsed?.current && typeof parsed.current.yamlText === "string"
        ? {
          yamlText: parsed.current.yamlText,
          selection: normalizeSelection(parsed.current.selection),
        }
        : null;

      if (current) {
        applyDocumentState(current);
      }
      if (typeof parsed?.lastHistoryLabel === "string") {
        setLastHistoryLabel(parsed.lastHistoryLabel);
      }
      setHistoryTick((t) => t + 1);
    } catch {
      // Ignore history restore issues.
    }
  }, []);

  useEffect(() => { localStorage.setItem("op3AssetsUrl", assetsUrl); }, [assetsUrl]);
  useEffect(() => { localStorage.setItem(PRESET_STORAGE_KEY, JSON.stringify(posePresets)); }, [posePresets]);
  useEffect(() => { localStorage.setItem("op3SendStepToWebots", sendStepToWebots ? "1" : "0"); }, [sendStepToWebots]);
  useEffect(() => { localStorage.setItem("op3JointGridColumns", String(jointGridColumns)); }, [jointGridColumns]);
  useEffect(() => {
    try {
      localStorage.setItem(SNAPSHOT_STORAGE_KEY, JSON.stringify(snapshots.slice(0, SNAPSHOT_LIMIT)));
    } catch {
      // Ignore snapshot persistence failures.
    }
  }, [snapshots]);
  useEffect(() => {
    try {
      localStorage.setItem(HISTORY_STORAGE_KEY, JSON.stringify({
        undo: historyRef.current.undo.slice(-DOC_HISTORY_LIMIT),
        redo: historyRef.current.redo.slice(-DOC_HISTORY_LIMIT),
        current: historyRef.current.lastCommitted,
        lastHistoryLabel,
      }));
    } catch {
      // Ignore history persistence failures.
    }
  }, [historyTick, lastHistoryLabel]);

  useEffect(() => {
    const handle = window.setTimeout(() => {
      try {
        if (!yamlText.trim()) {
          localStorage.removeItem(ACTION_DRAFT_KEY);
          localStorage.removeItem(ACTION_DRAFT_META_KEY);
          setHasLocalDraft(false);
          return;
        }
        localStorage.setItem(ACTION_DRAFT_KEY, yamlText);
        localStorage.setItem(
          ACTION_DRAFT_META_KEY,
          JSON.stringify({ saved_at: Date.now(), source: "autosave" })
        );
        setHasLocalDraft(true);
      } catch {
        // Ignore localStorage write errors.
      }
    }, 350);
    return () => window.clearTimeout(handle);
  }, [yamlText]);
  
  useEffect(() => {
    if (!yamlText.trim()) { setYamlData(null); setParseError(""); return; }
    const handle = window.setTimeout(() => {
      try {
        const parsed = YAML.load(yamlText);
        setYamlData(parsed || null);
        setParseError("");
      } catch (err) {
        setParseError(err?.message || "Invalid YAML");
        setYamlData(null);
      }
    }, 250);
    return () => window.clearTimeout(handle);
  }, [yamlText]);

  useEffect(() => {
    const handle = window.setTimeout(() => {
      if (suppressYamlHistoryRef.current) {
        suppressYamlHistoryRef.current = false;
        return;
      }
      if (!yamlDirtyRef.current) return;
      if (parseError) return;
      const committed = historyRef.current.lastCommitted;
      const selection = currentSelection();
      if (committed.yamlText === yamlText && sameSelection(committed.selection, selection)) {
        yamlDirtyRef.current = false;
        return;
      }
      pushUndoState("Edit YAML", committed, { coalesceKey: "yaml-edit" });
      historyRef.current.redo = [];
      updateCommittedState(yamlText, selection);
      setLastHistoryLabel("Edit YAML");
      setHistoryTick((t) => t + 1);
      yamlDirtyRef.current = false;
    }, 500);
    return () => window.clearTimeout(handle);
  }, [yamlText, parseError, selectedPageIndex, selectedStepIndex]);

  useEffect(() => {
    if (suppressYamlHistoryRef.current || yamlDirtyRef.current || parseError) return;
    const selection = currentSelection();
    const committed = historyRef.current.lastCommitted;
    if (committed.yamlText === yamlText && sameSelection(committed.selection, selection)) return;
    updateCommittedState(yamlText, selection);
    setHistoryTick((t) => t + 1);
  }, [yamlText, parseError, selectedPageIndex, selectedStepIndex]);

  useEffect(() => {
    if (pages.length && !pages.find(p => p.index === selectedPageIndex)) {
      setSelectedPageIndex(pages[0].index);
      setSelectedStepIndex(null);
    }
  }, [pages, selectedPageIndex]);

  useEffect(() => {
    if (!activePage) { setSelectedStepIndex(null); return; }
    if (selectedStepIndex !== null) {
      const foundStep = activePage.steps?.find(s => Number(s.index) === Number(selectedStepIndex));
      if (!foundStep && !activePage.steps?.[selectedStepIndex]) setSelectedStepIndex(null);
    }
  }, [activePage, selectedStepIndex]);

  useEffect(() => setJointDrafts({}), [selectedPageIndex, selectedStepIndex]);

  useEffect(() => {
    if (!activeStep) { setRangeStart(""); setRangeEnd(""); return; }
    const val = activeStep.index !== undefined ? activeStep.index : selectedStepIndex;
    if (val !== null) { setRangeStart(String(val)); setRangeEnd(String(val)); }
  }, [activeStep, selectedStepIndex]);

  useEffect(() => {
    if (selectedPageIndex === null || selectedPageIndex === undefined) return;
    setCopySourcePageIndex(prev => (prev === "" ? String(selectedPageIndex) : prev));
    setCopyTargetPageIndex(prev => (prev === "" ? String(selectedPageIndex) : prev));
  }, [selectedPageIndex]);

  useEffect(() => {
    if (selectedStepIndex === null || selectedStepIndex === undefined) return;
    setCopySourceStepIndex(prev => (prev === "" ? String(selectedStepIndex) : prev));
    setCopyTargetStepIndex(prev => (prev === "" ? String(selectedStepIndex) : prev));
  }, [selectedStepIndex]);

  useEffect(() => {
    recordRef.current = { lastTime: null, lastStepIndex: null, lastPageIndex: selectedPageIndex };
    setRecordElapsed(0); setRecordLastDelta(null); setRecordLastTicks(null);
  }, [recordEnabled, selectedPageIndex]);

  useEffect(() => {
    if (!recordEnabled) return;
    const timer = setInterval(() => {
      const last = recordRef.current.lastTime;
      if (last === null) setRecordElapsed(0);
      else setRecordElapsed((performance.now() - last) / 1000);
    }, 100);
    return () => clearInterval(timer);
  }, [recordEnabled]);

  // --- ROS ---
  useEffect(() => {
    if (!rosUrl) return;
    const ros = new ROSLIB.Ros({ url: rosUrl });
    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    jointPubRef.current = new ROSLIB.Topic({ ros, name: "/webots/joint_positions", messageType: "std_msgs/Float64MultiArray" });
    torquePubRef.current = new ROSLIB.Topic({ ros, name: "/robotis/sync_write_item", messageType: "robotis_controller_msgs/SyncWriteItem" });
    actionPagePubRef.current = new ROSLIB.Topic({ ros, name: "/robotis/action/page_num", messageType: "std_msgs/Int32" });
    enableModulePubRef.current = new ROSLIB.Topic({ ros, name: "/robotis/enable_ctrl_module", messageType: "std_msgs/String" });
    requestRef.current = new ROSLIB.Topic({ ros, name: "/bascorro_studio/request", messageType: "std_msgs/String" });
    
    resultSubRef.current = new ROSLIB.Topic({ ros, name: "/bascorro_studio/result", messageType: "std_msgs/String" });
    resultSubRef.current.subscribe(msg => {
      try {
        const payload = JSON.parse(msg.data);
        const msgText = payload.message || (payload.ok ? "Success" : "Error");
        setStatus(msgText);
        setStatusError(!payload.ok);
        if (payload.action === "export" && payload.yaml) setYamlText(payload.yaml);
        if (payload.action === "apply" && payload.ok) {
          try {
            localStorage.removeItem(ACTION_DRAFT_KEY);
            localStorage.removeItem(ACTION_DRAFT_META_KEY);
          } catch {
            // Ignore localStorage clear errors.
          }
          setHasLocalDraft(false);
        }
        
        const pending = pendingRunRef.current;
        if (pending && payload.request_id === pending.requestId) {
          pendingRunRef.current = null;
          if (payload.ok) {
            if (autoEnableAction) {
              enableModulePubRef.current?.publish(new ROSLIB.Message({ data: "action_module" }));
              setTimeout(() => actionPagePubRef.current?.publish(new ROSLIB.Message({ data: pending.pageIndex })), 300);
            } else {
              actionPagePubRef.current?.publish(new ROSLIB.Message({ data: pending.pageIndex }));
            }
          } else {
            setStatus("Scratch apply failed");
            setStatusError(true);
          }
        }
      } catch (e) { setStatus("Error parsing result"); setStatusError(true); }
    });

    const applyJointState = (msg) => {
      const next = { ...livePoseRef.current };
      msg.name.forEach((name, idx) => { if (JOINT_ID[name]) next[name] = msg.position[idx]; });
      livePoseRef.current = next;
      setLivePose(next);
    };

    jointHardwareSubRef.current = new ROSLIB.Topic({ ros, name: "/robotis/present_joint_states", messageType: "sensor_msgs/JointState" });
    jointHardwareSubRef.current.subscribe(msg => {
      lastJointSourceRef.current.hardware = Date.now();
      applyJointState(msg);
    });

    jointSimSubRef.current = new ROSLIB.Topic({ ros, name: "/robotis_op3/joint_states", messageType: "sensor_msgs/JointState" });
    jointSimSubRef.current.subscribe(msg => {
      const now = Date.now();
      if (now - lastJointSourceRef.current.hardware <= JOINT_SOURCE_TIMEOUT_MS) return;
      lastJointSourceRef.current.sim = now;
      applyJointState(msg);
    });

    return () => {
      jointHardwareSubRef.current?.unsubscribe();
      jointSimSubRef.current?.unsubscribe();
      resultSubRef.current?.unsubscribe();
      ros.close();
    };
  }, [rosUrl]);

  // --- Three.js ---
  useEffect(() => {
    if (!viewerEnabled || !viewerRef.current || !isActive) return;
    const container = viewerRef.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#f8fafc"); // Slate-50

    const width = container.clientWidth;
    const height = container.clientHeight;
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 20);
    camera.position.copy(defaultCameraPosRef.current);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.copy(defaultTargetRef.current);
    controls.update();

    cameraRef.current = camera;
    controlsRef.current = controls;

    const ambient = new THREE.AmbientLight(0xffffff, 0.7);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    dirLight.position.set(1.2, 1.5, 0.8);
    scene.add(ambient, dirLight);

    const grid = new THREE.GridHelper(2.4, 20, 0xd1d5db, 0xe2e8f0);
    grid.position.y = -0.32;
    scene.add(grid);

    const loader = new URDFLoader();
    const handleContextLost = (e) => { e.preventDefault(); setViewerNote("WebGL Lost"); setViewerEnabled(false); };
    renderer.domElement.addEventListener("webglcontextlost", handleContextLost);

    loader.load(`${assetsUrl}/robotis_op3.urdf`, (robot) => {
      applyRobotOrientation(robot, upright, layFlat);
      applyRobotMirror(robot, mirrorView);
      robotRef.current = robot;
      scene.add(robot);
    });

    let frameId;
    const animate = () => {
      frameId = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      const w = container.clientWidth, h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      cancelAnimationFrame(frameId);
      controls.dispose();
      renderer.dispose();
      if(container.contains(renderer.domElement)) container.removeChild(renderer.domElement);
    };
  }, [assetsUrl, viewerEnabled, isActive]);

  useEffect(() => { if(robotRef.current) applyRobotOrientation(robotRef.current, upright, layFlat); }, [upright, layFlat]);
  useEffect(() => { if(robotRef.current) applyRobotMirror(robotRef.current, mirrorView); }, [mirrorView]);

  const activePose = previewPose || livePose;
  useEffect(() => {
    if (!robotRef.current || !activePose) return;
    JOINT_ORDER.forEach(name => {
      const val = activePose[name];
      if (robotRef.current.joints[name] && Number.isFinite(val)) robotRef.current.joints[name].setJointValue(val);
    });
  }, [activePose]);

  useEffect(() => {
    if (!previewPose || !activeStep) return;
    setPreviewPose(buildPose(activeStep.positions || {}, livePoseRef.current));
  }, [activeStep, yamlData]);

  useEffect(() => {
    if (!activePage || selectedStepIndex === null) return;
    const steps = activePage.steps || [];
    if (!steps.length) { setTimelinePos(0); return; }
    const byIndex = steps.findIndex(s => Number(s.index) === Number(selectedStepIndex));
    const idx = Number.isInteger(byIndex) && byIndex >= 0 ? byIndex : selectedStepIndex;
    if (!Number.isFinite(idx)) return;
    const clampedIdx = clampNumber(Number(idx), 0, steps.length - 1);
    const start = timelineData.cumulative[clampedIdx] ?? 0;
    setTimelinePos(start);
  }, [activePage, selectedStepIndex, timelineData]);

  useEffect(() => {
    if (!activePage?.steps?.length) { setTimelinePos(0); setTimelinePlaying(false); return; }
    setTimelinePos(pos => clampNumber(Number(pos) || 0, 0, timelineData.total));
    setTimelinePlaying(false);
  }, [activePage, timelineData.total]);

  useEffect(() => {
    if (!timelinePlaying) { timelineRef.current.lastTime = null; return; }
    const total = timelineData.total;
    if (!activePage?.steps?.length || total <= 0) { setTimelinePlaying(false); return; }
    let frameId;
    const tick = (now) => {
      if (!timelineRef.current.lastTime) timelineRef.current.lastTime = now;
      const dt = (now - timelineRef.current.lastTime) / 1000;
      timelineRef.current.lastTime = now;
      setTimelinePos(prev => {
        const next = prev + dt * timelineSpeed;
        if (next >= total) {
          if (timelineLoop) {
            return 0;
          }
          setTimelinePlaying(false);
          return total;
        }
        return next;
      });
      frameId = requestAnimationFrame(tick);
    };
    frameId = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frameId);
  }, [timelinePlaying, timelineSpeed, activePage, timelineData.total]);

  useEffect(() => {
    if (!activePage || !activePage.steps?.length) return;
    if (!timelineData.durations.length) return;
    const steps = activePage.steps;
    const { index, t } = getTimelineSegment(timelinePos, timelineData.durations, timelineData.cumulative);
    const stepA = steps[index];
    if (!stepA) return;
    const poseA = buildPose(stepA.positions || {}, livePoseRef.current);
    if (index >= steps.length - 1) { setPreviewPose(poseA); return; }
    const stepB = steps[index + 1];
    const poseB = buildPose(stepB?.positions || {}, livePoseRef.current);
    const interp = {};
    JOINT_ORDER.forEach(name => {
      const a = poseA[name] ?? 0;
      const b = poseB[name] ?? a;
      interp[name] = a + (b - a) * t;
    });
    setPreviewPose(interp);
  }, [timelinePos, activePage, livePose, timelineData]);

  // --- Handlers (Requests, Updates, History) ---
  const sendRequest = (payload) => {
    if (!requestRef.current) { setStatus("ROS not connected"); setStatusError(true); return; }
    requestRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
  };

  const handleApply = () => { sendRequest({ action: "apply", yaml: yamlText }); setStatus("Applying..."); setStatusError(false); };
  const handleExport = () => { sendRequest({ action: "export", pages: exportPages }); setStatus("Exporting..."); setStatusError(false); };

  const updateYamlData = (label, updater, options = {}) => mutateYaml(label, updater, options);

  const updateActivePage = (updater, options = {}) => updateYamlData(options.label || "Edit page", (draft) => {
    const p = draft.pages?.find((i) => i.index === activePage?.index);
    if (p) updater(p);
  }, { coalesceKey: options.coalesceKey || "active-page" });

  const updateActiveStep = (updater, options = {}) => {
    if (!activePage || selectedStepIndex === null) return;
    updateYamlData(options.label || "Edit step", (draft) => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if(!p?.steps) return;
      const step = p.steps.find(s => Number(s.index) === Number(selectedStepIndex)) || p.steps[selectedStepIndex];
      if(step) updater(step, p);
    }, { coalesceKey: options.coalesceKey || "active-step" });
  };

  const findNextPageIndex = (existingPages) => {
    const used = new Set((existingPages || [])
      .map(p => Number(p.index))
      .filter(n => Number.isFinite(n) && n >= 1 && n <= 255));
    const maxUsed = used.size ? Math.max(...used) : 0;
    const candidate = maxUsed + 1;
    if (candidate >= 1 && candidate <= 255 && !used.has(candidate)) return candidate;
    for (let i = 1; i <= 255; i += 1) {
      if (!used.has(i)) return i;
    }
    return null;
  };

  const handleAddPage = () => {
    if (editorDisabled) return;
    const nextIndex = findNextPageIndex(pages);
    if (nextIndex === null) {
      setStatus("No free page index (1-255)");
      setStatusError(true);
      return;
    }
    const name = `Page ${nextIndex}`;
    const header = { repeat: 1, schedule: 0, stepnum: 0, speed: 32, accel: 0, next: 0, exit: 0 };
    updateYamlData("Add page", (draft) => {
      if (!Array.isArray(draft.pages)) draft.pages = [];
      draft.pages.push({ index: nextIndex, name, header, steps: [] });
      draft.pages.sort((a, b) => Number(a.index) - Number(b.index));
    }, {
      nextSelection: { pageIndex: nextIndex, stepIndex: null },
    });
    setPreviewPose(null);
    setStatus(`Added ${name}`);
    setStatusError(false);
  };

  const handleDuplicatePage = () => {
    if (!activePage || editorDisabled) return;
    const nextIndex = findNextPageIndex(pages);
    if (nextIndex === null) {
      setStatus("No free page index (1-255)");
      setStatusError(true);
      return;
    }

    const sourceIndex = activePage.index;
    const sourceName = typeof activePage.name === "string" && activePage.name.trim()
      ? activePage.name.trim()
      : `Page ${sourceIndex}`;

    updateYamlData("Duplicate page", (draft) => {
      const sourcePage = draft.pages?.find(p => p.index === sourceIndex);
      if (!sourcePage) return;
      if (!Array.isArray(draft.pages)) draft.pages = [];

      const duplicatePage = cloneData(sourcePage);
      duplicatePage.index = nextIndex;
      duplicatePage.name = `${sourceName} (copy)`;
      if (!Array.isArray(duplicatePage.steps)) duplicatePage.steps = [];
      duplicatePage.steps.forEach((step, idx) => { step.index = idx; });

      draft.pages.push(duplicatePage);
      draft.pages.sort((a, b) => Number(a.index) - Number(b.index));
    }, {
      nextSelection: { pageIndex: nextIndex, stepIndex: null },
    });

    setPreviewPose(null);
    setJointDrafts({});
    setStatus(`Duplicated page ${sourceIndex} -> ${nextIndex}`);
    setStatusError(false);
  };

  const handleMirrorPage = () => {
    if (!activePage || editorDisabled) return;
    const names = jointMeta.names.length ? jointMeta.names : JOINT_ORDER;
    const availableNames = new Set(names);
    const mirroredActive = activeStep
      ? mirrorStepPositions(activeStep.positions || {}, jointIdMap, availableNames)
      : null;
    updateYamlData("Mirror page", (draft) => {
      const page = draft.pages?.find(p => p.index === activePage.index);
      if (!page || !Array.isArray(page.steps)) return;
      page.steps.forEach((step, idx) => {
        const sourcePositions = step?.positions || {};
        step.positions = mirrorStepPositions(sourcePositions, jointIdMap, availableNames);
        step.index = idx;
      });
    });
    setJointDrafts({});
    setPreviewPose(mirroredActive ? buildPose(mirroredActive, livePoseRef.current) : null);
    setStatus(`Mirrored page ${activePage.index} L/R`);
    setStatusError(false);
  };

  const getInsertStepIndex = () => {
    if (!activePage) return 0;
    let insertAt = activePage.steps?.length || 0;
    if (selectedStepIndex !== null) {
      const byIndex = activePage.steps?.findIndex(s => Number(s.index) === Number(selectedStepIndex));
      if (Number.isInteger(byIndex) && byIndex >= 0) insertAt = byIndex + 1;
      else if (selectedStepIndex >= 0 && selectedStepIndex < (activePage.steps?.length || 0)) {
        insertAt = selectedStepIndex + 1;
      }
    }
    return insertAt;
  };

  const handleAddStep = () => {
    if (!activePage || editorDisabled) return;
    if ((activePage.steps?.length || 0) >= ACTION_MAX_STEPS) {
      setStatus(`Max ${ACTION_MAX_STEPS} steps per page`);
      setStatusError(true);
      return;
    }
    const insertAt = getInsertStepIndex();
    updateYamlData("Add step", (draft) => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: 0, time: 0, positions: {} };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    }, {
      nextSelection: { pageIndex: activePage.index, stepIndex: insertAt },
    });
    setPreviewPose(buildPose({}, livePoseRef.current));
    setStatus(`Added step ${insertAt}`);
    setStatusError(false);
  };

  const handleDuplicateStep = () => {
    if (!activePage || !activeStep || editorDisabled) return;
    if ((activePage.steps?.length || 0) >= ACTION_MAX_STEPS) {
      setStatus(`Max ${ACTION_MAX_STEPS} steps per page`);
      setStatusError(true);
      return;
    }
    const insertAt = getInsertStepIndex();
    const stepCopy = cloneData(activeStep.positions || {});
    updateYamlData("Duplicate step", (draft) => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: activeStep.pause ?? 0, time: activeStep.time ?? 0, positions: stepCopy };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    }, {
      nextSelection: { pageIndex: activePage.index, stepIndex: insertAt },
    });
    setPreviewPose(buildPose(stepCopy, livePoseRef.current));
    setStatus(`Duplicated step ${activeStep.index ?? selectedStepIndex}`);
    setStatusError(false);
  };

  const handleCopyStepToTarget = () => {
    if (editorDisabled || !yamlData) return;
    const sourcePage = Number(copySourcePageIndex);
    const sourceStep = Number(copySourceStepIndex);
    const targetPage = Number(copyTargetPageIndex);
    const targetStep = Number(copyTargetStepIndex);

    if (!Number.isInteger(sourcePage) || sourcePage < 1 || sourcePage > 255) {
      setStatus("Invalid source page (1-255)");
      setStatusError(true);
      return;
    }
    if (!Number.isInteger(targetPage) || targetPage < 1 || targetPage > 255) {
      setStatus("Invalid target page (1-255)");
      setStatusError(true);
      return;
    }
    if (!Number.isInteger(sourceStep) || sourceStep < 0 || sourceStep >= ACTION_MAX_STEPS) {
      setStatus(`Invalid source step (0-${ACTION_MAX_STEPS - 1})`);
      setStatusError(true);
      return;
    }
    if (!Number.isInteger(targetStep) || targetStep < 0 || targetStep >= ACTION_MAX_STEPS) {
      setStatus(`Invalid target step (0-${ACTION_MAX_STEPS - 1})`);
      setStatusError(true);
      return;
    }
    if (sourcePage === targetPage && sourceStep === targetStep) {
      setStatus("Source and target are same step");
      setStatusError(false);
      return;
    }

    const sourcePageData = pages.find(p => Number(p.index) === sourcePage);
    const targetPageData = pages.find(p => Number(p.index) === targetPage);
    if (!sourcePageData) {
      setStatus(`Source page ${sourcePage} not found`);
      setStatusError(true);
      return;
    }
    if (!targetPageData) {
      setStatus(`Target page ${targetPage} not found`);
      setStatusError(true);
      return;
    }
    const sourceStepData = findStepByIndex(sourcePageData, sourceStep);
    if (!sourceStepData) {
      setStatus(`Source step ${sourceStep} not found in page ${sourcePage}`);
      setStatusError(true);
      return;
    }

    const sourcePositions = sourceStepData.positions;
    const copiedStep = {
      positions: cloneData(sourcePositions || (Array.isArray(sourcePositions) ? [] : {})),
      time: sourceStepData.time ?? 0,
      pause: sourceStepData.pause ?? 0,
    };

    let copied = false;
    updateYamlData("Copy step", (draft) => {
      const targetPageDraft = draft.pages?.find(p => Number(p.index) === targetPage);
      if (!targetPageDraft) return;
      if (!Array.isArray(targetPageDraft.steps)) targetPageDraft.steps = [];
      while (targetPageDraft.steps.length <= targetStep) {
        if (targetPageDraft.steps.length >= ACTION_MAX_STEPS) return;
        targetPageDraft.steps.push({ index: targetPageDraft.steps.length, pause: 0, time: 0, positions: {} });
      }
      if (!targetPageDraft.steps[targetStep] || typeof targetPageDraft.steps[targetStep] !== "object") {
        targetPageDraft.steps[targetStep] = { index: targetStep, pause: 0, time: 0, positions: {} };
      }
      const targetStepDraft = targetPageDraft.steps[targetStep];
      targetStepDraft.positions = cloneData(copiedStep.positions);
      targetStepDraft.time = copiedStep.time;
      targetStepDraft.pause = copiedStep.pause;
      copied = true;
    }, {
      nextSelection: Number(selectedPageIndex) === targetPage && Number(selectedStepIndex) === targetStep
        ? { pageIndex: targetPage, stepIndex: targetStep }
        : currentSelection(),
    });

    if (!copied) {
      setStatus("Copy failed: target step unavailable");
      setStatusError(true);
      return;
    }

    if (Number(selectedPageIndex) === targetPage && Number(selectedStepIndex) === targetStep) {
      setPreviewPose(buildPose(copiedStep.positions || {}, livePoseRef.current));
      setJointDrafts({});
    }
    setStatus(`Copied P${sourcePage}S${sourceStep} -> P${targetPage}S${targetStep}`);
    setStatusError(false);
  };

  const handleCapturePose = () => {
    if (!activeStep || editorDisabled) return;
    const live = livePoseRef.current || {};
    const captureNames = editableJointNames.filter(name => manualOffJoints.has(name));
    if (!captureNames.length) {
      setStatus("No OFF joints selected");
      setStatusError(true);
      return;
    }
    const hasLive = captureNames.some(name => Number.isFinite(live[name]));
    if (!hasLive) {
      setStatus("No live pose available for OFF joints");
      setStatusError(true);
      return;
    }
    const captured = [];
    const previewPositions = cloneData(
      activeStep.positions || (Array.isArray(activeStep.positions) ? [] : {})
    );
    captureNames.forEach(name => {
      const rad = live[name];
      if (!Number.isFinite(rad)) return;
      const raw = toRawDegrees((rad * 180) / Math.PI);
      captured.push({ name, raw });
      setRawPosition(previewPositions, name, jointIdMap, raw);
    });
    updateActiveStep((s) => {
      if (!s.positions || typeof s.positions !== "object") s.positions = {};
      captureNames.forEach(name => {
        const rad = live[name];
        if (!Number.isFinite(rad)) return;
        const raw = toRawDegrees((rad * 180) / Math.PI);
        setRawPosition(s.positions, name, jointIdMap, raw);
      });
    }, { label: "Capture OFF joints", coalesceKey: "capture-off-joints" });
    setJointDrafts({});
    setPreviewPose(buildPose(previewPositions, livePoseRef.current));

    const capturedNames = captured.map(item => item.name);
    const capturedValues = captured.map(item => item.raw);
    const turnedOn = applyCapturedTargetsToRobot(capturedNames, capturedValues);

    if (turnedOn) {
      setManualOffJoints(prev => {
        const next = new Set(prev);
        capturedNames.forEach(name => next.delete(name));
        return next;
      });
      setStatus(`Captured ${captured.length} OFF joint(s) and auto ON`);
    } else if (captured.length) {
      setStatus(`Captured ${captured.length} OFF joint(s), auto ON skipped`);
    } else {
      setStatus("No OFF joints had live pose");
      setStatusError(true);
      return;
    }
    setStatusError(false);
  };

  const handleCapturePoseToNewStep = () => {
    if (!activePage || !activeStep || editorDisabled) return;
    if ((activePage.steps?.length || 0) >= ACTION_MAX_STEPS) {
      setStatus(`Max ${ACTION_MAX_STEPS} steps per page`);
      setStatusError(true);
      return;
    }
    const live = livePoseRef.current || {};
    const captureNames = editableJointNames.filter(name => manualOffJoints.has(name));
    if (!captureNames.length) {
      setStatus("No OFF joints selected");
      setStatusError(true);
      return;
    }
    const hasLive = captureNames.some(name => Number.isFinite(live[name]));
    if (!hasLive) {
      setStatus("No live pose available for OFF joints");
      setStatusError(true);
      return;
    }
    const captured = [];
    const positions = cloneData(
      activeStep.positions || (Array.isArray(activeStep.positions) ? [] : {})
    );
    captureNames.forEach(name => {
      const rad = live[name];
      if (!Number.isFinite(rad)) return;
      const raw = toRawDegrees((rad * 180) / Math.PI);
      captured.push({ name, raw });
      setRawPosition(positions, name, jointIdMap, raw);
    });
    const insertAt = getInsertStepIndex();
    updateYamlData("Capture OFF joints to new step", (draft) => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: activeStep.pause ?? 0, time: activeStep.time ?? 0, positions };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    }, {
      nextSelection: { pageIndex: activePage.index, stepIndex: insertAt },
    });
    setPreviewPose(buildPose(positions, livePoseRef.current));
    setJointDrafts({});

    const capturedNames = captured.map(item => item.name);
    const capturedValues = captured.map(item => item.raw);
    const turnedOn = applyCapturedTargetsToRobot(capturedNames, capturedValues);

    if (turnedOn) {
      setManualOffJoints(prev => {
        const next = new Set(prev);
        capturedNames.forEach(name => next.delete(name));
        return next;
      });
      setStatus(`Captured ${captured.length} OFF joint(s) to new step and auto ON`);
    } else if (captured.length) {
      setStatus(`Captured ${captured.length} OFF joint(s) to new step, auto ON skipped`);
    } else {
      setStatus("No OFF joints had live pose");
      setStatusError(true);
      return;
    }
    setStatusError(false);
  };

  const handleSwapLeftRight = () => {
    if (!activeStep || editorDisabled) return;
    const names = jointMeta.names.length ? jointMeta.names : JOINT_ORDER;
    const availableNames = new Set(names);
    const mirrored = mirrorStepPositions(activeStep.positions || {}, jointIdMap, availableNames);
    updateActiveStep((s) => {
      s.positions = cloneData(mirrored);
    }, { label: "Mirror step L/R", coalesceKey: "mirror-step" });
    setJointDrafts({});
    setPreviewPose(buildPose(mirrored, livePoseRef.current));
    setStatus("Mirrored step L/R");
    setStatusError(false);
  };

  const snapTimelineToNearestStep = () => {
    if (!activePage?.steps?.length || !timelineData.cumulative.length) return;
    const current = clampNumber(Number(timelinePos) || 0, 0, timelineData.total);
    let nearest = 0;
    let best = Infinity;
    timelineData.cumulative.forEach((start, idx) => {
      const dist = Math.abs(current - start);
      if (dist < best) {
        best = dist;
        nearest = idx;
      }
    });
    const snapped = timelineData.cumulative[nearest] ?? 0;
    setTimelinePos(snapped);
    setSelectedStepIndex(nearest);
  };

  const handleDeletePage = () => {
    if (!activePage || editorDisabled) return;
    const pageIndex = activePage.index;
    const pageName = activePage.name || "Untitled";
    if (!window.confirm(`Delete page ${pageIndex} (${pageName})?`)) return;
    const sortedPages = [...pages].sort((a, b) => Number(a.index) - Number(b.index));
    const currentIdx = sortedPages.findIndex(p => p.index === pageIndex);
    const nextPage = sortedPages[currentIdx + 1] || sortedPages[currentIdx - 1] || null;
    updateYamlData("Delete page", (draft) => {
      if (!Array.isArray(draft.pages)) return;
      draft.pages = draft.pages.filter(p => p.index !== pageIndex);
    }, {
      nextSelection: { pageIndex: nextPage ? nextPage.index : null, stepIndex: null },
    });
    setPreviewPose(null);
    setJointDrafts({});
    setStatus(`Deleted page ${pageIndex}`);
    setStatusError(false);
  };

  const handleDeleteStep = () => {
    if (!activePage || selectedStepIndex === null || editorDisabled) return;
    const steps = activePage.steps || [];
    if (!steps.length) return;
    const byIndex = steps.findIndex(s => Number(s.index) === Number(selectedStepIndex));
    const removeIndex = Number.isInteger(byIndex) && byIndex >= 0 ? byIndex : selectedStepIndex;
    if (removeIndex < 0 || removeIndex >= steps.length) return;
    const stepLabel = steps[removeIndex]?.index ?? removeIndex;
    if (!window.confirm(`Delete step ${stepLabel}?`)) return;
    const remaining = steps.filter((_, idx) => idx !== removeIndex);
    const nextSelected = remaining.length ? Math.min(removeIndex, remaining.length - 1) : null;
    const nextStep = nextSelected !== null ? remaining[nextSelected] : null;
    updateYamlData("Delete step", (draft) => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p?.steps) return;
      const byIdx = p.steps.findIndex(s => Number(s.index) === Number(selectedStepIndex));
      const removeAt = Number.isInteger(byIdx) && byIdx >= 0 ? byIdx : selectedStepIndex;
      if (removeAt < 0 || removeAt >= p.steps.length) return;
      p.steps.splice(removeAt, 1);
      p.steps.forEach((step, idx) => { step.index = idx; });
    }, {
      nextSelection: { pageIndex: activePage.index, stepIndex: nextSelected },
    });
    setPreviewPose(nextStep ? buildPose(nextStep.positions || {}, livePoseRef.current) : null);
    setJointDrafts({});
    setStatus(`Deleted step ${stepLabel}`);
    setStatusError(false);
  };

  const handleUndoDocument = () => {
    if (!historyRef.current.undo.length) return;
    const previous = historyRef.current.undo.pop();
    historyRef.current.redo.push(makeHistoryEntry(
      lastHistoryLabel || "Current",
      yamlText,
      currentSelection(),
    ));
    if (historyRef.current.redo.length > DOC_HISTORY_LIMIT) historyRef.current.redo.shift();
    historyCoalesceRef.current = { key: "", ts: 0 };
    applyDocumentState({
      yamlText: previous.yamlText,
      selection: previous.selection,
    });
    setLastHistoryLabel(previous.label || "Undo");
    setStatus(`Undo: ${previous.label || "Change"}`);
    setStatusError(false);
    setHistoryTick((t) => t + 1);
  };

  const handleRedoDocument = () => {
    if (!historyRef.current.redo.length) return;
    const next = historyRef.current.redo.pop();
    historyRef.current.undo.push(makeHistoryEntry(
      lastHistoryLabel || "Current",
      yamlText,
      currentSelection(),
    ));
    if (historyRef.current.undo.length > DOC_HISTORY_LIMIT) historyRef.current.undo.shift();
    historyCoalesceRef.current = { key: "", ts: 0 };
    applyDocumentState({
      yamlText: next.yamlText,
      selection: next.selection,
    });
    setLastHistoryLabel(next.label || "Redo");
    setStatus(`Redo: ${next.label || "Change"}`);
    setStatusError(false);
    setHistoryTick((t) => t + 1);
  };

  const canUndo = useMemo(() => historyRef.current.undo.length > 0, [historyTick]);
  const canRedo = useMemo(() => historyRef.current.redo.length > 0, [historyTick]);

  const handleCreateSnapshot = () => {
    const trimmed = snapshotName.trim();
    const generated = `Snapshot ${new Date().toLocaleTimeString()}`;
    const name = trimmed || generated;
    const entry = {
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
      name,
      ts: Date.now(),
      yamlText,
      selection: currentSelection(),
    };
    setSnapshots((prev) => [entry, ...prev].slice(0, SNAPSHOT_LIMIT));
    setSnapshotName("");
    setStatus(`Snapshot saved: ${name}`);
    setStatusError(false);
  };

  const handleRestoreSnapshot = (snapshot) => {
    if (!snapshot || typeof snapshot.yamlText !== "string") return;
    if (!window.confirm(`Restore snapshot "${snapshot.name}"?`)) return;
    commitDocumentState(
      `Restore snapshot: ${snapshot.name}`,
      { yamlText: snapshot.yamlText, selection: normalizeSelection(snapshot.selection) },
    );
    setStatus(`Restored snapshot: ${snapshot.name}`);
    setStatusError(false);
  };

  const handleDeleteSnapshot = (snapshotId) => {
    setSnapshots((prev) => prev.filter((item) => item.id !== snapshotId));
  };

  const handleRenameSnapshot = (snapshot) => {
    const nextName = window.prompt("Rename snapshot", snapshot?.name || "");
    if (!nextName || !nextName.trim()) return;
    const clean = nextName.trim();
    setSnapshots((prev) => prev.map((item) => (
      item.id === snapshot.id ? { ...item, name: clean } : item
    )));
  };

  // --- Keyboard Shortcuts ---
  useEffect(() => {
    const handleKey = (e) => {
      if (!(e.ctrlKey || e.metaKey)) return;
      const key = e.key.toLowerCase();
      if (key !== "z" && key !== "y") return;

      const tagName = e.target?.tagName || "";
      if (tagName === "INPUT" || tagName === "SELECT") return;
      if (tagName === "TEXTAREA" && yamlDirtyRef.current) return;

      e.preventDefault();
      if (key === "z") {
        e.shiftKey ? handleRedoDocument() : handleUndoDocument();
      } else {
        handleRedoDocument();
      }
    };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, [historyTick, yamlText, selectedPageIndex, selectedStepIndex, lastHistoryLabel]);

  // --- Joint Logic ---
  const handleJointDraftChange = (name, value) => {
    setJointDrafts(prev => ({ ...prev, [jointDraftKey(name)]: value }));
  };

  const publishSyncWriteItem = (itemName, jointNameList, values, setErrorStatus = true) => {
    const targets = Array.from(new Set((jointNameList || []).filter(Boolean)));
    if (!targets.length) return false;
    if (rosState !== "connected" || !torquePubRef.current) {
      if (setErrorStatus) {
        setStatus("ROS not connected");
        setStatusError(true);
      }
      return false;
    }
    if (!itemName) return false;
    if (!Array.isArray(values) || values.length !== targets.length) return false;
    torquePubRef.current.publish(new ROSLIB.Message({
      item_name: itemName,
      joint_name: targets,
      value: values,
    }));
    return true;
  };

  const publishTorqueEnable = (jointNameList, enable, setErrorStatus = true) => {
    const targets = Array.from(new Set((jointNameList || []).filter(Boolean)));
    return publishSyncWriteItem(
      "torque_enable",
      targets,
      targets.map(() => (enable ? 1 : 0)),
      setErrorStatus
    );
  };

  const applyCapturedTargetsToRobot = (jointNames, rawValues) => {
    const targets = Array.from(new Set((jointNames || []).filter(Boolean)));
    if (!targets.length) return false;
    if (rosState !== "connected" || !torquePubRef.current) return false;
    if (!Array.isArray(rawValues) || rawValues.length !== targets.length) return false;

    // Prevent active modules from overriding manual captured joint goals.
    enableModulePubRef.current?.publish(new ROSLIB.Message({ data: "none" }));

    const wroteGoal = publishSyncWriteItem("goal_position", targets, rawValues, false);
    const turnedOn = wroteGoal && publishTorqueEnable(targets, true, false);
    if (turnedOn) {
      // Reinforce once after torque ON to hold exact captured value.
      window.setTimeout(() => {
        publishSyncWriteItem("goal_position", targets, rawValues, false);
      }, 120);
    }
    return turnedOn;
  };

  const commitJointDraft = (name, value) => {
    if(value === "" || value === null) { setJointDrafts(p => { const n={...p}; delete n[jointDraftKey(name)]; return n; }); return; }
    const num = Number(value);
    if(!Number.isFinite(num)) return;
    updateActiveStep((s) => {
      if(!s.positions) s.positions = {};
      setRawPosition(s.positions, name, jointIdMap, toRawDegrees(num));
    }, {
      label: `Adjust ${name}`,
      coalesceKey: `joint-${name}`,
    });
    setJointDrafts(p => { const n={...p}; delete n[jointDraftKey(name)]; return n; });
  };

  const handleJointToggleOff = (name) => {
    const currentlyOff = manualOffJoints.has(name);
    const nextEnable = currentlyOff;
    if (!publishTorqueEnable([name], nextEnable)) return;
    setManualOffJoints(prev => {
      const next = new Set(prev);
      if (nextEnable) next.delete(name);
      else next.add(name);
      return next;
    });
    setStatus(`Torque ${nextEnable ? "ON" : "OFF"}: ${name}`);
    setStatusError(false);
  };

  const handleTorqueOnAllEditor = () => {
    const targets = editableJointNames.length ? editableJointNames : JOINT_ORDER;
    if (!publishTorqueEnable(targets, true)) return;
    setManualOffJoints(new Set());
    setStatus("Torque ON all (editor)");
    setStatusError(false);
  };

  const sendStepPoseToWebots = (step) => {
    if(!step || !jointPubRef.current) return;
    const pose = buildPose(step.positions || {}, livePoseRef.current);
    jointPubRef.current.publish(new ROSLIB.Message({ data: JOINT_ORDER.map(n => pose[n]??0) }));
    setStatus("Sent step to Webots"); setStatusError(false);
  };

  const buildScratchYaml = () => {
    if (!activePage || !activeStep) return null;
    const scratchIndex = Number(scratchPageIndex);
    if (!Number.isFinite(scratchIndex) || scratchIndex < 1 || scratchIndex > 255) {
      setStatus("Scratch page must be 1-255"); setStatusError(true); return null;
    }
    const header = activePage.header || {};
    const schedule = header.schedule === 10 || header.schedule === "time" ? 10 : 0;
    const scratchHeader = {
      repeat: Number(header.repeat) || 1,
      schedule,
      speed: Number(header.speed) || 0,
      accel: Number(header.accel) || 0,
      next: 0, exit: 0
    };
    const stepCopy = cloneData(activeStep.positions || {});
    const step = { index: 0, pause: activeStep.pause ?? 0, time: activeStep.time ?? 0, positions: stepCopy };
    const payload = { pages: [{ index: scratchIndex, name: `scratch_step`, header: scratchHeader, steps: [step] }] };
    return { scratchIndex, yaml: YAML.dump(payload, { sortKeys: false, lineWidth: -1 }) };
  };

  const buildRangeScratchYaml = () => {
    if (!activePage) return null;
    const scratchIndex = Number(scratchPageIndex);
    const startRaw = Number(rangeStart);
    const endRaw = Number(rangeEnd);
    if (!Number.isInteger(startRaw) || !Number.isInteger(endRaw) || startRaw > endRaw) {
      setStatus("Invalid range"); setStatusError(true); return null;
    }
    
    const steps = [];
    for (let idx = startRaw; idx <= endRaw; idx++) {
      const sourceStep = activePage.steps?.find(s => Number(s.index) === idx) || activePage.steps?.[idx];
      if (!sourceStep) { setStatus(`Missing step ${idx}`); setStatusError(true); return null; }
      steps.push({ index: steps.length, pause: sourceStep.pause ?? 0, time: sourceStep.time ?? 0, positions: cloneData(sourceStep.positions || {}) });
    }

    const header = activePage.header || {};
    const schedule = header.schedule === 10 || header.schedule === "time" ? 10 : 0;
    const scratchHeader = {
      repeat: Number(header.repeat) || 1,
      schedule,
      speed: Number(header.speed) || 0,
      accel: Number(header.accel) || 0,
      next: 0, exit: 0
    };

    const payload = { pages: [{ index: scratchIndex, name: `scratch_range`, header: scratchHeader, steps }] };
    return { scratchIndex, yaml: YAML.dump(payload, { sortKeys: false, lineWidth: -1 }) };
  };

  const queueScratchRun = (scratch, label) => {
    if (!requestRef.current) { setStatus("ROS not connected"); setStatusError(true); return; }
    if (!scratch) return;
    const requestId = `scratch-${Date.now()}`;
    pendingRunRef.current = { requestId, pageIndex: scratch.scratchIndex };
    sendRequest({ action: "apply", yaml: scratch.yaml, request_id: requestId });
    setStatus(label); setStatusError(false);
  };

  const handleRunStep = () => {
    const scratch = buildScratchYaml();
    queueScratchRun(scratch, "Running Step...");
  };

  const handleRunRange = () => {
    const scratch = buildRangeScratchYaml();
    queueScratchRun(scratch, "Running Sequence...");
  };

  const handleRunPage = () => {
    const pageIndex = activePage?.index ?? selectedPageIndex;
    if (pageIndex === null || pageIndex === undefined || !actionPagePubRef.current) return;
    const run = () => actionPagePubRef.current?.publish(new ROSLIB.Message({ data: pageIndex }));
    if (autoEnableAction) {
      enableModulePubRef.current?.publish(new ROSLIB.Message({ data: "action_module" }));
      setTimeout(run, 300);
    } else {
      run();
    }
  };

  const handleStopPage = () => {
    if (!actionPagePubRef.current) return;
    actionPagePubRef.current.publish(new ROSLIB.Message({ data: -1 }));
    setStatus("Stop sent");
  };

  const handleBrakePage = () => {
    if (!actionPagePubRef.current) return;
    actionPagePubRef.current.publish(new ROSLIB.Message({ data: -2 }));
    setStatus("Brake sent");
  };

  const handleStepClick = (page, step, index) => {
    // Record Logic
    if (recordEnabled && page) {
      const now = performance.now();
      const rec = recordRef.current;
      if (rec.lastPageIndex === page.index && rec.lastTime !== null && rec.lastStepIndex !== null) {
        const delta = (now - rec.lastTime)/1000;
        const speed = resolveSpeed(page.header);
        const ticks = recordMode === "time" ? secondsToTimeTicks(delta, speed) : secondsToPauseTicks(delta, speed);
        const targetLabel = recordMode === "time" ? "time" : "pause";
        // Update previous step
        updateYamlData("Record step timing", (draft) => {
          const p = draft.pages.find(i => i.index === page.index);
          const s = p.steps.find(x => Number(x.index) === Number(rec.lastStepIndex)) || p.steps[rec.lastStepIndex];
          if(s) s[targetLabel] = Math.max(1, ticks);
        }, { coalesceKey: "record-step-timing" });
        setRecordLastDelta(delta); setRecordLastTicks(ticks);
      }
      rec.lastTime = now; rec.lastStepIndex = index; rec.lastPageIndex = page.index;
    }

    const pose = buildPose(step.positions || {}, livePoseRef.current);
    setSelectedPageIndex(page.index); setSelectedStepIndex(index); setPreviewPose(pose);
    if(sendStepToWebots && jointPubRef.current) {
      jointPubRef.current.publish(new ROSLIB.Message({ data: JOINT_ORDER.map(n => pose[n]??0) }));
    }
  };

  // --- Render Helpers ---
  const activeHeader = activePage?.header || {};
  const activeStepLabel = activeStep ? (activeStep.index ?? selectedStepIndex) : "";
  const timelineMax = Math.max(0, timelineData.total);
  const timelineClamped = clampNumber(Number(timelinePos) || 0, 0, timelineMax);
  const timelineSegment = getTimelineSegment(timelineClamped, timelineData.durations, timelineData.cumulative);
  const timelineLeft = timelineSegment.index;
  const timelineRight = Math.min(timelineLeft + 1, Math.max(0, (activePage?.steps?.length || 0) - 1));
  const timelinePct = Math.round(timelineSegment.t * 100);
  const timelineHasRange = (activePage?.steps?.length || 0) > 1;
  const timelineLabel = timelineMax > 0 && (activePage?.steps?.length || 0) > 1
    ? `${timelineLeft}→${timelineRight} (${timelinePct}%)`
    : `Step ${timelineLeft}`;
  const undoCount = historyRef.current.undo.length;
  const redoCount = historyRef.current.redo.length;
  const recentSnapshots = snapshots.slice(0, 6);

  return (
    <div className="flex flex-col lg:flex-row h-full gap-6 p-2 overflow-y-auto lg:overflow-hidden bg-[#f8fafc]">
      {/* LEFT COLUMN: Lists & YAML */}
      <div className="flex flex-col w-full lg:w-1/4 lg:min-w-[280px] gap-4 h-full">
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col flex-1 overflow-hidden min-h-[300px]">
          <div className="p-4 border-b border-gray-100 flex justify-between items-center">
            <h2 className="text-lg font-bold font-display text-gray-800">Pages</h2>
            <div className="flex items-center gap-2">
              <button
                className="text-xs font-semibold text-gray-700 bg-gray-100 hover:bg-gray-200 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                onClick={handleAddPage}
                disabled={editorDisabled}
                title="Add new page"
              >
                <Plus size={12} /> Add Page
              </button>
              <button
                className="text-xs font-semibold text-gray-700 bg-gray-100 hover:bg-gray-200 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                onClick={handleDuplicatePage}
                disabled={editorDisabled || !activePage}
                title="Duplicate selected page with all steps"
              >
                Duplicate
              </button>
              <button
                className="text-xs font-semibold text-amber-700 bg-amber-50 hover:bg-amber-100 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                onClick={handleMirrorPage}
                disabled={editorDisabled || !activePage}
                title="Mirror all steps in selected page (true left/right)"
              >
                Mirror Page
              </button>
              <button
                className="text-xs font-semibold text-red-600 bg-red-50 hover:bg-red-100 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                onClick={handleDeletePage}
                disabled={editorDisabled || !activePage}
                title="Delete selected page"
              >
                <Trash2 size={12} /> Delete
              </button>
              <button className="text-xs font-medium text-undip-blue hover:underline" onClick={() => { setPreviewPose(null); setSelectedStepIndex(null); }}>
                Reset Live
              </button>
            </div>
          </div>
          <div className="p-2">
            <input 
              type="text" 
              placeholder="Filter pages..." 
              className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:border-undip-blue"
              value={pageFilter}
              onChange={e => setPageFilter(e.target.value)}
            />
          </div>
          <div className="flex-1 overflow-y-auto px-2 pb-2 space-y-1 custom-scrollbar">
            {visiblePages.length === 0 && <div className="text-center py-4 text-gray-400 text-sm">No pages found.</div>}
            {visiblePages.map(p => (
              <button 
                key={p.index}
                onClick={() => { setSelectedPageIndex(p.index); setSelectedStepIndex(null); setPreviewPose(null); }}
                className={`w-full flex items-center justify-between px-3 py-2 rounded-lg text-left transition-colors ${selectedPageIndex === p.index ? 'bg-undip-blue text-white shadow-md' : 'hover:bg-gray-50 text-gray-700'}`}
              >
                <div className="flex flex-col overflow-hidden">
                  <span className="font-bold text-sm truncate">{p.name || "Untitled"}</span>
                  <span className={`text-[10px] font-mono ${selectedPageIndex === p.index ? 'text-white/70' : 'text-gray-400'}`}>ID: {p.index}</span>
                </div>
                <span className={`text-xs font-mono px-2 py-0.5 rounded-full ${selectedPageIndex === p.index ? 'bg-white/20' : 'bg-gray-100'}`}>
                  {p.steps?.length || 0}
                </span>
              </button>
            ))}
          </div>
          <div className="p-2 border-t border-gray-100 bg-gray-50/50 flex-1 overflow-y-auto max-h-[40%] custom-scrollbar">
            <div className="flex items-center justify-between mb-2 px-1">
              <div className="text-xs font-bold text-gray-400 uppercase tracking-wider">
                Steps {activePage ? `(Page ${activePage.index})` : ""}
              </div>
              <div className="flex items-center gap-2">
                <button
                  className="text-[10px] font-semibold text-gray-700 bg-gray-100 hover:bg-gray-200 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                  onClick={handleAddStep}
                  disabled={editorDisabled || !activePage}
                  title="Insert step after current"
                >
                  <Plus size={10} /> Add Step
                </button>
                <button
                  className="text-[10px] font-semibold text-gray-700 bg-gray-100 hover:bg-gray-200 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                  onClick={handleCapturePoseToNewStep}
                  disabled={editorDisabled || !activePage || selectedStepIndex === null}
                  title="Capture live pose to a new step"
                >
                  Capture New
                </button>
                <button
                  className="text-[10px] font-semibold text-gray-700 bg-gray-100 hover:bg-gray-200 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                  onClick={handleDuplicateStep}
                  disabled={editorDisabled || !activePage || selectedStepIndex === null}
                  title="Duplicate selected step"
                >
                  Duplicate
                </button>
                <button
                  className="text-[10px] font-semibold text-red-600 bg-red-50 hover:bg-red-100 px-2 py-1 rounded flex items-center gap-1 disabled:opacity-50"
                  onClick={handleDeleteStep}
                  disabled={editorDisabled || !activePage || selectedStepIndex === null}
                  title="Delete selected step"
                >
                  <Trash2 size={10} /> Delete
                </button>
              </div>
            </div>
            {!activePage ? (
              <div className="text-center py-4 text-gray-400 text-xs">Select a page</div>
            ) : (
              <div className="space-y-1">
                {activePage.steps?.map((step, idx) => {
                  const sIdx = step.index ?? idx;
                  return (
                    <button
                      key={sIdx}
                      onClick={() => handleStepClick(activePage, step, sIdx)}
                      className={`w-full flex items-center justify-between px-3 py-2 rounded-lg text-xs font-mono transition-colors ${selectedStepIndex === sIdx ? 'bg-accent-yellow text-black shadow-sm font-bold' : 'bg-white border border-gray-200 hover:border-gray-300 text-gray-600'}`}
                    >
                      <span>Step {sIdx}</span>
                      <span className="opacity-60">T:{step.time} P:{step.pause}</span>
                    </button>
                  );
                })}
                {!activePage.steps?.length && <div className="text-center py-2 text-gray-400 text-xs">No steps</div>}
              </div>
            )}
          </div>
        </div>

        {/* Snapshot Lab */}
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-3">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-xs font-bold text-gray-600 uppercase tracking-wider">Snapshot Lab</h3>
            <span className="text-[10px] font-mono text-gray-400">{snapshots.length}/{SNAPSHOT_LIMIT}</span>
          </div>
          <div className="flex items-center gap-2 mb-2">
            <input
              type="text"
              placeholder="Snapshot name"
              value={snapshotName}
              onChange={(e) => setSnapshotName(e.target.value)}
              className="flex-1 px-2 py-1.5 bg-gray-50 border border-gray-200 rounded text-xs font-mono focus:outline-none focus:border-undip-blue"
            />
            <button
              onClick={handleCreateSnapshot}
              className="px-2.5 py-1.5 bg-undip-blue text-white rounded text-xs font-bold flex items-center gap-1 hover:bg-opacity-90"
              title="Save current document as snapshot"
            >
              <Save size={12} />
              Save
            </button>
          </div>
          <div className="space-y-1 max-h-[150px] overflow-y-auto custom-scrollbar pr-1">
            {recentSnapshots.length === 0 && (
              <div className="text-[11px] text-gray-400 py-1">No snapshots yet.</div>
            )}
            {recentSnapshots.map((snapshot) => (
              <div key={snapshot.id} className="border border-gray-200 rounded-md px-2 py-1.5 bg-gray-50">
                <div className="flex items-center justify-between gap-2">
                  <div className="min-w-0">
                    <div className="text-xs font-semibold text-gray-700 truncate">{snapshot.name}</div>
                    <div className="text-[10px] font-mono text-gray-400">
                      {new Date(snapshot.ts).toLocaleString()}
                    </div>
                  </div>
                  <div className="flex items-center gap-1">
                    <button
                      onClick={() => handleRestoreSnapshot(snapshot)}
                      className="px-1.5 py-0.5 text-[10px] font-semibold bg-white border border-gray-300 rounded hover:border-undip-blue hover:text-undip-blue"
                    >
                      Restore
                    </button>
                    <button
                      onClick={() => handleRenameSnapshot(snapshot)}
                      className="px-1.5 py-0.5 text-[10px] font-semibold bg-white border border-gray-300 rounded hover:border-gray-500"
                    >
                      Rename
                    </button>
                    <button
                      onClick={() => handleDeleteSnapshot(snapshot.id)}
                      className="px-1.5 py-0.5 text-[10px] font-semibold bg-red-50 border border-red-200 text-red-600 rounded hover:bg-red-100"
                    >
                      Del
                    </button>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* YAML Editor Toggle */}
        <div className={`bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col transition-all ${showYaml ? 'h-[300px]' : 'h-auto'}`}>
          <div className="p-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50 rounded-t-2xl">
            <div className="flex items-center gap-2">
              <span className="text-sm font-bold text-gray-700">YAML</span>
              {hasLocalDraft && (
                <span className="text-[10px] font-semibold text-emerald-700 bg-emerald-100 px-2 py-0.5 rounded-full">
                  Draft Saved
                </span>
              )}
              <button onClick={() => setShowYaml(!showYaml)} className="text-xs text-undip-blue hover:underline">
                {showYaml ? "Hide" : "Show"}
              </button>
            </div>
            <div className="flex gap-2">
              <button title="Apply" onClick={handleApply} className="p-1.5 bg-undip-blue text-white rounded hover:bg-opacity-90"><Upload size={14}/></button>
              <button title="Export" onClick={handleExport} className="p-1.5 bg-white border border-gray-300 text-gray-700 rounded hover:bg-gray-50"><Download size={14}/></button>
            </div>
          </div>
          {showYaml && (
            <div className="flex-1 p-0 overflow-hidden relative">
              <textarea 
                className="w-full h-full resize-none p-3 text-xs font-mono bg-[#1e1e1e] text-gray-300 focus:outline-none"
                value={yamlText}
                onChange={(e) => {
                  yamlDirtyRef.current = true;
                  setYamlText(e.target.value);
                }}
                spellCheck="false"
              />
              <div className={`absolute bottom-0 left-0 right-0 px-2 py-1 text-[10px] font-mono border-t border-white/10 ${statusError ? "bg-red-900/80 text-red-200" : "bg-black/50 text-green-400"}`}>
                {status || "Ready"} {lastHistoryLabel && `| Last: ${lastHistoryLabel}`} {parseError && `| ${parseError}`}
              </div>
            </div>
          )}
        </div>
      </div>

      {/* MIDDLE COLUMN: Editors */}
      <div className="flex flex-col flex-1 gap-4 h-full lg:overflow-y-auto custom-scrollbar pb-2">
        {/* Page Settings */}
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-5">
          <div className="flex justify-between items-center mb-4">
            <h2 className="text-lg font-bold font-display text-gray-800">Page Configuration</h2>
            <div className="flex items-center gap-2">
              <label className="flex items-center gap-2 text-xs font-medium text-gray-500 cursor-pointer">
                <input type="checkbox" checked={autoEnableAction} onChange={e => setAutoEnableAction(e.target.checked)} className="accent-undip-blue"/>
                Auto-Enable
              </label>
              <div className="h-4 w-px bg-gray-300 mx-1"></div>
              <button onClick={() => enableModulePubRef.current?.publish(new ROSLIB.Message({data:"action_module"}))} className="p-1.5 text-gray-500 hover:text-undip-blue hover:bg-blue-50 rounded" title="Enable Module"><RefreshCw size={16}/></button>
              <button onClick={handleRunPage} className="p-1.5 text-green-600 hover:bg-green-50 rounded" title="Run Page"><Play size={16}/></button>
              <button onClick={() => actionPagePubRef.current?.publish(new ROSLIB.Message({data: -1}))} className="p-1.5 text-red-600 hover:bg-red-50 rounded" title="Stop"><Square size={16}/></button>
            </div>
          </div>
          
          {!activePage ? (
            <div className="text-center py-8 text-gray-400 text-sm bg-gray-50 rounded-xl border border-dashed border-gray-200">No page selected</div>
          ) : (
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
              <div className="col-span-2">
                <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Name</label>
                <input type="text" value={activePage.name || ""} onChange={e => updateActivePage(p => p.name = e.target.value, { label: "Rename page", coalesceKey: "page-name" })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-bold text-gray-700 focus:border-undip-blue focus:ring-2 focus:ring-undip-blue/10 outline-none transition-all" />
              </div>
              <div>
                <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Schedule</label>
                <select value={scheduleMode} onChange={e => updateActivePage(p => { p.header = p.header||{}; p.header.schedule = e.target.value === "time" ? 10 : 0; }, { label: "Change schedule", coalesceKey: "page-schedule" })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none">
                  <option value="speed">Speed</option>
                  <option value="time">Time</option>
                </select>
              </div>
              <div>
                <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Repeat</label>
                <input type="number" value={activePage.header?.repeat ?? 0} onChange={e => updateActivePage(p => { p.header=p.header||{}; p.header.repeat = Number(e.target.value); }, { label: "Change repeat", coalesceKey: "page-repeat" })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none" />
              </div>
              {/* Other header fields simplified for brevity, assume similar pattern */}
              {["speed", "accel", "next", "exit"].map(f => (
                <div key={f}>
                  <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">{f}</label>
                  <input type="number" value={activePage.header?.[f] ?? 0} onChange={e => updateActivePage(p => { p.header=p.header||{}; p.header[f] = Number(e.target.value); }, { label: `Change ${f}`, coalesceKey: `page-${f}` })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none" />
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Step Editor */}
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-5 flex-1 flex flex-col min-h-[400px]">
          <div className="flex justify-between items-center mb-4">
            <h2 className="text-lg font-bold font-display text-gray-800">Step Editor <span className="text-gray-400 font-mono text-sm ml-2">{activeStepLabel !== "" ? `Step ${activeStepLabel}` : ""}</span></h2>
            <div className="flex gap-2">
              <button
                onClick={handleUndoDocument}
                disabled={!canUndo}
                className="px-2 py-1.5 text-gray-500 hover:text-undip-blue disabled:opacity-30 text-xs font-semibold flex items-center gap-1"
                title={`Undo (${undoCount})`}
              >
                <Undo size={16}/>
                {undoCount > 0 ? `(${undoCount})` : ""}
              </button>
              <button
                onClick={handleRedoDocument}
                disabled={!canRedo}
                className="px-2 py-1.5 text-gray-500 hover:text-undip-blue disabled:opacity-30 text-xs font-semibold flex items-center gap-1"
                title={`Redo (${redoCount})`}
              >
                <Redo size={16}/>
                {redoCount > 0 ? `(${redoCount})` : ""}
              </button>
              <div className="h-4 w-px bg-gray-300 mx-1 self-center"></div>
              <label className="flex items-center gap-1.5 text-xs font-medium text-gray-600 bg-gray-100 px-2 py-1 rounded cursor-pointer hover:bg-gray-200">
                <input type="checkbox" checked={recordEnabled} onChange={e => setRecordEnabled(e.target.checked)} className="accent-red-500"/>
                <span className={recordEnabled ? "text-red-500 animate-pulse" : ""}>REC</span>
              </label>
              <label className="flex items-center gap-1.5 text-xs font-medium text-gray-600 bg-gray-100 px-2 py-1 rounded cursor-pointer hover:bg-gray-200">
                <input type="checkbox" checked={sendStepToWebots} onChange={e => setSendStepToWebots(e.target.checked)} className="accent-undip-blue"/>
                Auto-Send
              </label>
              <label className="flex items-center gap-2 text-xs font-medium text-gray-600 bg-gray-100 px-2 py-1 rounded">
                <span className="uppercase tracking-wider text-[10px] text-gray-400">Limits</span>
                <select
                  value={limitPreset}
                  onChange={e => setLimitPreset(e.target.value)}
                  className="bg-transparent text-xs font-semibold text-gray-700 outline-none"
                >
                  <option value="default">Default</option>
                  <option value="walking">Walking</option>
                  <option value="gesture">Gesture</option>
                </select>
              </label>
              <button
                onClick={() => setJointGridColumns(jointGridColumns === 2 ? 1 : 2)}
                className="px-2 py-1 text-xs font-medium text-gray-600 bg-gray-100 rounded hover:bg-gray-200"
                title="Toggle joint layout"
              >
                {jointGridColumns === 2 ? "2 Row" : "1 Row"}
              </button>
            </div>
          </div>

          {!activeStep ? (
            <div className="flex-1 flex flex-col items-center justify-center bg-gray-50 rounded-xl border border-dashed border-gray-200 text-gray-400">
              <span className="text-sm">Select a step to edit joints</span>
            </div>
          ) : (
            <div className="flex flex-col h-full gap-4">
              <div className="flex gap-4 p-3 bg-gray-50 rounded-xl border border-gray-100">
                <div className="flex-1">
                  <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Time</label>
                  <input type="number" value={activeStep.time ?? 0} onChange={e => updateActiveStep(s => s.time = Number(e.target.value), { label: "Edit step time", coalesceKey: "step-time" })} className="w-full px-3 py-2 bg-white border border-gray-200 rounded-lg text-sm font-mono outline-none focus:border-undip-blue" />
                </div>
                <div className="flex-1">
                  <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Pause</label>
                  <input type="number" value={activeStep.pause ?? 0} onChange={e => updateActiveStep(s => s.pause = Number(e.target.value), { label: "Edit step pause", coalesceKey: "step-pause" })} className="w-full px-3 py-2 bg-white border border-gray-200 rounded-lg text-sm font-mono outline-none focus:border-undip-blue" />
                </div>
                <div className="flex items-end gap-2">
                  <button onClick={() => sendStepPoseToWebots(activeStep)} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-undip-blue hover:text-undip-blue rounded-lg text-xs font-bold flex items-center gap-1 transition-all"><Send size={14}/> Send</button>
                  <button onClick={handleCapturePose} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-green-500 hover:text-green-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Capture</button>
                  <button onClick={handleCapturePoseToNewStep} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-green-500 hover:text-green-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Capture New</button>
                  <button onClick={handleTorqueOnAllEditor} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-blue-500 hover:text-blue-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Torque ON All</button>
                  <button title="Mirror selected step (true left/right)" onClick={handleSwapLeftRight} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-amber-500 hover:text-amber-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Swap L/R</button>
                </div>
              </div>

              <div className="flex flex-wrap items-end gap-2 p-3 bg-gray-50 rounded-xl border border-gray-100">
                <div className="min-w-[110px]">
                  <label className="block text-[10px] font-bold text-gray-400 uppercase tracking-wider mb-1">Src Page</label>
                  <select
                    value={copySourcePageIndex}
                    onChange={e => setCopySourcePageIndex(e.target.value)}
                    className="w-full px-2 py-2 bg-white border border-gray-200 rounded-lg text-xs font-mono text-gray-700 outline-none focus:border-undip-blue"
                  >
                    <option value="">-</option>
                    {sortedPages.map(page => (
                      <option key={`src-page-${page.index}`} value={String(page.index)}>
                        {page.index}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="w-[90px]">
                  <label className="block text-[10px] font-bold text-gray-400 uppercase tracking-wider mb-1">Src Step</label>
                  <input
                    type="number"
                    min={0}
                    max={ACTION_MAX_STEPS - 1}
                    value={copySourceStepIndex}
                    onChange={e => setCopySourceStepIndex(e.target.value)}
                    className="w-full px-2 py-2 bg-white border border-gray-200 rounded-lg text-xs font-mono text-gray-700 outline-none focus:border-undip-blue"
                  />
                </div>
                <div className="text-xs font-semibold text-gray-400 pb-2">to</div>
                <div className="min-w-[110px]">
                  <label className="block text-[10px] font-bold text-gray-400 uppercase tracking-wider mb-1">Target Page</label>
                  <select
                    value={copyTargetPageIndex}
                    onChange={e => setCopyTargetPageIndex(e.target.value)}
                    className="w-full px-2 py-2 bg-white border border-gray-200 rounded-lg text-xs font-mono text-gray-700 outline-none focus:border-undip-blue"
                  >
                    <option value="">-</option>
                    {sortedPages.map(page => (
                      <option key={`target-page-${page.index}`} value={String(page.index)}>
                        {page.index}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="w-[90px]">
                  <label className="block text-[10px] font-bold text-gray-400 uppercase tracking-wider mb-1">Target Step</label>
                  <input
                    type="number"
                    min={0}
                    max={ACTION_MAX_STEPS - 1}
                    value={copyTargetStepIndex}
                    onChange={e => setCopyTargetStepIndex(e.target.value)}
                    className="w-full px-2 py-2 bg-white border border-gray-200 rounded-lg text-xs font-mono text-gray-700 outline-none focus:border-undip-blue"
                  />
                </div>
                <button
                  onClick={handleCopyStepToTarget}
                  disabled={editorDisabled || !sortedPages.length}
                  className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-undip-blue hover:text-undip-blue rounded-lg text-xs font-bold flex items-center gap-1 transition-all disabled:opacity-50"
                  title="Copy full step (positions + time + pause) from source to target"
                >
                  <Copy size={14} />
                  Copy Full Step
                </button>
              </div>

              <div className="flex flex-col gap-2 p-3 bg-gray-50 rounded-xl border border-gray-100">
                <div className="flex items-center justify-between">
                  <div className="text-xs font-bold text-gray-400 uppercase tracking-wider">Timeline Preview</div>
                  <div className="flex items-center gap-2">
                    <button
                      onClick={() => setTimelinePlaying(p => !p)}
                      disabled={!activePage || !timelineHasRange}
                      className="px-2 py-1 bg-white border border-gray-200 text-gray-700 hover:border-undip-blue hover:text-undip-blue rounded text-xs font-bold flex items-center gap-1 disabled:opacity-50"
                      title={timelinePlaying ? "Pause preview" : "Play preview"}
                    >
                      {timelinePlaying ? <Square size={12}/> : <Play size={12}/>}
                      {timelinePlaying ? "Pause" : "Play"}
                    </button>
                    <button
                      onClick={() => setTimelineLoop(p => !p)}
                      disabled={!activePage || !timelineHasRange}
                      className={`px-2 py-1 border rounded text-xs font-bold flex items-center gap-1 disabled:opacity-50 ${timelineLoop ? "bg-undip-blue text-white border-undip-blue" : "bg-white text-gray-700 border-gray-200 hover:border-undip-blue hover:text-undip-blue"}`}
                      title="Loop preview"
                    >
                      Loop
                    </button>
                    <label className="text-[10px] font-bold text-gray-400 uppercase tracking-wider">Speed</label>
                    <input
                      type="range"
                      min="0.25"
                      max="4"
                      step="0.25"
                      value={timelineSpeed}
                      onChange={e => setTimelineSpeed(Number(e.target.value))}
                      className="w-24 accent-undip-blue"
                    />
                    <span className="text-[10px] font-mono text-gray-500 w-8 text-right">{timelineSpeed.toFixed(2)}x</span>
                  </div>
                </div>
                <div className="flex items-center gap-3">
                  <input
                    type="range"
                    min="0"
                    max={timelineMax}
                    step="0.01"
                    value={timelineClamped}
                    onChange={e => { setTimelinePos(Number(e.target.value)); if (timelinePlaying) setTimelinePlaying(false); }}
                    onMouseUp={snapTimelineToNearestStep}
                    onTouchEnd={snapTimelineToNearestStep}
                    disabled={!activePage || !timelineHasRange}
                    className="flex-1 accent-undip-blue"
                  />
                  <span className="text-[10px] font-mono text-gray-500 w-24 text-right">{timelineLabel}</span>
                </div>
              </div>

              {/* Joint Grid */}
              <div className="flex-1 overflow-y-auto custom-scrollbar pr-2">
                <div className={`grid gap-3 ${jointGridColumns === 2 ? "grid-cols-1 md:grid-cols-2" : "grid-cols-1"}`}>
                  {jointNames.map(name => {
                    const raw = lookupRawPosition(activeStep.positions||{}, name, jointIdMap);
                    const norm = normalizeRaw(raw);
                    const legacyOff = isTorqueOff(raw);
                    const isOff = manualOffJoints.has(name);
                    const deg = norm === null ? 0 : toDegrees(norm);
                    const key = jointDraftKey(name);
                    const draft = jointDrafts[key] ?? (norm===null ? "" : deg.toFixed(1));
                    const displayId = JOINT_ID[name] ?? jointIdMap[name];
                    const idText = displayId ? `ID ${displayId}` : "ID ?";
                    const limit = activeLimitMap[name];
                    const outOfRange = Boolean(limit && !isOff && norm !== null && (deg < limit.min || deg > limit.max));
                    const rowClass = isOff
                      ? "bg-red-50 border-red-100 opacity-70"
                      : legacyOff
                        ? "bg-orange-50 border-orange-100"
                      : outOfRange
                        ? "bg-amber-50 border-amber-200"
                        : "bg-white border-gray-100 hover:border-gray-300";
                    const limitText = limit ? `${formatLimitDeg(limit.min)}°..${formatLimitDeg(limit.max)}°` : "";
                    
                    return (
                      <div key={name} className={`grid grid-cols-12 items-center gap-2 p-2 rounded-lg border transition-all ${rowClass}`}>
                        <div className="col-span-12 sm:col-span-4 min-w-0 flex flex-col">
                          <span className="text-[10px] font-mono text-gray-400 truncate">{name}</span>
                          <span className="text-xs font-bold text-gray-700 truncate" title={formatJointLabel(name)}>{formatJointLabel(name)}</span>
                          <span className="text-[10px] text-gray-500 truncate" title={formatJointLabelId(name)}>{idText} • {formatJointLabelId(name)}</span>
                          {outOfRange && (
                            <span className="text-[10px] font-bold text-amber-600 uppercase truncate" title="Outside recommended joint limit">
                              Limit {limitText}
                            </span>
                          )}
                        </div>
                        <input
                          type="range" min="-180" max="180" step="0.5"
                          value={Number.isFinite(Number(draft)) ? Number(draft) : deg}
                          onChange={e => commitJointDraft(name, e.target.value)}
                          disabled={editorDisabled || isOff}
                          title={limitText ? `Limit ${limitText}` : undefined}
                          className="col-span-12 sm:col-span-5 w-full min-w-0 accent-undip-blue h-1.5 bg-gray-200 rounded-full appearance-none cursor-pointer"
                        />
                        <input
                          type="number"
                          value={draft}
                          onChange={e => handleJointDraftChange(name, e.target.value)}
                          onBlur={e => commitJointDraft(name, e.target.value)}
                          onKeyDown={e => e.key === "Enter" && commitJointDraft(name, e.target.value)}
                          disabled={editorDisabled || isOff}
                          title={limitText ? `Limit ${limitText}` : undefined}
                          className="col-span-6 sm:col-span-2 w-full px-2 py-1 bg-gray-50 border border-gray-200 rounded text-xs font-mono text-right focus:outline-none focus:border-undip-blue"
                        />
                        <button
                          onClick={() => handleJointToggleOff(name)}
                          className={`col-span-6 sm:col-span-1 w-full px-2 py-1 rounded text-[10px] font-bold uppercase transition-colors ${isOff ? 'bg-red-100 text-red-600' : 'bg-gray-100 text-gray-500 hover:bg-gray-200'}`}
                        >
                          {isOff ? "OFF" : "ON"}
                        </button>
                      </div>
                    );
                  })}
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* RIGHT COLUMN: 3D View & Connection */}
      <div className="flex flex-col w-full lg:w-1/4 lg:min-w-[300px] gap-4 h-full">
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden flex flex-col min-h-[300px] lg:h-[400px]">
          <div className="p-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50">
            <h2 className="text-sm font-bold text-gray-700">3D Preview</h2>
            <div className="flex gap-1">
              <button onClick={() => rotateView(-45)} className="p-1.5 text-gray-500 hover:bg-white rounded"><RotateCcw size={14}/></button>
              <button onClick={() => rotateView(45)} className="p-1.5 text-gray-500 hover:bg-white rounded"><RotateCw size={14}/></button>
              <button onClick={() => setViewerEnabled(!viewerEnabled)} className="p-1.5 text-gray-500 hover:bg-white rounded">{viewerEnabled ? <Eye size={14}/> : <EyeOff size={14}/>}</button>
            </div>
          </div>
          <div className="relative flex-1 bg-slate-50 min-h-[250px]">
            {viewerEnabled ? (
              <div ref={viewerRef} className="w-full h-full" />
            ) : (
              <div className="w-full h-full flex flex-col items-center justify-center text-gray-400">
                <EyeOff size={32} className="mb-2 opacity-20"/>
                <span className="text-xs">Preview Hidden</span>
              </div>
            )}
            <div className="absolute bottom-2 left-2 right-2 flex justify-center gap-2">
              {[{l:"Upright", v:upright, s:setUpright}, {l:"Flat", v:layFlat, s:setLayFlat}, {l:"Mirror", v:mirrorView, s:setMirrorView}].map(opt => (
                <button 
                  key={opt.l}
                  onClick={() => opt.s(!opt.v)}
                  className={`px-3 py-1 rounded-full text-[10px] font-bold backdrop-blur-sm border transition-all ${opt.v ? 'bg-undip-blue/90 text-white border-transparent' : 'bg-white/80 text-gray-600 border-gray-200'}`}
                >
                  {opt.l}
                </button>
              ))}
            </div>
          </div>
        </div>

        {/* Scratch Pad / Runner */}
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-5 flex-1 overflow-y-auto">
          <SectionHeader title="Scratch Runner">
            <span className="text-[10px] font-mono bg-yellow-100 text-yellow-700 px-2 py-0.5 rounded">TEST MODE</span>
          </SectionHeader>
          <div className="space-y-4">
            <div>
              <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Target Page ID</label>
              <input type="number" value={scratchPageIndex} onChange={e => setScratchPageIndex(Number(e.target.value))} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:border-accent-yellow" />
            </div>
            
            <div className="p-3 bg-gray-50 rounded-xl border border-gray-100 space-y-3">
              <h4 className="text-xs font-bold text-gray-500 uppercase">Single Step</h4>
              <button 
                onClick={() => queueScratchRun(buildScratchYaml(), "Running Step...")}
                disabled={!activeStep || rosState !== "connected"}
                className="w-full py-2 bg-white border border-gray-200 hover:border-accent-yellow hover:text-yellow-700 text-gray-600 rounded-lg text-sm font-bold transition-all shadow-sm disabled:opacity-50"
              >
                Run Current Step
              </button>
            </div>

            <div className="p-3 bg-gray-50 rounded-xl border border-gray-100 space-y-3">
              <h4 className="text-xs font-bold text-gray-500 uppercase">Sequence Range</h4>
              <div className="flex items-center gap-2">
                <input type="number" placeholder="Start" value={rangeStart} onChange={e => setRangeStart(e.target.value)} className="w-full px-2 py-1 bg-white border border-gray-200 rounded text-center text-sm font-mono" />
                <span className="text-gray-400 text-xs">to</span>
                <input type="number" placeholder="End" value={rangeEnd} onChange={e => setRangeEnd(e.target.value)} className="w-full px-2 py-1 bg-white border border-gray-200 rounded text-center text-sm font-mono" />
              </div>
              <button 
                onClick={handleRunRange}
                disabled={!activePage || rosState !== "connected"}
                className="w-full py-2 bg-undip-blue text-white hover:bg-opacity-90 rounded-lg text-sm font-bold transition-all shadow-sm disabled:opacity-50"
              >
                Run Sequence
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
