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
  Plus
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
const HISTORY_LIMIT = 60;
const PRESET_STORAGE_KEY = "op3PosePresets";
const JOINT_SOURCE_TIMEOUT_MS = 1500;
const TICK_SECONDS = 0.008;

const DEFAULT_ASSETS = import.meta.env.VITE_ASSETS_URL || "http://localhost:8001";
const DEFAULT_ROSBRIDGE = import.meta.env.VITE_ROSBRIDGE_URL || "ws://localhost:9090";

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
  if (Array.isArray(positions)) {
    if (id !== undefined) positions[id] = value;
    return;
  }
  if (typeof positions === "object") positions[name] = value;
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

// --- Component ---

export default function ActionEditor({ isActive = true }) {
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
  const [historyTick, setHistoryTick] = useState(0);

  const [assetsUrl, setAssetsUrl] = useState(() => localStorage.getItem("op3AssetsUrl") || DEFAULT_ASSETS);
  const [rosUrl, setRosUrl] = useState(() => localStorage.getItem("op3RosUrl") || DEFAULT_ROSBRIDGE);
  const [rosState, setRosState] = useState("disconnected");

  const jointPubRef = useRef(null);
  const actionPagePubRef = useRef(null);
  const enableModulePubRef = useRef(null);
  const requestRef = useRef(null);
  const resultSubRef = useRef(null);
  const jointHardwareSubRef = useRef(null);
  const jointSimSubRef = useRef(null);
  const lastJointSourceRef = useRef({ hardware: 0, sim: 0 });
  const livePoseRef = useRef({});
  const pendingRunRef = useRef(null);
  const historyRef = useRef({});
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
  const jointNames = useMemo(() => showAllJoints && jointMeta.names.length ? jointMeta.names : JOINT_ORDER, [showAllJoints, jointMeta]);
  const visiblePages = useMemo(() => {
    const filter = pageFilter.trim().toLowerCase();
    if (!filter) return pages;
    return pages.filter(p => (p.name || "").toLowerCase().includes(filter) || String(p.index).includes(filter));
  }, [pages, pageFilter]);
  
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

  // --- Effects ---
  useEffect(() => { localStorage.setItem("op3AssetsUrl", assetsUrl); }, [assetsUrl]);
  useEffect(() => { localStorage.setItem("op3RosUrl", rosUrl); }, [rosUrl, autoEnableAction]);
  useEffect(() => { localStorage.setItem(PRESET_STORAGE_KEY, JSON.stringify(posePresets)); }, [posePresets]);
  useEffect(() => { localStorage.setItem("op3SendStepToWebots", sendStepToWebots ? "1" : "0"); }, [sendStepToWebots]);
  useEffect(() => { localStorage.setItem("op3JointGridColumns", String(jointGridColumns)); }, [jointGridColumns]);
  
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

  const cloneData = (data) => JSON.parse(JSON.stringify(data));
  const updateYamlData = (updater) => {
    if (!yamlData) return;
    const next = cloneData(yamlData);
    updater(next);
    setYamlData(next);
    setYamlText(YAML.dump(next, { sortKeys: false, lineWidth: -1 }));
  };

  const updateActivePage = (updater) => updateYamlData(draft => {
    const p = draft.pages?.find(i => i.index === activePage.index);
    if(p) updater(p);
  });

  const updateActiveStep = (updater) => {
    if (!activePage || selectedStepIndex === null) return;
    updateYamlData(draft => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if(!p?.steps) return;
      const step = p.steps.find(s => Number(s.index) === Number(selectedStepIndex)) || p.steps[selectedStepIndex];
      if(step) updater(step, p);
    });
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
    const header = { repeat: 1, schedule: 0, speed: 0, accel: 0, next: 0, exit: 0 };
    updateYamlData(draft => {
      if (!Array.isArray(draft.pages)) draft.pages = [];
      draft.pages.push({ index: nextIndex, name, header, steps: [] });
      draft.pages.sort((a, b) => Number(a.index) - Number(b.index));
    });
    setSelectedPageIndex(nextIndex);
    setSelectedStepIndex(null);
    setPreviewPose(null);
    setStatus(`Added ${name}`);
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
    const insertAt = getInsertStepIndex();
    updateYamlData(draft => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: 0, time: 0, positions: {} };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    });
    setSelectedStepIndex(insertAt);
    setPreviewPose(buildPose({}, livePoseRef.current));
    setStatus(`Added step ${insertAt}`);
    setStatusError(false);
  };

  const handleDuplicateStep = () => {
    if (!activePage || !activeStep || editorDisabled) return;
    const insertAt = getInsertStepIndex();
    const stepCopy = cloneData(activeStep.positions || {});
    updateYamlData(draft => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: activeStep.pause ?? 0, time: activeStep.time ?? 0, positions: stepCopy };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    });
    setSelectedStepIndex(insertAt);
    setPreviewPose(buildPose(stepCopy, livePoseRef.current));
    setStatus(`Duplicated step ${activeStep.index ?? selectedStepIndex}`);
    setStatusError(false);
  };

  const handleCapturePose = () => {
    if (!activeStep || editorDisabled) return;
    const live = livePoseRef.current || {};
    const captureNames = jointMeta.names.length ? jointMeta.names : JOINT_ORDER;
    const hasLive = captureNames.some(name => Number.isFinite(live[name]));
    if (!hasLive) {
      setStatus("No live pose available");
      setStatusError(true);
      return;
    }
    pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    updateActiveStep(s => {
      if (!s.positions || typeof s.positions !== "object") s.positions = {};
      captureNames.forEach(name => {
        const rad = live[name];
        if (!Number.isFinite(rad)) return;
        const raw = toRawDegrees((rad * 180) / Math.PI);
        setRawPosition(s.positions, name, jointIdMap, raw);
      });
    });
    setJointDrafts({});
    setPreviewPose({ ...live });
    setStatus("Captured live pose");
    setStatusError(false);
  };

  const handleCapturePoseToNewStep = () => {
    if (!activePage || !activeStep || editorDisabled) return;
    const live = livePoseRef.current || {};
    const captureNames = jointMeta.names.length ? jointMeta.names : JOINT_ORDER;
    const hasLive = captureNames.some(name => Number.isFinite(live[name]));
    if (!hasLive) {
      setStatus("No live pose available");
      setStatusError(true);
      return;
    }
    const positions = Array.isArray(activeStep.positions) ? [] : {};
    captureNames.forEach(name => {
      const rad = live[name];
      if (!Number.isFinite(rad)) return;
      const raw = toRawDegrees((rad * 180) / Math.PI);
      setRawPosition(positions, name, jointIdMap, raw);
    });
    const insertAt = getInsertStepIndex();
    updateYamlData(draft => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p) return;
      if (!Array.isArray(p.steps)) p.steps = [];
      const newStep = { index: 0, pause: activeStep.pause ?? 0, time: activeStep.time ?? 0, positions };
      p.steps.splice(insertAt, 0, newStep);
      p.steps.forEach((step, idx) => { step.index = idx; });
    });
    setSelectedStepIndex(insertAt);
    setPreviewPose({ ...live });
    setJointDrafts({});
    setStatus("Captured pose to new step");
    setStatusError(false);
  };

  const handleSwapLeftRight = () => {
    if (!activeStep || editorDisabled) return;
    const names = jointMeta.names.length ? jointMeta.names : JOINT_ORDER;
    const nameSet = new Set(names);
    const current = {};
    names.forEach(name => {
      current[name] = lookupRawPosition(activeStep.positions || {}, name, jointIdMap);
    });
    pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    updateActiveStep(s => {
      if (!s.positions || typeof s.positions !== "object") {
        s.positions = Array.isArray(activeStep.positions) ? [] : {};
      }
      names.forEach(name => {
        if (!name.startsWith("r_")) return;
        const other = `l_${name.slice(2)}`;
        if (!nameSet.has(other)) return;
        setRawPosition(s.positions, name, jointIdMap, current[other]);
        setRawPosition(s.positions, other, jointIdMap, current[name]);
      });
    });
    const previewPositions = cloneData(activeStep.positions || {});
    names.forEach(name => {
      if (!name.startsWith("r_")) return;
      const other = `l_${name.slice(2)}`;
      if (!nameSet.has(other)) return;
      setRawPosition(previewPositions, name, jointIdMap, current[other]);
      setRawPosition(previewPositions, other, jointIdMap, current[name]);
    });
    setJointDrafts({});
    setPreviewPose(buildPose(previewPositions, livePoseRef.current));
    setStatus("Swapped left/right joints");
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
    updateYamlData(draft => {
      if (!Array.isArray(draft.pages)) return;
      draft.pages = draft.pages.filter(p => p.index !== pageIndex);
    });
    clearPageStepHistory(pageIndex);
    setSelectedPageIndex(nextPage ? nextPage.index : null);
    setSelectedStepIndex(null);
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
    updateYamlData(draft => {
      const p = draft.pages?.find(i => i.index === activePage.index);
      if (!p?.steps) return;
      const byIdx = p.steps.findIndex(s => Number(s.index) === Number(selectedStepIndex));
      const removeAt = Number.isInteger(byIdx) && byIdx >= 0 ? byIdx : selectedStepIndex;
      if (removeAt < 0 || removeAt >= p.steps.length) return;
      p.steps.splice(removeAt, 1);
      p.steps.forEach((step, idx) => { step.index = idx; });
    });
    clearPageStepHistory(activePage.index);
    setSelectedStepIndex(nextSelected);
    setPreviewPose(nextStep ? buildPose(nextStep.positions || {}, livePoseRef.current) : null);
    setJointDrafts({});
    setStatus(`Deleted step ${stepLabel}`);
    setStatusError(false);
  };

  const stepHistoryKey = (pi=selectedPageIndex, si=selectedStepIndex) => (pi !== null && si !== null) ? `${pi}:${si}` : null;
  const snapshotStep = (s) => ({ positions: cloneData(s?.positions||{}), time: s?.time??0, pause: s?.pause??0 });
  const pushStepHistory = (key, snap) => {
    if(!key) return;
    const h = historyRef.current[key] || { undo: [], redo: [] };
    const last = h.undo[h.undo.length-1];
    if(last && JSON.stringify(last) === JSON.stringify(snap)) return;
    h.undo.push(cloneData(snap));
    if(h.undo.length > HISTORY_LIMIT) h.undo.shift();
    h.redo = [];
    historyRef.current[key] = h;
    setHistoryTick(t => t+1);
  };

  const clearPageStepHistory = (pageIndex) => {
    if (pageIndex === null || pageIndex === undefined) return;
    const prefix = `${pageIndex}:`;
    const next = {};
    Object.entries(historyRef.current).forEach(([key, value]) => {
      if (!key.startsWith(prefix)) next[key] = value;
    });
    historyRef.current = next;
    setHistoryTick(t => t+1);
  };

  const handleUndoStep = () => {
    const key = stepHistoryKey();
    if(!key || !activeStep) return;
    const h = historyRef.current[key];
    if(!h?.undo.length) return;
    const curr = snapshotStep(activeStep);
    const prev = h.undo.pop();
    h.redo.push(curr);
    updateActiveStep(s => { s.positions = prev.positions; s.time = prev.time; s.pause = prev.pause; });
    setHistoryTick(t => t+1); setJointDrafts({});
  };

  const handleRedoStep = () => {
    const key = stepHistoryKey();
    if(!key || !activeStep) return;
    const h = historyRef.current[key];
    if(!h?.redo.length) return;
    const curr = snapshotStep(activeStep);
    const next = h.redo.pop();
    h.undo.push(curr);
    updateActiveStep(s => { s.positions = next.positions; s.time = next.time; s.pause = next.pause; });
    setHistoryTick(t => t+1); setJointDrafts({});
  };

  const canUndo = useMemo(() => {
    const key = stepHistoryKey();
    if (!key) return false;
    const h = historyRef.current[key];
    return Boolean(h && h.undo.length);
  }, [activePage, selectedStepIndex, historyTick]);

  const canRedo = useMemo(() => {
    const key = stepHistoryKey();
    if (!key) return false;
    const h = historyRef.current[key];
    return Boolean(h && h.redo.length);
  }, [activePage, selectedStepIndex, historyTick]);

  // --- Keyboard Shortcuts ---
  useEffect(() => {
    const handleKey = (e) => {
      if (!activeStep || editorDisabled) return;
      if (e.target.tagName.match(/INPUT|TEXTAREA|SELECT/)) return;
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") {
        e.preventDefault(); e.shiftKey ? handleRedoStep() : handleUndoStep();
      } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "y") {
        e.preventDefault(); handleRedoStep();
      }
    };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, [activeStep, editorDisabled, historyTick]); // Dependencies important for closure capture

  // --- Joint Logic ---
  const handleJointDraftChange = (name, value) => {
    setJointDrafts(prev => ({ ...prev, [jointDraftKey(name)]: value }));
  };

  const commitJointDraft = (name, value) => {
    if(value === "" || value === null) { setJointDrafts(p => { const n={...p}; delete n[jointDraftKey(name)]; return n; }); return; }
    const num = Number(value);
    if(!Number.isFinite(num)) return;
    if(activeStep) pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    updateActiveStep(s => { if(!s.positions) s.positions={}; setRawPosition(s.positions, name, jointIdMap, toRawDegrees(num)); });
    setJointDrafts(p => { const n={...p}; delete n[jointDraftKey(name)]; return n; });
  };

  const handleJointToggleOff = (name, currentRaw) => {
    const nextVal = isTorqueOff(currentRaw) ? RAW_CENTER : "torque_off";
    if(activeStep) pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    updateActiveStep(s => { if(!s.positions) s.positions={}; setRawPosition(s.positions, name, jointIdMap, nextVal); });
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
        updateYamlData(draft => {
          const p = draft.pages.find(i => i.index === page.index);
          const s = p.steps.find(x => Number(x.index) === Number(rec.lastStepIndex)) || p.steps[rec.lastStepIndex];
          if(s) s[targetLabel] = Math.max(1, ticks);
        });
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

        {/* YAML Editor Toggle */}
        <div className={`bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col transition-all ${showYaml ? 'h-[300px]' : 'h-auto'}`}>
          <div className="p-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50 rounded-t-2xl">
            <div className="flex items-center gap-2">
              <span className="text-sm font-bold text-gray-700">YAML</span>
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
                onChange={e => setYamlText(e.target.value)}
                spellCheck="false"
              />
              <div className={`absolute bottom-0 left-0 right-0 px-2 py-1 text-[10px] font-mono border-t border-white/10 ${statusError ? "bg-red-900/80 text-red-200" : "bg-black/50 text-green-400"}`}>
                {status || "Ready"} {parseError && `| ${parseError}`}
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
                <input type="text" value={activePage.name || ""} onChange={e => updateActivePage(p => p.name = e.target.value)} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-bold text-gray-700 focus:border-undip-blue focus:ring-2 focus:ring-undip-blue/10 outline-none transition-all" />
              </div>
              <div>
                <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Schedule</label>
                <select value={scheduleMode} onChange={e => updateActivePage(p => { p.header = p.header||{}; p.header.schedule = e.target.value === "time" ? 10 : 0; })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none">
                  <option value="speed">Speed</option>
                  <option value="time">Time</option>
                </select>
              </div>
              <div>
                <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Repeat</label>
                <input type="number" value={activePage.header?.repeat ?? 0} onChange={e => updateActivePage(p => { p.header=p.header||{}; p.header.repeat = Number(e.target.value); })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none" />
              </div>
              {/* Other header fields simplified for brevity, assume similar pattern */}
              {["speed", "accel", "next", "exit"].map(f => (
                <div key={f}>
                  <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">{f}</label>
                  <input type="number" value={activePage.header?.[f] ?? 0} onChange={e => updateActivePage(p => { p.header=p.header||{}; p.header[f] = Number(e.target.value); })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none" />
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
              <button onClick={handleUndoStep} disabled={!canUndo} className="p-1.5 text-gray-500 hover:text-undip-blue disabled:opacity-30"><Undo size={16}/></button>
              <button onClick={handleRedoStep} disabled={!canRedo} className="p-1.5 text-gray-500 hover:text-undip-blue disabled:opacity-30"><Redo size={16}/></button>
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
                  <input type="number" value={activeStep.time ?? 0} onChange={e => updateActiveStep(s => s.time = Number(e.target.value))} className="w-full px-3 py-2 bg-white border border-gray-200 rounded-lg text-sm font-mono outline-none focus:border-undip-blue" />
                </div>
                <div className="flex-1">
                  <label className="block text-xs font-bold text-gray-400 uppercase tracking-wider mb-1">Pause</label>
                  <input type="number" value={activeStep.pause ?? 0} onChange={e => updateActiveStep(s => s.pause = Number(e.target.value))} className="w-full px-3 py-2 bg-white border border-gray-200 rounded-lg text-sm font-mono outline-none focus:border-undip-blue" />
                </div>
                <div className="flex items-end gap-2">
                  <button onClick={() => sendStepPoseToWebots(activeStep)} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-undip-blue hover:text-undip-blue rounded-lg text-xs font-bold flex items-center gap-1 transition-all"><Send size={14}/> Send</button>
                  <button onClick={handleCapturePose} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-green-500 hover:text-green-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Capture</button>
                  <button onClick={handleCapturePoseToNewStep} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-green-500 hover:text-green-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Capture New</button>
                  <button onClick={handleSwapLeftRight} className="px-3 py-2 bg-white border border-gray-200 text-gray-700 hover:border-amber-500 hover:text-amber-600 rounded-lg text-xs font-bold flex items-center gap-1 transition-all">Swap L/R</button>
                </div>
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
                    const isOff = isTorqueOff(raw);
                    const deg = norm === null ? 0 : toDegrees(norm);
                    const key = jointDraftKey(name);
                    const draft = jointDrafts[key] ?? (norm===null ? "" : deg.toFixed(1));
                    const displayId = JOINT_ID[name] ?? jointIdMap[name];
                    const idText = displayId ? `ID ${displayId}` : "ID ?";
                    const limit = activeLimitMap[name];
                    const outOfRange = Boolean(limit && !isOff && norm !== null && (deg < limit.min || deg > limit.max));
                    const rowClass = isOff
                      ? "bg-red-50 border-red-100 opacity-70"
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
                          onClick={() => handleJointToggleOff(name, raw)}
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
