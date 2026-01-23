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
  RefreshCw
} from "lucide-react";

// --- Constants & Helpers ---

const JOINT_ORDER = [
  "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
  "r_el", "l_el", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
  "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch",
  "r_ank_roll", "l_ank_roll", "head_pan", "head_tilt",
];

const JOINT_ID = {
  r_sho_pitch: 1, l_sho_pitch: 2, r_sho_roll: 3, l_sho_roll: 4,
  r_el: 5, l_el: 6, r_hip_yaw: 7, l_hip_yaw: 8, r_hip_roll: 9, l_hip_roll: 10,
  r_hip_pitch: 11, l_hip_pitch: 12, r_knee: 13, l_knee: 14, r_ank_pitch: 15,
  l_ank_pitch: 16, r_ank_roll: 17, l_ank_roll: 18, head_pan: 19, head_tilt: 20,
};

const JOINT_LABELS = {
  r_sho_pitch: "R Shoulder Pitch", l_sho_pitch: "L Shoulder Pitch",
  r_sho_roll: "R Shoulder Roll", l_sho_roll: "L Shoulder Roll",
  r_el: "R Elbow", l_el: "L Elbow",
  r_hip_yaw: "R Hip Yaw", l_hip_yaw: "L Hip Yaw",
  r_hip_roll: "R Hip Roll", l_hip_roll: "L Hip Roll",
  r_hip_pitch: "R Hip Pitch", l_hip_pitch: "L Hip Pitch",
  r_knee: "R Knee", l_knee: "L Knee",
  r_ank_pitch: "R Ankle Pitch", l_ank_pitch: "L Ankle Pitch",
  r_ank_roll: "R Ankle Roll", l_ank_roll: "L Ankle Roll",
  head_pan: "Head Pan", head_tilt: "Head Tilt",
};

const RAW_CENTER = 2048;
const RAW_RANGE = 2048;
const HISTORY_LIMIT = 60;
const PRESET_STORAGE_KEY = "op3PosePresets";

const DEFAULT_ASSETS = import.meta.env.VITE_ASSETS_URL || "http://localhost:8001";
const DEFAULT_ROSBRIDGE = import.meta.env.VITE_ROSBRIDGE_URL || "ws://localhost:9090";

function toRadians(raw) { return ((raw - RAW_CENTER) * Math.PI) / RAW_RANGE; }
function toDegrees(raw) { return ((raw - RAW_CENTER) * 180) / RAW_RANGE; }
function clampRaw(value) { return Math.max(0, Math.min(4095, value)); }
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

export default function ActionEditor() {
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
  const jointSubRef = useRef(null);
  const livePoseRef = useRef({});
  const pendingRunRef = useRef(null);
  const historyRef = useRef({});
  const recordRef = useRef({ lastTime: null, lastStepIndex: null, lastPageIndex: null });

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

    jointSubRef.current = new ROSLIB.Topic({ ros, name: "/robotis_op3/joint_states", messageType: "sensor_msgs/JointState" });
    jointSubRef.current.subscribe(msg => {
      const next = { ...livePoseRef.current };
      msg.name.forEach((name, idx) => { if (JOINT_ID[name]) next[name] = msg.position[idx]; });
      livePoseRef.current = next;
      setLivePose(next);
    });

    return () => {
      jointSubRef.current?.unsubscribe();
      resultSubRef.current?.unsubscribe();
      ros.close();
    };
  }, [rosUrl]);

  // --- Three.js ---
  useEffect(() => {
    if (!viewerEnabled || !viewerRef.current) return;
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
  }, [assetsUrl, viewerEnabled]);

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
    const scratchHeader = {
      repeat: Number(header.repeat) || 1,
      schedule: header.schedule === 10 ? 10 : 0,
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
    const scratchHeader = {
      repeat: Number(header.repeat) || 1,
      schedule: header.schedule === 10 ? 10 : 0,
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

  return (
    <div className="flex h-full gap-6 p-2 overflow-hidden bg-[#f8fafc]">
      {/* LEFT COLUMN: Lists & YAML */}
      <div className="flex flex-col w-1/4 min-w-[280px] gap-4 h-full">
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col flex-1 overflow-hidden">
          <div className="p-4 border-b border-gray-100 flex justify-between items-center">
            <h2 className="text-lg font-bold font-display text-gray-800">Pages</h2>
            <button className="text-xs font-medium text-undip-blue hover:underline" onClick={() => { setPreviewPose(null); setSelectedStepIndex(null); }}>
              Reset Live
            </button>
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
            <div className="text-xs font-bold text-gray-400 uppercase tracking-wider mb-2 px-1">
              Steps {activePage ? `(Page ${activePage.index})` : ""}
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
      <div className="flex flex-col flex-1 gap-4 h-full overflow-y-auto custom-scrollbar pb-2">
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
              <button onClick={() => actionPagePubRef.current?.publish(new ROSLIB.Message({data: activePage?.index}))} className="p-1.5 text-green-600 hover:bg-green-50 rounded" title="Run Page"><Play size={16}/></button>
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
                <select value={activePage.header?.schedule === 10 ? "time" : "speed"} onChange={e => updateActivePage(p => { p.header = p.header||{}; p.header.schedule = e.target.value; })} disabled={editorDisabled} className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono text-gray-700 outline-none">
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
                </div>
              </div>

              {/* Joint Grid */}
              <div className="flex-1 overflow-y-auto custom-scrollbar pr-2">
                <div className="grid grid-cols-1 xl:grid-cols-2 gap-3">
                  {jointNames.map(name => {
                    const raw = lookupRawPosition(activeStep.positions||{}, name, jointIdMap);
                    const norm = normalizeRaw(raw);
                    const isOff = isTorqueOff(raw);
                    const deg = norm === null ? 0 : toDegrees(norm);
                    const key = jointDraftKey(name);
                    const draft = jointDrafts[key] ?? (norm===null ? "" : deg.toFixed(1));
                    
                    return (
                      <div key={name} className={`flex items-center gap-3 p-2 rounded-lg border transition-all ${isOff ? 'bg-red-50 border-red-100 opacity-70' : 'bg-white border-gray-100 hover:border-gray-300'}`}>
                        <div className="w-24 flex flex-col">
                          <span className="text-[10px] font-mono text-gray-400">{name}</span>
                          <span className="text-xs font-bold text-gray-700 truncate" title={formatJointLabel(name)}>{formatJointLabel(name)}</span>
                        </div>
                        <input 
                          type="range" min="-180" max="180" step="0.5" 
                          value={Number.isFinite(Number(draft)) ? Number(draft) : deg} 
                          onChange={e => commitJointDraft(name, e.target.value)}
                          disabled={editorDisabled || isOff}
                          className="flex-1 accent-undip-blue h-1.5 bg-gray-200 rounded-full appearance-none cursor-pointer"
                        />
                        <input 
                          type="number" 
                          value={draft}
                          onChange={e => handleJointDraftChange(name, e.target.value)}
                          onBlur={e => commitJointDraft(name, e.target.value)}
                          onKeyDown={e => e.key === "Enter" && commitJointDraft(name, e.target.value)}
                          disabled={editorDisabled || isOff}
                          className="w-16 px-2 py-1 bg-gray-50 border border-gray-200 rounded text-xs font-mono text-right focus:outline-none focus:border-undip-blue"
                        />
                        <button 
                          onClick={() => handleJointToggleOff(name, raw)}
                          className={`px-2 py-1 rounded text-[10px] font-bold uppercase transition-colors ${isOff ? 'bg-red-100 text-red-600' : 'bg-gray-100 text-gray-500 hover:bg-gray-200'}`}
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
      <div className="flex flex-col w-1/4 min-w-[300px] gap-4 h-full">
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden flex flex-col h-[400px]">
          <div className="p-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50">
            <h2 className="text-sm font-bold text-gray-700">3D Preview</h2>
            <div className="flex gap-1">
              <button onClick={() => rotateView(-45)} className="p-1.5 text-gray-500 hover:bg-white rounded"><RotateCcw size={14}/></button>
              <button onClick={() => rotateView(45)} className="p-1.5 text-gray-500 hover:bg-white rounded"><RotateCw size={14}/></button>
              <button onClick={() => setViewerEnabled(!viewerEnabled)} className="p-1.5 text-gray-500 hover:bg-white rounded">{viewerEnabled ? <Eye size={14}/> : <EyeOff size={14}/>}</button>
            </div>
          </div>
          <div className="relative flex-1 bg-slate-50">
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