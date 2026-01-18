import { useEffect, useMemo, useRef, useState } from "react";
import YAML from "js-yaml";
import ROSLIB from "roslib";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

const JOINT_ORDER = [
  "r_sho_pitch",
  "l_sho_pitch",
  "r_sho_roll",
  "l_sho_roll",
  "r_el",
  "l_el",
  "r_hip_yaw",
  "l_hip_yaw",
  "r_hip_roll",
  "l_hip_roll",
  "r_hip_pitch",
  "l_hip_pitch",
  "r_knee",
  "l_knee",
  "r_ank_pitch",
  "l_ank_pitch",
  "r_ank_roll",
  "l_ank_roll",
  "head_pan",
  "head_tilt",
];

const JOINT_ID = {
  r_sho_pitch: 1,
  l_sho_pitch: 2,
  r_sho_roll: 3,
  l_sho_roll: 4,
  r_el: 5,
  l_el: 6,
  r_hip_yaw: 7,
  l_hip_yaw: 8,
  r_hip_roll: 9,
  l_hip_roll: 10,
  r_hip_pitch: 11,
  l_hip_pitch: 12,
  r_knee: 13,
  l_knee: 14,
  r_ank_pitch: 15,
  l_ank_pitch: 16,
  r_ank_roll: 17,
  l_ank_roll: 18,
  head_pan: 19,
  head_tilt: 20,
};

const JOINT_LABELS = {
  r_sho_pitch: "Right shoulder pitch",
  l_sho_pitch: "Left shoulder pitch",
  r_sho_roll: "Right shoulder roll",
  l_sho_roll: "Left shoulder roll",
  r_el: "Right elbow",
  l_el: "Left elbow",
  r_hip_yaw: "Right hip yaw",
  l_hip_yaw: "Left hip yaw",
  r_hip_roll: "Right hip roll",
  l_hip_roll: "Left hip roll",
  r_hip_pitch: "Right hip pitch",
  l_hip_pitch: "Left hip pitch",
  r_knee: "Right knee",
  l_knee: "Left knee",
  r_ank_pitch: "Right ankle pitch",
  l_ank_pitch: "Left ankle pitch",
  r_ank_roll: "Right ankle roll",
  l_ank_roll: "Left ankle roll",
  head_pan: "Head pan",
  head_tilt: "Head tilt",
};

const RAW_CENTER = 2048;
const RAW_RANGE = 2048;
const HISTORY_LIMIT = 60;
const PRESET_STORAGE_KEY = "op3PosePresets";

const DEFAULT_ASSETS =
  import.meta.env.VITE_ASSETS_URL || "http://localhost:8001";
const DEFAULT_ROSBRIDGE =
  import.meta.env.VITE_ROSBRIDGE_URL || "ws://localhost:9090";

function toRadians(raw) {
  return ((raw - RAW_CENTER) * Math.PI) / RAW_RANGE;
}

function toDegrees(raw) {
  return ((raw - RAW_CENTER) * 180) / RAW_RANGE;
}

function clampRaw(value) {
  return Math.max(0, Math.min(4095, value));
}

function toRawDegrees(degrees) {
  return clampRaw(Math.round((degrees * RAW_RANGE) / 180 + RAW_CENTER));
}

function isTorqueOff(value) {
  return typeof value === "string" && value.toLowerCase() === "torque_off";
}

function isInvalidRaw(value) {
  if (value === null || value === undefined) {
    return true;
  }
  if (typeof value === "string") {
    const lowered = value.toLowerCase();
    return lowered === "torque_off" || lowered === "invalid" || lowered === "none";
  }
  return false;
}

function normalizeRaw(value) {
  if (isInvalidRaw(value)) {
    return null;
  }
  if (typeof value === "number") {
    return Math.round(value);
  }
  if (typeof value === "string") {
    const trimmed = value.trim();
    if (!trimmed) {
      return null;
    }
    const parsed = Number(trimmed);
    if (Number.isFinite(parsed)) {
      return Math.round(parsed);
    }
  }
  return null;
}

function formatJointLabel(name) {
  if (JOINT_LABELS[name]) {
    return JOINT_LABELS[name];
  }
  const spaced = String(name || "").replace(/_/g, " ");
  return spaced.replace(/\b[a-z]/g, (letter) => letter.toUpperCase());
}

function lookupRawPosition(positions, name, idMap) {
  if (!positions) {
    return undefined;
  }
  if (Array.isArray(positions)) {
    const id = idMap[name];
    if (!id) {
      return undefined;
    }
    return positions[id];
  }
  if (typeof positions === "object") {
    if (Object.prototype.hasOwnProperty.call(positions, name)) {
      return positions[name];
    }
    const id = idMap[name];
    if (id) {
      const idKey = `id_${id}`;
      if (Object.prototype.hasOwnProperty.call(positions, idKey)) {
        return positions[idKey];
      }
    }
  }
  return undefined;
}

function setRawPosition(positions, name, idMap, value) {
  if (!positions) {
    return;
  }
  const id = idMap[name];
  if (Array.isArray(positions)) {
    if (id !== undefined) {
      positions[id] = value;
    }
    return;
  }
  if (typeof positions === "object") {
    positions[name] = value;
  }
}

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

export default function App() {
  const [yamlText, setYamlText] = useState("");
  const [yamlData, setYamlData] = useState(null);
  const [parseError, setParseError] = useState("");
  const [selectedPageIndex, setSelectedPageIndex] = useState(null);
  const [selectedStepIndex, setSelectedStepIndex] = useState(null);
  const [showAllJoints, setShowAllJoints] = useState(false);
  const [upright, setUpright] = useState(true);
  const [layFlat, setLayFlat] = useState(false);
  const [mirrorView, setMirrorView] = useState(false);
  const [viewerEnabled, setViewerEnabled] = useState(true);
  const [viewerNote, setViewerNote] = useState("");
  const [sendStepToWebots, setSendStepToWebots] = useState(() => {
    const raw = localStorage.getItem("op3SendStepToWebots");
    if (raw === null || raw === undefined) {
      return false;
    }
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
      const parsed = raw ? JSON.parse(raw) : [];
      return Array.isArray(parsed) ? parsed : [];
    } catch (err) {
      return [];
    }
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

  const [assetsUrl, setAssetsUrl] = useState(() => {
    return localStorage.getItem("op3AssetsUrl") || DEFAULT_ASSETS;
  });
  const [rosUrl, setRosUrl] = useState(() => {
    return localStorage.getItem("op3RosUrl") || DEFAULT_ROSBRIDGE;
  });
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
  const recordRef = useRef({
    lastTime: null,
    lastStepIndex: null,
    lastPageIndex: null,
  });

  const viewerRef = useRef(null);
  const robotRef = useRef(null);
  const cameraRef = useRef(null);
  const controlsRef = useRef(null);
  const defaultCameraPosRef = useRef(new THREE.Vector3(0.6, 0.35, 1.4));
  const defaultTargetRef = useRef(new THREE.Vector3(0, 0, 0));

  const pages = useMemo(() => {
    return yamlData && Array.isArray(yamlData.pages) ? yamlData.pages : [];
  }, [yamlData]);

  const jointMeta = useMemo(() => {
    const order = yamlData?.meta?.joint_order || [];
    const names = [];
    const nameToId = {};
    order.forEach((entry) => {
      if (!entry || !entry.name) {
        return;
      }
      names.push(entry.name);
      nameToId[entry.name] = entry.id;
    });
    return { names, nameToId };
  }, [yamlData]);

  const jointIdMap = useMemo(() => {
    return Object.keys(jointMeta.nameToId).length ? jointMeta.nameToId : JOINT_ID;
  }, [jointMeta]);

  const jointNames = useMemo(() => {
    if (showAllJoints && jointMeta.names.length) {
      return jointMeta.names;
    }
    return JOINT_ORDER;
  }, [showAllJoints, jointMeta]);

  const visiblePages = useMemo(() => {
    const filter = pageFilter.trim().toLowerCase();
    if (!filter) {
      return pages;
    }
    return pages.filter((page) => {
      const name = (page.name || "").toLowerCase();
      return name.includes(filter) || String(page.index).includes(filter);
    });
  }, [pages, pageFilter]);

  const activePage = useMemo(() => {
    if (selectedPageIndex === null || selectedPageIndex === undefined) {
      return null;
    }
    return pages.find((page) => page.index === selectedPageIndex) || null;
  }, [pages, selectedPageIndex]);

  const activeStep = useMemo(() => {
    if (!activePage || selectedStepIndex === null || selectedStepIndex === undefined) {
      return null;
    }
    const byIndex = activePage.steps?.find(
      (step) => Number(step.index) === Number(selectedStepIndex)
    );
    if (byIndex) {
      return byIndex;
    }
    if (Array.isArray(activePage.steps) && activePage.steps[selectedStepIndex]) {
      return activePage.steps[selectedStepIndex];
    }
    return null;
  }, [activePage, selectedStepIndex]);

  const editorDisabled = !yamlData || Boolean(parseError);

  const applyRobotOrientation = (robot, uprightValue, layFlatValue) => {
    const baseRotationX = -Math.PI / 2;
    const extra = uprightValue ? 0 : Math.PI;
    const flat = layFlatValue ? Math.PI / 2 : 0;
    robot.rotation.set(baseRotationX + extra + flat, 0, 0);
  };

  const applyRobotMirror = (robot, mirrorValue) => {
    robot.scale.x = mirrorValue ? -1 : 1;
    robot.traverse((child) => {
      if (!child.isMesh || !child.material) {
        return;
      }
      if (Array.isArray(child.material)) {
        child.material.forEach((mat) => {
          mat.side = THREE.DoubleSide;
          mat.needsUpdate = true;
        });
      } else {
        child.material.side = THREE.DoubleSide;
        child.material.needsUpdate = true;
      }
    });
  };

  const resetView = () => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) {
      return;
    }
    camera.position.copy(defaultCameraPosRef.current);
    controls.target.copy(defaultTargetRef.current);
    camera.lookAt(defaultTargetRef.current);
    controls.update();
  };

  const rotateView = (degrees) => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) {
      return;
    }
    const target = controls.target.clone();
    const offset = camera.position.clone().sub(target);
    offset.applyAxisAngle(
      new THREE.Vector3(0, 1, 0),
      THREE.MathUtils.degToRad(degrees)
    );
    camera.position.copy(target.clone().add(offset));
    camera.lookAt(target);
    controls.update();
  };

  useEffect(() => {
    localStorage.setItem("op3AssetsUrl", assetsUrl);
  }, [assetsUrl]);

  useEffect(() => {
    localStorage.setItem("op3RosUrl", rosUrl);
  }, [rosUrl, autoEnableAction]);

  useEffect(() => {
    localStorage.setItem(PRESET_STORAGE_KEY, JSON.stringify(posePresets));
  }, [posePresets]);

  useEffect(() => {
    localStorage.setItem("op3SendStepToWebots", sendStepToWebots ? "1" : "0");
  }, [sendStepToWebots]);

  useEffect(() => {
    if (viewerEnabled) {
      setViewerNote("");
    }
  }, [viewerEnabled]);

  useEffect(() => {
    if (!yamlText.trim()) {
      setYamlData(null);
      setParseError("");
      return undefined;
    }

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
    if (!pages.length) {
      setSelectedPageIndex(null);
      setSelectedStepIndex(null);
      return;
    }
    const found = pages.find((page) => page.index === selectedPageIndex);
    if (!found) {
      setSelectedPageIndex(pages[0].index);
      setSelectedStepIndex(null);
    }
  }, [pages, selectedPageIndex]);

  useEffect(() => {
    if (!activePage) {
      setSelectedStepIndex(null);
      return;
    }
    if (selectedStepIndex === null || selectedStepIndex === undefined) {
      return;
    }
    const foundStep = activePage.steps?.find(
      (step) => Number(step.index) === Number(selectedStepIndex)
    );
    if (!foundStep && !activePage.steps?.[selectedStepIndex]) {
      setSelectedStepIndex(null);
    }
  }, [activePage, selectedStepIndex]);

  useEffect(() => {
    setJointDrafts({});
  }, [selectedPageIndex, selectedStepIndex]);

  useEffect(() => {
    if (!activeStep) {
      setRangeStart("");
      setRangeEnd("");
      return;
    }
    const indexValue =
      activeStep.index !== undefined && activeStep.index !== null
        ? activeStep.index
        : selectedStepIndex;
    if (indexValue !== null && indexValue !== undefined) {
      setRangeStart(String(indexValue));
      setRangeEnd(String(indexValue));
    }
  }, [activeStep, selectedStepIndex]);

  useEffect(() => {
    recordRef.current = {
      lastTime: null,
      lastStepIndex: null,
      lastPageIndex: selectedPageIndex,
    };
    setRecordElapsed(0);
    setRecordLastDelta(null);
    setRecordLastTicks(null);
  }, [recordEnabled, selectedPageIndex]);

  useEffect(() => {
    if (!recordEnabled) {
      return undefined;
    }
    const timer = window.setInterval(() => {
      const lastTime = recordRef.current.lastTime;
      if (lastTime === null || lastTime === undefined) {
        setRecordElapsed(0);
        return;
      }
      const delta = (window.performance.now() - lastTime) / 1000;
      setRecordElapsed(delta);
    }, 100);
    return () => window.clearInterval(timer);
  }, [recordEnabled]);

  useEffect(() => {
    if (!rosUrl) {
      return undefined;
    }

    const ros = new ROSLIB.Ros({ url: rosUrl });
    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    jointPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/webots/joint_positions",
      messageType: "std_msgs/Float64MultiArray",
    });

    actionPagePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/action/page_num",
      messageType: "std_msgs/Int32",
    });

    enableModulePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/enable_ctrl_module",
      messageType: "std_msgs/String",
    });

    requestRef.current = new ROSLIB.Topic({
      ros,
      name: "/op3_action_web/request",
      messageType: "std_msgs/String",
    });

    resultSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/op3_action_web/result",
      messageType: "std_msgs/String",
    });

    resultSubRef.current.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        setStatus(formatResultMessage(payload));
        setStatusError(payload.ok === false);
        if (payload.action === "export" && payload.yaml) {
          setYamlText(payload.yaml);
        }
        const pending = pendingRunRef.current;
        if (pending && payload.request_id === pending.requestId) {
          pendingRunRef.current = null;
          if (payload.ok) {
            if (autoEnableAction) {
              publishEnableActionModule();
              window.setTimeout(() => {
                publishActionPage(pending.pageIndex);
              }, 300);
            } else {
              publishActionPage(pending.pageIndex);
            }
          } else {
            setStatus("Scratch step apply failed");
            setStatusError(true);
          }
        }
      } catch (err) {
        setStatus(msg.data);
        setStatusError(false);
      }
    });

    jointSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis_op3/joint_states",
      messageType: "sensor_msgs/JointState",
    });

    jointSubRef.current.subscribe((msg) => {
      const nextPose = { ...livePoseRef.current };
      msg.name.forEach((name, idx) => {
        if (JOINT_ID[name]) {
          nextPose[name] = msg.position[idx];
        }
      });
      livePoseRef.current = nextPose;
      setLivePose(nextPose);
    });

    return () => {
      jointSubRef.current?.unsubscribe();
      resultSubRef.current?.unsubscribe();
      actionPagePubRef.current = null;
      enableModulePubRef.current = null;
      ros.close();
    };
  }, [rosUrl]);

  useEffect(() => {
    if (!viewerEnabled) {
      return undefined;
    }
    const container = viewerRef.current;
    if (!container) {
      return undefined;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#f4f0e6");

    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 20);
    camera.position.copy(defaultCameraPosRef.current);

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.copy(defaultTargetRef.current);
    controls.update();

    cameraRef.current = camera;
    controlsRef.current = controls;

    const ambient = new THREE.AmbientLight(0xffffff, 0.7);
    const directional = new THREE.DirectionalLight(0xffffff, 0.6);
    directional.position.set(1.2, 1.5, 0.8);
    scene.add(ambient, directional);

    const grid = new THREE.GridHelper(2.4, 20, 0xd4cbb7, 0xe4ddcc);
    grid.position.y = -0.32;
    scene.add(grid);

    const loader = new URDFLoader();
    const handleContextLost = (event) => {
      event.preventDefault();
      setViewerNote("WebGL context lost. Toggle preview to restart.");
      setViewerEnabled(false);
    };

    renderer.domElement.addEventListener("webglcontextlost", handleContextLost);

    loader.load(`${assetsUrl}/robotis_op3.urdf`, (robot) => {
      applyRobotOrientation(robot, upright, layFlat);
      applyRobotMirror(robot, mirrorView);
      robotRef.current = robot;
      scene.add(robot);
    });

    let frameId = 0;
    const animate = () => {
      frameId = window.requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      const nextWidth = container.clientWidth || width;
      const nextHeight = container.clientHeight || height;
      camera.aspect = nextWidth / nextHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(nextWidth, nextHeight);
    };

    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      window.cancelAnimationFrame(frameId);
      controls.dispose();
      renderer.domElement.removeEventListener("webglcontextlost", handleContextLost);
      renderer.dispose();
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
      cameraRef.current = null;
      controlsRef.current = null;
      robotRef.current = null;
    };
  }, [assetsUrl, viewerEnabled]);

  useEffect(() => {
    if (robotRef.current) {
      applyRobotOrientation(robotRef.current, upright, layFlat);
    }
  }, [upright, layFlat]);

  useEffect(() => {
    if (robotRef.current) {
      applyRobotMirror(robotRef.current, mirrorView);
    }
  }, [mirrorView]);

  const activePose = previewPose || livePose;

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot || !activePose) {
      return;
    }
    JOINT_ORDER.forEach((name) => {
      const value = activePose[name];
      if (robot.joints[name] && Number.isFinite(value)) {
        robot.joints[name].setJointValue(value);
      }
    });
  }, [activePose]);

  useEffect(() => {
    if (!previewPose || !activeStep) {
      return;
    }
    const pose = buildPose(activeStep.positions || {}, livePoseRef.current);
    setPreviewPose(pose);
  }, [activeStep, yamlData]);

  const sendRequest = (payload) => {
    if (!requestRef.current) {
      setStatus("ROS not connected");
      setStatusError(true);
      return;
    }
    requestRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
  };

  const handleApply = () => {
    sendRequest({ action: "apply", yaml: yamlText });
    setStatus("Applying YAML...");
    setStatusError(false);
  };

  const handleExport = () => {
    sendRequest({ action: "export", pages: exportPages });
    setStatus("Exporting YAML...");
    setStatusError(false);
  };

  const cloneData = (data) => {
    if (typeof structuredClone === "function") {
      return structuredClone(data);
    }
    return JSON.parse(JSON.stringify(data));
  };

  const updateYamlData = (updater) => {
    if (!yamlData) {
      return;
    }
    const next = cloneData(yamlData);
    updater(next);
    const nextText = YAML.dump(next, { sortKeys: false, lineWidth: -1 });
    setYamlData(next);
    setYamlText(nextText);
  };

  const updateActivePage = (updater) => {
    if (!activePage) {
      return;
    }
    updateYamlData((draft) => {
      const page = draft.pages?.find((item) => item.index === activePage.index);
      if (!page) {
        return;
      }
      updater(page);
    });
  };

  const findStepByIndex = (page, index) => {
    if (!page || !Array.isArray(page.steps)) {
      return null;
    }
    const byIndex = page.steps.find(
      (step) => Number(step.index) === Number(index)
    );
    return byIndex || page.steps[index] || null;
  };

  const updateActiveStep = (updater) => {
    if (!activePage || selectedStepIndex === null || selectedStepIndex === undefined) {
      return;
    }
    updateYamlData((draft) => {
      const page = draft.pages?.find((item) => item.index === activePage.index);
      if (!page || !Array.isArray(page.steps)) {
        return;
      }
      const step = findStepByIndex(page, selectedStepIndex);
      if (!step) {
        return;
      }
      updater(step, page);
    });
  };

  const updateStepByIndex = (pageIndex, stepIndex, updater) => {
    if (pageIndex === null || pageIndex === undefined) {
      return;
    }
    updateYamlData((draft) => {
      const page = draft.pages?.find((item) => item.index === pageIndex);
      if (!page || !Array.isArray(page.steps)) {
        return;
      }
      const step = findStepByIndex(page, stepIndex);
      if (!step) {
        return;
      }
      updater(step, page);
    });
  };

  const clipStatus = (value) => {
    const text = String(value || "").replace(/\s+/g, " ").trim();
    if (!text) {
      return "";
    }
    if (text.length > 260) {
      return `${text.slice(0, 260)}…`;
    }
    return text;
  };

  const extractLastLine = (value) => {
    const lines = String(value || "")
      .split("\n")
      .map((line) => line.trim())
      .filter(Boolean);
    if (!lines.length) {
      return "";
    }
    return lines[lines.length - 1];
  };

  const formatResultMessage = (payload) => {
    const parts = [];
    if (payload?.message) {
      parts.push(clipStatus(payload.message));
    }
    if (payload && payload.ok === false) {
      const lastErr = extractLastLine(payload.stderr);
      const lastOut = extractLastLine(payload.stdout);
      if (lastErr) {
        parts.push(clipStatus(lastErr));
      } else if (lastOut) {
        parts.push(clipStatus(lastOut));
      }
    }
    return parts.filter(Boolean).join(" | ") || "Result received";
  };

  const parseSchedule = (header) => {
    const value = header?.schedule;
    if (value === "time" || value === 10 || value === "0x0a") {
      return "time";
    }
    return "speed";
  };

  const resolveSpeed = (header) => {
    const raw = Number(header?.speed);
    if (Number.isFinite(raw) && raw > 0) {
      return raw;
    }
    return 32;
  };

  const clampByte = (value) => {
    return Math.max(0, Math.min(255, value));
  };

  const secondsToTimeTicks = (seconds, speed) => {
    if (!Number.isFinite(seconds)) {
      return 0;
    }
    const ticks = (seconds / 0.008) * (32 / speed);
    return clampByte(Math.round(ticks));
  };

  const secondsToPauseTicks = (seconds, speed) => {
    if (!Number.isFinite(seconds)) {
      return 0;
    }
    const ticks = (seconds / 0.008) * (speed / 32);
    return clampByte(Math.round(ticks));
  };

  const stepHistoryKey = (pageIndex = null, stepIndex = null) => {
    const pageValue =
      pageIndex !== null && pageIndex !== undefined
        ? pageIndex
        : activePage?.index;
    const stepValue =
      stepIndex !== null && stepIndex !== undefined ? stepIndex : selectedStepIndex;
    if (pageValue === null || pageValue === undefined) {
      return null;
    }
    if (stepValue === null || stepValue === undefined) {
      return null;
    }
    return `${pageValue}:${stepValue}`;
  };

  const snapshotStep = (step) => {
    return {
      positions: cloneData(step?.positions || {}),
      time: step?.time ?? 0,
      pause: step?.pause ?? 0,
    };
  };

  const applyStepSnapshot = (step, snapshot) => {
    step.positions = cloneData(snapshot?.positions || {});
    step.time = snapshot?.time ?? 0;
    step.pause = snapshot?.pause ?? 0;
  };

  const pushStepHistory = (key, stepSnapshot) => {
    if (!key) {
      return;
    }
    const history = historyRef.current[key] || { undo: [], redo: [] };
    const snapshot = cloneData(stepSnapshot || {});
    const last = history.undo[history.undo.length - 1];
    if (last && JSON.stringify(last) === JSON.stringify(snapshot)) {
      historyRef.current[key] = history;
      return;
    }
    history.undo.push(snapshot);
    if (history.undo.length > HISTORY_LIMIT) {
      history.undo.shift();
    }
    history.redo = [];
    historyRef.current[key] = history;
    setHistoryTick((tick) => tick + 1);
  };

  const handleUndoStep = () => {
    const key = stepHistoryKey();
    if (!key || !activeStep) {
      return;
    }
    const history = historyRef.current[key];
    if (!history || history.undo.length === 0) {
      return;
    }
    const current = snapshotStep(activeStep);
    const snapshot = history.undo.pop();
    history.redo.push(current);
    historyRef.current[key] = history;
    updateActiveStep((step) => {
      applyStepSnapshot(step, snapshot);
    });
    setHistoryTick((tick) => tick + 1);
    setJointDrafts({});
  };

  const handleRedoStep = () => {
    const key = stepHistoryKey();
    if (!key || !activeStep) {
      return;
    }
    const history = historyRef.current[key];
    if (!history || history.redo.length === 0) {
      return;
    }
    const current = snapshotStep(activeStep);
    const snapshot = history.redo.pop();
    history.undo.push(current);
    historyRef.current[key] = history;
    updateActiveStep((step) => {
      applyStepSnapshot(step, snapshot);
    });
    setHistoryTick((tick) => tick + 1);
    setJointDrafts({});
  };

  const canUndo = useMemo(() => {
    const key = stepHistoryKey();
    if (!key) {
      return false;
    }
    const history = historyRef.current[key];
    return Boolean(history && history.undo.length);
  }, [activePage, selectedStepIndex, historyTick]);

  const canRedo = useMemo(() => {
    const key = stepHistoryKey();
    if (!key) {
      return false;
    }
    const history = historyRef.current[key];
    return Boolean(history && history.redo.length);
  }, [activePage, selectedStepIndex, historyTick]);

  useEffect(() => {
    const handleKey = (event) => {
      if (!activeStep || editorDisabled) {
        return;
      }
      const target = event.target;
      if (
        target &&
        (target.tagName === "INPUT" ||
          target.tagName === "TEXTAREA" ||
          target.tagName === "SELECT" ||
          target.isContentEditable)
      ) {
        return;
      }
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
        event.preventDefault();
        if (event.shiftKey) {
          handleRedoStep();
        } else {
          handleUndoStep();
        }
      } else if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "y") {
        event.preventDefault();
        handleRedoStep();
      }
    };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, [activeStep, editorDisabled, handleUndoStep, handleRedoStep]);

  const publishActionPage = (value) => {
    if (!actionPagePubRef.current) {
      setStatus("ROS not connected");
      setStatusError(true);
      return;
    }
    actionPagePubRef.current.publish(new ROSLIB.Message({ data: value }));
    setStatus(`Action page command: ${value}`);
    setStatusError(false);
  };

  const publishEnableActionModule = () => {
    if (!enableModulePubRef.current) {
      setStatus("ROS not connected");
      setStatusError(true);
      return;
    }
    enableModulePubRef.current.publish(
      new ROSLIB.Message({ data: "action_module" })
    );
    setStatus("Enabling action module...");
    setStatusError(false);
  };

  const handleRunPage = () => {
    if (!activePage) {
      return;
    }
    if (autoEnableAction) {
      publishEnableActionModule();
      window.setTimeout(() => {
        publishActionPage(activePage.index);
      }, 300);
    } else {
      publishActionPage(activePage.index);
    }
  };

  const buildScratchYaml = () => {
    if (!activePage || !activeStep) {
      return null;
    }
    const scratchIndex = Number(scratchPageIndex);
    if (!Number.isFinite(scratchIndex) || scratchIndex < 1 || scratchIndex > 255) {
      setStatus("Scratch page must be 1-255");
      setStatusError(true);
      return null;
    }
    const header = activePage.header || {};
    const repeatValue =
      Number.isFinite(header.repeat) && Number(header.repeat) > 0
        ? Math.round(Number(header.repeat))
        : 1;
    const scratchHeader = {
      repeat: repeatValue,
      schedule: scheduleValue,
      speed: Number.isFinite(header.speed) ? Math.round(header.speed) : 0,
      accel: Number.isFinite(header.accel) ? Math.round(header.accel) : 0,
      next: 0,
      exit: 0,
    };
    const stepCopy = cloneData(activeStep.positions || {});
    const step = {
      index: 0,
      pause: activeStep.pause ?? 0,
      time: activeStep.time ?? 0,
      positions: stepCopy,
    };
    const payload = {
      pages: [
        {
          index: scratchIndex,
          name: `scratch_${activePage.index}_step_${activeStepLabel}`,
          header: scratchHeader,
          steps: [step],
        },
      ],
    };
    return { scratchIndex, yaml: YAML.dump(payload, { sortKeys: false, lineWidth: -1 }) };
  };

  const buildRangeScratchYaml = () => {
    if (!activePage) {
      return null;
    }
    const scratchIndex = Number(scratchPageIndex);
    if (!Number.isFinite(scratchIndex) || scratchIndex < 1 || scratchIndex > 255) {
      setStatus("Scratch page must be 1-255");
      setStatusError(true);
      return null;
    }
    if (String(rangeStart).trim() === "" || String(rangeEnd).trim() === "") {
      setStatus("Range start/end is required");
      setStatusError(true);
      return null;
    }
    const startRaw = Number(rangeStart);
    const endRaw = Number(rangeEnd);
    if (!Number.isInteger(startRaw) || !Number.isInteger(endRaw)) {
      setStatus("Range steps must be whole numbers");
      setStatusError(true);
      return null;
    }
    if (startRaw < 0 || endRaw < 0 || startRaw > 6 || endRaw > 6) {
      setStatus("Range steps must be between 0 and 6");
      setStatusError(true);
      return null;
    }
    if (startRaw > endRaw) {
      setStatus("Range start must be <= end");
      setStatusError(true);
      return null;
    }

    const header = activePage.header || {};
    const repeatValue =
      Number.isFinite(header.repeat) && Number(header.repeat) > 0
        ? Math.round(Number(header.repeat))
        : 1;
    const scratchHeader = {
      repeat: repeatValue,
      schedule: scheduleValue,
      speed: Number.isFinite(header.speed) ? Math.round(header.speed) : 0,
      accel: Number.isFinite(header.accel) ? Math.round(header.accel) : 0,
      next: 0,
      exit: 0,
    };

    const steps = [];
    for (let idx = startRaw; idx <= endRaw; idx += 1) {
      const sourceStep = findStepByIndex(activePage, idx);
      if (!sourceStep) {
        setStatus(`Missing step ${idx} on page ${activePage.index}`);
        setStatusError(true);
        return null;
      }
      steps.push({
        index: steps.length,
        pause: sourceStep.pause ?? 0,
        time: sourceStep.time ?? 0,
        positions: cloneData(sourceStep.positions || {}),
      });
    }

    const payload = {
      pages: [
        {
          index: scratchIndex,
          name: `scratch_${activePage.index}_steps_${startRaw}_${endRaw}`,
          header: scratchHeader,
          steps,
        },
      ],
    };
    return { scratchIndex, yaml: YAML.dump(payload, { sortKeys: false, lineWidth: -1 }) };
  };

  const queueScratchRun = (scratch, label) => {
    if (!requestRef.current) {
      setStatus("ROS not connected");
      setStatusError(true);
      return;
    }
    const requestId = `scratch-${Date.now()}`;
    pendingRunRef.current = { requestId, pageIndex: scratch.scratchIndex };
    sendRequest({ action: "apply", yaml: scratch.yaml, request_id: requestId });
    setStatus(label);
    setStatusError(false);
  };

  const handleRunStep = () => {
    if (!activePage || !activeStep) {
      return;
    }
    const scratch = buildScratchYaml();
    if (!scratch) {
      return;
    }
    queueScratchRun(scratch, `Writing scratch page ${scratch.scratchIndex}...`);
  };

  const handleRunRange = () => {
    if (!activePage) {
      return;
    }
    const scratch = buildRangeScratchYaml();
    if (!scratch) {
      return;
    }
    queueScratchRun(
      scratch,
      `Writing scratch page ${scratch.scratchIndex} (steps ${rangeStart}-${rangeEnd})...`
    );
  };

  const handleStopPage = () => {
    publishActionPage(-1);
  };

  const handleBrakePage = () => {
    publishActionPage(-2);
  };

  const handleSavePreset = () => {
    if (!activeStep) {
      return;
    }
    const name = presetName.trim();
    if (!name) {
      setStatus("Preset name is required");
      setStatusError(true);
      return;
    }
    const snapshot = cloneData(activeStep.positions || {});
    setPosePresets((prev) => {
      const filtered = prev.filter((preset) => preset.name !== name);
      return [{ name, positions: snapshot }, ...filtered].slice(0, 40);
    });
    setPresetName("");
    setStatus(`Saved preset ${name}`);
    setStatusError(false);
  };

  const handleApplyPreset = (preset) => {
    if (!preset || !activeStep) {
      return;
    }
    pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    updateActiveStep((step) => {
      step.positions = cloneData(preset.positions || {});
    });
    setStatus(`Applied preset ${preset.name}`);
    setStatusError(false);
  };

  const handleDeletePreset = (name) => {
    setPosePresets((prev) => prev.filter((preset) => preset.name !== name));
    setStatus(`Deleted preset ${name}`);
    setStatusError(false);
  };

  const handlePageNameChange = (value) => {
    updateActivePage((page) => {
      page.name = value;
    });
  };

  const handlePageHeaderNumber = (field, value) => {
    const parsed = Number(value);
    if (!Number.isFinite(parsed)) {
      return;
    }
    updateActivePage((page) => {
      page.header = page.header || {};
      page.header[field] = Math.round(parsed);
    });
  };

  const handlePageHeaderSchedule = (value) => {
    updateActivePage((page) => {
      page.header = page.header || {};
      page.header.schedule = value;
    });
  };

  const handleStepFieldNumber = (field, value) => {
    const parsed = Number(value);
    if (!Number.isFinite(parsed)) {
      return;
    }
    if (activeStep) {
      pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    }
    updateActiveStep((step) => {
      step[field] = Math.round(parsed);
    });
  };

  const jointDraftKey = (name) => {
    const pageKey = selectedPageIndex ?? "none";
    const stepKey = selectedStepIndex ?? "none";
    return `${pageKey}:${stepKey}:${name}`;
  };

  const handleJointDraftChange = (name, value) => {
    const key = jointDraftKey(name);
    setJointDrafts((prev) => ({ ...prev, [key]: value }));
  };

  const clearJointDraft = (name) => {
    const key = jointDraftKey(name);
    setJointDrafts((prev) => {
      if (!Object.prototype.hasOwnProperty.call(prev, key)) {
        return prev;
      }
      const next = { ...prev };
      delete next[key];
      return next;
    });
  };

  const commitJointDraft = (name, value) => {
    if (value === "" || value === null || value === undefined) {
      clearJointDraft(name);
      return;
    }
    const parsed = Number(value);
    if (!Number.isFinite(parsed)) {
      return;
    }
    if (activeStep) {
      pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    }
    const rawValue = toRawDegrees(parsed);
    updateActiveStep((step) => {
      step.positions = step.positions || {};
      setRawPosition(step.positions, name, jointIdMap, rawValue);
    });
    clearJointDraft(name);
  };

  const handleJointToggleOff = (name, currentRaw) => {
    const nextValue = isTorqueOff(currentRaw) ? RAW_CENTER : "torque_off";
    if (activeStep) {
      pushStepHistory(stepHistoryKey(), snapshotStep(activeStep));
    }
    updateActiveStep((step) => {
      step.positions = step.positions || {};
      setRawPosition(step.positions, name, jointIdMap, nextValue);
    });
    clearJointDraft(name);
  };

  const sendStepPoseToWebots = (step) => {
    if (!step) {
      return;
    }
    if (!jointPubRef.current) {
      setStatus("ROS not connected");
      setStatusError(true);
      return;
    }
    const pose = buildPose(step.positions || {}, livePoseRef.current);
    const data = JOINT_ORDER.map((name) => pose[name] ?? 0);
    jointPubRef.current.publish(new ROSLIB.Message({ data }));
    setStatus("Sent step pose to Webots");
    setStatusError(false);
  };

  const recordStepSelection = (page, stepIndex) => {
    if (!recordEnabled || !page) {
      return;
    }
    const now = window.performance.now();
    const record = recordRef.current;
    const samePage = record.lastPageIndex === page.index;
    if (!samePage || record.lastTime === null || record.lastStepIndex === null) {
      record.lastTime = now;
      record.lastStepIndex = stepIndex;
      record.lastPageIndex = page.index;
      setRecordElapsed(0);
      setRecordLastDelta(null);
      setRecordLastTicks(null);
      setStatus(`Record start at step ${stepIndex}`);
      setStatusError(false);
      return;
    }

    const deltaSec = (now - record.lastTime) / 1000;
    const speed = resolveSpeed(page.header);
    const schedule = parseSchedule(page.header);

    if (recordMode === "time" && schedule !== "time") {
      setStatus("Record time needs schedule=time");
      setStatusError(true);
      record.lastTime = now;
      record.lastStepIndex = stepIndex;
      record.lastPageIndex = page.index;
      return;
    }

    let targetIndex = stepIndex;
    let ticks = 0;
    let label = "time";
    if (recordMode === "time") {
      ticks = secondsToTimeTicks(deltaSec, speed);
      ticks = Math.max(1, ticks);
    } else {
      ticks = secondsToPauseTicks(deltaSec, speed);
      targetIndex = record.lastStepIndex;
      label = "pause";
    }

    const targetStep = findStepByIndex(page, targetIndex);
    const key = stepHistoryKey(page.index, targetIndex);
    if (targetStep) {
      pushStepHistory(key, snapshotStep(targetStep));
      updateStepByIndex(page.index, targetIndex, (step) => {
        step[label] = ticks;
      });
    }

    setRecordLastDelta(deltaSec);
    setRecordLastTicks(ticks);
    setStatus(`Recorded ${deltaSec.toFixed(2)}s -> ${label} ${ticks}`);
    setStatusError(false);

    record.lastTime = now;
    record.lastStepIndex = stepIndex;
    record.lastPageIndex = page.index;
  };

  const handleStepClick = (page, step, stepIndex) => {
    recordStepSelection(page, stepIndex);
    const pose = buildPose(step.positions || {}, livePoseRef.current);
    setSelectedPageIndex(page.index);
    setSelectedStepIndex(stepIndex);
    setPreviewPose(pose);
    if (sendStepToWebots && jointPubRef.current) {
      const data = JOINT_ORDER.map((name) => pose[name] ?? 0);
      jointPubRef.current.publish(new ROSLIB.Message({ data }));
    }
  };

  const clearPreview = () => {
    setPreviewPose(null);
    setSelectedStepIndex(null);
  };

  const activeHeader = activePage?.header || {};
  const activeStepLabel =
    activeStep && activeStep.index !== undefined
      ? activeStep.index
      : selectedStepIndex ?? "";
  const scheduleValue =
    activeHeader.schedule === "time" ||
    activeHeader.schedule === 10 ||
    activeHeader.schedule === "0x0a"
      ? "time"
      : "speed";

  return (
    <div className="app">
      <header className="hero">
        <div>
          <p className="kicker">BASCORRO op3 Action Studio</p>
          <h1>Pose editor with live Webots preview.</h1>
          <p className="subtle">
            Paste or export YAML, click a step, and the robot mirrors it in 3D.
          </p>
        </div>
        <div className="hero-status">
          <div className={`pill ${rosState}`}>ROS: {rosState}</div>
          <div className="pill">Assets: {assetsUrl}</div>
        </div>
      </header>

      <main className="grid">
        <div className="left-stack">
          <section className="panel browser">
            <div className="panel-header">
              <h2>Pages</h2>
              <div className="actions">
                <button className="ghost" type="button" onClick={clearPreview}>
                  Live Pose
                </button>
              </div>
            </div>
            <input
              className="page-filter"
              type="text"
              value={pageFilter}
              onChange={(event) => setPageFilter(event.target.value)}
              placeholder="Filter pages by name or index"
            />
            <div className="page-list">
              {pages.length === 0 && (
                <div className="empty">No pages loaded. Export first.</div>
              )}
              {visiblePages.map((page) => (
                <button
                  key={page.index}
                  type="button"
                  className={`page-button${
                    selectedPageIndex !== null && page.index === selectedPageIndex
                      ? " active"
                      : ""
                  }`}
                  onClick={() => {
                    setSelectedPageIndex(page.index);
                    setSelectedStepIndex(null);
                    setPreviewPose(null);
                  }}
                >
                  <span className="page-index">{page.index}</span>
                  <span className="page-name">
                    {page.name || "Untitled"}
                  </span>
                  <span className="page-steps">
                    {page.steps ? page.steps.length : 0} steps
                  </span>
                </button>
              ))}
            </div>
            <div className="step-list">
              <div className="step-title">
                {activePage
                  ? `Steps for page ${activePage.index}`
                  : "Select a page"}
              </div>
              {activePage && activePage.steps && activePage.steps.length > 0 ? (
                activePage.steps.map((step, stepIdx) => {
                  const stepKey =
                    step && step.index !== undefined ? Number(step.index) : stepIdx;
                  return (
                    <button
                      key={stepKey}
                      type="button"
                      className={`step-button${
                        selectedStepIndex !== null && stepKey === selectedStepIndex
                          ? " active"
                          : ""
                      }`}
                      onClick={() => handleStepClick(activePage, step, stepKey)}
                    >
                      <span>Step {stepKey}</span>
                      <span>
                        pause {step.pause} | time {step.time}
                      </span>
                    </button>
                  );
                })
              ) : (
                <div className="empty">No steps on this page.</div>
              )}
            </div>
          </section>

          <section className={`panel editor${showYaml ? "" : " collapsed"}`}>
            <div className="panel-header">
              <h2>YAML Deck</h2>
              <div className="actions">
                <button
                  className="ghost"
                  type="button"
                  onClick={() => setShowYaml((value) => !value)}
                >
                  {showYaml ? "Hide" : "Show"}
                </button>
                <input
                  type="text"
                  value={exportPages}
                  onChange={(event) => setExportPages(event.target.value)}
                  placeholder="used | all | 120,121"
                />
                <button className="ghost" type="button" onClick={handleExport}>
                  Export
                </button>
                <button className="primary" type="button" onClick={handleApply}>
                  Apply
                </button>
              </div>
            </div>
            {showYaml ? (
              <>
                <textarea
                  className="yaml-input"
                  value={yamlText}
                  onChange={(event) => setYamlText(event.target.value)}
                  placeholder="Paste YAML from action_yaml.py export..."
                  spellCheck="false"
                />
                <div className="panel-footer">
                  <span className={parseError ? "error" : "ok"}>
                    {parseError ? `Parse: ${parseError}` : "Parse: OK"}
                  </span>
                  <span className={statusError ? "error" : "ok"}>{status}</span>
                </div>
              </>
            ) : (
              <div className="panel-footer">
                <span className={parseError ? "error" : "ok"}>
                  {parseError ? `Parse: ${parseError}` : "Parse: OK"}
                </span>
                <span className={statusError ? "error" : "ok"}>{status}</span>
              </div>
            )}
          </section>
        </div>

        <div className="middle-stack">
          <section className="panel page-editor">
            <div className="panel-header">
              <h2>Page Editor</h2>
              <div className="actions">
                <label className="toggle">
                  <input
                    type="checkbox"
                    checked={autoEnableAction}
                    onChange={(event) => setAutoEnableAction(event.target.checked)}
                  />
                  <span>Auto enable</span>
                </label>
                <button
                  className="ghost"
                  type="button"
                  onClick={publishEnableActionModule}
                  disabled={rosState !== "connected"}
                >
                  Enable
                </button>
                <button
                  className="ghost"
                  type="button"
                  onClick={handleRunPage}
                  disabled={!activePage || rosState !== "connected"}
                >
                  Run
                </button>
                <button
                  className="ghost"
                  type="button"
                  onClick={handleStopPage}
                  disabled={rosState !== "connected"}
                >
                  Stop
                </button>
                <button
                  className="ghost"
                  type="button"
                  onClick={handleBrakePage}
                  disabled={rosState !== "connected"}
                >
                  Brake
                </button>
              </div>
            </div>
            {!activePage ? (
              <div className="empty">Select a page to edit.</div>
            ) : (
              <>
                <div className="page-fields">
                  <label>
                    Page
                    <input type="text" value={activePage.index} readOnly />
                  </label>
                  <label>
                    Name
                    <input
                      type="text"
                      value={activePage.name || ""}
                      onChange={(event) => handlePageNameChange(event.target.value)}
                      disabled={editorDisabled}
                    />
                  </label>
                  <label>
                    Schedule
                    <select
                      value={scheduleValue}
                      onChange={(event) =>
                        handlePageHeaderSchedule(event.target.value)
                      }
                      disabled={editorDisabled}
                    >
                      <option value="speed">speed</option>
                      <option value="time">time</option>
                    </select>
                  </label>
                  <label>
                    Repeat
                    <input
                      type="number"
                      value={activeHeader.repeat ?? 0}
                      onChange={(event) =>
                        handlePageHeaderNumber("repeat", event.target.value)
                      }
                      disabled={editorDisabled}
                    />
                  </label>
                  <label>
                    Speed
                    <input
                      type="number"
                      value={activeHeader.speed ?? 0}
                      onChange={(event) =>
                        handlePageHeaderNumber("speed", event.target.value)
                      }
                      disabled={editorDisabled}
                    />
                  </label>
                  <label>
                    Accel
                    <input
                      type="number"
                      value={activeHeader.accel ?? 0}
                      onChange={(event) =>
                        handlePageHeaderNumber("accel", event.target.value)
                      }
                      disabled={editorDisabled}
                    />
                  </label>
                  <label>
                    Next
                    <input
                      type="number"
                      value={activeHeader.next ?? 0}
                      onChange={(event) =>
                        handlePageHeaderNumber("next", event.target.value)
                      }
                      disabled={editorDisabled}
                    />
                  </label>
                  <label>
                    Exit
                    <input
                      type="number"
                      value={activeHeader.exit ?? 0}
                      onChange={(event) =>
                        handlePageHeaderNumber("exit", event.target.value)
                      }
                      disabled={editorDisabled}
                    />
                  </label>
                </div>

                <div className="step-editor">
                  <div className="panel-header compact">
                    <h3>Step Editor</h3>
                    <div className="actions">
                      <button
                        className="ghost"
                        type="button"
                        onClick={handleUndoStep}
                        disabled={editorDisabled || !canUndo}
                        title="Undo (Ctrl+Z)"
                      >
                        Undo
                      </button>
                      <button
                        className="ghost"
                        type="button"
                        onClick={handleRedoStep}
                        disabled={editorDisabled || !canRedo}
                        title="Redo (Ctrl+Shift+Z)"
                      >
                        Redo
                      </button>
                      <label className="toggle">
                        <input
                          type="checkbox"
                          checked={recordEnabled}
                          onChange={(event) =>
                            setRecordEnabled(event.target.checked)
                          }
                          disabled={editorDisabled}
                        />
                        <span>Record</span>
                      </label>
                      <select
                        className="record-select"
                        value={recordMode}
                        onChange={(event) => setRecordMode(event.target.value)}
                        disabled={editorDisabled || !recordEnabled}
                      >
                        <option value="time">to time</option>
                        <option value="pause">to pause</option>
                      </select>
                      <span
                        className={`record-indicator${
                          recordEnabled ? " active" : ""
                        }`}
                      >
                        {recordEnabled
                          ? recordLastDelta !== null
                            ? `Δ ${recordLastDelta.toFixed(2)}s · ${recordMode} ${recordLastTicks ?? "-"}`
                            : `Δ ${recordElapsed.toFixed(2)}s`
                          : "Record off"}
                      </span>
                      <label className="toggle">
                        <input
                          type="checkbox"
                          checked={sendStepToWebots}
                          onChange={(event) =>
                            setSendStepToWebots(event.target.checked)
                          }
                          disabled={editorDisabled}
                        />
                        <span>Auto send</span>
                      </label>
                      <label className="toggle">
                        <input
                          type="checkbox"
                          checked={showAllJoints}
                          onChange={(event) =>
                            setShowAllJoints(event.target.checked)
                          }
                          disabled={editorDisabled}
                        />
                        <span>All joints</span>
                      </label>
                      <span className="pill small">deg</span>
                    </div>
                  </div>
                  {activeStep ? (
                    <>
                      <div className="step-fields">
                        <label>
                          Step
                          <input type="text" value={activeStepLabel} readOnly />
                        </label>
                        <label>
                          Pause
                          <input
                            type="number"
                            value={activeStep.pause ?? 0}
                            onChange={(event) =>
                              handleStepFieldNumber("pause", event.target.value)
                            }
                            disabled={editorDisabled}
                          />
                        </label>
                        <label>
                          Time
                          <input
                            type="number"
                            value={activeStep.time ?? 0}
                            onChange={(event) =>
                              handleStepFieldNumber("time", event.target.value)
                            }
                            disabled={editorDisabled}
                          />
                        </label>
                      </div>

                      <div className="step-actions">
                        <label>
                          Scratch page
                          <input
                            type="number"
                            value={scratchPageIndex}
                            onChange={(event) =>
                              setScratchPageIndex(Number(event.target.value))
                            }
                            disabled={editorDisabled}
                          />
                        </label>
                        <button
                          className="ghost"
                          type="button"
                          onClick={() => sendStepPoseToWebots(activeStep)}
                          disabled={editorDisabled || !activeStep || rosState !== "connected"}
                        >
                          Send Step
                        </button>
                        <button
                          className="ghost"
                          type="button"
                          onClick={handleRunStep}
                          disabled={
                            editorDisabled ||
                            !activeStep ||
                            rosState !== "connected"
                          }
                        >
                          Run Step
                        </button>
                        <div className="range-field">
                          <span>Range</span>
                          <div className="range-inputs">
                            <input
                              type="number"
                              value={rangeStart}
                              onChange={(event) => setRangeStart(event.target.value)}
                              disabled={editorDisabled}
                            />
                            <span>to</span>
                            <input
                              type="number"
                              value={rangeEnd}
                              onChange={(event) => setRangeEnd(event.target.value)}
                              disabled={editorDisabled}
                            />
                          </div>
                        </div>
                        <button
                          className="ghost"
                          type="button"
                          onClick={handleRunRange}
                          disabled={
                            editorDisabled || !activePage || rosState !== "connected"
                          }
                        >
                          Run Range
                        </button>
                      </div>

                      <div className="preset-panel">
                        <div className="preset-header">
                          <span>Presets</span>
                          <div className="preset-actions">
                            <input
                              type="text"
                              value={presetName}
                              onChange={(event) =>
                                setPresetName(event.target.value)
                              }
                              placeholder="pose name"
                              disabled={editorDisabled}
                            />
                            <button
                              className="ghost"
                              type="button"
                              onClick={handleSavePreset}
                              disabled={editorDisabled || !activeStep}
                            >
                              Save
                            </button>
                          </div>
                        </div>
                        {posePresets.length ? (
                          <div className="preset-list">
                            {posePresets.map((preset) => (
                              <div className="preset-row" key={preset.name}>
                                <span className="preset-name">{preset.name}</span>
                                <div className="preset-buttons">
                                  <button
                                    className="ghost"
                                    type="button"
                                    onClick={() => handleApplyPreset(preset)}
                                    disabled={editorDisabled || !activeStep}
                                  >
                                    Apply
                                  </button>
                                  <button
                                    className="ghost"
                                    type="button"
                                    onClick={() => handleDeletePreset(preset.name)}
                                    disabled={editorDisabled}
                                  >
                                    Delete
                                  </button>
                                </div>
                              </div>
                            ))}
                          </div>
                        ) : (
                          <div className="empty small">
                            No presets saved yet.
                          </div>
                        )}
                      </div>

                      <div className="joint-grid">
                        {jointNames.map((name) => {
                          const rawValue = lookupRawPosition(
                            activeStep.positions || {},
                            name,
                            jointIdMap
                          );
                          const label = formatJointLabel(name);
                          const normalized = normalizeRaw(rawValue);
                          const isOff = isTorqueOff(rawValue);
                          const degValue =
                            normalized === null ? "" : toDegrees(normalized).toFixed(1);
                          const degNumber = normalized === null ? 0 : toDegrees(normalized);
                          const key = jointDraftKey(name);
                          const draftValue =
                            Object.prototype.hasOwnProperty.call(jointDrafts, key)
                              ? jointDrafts[key]
                              : degValue;
                          const draftNumber = Number(draftValue);
                          const sliderValue = Number.isFinite(draftNumber)
                            ? draftNumber
                            : degNumber;
                          const rawLabel = isOff
                            ? "off"
                            : normalized === null
                              ? "n/a"
                              : normalized;

                          return (
                            <div className="joint-row" key={name}>
                              <div className="joint-name" title={label}>
                                <span className="joint-code">{name}</span>
                                <span className="joint-label">{label}</span>
                              </div>
                              <input
                                className="joint-slider"
                                type="range"
                                min="-180"
                                max="180"
                                step="0.5"
                                value={sliderValue}
                                onChange={(event) =>
                                  commitJointDraft(name, event.target.value)
                                }
                                disabled={editorDisabled || isOff}
                              />
                              <input
                                type="number"
                                step="0.1"
                                value={draftValue}
                                onChange={(event) =>
                                  handleJointDraftChange(name, event.target.value)
                                }
                                onBlur={(event) =>
                                  commitJointDraft(name, event.target.value)
                                }
                                onKeyDown={(event) => {
                                  if (event.key === "Enter") {
                                    commitJointDraft(name, event.target.value);
                                    event.currentTarget.blur();
                                  }
                                }}
                                disabled={editorDisabled || isOff}
                              />
                              <div className="joint-raw">{rawLabel}</div>
                              <button
                                type="button"
                                className={`chip ${isOff ? "active" : ""}`}
                                onClick={() => handleJointToggleOff(name, rawValue)}
                                disabled={editorDisabled}
                              >
                                Off
                              </button>
                            </div>
                          );
                        })}
                      </div>

                      <div className="joint-guide">
                        <button
                          className="ghost"
                          type="button"
                          onClick={() => setShowJointGuide((value) => !value)}
                        >
                          {showJointGuide ? "Hide joint guide" : "Show joint guide"}
                        </button>
                        {showJointGuide ? (
                          <div className="joint-guide-list">
                            {jointNames.map((name) => (
                              <div className="joint-guide-row" key={name}>
                                <span className="joint-code">{name}</span>
                                <span className="joint-label">
                                  {formatJointLabel(name)}
                                </span>
                              </div>
                            ))}
                          </div>
                        ) : null}
                      </div>
                    </>
                  ) : (
                    <div className="empty">Select a step to edit joints.</div>
                  )}
                </div>

                <div className="panel-footer">
                  <span className="hint">
                    Run publishes to /robotis/action/page_num (action module must be
                    running).
                  </span>
                </div>
              </>
            )}
          </section>
        </div>

        <div className="right-stack">
          <section className="panel viewer">
            <div className="panel-header">
              <h2>Preview</h2>
              <div className="actions">
                <button
                  className="ghost"
                  type="button"
                  onClick={() => rotateView(-90)}
                >
                  Rotate Left
                </button>
                <button
                  className="ghost"
                  type="button"
                  onClick={() => rotateView(90)}
                >
                  Rotate Right
                </button>
                <button className="ghost" type="button" onClick={resetView}>
                  Reset View
                </button>
                <button
                  className="ghost"
                  type="button"
                  onClick={() => setViewerEnabled((value) => !value)}
                >
                  {viewerEnabled ? "Hide Preview" : "Show Preview"}
                </button>
                <label className="toggle">
                  <input
                    type="checkbox"
                    checked={upright}
                    onChange={(event) => setUpright(event.target.checked)}
                  />
                  <span>Upright</span>
                </label>
                <label className="toggle">
                  <input
                    type="checkbox"
                    checked={layFlat}
                    onChange={(event) => setLayFlat(event.target.checked)}
                  />
                  <span>Lay flat</span>
                </label>
                <label className="toggle">
                  <input
                    type="checkbox"
                    checked={mirrorView}
                    onChange={(event) => setMirrorView(event.target.checked)}
                  />
                  <span>Mirror</span>
                </label>
                <span className="pill">{previewPose ? "Preview" : "Live"}</span>
              </div>
            </div>
            {viewerEnabled ? (
              <div className="viewer-canvas" ref={viewerRef} />
            ) : (
              <div className="viewer-placeholder">
                <p>Preview hidden to save GPU.</p>
                <button
                  className="ghost"
                  type="button"
                  onClick={() => setViewerEnabled(true)}
                >
                  Show Preview
                </button>
              </div>
            )}
            <div className="panel-footer">
              <span className="hint">
                {viewerEnabled
                  ? "Select a step to preview in 3D. Use Send Step or Auto send to move Webots."
                  : "Preview is disabled. Re-enable when you need it."}
              </span>
              {viewerNote ? <span className="viewer-note">{viewerNote}</span> : null}
            </div>
          </section>

          <section className="panel connection">
            <div className="panel-header">
              <h2>Connections</h2>
              <div className="actions">
                <span className={`pill small ${rosState}`}>{rosState}</span>
              </div>
            </div>
            <label>
              Rosbridge URL
              <input
                type="text"
                value={rosUrl}
                onChange={(event) => setRosUrl(event.target.value)}
                placeholder="ws://localhost:9090"
              />
            </label>
            <label>
              Assets URL
              <input
                type="text"
                value={assetsUrl}
                onChange={(event) => setAssetsUrl(event.target.value)}
                placeholder="http://localhost:8001"
              />
            </label>
            <p className="subtle">
              Change URLs to match your tmux stack or remote machine.
            </p>
          </section>
        </div>
      </main>
    </div>
  );
}
