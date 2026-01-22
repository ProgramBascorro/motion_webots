import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import ActionEditor from "./ActionEditor.jsx";
import GamepadVisualizer from "./GamepadVisualizer.jsx";

const DEFAULT_ROSBRIDGE =
  import.meta.env.VITE_ROSBRIDGE_URL || "ws://localhost:9090";
const DEFAULT_OVERLAY_TOPIC =
  import.meta.env.VITE_OVERLAY_TOPIC || "/vision/yolo/debug";

const PARAM_TYPES = {
  bool: 1,
  int: 2,
  double: 3,
  string: 4,
};

const YOLO_PARAM_KEYS = [
  "ball_confidence_threshold",
  "goalpost_confidence_threshold",
  "robot_confidence_threshold",
];

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function formatNumber(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "n/a";
  }
  return Number(value).toFixed(digits);
}

function formatPercent(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "n/a";
  }
  return `${Math.round(value)}%`;
}

function formatAge(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(seconds)) {
    return "n/a";
  }
  if (seconds < 1) {
    return `${Math.round(seconds * 1000)}ms`;
  }
  return `${seconds.toFixed(1)}s`;
}

function decodeImage(msg) {
  if (!msg || !msg.data || !msg.width || !msg.height) {
    return null;
  }
  const { width, height, encoding } = msg;
  const bin = atob(msg.data);
  const raw = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i += 1) {
    raw[i] = bin.charCodeAt(i);
  }

  const pixels = new Uint8ClampedArray(width * height * 4);
  if (encoding === "rgb8" || encoding === "bgr8") {
    const isBgr = encoding === "bgr8";
    for (let i = 0; i < width * height; i += 1) {
      const offset = i * 3;
      const r = isBgr ? raw[offset + 2] : raw[offset];
      const g = raw[offset + 1];
      const b = isBgr ? raw[offset] : raw[offset + 2];
      const out = i * 4;
      pixels[out] = r;
      pixels[out + 1] = g;
      pixels[out + 2] = b;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  if (encoding === "mono8") {
    for (let i = 0; i < width * height; i += 1) {
      const val = raw[i];
      const out = i * 4;
      pixels[out] = val;
      pixels[out + 1] = val;
      pixels[out + 2] = val;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  return null;
}

function makeParamValue(type, rawValue) {
  const value = String(rawValue ?? "").trim();
  if (type === "bool") {
    return { type: PARAM_TYPES.bool, bool_value: value === "true" };
  }
  if (type === "int") {
    return { type: PARAM_TYPES.int, integer_value: Number(value) };
  }
  if (type === "string") {
    return { type: PARAM_TYPES.string, string_value: value };
  }
  return { type: PARAM_TYPES.double, double_value: Number(value) };
}

export default function App() {
  const [view, setView] = useState("studio");
  const [rosUrl, setRosUrl] = useState(DEFAULT_ROSBRIDGE);
  const [overlayTopic, setOverlayTopic] = useState(DEFAULT_OVERLAY_TOPIC);
  const [rosState, setRosState] = useState("disconnected");
  const [metrics, setMetrics] = useState(null);
  const [events, setEvents] = useState([]);
  const [overlayStats, setOverlayStats] = useState({
    fps: 0,
    lastFrameMs: null,
    dropped: 0,
  });
  const [studioStatus, setStudioStatus] = useState("");
  const [studioError, setStudioError] = useState(false);
  const [yoloParams, setYoloParams] = useState({
    ball_confidence_threshold: 0.2,
    goalpost_confidence_threshold: 0.5,
    robot_confidence_threshold: 0.2,
  });
  const [walkingParams, setWalkingParams] = useState({
    x_move_amplitude: 0.0,
    y_move_amplitude: 0.0,
    angle_move_amplitude: 0.0,
  });
  const [walkingFull, setWalkingFull] = useState(null);
  const [paramNode, setParamNode] = useState("op3_yolo_vision");
  const [paramName, setParamName] = useState("ball_confidence_threshold");
  const [paramType, setParamType] = useState("double");
  const [paramValue, setParamValue] = useState("0.2");
  const [joyState, setJoyState] = useState(null);
  const [showOverlay, setShowOverlay] = useState(true);

  const rosRef = useRef(null);
  const overlayCanvasRef = useRef(null);
  const overlayLastRef = useRef({ stampMs: null, fps: 0, dropped: 0 });
  const showOverlayRef = useRef(true);
  const metricsSubRef = useRef(null);
  const eventsSubRef = useRef(null);
  const overlaySubRef = useRef(null);
  const joySubRef = useRef(null);
  const joyPubRef = useRef(null);
  const initPosePubRef = useRef(null);
  const walkingCommandPubRef = useRef(null);
  const torquePubRef = useRef(null);
  const walkingParamPubRef = useRef(null);
  const walkingGetServiceRef = useRef(null);
  const yoloSetServiceRef = useRef(null);
  const yoloGetServiceRef = useRef(null);
  const snapshotServiceRef = useRef(null);
  const bagStartServiceRef = useRef(null);
  const bagStopServiceRef = useRef(null);

  useEffect(() => {
    showOverlayRef.current = showOverlay;
  }, [showOverlay]);

  useEffect(() => {
    if (!rosUrl) {
      return undefined;
    }

    const ros = new ROSLIB.Ros({ url: rosUrl });
    rosRef.current = ros;

    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    metricsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/metrics",
      messageType: "std_msgs/String",
    });
    eventsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/events",
      messageType: "std_msgs/String",
    });
    joySubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    joyPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    initPosePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/base/ini_pose",
      messageType: "std_msgs/String",
    });
    walkingCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/command",
      messageType: "std_msgs/String",
    });
    torquePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/sync_write_item",
      messageType: "robotis_controller_msgs/SyncWriteItem",
    });
    walkingParamPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/set_params",
      messageType: "op3_walking_module_msgs/WalkingParam",
    });
    walkingGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/walking/get_params",
      serviceType: "op3_walking_module_msgs/srv/GetWalkingParam",
    });
    yoloSetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    yoloGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    snapshotServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/snapshot",
      serviceType: "std_srvs/srv/Trigger",
    });
    bagStartServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/bag_start",
      serviceType: "std_srvs/srv/Trigger",
    });
    bagStopServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/bag_stop",
      serviceType: "std_srvs/srv/Trigger",
    });

    metricsSubRef.current.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        setMetrics(payload);
      } catch (err) {
        setStudioStatus("Metrics parse error");
        setStudioError(true);
      }
    });

    eventsSubRef.current.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        if (Array.isArray(payload.events)) {
          setEvents(payload.events);
        }
      } catch (err) {
        setStudioStatus("Events parse error");
        setStudioError(true);
      }
    });

    joySubRef.current.subscribe((msg) => setJoyState(msg));

    const loadYoloParams = () => {
      if (!yoloGetServiceRef.current) {
        return;
      }
      const request = new ROSLIB.ServiceRequest({
        names: YOLO_PARAM_KEYS,
      });
      yoloGetServiceRef.current.callService(request, (result) => {
        if (!result || !Array.isArray(result.values)) {
          return;
        }
        const next = { ...yoloParams };
        result.values.forEach((value, idx) => {
          const key = YOLO_PARAM_KEYS[idx];
          if (key && value && typeof value.double_value === "number") {
            next[key] = value.double_value;
          }
        });
        setYoloParams(next);
      });
    };

    loadYoloParams();

    return () => {
      metricsSubRef.current?.unsubscribe();
      eventsSubRef.current?.unsubscribe();
      joySubRef.current?.unsubscribe();
      joyPubRef.current = null;
      ros.close();
    };
  }, [rosUrl]);

  useEffect(() => {
    if (!rosRef.current || rosState === "disconnected") {
      return undefined;
    }
    overlaySubRef.current?.unsubscribe();
    overlaySubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: overlayTopic,
      messageType: "sensor_msgs/Image",
    });

    overlaySubRef.current.subscribe((msg) => {
      if (!showOverlayRef.current) {
        return;
      }
      const imageData = decodeImage(msg);
      if (!imageData) {
        return;
      }
      const canvas = overlayCanvasRef.current;
      if (!canvas) {
        return;
      }
      if (canvas.width !== imageData.width || canvas.height !== imageData.height) {
        canvas.width = imageData.width;
        canvas.height = imageData.height;
      }
      const ctx = canvas.getContext("2d");
      ctx.putImageData(imageData, 0, 0);

      const now = performance.now();
      const lastStamp = overlayLastRef.current.stampMs;
      let fps = overlayLastRef.current.fps;
      if (lastStamp) {
        const delta = now - lastStamp;
        if (delta > 0) {
          fps = 1000 / delta;
        }
      }
      overlayLastRef.current = {
        stampMs: now,
        fps,
        dropped: overlayLastRef.current.dropped,
      };
      setOverlayStats({
        fps,
        lastFrameMs: now,
        dropped: overlayLastRef.current.dropped,
      });
    });

    return () => {
      overlaySubRef.current?.unsubscribe();
    };
  }, [overlayTopic, rosState]);

  const battery = metrics?.battery || {};
  const imu = metrics?.imu || {};
  const torque = metrics?.torque || {};
  const system = metrics?.system || {};
  const network = metrics?.network || {};
  const heartbeat = metrics?.heartbeat || {};
  const jointNames = metrics?.joint_names || [];
  const torqueMap = torque?.joints || {};

  const batteryAlert = useMemo(() => {
    if (!battery || battery.voltage === undefined || battery.voltage === null) {
      return "unknown";
    }
    if (battery.voltage <= battery.warn_voltage) {
      return "warn";
    }
    if (battery.voltage <= battery.match_voltage) {
      return "match";
    }
    return "ok";
  }, [battery]);

  const handleInitPose = () => {
    if (!initPosePubRef.current) {
      setStudioStatus("ROS not connected");
      setStudioError(true);
      return;
    }
    initPosePubRef.current.publish(new ROSLIB.Message({ data: "ini_pose" }));
    setStudioStatus("Init pose requested");
    setStudioError(false);
  };

  const handleSoftStop = () => {
    if (!walkingCommandPubRef.current) {
      setStudioStatus("ROS not connected");
      setStudioError(true);
      return;
    }
    walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: "stop" }));
    setStudioStatus("Soft stop sent");
    setStudioError(false);
  };

  const handleTorque = (enable) => {
    if (!torquePubRef.current || jointNames.length === 0) {
      setStudioStatus("No joint list available");
      setStudioError(true);
      return;
    }
    const values = jointNames.map(() => (enable ? 1 : 0));
    torquePubRef.current.publish(
      new ROSLIB.Message({
        item_name: "torque_enable",
        joint_name: jointNames,
        value: values,
      })
    );
    setStudioStatus(enable ? "Torque ON requested" : "Torque OFF requested");
    setStudioError(false);
  };

  const callTrigger = (serviceRef, label) => {
    if (!serviceRef.current) {
      setStudioStatus("Service not available");
      setStudioError(true);
      return;
    }
    serviceRef.current.callService(new ROSLIB.ServiceRequest({}), (result) => {
      if (!result || !result.success) {
        setStudioStatus(`${label} failed: ${result?.message || "error"}`);
        setStudioError(true);
        return;
      }
      setStudioStatus(result.message || `${label} OK`);
      setStudioError(false);
    });
  };

  const handleSnapshot = () => callTrigger(snapshotServiceRef, "Snapshot");
  const handleBagStart = () => callTrigger(bagStartServiceRef, "Bag start");
  const handleBagStop = () => callTrigger(bagStopServiceRef, "Bag stop");

  const publishJoy = useCallback(
    (payload) => {
      if (!joyPubRef.current || rosState !== "connected") {
        return false;
      }
      const now = Date.now();
      joyPubRef.current.publish(
        new ROSLIB.Message({
          header: {
            stamp: {
              sec: Math.floor(now / 1000),
              nanosec: (now % 1000) * 1000000,
            },
            frame_id: "bascorro_studio",
          },
          axes: payload?.axes || [],
          buttons: payload?.buttons || [],
        })
      );
      return true;
    },
    [rosState]
  );

  const applyYoloParams = () => {
    if (!yoloSetServiceRef.current) {
      setStudioStatus("YOLO param service not available");
      setStudioError(true);
      return;
    }
    const parameters = YOLO_PARAM_KEYS.map((name) => ({
      name,
      value: makeParamValue("double", yoloParams[name]),
    }));
    yoloSetServiceRef.current.callService(
      new ROSLIB.ServiceRequest({ parameters }),
      (result) => {
        if (!result || !Array.isArray(result.results)) {
          setStudioStatus("YOLO params update failed");
          setStudioError(true);
          return;
        }
        setStudioStatus("YOLO thresholds updated");
        setStudioError(false);
      }
    );
  };

  const loadWalkingParams = () => {
    if (!walkingGetServiceRef.current) {
      setStudioStatus("Walking param service not available");
      setStudioError(true);
      return;
    }
    walkingGetServiceRef.current.callService(
      new ROSLIB.ServiceRequest({ get_param: true }),
      (result) => {
        if (!result || !result.parameters) {
          setStudioStatus("Walking params unavailable");
          setStudioError(true);
          return;
        }
        setWalkingFull(result.parameters);
        setWalkingParams({
          x_move_amplitude: result.parameters.x_move_amplitude ?? 0.0,
          y_move_amplitude: result.parameters.y_move_amplitude ?? 0.0,
          angle_move_amplitude: result.parameters.angle_move_amplitude ?? 0.0,
        });
        setStudioStatus("Walking params loaded");
        setStudioError(false);
      }
    );
  };

  const applyWalkingParams = () => {
    if (!walkingParamPubRef.current || !walkingFull) {
      setStudioStatus("Load walking params first");
      setStudioError(true);
      return;
    }
    const payload = { ...walkingFull };
    payload.x_move_amplitude = Number(walkingParams.x_move_amplitude);
    payload.y_move_amplitude = Number(walkingParams.y_move_amplitude);
    payload.angle_move_amplitude = Number(walkingParams.angle_move_amplitude);
    walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
    setStudioStatus("Walking params sent");
    setStudioError(false);
  };

  const applyParam = () => {
    if (!paramNode || !paramName) {
      setStudioStatus("Parameter name required");
      setStudioError(true);
      return;
    }
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: `/${paramNode}/set_parameters`,
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    const parameters = [
      {
        name: paramName,
        value: makeParamValue(paramType, paramValue),
      },
    ];
    service.callService(
      new ROSLIB.ServiceRequest({ parameters }),
      (result) => {
        if (!result || !Array.isArray(result.results)) {
          setStudioStatus("Parameter update failed");
          setStudioError(true);
          return;
        }
        setStudioStatus("Parameter updated");
        setStudioError(false);
      }
    );
  };

  const torqueEntries = useMemo(() => {
    const entries = Object.entries(torqueMap || {});
    entries.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
    return entries.slice(0, 12);
  }, [torqueMap]);

  const overlayLagSec = overlayStats.lastFrameMs
    ? (performance.now() - overlayStats.lastFrameMs) / 1000
    : null;

  return (
    <div className="studio">
      <header className="studio-header">
        <div className="studio-title">
          <p className="studio-kicker">Bascorro Studio</p>
          <h1>Match-ready control, vision, and telemetry.</h1>
          <p className="studio-sub">
            Health, overlays, tuning, and action pages in one cockpit.
          </p>
        </div>
        <div className="studio-rail">
          <div className={`status-pill ${rosState}`}>ROS: {rosState}</div>
          <div className={`status-pill ${batteryAlert}`}>
            Battery: {formatNumber(battery.voltage, 2)}V ·{" "}
            {formatPercent(battery.percent)}
          </div>
          <div className={`status-pill ${studioError ? "error" : "ok"}`}>
            {studioStatus || "Studio ready"}
          </div>
        </div>
      </header>

      <div className="studio-tabs">
        <button
          type="button"
          className={view === "studio" ? "tab active" : "tab"}
          onClick={() => setView("studio")}
        >
          Studio
        </button>
        <button
          type="button"
          className={view === "action" ? "tab active" : "tab"}
          onClick={() => setView("action")}
        >
          Action Editor
        </button>
      </div>

      {view === "action" ? (
        <div className="studio-action-wrap">
          <ActionEditor />
        </div>
      ) : (
        <main className="studio-grid">
          <section className="card">
            <div className="card-header">
              <h2>Health & Safety</h2>
              <div className="card-actions">
                <button className="ghost" type="button" onClick={handleInitPose}>
                  Panic: Init Pose
                </button>
                <button className="ghost" type="button" onClick={handleSoftStop}>
                  Soft Stop
                </button>
                <button className="ghost" type="button" onClick={() => handleTorque(false)}>
                  Torque Off
                </button>
                <button className="primary" type="button" onClick={() => handleTorque(true)}>
                  Torque On
                </button>
              </div>
            </div>
            <div className="stat-grid">
              <div>
                <h3>Battery</h3>
                <p className="stat">
                  {formatNumber(battery.voltage, 2)}V ·{" "}
                  {formatPercent(battery.percent)}
                </p>
                <p className="hint">
                  Warn {formatNumber(battery.warn_voltage, 1)}V · Match{" "}
                  {formatNumber(battery.match_voltage, 1)}V
                </p>
              </div>
              <div>
                <h3>IMU</h3>
                <p className="stat">
                  Roll {formatNumber(imu.roll, 1)}° · Pitch{" "}
                  {formatNumber(imu.pitch, 1)}° · Yaw {formatNumber(imu.yaw, 1)}°
                </p>
                <p className="hint">
                  Fall: {metrics?.fall?.state || "unknown"} · Last{" "}
                  {metrics?.fall?.last_reason || "n/a"}
                </p>
              </div>
              <div>
                <h3>Torque load</h3>
                <p className="stat">
                  Max {formatNumber(torque.max, 2)} · Avg{" "}
                  {formatNumber(torque.avg, 2)}
                </p>
                <p className="hint">Top joints below.</p>
              </div>
              <div>
                <h3>Servo temp</h3>
                <p className="stat">No data</p>
                <p className="hint">Publish temps to enable heat map.</p>
              </div>
            </div>
            <div className="mini-grid">
              {torqueEntries.length === 0 ? (
                <p className="hint">Torque data not available.</p>
              ) : (
                torqueEntries.map(([joint, value]) => (
                  <div key={joint} className="mini-card">
                    <span>{joint}</span>
                    <strong>{formatNumber(value, 2)}</strong>
                  </div>
                ))
              )}
            </div>
          </section>

          <section className="card vision-card">
            <div className="card-header">
              <h2>Vision Overlay</h2>
              <div className="card-actions">
                <button
                  className="ghost"
                  type="button"
                  onClick={() => setShowOverlay((prev) => !prev)}
                >
                  {showOverlay ? "Pause Overlay" : "Resume Overlay"}
                </button>
                <button className="ghost" type="button" onClick={handleSnapshot}>
                  Snapshot
                </button>
              </div>
            </div>
            <div className="vision-body">
              <div className="vision-frame">
                {showOverlay ? (
                  <canvas ref={overlayCanvasRef} />
                ) : (
                  <div className="vision-placeholder">Overlay paused.</div>
                )}
              </div>
              <div className="vision-meta">
                <div>
                  <h3>Stream</h3>
                  <p className="stat">{overlayTopic}</p>
                  <label className="field">
                    Overlay topic
                    <input
                      type="text"
                      value={overlayTopic}
                      onChange={(event) => setOverlayTopic(event.target.value)}
                    />
                  </label>
                </div>
                <div>
                  <h3>Performance</h3>
                  <p className="stat">
                    {formatNumber(overlayStats.fps, 1)} fps
                  </p>
                  <p className="hint">Last frame: {formatAge(overlayLagSec)}</p>
                </div>
              </div>
            </div>
            <div className="card-footer">
              <div className="yolo-tuning">
                <h3>YOLO thresholds</h3>
                <div className="slider-grid">
                  {YOLO_PARAM_KEYS.map((key) => (
                    <label key={key}>
                      {key.replace(/_/g, " ")}
                      <input
                        type="range"
                        min="0"
                        max="1"
                        step="0.01"
                        value={clamp(yoloParams[key], 0, 1)}
                        onChange={(event) =>
                          setYoloParams((prev) => ({
                            ...prev,
                            [key]: Number(event.target.value),
                          }))
                        }
                      />
                      <span>{formatNumber(yoloParams[key], 2)}</span>
                    </label>
                  ))}
                </div>
                <button className="ghost" type="button" onClick={applyYoloParams}>
                  Apply thresholds
                </button>
              </div>
            </div>
          </section>

          <section className="card">
            <div className="card-header">
              <h2>Teleop & Tuning</h2>
            </div>
            <div className="teleop-grid">
              <div className="teleop-gamepad">
                <GamepadVisualizer
                  joy={joyState}
                  rosConnected={rosState === "connected"}
                  publishJoy={publishJoy}
                />
              </div>
              <div>
                <h3>Walking params</h3>
                <div className="form-grid">
                  <label>
                    X amplitude
                    <input
                      type="number"
                      step="0.001"
                      value={walkingParams.x_move_amplitude}
                      onChange={(event) =>
                        setWalkingParams((prev) => ({
                          ...prev,
                          x_move_amplitude: event.target.value,
                        }))
                      }
                    />
                  </label>
                  <label>
                    Y amplitude
                    <input
                      type="number"
                      step="0.001"
                      value={walkingParams.y_move_amplitude}
                      onChange={(event) =>
                        setWalkingParams((prev) => ({
                          ...prev,
                          y_move_amplitude: event.target.value,
                        }))
                      }
                    />
                  </label>
                  <label>
                    Angle amplitude
                    <input
                      type="number"
                      step="0.001"
                      value={walkingParams.angle_move_amplitude}
                      onChange={(event) =>
                        setWalkingParams((prev) => ({
                          ...prev,
                          angle_move_amplitude: event.target.value,
                        }))
                      }
                    />
                  </label>
                </div>
                <div className="row-actions">
                  <button className="ghost" type="button" onClick={loadWalkingParams}>
                    Load
                  </button>
                  <button className="primary" type="button" onClick={applyWalkingParams}>
                    Apply
                  </button>
                </div>
              </div>
              <div>
                <h3>Parameter pad</h3>
                <div className="form-grid">
                  <label>
                    Node
                    <input
                      type="text"
                      value={paramNode}
                      onChange={(event) => setParamNode(event.target.value)}
                    />
                  </label>
                  <label>
                    Param
                    <input
                      type="text"
                      value={paramName}
                      onChange={(event) => setParamName(event.target.value)}
                    />
                  </label>
                  <label>
                    Type
                    <select
                      value={paramType}
                      onChange={(event) => setParamType(event.target.value)}
                    >
                      <option value="double">double</option>
                      <option value="int">int</option>
                      <option value="bool">bool</option>
                      <option value="string">string</option>
                    </select>
                  </label>
                  <label>
                    Value
                    <input
                      type="text"
                      value={paramValue}
                      onChange={(event) => setParamValue(event.target.value)}
                    />
                  </label>
                </div>
                <button className="ghost" type="button" onClick={applyParam}>
                  Apply param
                </button>
              </div>
            </div>
          </section>

          <section className="card">
            <div className="card-header">
              <h2>System & Network</h2>
              <div className="card-actions">
                <button className="ghost" type="button" onClick={handleBagStart}>
                  Start bag
                </button>
                <button className="ghost" type="button" onClick={handleBagStop}>
                  Stop bag
                </button>
              </div>
            </div>
            <div className="stat-grid">
              <div>
                <h3>CPU</h3>
                <p className="stat">{formatPercent(system.cpu_percent)}</p>
                <p className="hint">Load: {formatNumber(system.load, 2)}</p>
              </div>
              <div>
                <h3>Memory</h3>
                <p className="stat">{formatPercent(system.mem_percent)}</p>
                <p className="hint">
                  {formatNumber(system.mem_used_mb, 0)} /{" "}
                  {formatNumber(system.mem_total_mb, 0)} MB
                </p>
              </div>
              <div>
                <h3>Disk</h3>
                <p className="stat">{formatPercent(system.disk_percent)}</p>
                <p className="hint">
                  {formatNumber(system.disk_used_gb, 1)} /{" "}
                  {formatNumber(system.disk_total_gb, 1)} GB
                </p>
              </div>
              <div>
                <h3>Network</h3>
                <p className="stat">
                  {formatNumber(network.latency_ms, 0)} ms
                </p>
                <p className="hint">
                  RX {formatNumber(network.rx_kbps, 0)} kbps · TX{" "}
                  {formatNumber(network.tx_kbps, 0)} kbps
                </p>
              </div>
            </div>
            <div className="heartbeat">
              <h3>ROS heartbeat</h3>
              {heartbeat.topics ? (
                <div className="mini-grid">
                  {Object.entries(heartbeat.topics).map(([topic, age]) => (
                    <div
                      key={topic}
                      className={
                        heartbeat.stale && heartbeat.stale.includes(topic)
                          ? "mini-card warn"
                          : "mini-card"
                      }
                    >
                      <span>{topic}</span>
                      <strong>{formatAge(age)}</strong>
                    </div>
                  ))}
                </div>
              ) : (
                <p className="hint">Heartbeat data not available.</p>
              )}
            </div>
          </section>

          <section className="card events-card">
            <div className="card-header">
              <h2>Event Timeline</h2>
            </div>
            <div className="events-list">
              {events.length === 0 ? (
                <p className="hint">No events yet.</p>
              ) : (
                events.slice().reverse().map((event) => (
                  <div key={event.id || `${event.ts}-${event.type}`} className="event-row">
                    <span className="event-time">
                      {new Date(event.ts * 1000).toLocaleTimeString()}
                    </span>
                    <span className={`event-type ${event.type || "info"}`}>
                      {event.type || "info"}
                    </span>
                    <span className="event-msg">{event.message}</span>
                  </div>
                ))
              )}
            </div>
          </section>
        </main>
      )}
    </div>
  );
}
