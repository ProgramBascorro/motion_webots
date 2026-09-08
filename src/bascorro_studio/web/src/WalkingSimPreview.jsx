// Walking simulation preview — a self-contained 3D viewer that mirrors OP3
// joint states. Single rosbridge, two subscriptions on the real domain:
//   /bascorro_studio/sim_preview/joint_states  — bridged from sim DOMAIN
//   /robotis/present_joint_states              — real robot
// The freshest publisher wins. Used on the Walking page so users can verify
// gait parameters in simulation before applying them to hardware.

import { useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { RotateCw, RotateCcw, Eye, EyeOff, RefreshCw, Wifi, WifiOff, Play, Square, Copy, Check } from "lucide-react";

const FRESH_WINDOW_MS = 1500;
const DEFAULT_ASSETS_URL = "http://localhost:8001";

function applyUpright(robot) {
  // OP3 URDF is Z-up; three.js camera looks down -Z. Rotate -90° around X
  // so the robot stands upright in the viewer with feet on the grid.
  robot.rotation.set(-Math.PI / 2, 0, 0);
}

export default function WalkingSimPreview({
  rosRef, rosState, isActive, onStartSimWalk, onStopSimWalk,
}) {
  const containerRef = useRef(null);
  const robotRef = useRef(null);
  const cameraRef = useRef(null);
  const controlsRef = useRef(null);
  const lastSimRxRef = useRef(0);
  const lastRealRxRef = useRef(0);
  const pendingPoseRef = useRef(null);
  const defaultCameraPosRef = useRef(new THREE.Vector3(0.55, 0.35, 0.65));
  const defaultTargetRef = useRef(new THREE.Vector3(0, 0, 0));

  const [viewerEnabled, setViewerEnabled] = useState(true);
  const [assetsUrl] = useState(() => localStorage.getItem("op3AssetsUrl") || DEFAULT_ASSETS_URL);
  const [robotLoaded, setRobotLoaded] = useState(false);
  const [loadError, setLoadError] = useState("");
  const [simRxCount, setSimRxCount] = useState(0);
  const [realRxCount, setRealRxCount] = useState(0);
  const [, setNowTick] = useState(0);

  // backendRunning mirrors the apply_node's _sim_proc handle; updates on
  // each sim_launch/sim_stop/sim_status response.
  const [backendRunning, setBackendRunning] = useState(false);
  const [launchStatus, setLaunchStatus] = useState("");
  const [launchBusy, setLaunchBusy] = useState(false);
  const [fallbackCmd, setFallbackCmd] = useState("");
  const [copyOk, setCopyOk] = useState(false);
  const [liveness, setLiveness] = useState([]); // per-child {label, pid, alive, log}
  const [logDir, setLogDir] = useState("");
  const resultSubRef = useRef(null);
  const requestPubRef = useRef(null);

  const rosConnected = rosState === "connected";

  // Tick the clock once a second so the "live/idle" status decays without
  // requiring a fresh joint state message. (Was 500ms; halved the cadence
  // because combined with other state updates it was sometimes pushing the
  // update queue past React's depth limit.)
  useEffect(() => {
    const id = window.setInterval(() => setNowTick(t => (t + 1) % 1e6), 1000);
    return () => window.clearInterval(id);
  }, []);

  // Studio backend request channel — same topic the ActionEditor uses for
  // bin operations. We add sim_launch / sim_stop / sim_status actions on
  // top of it so all studio→backend traffic flows through one socket.
  useEffect(() => {
    const ros = rosRef?.current;
    if (!ros || !rosConnected) return;
    requestPubRef.current = new ROSLIB.Topic({
      ros, name: "/bascorro_studio/request", messageType: "std_msgs/String",
    });
    resultSubRef.current = new ROSLIB.Topic({
      ros, name: "/bascorro_studio/result", messageType: "std_msgs/String",
    });
    resultSubRef.current.subscribe(msg => {
      try {
        const payload = JSON.parse(msg.data);
        const action = payload.action;
        if (action !== "sim_launch" && action !== "sim_stop" && action !== "sim_status") return;
        const running = Boolean(payload.running);
        // Only setState when the value actually changes. Auto-poll fires
        // every 3s; with naive setState every poll, React re-renders even
        // when nothing changed. That's normally fine but combined with the
        // 500ms tick interval below it stacks state churn that ultimately
        // shows up as "Maximum update depth exceeded" in StrictMode.
        setBackendRunning(prev => prev === running ? prev : running);
        setLaunchBusy(prev => prev ? false : prev);
        if (payload.message) {
          setLaunchStatus(prev => prev === payload.message ? prev : payload.message);
        }
        const fallback = payload.ok === false && payload.fallback_cmd ? payload.fallback_cmd : "";
        setFallbackCmd(prev => prev === fallback ? prev : fallback);
        if (Array.isArray(payload.liveness)) {
          // Shallow-compare on PID + alive so a stable backend response
          // doesn't trigger needless re-renders.
          setLiveness(prev => {
            if (prev.length !== payload.liveness.length) return payload.liveness;
            for (let i = 0; i < prev.length; i++) {
              const a = prev[i], b = payload.liveness[i];
              if (a.pid !== b.pid || a.alive !== b.alive || a.ros_domain_id !== b.ros_domain_id) {
                return payload.liveness;
              }
            }
            return prev;
          });
        }
        if (typeof payload.log_dir === "string") {
          setLogDir(prev => prev === payload.log_dir ? prev : payload.log_dir);
        }
      } catch { /* ignore non-JSON */ }
    });
    // Probe current sim state ONCE on mount so the button reflects
    // reality. (We removed the 3s auto-poll — combined with StrictMode's
    // double-effect-firing in dev it pushed state updates past React's
    // max-depth limit. The strip refreshes on each sim_launch/sim_stop
    // response anyway, which covers the common case.)
    window.setTimeout(() => {
      try {
        requestPubRef.current?.publish(new ROSLIB.Message({ data: JSON.stringify({ action: "sim_status" }) }));
      } catch { /* noop */ }
    }, 400);
    return () => {
      resultSubRef.current?.unsubscribe();
      resultSubRef.current = null;
      requestPubRef.current = null;
    };
  }, [rosRef, rosConnected]);

  const sendSimAction = (action) => {
    if (!requestPubRef.current) {
      setLaunchStatus("ROS belum terhubung");
      return;
    }
    setLaunchBusy(true);
    setLaunchStatus(action === "sim_launch" ? "Meluncurkan simulasi (manager_sim + bridge)…" : "Menghentikan simulasi…");
    setFallbackCmd("");
    try {
      requestPubRef.current.publish(new ROSLIB.Message({ data: JSON.stringify({ action }) }));
    } catch (err) {
      setLaunchBusy(false);
      setLaunchStatus(`Gagal kirim request: ${err?.message || err}`);
    }
  };

  const copyFallback = async () => {
    const cmd = fallbackCmd || "ros2 launch op3_manager op3_simulation.launch.py";
    try {
      await navigator.clipboard.writeText(cmd);
      setCopyOk(true);
      window.setTimeout(() => setCopyOk(false), 1800);
    } catch { /* clipboard blocked — user can copy manually */ }
  };

  // Single rosbridge, two subscriptions on the real domain:
  //   /bascorro_studio/sim_preview/joint_states  → sim DOMAIN (via bridge)
  //   /robotis/present_joint_states              → real robot
  // Freshest publisher wins. When the standalone sim is up, the preview
  // topic is the live one; when it's down, present_joint_states from the
  // real robot drives the viewer.
  useEffect(() => {
    const applyJointMap = (msg, sourceRef, setCount) => {
      sourceRef.current = Date.now();
      setCount(c => (c + 1) % 1e6);
      const map = {};
      const names = msg.name || [];
      const positions = msg.position || [];
      for (let i = 0; i < names.length; i++) {
        const value = Number(positions[i]);
        if (Number.isFinite(value)) map[names[i]] = value;
      }
      pendingPoseRef.current = map;
    };

    const ros = rosRef?.current;
    if (!rosConnected || !ros) return undefined;

    const subTargets = [];
    const simTopic = new ROSLIB.Topic({
      ros, name: "/bascorro_studio/sim_preview/joint_states",
      messageType: "sensor_msgs/JointState",
    });
    simTopic.subscribe(msg => applyJointMap(msg, lastSimRxRef, setSimRxCount));
    subTargets.push(simTopic);

    const realTopic = new ROSLIB.Topic({
      ros, name: "/robotis/present_joint_states",
      messageType: "sensor_msgs/JointState",
    });
    realTopic.subscribe(msg => applyJointMap(msg, lastRealRxRef, setRealRxCount));
    subTargets.push(realTopic);

    return () => { subTargets.forEach(t => t.unsubscribe()); };
  }, [rosRef, rosConnected]);

  // Build the three.js scene + URDF. Re-runs when the viewer becomes active
  // (tab switch) or the assets URL changes.
  useEffect(() => {
    if (!viewerEnabled || !containerRef.current || !isActive) return;
    const container = containerRef.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#f8fafc");

    const width = container.clientWidth || 480;
    const height = container.clientHeight || 320;
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

    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 0.55);
    dir.position.set(1.2, 1.5, 0.8);
    scene.add(dir);

    const grid = new THREE.GridHelper(2.4, 20, 0xd1d5db, 0xe2e8f0);
    grid.position.y = -0.32;
    scene.add(grid);

    setLoadError("");
    setRobotLoaded(false);
    const loader = new URDFLoader();
    loader.load(
      `${assetsUrl}/robotis_op3.urdf`,
      (robot) => {
        Object.values(robot.joints || {}).forEach(joint => { joint.ignoreLimits = true; });
        applyUpright(robot);
        robotRef.current = robot;
        scene.add(robot);
        setRobotLoaded(true);
      },
      undefined,
      (err) => { setLoadError(`Gagal load URDF: ${err?.message || err}`); },
    );

    let frameId;
    const animate = () => {
      frameId = requestAnimationFrame(animate);
      const pending = pendingPoseRef.current;
      if (pending && robotRef.current) {
        const robot = robotRef.current;
        Object.entries(pending).forEach(([name, value]) => {
          const joint = robot.joints?.[name];
          if (joint && typeof joint.setJointValue === "function") joint.setJointValue(value);
        });
      }
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      const w = container.clientWidth, h = container.clientHeight;
      if (!w || !h) return;
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
      if (container.contains(renderer.domElement)) container.removeChild(renderer.domElement);
      robotRef.current = null;
      setRobotLoaded(false);
    };
  }, [viewerEnabled, isActive, assetsUrl]);

  const sourceStatus = useMemo(() => {
    const now = Date.now();
    const simFresh = (now - lastSimRxRef.current) < FRESH_WINDOW_MS;
    const realFresh = (now - lastRealRxRef.current) < FRESH_WINDOW_MS;
    // sim_preview is the bridged topic — if it's fresh, the standalone sim
    // is the one driving the viewer. Otherwise fall back to the real robot.
    if (simFresh) return { kind: "sim", label: "SIM LIVE (domain 42)", color: "bg-emerald-500", text: "text-emerald-700", bg: "bg-emerald-50" };
    if (realFresh) return { kind: "real", label: "ROBOT LIVE (real)", color: "bg-blue-500", text: "text-blue-700", bg: "bg-blue-50" };
    return { kind: "idle", label: "Idle", color: "bg-gray-400", text: "text-gray-600", bg: "bg-gray-50" };
  }, [simRxCount, realRxCount]);

  const rotateView = (deltaDeg) => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;
    const rad = (deltaDeg * Math.PI) / 180;
    const offset = camera.position.clone().sub(controls.target);
    const spherical = new THREE.Spherical().setFromVector3(offset);
    spherical.theta += rad;
    offset.setFromSpherical(spherical);
    camera.position.copy(controls.target.clone().add(offset));
    controls.update();
  };

  const resetView = () => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;
    camera.position.copy(defaultCameraPosRef.current);
    controls.target.copy(defaultTargetRef.current);
    controls.update();
  };

  const isIdle = sourceStatus.kind === "idle";

  return (
    <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden flex flex-col">
      <div className="px-4 py-3 border-b border-gray-100 bg-gray-50/50 flex items-center justify-between gap-2 flex-wrap">
        <div className="flex items-center gap-2 min-w-0">
          <h3 className="text-sm font-bold text-gray-800">Simulasi 3D</h3>
          <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${sourceStatus.bg} ${sourceStatus.text} flex items-center gap-1`}>
            <span className={`inline-block w-1.5 h-1.5 rounded-full ${sourceStatus.color}`} />
            {sourceStatus.label}
          </span>
          {!rosConnected && (
            <span className="px-2 py-0.5 rounded-full text-[10px] font-bold uppercase bg-red-50 text-red-600 flex items-center gap-1">
              <WifiOff size={10}/> ROS Disconnected
            </span>
          )}
        </div>
        <div className="flex items-center gap-2 shrink-0">
          {/* Launch/Stop simulator buttons — primary action for users who
              haven't started the standalone sim yet. State priority:
              backendRunning > topic source so the button updates
              immediately on click. */}
          {!backendRunning ? (
            <button
              onClick={() => sendSimAction("sim_launch")}
              disabled={!rosConnected || launchBusy}
              className="inline-flex items-center gap-1.5 px-3 py-1.5 bg-green-600 hover:bg-green-700 text-white rounded-lg text-xs font-bold disabled:opacity-50"
              title="Jalankan manager_sim + bridge (butuh ~5 detik)"
            >
              <Play size={14}/> Mulai Simulasi
            </button>
          ) : (
            <button
              onClick={() => sendSimAction("sim_stop")}
              disabled={!rosConnected || launchBusy}
              className="inline-flex items-center gap-1.5 px-3 py-1.5 bg-white border border-red-300 text-red-600 hover:bg-red-50 rounded-lg text-xs font-bold disabled:opacity-50"
              title="Hentikan manager_sim + bridge"
            >
              <Square size={14}/> Stop Simulasi
            </button>
          )}
          {/* Dedicated walk Start/Stop — only active while the standalone sim
              is up. Routes through the same /robotis/walking_command path but
              UX-wise lives next to the sim viewer so users don't reach for
              "Apply & Start" (which also targets the real robot). */}
          {backendRunning && onStartSimWalk && (
            <button
              onClick={() => onStartSimWalk()}
              disabled={!rosConnected}
              className="inline-flex items-center gap-1.5 px-3 py-1.5 bg-emerald-600 hover:bg-emerald-700 text-white rounded-lg text-xs font-bold disabled:opacity-50"
              title="Apply parameter + start walking — hanya di simulator, robot asli tidak ikut bergerak"
            >
              <Play size={14}/> Sim Start
            </button>
          )}
          {backendRunning && onStopSimWalk && (
            <button
              onClick={() => onStopSimWalk()}
              disabled={!rosConnected}
              className="inline-flex items-center gap-1.5 px-3 py-1.5 bg-white border border-amber-300 text-amber-700 hover:bg-amber-50 rounded-lg text-xs font-bold disabled:opacity-50"
              title="Stop walking di simulator (tidak menyentuh robot asli)"
            >
              <Square size={14}/> Sim Stop
            </button>
          )}
          <div className="h-5 w-px bg-gray-300 mx-1" />
          <button onClick={() => rotateView(-30)} className="p-1.5 text-gray-500 hover:bg-white hover:text-undip-blue rounded" title="Putar kamera kiri"><RotateCcw size={14}/></button>
          <button onClick={() => rotateView(30)} className="p-1.5 text-gray-500 hover:bg-white hover:text-undip-blue rounded" title="Putar kamera kanan"><RotateCw size={14}/></button>
          <button onClick={resetView} className="p-1.5 text-gray-500 hover:bg-white hover:text-undip-blue rounded" title="Reset view"><RefreshCw size={14}/></button>
          <button onClick={() => setViewerEnabled(v => !v)} className="p-1.5 text-gray-500 hover:bg-white hover:text-undip-blue rounded" title={viewerEnabled ? "Sembunyikan viewer" : "Tampilkan viewer"}>
            {viewerEnabled ? <Eye size={14}/> : <EyeOff size={14}/>}
          </button>
        </div>
      </div>
      {launchStatus && (
        <div className={`px-4 py-1.5 text-[11px] font-mono border-b border-gray-100 ${fallbackCmd ? "bg-red-50 text-red-700" : "bg-blue-50 text-blue-700"}`}>
          {launchStatus}
        </div>
      )}
      {liveness.length > 0 && (
        <div className="px-4 py-1.5 text-[11px] font-mono border-b border-gray-100 bg-gray-50 text-gray-700 flex flex-wrap items-center gap-2">
          {liveness.map(p => {
            // A child whose actual ROS_DOMAIN_ID is anything other than 42
            // is leaking onto the real-robot domain even though Popen said
            // otherwise — surface that loudly so the user knows isolation
            // is the broken layer.
            const domainOk = !p.alive || p.ros_domain_id === "42";
            const tone = !p.alive
              ? "bg-red-100 text-red-800"
              : domainOk ? "bg-emerald-100 text-emerald-800" : "bg-amber-100 text-amber-800";
            const dot = !p.alive ? "bg-red-500" : domainOk ? "bg-emerald-500" : "bg-amber-500";
            return (
              <span key={p.label} className={`inline-flex items-center gap-1 px-1.5 py-0.5 rounded ${tone}`}>
                <span className={`inline-block w-1.5 h-1.5 rounded-full ${dot}`} />
                {p.label}={p.alive ? p.pid : "dead"}
                {p.alive && (
                  <span className="opacity-70">·DOMAIN={p.ros_domain_id || "?"}</span>
                )}
              </span>
            );
          })}
          {logDir && (
            <span className="ml-auto text-gray-500">
              tail -f {logDir}/&lt;label&gt;.log
            </span>
          )}
        </div>
      )}
      {fallbackCmd && (
        <div className="px-4 py-2 border-b border-gray-100 bg-amber-50 text-amber-900 text-[11px] leading-snug">
          <div className="font-bold mb-1">Auto-launch gagal. Jalankan manual di terminal:</div>
          <div className="flex items-center gap-2 bg-white border border-amber-200 rounded px-2 py-1.5 font-mono text-[11px]">
            <code className="flex-1 truncate">{fallbackCmd}</code>
            <button onClick={copyFallback} className="shrink-0 inline-flex items-center gap-1 px-2 py-0.5 bg-amber-100 hover:bg-amber-200 rounded text-[10px] font-bold">
              {copyOk ? <><Check size={10}/> Tersalin</> : <><Copy size={10}/> Copy</>}
            </button>
          </div>
        </div>
      )}
      <div className="relative flex-1 bg-slate-50 min-h-[320px]">
        {viewerEnabled ? (
          <div ref={containerRef} className="w-full h-full" />
        ) : (
          <div className="w-full h-full flex flex-col items-center justify-center text-gray-400">
            <EyeOff size={32} className="mb-2 opacity-20"/>
            <span className="text-xs">Viewer disembunyikan</span>
          </div>
        )}
        {viewerEnabled && !robotLoaded && !loadError && (
          <div className="absolute inset-0 flex items-center justify-center text-xs text-gray-400 pointer-events-none">
            Memuat URDF…
          </div>
        )}
        {loadError && (
          <div className="absolute top-2 left-2 right-2 bg-red-50 border border-red-200 text-red-700 text-[11px] font-mono rounded px-2 py-1.5">
            {loadError}
          </div>
        )}
        {viewerEnabled && robotLoaded && isIdle && !backendRunning && (
          <div className="absolute bottom-2 left-2 right-2 bg-white/95 backdrop-blur border border-gray-200 text-gray-700 text-[11px] rounded-lg px-3 py-2 leading-snug shadow-sm">
            <div className="flex items-center gap-1.5 mb-1 font-bold text-gray-800">
              <Wifi size={11}/> Belum ada data joint
            </div>
            <div className="text-gray-600">
              Klik <b>Mulai Simulasi</b> di atas untuk menjalankan manager_sim + bridge (tanpa robot fisik) — viewer akan render gerakan walking_module langsung di sini.
            </div>
          </div>
        )}
        {viewerEnabled && robotLoaded && isIdle && backendRunning && (
          <div className="absolute bottom-2 left-2 right-2 bg-blue-50/95 backdrop-blur border border-blue-200 text-blue-800 text-[11px] rounded-lg px-3 py-2 leading-snug shadow-sm">
            <div className="flex items-center gap-1.5 font-bold">
              <Wifi size={11}/> manager_sim booting…
            </div>
            <div className="text-blue-700/90 mt-0.5">Tunggu ~5 detik sampai manager_sim ready. Klik Sim Start untuk preview gerakan walking_module di sini (tanpa robot fisik).</div>
          </div>
        )}
        {/* When sim is up and joint states stream from the isolated bridge,
            confirm that Sim Start will only touch the virtual robot. */}
        {viewerEnabled && robotLoaded && sourceStatus.kind === "sim" && backendRunning && (
          <div className="absolute top-2 left-2 right-2 bg-emerald-50/95 backdrop-blur border border-emerald-200 text-emerald-800 text-[11px] rounded-lg px-3 py-1.5 leading-snug shadow-sm flex items-center gap-2">
            <Wifi size={11} className="shrink-0"/>
            <span><b>Sim bridge tersambung (domain 42)</b> — klik <b>Sim Start</b> untuk menggerakkan robot virtual. Robot asli pada domain default tidak ikut menerima command.</span>
          </div>
        )}
      </div>
    </div>
  );
}
