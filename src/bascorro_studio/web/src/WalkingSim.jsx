import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Box, RotateCcw, Pause, Play } from "lucide-react";

// Live 3D mirror of the robot for the Walking page. Self-contained: it opens its
// own rosbridge connection, subscribes to the joint states the robot/sim publish,
// and drives the URDF every frame so the model moves exactly as the robot moves.
const DEFAULT_ASSETS = import.meta.env.VITE_ASSETS_URL || "http://localhost:8001";
const HARDWARE_TOPIC = "/robotis/present_joint_states";
const SIM_TOPIC = "/robotis_op3/joint_states";
const SOURCE_TIMEOUT_MS = 1500; // hardware wins over sim while it keeps publishing

export default function WalkingSim({ rosUrl = "", active = true }) {
  const mountRef = useRef(null);
  const robotRef = useRef(null);
  const cameraRef = useRef(null);
  const controlsRef = useRef(null);
  const livePoseRef = useRef({});
  const holdRef = useRef(false);
  const lastSourceRef = useRef({ hardware: 0, sim: 0 });
  const lastUpdateRef = useRef(0);
  const defaultCamPos = useRef(new THREE.Vector3(0.6, 0.35, 1.4));
  const defaultTarget = useRef(new THREE.Vector3(0, 0, 0));

  const [rosState, setRosState] = useState("disconnected");
  const [hold, setHold] = useState(false);
  const [liveAge, setLiveAge] = useState(Infinity);

  const assetsUrl =
    (typeof localStorage !== "undefined" && localStorage.getItem("op3AssetsUrl")) || DEFAULT_ASSETS;

  useEffect(() => {
    holdRef.current = hold;
  }, [hold]);

  // --- ROS joint subscription ---
  useEffect(() => {
    if (!rosUrl || !active) return;
    const ros = new ROSLIB.Ros({ url: rosUrl });
    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    const applyJointState = (msg, source) => {
      if (holdRef.current) return;
      const now = Date.now();
      // sim only fills in while hardware is silent, so we never fight two sources
      if (source === "sim" && now - lastSourceRef.current.hardware <= SOURCE_TIMEOUT_MS) return;
      lastSourceRef.current[source] = now;
      const next = { ...livePoseRef.current };
      (msg.name || []).forEach((name, i) => {
        const v = msg.position?.[i];
        if (typeof v === "number" && Number.isFinite(v)) next[name] = v;
      });
      livePoseRef.current = next;
      lastUpdateRef.current = now;
    };

    const hw = new ROSLIB.Topic({ ros, name: HARDWARE_TOPIC, messageType: "sensor_msgs/JointState" });
    hw.subscribe((m) => applyJointState(m, "hardware"));
    const sim = new ROSLIB.Topic({ ros, name: SIM_TOPIC, messageType: "sensor_msgs/JointState" });
    sim.subscribe((m) => applyJointState(m, "sim"));

    return () => {
      hw.unsubscribe();
      sim.unsubscribe();
      ros.close();
    };
  }, [rosUrl, active]);

  // --- Three.js scene ---
  useEffect(() => {
    if (!active || !mountRef.current) return;
    const container = mountRef.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#f8fafc");

    const width = container.clientWidth || 600;
    const height = container.clientHeight || 360;
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 20);
    camera.position.copy(defaultCamPos.current);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.copy(defaultTarget.current);
    controls.update();
    controlsRef.current = controls;

    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(1.2, 1.5, 0.8);
    scene.add(dir);
    const grid = new THREE.GridHelper(2.4, 20, 0xd1d5db, 0xe2e8f0);
    grid.position.y = -0.32;
    scene.add(grid);

    let disposed = false;
    const loader = new URDFLoader();
    loader.load(`${assetsUrl}/robotis_op3.urdf`, (robot) => {
      if (disposed) return;
      robot.rotation.set(-Math.PI / 2, 0, 0); // stand upright
      robot.traverse((child) => {
        if (child.isMesh && child.material) {
          const mats = Array.isArray(child.material) ? child.material : [child.material];
          mats.forEach((m) => {
            m.side = THREE.DoubleSide;
            m.needsUpdate = true;
          });
        }
      });
      robotRef.current = robot;
      scene.add(robot);
    });

    let frameId;
    const animate = () => {
      frameId = requestAnimationFrame(animate);
      const robot = robotRef.current;
      const pose = livePoseRef.current;
      if (robot && pose) {
        for (const name in pose) {
          const joint = robot.joints?.[name];
          const value = pose[name];
          if (joint && Number.isFinite(value)) joint.setJointValue(value);
        }
      }
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const onResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (!w || !h) return;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener("resize", onResize);

    return () => {
      disposed = true;
      window.removeEventListener("resize", onResize);
      cancelAnimationFrame(frameId);
      controls.dispose();
      renderer.dispose();
      if (container.contains(renderer.domElement)) container.removeChild(renderer.domElement);
      robotRef.current = null;
    };
  }, [active, assetsUrl]);

  // ticker for the "live" badge (ms since last joint update)
  useEffect(() => {
    if (!active) return;
    const t = setInterval(() => setLiveAge(Date.now() - (lastUpdateRef.current || 0)), 400);
    return () => clearInterval(t);
  }, [active]);

  const resetView = () => {
    const cam = cameraRef.current;
    const ctr = controlsRef.current;
    if (!cam || !ctr) return;
    cam.position.copy(defaultCamPos.current);
    ctr.target.copy(defaultTarget.current);
    cam.lookAt(defaultTarget.current);
    ctr.update();
  };

  const live = liveAge < SOURCE_TIMEOUT_MS;
  const statusLabel = live
    ? "Robot Live Model"
    : rosState === "connected"
      ? "Menunggu data joint"
      : "Tidak terhubung";

  return (
    <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
      <div className="p-4 border-b border-gray-100 flex items-center justify-between gap-3 flex-wrap">
        <div className="flex items-center gap-2 min-w-0">
          <Box size={18} className="text-undip-blue" />
          <h3 className="font-bold text-gray-900">Simulasi 3D</h3>
          <span
            className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${
              live ? "bg-green-50 text-green-700" : "bg-gray-100 text-gray-500"
            }`}
          >
            {statusLabel}
          </span>
        </div>
        <div className="flex items-center gap-2 shrink-0">
          <button
            onClick={resetView}
            title="Reset tampilan kamera"
            className="p-1.5 rounded-lg border border-gray-200 text-gray-600 hover:bg-gray-50"
          >
            <RotateCcw size={14} />
          </button>
          <button
            onClick={() => setHold((h) => !h)}
            title={hold ? "Lanjutkan mengikuti robot" : "Bekukan simulasi (berhenti mengikuti robot)"}
            className={`inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold transition-colors ${
              hold ? "bg-amber-500 text-white hover:bg-amber-600" : "bg-green-600 text-white hover:bg-green-700"
            }`}
          >
            {hold ? (
              <>
                <Play size={14} /> Lanjut
              </>
            ) : (
              <>
                <Pause size={14} /> Hold Simulasi
              </>
            )}
          </button>
        </div>
      </div>
      <div ref={mountRef} className="w-full h-[360px] bg-slate-50" />
    </div>
  );
}
