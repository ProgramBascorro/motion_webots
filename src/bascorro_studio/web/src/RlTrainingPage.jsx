import { useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { Brain, Dice5, Download, OctagonX, Play, RefreshCw, Save, Trash2, Trophy } from "lucide-react";
import { normalizeWalkingParams, parseWalkingNumber, toWalkingParamMsg } from "./walkingParams.js";
import { clamp, proposeCandidate } from "./training/learn.js";
import { recordImu } from "./training/imuRecorder.js";
import { turnSummary, turnText } from "./training/imuMetrics.js";
import { lockRobot, unlockRobot } from "./training/robotLock.js";

// RL Training di robot asli dengan penilaian operator.
//
// Tiap percobaan: ambil DATA AWAL dari Studio (bawaan: versi walking "geser
// kanan" terbaru; bisa juga tabel Walking yang terbuka atau versi lain), acak
// sebagian parameternya di dalam rentang sempit di sekitarnya, jalankan di robot, lalu operator menilai:
// kestabilan (normal / hampir jatuh / jatuh) dan gerakan yang benar-benar
// terjadi (jalan maju / mutar / geser ...). Skor = kestabilan + cocok-tidaknya
// gerakan dengan target. Mode "belajar" mengacak di sekitar percobaan terbaik
// sesi itu, dengan sebaran yang menyempit seiring bertambahnya percobaan.
//
// SENGAJA tidak meniru urutan demo D58 (init "mutar kanan" lalu preset dari
// walking_versions.json): operator minta data awalnya dari Studio. Stance
// percobaan = data awal itu sendiri dengan amplitudo gerak nol, jadi offset Init
// Pose tidak pernah berubah di tengah gait (perubahan offset saat berjalan
// dipakai seketika oleh walking module, tanpa ramp).
//
// Percobaan yang sudah dinilai disimpan di ROBOT lewat apply_node
// (rl_runs/studio/trials.json), bukan localStorage, dan TIDAK dijadikan versi
// walking -- operator: daftar Versi Tersimpan jadi menumpuk. Kalau datanya sudah
// banyak, operator minta dihitung terpisah.
//
// Diperbarui dari ALPHONSE_NEW_V3 (2026-11-04), disesuaikan untuk CHRONUS:
// - Mode belajar dipindah ke training/learn.js (rumusnya tidak berubah), dipakai
//   bersama tab Action Training.
// - IMU direkam tiap percobaan (kecepatan putar, goyang, miring): pembanding
//   objektif untuk "geser tapi ikut mutar".
// - Parameter dikirim lalu DIBACA BALIK (/robotis/walking/get_params) sebelum
//   "start"; tanpa itu percobaan bisa dinilai atas angka yang tidak sampai ke robot.
// - Bisa diacak (bawaan mati): hip pitch / pitch offset dan soft_start_time --
//   keluhan "tidak stabil di AWAL" geser kiri (2026-10-29) tidak terjangkau
//   7 parameter lama. Di CHRONUS offset menggeser stance relatif page 2 terhadap
//   acuan TETAP (REF_* op3_walking_module.h), jadi angka yang sama selalu memberi
//   stance yang sama; trik "patok acuan dulu" milik ALPHONSE tidak diperlukan.
// - Yang TIDAK diambil dari ALPHONSE: berhenti pakai "stop" (CHRONUS punya
//   soft_stop), urutan page 2 -> acuan -> modul milik App.jsx ALPHONSE, batas
//   z_move 0,070 (CHRONUS tetap 0,025, lihat di bawah), preset D60.

const PARAM_TYPE_DOUBLE = 3;
const WALKING_ENABLE_SETTLE_MS = 1200;   // sama dengan App.jsx
const STANCE_SETTLE_MS = 2500;           // ramp iniPoseTraGene >= 1,5 dtk + pembatas 25 deg/s
const READ_BACK_ATTEMPTS = 3;
const READ_BACK_MS = 150;
const LOCK_NAME = "RL Training";
const RESULT_TOPIC = "/bascorro_studio/result";
const REQUEST_TOPIC = "/bascorro_studio/request";

// lo/hi = batas KERAS. Nilai acak, rentang yang diketik operator, dan data awal
// semuanya dijepit ke sini sebelum dikirim ke robot.
//
// z_move dibatasi 0,025: 2026-10-27 servo 17 (ujung rantai kaki kanan) reset
// daya berulang kali tiap "geser kanan" dengan z_move 0,035 -- lutut diperintah
// melompat 20-30 deg per siklus 8 ms dan arusnya menjatuhkan tegangan. Versi
// lain yang tidak bermasalah semuanya <= 0,030, kebanyakan 0,025.
const RL_PARAMS = [
  { key: "x_move_amplitude", label: "X move (maju +, m)", spread: 0.002, lo: -0.040, hi: 0.040, step: 0.001 },
  { key: "y_move_amplitude", label: "Y move (kanan −, m)", spread: 0.003, lo: -0.040, hi: 0.040, step: 0.001 },
  { key: "angle_move_amplitude", label: "Angle move (rad)", spread: 0.01, lo: -0.35, hi: 0.35, step: 0.005 },
  // 0,55-0,75 atas permintaan operator 2026-10-27: >= 1 dtk terlalu lambat.
  { key: "period_time", label: "Period time (dtk)", spread: 0.02, lo: 0.55, hi: 0.75, step: 0.005 },
  { key: "dsp_ratio", label: "DSP ratio", spread: 0.03, lo: 0.05, hi: 0.60, step: 0.01 },
  { key: "z_move_amplitude", label: "Angkat kaki (m)", spread: 0.002, lo: 0.005, hi: 0.025, step: 0.001 },
  { key: "y_swap_amplitude", label: "Goyang kiri-kanan (m)", spread: 0.002, lo: 0.0, hi: 0.035, step: 0.001 },
  // Bawaan MATI (tidak masuk DEFAULT_ON). Batas keras mencakup semua versi walking
  // CHRONUS yang ada (hip pitch 0,06..0,16, pitch -0,25..-0,006), supaya data awal
  // tidak ikut terjepit; ALPHONSE memakai acuan page 2 +-0,06 / +-0,05, yang di sini
  // akan menjepit pitch -0,09 / -0,15 milik "geser kiri RL" / "geser kanan RL 2".
  // Dipasang saat robot berdiri diam (stance), tidak berubah di tengah gait.
  { key: "hip_pitch_offset", label: "Hip pitch offset (rad)", spread: 0.02, lo: 0.04, hi: 0.20, step: 0.005 },
  { key: "init_pitch_offset", label: "Pitch offset (rad)", spread: 0.02, lo: -0.26, hi: 0.05, step: 0.005 },
  // Parameter node /op3_walking_module, bukan field WalkingParam.
  { key: "soft_start_time", label: "Soft start (dtk)", spread: 0.15, lo: 0.0, hi: 2.0, step: 0.05 },
];
// spread = setengah lebar rentang bawaan di sekitar data awal. Sengaja sempit
// (tahap memperhalus): operator minta parameter tidak jauh dari data awal.
// Angle +-0,01 karena 38 percobaan sesi 27/10 menunjukkan 0,02 dan 0,05 jatuh
// 50% sementara 0,03-0,04 nyaris tidak pernah; rentang yang lebih lebar hanya
// membuang percobaan ke daerah yang sudah terbukti buruk.
//
// Data awal otomatis per target: nama pertama di daftarnya yang ada versinya
// (updatedAt terbaru kalau namanya kembar). "geser kanan RL" = hasil hitungan
// sesi 27/10. Nama mutar = preset D58 (preset_turn_left_/right_). Target tanpa
// daftar (jalan maju) memakai tabel Walking yang sedang terbuka -- jalan lurus
// bukan titik awal untuk menggeser, dan sebaliknya.
const DEFAULT_BASE_NAMES = {
  geser_kanan: ["geser kanan rl", "geser kanan"],
  geser_kiri: ["geser kiri rl", "geser kiri"],
  mutar_kanan: ["mutar kanan rl", "mutar kanan"],
  mutar_kiri: ["mutar kiri rl", "mutar kiri"],
};
const baseNamesOf = (target) => DEFAULT_BASE_NAMES[target] || [];
const DEFAULT_ON = new Set(["x_move_amplitude", "y_move_amplitude", "angle_move_amplitude", "period_time"]);

const STABILITY = [
  { id: "normal", label: "Normal", score: 2, cls: "bg-green-600" },
  { id: "hampir_jatuh", label: "Hampir jatuh", score: 0, cls: "bg-amber-500" },
  { id: "jatuh", label: "Jatuh", score: -4, cls: "bg-red-600" },
];
// Mutar dipecah menurut arah (seperti putar_kiri/kanan ALPHONSE) supaya cocok
// dengan arah putar IMU. "mutar" lama tetap dikenali untuk riwayat.
const MOVES = [
  { id: "maju", label: "Jalan maju" },
  { id: "mutar_kiri", label: "Mutar kiri" },
  { id: "mutar_kanan", label: "Mutar kanan" },
  { id: "geser_kanan", label: "Geser kanan" },
  { id: "geser_kiri", label: "Geser kiri" },
  { id: "lainnya", label: "Diam / tidak jelas" },
];
const TARGETS = MOVES.filter((m) => m.id !== "lainnya");
const MOVE_LABELS = [...MOVES, { id: "mutar", label: "Mutar (arah tak dicatat)" }];
// Kesesuaian dengan target dinilai terpisah dari "gerakan yang terjadi", supaya
// operator bisa bilang "mendekati" (mis. geser kanan tapi masih ikut maju).
// Otomatis terisi dari tombol gerakan; operator boleh menggantinya.
const MATCH = [
  { id: "sesuai", label: "Sesuai target", score: 3, cls: "bg-green-600" },
  { id: "mendekati", label: "Mendekati", score: 1, cls: "bg-amber-500" },
  { id: "tidak", label: "Tidak sesuai", score: -1, cls: "bg-red-600" },
];
const STABILITY_SCORE = Object.fromEntries(STABILITY.map((s) => [s.id, s.score]));
const MATCH_SCORE = Object.fromEntries(MATCH.map((m) => [m.id, m.score]));
// Percobaan lama (sebelum ada kolom kesesuaian) diturunkan dari gerakannya.
const matchOf = (t) => t.match || (t.move === t.target ? "sesuai" : "tidak");
// Skor dihitung ULANG dari penilaiannya, bukan dari t.score tersimpan, supaya
// percobaan lama dan baru sebanding (rumus lama: cocok +2 / tidak -1).
const scoreOf = (t) => (STABILITY_SCORE[t.stability] ?? 0) + (MATCH_SCORE[matchOf(t)] ?? 0);
const isGood = (t) => t.stability !== "jatuh" && (matchOf(t) === "sesuai" || matchOf(t) === "mendekati");
const isBad = (t) => t.stability === "jatuh" || matchOf(t) === "tidak";
const labelOf = (list, id) => list.find((x) => x.id === id)?.label || id || "-";

const BTN = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";
const CARD = "bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col overflow-hidden";
const CARD_HEAD = "px-5 py-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50";
const CARD_TITLE = "text-sm font-bold font-display text-gray-800 uppercase tracking-wide";

const fmt = (v, step) => (Number.isFinite(v) ? v.toFixed(step < 0.01 ? 4 : 3) : "-");
const makeId = () => `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;

// Pesan WalkingParam float32: 0,0063 kembali sebagai 0,006299999.
const sameValue = (runtime, expected) => {
  if (typeof expected === "boolean") return Boolean(runtime) === expected;
  const r = Number(runtime);
  const e = Number(expected);
  return Number.isFinite(r) && Math.abs(r - e) <= 1e-4 * Math.max(1, Math.abs(e));
};
const paramsLanded = (runtime, msg) => Object.keys(msg).every((key) => sameValue(runtime?.[key], msg[key]));

function defaultRanges(base) {
  const out = {};
  RL_PARAMS.forEach((p) => {
    const center = clamp(Number(base?.[p.key]) || 0, p.lo, p.hi);
    out[p.key] = {
      on: DEFAULT_ON.has(p.key),
      min: clamp(center - p.spread, p.lo, p.hi),
      max: clamp(center + p.spread, p.lo, p.hi),
    };
  });
  return out;
}

// Data awal dijepit ke batas keras; kembalikan juga daftar yang terjepit supaya
// operator tahu angka tabelnya tidak dipakai apa adanya.
function clampBase(params) {
  const next = { ...params };
  const clamped = [];
  RL_PARAMS.forEach((p) => {
    const v = Number(next[p.key]);
    const c = clamp(Number.isFinite(v) ? v : 0, p.lo, p.hi);
    if (c !== v) clamped.push(`${p.label}: ${fmt(v, p.step)} → ${fmt(c, p.step)}`);
    next[p.key] = c;
  });
  return { params: next, clamped };
}

export default function RlTrainingPage({
  ros,
  rosState,
  sendStatus,
  studioParams,
  studioVersionName,
  walkingVersions,
  onBeforeWalking,
}) {
  const connected = rosState === "connected" && !!ros;

  // --- data awal ---
  // "auto" = versi bawaan target (baseNamesOf) dengan updatedAt terbaru (nama
  // boleh kembar, aturan yang sama dengan demo D58); "table" = tabel Walking;
  // selain itu id versi.
  const [target, setTarget] = useState("geser_kanan");
  const [baseSource, setBaseSource] = useState("auto");
  const autoBase = useMemo(() => {
    for (const name of baseNamesOf(target)) {
      let best = null;
      (walkingVersions || []).forEach((v) => {
        if (String(v.name || "").trim().toLowerCase() !== name) return;
        if (!best || (v.updatedAt || 0) > (best.updatedAt || 0)) best = v;
      });
      if (best) return best;
    }
    return null;
  }, [walkingVersions, target]);
  const autoBaseId = autoBase?.id || "";
  const effectiveSource = baseSource === "auto" ? autoBaseId || "table" : baseSource;
  const baseInfo = useMemo(() => {
    if (effectiveSource === "table") {
      return {
        name: studioVersionName ? `Tabel Walking (${studioVersionName})` : "Tabel Walking Studio",
        params: normalizeWalkingParams(studioParams),
      };
    }
    const v = (walkingVersions || []).find((item) => item.id === effectiveSource);
    return v ? { name: v.name, params: normalizeWalkingParams(v.params) } : null;
  }, [effectiveSource, studioParams, studioVersionName, walkingVersions]);
  const base = useMemo(() => (baseInfo ? clampBase(baseInfo.params) : null), [baseInfo]);

  // --- pengaturan ---
  const [mode, setMode] = useState("learn");                 // "random" | "learn"
  const [durationText, setDurationText] = useState("4");
  const [session, setSession] = useState(() => `sesi ${new Date().toLocaleDateString("id-ID")}`);
  // Sampai operator memilih/membuat sesi sendiri, halaman mengikuti sesi
  // percobaan TERAKHIR di robot. Tanpa ini nama bawaan (tanggal hari ini)
  // berganti lewat tengah malam dan riwayat semalam tampak "hilang".
  const sessionTouchedRef = useRef(false);
  const [ranges, setRanges] = useState(() => defaultRanges(null));
  const rangesInitRef = useRef(false);
  // Data awal berganti -> rentang dihitung ulang di sekitar data awal yang baru.
  // Effect ini harus SEBELUM effect pengisi di bawah (urutan effect = urutan tulis).
  useEffect(() => {
    rangesInitRef.current = false;
  }, [effectiveSource]);
  useEffect(() => {
    if (rangesInitRef.current || !base) return;
    rangesInitRef.current = true;
    setRanges(defaultRanges(base.params));
  }, [base]);

  // --- percobaan ---
  const [phase, setPhase] = useState("idle");               // idle | siap | jalan | berhenti | nilai
  const [countdown, setCountdown] = useState(0);
  const [pending, setPending] = useState(null);
  const [stability, setStability] = useState("");
  const [move, setMove] = useState("");
  const [match, setMatch] = useState("");
  const [note, setNote] = useState("");
  const [trials, setTrials] = useState([]);
  const [storePath, setStorePath] = useState("");
  const abortRef = useRef(false);
  const busy = phase === "siap" || phase === "jalan" || phase === "berhenti";

  // --- ROS ---
  const rosRefs = useRef({});
  useEffect(() => {
    if (!connected) return undefined;
    const refs = {
      cmd: new ROSLIB.Topic({ ros, name: "/robotis/walking/command", messageType: "std_msgs/String" }),
      param: new ROSLIB.Topic({ ros, name: "/robotis/walking/set_params", messageType: "op3_walking_module_msgs/WalkingParam" }),
      module: new ROSLIB.Topic({ ros, name: "/robotis/enable_ctrl_module", messageType: "std_msgs/String" }),
      request: new ROSLIB.Topic({ ros, name: REQUEST_TOPIC, messageType: "std_msgs/String" }),
      result: new ROSLIB.Topic({ ros, name: RESULT_TOPIC, messageType: "std_msgs/String" }),
      jointModule: new ROSLIB.Service({ ros, name: "/robotis/get_present_joint_ctrl_modules", serviceType: "robotis_controller_msgs/srv/GetJointModule" }),
      nodeParam: new ROSLIB.Service({ ros, name: "/op3_walking_module/set_parameters", serviceType: "rcl_interfaces/srv/SetParameters" }),
      getParams: new ROSLIB.Service({ ros, name: "/robotis/walking/get_params", serviceType: "op3_walking_module_msgs/srv/GetWalkingParam" }),
    };
    rosRefs.current = refs;
    refs.result.subscribe((msg) => {
      let payload;
      try {
        payload = JSON.parse(msg.data);
      } catch {
        return;
      }
      if (typeof payload?.action !== "string" || !payload.action.startsWith("rl_")) return;
      if (payload.ok === false) {
        sendStatus(`RL: ${payload.message || "gagal"}`, true);
        return;
      }
      if (Array.isArray(payload.trials)) {
        setTrials(payload.trials);
        if (!sessionTouchedRef.current && payload.trials.length > 0) {
          const latest = payload.trials.reduce((a, b) => ((b.at || 0) > (a.at || 0) ? b : a));
          if (latest.session) setSession(latest.session);
        }
      }
      if (payload.path) setStorePath(payload.path);
      if (payload.action === "rl_add_trial" && payload.trial) {
        sendStatus(`RL: percobaan ke-${payload.trial.no} tersimpan`);
      }
    });
    const timer = window.setTimeout(() => requestStore({ action: "rl_get" }), 500);
    return () => {
      window.clearTimeout(timer);
      refs.result.unsubscribe();
      rosRefs.current = {};
    };
    // requestStore hanya memakai rosRefs.current; sengaja tidak masuk deps.
  }, [ros, connected]);

  const requestStore = (payload) => {
    const pub = rosRefs.current.request;
    if (!pub) {
      sendStatus("RL: robot belum tersambung", true);
      return;
    }
    pub.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
  };

  const walkingMsg = (params) => toWalkingParamMsg(normalizeWalkingParams(params));
  const publishParams = (params) => {
    rosRefs.current.param?.publish(new ROSLIB.Message(walkingMsg(params)));
  };
  const command = (text) => rosRefs.current.cmd?.publish(new ROSLIB.Message({ data: text }));

  const callService = (service, request) =>
    new Promise((resolve) => {
      if (!service) {
        resolve(null);
        return;
      }
      const timer = window.setTimeout(() => resolve(null), 2000);
      service.callService(
        new ROSLIB.ServiceRequest(request),
        (res) => { window.clearTimeout(timer); resolve(res); },
        () => { window.clearTimeout(timer); resolve(null); }
      );
    });

  // Tidur yang bisa dibatalkan tombol STOP.
  const wait = async (ms, onTick) => {
    const end = Date.now() + ms;
    while (Date.now() < end) {
      if (abortRef.current) return false;
      onTick?.(Math.ceil((end - Date.now()) / 1000));
      await new Promise((r) => window.setTimeout(r, Math.min(100, end - Date.now())));
    }
    return !abortRef.current;
  };

  const demoRunning = () =>
    new Promise((resolve) => {
      ros.getNodes(
        (nodes) => resolve((nodes || []).some((n) => /demo|rl_geser_kanan/.test(n))),
        () => resolve(false)
      );
    });

  // Aktifkan walking_module HANYA kalau belum aktif. Mengaktifkannya ulang
  // memicu penangkapan stance baru dari pose saat itu -- kalau robot sedang di
  // stance walking yang bergeser dari page 2, geseran itu ikut terkunci.
  const ensureWalkingModule = async () => {
    const res = await callService(rosRefs.current.jointModule, { joint_name: ["r_knee", "l_knee"] });
    if (res?.module_name?.length && res.module_name.every((m) => m === "walking_module")) return true;
    onBeforeWalking?.();
    rosRefs.current.module?.publish(new ROSLIB.Message({ data: "walking_module" }));
    return wait(WALKING_ENABLE_SETTLE_MS);
  };

  // Kirim lalu BACA BALIK sampai walking_module memegang angka yang sama. Hanya
  // saat diam: sesudah "stop" modul sendiri menolkan x/y/angle (processPhase).
  const applyParams = async (params) => {
    const msg = walkingMsg(params);
    for (let attempt = 0; attempt < READ_BACK_ATTEMPTS; attempt += 1) {
      rosRefs.current.param?.publish(new ROSLIB.Message(msg));
      if (!(await wait(READ_BACK_MS))) return false;
      const res = await callService(rosRefs.current.getParams, { get_param: true });
      if (paramsLanded(res?.parameters, msg)) return true;
    }
    if (!abortRef.current) {
      sendStatus("RL: walking_module tidak memegang parameter percobaan (dibaca balik dari /robotis/walking/get_params) — dibatalkan", true);
    }
    return false;
  };

  const applySoftStart = async (seconds) => {
    const res = await callService(rosRefs.current.nodeParam, {
      parameters: [{ name: "soft_start_time", value: { type: PARAM_TYPE_DOUBLE, double_value: Number(seconds) } }],
    });
    if (res?.results?.[0]?.successful) return true;
    if (!abortRef.current) sendStatus("RL: soft_start_time tidak diterima /op3_walking_module — dibatalkan", true);
    return false;
  };

  // --- kandidat parameter ---
  const sessionTrials = useMemo(
    () => trials.filter((t) => t.session === session).sort((a, b) => (b.no || 0) - (a.no || 0)),
    [trials, session]
  );
  // Riwayat yang boleh dipelajari: sesi ini, target ini, DAN data awal yang sama.
  // Parameter yang tidak diacak ikut data awal, jadi percobaan dari data awal lain
  // (mis. #1-5 sesi 27/10 yang memakai tabel Walking) tidak sebanding.
  const history = useMemo(
    () => sessionTrials.filter((t) => t.target === target && t.base_name === baseInfo?.name),
    [sessionTrials, target, baseInfo]
  );
  const best = useMemo(
    () => [...history].sort((a, b) => scoreOf(b) - scoreOf(a) || (b.no || 0) - (a.no || 0)).slice(0, 3),
    [history]
  );

  // Mode "belajar": training/learn.js (rumus asli tab ini, kini dipakai bersama
  // tab Action Training). Elit = SESUAI atau MENDEKATI target dan tidak jatuh;
  // "sesuai + normal" (5) menarik 4x lebih kuat dari "mendekati + normal" (3) dan
  // menyempitkan sebaran. Belum ada yang sesuai/mendekati -> acak penuh.
  const makeCandidate = () => proposeCandidate({
    params: RL_PARAMS.filter((p) => ranges[p.key]?.on).map((p) => {
      const r = ranges[p.key];
      return {
        key: p.key, step: p.step,
        lo: clamp(Math.min(r.min, r.max), p.lo, p.hi),
        hi: clamp(Math.max(r.min, r.max), p.lo, p.hi),
      };
    }),
    history,   // terurut terbaru dulu
    mode, scoreOf, isGood, isBad,
    isSolid: (t) => matchOf(t) === "sesuai" && t.stability === "normal",
    valueOf: (t, key) => Number(t.params?.[key] ?? base.params[key]),
    exploreBasis: "eksplorasi: belum ada yang sesuai/mendekati target",
  });

  // --- satu percobaan ---
  const runTrial = async () => {
    if (!connected || !base) return;
    const holder = lockRobot(LOCK_NAME);
    if (holder) {
      sendStatus(`RL: robot sedang dipakai tab ${holder}`, true);
      return;
    }
    try {
      await runTrialLocked();
    } finally {
      unlockRobot(LOCK_NAME);
    }
  };

  const runTrialLocked = async () => {
    if (await demoRunning()) {
      sendStatus("RL: node demo / rl_geser_kanan sedang jalan — matikan dulu, perintah walking akan saling timpa", true);
      return;
    }
    const duration = clamp(parseWalkingNumber(durationText, 4), 1, 20);
    const { varied, learn, basis } = makeCandidate();
    const full = { ...base.params, ...varied };
    const still = { ...full, x_move_amplitude: 0, y_move_amplitude: 0, angle_move_amplitude: 0 };
    abortRef.current = false;
    setPending({
      id: makeId(),
      session,
      target,
      base_name: baseInfo.name,
      mode: learn ? "belajar" : "acak",
      basis,
      duration,
      varied,
      params: full,
    });
    setStability("");
    setMove("");
    setMatch("");
    setNote("");

    setPhase("siap");
    setCountdown(0);
    let started = false;
    // IMU direkam sejak stance diam (jendela bias gyro 1 s sebelum start) sampai
    // sesudah berhenti; ringkasannya tampil di samping penilaian operator.
    let rec = null;
    const marks = {};
    let ok = await ensureWalkingModule();
    // Stance data awal dulu, selagi diam -- walking module hanya melandaikan
    // perubahan offset saat diam.
    if (ok) ok = await applyParams(still);
    if (ok) ok = await applySoftStart(full.soft_start_time);
    if (ok) {
      rec = recordImu(ros);
      ok = await wait(STANCE_SETTLE_MS, setCountdown);
    }
    if (ok) ok = await applyParams(full);
    if (ok) {
      command("start");
      marks.start = rec.last();
      started = true;
      setPhase("jalan");
      ok = await wait(duration * 1000, setCountdown);
    }
    if (ok) {
      command("soft_stop");
      marks.stop = rec.last();
      setPhase("berhenti");
      await wait((Number(full.soft_start_time) + Number(full.period_time) + 0.3) * 1000, setCountdown);
    }
    publishParams(still);
    rec?.stop();
    if (!started && !abortRef.current) {
      // Gagal sebelum "start" (pesannya sudah tampil): robot tidak berjalan,
      // jadi tidak ada yang bisa dinilai.
      setPending(null);
      setPhase("idle");
      return;
    }
    if (rec && started) {
      // null = IMU tidak terbit / rekaman jalan terlalu pendek (ditampilkan begitu).
      const imu = marks.start ? turnSummary(rec.samples, marks.start, marks.stop ?? rec.last(), rec.last()) : null;
      setPending((prev) => (prev ? { ...prev, imu } : prev));
    }
    setPhase("nilai");
  };

  const emergencyStop = () => {
    abortRef.current = true;
    command("stop");
    setNote((prev) => prev || "dihentikan operator (STOP)");
    sendStatus("RL: STOP — robot dihentikan", true);
  };

  const score = stability && move && match ? STABILITY_SCORE[stability] + MATCH_SCORE[match] : null;
  const chooseMove = (id) => {
    setMove(id);
    setMatch(id === pending?.target ? "sesuai" : "tidak");
  };

  const saveRating = () => {
    if (!pending || score === null) return;
    requestStore({
      action: "rl_add_trial",
      trial: { ...pending, stability, move, match, score, note: note.trim() },
    });
    setPending(null);
    setPhase("idle");
  };

  const discardTrial = () => {
    setPending(null);
    setPhase("idle");
    sendStatus("RL: percobaan dibuang (tidak dihitung)");
  };

  const exportCsv = () => {
    const keys = RL_PARAMS.map((p) => p.key);
    const rows = [["no", "waktu", "target", "data_awal", "mode", "durasi", ...keys, "kestabilan", "gerakan", "kesesuaian", "skor",
      "putar_dps", "putar_total_deg", "goyang_dps", "miring_maks_deg", "catatan"]];
    [...sessionTrials].reverse().forEach((t) => {
      rows.push([
        t.no, new Date(t.at).toISOString(), t.target, t.base_name, t.mode, t.duration,
        ...keys.map((k) => t.params?.[k]), t.stability, t.move, matchOf(t), scoreOf(t),
        t.imu?.turn_dps, t.imu?.turn_deg, t.imu?.wobble_dps, t.imu?.tilt_max_deg, t.note || "",
      ]);
    });
    const csv = rows.map((r) => r.map((c) => `"${String(c ?? "").replace(/"/g, '""')}"`).join(",")).join("\n");
    const url = URL.createObjectURL(new Blob([csv], { type: "text/csv" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = `rl_${session.replace(/[^\w-]+/g, "_")}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const setRange = (key, field, value) =>
    setRanges((prev) => ({ ...prev, [key]: { ...prev[key], [field]: value } }));

  const sessions = useMemo(() => {
    const set = new Set(trials.map((t) => t.session));
    set.add(session);
    return [...set];
  }, [trials, session]);
  const nextNo = 1 + Math.max(0, ...sessionTrials.map((t) => t.no || 0));

  // --- render ---
  return (
    <div className="h-full overflow-y-auto bg-gray-50/50 p-4 md:p-6">
      <div className="grid grid-cols-1 xl:grid-cols-12 gap-6">
        {/* Pengaturan */}
        <div className={`${CARD} xl:col-span-5`}>
          <div className={CARD_HEAD}>
            <div className="flex items-center gap-2">
              <Brain size={18} className="text-undip-blue" />
              <h2 className={CARD_TITLE}>RL Training — pengaturan</h2>
            </div>
          </div>
          <div className="p-5 space-y-4 text-sm">
            <label className="block">
              <span className="text-xs font-bold text-gray-600">Data awal (dari Studio)</span>
              <select
                className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm"
                value={baseSource}
                onChange={(e) => setBaseSource(e.target.value)}
                disabled={busy}
              >
                <option value="auto">
                  {autoBase
                    ? `Otomatis: versi "${autoBase.name}"`
                    : baseNamesOf(target).length
                      ? `Otomatis: ${baseNamesOf(target).at(-1)} (tidak ada → tabel Walking)`
                      : "Otomatis: tabel Walking (target ini tanpa versi bawaan)"}
                </option>
                <option value="table">Tabel Walking yang sedang terbuka</option>
                {(walkingVersions || []).map((v) => (
                  <option key={v.id} value={v.id}>Versi: {v.name}</option>
                ))}
              </select>
              {base?.clamped?.length > 0 && (
                <span className="block mt-1 text-[11px] text-amber-700">
                  Dijepit ke batas aman: {base.clamped.join("; ")}
                </span>
              )}
            </label>

            <div className="grid grid-cols-2 gap-3">
              <label className="block">
                <span className="text-xs font-bold text-gray-600">Target gerakan</span>
                <select className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm" value={target}
                  onChange={(e) => setTarget(e.target.value)} disabled={busy}>
                  {TARGETS.map((t) => <option key={t.id} value={t.id}>{t.label}</option>)}
                </select>
              </label>
              <label className="block">
                <span className="text-xs font-bold text-gray-600">Durasi jalan (dtk)</span>
                <input type="text" inputMode="decimal" className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm"
                  value={durationText} onChange={(e) => setDurationText(e.target.value)} disabled={busy} />
              </label>
              <label className="block">
                <span className="text-xs font-bold text-gray-600">Mode acak</span>
                <select className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm" value={mode}
                  onChange={(e) => setMode(e.target.value)} disabled={busy}>
                  <option value="learn">Belajar (acak di sekitar yang terbaik)</option>
                  <option value="random">Acak penuh dalam rentang</option>
                </select>
              </label>
              <label className="block">
                <span className="text-xs font-bold text-gray-600">Sesi</span>
                <div className="mt-1 flex gap-2">
                  <select className="flex-1 min-w-0 border border-gray-200 rounded-lg px-3 py-2 text-sm" value={session}
                    disabled={busy}
                    onChange={(e) => { sessionTouchedRef.current = true; setSession(e.target.value); }}>
                    {sessions.map((s) => (
                      <option key={s} value={s}>{s} ({trials.filter((t) => t.session === s).length})</option>
                    ))}
                  </select>
                  <button type="button" className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={busy}
                    onClick={() => {
                      const name = window.prompt("Nama sesi baru", `sesi ${new Date().toLocaleDateString("id-ID")}`);
                      if (name && name.trim()) { sessionTouchedRef.current = true; setSession(name.trim()); }
                    }}>
                    Baru
                  </button>
                </div>
              </label>
            </div>

            <div>
              <div className="flex justify-between items-center mb-2">
                <span className="text-xs font-bold text-gray-600">Parameter yang diacak</span>
                <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={busy || !base}
                  onClick={() => base && setRanges(defaultRanges(base.params))}>
                  <RefreshCw size={12} /> Rentang dari data awal
                </button>
              </div>
              <table className="w-full text-xs">
                <thead>
                  <tr className="text-gray-500">
                    <th className="text-left font-semibold py-1">Parameter</th>
                    <th className="text-right font-semibold">Awal</th>
                    <th className="text-right font-semibold">Min</th>
                    <th className="text-right font-semibold">Max</th>
                  </tr>
                </thead>
                <tbody>
                  {RL_PARAMS.map((p) => {
                    const r = ranges[p.key];
                    return (
                      <tr key={p.key} className="border-t border-gray-100">
                        <td className="py-1.5">
                          <label className="inline-flex items-center gap-2">
                            <input type="checkbox" checked={!!r?.on} disabled={busy}
                              onChange={(e) => setRange(p.key, "on", e.target.checked)} />
                            {p.label}
                          </label>
                        </td>
                        <td className="text-right font-mono text-gray-500">{fmt(base?.params[p.key], p.step)}</td>
                        {["min", "max"].map((f) => (
                          <td key={f} className="text-right pl-2">
                            <input type="text" inputMode="decimal" disabled={busy || !r?.on}
                              className="w-20 border border-gray-200 rounded px-1.5 py-1 text-right font-mono disabled:bg-gray-50"
                              defaultValue={fmt(r?.[f], p.step)}
                              key={`${p.key}-${f}-${r?.[f]}`}
                              onBlur={(e) => setRange(p.key, f, clamp(parseWalkingNumber(e.target.value, r[f]), p.lo, p.hi))} />
                          </td>
                        ))}
                      </tr>
                    );
                  })}
                </tbody>
              </table>
              <p className="mt-2 text-[11px] text-gray-500">
                Offset Init Pose selain hip pitch dan pitch, balance, dan field lain tidak diacak — dipakai apa
                adanya dari data awal. Hip pitch / pitch dipasang saat robot berdiri diam sebelum jalan; semua
                angka dibaca balik dari robot sebelum start. Angkat kaki dibatasi 0,025 m (servo 17 reset pada 0,035).
              </p>
            </div>
          </div>
        </div>

        {/* Percobaan + penilaian */}
        <div className={`${CARD} xl:col-span-7`}>
          <div className={CARD_HEAD}>
            <div className="flex items-center gap-2">
              <Dice5 size={18} className="text-undip-blue" />
              <h2 className={CARD_TITLE}>Percobaan ke-{nextNo}</h2>
            </div>
            <span className="text-xs text-gray-500">{sessionTrials.length} tersimpan di sesi ini</span>
          </div>
          <div className="p-5 space-y-4">
            <div className="flex flex-wrap gap-2 items-center">
              <button className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90 px-5 py-3 text-sm`}
                disabled={!connected || !base || busy || phase === "nilai"} onClick={runTrial}>
                <Play size={16} /> Play percobaan
              </button>
              <button className={`${BTN} bg-red-600 text-white hover:bg-red-700 px-5 py-3 text-sm`}
                disabled={!connected} onClick={emergencyStop}>
                <OctagonX size={16} /> STOP
              </button>
              <span className="text-sm text-gray-600">
                {!connected && "Robot belum tersambung"}
                {connected && phase === "idle" && "Siap. Robot sebaiknya digantung untuk percobaan pertama."}
                {phase === "siap" && `Menuju stance data awal… ${countdown}`}
                {phase === "jalan" && `Berjalan… ${countdown} dtk`}
                {phase === "berhenti" && `Soft stop… ${countdown}`}
                {phase === "nilai" && "Nilai percobaan ini:"}
              </span>
            </div>

            {pending && (
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs">
                <div className="mb-1 text-gray-500">
                  Data awal: <b className="text-gray-700">{pending.base_name}</b> · mode {pending.mode} · target{" "}
                  <b className="text-gray-700">{labelOf(TARGETS, pending.target)}</b>
                  {pending.basis && <> · {pending.basis}</>}
                </div>
                <div className="flex flex-wrap gap-x-4 gap-y-1 font-mono">
                  {Object.entries(pending.varied).map(([k, v]) => (
                    <span key={k}>{RL_PARAMS.find((p) => p.key === k)?.label}: <b>{v}</b></span>
                  ))}
                  {Object.keys(pending.varied).length === 0 && <span>(tidak ada yang diacak — data awal apa adanya)</span>}
                </div>
                {pending.imu !== undefined && (
                  <div className="mt-2 flex flex-wrap gap-x-4 gap-y-1 text-gray-600">
                    {pending.imu ? (
                      <>
                        <span>IMU putar <b className="text-gray-800">{turnText(pending.imu.turn_dps)}</b> (total {Math.abs(pending.imu.turn_deg).toFixed(0)}°)</span>
                        <span>goyang {pending.imu.wobble_dps} °/s</span>
                        {pending.imu.tilt_max_deg !== null && <span>miring maks {pending.imu.tilt_max_deg}°</span>}
                      </>
                    ) : <span>IMU: tidak ada rekaman / rekaman jalan terlalu pendek</span>}
                  </div>
                )}
              </div>
            )}

            {phase === "nilai" && pending && (
              <div className="space-y-3">
                <div>
                  <div className="text-xs font-bold text-gray-600 mb-1">Kestabilan</div>
                  <div className="flex flex-wrap gap-2">
                    {STABILITY.map((s) => (
                      <button key={s.id} onClick={() => setStability(s.id)}
                        className={`${BTN} px-4 py-2.5 text-sm ${stability === s.id ? `${s.cls} text-white` : "bg-gray-100 text-gray-700 hover:bg-gray-200"}`}>
                        {s.label}
                      </button>
                    ))}
                  </div>
                </div>
                <div>
                  <div className="text-xs font-bold text-gray-600 mb-1">Gerakan yang terjadi</div>
                  <div className="flex flex-wrap gap-2">
                    {MOVES.map((m) => (
                      <button key={m.id} onClick={() => chooseMove(m.id)}
                        className={`${BTN} px-4 py-2.5 text-sm ${move === m.id ? "bg-undip-blue text-white" : "bg-gray-100 text-gray-700 hover:bg-gray-200"}`}>
                        {m.label}
                      </button>
                    ))}
                  </div>
                </div>
                <div>
                  <div className="text-xs font-bold text-gray-600 mb-1">
                    Kesesuaian dengan target ({labelOf(TARGETS, pending.target)})
                    <span className="font-normal text-gray-400"> — "Mendekati" = arahnya benar tapi masih ikut maju/miring/sebagian</span>
                  </div>
                  <div className="flex flex-wrap gap-2">
                    {MATCH.map((m) => (
                      <button key={m.id} onClick={() => setMatch(m.id)} disabled={!move}
                        className={`${BTN} px-4 py-2.5 text-sm ${match === m.id ? `${m.cls} text-white` : "bg-gray-100 text-gray-700 hover:bg-gray-200"}`}>
                        {m.label}
                      </button>
                    ))}
                  </div>
                </div>
                <input className="w-full border border-gray-200 rounded-lg px-3 py-2 text-sm" placeholder="Catatan (opsional)"
                  value={note} onChange={(e) => setNote(e.target.value)} />
                <div className="flex flex-wrap items-center gap-2">
                  <button className={`${BTN} bg-green-600 text-white hover:bg-green-700 px-4 py-2.5 text-sm`}
                    disabled={score === null} onClick={saveRating}>
                    <Save size={14} /> Simpan penilaian
                  </button>
                  <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200 px-4 py-2.5 text-sm`} onClick={discardTrial}>
                    Buang (tidak dihitung)
                  </button>
                  {score !== null && <span className="text-sm text-gray-700">Skor: <b>{score}</b></span>}
                </div>
              </div>
            )}

            {best.length > 0 && (
              <div className="rounded-xl border border-amber-200 bg-amber-50 p-3 text-xs">
                <div className="flex items-center gap-1 font-bold text-amber-800 mb-1">
                  <Trophy size={14} /> Terbaik untuk target {labelOf(TARGETS, target)} (data awal ini)
                </div>
                {best.map((t) => (
                  <div key={t.id} className="flex flex-wrap items-center gap-x-3">
                    <span>#{t.no} skor <b>{scoreOf(t)}</b> ({labelOf(STABILITY, t.stability)}, {labelOf(MATCH, matchOf(t))}, {labelOf(MOVE_LABELS, t.move)})
                      {t.imu && <> · putar {turnText(t.imu.turn_dps)}</>}</span>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>

        {/* Riwayat */}
        <div className={`${CARD} xl:col-span-12`}>
          <div className={CARD_HEAD}>
            <h2 className={CARD_TITLE}>Riwayat — {session}</h2>
            <div className="flex gap-2">
              <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={!connected}
                onClick={() => requestStore({ action: "rl_get" })}>
                <RefreshCw size={12} /> Muat ulang
              </button>
              <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={sessionTrials.length === 0}
                onClick={exportCsv}>
                <Download size={12} /> CSV
              </button>
            </div>
          </div>
          <div className="p-5 overflow-x-auto">
            {sessionTrials.length === 0 ? (
              <p className="text-sm text-gray-500">Belum ada percobaan tersimpan di sesi ini.</p>
            ) : (
              <table className="w-full text-xs whitespace-nowrap">
                <thead>
                  <tr className="text-gray-500 text-left">
                    <th className="py-1 pr-3">#</th>
                    <th className="pr-3">Waktu</th>
                    <th className="pr-3">Target</th>
                    {RL_PARAMS.map((p) => <th key={p.key} className="pr-3 text-right">{p.label}</th>)}
                    <th className="pr-3">Kestabilan</th>
                    <th className="pr-3">Gerakan</th>
                    <th className="pr-3">Kesesuaian</th>
                    <th className="pr-3 text-right">Skor</th>
                    <th className="pr-3 text-right" title="Rata-rata kecepatan putar dari gyro z selama berjalan">Putar IMU</th>
                    <th className="pr-3 text-right" title="Goyang roll/pitch selama berjalan (RMS gyro)">Goyang °/s</th>
                    <th className="pr-3">Catatan</th>
                    <th />
                  </tr>
                </thead>
                <tbody>
                  {sessionTrials.map((t) => (
                    <tr key={t.id} className="border-t border-gray-100">
                      <td className="py-1.5 pr-3 font-bold">{t.no}</td>
                      <td className="pr-3 text-gray-500">{new Date(t.at).toLocaleTimeString("id-ID")}</td>
                      <td className="pr-3">{labelOf(MOVE_LABELS, t.target)}</td>
                      {RL_PARAMS.map((p) => (
                        <td key={p.key} className={`pr-3 text-right font-mono ${t.varied && p.key in t.varied ? "text-undip-blue font-bold" : "text-gray-400"}`}>
                          {fmt(Number(t.params?.[p.key]), p.step)}
                        </td>
                      ))}
                      <td className="pr-3">{labelOf(STABILITY, t.stability)}</td>
                      <td className="pr-3">{labelOf(MOVE_LABELS, t.move)}</td>
                      <td className="pr-3">{labelOf(MATCH, matchOf(t))}</td>
                      <td className="pr-3 text-right font-bold">{scoreOf(t)}</td>
                      <td className="pr-3 text-right font-mono">{turnText(t.imu?.turn_dps)}</td>
                      <td className="pr-3 text-right font-mono">{t.imu?.wobble_dps ?? "-"}</td>
                      <td className="pr-3 text-gray-500 max-w-[16rem] truncate">{t.note}</td>
                      <td className="py-1">
                        <button className="text-gray-400 hover:text-red-600" disabled={!connected}
                          onClick={() => window.confirm(`Hapus percobaan #${t.no}?`) && requestStore({ action: "rl_delete_trial", id: t.id })}
                          title="Hapus">
                          <Trash2 size={14} />
                        </button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
            {storePath && <p className="mt-3 text-[11px] text-gray-400">Disimpan di robot: {storePath}</p>}
          </div>
        </div>
      </div>
    </div>
  );
}
