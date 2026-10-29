import { useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { Brain, Dice5, Download, OctagonX, Play, RefreshCw, Save, Trash2, Trophy } from "lucide-react";
import {
  applyWalkingDraft, parseWalkingNumber, snapWalkingRuntime, validateWalkingParams, walkingPoseFarFromReference,
} from "./walkingParams.js";
import { WALKING_POSE_REFERENCE } from "./walkingPoseReference.js";

// RL Training di robot asli dengan penilaian operator (dipasang dari CHRONUS
// 2026-10-28).
//
// Tiap percobaan: ambil DATA AWAL dari Studio (bawaan: versi walking "JALAN
// BARU" terbaru; bisa juga tabel Walking yang terbuka atau versi lain), acak
// sebagian parameternya di dalam rentang sempit di sekitarnya, jalankan di robot, lalu operator menilai:
// kestabilan (normal / hampir jatuh / jatuh) dan gerakan yang benar-benar
// terjadi (jalan maju / mutar / geser ...). Skor = kestabilan + cocok-tidaknya
// gerakan dengan target. Mode "belajar" mengacak di sekitar percobaan terbaik
// sesi itu, dengan sebaran yang menyempit seiring bertambahnya percobaan.
//
// SENGAJA tidak meniru urutan demo: operator minta data awalnya dari Studio.
// Stance percobaan = data awal itu sendiri dengan amplitudo gerak nol, jadi
// offset Init Pose tidak pernah berubah di tengah gait (perubahan offset saat
// berjalan dipakai seketika oleh walking module, tanpa ramp).
//
// KHUSUS ALPHONSE:
// - walking_module dinyalakan lewat enableWalking() milik App.jsx, urutan yang
//   sama dengan Apply & Start (batas kecepatan servo dilepas, page 2 diputar,
//   acuan pose dipatok, baru modul). walking_bias_ direkam di tick pertama
//   sesudah modul menyala, jadi enable_ctrl_module polos membuat bentuk stance
//   bergantung pada pose apa pun yang kebetulan sedang dipegang.
// - Parameter dikirim lewat applyWalkingDraft(): dibaca balik dari
//   /robotis/walking/get_params sampai cocok. Tanpa itu percobaan bisa dinilai
//   atas angka yang tidak pernah sampai ke robot.
// - Berhenti = "stop". Walking module ini tidak mengenal "soft_stop"; "stop"
//   sudah melandai ke nol selama soft_start_time kalau nilainya > 0.
//
// Percobaan yang sudah dinilai disimpan di ROBOT lewat apply_node
// (rl_runs/studio/trials.json), bukan localStorage, dan TIDAK dijadikan versi
// walking -- operator CHRONUS: daftar Versi Tersimpan jadi menumpuk. Percobaan
// menu Training (data/autotune) dimasukkan lewat scripts/rl_impor_training.py.
// Kalau datanya sudah banyak, hitung lewat scripts/rl_hitung_template.py.

const STANCE_SETTLE_MS = 2500;           // offset diam dilandaikan 25 deg/s (idle_track_max_step_)
const RESULT_TOPIC = "/bascorro_studio/result";
const REQUEST_TOPIC = "/bascorro_studio/request";

// lo/hi = batas KERAS. Nilai acak, rentang yang diketik operator, dan data awal
// semuanya dijepit ke sini sebelum dikirim ke robot.
//
// z_move dibatasi 0,070 (keputusan operator 2026-10-28): sama dengan batas atas
// autotune ALPHONSE. JALAN BARU memakai 0,05; jb1 #50 dengan z 0,067 jatuh.
// (Batas CHRONUS 0,025 khusus servo 17 di sana, tidak berlaku di sini.)
const RL_PARAMS = [
  { key: "x_move_amplitude", label: "X move (maju +, m)", spread: 0.002, lo: -0.040, hi: 0.040, step: 0.001 },
  { key: "y_move_amplitude", label: "Y move (kanan −, m)", spread: 0.003, lo: -0.040, hi: 0.040, step: 0.001 },
  { key: "angle_move_amplitude", label: "Angle move (rad)", spread: 0.01, lo: -0.35, hi: 0.35, step: 0.005 },
  // 0,55-0,75 dari CHRONUS (operator di sana: >= 1 dtk terlalu lambat);
  // JALAN BARU 0,65 ada di dalamnya.
  { key: "period_time", label: "Period time (dtk)", spread: 0.02, lo: 0.55, hi: 0.75, step: 0.005 },
  { key: "dsp_ratio", label: "DSP ratio", spread: 0.03, lo: 0.05, hi: 0.60, step: 0.01 },
  { key: "z_move_amplitude", label: "Angkat kaki (m)", spread: 0.002, lo: 0.005, hi: 0.070, step: 0.001 },
  { key: "y_swap_amplitude", label: "Goyang kiri-kanan (m)", spread: 0.002, lo: 0.0, hi: 0.035, step: 0.001 },
];
// spread = setengah lebar rentang bawaan di sekitar data awal. Sengaja sempit
// (tahap memperhalus): operator minta parameter tidak jauh dari data awal.
// Angle +-0,01 dibawa dari CHRONUS (di sana 0,02 dan 0,05 jatuh 50%); autotune
// jb1 di robot ini menjelajah -0,016..+0,006 rad.
//
// Data awal otomatis: nama pertama di daftar ini yang ada versinya (updatedAt
// terbaru kalau namanya kembar). "JALAN BARU RL" = nama bawaan hasil
// rl_hitung_template.py --tulis; selama belum ada, JALAN BARU (template jalan
// maju pilihan operator 2026-10-28, juga data awal semua sesi menu Training).
const DEFAULT_BASE_NAMES = ["jalan baru rl", "jalan baru"];
const DEFAULT_ON = new Set(["x_move_amplitude", "y_move_amplitude", "angle_move_amplitude", "period_time"]);

const STABILITY = [
  { id: "normal", label: "Normal", score: 2, cls: "bg-green-600" },
  { id: "hampir_jatuh", label: "Hampir jatuh", score: 0, cls: "bg-amber-500" },
  { id: "jatuh", label: "Jatuh", score: -4, cls: "bg-red-600" },
];
const MOVES = [
  { id: "maju", label: "Jalan maju" },
  { id: "mutar", label: "Mutar" },
  { id: "geser_kanan", label: "Geser kanan" },
  { id: "geser_kiri", label: "Geser kiri" },
  { id: "lainnya", label: "Diam / tidak jelas" },
];
const TARGETS = MOVES.filter((m) => m.id !== "lainnya");
// Kesesuaian dengan target dinilai terpisah dari "gerakan yang terjadi", supaya
// operator bisa bilang "mendekati" (mis. maju tapi masih berbelok).
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

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const fmt = (v, step) => (Number.isFinite(v) ? v.toFixed(step < 0.01 ? 4 : 3) : "-");
const makeId = () => `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
// Box-Muller: sampel normal baku.
const randn = () => {
  let u = 0;
  while (u === 0) u = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
};

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
  normalizeParams,
  enableWalking,
  cancelWalking,
}) {
  const connected = rosState === "connected" && !!ros;

  // --- data awal ---
  // "auto" = versi "JALAN BARU" dengan updatedAt terbaru (nama boleh kembar);
  // "table" = tabel Walking; selain itu id versi.
  const [baseSource, setBaseSource] = useState("auto");
  const autoBase = useMemo(() => {
    for (const name of DEFAULT_BASE_NAMES) {
      let best = null;
      (walkingVersions || []).forEach((v) => {
        if (String(v.name || "").trim().toLowerCase() !== name) return;
        if (!best || (v.updatedAt || 0) > (best.updatedAt || 0)) best = v;
      });
      if (best) return best;
    }
    return null;
  }, [walkingVersions]);
  const autoBaseId = autoBase?.id || "";
  const effectiveSource = baseSource === "auto" ? autoBaseId || "table" : baseSource;
  const baseInfo = useMemo(() => {
    if (effectiveSource === "table") {
      return {
        name: studioVersionName ? `Tabel Walking (${studioVersionName})` : "Tabel Walking Studio",
        params: normalizeParams(studioParams),
      };
    }
    const v = (walkingVersions || []).find((item) => item.id === effectiveSource);
    return v ? { name: v.name, params: normalizeParams(v.params) } : null;
  }, [effectiveSource, studioParams, studioVersionName, walkingVersions, normalizeParams]);
  const base = useMemo(() => (baseInfo ? clampBase(baseInfo.params) : null), [baseInfo]);
  // Offset Init Pose tidak diacak, tapi data awal yang jauh dari page 2 membuat
  // tiap percobaan condong -- Apply & Start menanyakannya, di sini diperingatkan.
  const farFromPage2 = useMemo(
    () => (base ? walkingPoseFarFromReference(base.params, WALKING_POSE_REFERENCE.offsets) : []),
    [base]
  );

  // --- pengaturan ---
  const [target, setTarget] = useState("maju");
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
      request: new ROSLIB.Topic({ ros, name: REQUEST_TOPIC, messageType: "std_msgs/String" }),
      result: new ROSLIB.Topic({ ros, name: RESULT_TOPIC, messageType: "std_msgs/String" }),
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

  // Kirim lalu baca balik sampai walking_module memegang angka yang sama.
  const applyParams = async (params) => {
    try {
      await applyWalkingDraft({
        draft: params,
        baseline: params,
        read: async () => {
          const res = await callService(rosRefs.current.getParams, { get_param: true });
          if (!res?.parameters) throw new Error("/robotis/walking/get_params tidak menjawab");
          return snapWalkingRuntime(validateWalkingParams(res.parameters));
        },
        publish: (payload) => rosRefs.current.param?.publish(new ROSLIB.Message(payload)),
        check: () => {
          if (abortRef.current) throw new Error("dihentikan operator");
        },
        onRead: () => {},
        wait,
      });
      return true;
    } catch (error) {
      if (!abortRef.current) sendStatus(`RL: ${error.message}`, true);
      return false;
    }
  };

  // Tanpa baca balik: dipakai sesudah STOP, saat check() di atas pasti menolak.
  const publishParams = (params) => {
    try {
      rosRefs.current.param?.publish(new ROSLIB.Message(validateWalkingParams(params)));
    } catch (error) {
      sendStatus(`RL: ${error.message}`, true);
    }
  };

  // op_demo_node dan pelatih menu Training (walking_autotune / action_autotune)
  // sama-sama menulis perintah walking.
  const demoRunning = () =>
    new Promise((resolve) => {
      ros.getNodes(
        (nodes) => resolve((nodes || []).some((n) => /demo|autotune/.test(n))),
        () => resolve(false)
      );
    });

  // --- kandidat parameter ---
  const sessionTrials = useMemo(
    () => trials.filter((t) => t.session === session).sort((a, b) => (b.no || 0) - (a.no || 0)),
    [trials, session]
  );
  // Riwayat yang boleh dipelajari: sesi ini, target ini, DAN data awal yang sama.
  // Parameter yang tidak diacak ikut data awal, jadi percobaan dari data awal lain
  // (mis. yang memakai tabel Walking) tidak sebanding.
  const history = useMemo(
    () => sessionTrials.filter((t) => t.target === target && t.base_name === baseInfo?.name),
    [sessionTrials, target, baseInfo]
  );
  const best = useMemo(
    () => [...history].sort((a, b) => scoreOf(b) - scoreOf(a) || (b.no || 0) - (a.no || 0)).slice(0, 3),
    [history]
  );

  // Mode "belajar" (cross-entropy sederhana, dipandu penilaian operator):
  //  1. elit = percobaan yang SESUAI atau MENDEKATI target dan tidak jatuh,
  //     diambil separuh teratas menurut skor (maks 5);
  //  2. pusat = rata-rata elit berbobot 2^(skor - skor_tertinggi): "sesuai +
  //     normal" (5) menarik 4x lebih kuat dari "mendekati + normal" (3);
  //  3. sebaran mulai 1/4 rentang dan MENYEMPIT tiap kali ada "sesuai + normal"
  //     (memperhalus di sekitar yang sudah benar), tidak lebih kecil dari 1/10
  //     rentang -- elit yang kembar membuat sebarannya nol, dan tanpa lantai ini
  //     robot cuma mengulang angka yang sama tanpa peluang membaik;
  //     kalau elit >= 3, ikut dicampur dengan sebaran elit itu sendiri;
  //  4. tiga percobaan terakhir gagal semua (jatuh / tidak sesuai) -> sebaran
  //     x1,5 supaya tidak terjebak di sekitar titik yang ternyata buruk.
  // Belum ada satu pun yang sesuai/mendekati -> eksplorasi acak penuh dalam rentang.
  const makeCandidate = () => {
    const varied = {};
    const active = RL_PARAMS.filter((p) => ranges[p.key]?.on);
    const good = history.filter(isGood).sort((a, b) => scoreOf(b) - scoreOf(a) || (b.no || 0) - (a.no || 0));
    const learn = mode === "learn" && good.length > 0;
    let elite = [];
    let weights = [];
    let shrink = 1;
    let widen = 1;
    if (learn) {
      elite = good.slice(0, Math.min(5, Math.max(1, Math.ceil(good.length / 2))));
      const top = scoreOf(elite[0]);
      weights = elite.map((t) => 2 ** (scoreOf(t) - top));
      const solid = history.filter((t) => matchOf(t) === "sesuai" && t.stability === "normal").length;
      shrink = Math.max(0.2, 1 / (1 + 0.5 * solid));
      const last3 = history.slice(0, 3);   // history terurut terbaru dulu
      if (last3.length === 3 && last3.every(isBad)) widen = 1.5;
    }
    const wSum = weights.reduce((s, x) => s + x, 0);
    active.forEach((p) => {
      const r = ranges[p.key];
      const lo = clamp(Math.min(r.min, r.max), p.lo, p.hi);
      const hi = clamp(Math.max(r.min, r.max), p.lo, p.hi);
      const span = hi - lo;
      let v;
      if (learn) {
        const vals = elite.map((t) => Number(t.params?.[p.key] ?? base.params[p.key]));
        const center = vals.reduce((s, x, i) => s + weights[i] * x, 0) / wSum;
        let sigma = (span / 4) * shrink;
        if (elite.length >= 3) {
          const sd = Math.sqrt(vals.reduce((s, x, i) => s + weights[i] * (x - center) ** 2, 0) / wSum);
          sigma = 0.5 * sigma + 0.5 * sd;
        }
        sigma = clamp(sigma * widen, span / 10, span / 2);
        v = center + randn() * sigma;
      } else {
        v = lo + Math.random() * span;
      }
      v = clamp(v, lo, hi);
      varied[p.key] = Number((Math.round(v / p.step) * p.step).toFixed(4));
    });
    let basis = "acak penuh dalam rentang";
    if (learn) {
      basis = `pusat dari ${elite.map((t) => `#${t.no}`).join(", ")} · sebaran ${Math.round(shrink * widen * 100)}%`
        + (widen > 1 ? " (dilebarkan: 3 terakhir gagal)" : "");
    } else if (mode === "learn") {
      basis = "eksplorasi: belum ada yang sesuai/mendekati target";
    }
    return { varied, learn, basis };
  };

  // --- satu percobaan ---
  const runTrial = async () => {
    if (!connected || !base) return;
    if (await demoRunning()) {
      sendStatus("RL: node demo / autotune sedang jalan — matikan dulu, perintah walking akan saling timpa", true);
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
    // Hanya menyalakan modul kalau kaki belum dipegang walking_module (lihat
    // initThenEnableWalking di App.jsx). Menyalakannya ulang merekam bias baru.
    let ok = (await enableWalking(still)) && !abortRef.current;
    if (ok) {
      // Stance data awal dulu, selagi diam -- walking module hanya melandaikan
      // perubahan offset saat diam.
      ok = await applyParams(still);
      ok = ok && (await wait(STANCE_SETTLE_MS, setCountdown));
    }
    if (ok) ok = await applyParams(full);
    if (ok) {
      command("start");
      started = true;
      setPhase("jalan");
      ok = await wait(duration * 1000, setCountdown);
    }
    if (ok) {
      command("stop");
      setPhase("berhenti");
      await wait((Number(full.soft_start_time) + Number(full.period_time) + 0.3) * 1000, setCountdown);
    }
    publishParams(still);
    if (!started && !abortRef.current) {
      // Gagal sebelum "start" (pesannya sudah tampil): robot tidak berjalan,
      // jadi tidak ada yang bisa dinilai.
      setPending(null);
      setPhase("idle");
      return;
    }
    setPhase("nilai");
  };

  const emergencyStop = () => {
    abortRef.current = true;
    cancelWalking?.();
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
    const rows = [["no", "waktu", "target", "data_awal", "mode", "durasi", ...keys, "kestabilan", "gerakan", "kesesuaian", "skor", "catatan"]];
    [...sessionTrials].reverse().forEach((t) => {
      rows.push([
        t.no, new Date(t.at).toISOString(), t.target, t.base_name, t.mode, t.duration,
        ...keys.map((k) => t.params?.[k]), t.stability, t.move, matchOf(t), scoreOf(t), t.note || "",
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
                  {autoBase ? `Otomatis: versi "${autoBase.name}"` : "Otomatis: JALAN BARU (tidak ada → tabel Walking)"}
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
              {farFromPage2.length > 0 && (
                <span className="block mt-1 text-[11px] text-amber-700">
                  Init pose jauh dari page 2 ({farFromPage2.map(({ key, selisih }) => `${key} ${selisih > 0 ? "+" : ""}${Number(selisih.toPrecision(3))}`).join(", ")}) — robot bisa condong. Pegang robot.
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
                Offset Init Pose, balance, dan field lain tidak diacak — dipakai apa adanya dari data awal.
                Angkat kaki dibatasi 0,070 m (batas atas autotune ALPHONSE).
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
                    <span className="font-normal text-gray-400"> — "Mendekati" = arahnya benar tapi masih berbelok/ikut maju/miring/sebagian</span>
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
                    <span>#{t.no} skor <b>{scoreOf(t)}</b> ({labelOf(STABILITY, t.stability)}, {labelOf(MATCH, matchOf(t))}, {labelOf(MOVES, t.move)})</span>
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
                    <th className="pr-3">Catatan</th>
                    <th />
                  </tr>
                </thead>
                <tbody>
                  {sessionTrials.map((t) => (
                    <tr key={t.id} className="border-t border-gray-100">
                      <td className="py-1.5 pr-3 font-bold">{t.no}</td>
                      <td className="pr-3 text-gray-500">{new Date(t.at).toLocaleTimeString("id-ID")}</td>
                      <td className="pr-3">{labelOf(TARGETS, t.target)}</td>
                      {RL_PARAMS.map((p) => (
                        <td key={p.key} className={`pr-3 text-right font-mono ${t.varied && p.key in t.varied ? "text-undip-blue font-bold" : "text-gray-400"}`}>
                          {fmt(Number(t.params?.[p.key]), p.step)}
                        </td>
                      ))}
                      <td className="pr-3">{labelOf(STABILITY, t.stability)}</td>
                      <td className="pr-3">{labelOf(MOVES, t.move)}</td>
                      <td className="pr-3">{labelOf(MATCH, matchOf(t))}</td>
                      <td className="pr-3 text-right font-bold">{scoreOf(t)}</td>
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
