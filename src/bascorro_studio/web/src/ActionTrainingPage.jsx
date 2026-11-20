import { useEffect, useMemo, useRef, useState } from "react";
import { Dice5, Eraser, Footprints, OctagonX, Play, Plus, RefreshCw, Save, Trash2, Trophy } from "lucide-react";
import { clamp, proposeCandidate } from "./training/learn.js";
import { recordImu } from "./training/imuRecorder.js";
import { SCRATCH_PAGE, TrialAborted, runActionTrial } from "./training/actionTrial.js";
import { createActionRobot } from "./training/actionRobot.js";
import { createStudioClient } from "./training/studioClient.js";
import { lockRobot, unlockRobot } from "./training/robotLock.js";

// Action Training: melatih NILAI SERVO page Action Editor di robot asli dengan
// penilaian operator. Diadaptasi dari tab yang sama di ALPHONSE_NEW_V3.
//
// Variabel = perubahan derajat (arah raw, angka yang sama dengan Action Editor)
// pada satu sendi di rentang step, atau pengali waktu step. Usulannya disusun
// apply_node dari data servo page itu (bascorro_studio/training/action_trainer.py):
//   tendangan  A tidak jatuh (kaki tumpu), B kena/lurus/jauh (kaki tendang di step
//              menendang), C cepat kembali (waktu step sesudahnya);
//   bangun     kecepatan seluruh page;
//   pose/lain  tidak ada usulan, operator menambah sendiri.
// Mode "belajar" sama dengan tab RL Training (training/learn.js).
//
// Satu percobaan (training/actionTrial.js): kandidat ditulis ke page kerja 254
// (page asli tidak disentuh) -> page 2 -> rekam IMU diam 1 s -> page 254 diputar
// sampai selesai -> rekam 1,5 s lagi -> metrik dihitung apply_node. Page bangun
// dimulai dari BERBARING, tanpa page 2. Operator lalu menilai kestabilan dan hasil;
// yang terbaik disimpan ke page lewat apply_node, dengan cadangan .bin utuh.
//
// CHRONUS: jenis page dibaca dari NAMA page (enum Motion_Index di op_demo.h tidak
// sejalan dengan .bin ini), dan page bangun/pose kiper ikut bisa dilatih -- bukan
// hanya tendangan seperti di ALPHONSE.

const LOCK_NAME = "Action Training";
const RAW_PER_DEG = 4096 / 360;

const STABILITY = [
  { id: "normal", label: "Normal", score: 2, cls: "bg-green-600" },
  { id: "hampir_jatuh", label: "Hampir jatuh (ditangkap)", score: 0, cls: "bg-amber-500" },
  { id: "jatuh", label: "Jatuh", score: -4, cls: "bg-red-600" },
];
// Penilaian hasil menurut jenis page. Pilihan pertama = hasil yang dicari
// (dipakai sebagai "kokoh" oleh mode belajar). Skor: +3 / +1 / -1, sama dengan
// kesesuaian tab RL Training, jadi "jatuh" (-4) selalu kalah.
const OK3 = "bg-green-600";
const OK1 = "bg-amber-500";
const BAD = "bg-red-600";
const OUTCOMES = {
  kick: {
    title: "Bola", distance: true, options: [
      { id: "lurus", label: "Kena & lurus", score: 3, cls: OK3 },
      { id: "kiri", label: "Kena, melenceng kiri", score: 1, cls: OK1 },
      { id: "kanan", label: "Kena, melenceng kanan", score: 1, cls: OK1 },
      { id: "tidak", label: "Tidak kena", score: -1, cls: BAD },
    ],
  },
  getup: {
    title: "Bangun", distance: false, options: [
      { id: "berhasil", label: "Berdiri sendiri", score: 3, cls: OK3 },
      { id: "dibantu", label: "Berdiri, dibantu / terhuyung", score: 1, cls: OK1 },
      { id: "gagal", label: "Gagal bangun", score: -1, cls: BAD },
    ],
  },
  pose: {
    title: "Hasil", distance: false, options: [
      { id: "sesuai", label: "Sesuai harapan", score: 3, cls: OK3 },
      { id: "mendekati", label: "Mendekati", score: 1, cls: OK1 },
      { id: "tidak", label: "Tidak sesuai", score: -1, cls: BAD },
    ],
  },
};
OUTCOMES.umum = OUTCOMES.pose;
const PROFILE_TEXT = { kick: "Tendangan", getup: "Bangun", pose: "Pose", umum: "Lainnya" };
const GROUP_TEXT = { A: "A · tidak jatuh / utama", B: "B · kena, lurus, jauh", C: "C · cepat kembali", M: "manual" };

const outcomeSet = (profile) => OUTCOMES[profile] || OUTCOMES.umum;
const STABILITY_SCORE = Object.fromEntries(STABILITY.map((s) => [s.id, s.score]));
const outcomeScore = (t) => outcomeSet(t.profile).options.find((o) => o.id === t.outcome)?.score ?? 0;
// Jarak hanya menambah (maks +1,5): tidak pernah menutup selisih "lurus" vs "melenceng".
const distanceBonus = (t) => (outcomeSet(t.profile).distance ? Math.min(1.5, Math.max(0, Number(t.distance_cm) || 0) / 200) : 0);
const scoreOf = (t) => (STABILITY_SCORE[t.stability] ?? 0) + outcomeScore(t) + distanceBonus(t);
const isGood = (t) => t.stability !== "jatuh" && outcomeScore(t) > 0;
const isBad = (t) => t.stability === "jatuh" || outcomeScore(t) < 0;
const isSolid = (t) => t.stability === "normal" && t.outcome === outcomeSet(t.profile).options[0].id;
// "Cepat kembali tenang" = goyang IMU sesudah gerakan: pemecah seri.
const settleOf = (t) => Number(t.metrics?.settle_gyro_dps ?? Infinity);
const byBest = (a, b) => scoreOf(b) - scoreOf(a) || settleOf(a) - settleOf(b) || (b.no || 0) - (a.no || 0);
const labelOf = (list, id) => list.find((x) => x.id === id)?.label || id || "-";

const BTN = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";
const BTN_GRAY = `${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`;
const CARD = "bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col overflow-hidden";
const CARD_HEAD = "px-5 py-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50";
const CARD_TITLE = "text-sm font-bold font-display text-gray-800 uppercase tracking-wide";
const INPUT = "border border-gray-200 rounded px-1.5 py-1 text-right font-mono disabled:bg-gray-50";

const makeId = () => `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
const num = (text, fallback) => {
  const v = Number(String(text).trim().replace(",", "."));
  return String(text).trim() !== "" && Number.isFinite(v) ? v : fallback;
};
const deg = (raw) => ((raw - 2048) * 360) / 4096;
const fmtValue = (v, unit) => (unit === "x" ? `×${Number(v).toFixed(2)}` : `${v > 0 ? "+" : ""}${Number(v).toFixed(1)}°`);
const fmt1 = (v) => (Number.isFinite(v) ? v.toFixed(1) : "-");
const variablesFrom = (detail) => (detail?.variables || []).map((v) => ({ ...v, on: v.group === "A", fixed: v.neutral }));

const MANAGER_TEXT = {
  sama: { cls: "text-green-700", text: "op3_manager memutar .bin ini" },
  beda: { cls: "text-red-700", text: "op3_manager memutar .bin LAIN — percobaan ditolak" },
  tidak_diketahui: { cls: "text-gray-500", text: "berkas op3_manager tidak bisa dipastikan" },
};

export default function ActionTrainingPage({ ros, rosState, sendStatus, onBeforeAction }) {
  const connected = rosState === "connected" && !!ros;

  const [pages, setPages] = useState(null);
  const [pageIndex, setPageIndex] = useState(null);
  const [detail, setDetail] = useState(null);
  const [vars, setVars] = useState([]);
  const [mode, setMode] = useState("learn");              // learn | random | asli
  const [session, setSession] = useState(() => `sesi ${new Date().toLocaleDateString("id-ID")}`);
  const sessionTouchedRef = useRef(false);
  const [trials, setTrials] = useState([]);
  const [storePath, setStorePath] = useState("");
  const [phase, setPhase] = useState("idle");             // idle | jalan | nilai
  const [stage, setStage] = useState("");
  const [pending, setPending] = useState(null);
  const [stability, setStability] = useState("");
  const [outcome, setOutcome] = useState("");
  const [distanceText, setDistanceText] = useState("");
  const [note, setNote] = useState("");
  const [manual, setManual] = useState({ joint: "", from: 2, to: 2 });
  const [deploySource, setDeploySource] = useState(null);
  const [deployForm, setDeployForm] = useState({ target: "", name: "", overwrite: false });
  const [deployResult, setDeployResult] = useState(null);
  const abortRef = useRef(false);
  const busy = phase === "jalan";

  // --- ROS ---
  const clientRef = useRef(null);
  const robotRef = useRef(null);
  const onBeforeActionRef = useRef(onBeforeAction);
  onBeforeActionRef.current = onBeforeAction;
  useEffect(() => {
    if (!connected) return undefined;
    const client = createStudioClient(ros, {
      prefix: "act_",
      onResult: (payload) => {
        if (Array.isArray(payload.trials)) setTrials(payload.trials);
        if (payload.action === "act_get" && payload.path) setStorePath(payload.path);
      },
    });
    const robot = createActionRobot(ros, { onPrepare: () => onBeforeActionRef.current?.() });
    clientRef.current = client;
    robotRef.current = robot;
    const timer = window.setTimeout(() => {
      client.ask({ action: "act_get" }).catch(() => {});
      loadPages();
    }, 500);
    return () => {
      window.clearTimeout(timer);
      client.close();
      robot.close();
      clientRef.current = null;
      robotRef.current = null;
    };
    // loadPages hanya memakai clientRef.current; sengaja tidak masuk deps.
  }, [ros, connected]);

  const ask = (payload) => (clientRef.current
    ? clientRef.current.ask(payload)
    : Promise.reject(new Error("robot belum tersambung")));
  const report = (error) => sendStatus(`Action Training: ${error.message}`, true);

  const loadPages = () => ask({ action: "act_pages" })
    .then((res) => {
      setPages(res);
      setPageIndex((prev) => prev ?? res.pages[0]?.index ?? null);
    })
    .catch(report);

  const loadDetail = (index) => ask({ action: "act_page", page: index })
    .then((res) => {
      setDetail(res);
      setVars(variablesFrom(res));
      setManual({ joint: "", from: Math.min(2, res.steps.length), to: Math.max(1, res.steps.length - 1) });
    })
    .catch((error) => { setDetail(null); report(error); });

  useEffect(() => {
    if (connected && pageIndex !== null) {
      setDeploySource(null);
      setDeployResult(null);
      loadDetail(pageIndex);
    }
  }, [pageIndex, connected]);

  // Sampai operator memilih sesi sendiri, ikut sesi percobaan TERAKHIR (nama
  // bawaan = tanggal hari ini; lewat tengah malam riwayat semalam tampak hilang).
  useEffect(() => {
    if (sessionTouchedRef.current || trials.length === 0) return;
    const latest = trials.reduce((a, b) => ((b.at || 0) > (a.at || 0) ? b : a));
    if (latest.session) setSession(latest.session);
  }, [trials]);

  // Riwayat yang boleh dipelajari: sesi ini, page ini, DAN isi page yang sama
  // (sha) -- kandidat dari isi page yang lain tidak sebanding.
  const pageTrials = useMemo(
    () => trials.filter((t) => t.page === detail?.index).sort((a, b) => (b.at || 0) - (a.at || 0)),
    [trials, detail]
  );
  const history = useMemo(
    () => pageTrials.filter((t) => t.session === session && t.page_sha === detail?.sha)
      .sort((a, b) => (b.no || 0) - (a.no || 0)),
    [pageTrials, session, detail]
  );
  const best = useMemo(() => [...history].sort(byBest).slice(0, 3), [history]);
  const sessions = useMemo(() => {
    const set = new Set(trials.map((t) => t.session));
    set.add(session);
    return [...set];
  }, [trials, session]);
  const profile = detail?.profile || "umum";
  const outcomes = outcomeSet(profile);

  // --- variabel ---
  const setVar = (key, patch) => setVars((prev) => prev.map((v) => (v.key === key ? { ...v, ...patch } : v)));
  const chooseStage = (group) => setVars((prev) => prev.map((v) => ({ ...v, on: v.group === group })));
  const useBestAsFixed = () => {
    if (!best[0]) return;
    setVars((prev) => prev.map((v) => ({ ...v, fixed: Number(best[0].values?.[v.key] ?? v.neutral) })));
    sendStatus(`Action Training: nilai tetap diambil dari percobaan #${best[0].no}`);
  };
  const addManual = () => {
    if (!detail || !manual.joint) return;
    const from = clamp(Math.round(Number(manual.from)), 1, detail.steps.length);
    const to = clamp(Math.round(Number(manual.to)), from, detail.steps.length);
    const key = `${manual.joint}@${from}-${to}`;
    if (vars.some((v) => v.key === key)) return;
    setVars((prev) => [...prev, {
      key, label: `${detail.joint_labels[manual.joint]} · step ${from === to ? from : `${from}-${to}`}`,
      group: "M", why: "ditambah operator", unit: "deg", neutral: 0, step: 0.5, lo: -3, hi: 3,
      hard_lo: -detail.hard.deg, hard_hi: detail.hard.deg, on: true, fixed: 0,
    }]);
  };

  const buildValues = () => {
    if (mode === "asli") {
      return { values: Object.fromEntries(vars.map((v) => [v.key, v.neutral])), learn: false,
        basis: "page asli tanpa perubahan (pembanding)" };
    }
    const { varied, learn, basis } = proposeCandidate({
      params: vars.filter((v) => v.on).map((v) => ({
        key: v.key, step: v.step,
        lo: clamp(Math.min(v.lo, v.hi), v.hard_lo, v.hard_hi),
        hi: clamp(Math.max(v.lo, v.hi), v.hard_lo, v.hard_hi),
      })),
      history, mode, scoreOf, isGood, isBad, isSolid,
      valueOf: (t, key) => Number(t.values?.[key] ?? vars.find((v) => v.key === key)?.neutral ?? 0),
    });
    const values = Object.fromEntries(vars.map((v) => [v.key, v.on ? varied[v.key] : Number(v.fixed)]));
    return { values, learn, basis };
  };

  // Node demo juga memutar page dan memindah modul.
  const demoRunning = () => new Promise((resolve) => {
    ros.getNodes((nodes) => resolve((nodes || []).some((n) => /demo|rl_geser_kanan/.test(n))), () => resolve(false));
  });

  // Tidur yang bisa dibatalkan tombol STOP.
  const wait = async (ms) => {
    const end = Date.now() + ms;
    while (Date.now() < end) {
      if (abortRef.current) return false;
      await new Promise((r) => window.setTimeout(r, Math.min(50, end - Date.now())));
    }
    return !abortRef.current;
  };

  // --- satu percobaan ---
  const runTrial = async () => {
    if (!connected || !detail || busy || phase === "nilai" || !robotRef.current) return;
    const holder = lockRobot(LOCK_NAME);
    if (holder) {
      sendStatus(`Action Training: robot sedang dipakai tab ${holder}`, true);
      return;
    }
    try {
      if (await demoRunning()) {
        sendStatus("Action Training: node demo sedang jalan — matikan dulu, page dan modul akan saling timpa", true);
        return;
      }
      const { values, learn, basis } = buildValues();
      abortRef.current = false;
      setPending(null);
      setPhase("jalan");
      setStage(`kandidat ke page ${SCRATCH_PAGE}`);
      const loaded = await ask({ action: "act_load", page: detail.index, sha: detail.sha, values });
      const run = await runActionTrial({
        robot: robotRef.current,
        startRecording: (onSample) => recordImu(ros, onSample),
        wait, onStage: setStage, lying: profile === "getup", plannedS: loaded.planned_s,
      });
      setStage("menghitung metrik");
      const { metrics: m = {} } = await ask({ action: "act_metrics", samples: run.samples, marks: run.marks, profile });
      setPending({
        id: makeId(), session, page: detail.index, page_name: detail.name, page_sha: detail.sha, profile,
        mode: mode === "asli" ? "asli" : learn ? "belajar" : "acak", basis, values,
        varied: mode === "asli" ? [] : vars.filter((v) => v.on).map((v) => v.key),
        diff: loaded.diff, clamped: loaded.clamped, braked: run.braked, timed_out: run.timedOut,
        metrics: Object.fromEntries(["valid", "fell", "duration_s", "gyro_rms_dps", "tilt_max_deg", "settle_dev_deg",
          "settle_gyro_dps", "yaw_abs_deg", "success", "time_to_upright_s", "samples"].map((k) => [k, m[k] ?? null])),
      });
      setStability(m.fell ? "jatuh" : "");
      setOutcome("");
      setDistanceText("");
      setNote(run.braked ? "direm otomatis (badan miring jauh)" : run.timedOut ? "melewati batas waktu, direm" : "");
      setPhase("nilai");
    } catch (error) {
      if (!(error instanceof TrialAborted)) report(error);
      setPhase("idle");
    } finally {
      setStage("");
      unlockRobot(LOCK_NAME);
    }
  };

  const emergencyStop = () => {
    abortRef.current = true;
    robotRef.current?.brake();
    sendStatus("Action Training: STOP — page direm", true);
  };

  const ratedScore = stability && outcome
    ? scoreOf({ profile, stability, outcome, distance_cm: num(distanceText, 0) }) : null;
  const saveRating = () => {
    if (!pending || ratedScore === null) return;
    ask({
      action: "act_add_trial",
      trial: { ...pending, stability, outcome, distance_cm: outcomes.distance ? num(distanceText, null) : null,
        score: ratedScore, note: note.trim() },
    })
      .then((res) => sendStatus(`Action Training: percobaan ke-${res.trial?.no} tersimpan`))
      .catch(report);
    setPending(null);
    setPhase("idle");
  };
  const discardTrial = () => {
    setPending(null);
    setPhase("idle");
    sendStatus("Action Training: percobaan dibuang (tidak dihitung)");
  };
  const deleteTrial = (t) => {
    if (!window.confirm(`Hapus percobaan #${t.no} (${t.session})?`)) return;
    ask({ action: "act_delete_trial", id: t.id }).catch(report);
  };
  const clearScratch = () => ask({ action: "act_restore" })
    .then((res) => {
      sendStatus(res.restored ? `Action Training: page ${SCRATCH_PAGE} dikosongkan` : `Page ${SCRATCH_PAGE} tidak berisi kandidat`);
      loadPages();
    })
    .catch(report);

  // --- simpan hasil ---
  const openDeploy = (t) => {
    setDeploySource(t);
    setDeployResult(null);
    setDeployForm({ target: String(pages?.free ?? ""), name: (detail?.name || "").slice(0, 13), overwrite: false });
  };
  const deploy = () => {
    if (!deploySource || !detail) return;
    const target = Math.round(num(deployForm.target, NaN));
    if (target === detail.index
      && !window.confirm(`Timpa page asli ${detail.index} ${detail.name}? Cadangan .bin dibuat dulu.`)) return;
    if (target === 2 && !window.confirm("Page 2 = WALKING_READY, pose dasar semua gerakan walking. Tetap timpa?")) return;
    ask({
      action: "act_deploy", page: detail.index, sha: detail.sha, values: deploySource.values, target,
      name: deployForm.name.trim() || null, overwrite: deployForm.overwrite || target === detail.index,
    })
      .then((res) => {
        setDeployResult(res);
        sendStatus(`Action Training: hasil #${deploySource.no} disimpan ke page ${res.target} (${res.changed} nilai berubah)`);
        loadPages();
        if (res.target === detail.index) loadDetail(detail.index);
      })
      .catch(report);
  };

  const pageOption = (p) => `${p.index} · ${p.name}${p.motion ? ` (demo: ${p.motion})` : ""} · ${p.steps} step`;
  const phases = detail?.phases;
  const nameMismatch = phases?.kick && phases?.name_side && phases.kick !== phases.name_side;
  const varOf = (key) => vars.find((v) => v.key === key);
  const nextNo = 1 + Math.max(0, ...trials.filter((t) => t.session === session).map((t) => t.no || 0));
  // Sel tabel step yang sedang dilatih, untuk disorot.
  const trained = useMemo(() => {
    const set = new Set();
    vars.filter((v) => v.on).forEach((v) => {
      const [name, steps] = v.key.split("@");
      const [a, b = a] = steps.split("-").map(Number);
      for (let s = a; s <= b; s += 1) set.add(`${name}:${s}`);
    });
    return set;
  }, [vars]);
  const manager = MANAGER_TEXT[pages?.manager?.state] || null;
  const idleHint = profile === "getup"
    ? "Siap. Baringkan robot di matras sesuai page (tengkurap/telentang), tali kendur."
    : profile === "kick"
      ? "Siap. Gantung/pegang robot; letakkan bola di tanda yang sama tiap percobaan."
      : "Siap. Gantung/pegang robot untuk percobaan pertama.";

  return (
    <div className="h-full overflow-y-auto bg-gray-50/50 p-4 md:p-6">
      <div className="grid grid-cols-1 xl:grid-cols-12 gap-6">
        {/* Page + variabel */}
        <div className={`${CARD} xl:col-span-5`}>
          <div className={CARD_HEAD}>
            <div className="flex items-center gap-2">
              <Footprints size={18} className="text-undip-blue" />
              <h2 className={CARD_TITLE}>Action Training — page</h2>
            </div>
            <button className={BTN_GRAY} disabled={!connected || busy}
              onClick={() => { loadPages(); if (pageIndex !== null) loadDetail(pageIndex); }}>
              <RefreshCw size={12} /> Muat ulang
            </button>
          </div>
          <div className="p-5 space-y-4 text-sm">
            <label className="block">
              <span className="text-xs font-bold text-gray-600">Page yang dilatih</span>
              <select className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm"
                value={pageIndex ?? ""} disabled={busy || phase === "nilai" || !pages}
                onChange={(e) => setPageIndex(Number(e.target.value))}>
                {!pages && <option value="">{connected ? "Memuat daftar page…" : "Robot belum tersambung"}</option>}
                {pages && Object.entries(PROFILE_TEXT).map(([key, label]) => {
                  const list = pages.pages.filter((p) => p.profile === key);
                  return list.length > 0 && (
                    <optgroup key={key} label={label}>
                      {list.map((p) => <option key={p.index} value={p.index}>{pageOption(p)}</option>)}
                    </optgroup>
                  );
                })}
              </select>
              {pages && (
                <span className="block mt-1 text-[11px] text-gray-500 break-all">
                  {pages.bin} · page {SCRATCH_PAGE}: {pages.scratch}
                  {manager && <> · <span className={manager.cls}>{manager.text}</span></>}
                </span>
              )}
              {pages?.notes?.length > 0 && (
                <span className="block mt-1 text-[11px] text-amber-700">{pages.notes.join(" · ")}</span>
              )}
            </label>

            {detail && (
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs space-y-1">
                <div>
                  <b className="text-gray-800">{detail.index} {detail.name}</b> · {PROFILE_TEXT[profile]} ({detail.profile_reason})
                  · {detail.steps.length} step · ~{detail.planned_s} dtk (rantai {detail.chain.join(" → ")}) · sha {detail.sha}
                </div>
                {phases?.kick && (
                  <div className="text-gray-600">
                    Kaki tendang <b>{phases.kick === "l" ? "kiri" : "kanan"}</b>, tumpu{" "}
                    <b>{phases.support === "l" ? "kiri" : "kanan"}</b> · satu kaki step {phases.single?.join("–")} · {phases.evidence}
                  </div>
                )}
                {nameMismatch && (
                  <div className="text-amber-700">
                    Nama page menyebut {phases.name_side === "l" ? "kiri" : "kanan"}, tapi data servo: kaki{" "}
                    {phases.kick === "l" ? "kiri" : "kanan"} yang terangkat.
                  </div>
                )}
                {detail.errors.map((e) => <div key={e} className="text-red-700">✗ {e}</div>)}
                {detail.warnings.map((w) => <div key={w} className="text-amber-700">! {w}</div>)}
              </div>
            )}

            {detail && (
              <div>
                <div className="flex flex-wrap justify-between items-center gap-2 mb-2">
                  <span className="text-xs font-bold text-gray-600">Variabel (perubahan dari page asli)</span>
                  <div className="flex flex-wrap gap-1">
                    {["A", "B", "C"].filter((g) => vars.some((v) => v.group === g)).map((g) => (
                      <button key={g} className={BTN_GRAY} disabled={busy} onClick={() => chooseStage(g)} title={GROUP_TEXT[g]}>
                        Tahap {g}
                      </button>
                    ))}
                    <button className={BTN_GRAY} disabled={busy || !best[0]} onClick={useBestAsFixed}
                      title="Nilai tetap (variabel yang tidak dicentang) = percobaan terbaik">
                      <Trophy size={12} /> Tetap = terbaik
                    </button>
                  </div>
                </div>
                {vars.length === 0 && (
                  <p className="text-xs text-gray-500">
                    Tidak ada usulan otomatis untuk page {PROFILE_TEXT[profile].toLowerCase()} ini — tambah variabel di bawah.
                  </p>
                )}
                {vars.length > 0 && (
                  <table className="w-full text-xs">
                    <thead>
                      <tr className="text-gray-500">
                        <th className="text-left font-semibold py-1">Variabel</th>
                        <th className="text-right font-semibold">Min</th>
                        <th className="text-right font-semibold">Max</th>
                        <th className="text-right font-semibold" title="Dipakai kalau tidak dicentang">Tetap</th>
                        <th />
                      </tr>
                    </thead>
                    <tbody>
                      {vars.map((v) => (
                        <tr key={v.key} className="border-t border-gray-100">
                          <td className="py-1.5">
                            <label className="inline-flex items-start gap-2">
                              <input type="checkbox" className="mt-0.5" checked={v.on} disabled={busy}
                                onChange={(e) => setVar(v.key, { on: e.target.checked })} />
                              <span>
                                <span className="font-semibold text-gray-700">{v.group}</span> {v.label}
                                <span className="block text-[10px] text-gray-400">{v.why}</span>
                              </span>
                            </label>
                          </td>
                          {["lo", "hi", "fixed"].map((f) => (
                            <td key={f} className="text-right pl-1">
                              <input type="text" inputMode="decimal" className={`${INPUT} w-14`}
                                disabled={busy || (f === "fixed" ? v.on : !v.on)}
                                defaultValue={v[f]} key={`${v.key}-${f}-${v[f]}`}
                                onBlur={(e) => setVar(v.key, { [f]: clamp(num(e.target.value, v[f]), v.hard_lo, v.hard_hi) })} />
                            </td>
                          ))}
                          <td className="pl-1">
                            {v.group === "M" && (
                              <button className="text-gray-400 hover:text-red-600" disabled={busy}
                                onClick={() => setVars((prev) => prev.filter((x) => x.key !== v.key))} title="Hapus variabel">
                                <Trash2 size={12} />
                              </button>
                            )}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                )}
                <div className="mt-2 flex flex-wrap items-center gap-1 text-xs">
                  <select className="border border-gray-200 rounded px-1.5 py-1" value={manual.joint} disabled={busy}
                    onChange={(e) => setManual((m) => ({ ...m, joint: e.target.value }))}>
                    <option value="">Tambah sendi…</option>
                    {Object.entries(detail.joint_labels).map(([j, label]) => <option key={j} value={j}>{label}</option>)}
                  </select>
                  step
                  <input type="number" min={1} max={detail.steps.length} className={`${INPUT} w-12`} value={manual.from}
                    disabled={busy} onChange={(e) => setManual((m) => ({ ...m, from: e.target.value }))} />
                  –
                  <input type="number" min={1} max={detail.steps.length} className={`${INPUT} w-12`} value={manual.to}
                    disabled={busy} onChange={(e) => setManual((m) => ({ ...m, to: e.target.value }))} />
                  <button className={BTN_GRAY} disabled={busy || !manual.joint} onClick={addManual}>
                    <Plus size={12} /> Tambah
                  </button>
                </div>
                <p className="mt-2 text-[11px] text-gray-500">
                  Derajat = arah raw servo (angka yang sama dengan Action Editor, 11,38 raw per derajat). Batas keras
                  ±{detail.hard.deg}° dari page asli, waktu ×{detail.hard.time.join("–")}. Rentang usulan sengaja
                  sempit (servo 17 pernah reset daya saat lutut melompat jauh). Step pertama dan terakhir tidak
                  diusulkan supaya sambungan dengan page 2 tetap mulus.
                </p>
              </div>
            )}

            {detail && (
              <details className="text-xs">
                <summary className="cursor-pointer font-bold text-gray-600">Isi page (derajat per step)</summary>
                <div className="mt-2 overflow-x-auto">
                  <table className="text-[11px] font-mono">
                    <thead>
                      <tr className="text-gray-500">
                        <th className="text-left pr-2 font-sans">sendi</th>
                        {detail.steps.map((s, i) => (
                          <th key={i} className="text-right px-1" title={`time ${s.time}, pause ${s.pause}`}>
                            s{i + 1}<span className="block text-[9px] font-normal">{s.seconds}s</span>
                          </th>
                        ))}
                      </tr>
                    </thead>
                    <tbody>
                      {Object.keys(detail.joint_labels).map((j) => (
                        <tr key={j}>
                          <td className="pr-2 font-sans text-gray-600 whitespace-nowrap">{detail.joint_labels[j]}</td>
                          {detail.steps.map((s, i) => {
                            const raw = s.positions[j];
                            return (
                              <td key={i} className={`text-right px-1 ${trained.has(`${j}:${i + 1}`) ? "bg-amber-100" : ""}`}>
                                {typeof raw === "number" ? deg(raw).toFixed(1) : raw === "torque_off" ? "off" : "–"}
                              </td>
                            );
                          })}
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </details>
            )}
          </div>
        </div>

        {/* Percobaan + penilaian */}
        <div className={`${CARD} xl:col-span-7`}>
          <div className={CARD_HEAD}>
            <div className="flex items-center gap-2">
              <Dice5 size={18} className="text-undip-blue" />
              <h2 className={CARD_TITLE}>Percobaan ke-{nextNo}</h2>
            </div>
            <span className="text-xs text-gray-500">{history.length} dari isi page ini di sesi ini</span>
          </div>
          <div className="p-5 space-y-4">
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 text-sm">
              <label className="block">
                <span className="text-xs font-bold text-gray-600">Mode</span>
                <select className="mt-1 w-full border border-gray-200 rounded-lg px-3 py-2 text-sm" value={mode}
                  onChange={(e) => setMode(e.target.value)} disabled={busy}>
                  <option value="learn">Belajar (acak di sekitar yang terbaik)</option>
                  <option value="random">Acak penuh dalam rentang</option>
                  <option value="asli">Page asli (pembanding, tanpa perubahan)</option>
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
                  <button type="button" className={BTN_GRAY} disabled={busy}
                    onClick={() => {
                      const name = window.prompt("Nama sesi baru", `sesi ${new Date().toLocaleDateString("id-ID")}`);
                      if (name && name.trim()) { sessionTouchedRef.current = true; setSession(name.trim()); }
                    }}>
                    Baru
                  </button>
                </div>
              </label>
            </div>

            <div className="flex flex-wrap gap-2 items-center">
              <button className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90 px-5 py-3 text-sm`}
                disabled={!connected || !detail || detail.errors.length > 0 || busy || phase === "nilai"
                  || pages?.manager?.state === "beda"}
                onClick={runTrial}>
                <Play size={16} /> Play percobaan
              </button>
              <button className={`${BTN} bg-red-600 text-white hover:bg-red-700 px-5 py-3 text-sm`}
                disabled={!connected} onClick={emergencyStop}>
                <OctagonX size={16} /> STOP
              </button>
              <button className={BTN_GRAY} disabled={!connected || busy} onClick={clearScratch}
                title={`Kembalikan page ${SCRATCH_PAGE} ke kosong`}>
                <Eraser size={12} /> Kosongkan page {SCRATCH_PAGE}
              </button>
              <span className="text-sm text-gray-600">
                {!connected && "Robot belum tersambung"}
                {connected && phase === "idle" && idleHint}
                {phase === "jalan" && `Berjalan: ${stage}…`}
                {phase === "nilai" && "Nilai percobaan ini:"}
              </span>
            </div>

            {pending && (
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs space-y-2">
                <div className="text-gray-500">Mode {pending.mode}{pending.basis && <> · {pending.basis}</>}</div>
                <div className="flex flex-wrap gap-x-4 gap-y-1 font-mono">
                  {pending.varied.map((k) => (
                    <span key={k}>{varOf(k)?.label || k}: <b>{fmtValue(pending.values[k], varOf(k)?.unit)}</b></span>
                  ))}
                  {pending.varied.length === 0 && <span>(tidak ada yang diacak)</span>}
                </div>
                <details>
                  <summary className="cursor-pointer text-gray-600">
                    {pending.diff.length} nilai servo/waktu berubah di page {SCRATCH_PAGE}
                    {pending.clamped.length > 0 && <span className="text-amber-700"> · terjepit 0..4095: {pending.clamped.join(", ")}</span>}
                  </summary>
                  <div className="mt-1 font-mono text-[11px] text-gray-600 max-h-40 overflow-y-auto">
                    {pending.diff.map((d) => (
                      <div key={`${d.step}-${d.joint}`}>
                        step {d.step} · {d.joint === "time" ? `time/pause ${d.before.join("/")} → ${d.after.join("/")}`
                          : `${detail?.joint_labels?.[d.joint] || d.joint}: ${d.before} → ${d.after} (${(d.after - d.before) > 0 ? "+" : ""}${((d.after - d.before) / RAW_PER_DEG).toFixed(1)}°)`}
                      </div>
                    ))}
                  </div>
                </details>
                <div className="flex flex-wrap gap-x-4 gap-y-1 text-gray-600">
                  {!pending.metrics.valid && <span className="text-amber-700">Metrik IMU tidak sah (rekaman terlalu pendek)</span>}
                  {pending.metrics.fell && <b className="text-red-700">IMU: jatuh terdeteksi</b>}
                  {pending.profile === "getup" && pending.metrics.valid && (
                    <span>{pending.metrics.success ? `IMU: tegak dalam ${fmt1(pending.metrics.time_to_upright_s)} dtk` : "IMU: belum tegak di akhir"}</span>
                  )}
                  <span>goyang {fmt1(pending.metrics.gyro_rms_dps)} °/s</span>
                  <span>miring maks {fmt1(pending.metrics.tilt_max_deg)}°</span>
                  <span>miring akhir {fmt1(pending.metrics.settle_dev_deg)}°</span>
                  <span>goyang sesudah {fmt1(pending.metrics.settle_gyro_dps)} °/s</span>
                  <span>putar {fmt1(pending.metrics.yaw_abs_deg)}°</span>
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
                  <div className="text-xs font-bold text-gray-600 mb-1">{outcomes.title}</div>
                  <div className="flex flex-wrap gap-2">
                    {outcomes.options.map((o) => (
                      <button key={o.id} onClick={() => setOutcome(o.id)}
                        className={`${BTN} px-4 py-2.5 text-sm ${outcome === o.id ? `${o.cls} text-white` : "bg-gray-100 text-gray-700 hover:bg-gray-200"}`}>
                        {o.label}
                      </button>
                    ))}
                  </div>
                </div>
                <div className="flex flex-wrap gap-3 items-end">
                  {outcomes.distance && (
                    <label className="block">
                      <span className="text-xs font-bold text-gray-600">Jarak bola (cm, boleh kosong)</span>
                      <input type="text" inputMode="decimal" className="mt-1 block w-32 border border-gray-200 rounded-lg px-3 py-2 text-sm"
                        value={distanceText} onChange={(e) => setDistanceText(e.target.value)} />
                    </label>
                  )}
                  <label className="block flex-1 min-w-[12rem]">
                    <span className="text-xs font-bold text-gray-600">Catatan</span>
                    <input type="text" className="mt-1 block w-full border border-gray-200 rounded-lg px-3 py-2 text-sm"
                      value={note} onChange={(e) => setNote(e.target.value)} />
                  </label>
                </div>
                <div className="flex flex-wrap gap-2 items-center">
                  <button className={`${BTN} bg-green-600 text-white hover:bg-green-700 px-5 py-2.5 text-sm`}
                    disabled={ratedScore === null} onClick={saveRating}>
                    <Save size={14} /> Simpan penilaian {ratedScore !== null && `(skor ${ratedScore.toFixed(1)})`}
                  </button>
                  <button className={`${BTN_GRAY} px-4 py-2.5 text-sm`} onClick={discardTrial}>
                    Buang (tidak dihitung)
                  </button>
                </div>
              </div>
            )}

            {best.length > 0 && (
              <div className="rounded-xl border border-amber-200 bg-amber-50 p-3 text-xs space-y-1">
                <div className="font-bold text-amber-800 flex items-center gap-1"><Trophy size={14} /> Terbaik di sesi ini (isi page ini)</div>
                {best.map((t) => (
                  <div key={t.id} className="flex flex-wrap items-center gap-2">
                    <span>#{t.no} skor <b>{scoreOf(t).toFixed(1)}</b> ({labelOf(STABILITY, t.stability)},{" "}
                      {labelOf(outcomeSet(t.profile).options, t.outcome)}{t.distance_cm ? `, ${t.distance_cm} cm` : ""})
                      · goyang sesudah {fmt1(t.metrics?.settle_gyro_dps)} °/s</span>
                    <button className={`${BTN} bg-white border border-amber-300 text-amber-800 hover:bg-amber-100 py-1`}
                      disabled={busy} onClick={() => openDeploy(t)}>
                      <Save size={12} /> Simpan ke page…
                    </button>
                  </div>
                ))}
              </div>
            )}

            {deploySource && detail && (
              <div className="rounded-xl border border-gray-200 p-3 text-xs space-y-2">
                <div className="font-bold text-gray-700">
                  Simpan percobaan #{deploySource.no} sebagai page (isi asli page {detail.index} + nilai percobaan itu)
                </div>
                <div className="flex flex-wrap gap-3 items-end">
                  <label className="block">
                    <span className="text-gray-600">Page tujuan</span>
                    <input type="text" inputMode="numeric" className="mt-1 block w-20 border border-gray-200 rounded-lg px-2 py-1.5"
                      value={deployForm.target} onChange={(e) => setDeployForm((f) => ({ ...f, target: e.target.value }))} />
                  </label>
                  <label className="block">
                    <span className="text-gray-600">Nama (maks 13)</span>
                    <input type="text" maxLength={13} className="mt-1 block w-36 border border-gray-200 rounded-lg px-2 py-1.5"
                      value={deployForm.name} onChange={(e) => setDeployForm((f) => ({ ...f, name: e.target.value }))} />
                  </label>
                  <label className="inline-flex items-center gap-1.5 pb-2">
                    <input type="checkbox" checked={deployForm.overwrite}
                      onChange={(e) => setDeployForm((f) => ({ ...f, overwrite: e.target.checked }))} />
                    timpa kalau page tujuan berisi gerakan
                  </label>
                  <button className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90`} disabled={busy} onClick={deploy}>
                    <Save size={12} /> Simpan
                  </button>
                  <button className={BTN_GRAY} onClick={() => setDeploySource(null)}>Batal</button>
                </div>
                <p className="text-gray-500">
                  Saran: simpan ke page kosong dulu (bawaan: page kosong pertama, {pages?.free ?? "-"}), uji, baru timpa
                  page {detail.index}. Demo memutar page menurut nomornya (D58: tahan bola 14, bangun belakang 12).
                  Cadangan utuh .bin dibuat sebelum menulis; action_module langsung memakai isi baru saat page diputar.
                </p>
                {deployResult && (
                  <p className="text-green-700 break-all">
                    Tersimpan di page {deployResult.target} ({deployResult.name}), {deployResult.changed} nilai berubah.
                    Cadangan: {deployResult.backup}. Tab Action: muat ulang dari bin (↓) sebelum menyunting di sana.
                    {deployResult.notes?.map((n) => <span key={n} className="block text-amber-700">{n}</span>)}
                  </p>
                )}
              </div>
            )}

            <div>
              <div className="flex justify-between items-center mb-1">
                <span className="text-xs font-bold text-gray-600">Riwayat page ini ({pageTrials.length})</span>
                {storePath && <span className="text-[10px] text-gray-400 break-all">{storePath}</span>}
              </div>
              <div className="overflow-x-auto">
                <table className="w-full text-xs">
                  <thead>
                    <tr className="text-gray-500 text-left">
                      <th className="font-semibold py-1 pr-2">#</th>
                      <th className="font-semibold pr-2">Sesi</th>
                      <th className="font-semibold pr-2">Mode</th>
                      <th className="font-semibold pr-2 text-right">Skor</th>
                      <th className="font-semibold pr-2">Stabil</th>
                      <th className="font-semibold pr-2">Hasil</th>
                      <th className="font-semibold pr-2 text-right">Jarak</th>
                      <th className="font-semibold pr-2 text-right" title="Goyang IMU sesudah gerakan">Sesudah</th>
                      <th className="font-semibold pr-2">Nilai</th>
                      <th />
                    </tr>
                  </thead>
                  <tbody>
                    {pageTrials.map((t) => (
                      <tr key={t.id} className={`border-t border-gray-100 align-top ${t.page_sha !== detail?.sha ? "text-gray-400" : ""}`}>
                        <td className="py-1 pr-2">{t.no}</td>
                        <td className="pr-2 whitespace-nowrap">{t.session}</td>
                        <td className="pr-2">{t.mode}</td>
                        <td className="pr-2 text-right font-mono">{scoreOf(t).toFixed(1)}</td>
                        <td className="pr-2">{labelOf(STABILITY, t.stability)}</td>
                        <td className="pr-2">{labelOf(outcomeSet(t.profile).options, t.outcome)}</td>
                        <td className="pr-2 text-right">{t.distance_cm ?? "-"}</td>
                        <td className="pr-2 text-right font-mono">{fmt1(t.metrics?.settle_gyro_dps)}</td>
                        <td className="pr-2 font-mono text-[10px]">
                          {(t.varied || []).map((k) => `${k}=${t.values?.[k]}`).join(" ") || "page asli"}
                          {t.page_sha !== detail?.sha && <span className="block font-sans">isi page lain (sha {t.page_sha})</span>}
                          {t.note && <span className="block font-sans text-gray-500">{t.note}</span>}
                        </td>
                        <td className="whitespace-nowrap">
                          {t.page_sha === detail?.sha && (
                            <button className="text-gray-400 hover:text-undip-blue mr-2" disabled={busy}
                              onClick={() => openDeploy(t)} title="Simpan ke page">
                              <Save size={12} />
                            </button>
                          )}
                          <button className="text-gray-400 hover:text-red-600" disabled={busy}
                            onClick={() => deleteTrial(t)} title="Hapus percobaan">
                            <Trash2 size={12} />
                          </button>
                        </td>
                      </tr>
                    ))}
                    {pageTrials.length === 0 && (
                      <tr><td colSpan={10} className="py-2 text-gray-400">Belum ada percobaan untuk page ini.</td></tr>
                    )}
                  </tbody>
                </table>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
