import { useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { Dice5, Eraser, Footprints, OctagonX, Play, Plus, RefreshCw, Save, Trash2, Trophy } from "lucide-react";
import { recordImu, tiltDeg } from "./imuRecording.js";
import { clamp, proposeCandidate } from "./trainingLearn.js";

// Action Training: melatih NILAI SERVO page Action Editor di robot asli dengan
// penilaian operator (keputusan operator 2026-10-31: semua page tendangan di
// op_demo.h -- kanan 105/32/218/220/6, kiri 104/219/221/7).
//
// Variabel = perubahan derajat (arah raw, angka yang sama dengan Action Editor)
// pada satu sendi di rentang step, atau pengali waktu step. Usulannya disusun
// server dari data servo page itu (scripts/training_action_web.py), bertahap:
//   A tidak jatuh (kaki tumpu), B kena / lurus / jauh (kaki tendang di step
//   menendang), C cepat kembali (waktu step sesudahnya).
// Batas = nilai page asli +- margin, dijepit server ke batas keras.
//
// Satu percobaan: kandidat ditulis ke page kerja 254 (page asli tidak
// disentuh) -> page 2 (posisi siap, sama dengan Apply & Start) -> rekam IMU
// diam 1 s -> page 254 diputar sampai selesai (termasuk page lanjutannya) ->
// rekam 1,5 s lagi -> metrik dihitung server dengan rumus menu Training.
// Operator lalu menilai: stabil, kena, jarak. Yang terbaik disimpan ke page
// lewat server, dengan cadangan .bin utuh.
//
// Skor (prioritas operator: tidak jatuh > kena & lurus > jarak > cepat
// kembali): stabil + kena + jarak (maks +1,5, tidak pernah menutup selisih
// "lurus" vs "melenceng"). "Cepat kembali" = goyang IMU sesudah gerakan,
// dipakai sebagai pemecah seri.

const RESULT_TOPIC = "/bascorro_studio/result";
const REQUEST_TOPIC = "/bascorro_studio/request";
const SCRATCH_PAGE = 254;      // page kerja training, sama dengan menu Training
const BRAKE_PAGE = -2;         // action_module: hentikan seketika
const STAND_MS = 1000;         // rekaman diam sebelum page diputar = acuan metrik
const SETTLE_MS = 1500;        // autotune_action_core.SETTLE_S
const FALL_DEG = 50;           // rem kalau badan miring sejauh ini dari posisi siap
const REPLY_MS = 10000;
const RAW_PER_DEG = 4096 / 360;

const STABILITY = [
  { id: "normal", label: "Normal", score: 2, cls: "bg-emerald-600" },
  { id: "hampir_jatuh", label: "Hampir jatuh (ditangkap)", score: 0, cls: "bg-amber-500" },
  { id: "jatuh", label: "Jatuh", score: -4, cls: "bg-red-600" },
];
const HIT = [
  { id: "lurus", label: "Kena & lurus", score: 3, cls: "bg-emerald-600" },
  { id: "kiri", label: "Kena, melenceng kiri", score: 1, cls: "bg-amber-500" },
  { id: "kanan", label: "Kena, melenceng kanan", score: 1, cls: "bg-amber-500" },
  { id: "tidak", label: "Tidak kena", score: -1, cls: "bg-red-600" },
];
const STABILITY_SCORE = Object.fromEntries(STABILITY.map((s) => [s.id, s.score]));
const HIT_SCORE = Object.fromEntries(HIT.map((h) => [h.id, h.score]));
const GROUP_TEXT = { A: "A · tidak jatuh", B: "B · kena, lurus, jauh", C: "C · cepat kembali", M: "manual" };

const distanceBonus = (t) => Math.min(1.5, Math.max(0, Number(t.distance_cm) || 0) / 200);
const scoreOf = (t) => (STABILITY_SCORE[t.stability] ?? 0) + (HIT_SCORE[t.hit] ?? 0) + distanceBonus(t);
const isGood = (t) => t.stability !== "jatuh" && !!t.hit && t.hit !== "tidak";
const isBad = (t) => t.stability === "jatuh" || t.hit === "tidak";
const isSolid = (t) => t.stability === "normal" && t.hit === "lurus";
const settleOf = (t) => Number(t.metrics?.settle_gyro_dps ?? Infinity);
const byBest = (a, b) => scoreOf(b) - scoreOf(a) || settleOf(a) - settleOf(b) || (b.no || 0) - (a.no || 0);
const labelOf = (list, id) => list.find((x) => x.id === id)?.label || id || "-";

const BTN = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";
const CARD = "bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col overflow-hidden";
const CARD_HEAD = "px-5 py-3 border-b border-gray-100 flex justify-between items-center bg-gray-50/50";
const CARD_TITLE = "text-sm font-bold font-display text-gray-800 uppercase tracking-wide";
const INPUT = "border border-gray-200 rounded px-1.5 py-1 text-right font-mono disabled:bg-gray-50";

const makeId = () => `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
const num = (text, fallback) => {
  const v = Number(String(text).trim().replace(",", "."));
  return Number.isFinite(v) ? v : fallback;
};
const deg = (raw) => (raw - 2048) * 360 / 4096;
const fmtValue = (v, unit) => (unit === "x" ? `×${Number(v).toFixed(2)}` : `${v > 0 ? "+" : ""}${Number(v).toFixed(1)}°`);

function variablesFrom(detail) {
  return (detail?.variables || []).map((v) => ({ ...v, on: v.group === "A", fixed: v.neutral }));
}

export default function ActionTrainingPage({ ros, rosState, sendStatus, runActionTrial, cancelOperation }) {
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
  const [hit, setHit] = useState("");
  const [distanceText, setDistanceText] = useState("");
  const [note, setNote] = useState("");
  const [manual, setManual] = useState({ joint: "", from: 2, to: 2 });
  const [deploySource, setDeploySource] = useState(null);
  const [deployForm, setDeployForm] = useState({ target: "", name: "", overwrite: false });
  const [deployResult, setDeployResult] = useState(null);
  const abortRef = useRef(false);

  // --- ROS: permintaan ke apply_node dijawab lewat request_id ---
  const rosRefs = useRef({});
  const waitingRef = useRef(new Map());
  useEffect(() => {
    if (!connected) return undefined;
    const refs = {
      request: new ROSLIB.Topic({ ros, name: REQUEST_TOPIC, messageType: "std_msgs/String" }),
      result: new ROSLIB.Topic({ ros, name: RESULT_TOPIC, messageType: "std_msgs/String" }),
      page: new ROSLIB.Topic({ ros, name: "/robotis/action/page_num", messageType: "std_msgs/Int32" }),
    };
    // Diiklankan sekarang: pesan pertama sesudah iklan bisa hilang selagi DDS
    // mencari pelanggannya, padahal pesan pertama di sini bisa jadi rem.
    refs.page.advertise();
    rosRefs.current = refs;
    refs.result.subscribe((msg) => {
      let payload;
      try {
        payload = JSON.parse(msg.data);
      } catch {
        return;
      }
      if (typeof payload?.action !== "string" || !payload.action.startsWith("act_")) return;
      if (Array.isArray(payload.trials)) setTrials(payload.trials);
      if (payload.action === "act_get" && payload.path) setStorePath(payload.path);
      const waiting = waitingRef.current.get(payload.request_id);
      if (!waiting) return;
      waitingRef.current.delete(payload.request_id);
      window.clearTimeout(waiting.timer);
      if (payload.ok === false) waiting.reject(new Error(payload.message || `${payload.action} gagal`));
      else waiting.resolve(payload);
    });
    const timer = window.setTimeout(() => {
      ask({ action: "act_get" }).catch(() => {});
      loadPages();
    }, 500);
    return () => {
      window.clearTimeout(timer);
      refs.result.unsubscribe();
      refs.page.unadvertise();
      waitingRef.current.forEach((w) => { window.clearTimeout(w.timer); w.reject(new Error("koneksi ROS berganti")); });
      waitingRef.current.clear();
      rosRefs.current = {};
    };
    // ask/loadPages hanya memakai rosRefs.current; sengaja tidak masuk deps.
  }, [ros, connected]);

  const ask = (payload) => new Promise((resolve, reject) => {
    const pub = rosRefs.current.request;
    if (!pub) {
      reject(new Error("robot belum tersambung"));
      return;
    }
    const id = makeId();
    const timer = window.setTimeout(() => {
      waitingRef.current.delete(id);
      reject(new Error(`${payload.action}: server Studio (apply_node) tidak menjawab`));
    }, REPLY_MS);
    waitingRef.current.set(id, { resolve, reject, timer });
    pub.publish(new ROSLIB.Message({ data: JSON.stringify({ ...payload, request_id: id }) }));
  });

  const report = (error) => sendStatus(`Action Training: ${error.message}`, true);
  const publishPage = (page) => rosRefs.current.page?.publish(new ROSLIB.Message({ data: page }));

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
      setManual((prev) => ({ ...prev, joint: "", from: 2, to: Math.max(2, res.steps.length - 1) }));
    })
    .catch((error) => { setDetail(null); report(error); });

  useEffect(() => {
    if (connected && pageIndex !== null) {
      setDeploySource(null);
      setDeployResult(null);
      loadDetail(pageIndex);
    }
  }, [pageIndex, connected]);

  // Sampai operator memilih sesi sendiri, ikut sesi percobaan TERAKHIR.
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
  const busy = phase === "jalan";

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

  // op_demo_node dan pelatih menu Training juga memutar page.
  const demoRunning = () =>
    new Promise((resolve) => {
      ros.getNodes(
        (nodes) => resolve((nodes || []).some((n) => /demo|autotune/.test(n))),
        () => resolve(false)
      );
    });

  // Rekaman IMU + rem darurat kalau badan miring jauh dari posisi siap; jatuh
  // yang dicatat dideteksi ulang di server.
  const startRecording = () => {
    let ref = null;
    const rec = recordImu(ros, (sample) => {
      const tilt = tiltDeg(sample);
      if (!ref) ref = tilt;
      if (rec.watchFall && !rec.fell && Math.hypot(tilt.roll - ref.roll, tilt.pitch - ref.pitch) > FALL_DEG) {
        rec.fell = true;
        publishPage(BRAKE_PAGE);
      }
    });
    rec.fell = false;
    rec.watchFall = false;
    return rec;
  };

  // --- satu percobaan ---
  const runTrial = async () => {
    if (!connected || !detail || busy || phase === "nilai") return;
    if (await demoRunning()) {
      sendStatus("Action Training: node demo / autotune sedang jalan — matikan dulu, page akan saling timpa", true);
      return;
    }
    const { values, learn, basis } = buildValues();
    abortRef.current = false;
    setPending(null);
    setPhase("jalan");
    let outcome = null;
    const ok = await runActionTrial(async (context, robot) => {
      setStage(`kandidat ke page ${SCRATCH_PAGE}`);
      const loaded = await ask({ action: "act_load", page: detail.index, sha: detail.sha, values });
      context.check();
      setStage("page 2 (posisi siap)");
      await robot.playWalkingReady();
      const rec = startRecording();
      try {
        setStage("merekam diam");
        await context.wait(STAND_MS);
        if (rec.samples.length < 10) throw new Error("IMU tidak terbit (/robotis/open_cr/imu)");
        const marks = { stand0: rec.samples[0][0], start: rec.last(), planned_s: loaded.planned_s };
        rec.watchFall = true;
        setStage(`memutar page ${SCRATCH_PAGE}`);
        publishPage(SCRATCH_PAGE);
        const begun = Date.now();
        const timeoutMs = (1.5 * loaded.planned_s + 5) * 1000;
        let started = false;
        let timedOut = false;
        for (;;) {
          await context.wait(100);
          if (rec.fell) break;                          // sudah direm di rekaman
          const running = await robot.isRunning();
          started = started || running;
          if (!running && (started || Date.now() - begun > 1000)) break;
          if (Date.now() - begun > timeoutMs) {
            publishPage(BRAKE_PAGE);
            timedOut = true;
            break;
          }
        }
        if (!started && !rec.fell) {
          throw new Error(`action_module tidak memutar page ${SCRATCH_PAGE} (lihat /robotis/status)`);
        }
        marks.done = rec.last();
        setStage("merekam sesudah gerakan");
        await context.wait(SETTLE_MS);
        marks.end = rec.last();
        rec.stop();
        const res = await ask({ action: "act_metrics", samples: rec.samples, marks, profile: detail.profile });
        outcome = { loaded, metrics: res.metrics, timedOut, braked: rec.fell };
      } finally {
        rec.stop();
      }
    });
    setStage("");
    if (!ok || !outcome) {
      setPhase("idle");
      return;
    }
    const m = outcome.metrics || {};
    setPending({
      id: makeId(), session, page: detail.index, page_name: detail.name, page_sha: detail.sha,
      profile: detail.profile, mode: mode === "asli" ? "asli" : learn ? "belajar" : "acak", basis, values,
      varied: mode === "asli" ? [] : vars.filter((v) => v.on).map((v) => v.key),
      diff: outcome.loaded.diff, clamped: outcome.loaded.clamped, braked: outcome.braked,
      timed_out: outcome.timedOut,
      metrics: Object.fromEntries(["valid", "fell", "duration_s", "gyro_rms_dps", "tilt_max_deg", "settle_dev_deg",
        "settle_gyro_dps", "yaw_abs_deg", "samples"].map((k) => [k, m[k] ?? null])),
    });
    setStability(m.fell ? "jatuh" : "");
    setHit("");
    setDistanceText("");
    setNote(outcome.braked ? "direm otomatis (badan miring jauh)" : outcome.timedOut ? "melewati batas waktu, direm" : "");
    setPhase("nilai");
  };

  const emergencyStop = () => {
    abortRef.current = true;
    publishPage(BRAKE_PAGE);
    cancelOperation?.();
    sendStatus("Action Training: STOP — page direm", true);
  };

  const ratedScore = stability && hit ? scoreOf({ stability, hit, distance_cm: num(distanceText, 0) }) : null;
  const saveRating = () => {
    if (!pending || ratedScore === null) return;
    const distance = num(distanceText, null);
    ask({
      action: "act_add_trial",
      trial: { ...pending, stability, hit, distance_cm: distance, score: ratedScore, note: note.trim() },
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
    if (target === detail.index && !window.confirm(`Timpa page asli ${detail.index} ${detail.name}? Cadangan .bin dibuat dulu.`)) return;
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

  const pageOption = (p) => `${p.index} · ${p.name}${p.motion ? ` (${p.motion})` : ""} · ${p.steps} step`;
  const phases = detail?.phases;
  const nameMismatch = phases?.kick && phases?.name_side && phases.kick !== phases.name_side;
  const varOf = (key) => vars.find((v) => v.key === key);
  // apply_node menomori per sesi, lintas page.
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
            <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={!connected || busy}
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
                {!pages && <option value="">Memuat daftar page…</option>}
                {pages && (
                  <>
                    <optgroup label="Tendangan (op_demo.h)">
                      {pages.pages.filter((p) => p.kick).map((p) => <option key={p.index} value={p.index}>{pageOption(p)}</option>)}
                    </optgroup>
                    <optgroup label="Page lain">
                      {pages.pages.filter((p) => !p.kick).map((p) => <option key={p.index} value={p.index}>{pageOption(p)}</option>)}
                    </optgroup>
                  </>
                )}
              </select>
              {pages && (
                <span className="block mt-1 text-[11px] text-gray-500 break-all">
                  {pages.bin} · page {SCRATCH_PAGE}: {pages.scratch}
                </span>
              )}
            </label>

            {detail && (
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs space-y-1">
                <div>
                  <b className="text-gray-800">{detail.index} {detail.name}</b> · {detail.steps.length} step ·
                  {" "}~{detail.planned_s} dtk (rantai {detail.chain.join(" → ")}) · sha {detail.sha}
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
                    {phases.kick === "l" ? "kiri" : "kanan"} yang menendang.
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
                      <button key={g} className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={busy}
                        onClick={() => chooseStage(g)} title={GROUP_TEXT[g]}>
                        Tahap {g}
                      </button>
                    ))}
                    <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={busy || !best[0]}
                      onClick={useBestAsFixed} title="Nilai tetap (variabel yang tidak dicentang) = percobaan terbaik">
                      <Trophy size={12} /> Tetap = terbaik
                    </button>
                  </div>
                </div>
                {vars.length === 0 && (
                  <p className="text-xs text-gray-500">
                    Page ini tidak terbaca sebagai gerakan satu kaki — tambah variabel manual di bawah.
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
                  <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={busy || !manual.joint}
                    onClick={addManual}>
                    <Plus size={12} /> Tambah
                  </button>
                </div>
                <p className="mt-2 text-[11px] text-gray-500">
                  Derajat = arah raw servo (angka yang sama dengan Action Editor, 11,38 raw per derajat). Batas keras
                  ±{detail.hard.deg}° dari page asli, waktu ×{detail.hard.time.join("–")}. Step pertama dan terakhir
                  tidak diusulkan supaya sambungan dengan page 2 tetap mulus.
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

            <div className="flex flex-wrap gap-2 items-center">
              <button className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90 px-5 py-3 text-sm`}
                disabled={!connected || !detail || detail.errors.length > 0 || busy || phase === "nilai"} onClick={runTrial}>
                <Play size={16} /> Play percobaan
              </button>
              <button className={`${BTN} bg-red-600 text-white hover:bg-red-700 px-5 py-3 text-sm`}
                disabled={!connected} onClick={emergencyStop}>
                <OctagonX size={16} /> STOP
              </button>
              <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} disabled={!connected || busy}
                onClick={clearScratch} title={`Kembalikan page ${SCRATCH_PAGE} ke kosong`}>
                <Eraser size={12} /> Kosongkan page {SCRATCH_PAGE}
              </button>
              <span className="text-sm text-gray-600">
                {!connected && "Robot belum tersambung"}
                {connected && phase === "idle" && "Siap. Pegang robot; letakkan bola di tanda yang sama tiap percobaan."}
                {phase === "jalan" && `Berjalan: ${stage}…`}
                {phase === "nilai" && "Nilai percobaan ini:"}
              </span>
            </div>

            {pending && (
              <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs space-y-2">
                <div className="text-gray-500">
                  Mode {pending.mode}{pending.basis && <> · {pending.basis}</>}
                </div>
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
                          : `${detail?.joint_labels?.[d.joint] || d.joint}: ${d.before} → ${d.after} (${(d.after - d.before) / RAW_PER_DEG > 0 ? "+" : ""}${((d.after - d.before) / RAW_PER_DEG).toFixed(1)}°)`}
                      </div>
                    ))}
                  </div>
                </details>
                <div className="flex flex-wrap gap-x-4 gap-y-1 text-gray-600">
                  {!pending.metrics.valid && <span className="text-amber-700">Metrik IMU tidak sah (rekaman terlalu pendek)</span>}
                  {pending.metrics.fell && <b className="text-red-700">IMU: jatuh terdeteksi</b>}
                  <span>goyang {pending.metrics.gyro_rms_dps?.toFixed(1) ?? "-"} °/s</span>
                  <span>miring maks {pending.metrics.tilt_max_deg?.toFixed(1) ?? "-"}°</span>
                  <span>miring akhir {pending.metrics.settle_dev_deg?.toFixed(1) ?? "-"}°</span>
                  <span>goyang sesudah {pending.metrics.settle_gyro_dps?.toFixed(1) ?? "-"} °/s</span>
                  <span>putar {pending.metrics.yaw_abs_deg?.toFixed(1) ?? "-"}°</span>
                </div>
              </div>
            )}

            {phase === "nilai" && pending && (
              <div className="space-y-3">
                <div>
                  <div className="text-xs font-bold text-gray-600 mb-1">Stabil</div>
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
                  <div className="text-xs font-bold text-gray-600 mb-1">Bola</div>
                  <div className="flex flex-wrap gap-2">
                    {HIT.map((h) => (
                      <button key={h.id} onClick={() => setHit(h.id)}
                        className={`${BTN} px-4 py-2.5 text-sm ${hit === h.id ? `${h.cls} text-white` : "bg-gray-100 text-gray-700 hover:bg-gray-200"}`}>
                        {h.label}
                      </button>
                    ))}
                  </div>
                </div>
                <div className="flex flex-wrap gap-3 items-end">
                  <label className="block">
                    <span className="text-xs font-bold text-gray-600">Jarak bola (cm, boleh kosong)</span>
                    <input type="text" inputMode="decimal" className="mt-1 block w-32 border border-gray-200 rounded-lg px-3 py-2 text-sm"
                      value={distanceText} onChange={(e) => setDistanceText(e.target.value)} />
                  </label>
                  <label className="block flex-1 min-w-[12rem]">
                    <span className="text-xs font-bold text-gray-600">Catatan</span>
                    <input type="text" className="mt-1 block w-full border border-gray-200 rounded-lg px-3 py-2 text-sm"
                      value={note} onChange={(e) => setNote(e.target.value)} />
                  </label>
                </div>
                <div className="flex flex-wrap gap-2 items-center">
                  <button className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90 px-5 py-2.5 text-sm`}
                    disabled={ratedScore === null} onClick={saveRating}>
                    <Save size={14} /> Simpan penilaian {ratedScore !== null && `(skor ${ratedScore.toFixed(1)})`}
                  </button>
                  <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200 px-4 py-2.5 text-sm`} onClick={discardTrial}>
                    Buang
                  </button>
                </div>
              </div>
            )}

            {/* Terbaik + simpan hasil */}
            {best.length > 0 && (
              <div className="rounded-xl border border-emerald-200 bg-emerald-50/50 p-3 text-xs space-y-1">
                <div className="font-bold text-emerald-800 flex items-center gap-1"><Trophy size={12} /> Terbaik di sesi ini</div>
                {best.map((t) => (
                  <div key={t.id} className="flex flex-wrap items-center gap-2">
                    <span>#{t.no} skor <b>{scoreOf(t).toFixed(1)}</b> ({labelOf(STABILITY, t.stability)}, {labelOf(HIT, t.hit)}
                      {t.distance_cm ? `, ${t.distance_cm} cm` : ""}) · goyang sesudah {t.metrics?.settle_gyro_dps?.toFixed(1) ?? "-"} °/s</span>
                    <button className={`${BTN} bg-white border border-emerald-300 text-emerald-800 hover:bg-emerald-100 py-1`}
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
                  <button className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} onClick={() => setDeploySource(null)}>
                    Batal
                  </button>
                </div>
                <p className="text-gray-500">
                  Saran: simpan ke page kosong dulu (bawaan: page kosong pertama, {pages?.free ?? "-"}), uji, baru timpa
                  page {detail.index}. D60 dan demo memutar page {detail.index} menurut nomornya. Cadangan utuh .bin
                  dibuat sebelum menulis; action_module langsung memakai isi baru saat page diputar.
                </p>
                {deployResult && (
                  <p className="text-emerald-700 break-all">
                    Tersimpan di page {deployResult.target} ({deployResult.name}), {deployResult.changed} nilai berubah.
                    Cadangan: {deployResult.backup}. Tab Action Studio: tekan Reload.
                  </p>
                )}
              </div>
            )}

            {/* Riwayat */}
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
                      <th className="font-semibold pr-2">Bola</th>
                      <th className="font-semibold pr-2 text-right">Jarak</th>
                      <th className="font-semibold pr-2 text-right" title="goyang IMU sesudah gerakan">Sesudah</th>
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
                        <td className="pr-2">{labelOf(HIT, t.hit)}</td>
                        <td className="pr-2 text-right">{t.distance_cm ?? "-"}</td>
                        <td className="pr-2 text-right font-mono">{t.metrics?.settle_gyro_dps?.toFixed(1) ?? "-"}</td>
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
