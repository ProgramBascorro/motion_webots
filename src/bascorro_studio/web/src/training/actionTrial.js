// Urutan SATU percobaan tab Action Training -- MURNI: robot, perekam IMU, dan
// jam disuntikkan, jadi urutannya diuji tanpa robot (web/test/actionTrial.test.mjs).
// Adaptor ROS-nya ada di actionRobot.js.
//
//   batas kecepatan servo dilepas (Torque ON "Lambat" masih memasangnya)
//   -> action_module memegang sendi (dicek lewat layanan, bukan ditebak)
//   -> page 2 WALKING_READY sampai selesai          (bukan untuk page bangun)
//   -> rekam IMU diam 1 s = acuan metrik            (page bangun: wajib berbaring)
//   -> page kerja 254 (kandidat) diputar sampai action_module selesai,
//      direm kalau badan miring > 50 deg dari posisi siap atau melewati batas waktu
//   -> rekam 1,5 s lagi, lalu metrik dihitung apply_node (act_metrics).
//
// Diadaptasi dari runActionTrial ALPHONSE_NEW_V3. Di sana urutan ini bergantung pada
// runRobotOperation milik App.jsx ALPHONSE, yang tidak ada di CHRONUS; di sini ia
// berdiri sendiri dan App.jsx cukup memberi prepare() (lepas batas kecepatan +
// tandai walking_module tidak aktif).

import { tiltDeg } from "./imuMetrics.js";

export const SCRATCH_PAGE = 254;
export const WALKING_READY_PAGE = 2;
export const BRAKE_PAGE = -2;          // action_module: hentikan seketika
export const FALL_BRAKE_DEG = 50;      // rem kalau badan miring sejauh ini dari posisi siap
export const LYING_MIN_DEG = 45;       // page bangun hanya dimulai kalau robot memang berbaring
export const MIN_SAMPLES = 10;

export const TIMING = {
  prepareMs: 250,         // restoreFullSpeed: dua sync_write_item berjarak 60 ms
  moduleSettleMs: 600,    // App.jsx ACTION_MODULE_SETTLE_MS
  pageStartMs: 1000,      // is_running tidak pernah true selama ini = page ditolak
  readyTimeoutMs: 6000,   // page 2 (~1,6 s di CHRONUS) harus selesai dalam ini
  standMs: 1000,          // rekaman diam sebelum kandidat
  settleMs: 1500,         // imu_metrics.SETTLE_S
  pollMs: 100,
  maxServiceMisses: 5,    // is_running tidak menjawab sekian kali berturut-turut = berhenti
};

export class TrialAborted extends Error {
  constructor(message = "dihentikan operator") {
    super(message);
    this.name = "TrialAborted";
  }
}

const tiltFrom = (ref, sample) => {
  const t = tiltDeg(sample);
  return Math.hypot(t.roll - ref.roll, t.pitch - ref.pitch);
};

export function isLying(samples, minDeg = LYING_MIN_DEG) {
  if (!samples.length) return false;
  const mean = samples.reduce((sum, s) => {
    const t = tiltDeg(s);
    return sum + Math.max(Math.abs(t.roll), Math.abs(t.pitch));
  }, 0) / samples.length;
  return mean >= minDeg;
}

// robot: { prepare(), useActionModule(), legsOwnedBy(name) -> bool|null, playPage(n),
//          isRunning() -> bool|null (null = layanan tidak menjawab), brake() }
// startRecording(onSample) -> { samples, last(), stop() }
// wait(ms) -> false kalau operator menekan STOP
// Hasil: { samples, marks: {stand0, start, done, end, planned_s}, timedOut, braked }
export async function runActionTrial({
  robot, startRecording, wait, now = () => Date.now(), onStage = () => {},
  lying = false, plannedS, timing = TIMING,
}) {
  const step = async (ms) => {
    if (!(await wait(ms))) throw new TrialAborted();
  };

  // Putar satu page sampai action_module melapor selesai.
  const playToEnd = async (page, timeoutMs, stopEarly = () => false) => {
    robot.playPage(page);
    const begun = now();
    let started = false;
    let misses = 0;
    for (;;) {
      await step(timing.pollMs);
      if (stopEarly()) return { started: true, timedOut: false };
      const running = await robot.isRunning();
      if (running === null) {
        misses += 1;
        if (misses >= timing.maxServiceMisses) {
          robot.brake();
          throw new Error("/robotis/action/is_running tidak menjawab -- page direm");
        }
        continue;
      }
      misses = 0;
      started = started || running;
      const elapsed = now() - begun;
      if (!running && (started || elapsed > timing.pageStartMs)) return { started, timedOut: false };
      if (elapsed > timeoutMs) {
        robot.brake();
        return { started, timedOut: true };
      }
    }
  };

  onStage("melepas batas kecepatan servo");
  robot.prepare();
  await step(timing.prepareMs);
  onStage("pindah ke action_module");
  robot.useActionModule();
  await step(timing.moduleSettleMs);
  const owned = await robot.legsOwnedBy("action_module");
  if (owned === null) throw new Error("/robotis/get_present_joint_ctrl_modules tidak menjawab -- op3_manager hidup?");
  if (!owned) throw new Error("action_module belum memegang kaki -- walking masih berjalan? Stop walking dulu");

  if (!lying) {
    onStage(`page ${WALKING_READY_PAGE} (posisi siap)`);
    const ready = await playToEnd(WALKING_READY_PAGE, timing.readyTimeoutMs);
    if (ready.timedOut) throw new Error(`page ${WALKING_READY_PAGE} belum selesai diputar -- direm`);
    if (!ready.started) throw new Error(`action_module tidak memutar page ${WALKING_READY_PAGE} (lihat /robotis/status)`);
  }

  let ref = null;
  let watchFall = false;
  let braked = false;
  const rec = startRecording((sample) => {
    if (!ref) ref = tiltDeg(sample);
    if (watchFall && !braked && tiltFrom(ref, sample) > FALL_BRAKE_DEG) {
      braked = true;
      robot.brake();
    }
  });
  try {
    onStage("merekam diam");
    await step(timing.standMs);
    if (rec.samples.length < MIN_SAMPLES) throw new Error("IMU tidak terbit (/robotis/open_cr/imu)");
    if (lying && !isLying(rec.samples)) {
      throw new Error(`robot belum berbaring (miring < ${LYING_MIN_DEG}°) -- percobaan bangun dimulai dari berbaring`);
    }
    const marks = { stand0: rec.samples[0][0], start: rec.last(), planned_s: plannedS };
    watchFall = !lying;
    onStage(`memutar page ${SCRATCH_PAGE}`);
    const run = await playToEnd(SCRATCH_PAGE, (1.5 * plannedS + 5) * 1000, () => braked);
    if (!run.started) throw new Error(`action_module tidak memutar page ${SCRATCH_PAGE} (lihat /robotis/status)`);
    marks.done = rec.last();
    onStage("merekam sesudah gerakan");
    await step(timing.settleMs);
    marks.end = rec.last();
    return { samples: rec.samples.slice(), marks, timedOut: run.timedOut, braked };
  } finally {
    rec.stop();
  }
}
