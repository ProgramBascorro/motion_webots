import assert from "node:assert/strict";
import test from "node:test";
import {
  BRAKE_PAGE, SCRATCH_PAGE, TrialAborted, WALKING_READY_PAGE, isLying, runActionTrial,
} from "../src/training/actionTrial.js";

// Robot tiruan + jam tiruan: wait(ms) memajukan jam dan menerbitkan sampel IMU
// 125 Hz dengan pitch = pitchAt(t). Page "berjalan" selama durasinya sejak diputar.
function simulate({ pitchAt = () => 0, durations = { [WALKING_READY_PAGE]: 1.6, [SCRATCH_PAGE]: 2.0 },
  owned = true, refuse = [], abortAt = Infinity } = {}) {
  let t = 0;
  let playing = null;
  const log = [];
  let listener = null;
  const robot = {
    prepare: () => log.push("prepare"),
    useActionModule: () => log.push("action_module"),
    legsOwnedBy: async () => owned,
    playPage: (n) => {
      log.push(`play ${n}`);
      if (!refuse.includes(n)) playing = { n, until: t + (durations[n] ?? 1) };
    },
    isRunning: async () => Boolean(playing && t < playing.until),
    brake: () => { log.push(`play ${BRAKE_PAGE}`); playing = null; },
  };
  const half = (deg) => (deg * Math.PI) / 360;
  const startRecording = (onSample) => {
    const rec = { samples: [], stopped: false };
    listener = (sample) => { if (!rec.stopped) { rec.samples.push(sample); onSample(sample); } };
    rec.last = () => rec.samples.at(-1)?.[0] ?? 0;
    rec.stop = () => { rec.stopped = true; };
    return rec;
  };
  const wait = async (ms) => {
    const end = t + ms / 1000;
    while (t < end - 1e-9) {
      t = Math.min(end, t + 1 / 125);
      const p = pitchAt(t);
      listener?.([Number(t.toFixed(4)), 0, 0, 0, 0, Math.sin(half(p)), 0, Math.cos(half(p))]);
    }
    return t < abortAt;
  };
  const now = () => t * 1000;
  return { robot, startRecording, wait, now, log, time: () => t };
}

test("percobaan normal: page 2 -> diam -> 254 -> sesudahnya", async () => {
  const sim = simulate();
  const stages = [];
  const out = await runActionTrial({ ...sim, plannedS: 2.0, onStage: (s) => stages.push(s) });
  assert.deepEqual(sim.log, ["prepare", "action_module", `play ${WALKING_READY_PAGE}`, `play ${SCRATCH_PAGE}`]);
  const { stand0, start, done, end } = out.marks;
  assert.ok(start - stand0 >= 0.95, "rekaman diam ~1 s");
  assert.ok(Math.abs(done - start - 2.0) < 0.15, `durasi page ${done - start}`);
  assert.ok(Math.abs(end - done - 1.5) < 0.02);
  assert.equal(out.braked, false);
  assert.equal(out.timedOut, false);
  assert.ok(stages.includes(`memutar page ${SCRATCH_PAGE}`));
});

test("badan miring > 50 deg saat page diputar: direm otomatis", async () => {
  // pose siap tegak; jatuh ke depan mulai 0,5 s sesudah 254 diputar
  let fallFrom = null;
  const sim = simulate({
    pitchAt: (t) => (fallFrom !== null && t > fallFrom ? Math.min(90, (t - fallFrom) * 150) : 0),
    durations: { [WALKING_READY_PAGE]: 1.6, [SCRATCH_PAGE]: 3.0 },
  });
  const play = sim.robot.playPage;
  sim.robot.playPage = (n) => { if (n === SCRATCH_PAGE) fallFrom = sim.time() + 0.5; play(n); };
  const out = await runActionTrial({ ...sim, plannedS: 3.0 });
  assert.equal(out.braked, true);
  assert.ok(sim.log.includes(`play ${BRAKE_PAGE}`));
  assert.ok(out.marks.done - out.marks.start < 1.5, "berhenti jauh sebelum page selesai");
});

test("page bangun: tanpa page 2, wajib berbaring, tanpa rem jatuh", async () => {
  const lyingSim = simulate({ pitchAt: (t) => (t < 2 ? 88 : 5) });
  const out = await runActionTrial({ ...lyingSim, plannedS: 2.0, lying: true });
  assert.ok(!lyingSim.log.includes(`play ${WALKING_READY_PAGE}`));
  assert.equal(out.braked, false);

  const standing = simulate();
  await assert.rejects(runActionTrial({ ...standing, plannedS: 2.0, lying: true }), /belum berbaring/);
  assert.ok(!standing.log.includes(`play ${SCRATCH_PAGE}`), "page bangun tidak diputar saat berdiri");
});

test("STOP operator menghentikan percobaan", async () => {
  const sim = simulate({ abortAt: 2.5 });
  await assert.rejects(runActionTrial({ ...sim, plannedS: 2.0 }), (e) => e instanceof TrialAborted);
});

test("kaki masih dipegang walking / page ditolak: galat jelas, kandidat tidak diputar", async () => {
  const walking = simulate({ owned: false });
  await assert.rejects(runActionTrial({ ...walking, plannedS: 2.0 }), /Stop walking dulu/);
  assert.ok(!walking.log.includes(`play ${WALKING_READY_PAGE}`));

  const refused = simulate({ refuse: [SCRATCH_PAGE] });
  await assert.rejects(runActionTrial({ ...refused, plannedS: 2.0 }), /tidak memutar page 254/);
});

test("isLying", () => {
  const sample = (deg) => [0, 0, 0, 0, 0, Math.sin((deg * Math.PI) / 360), 0, Math.cos((deg * Math.PI) / 360)];
  assert.equal(isLying([sample(85), sample(80)]), true);
  assert.equal(isLying([sample(10)]), false);
  assert.equal(isLying([]), false);
});
