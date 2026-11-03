import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import {
  WALKING_KEYS, WALKING_BOOL_FIELDS, WALKING_INT_FIELDS, parseWalkingNumber,
  validateWalkingParams, parametersEqual, mergeWalkingEdits, applyWalkingDraft,
  snapFloat32, snapWalkingRuntime, legReachNeeded, checkLegReach, LEG_REACH_M,
} from "../src/walkingParams.js";

function walkingParams(overrides = {}) {
  return {
    ...Object.fromEntries(WALKING_KEYS.map((key) => [key, WALKING_BOOL_FIELDS.has(key) ? false : 0])),
    period_time: 0.8, dsp_ratio: 0.4, step_fb_ratio: 0.25, soft_start_time: 2,
    // init_z_offset 0 berarti kaki diminta lurus penuh 220,2 mm, dan satu
    // langkah saja sudah melewati jangkauan -- Apply memang menolaknya sekarang
    // (checkLegReach). Bawaan uji dibuat sedalam preset asli supaya setiap uji
    // di berkas ini menguji maksudnya sendiri, bukan tersandung pose mustahil.
    init_z_offset: 0.0408,
    ...overrides,
  };
}

test("walking schema matches the actual ROS message", () => {
  const message = readFileSync(new URL("../../../ROBOTIS-OP3-msgs/op3_walking_module_msgs/msg/WalkingParam.msg", import.meta.url), "utf8");
  const keys = [...message.matchAll(/^(?:float32|int32|bool)\s+(\w+)/gm)].map((match) => match[1]);
  assert.deepEqual([...WALKING_KEYS].sort(), keys.sort());
});

test("decimal comma is accepted without turning blank inputs into zero", () => {
  assert.equal(parseWalkingNumber(" -0,025 ", NaN), -0.025);
  for (const value of ["", " ", "-", ".", "1e", "NaN", "Infinity", null]) {
    assert.ok(Number.isNaN(parseWalkingNumber(value, NaN)));
    assert.throws(() => validateWalkingParams(walkingParams({ period_time: value })));
  }
});

test("walking rejects singular timing, malformed flags, fractional gains and missing ABI fields", () => {
  for (const invalid of [
    { period_time: 0 }, { period_time: -1 }, { dsp_ratio: 1 }, { dsp_ratio: -0.1 },
    { step_fb_ratio: 1.1 }, { soft_start_time: -1 }, { z_move_amplitude: -0.1 },
    { balance_enable: "false" }, { p_gain: 1.5 }, { p_gain: -1 },
    { angle_move_amplitude_turn_in: undefined }, { init_x_offset: 1e100 },
  ]) assert.throws(() => validateWalkingParams(walkingParams(invalid)));
  assert.equal(validateWalkingParams(walkingParams({ init_x_offset: "-0,03" })).init_x_offset, -0.03);
});

test("float32 readback does not leave a false dirty marker", () => {
  const expected = walkingParams({ x_move_amplitude: 0.016 });
  const observed = walkingParams({ x_move_amplitude: Math.fround(0.016), period_time: Math.fround(0.8) });
  assert.ok(parametersEqual(expected, observed, WALKING_KEYS));
  assert.ok(!parametersEqual({ ...expected, x_move_amplitude: "" }, observed, WALKING_KEYS));
  assert.ok(!parametersEqual({ ...expected, x_move_amplitude: 0.017 }, observed, WALKING_KEYS));
});

test("Apply sends the whole draft, not just the fields that differ", () => {
  const baseline = walkingParams({ balance_enable: true, p_gain: 36 });
  const current = walkingParams({ balance_enable: false, p_gain: 16 });
  const merged = mergeWalkingEdits({ ...baseline, x_move_amplitude: "0,016" }, baseline, current);
  assert.equal(merged.balance_enable, true, "yang dikirim isi kotak isian, bukan nilai robot");
  assert.equal(merged.p_gain, 36);
  assert.equal(merged.x_move_amplitude, 0.016, "koma desimal tetap diterima");
});

// Regresi yang terukur di robot 2026-09-08: walking_module menolkan amplitudo
// tiap Stop, jadi nilai robot bergeser menjauh dari kotak isian TANPA operator
// mengubah apa pun. Kalau Apply cuma mengirim selisih terhadap baseline, panjang
// langkah tidak pernah dikembalikan dan robot berjalan dengan 0.
test("a runtime that drifted to zero is overwritten by the draft", () => {
  const baseline = walkingParams({ x_move_amplitude: 0.016 });
  const draft = { ...baseline };                       // operator tidak mengubah apa pun
  const current = walkingParams({ x_move_amplitude: 0 });   // robot dinolkan oleh Stop
  const merged = mergeWalkingEdits(draft, baseline, current);
  assert.equal(merged.x_move_amplitude, 0.016,
    "panjang langkah harus dikirim ulang walaupun draft sama dengan baseline");
});

test("walking Apply waits for actual readback instead of echoing the payload", async () => {
  const baseline = walkingParams();
  const draft = { ...baseline, x_move_amplitude: 0.016 };
  const reads = [baseline, baseline, { ...draft, x_move_amplitude: Math.fround(0.016) }];
  const published = [];
  const observed = [];
  const result = await applyWalkingDraft({
    draft, baseline, read: async () => reads.shift(), publish: (payload) => published.push(payload),
    check: () => {}, onRead: (params) => observed.push(params), wait: async () => {},
  });
  assert.equal(published.length, 1);
  assert.equal(observed.length, 3);
  assert.equal(result.x_move_amplitude, Math.fround(0.016));
});

test("invalid or cancelled Apply never publishes", async () => {
  const baseline = walkingParams();
  let published = 0;
  const options = {
    draft: { ...baseline, x_move_amplitude: 0.01 }, baseline, read: async () => baseline,
    publish: () => { published += 1; }, check: () => {}, onRead: () => {}, wait: async () => {},
  };
  await assert.rejects(applyWalkingDraft({ ...options, draft: { ...baseline, period_time: "" } }));
  let checks = 0;
  await assert.rejects(applyWalkingDraft({ ...options, check: () => { if (++checks === 2) throw new Error("Stop"); } }), /Stop/);
  assert.equal(published, 0);
});

test("a competing walking writer produces an error, not false success", async () => {
  const baseline = walkingParams();
  await assert.rejects(applyWalkingDraft({
    draft: { ...baseline, x_move_amplitude: 0.01 }, baseline,
    read: async () => baseline, publish: () => {}, check: () => {}, onRead: () => {}, wait: async () => {},
  }), /belum terkonfirmasi/);
});






// WalkingParam bertipe float32. Yang dikirim 0.028 kembali sebagai
// 0.02800000086426735, dan applyWalkingDraft() menuliskan hasil baca ulang itu
// kembali ke kotak isian -- jadi tiap Apply membuat setelan terlihat berubah
// sendiri. Yang dirapikan hanya penulisannya; float32-nya harus tetap sama.
test("float32 read-back is snapped for display without changing the value", () => {
  const asFloat32 = (value) => Math.fround(value);
  for (const typed of [-0.03, 0.028, 0.0408, -0.12, 0.77, 0.4, 0.05, 0.016, 0.037,
                       0.35, 0.7, 0.9, 0.0021, 0.14, 0]) {
    const readBack = asFloat32(typed);
    const snapped = snapFloat32(readBack);
    assert.equal(snapped, typed, `${typed} seharusnya kembali rapi`);
    assert.equal(Math.fround(snapped), readBack, `${typed} tidak boleh berubah nilainya`);
  }
});

test("snapWalkingRuntime leaves booleans and integer gains alone", () => {
  const runtime = walkingParams({
    init_x_offset: Math.fround(-0.03), period_time: Math.fround(0.77),
    balance_enable: true, move_aim_on: false, p_gain: 36, i_gain: 0, d_gain: 0,
  });
  const snapped = snapWalkingRuntime(runtime);
  assert.equal(snapped.init_x_offset, -0.03);
  assert.equal(snapped.period_time, 0.77);
  assert.equal(snapped.balance_enable, true);
  assert.equal(snapped.move_aim_on, false);
  for (const key of WALKING_INT_FIELDS) assert.equal(snapped[key], runtime[key]);
  assert.deepEqual(Object.keys(snapped).sort(), Object.keys(runtime).sort());
});

// Yang sebenarnya dikeluhkan operator: Apply mengembalikan hasil BACA ULANG, dan
// itu yang mendarat di kotak isian. Sesudah dirapikan, Apply tanpa perubahan
// apa pun harus mengembalikan angka yang sama persis dengan yang diketik.
test("apply without edits does not rewrite the operator's numbers", async () => {
  const typed = walkingParams({ init_x_offset: -0.03, period_time: 0.77, y_swap_amplitude: 0.0021 });
  const robot = () => snapWalkingRuntime(Object.fromEntries(
    Object.entries(typed).map(([key, value]) =>
      [key, typeof value === "number" && !WALKING_INT_FIELDS.has(key) ? Math.fround(value) : value])));
  const result = await applyWalkingDraft({
    draft: typed, baseline: typed, read: async () => robot(),
    publish: () => assert.fail("tidak ada yang berubah, seharusnya tidak menerbitkan apa pun"),
    check: () => {}, onRead: () => {}, wait: async () => {},
  });
  assert.equal(result.init_x_offset, -0.03);
  assert.equal(result.period_time, 0.77);
  assert.equal(result.y_swap_amplitude, 0.0021);
});

// 2026-09-09: operator minta bisa langsung memakai versi tersimpan tanpa harus
// menekan Load di baris PARAMETER lebih dulu. Itu aman karena baseline sudah
// tidak menentukan isi yang diterbitkan (mergeWalkingEdits mengirim seluruh
// draft) dan applyWalkingDraft membaca robot sendiri sebelum menerbitkan.
test("Apply works with no prior robot read, using the draft as its own baseline", async () => {
  const versi = walkingParams({ x_move_amplitude: 0.016, init_pitch_offset: -0.12 });
  const runtime = walkingParams({ x_move_amplitude: 0, init_pitch_offset: 0 });
  const dikirim = [];
  let dibaca = 0;
  const hasil = await applyWalkingDraft({
    draft: versi,
    baseline: versi,                       // tidak ada pembacaan sebelumnya
    read: async () => (dibaca++ === 0 ? runtime : { ...versi }),
    publish: (payload) => dikirim.push(payload),
    check: () => {}, onRead: () => {}, wait: async () => {},
  });
  assert.equal(dikirim.length, 1, "tetap menerbitkan walaupun belum pernah Load");
  assert.equal(dikirim[0].x_move_amplitude, 0.016, "isi versi tersimpan yang dikirim");
  assert.equal(dikirim[0].init_pitch_offset, -0.12);
  assert.equal(hasil.x_move_amplitude, 0.016);
  assert.ok(dibaca >= 2, "robot tetap dibaca sebelum dan sesudah menerbitkan");
});

// TERJADI DI ROBOT 2026-09-09. Versi tersimpan "JALAN MENUJU MEKKAH" berisi
// init_x_offset 0,3 m. Kaki diminta menjangkau ~300 mm padahal paha + betis
// cuma 220,2 mm, jadi IK gagal SETIAP siklus dan walking_module melewati
// seluruh siklus gait -- Apply & Start tampak sukses tapi robot diam total.
const poseKaki = (ubah = {}) => walkingParams({
  init_x_offset: -0.018, init_y_offset: 0.028, init_z_offset: 0.0408,
  x_move_amplitude: 0.016, y_move_amplitude: 0, ...ubah,
});

test("Apply refuses an init pose the leg cannot physically reach", async () => {
  let published = 0;
  const jalankan = (draft) => applyWalkingDraft({
    draft, baseline: poseKaki(), read: async () => draft,
    publish: () => { published += 1; }, check: () => {}, onRead: () => {}, wait: async () => {},
  });
  await jalankan(poseKaki());                       // terjangkau: lolos
  for (const mustahil of [
    { init_x_offset: 0.3 }, { init_x_offset: -0.3 },  // nilai yang benar-benar terjadi
    { init_x_offset: -0.18 },                         // "kelebihan satu angka nol"
    { init_z_offset: -0.2 },                          // jongkok lebih dalam dari jangkauan
  ]) {
    await assert.rejects(() => jalankan(poseKaki(mustahil)), /Kaki tidak sampai/,
      JSON.stringify(mustahil));
  }
  assert.equal(published, 0, "yang terjangkau pun tidak perlu terbit: draft == runtime");
});

// Penjaga ini HANYA untuk draft yang akan dikirim. Kalau ia dipasang di
// validateWalkingParams, pembacaan dari robot ikut ditolak -- dan saat robot
// sedang memegang setelan mustahil (persis keadaan 9 Sep), operator tidak bisa
// Load, tidak bisa Apply, tidak punya jalan keluar sama sekali.
test("a robot already holding an unreachable pose can still be read and fixed", async () => {
  const rusak = poseKaki({ init_x_offset: 0.3 });
  assert.doesNotThrow(() => validateWalkingParams(rusak), "pembacaan robot tidak boleh ditolak");
  const published = [];
  const sehat = poseKaki();
  const reads = [rusak, { ...sehat }];
  const hasil = await applyWalkingDraft({
    draft: sehat, baseline: rusak, read: async () => reads.shift(),
    publish: (p) => published.push(p), check: () => {}, onRead: () => {}, wait: async () => {},
  });
  assert.equal(published.length, 1, "draft sehat tetap bisa dikirim untuk memperbaiki");
  assert.equal(hasil.init_x_offset, -0.018);
});

test("leg reach accounts for a full step, not just the standing pose", () => {
  const diam = poseKaki({ init_x_offset: 0.125, x_move_amplitude: 0 });
  assert.ok(legReachNeeded(diam) <= LEG_REACH_M, "berdiri saja: masih sampai");
  assert.doesNotThrow(() => checkLegReach(diam));
  const melangkah = poseKaki({ init_x_offset: 0.125, x_move_amplitude: 0.02 });
  assert.ok(legReachNeeded(melangkah) > LEG_REACH_M, "dengan langkah penuh: tidak sampai");
  assert.throws(() => checkLegReach(melangkah), /Kaki tidak sampai/);
});
