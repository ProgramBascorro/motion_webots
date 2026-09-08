import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import {
  WALKING_KEYS, WALKING_BOOL_FIELDS, WALKING_INT_FIELDS, parseWalkingNumber,
  validateWalkingParams, parametersEqual, mergeWalkingEdits, applyWalkingDraft,
  snapFloat32, snapWalkingRuntime,
} from "../src/walkingParams.js";
import {
  KICK_DEFAULT_PARAMS, KICK_PARAM_KEYS, validateKickParams,
  decodeKickParameters, makeKickParameters,
} from "../src/kickParams.js";

function walkingParams(overrides = {}) {
  return {
    ...Object.fromEntries(WALKING_KEYS.map((key) => [key, WALKING_BOOL_FIELDS.has(key) ? false : 0])),
    period_time: 0.8, dsp_ratio: 0.4, step_fb_ratio: 0.25, soft_start_time: 2,
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

test("kick reference and keys match the current launch", () => {
  const launch = readFileSync(new URL("../../../ROBOTIS-OP3-Demo/op3_demo/launch/demo.launch.xml", import.meta.url), "utf8");
  for (const key of KICK_PARAM_KEYS) {
    const match = launch.match(new RegExp('<arg name="' + key + '" default="([^"]+)"'));
    assert.ok(match, key);
    assert.equal(Number(match[1]), KICK_DEFAULT_PARAMS[key], key);
  }
});

test("kick rejects invalid geometry and values that the controller would silently clamp", () => {
  for (const invalid of [
    { kick_distance: "" }, { kick_distance: 0 }, { kick_camera_height: 0.05 },
    { kick_camera_pitch_offset_deg: -60.01 }, { kick_camera_pitch_offset_deg: 60.01 },
    { kick_camera_pitch_offset_deg: NaN }, { kick_camera_pitch_offset_deg: Infinity },
    { kick_fov_width_deg: 90 }, { kick_ball_radius_margin: 1 }, { kick_head_tilt_sign: 0 },
    { kick_distance_filter_n: 2.5 }, { kick_radius_ready_enable: 0.5 }, { kick_ball_radius_px: -1 },
    { kick_distance_filter_n: 2147483648 },
  ]) assert.throws(() => validateKickParams({ ...KICK_DEFAULT_PARAMS, ...invalid }));
});

test("missing kick parameters never become fake loaded defaults", () => {
  const values = KICK_PARAM_KEYS.map((key) => ({ type: 3, double_value: KICK_DEFAULT_PARAMS[key] }));
  assert.deepEqual(decodeKickParameters({ values }).params, KICK_DEFAULT_PARAMS);
  assert.throws(() => decodeKickParameters({ values: values.slice(1) }));
  assert.throws(() => decodeKickParameters({ values: [{ type: 0 }, ...values.slice(1)] }));
  assert.throws(() => decodeKickParameters({ values: [{ type: 3, double_value: NaN }, ...values.slice(1)] }));
});

test("camera pitch correction preserves its sign and numeric ROS type", () => {
  const types = Object.fromEntries(KICK_PARAM_KEYS.map((key) => [key, 3]));
  for (const offset of [-60, 0, 30.84, 60]) {
    const params = { ...KICK_DEFAULT_PARAMS, kick_camera_pitch_offset_deg: offset };
    assert.equal(validateKickParams(params).kick_camera_pitch_offset_deg, offset);
    const written = makeKickParameters(params, types).find(({ name }) => name === "kick_camera_pitch_offset_deg");
    assert.deepEqual(written.value, { type: 3, double_value: offset });
  }
});

test("kick writes preserve declared double/integer types", () => {
  const types = Object.fromEntries(KICK_PARAM_KEYS.map((key) => [key, 3]));
  types.kick_ready_count = 2;
  const parameters = makeKickParameters(KICK_DEFAULT_PARAMS, types);
  assert.deepEqual(parameters.find(({ name }) => name === "kick_ready_count").value, { type: 2, integer_value: 4 });
  assert.deepEqual(parameters.find(({ name }) => name === "kick_radius_ready_enable").value,
    { type: 3, double_value: KICK_DEFAULT_PARAMS.kick_radius_ready_enable });
  assert.throws(() => makeKickParameters(KICK_DEFAULT_PARAMS, { ...types, kick_distance: 2 }));
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
