import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import {
  WALKING_KEYS, WALKING_BOOL_FIELDS, parseWalkingNumber, validateWalkingParams,
  parametersEqual, mergeWalkingEdits, applyWalkingDraft,
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

test("only deliberate edits override a fresh runtime snapshot", () => {
  const baseline = walkingParams({ balance_enable: true, p_gain: 36 });
  const current = walkingParams({ balance_enable: false, p_gain: 16 });
  const merged = mergeWalkingEdits({ ...baseline, x_move_amplitude: "0,016" }, baseline, current);
  assert.equal(merged.balance_enable, false);
  assert.equal(merged.p_gain, 16);
  assert.equal(merged.x_move_amplitude, 0.016);
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
