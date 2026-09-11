import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import yaml from "js-yaml";
import {
  WALKING_POSE_REFERENCE, walkingReferenceOffset,
  walkingOffsetToDisplay, walkingOffsetFromDisplay,
} from "../src/walkingPoseReference.js";
import {
  WALKING_KEYS, WALKING_BOOL_FIELDS, validateWalkingParams,
  applyWalkingDraft, checkLegReach,
} from "../src/walkingParams.js";

function nativeParams(overrides = {}) {
  return {
    ...Object.fromEntries(WALKING_KEYS.map((key) => [key, WALKING_BOOL_FIELDS.has(key) ? false : 0])),
    ...WALKING_POSE_REFERENCE.offsets,
    period_time: 0.8, dsp_ratio: 0.58, step_fb_ratio: 0.2,
    init_y_offset: 0.028, z_move_amplitude: 0.037,
    ...overrides,
  };
}

test("the fixed page-2 display reference does not zero timing or extra foot separation", () => {
  assert.ok(Object.isFrozen(WALKING_POSE_REFERENCE.offsets));
  assert.equal(walkingReferenceOffset("init_x_offset"), -0.02);
  assert.equal(walkingReferenceOffset("init_z_offset"), 0.0408);
  assert.equal(walkingReferenceOffset("init_pitch_offset"), -0.07);
  assert.equal(walkingReferenceOffset("hip_pitch_offset"), 0.1396);
  assert.equal(walkingReferenceOffset("init_y_offset"), 0);
  assert.equal(walkingOffsetToDisplay(0.028, walkingReferenceOffset("init_y_offset")), 0.028);
  for (const key of ["period_time", "pelvis_offset", "x_move_amplitude", "p_gain", "balance_enable"]) {
    assert.equal(walkingReferenceOffset(key), null, key);
  }
  assert.equal(walkingReferenceOffset("init_x_offset", false), null);
  assert.equal(walkingReferenceOffset("toString"), null);
});

test("zero means the page-2 reference value, not a zero-valued ROS pose", () => {
  for (const [key, offset] of Object.entries(WALKING_POSE_REFERENCE.offsets)) {
    assert.equal(walkingOffsetToDisplay(offset, offset), 0, key);
    assert.equal(Number(walkingOffsetFromDisplay("0", offset)), offset, key);
    assert.equal(walkingOffsetToDisplay(Math.fround(offset), offset), 0, key);
  }
  const params = nativeParams(Object.fromEntries(Object.entries(WALKING_POSE_REFERENCE.offsets)
    .map(([key, offset]) => [key, walkingOffsetFromDisplay("0", offset)])));
  assert.doesNotThrow(() => checkLegReach(validateWalkingParams(params)));
});

test("native presets display as deltas without mutating their numbers", () => {
  const native = Object.freeze(nativeParams({ init_x_offset: -0.03, init_pitch_offset: -0.12, hip_pitch_offset: 0.14 }));
  const before = JSON.stringify(native);
  const displayed = Object.fromEntries(Object.entries(native)
    .map(([key, value]) => [key, walkingOffsetToDisplay(value, walkingReferenceOffset(key))]));
  assert.equal(displayed.init_x_offset, -0.01);
  assert.equal(displayed.init_pitch_offset, -0.05);
  assert.equal(displayed.init_z_offset, 0);
  assert.equal(displayed.init_y_offset, 0.028);
  assert.equal(displayed.hip_pitch_offset, 0.0004);
  assert.equal(displayed.period_time, 0.8);
  assert.equal(displayed.z_move_amplitude, 0.037);
  assert.equal(JSON.stringify(native), before);
  assert.deepEqual(validateWalkingParams(native), native);
});

test("all six demo presets round-trip in ROS float32 without a storage migration", () => {
  const document = yaml.load(readFileSync(new URL("../../../ROBOTIS-OP3-Demo/op3_demo/config/preset_walking.yaml", import.meta.url), "utf8"));
  const snapshot = JSON.stringify(document);
  assert.equal(Object.keys(document).length, 6);
  for (const [name, preset] of Object.entries(document)) {
    for (const [key, value] of Object.entries(preset)) {
      const offset = walkingReferenceOffset(key);
      const display = walkingOffsetToDisplay(value, offset);
      const restored = walkingOffsetFromDisplay(display, offset);
      if (offset === null) assert.equal(restored, value, `${name}.${key}`);
      else assert.equal(Math.fround(restored), Math.fround(value), `${name}.${key}`);
    }
  }
  assert.equal(JSON.stringify(document), snapshot);
});

test("decimal commas, signed offsets and float32 readback stay consistent", () => {
  assert.equal(walkingOffsetFromDisplay("-0,01", -0.02), -0.03);
  assert.equal(walkingOffsetFromDisplay("0,01", -0.02), -0.01);
  assert.equal(walkingOffsetFromDisplay("0,02", 0.0408), 0.0608);
  assert.equal(walkingOffsetToDisplay(Math.fround(-0.03), -0.02), -0.01);
  for (const offset of Object.values(WALKING_POSE_REFERENCE.offsets)) {
    for (let step = -2000; step <= 2000; step += 1) {
      const native = Math.fround(step / 10000);
      const display = walkingOffsetToDisplay(native, offset);
      assert.equal(Math.fround(walkingOffsetFromDisplay(display, offset)), native);
    }
  }
});

test("blank and incomplete offsets remain invalid rather than becoming the reference", () => {
  for (const entered of ["", " ", "-", ".", "-.", "1e", "NaN", "Infinity"]) {
    const converted = walkingOffsetFromDisplay(entered, -0.02);
    assert.equal(converted, entered);
    assert.equal(walkingOffsetToDisplay(converted, -0.02), entered);
    assert.throws(() => validateWalkingParams(nativeParams({ init_x_offset: converted })));
  }
  assert.throws(() => validateWalkingParams(nativeParams({ init_x_offset: walkingOffsetFromDisplay("1e100", -0.02) })));
});

test("changing display mode leaves all native values and non-pose inputs intact", () => {
  const native = Object.freeze(nativeParams({ init_x_offset: -0.03 }));
  const before = JSON.stringify(native);
  for (const relative of [true, false, true, false]) {
    for (const [key, value] of Object.entries(native)) {
      const offset = walkingReferenceOffset(key, relative);
      const displayed = walkingOffsetToDisplay(value, offset);
      if (!relative) assert.equal(displayed, value);
    }
  }
  for (const value of [true, false, "0,8", "0.", 36, "-"]) {
    assert.equal(walkingOffsetToDisplay(value, null), value);
    assert.equal(walkingOffsetFromDisplay(value, null), value);
  }
  assert.equal(JSON.stringify(native), before);
});

test("Apply sends native values, not page-2 deltas, and retains untouched fields", async () => {
  const baseline = nativeParams();
  const draft = { ...baseline, init_x_offset: walkingOffsetFromDisplay("-0.01", -0.02) };
  const reads = [baseline, draft];
  const published = [];
  const result = await applyWalkingDraft({
    draft, baseline, read: async () => reads.shift(), publish: (payload) => published.push(payload),
    check: () => {}, onRead: () => {}, wait: async () => {},
  });
  assert.deepEqual(published, [{ ...baseline, init_x_offset: -0.03 }]);
  assert.equal(result.init_z_offset, 0.0408);
  assert.equal(result.init_pitch_offset, -0.07);
  assert.equal(result.period_time, 0.8);
});

test("viewing a preset then applying without editing does not publish changed offsets", async () => {
  const baseline = nativeParams({ init_x_offset: -0.03 });
  for (const [key, value] of Object.entries(baseline)) walkingOffsetToDisplay(value, walkingReferenceOffset(key));
  let published = false;
  const result = await applyWalkingDraft({
    draft: baseline, baseline, read: async () => baseline, publish: () => { published = true; },
    check: () => {}, onRead: () => {}, wait: async () => {},
  });
  assert.equal(published, false);
  assert.deepEqual(result, baseline);
});

test("loading a saved preset checks connection state in the App scope", () => {
  const source = readFileSync(new URL("../src/App.jsx", import.meta.url), "utf8");
  const loader = source.slice(source.indexOf("  const loadWalkingVersion ="), source.indexOf("  const deleteWalkingVersion ="));
  assert.match(loader, /if \(rosState !== "connected"\)/);
  assert.doesNotMatch(loader, /\brosConnected\b/);
  assert.match(loader, /setWalkingParams\(normalizeWalkingParams\(selected.params\)\)/);
});
