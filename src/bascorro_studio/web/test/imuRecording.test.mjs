import assert from "node:assert/strict";
import test from "node:test";
import { tiltDeg, turnSummary, turnText } from "../src/imuRecording.js";

// 1 s diam (bias gyro z 0,01 rad/s), 5 s berputar ke kanan 0,3 rad/s sambil
// bergoyang, 1 s melambat sampai diam; 125 Hz.
function recording({ turn = -0.3, bias = 0.01, pitchDeg = 7 } = {}) {
  const half = (pitchDeg * Math.PI) / 360;
  const samples = [];
  for (let i = 0; i <= 7 * 125; i += 1) {
    const t = 100 + i / 125;
    const walking = t >= 101 && t <= 106;
    const slowing = t > 106 ? Math.max(0, 1 - (t - 106)) : 0;
    const gz = bias + (walking ? turn : turn * slowing);
    const gx = walking ? 0.2 * Math.sin(2 * Math.PI * 1.7 * t) : 0;
    samples.push([t, gx, 0, gz, 0, Math.sin(half), 0, Math.cos(half)]);
  }
  return samples;
}

test("putaran = integral gyro z dikurangi bias diam", () => {
  const out = turnSummary(recording(), 101, 106, 107);
  assert.ok(Math.abs(out.turn_dps - (-0.3 * 180) / Math.PI) < 0.2, `turn_dps ${out.turn_dps}`);
  // total ikut fase melambat: 5 s x 0,3 + 0,5 s x 0,3 (rata-rata separuh)
  assert.ok(Math.abs(out.turn_deg - (-1.65 * 180) / Math.PI) < 1.5, `turn_deg ${out.turn_deg}`);
  assert.ok(out.wobble_dps > 5 && out.wobble_dps < 10, `wobble ${out.wobble_dps}`);
  assert.equal(out.tilt_max_deg, 0);
  assert.equal(out.bias_still, true);
});

test("rekaman jalan terlalu pendek -> null", () => {
  assert.equal(turnSummary(recording(), 101, 101.02, 101.5), null);
  assert.equal(turnSummary([], 1, 2, 3), null);
});

test("tilt dan teks arah", () => {
  const half = (10 * Math.PI) / 360;
  assert.ok(Math.abs(tiltDeg([0, 0, 0, 0, 0, Math.sin(half), 0, Math.cos(half)]).pitch - 10) < 1e-9);
  assert.equal(turnText(-23.44), "kanan 23.4°/s");
  assert.equal(turnText(12), "kiri 12.0°/s");
  assert.equal(turnText(0.4), "lurus 0.4°/s");
  assert.equal(turnText(null), "-");
});
