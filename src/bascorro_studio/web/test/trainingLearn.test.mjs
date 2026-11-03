import assert from "node:assert/strict";
import test from "node:test";
import { proposeCandidate, randn } from "../src/trainingLearn.js";

// Generator acak yang bisa diulang (LCG), supaya hasil tiap uji tetap sama.
function seeded(seed = 1) {
  let state = seed;
  return () => {
    state = (state * 1664525 + 1013904223) % 4294967296;
    return state / 4294967296;
  };
}

const PARAMS = [{ key: "x", lo: -10, hi: 10, step: 0.5 }];
const scoreOf = (t) => t.score;
const isGood = (t) => t.score >= 3;
const isBad = (t) => t.score < 0;
const isSolid = (t) => t.score >= 5;
const valueOf = (t, key) => t.values[key];

function propose(history, options = {}) {
  return proposeCandidate({
    params: PARAMS, history, mode: "learn", scoreOf, isGood, isBad, isSolid, valueOf, random: seeded(), ...options,
  });
}

test("tanpa percobaan baik: acak penuh di dalam rentang, dibulatkan ke step", () => {
  for (let seed = 1; seed < 50; seed += 1) {
    const out = propose([{ no: 1, score: -1, values: { x: 3 } }], { random: seeded(seed) });
    assert.equal(out.learn, false);
    assert.ok(out.varied.x >= -10 && out.varied.x <= 10);
    assert.ok(Number.isInteger(out.varied.x * 2), `${out.varied.x} bukan kelipatan 0,5`);
  }
  assert.equal(propose([]).basis, "eksplorasi: belum ada percobaan yang baik");
  assert.equal(propose([], { mode: "random" }).basis, "acak penuh dalam rentang");
});

test("belajar: pusat di elit, sebaran mengecil setelah banyak hasil kokoh", () => {
  const history = [5, 5, 5, 5, 5, 5].map((score, i) => ({ no: 6 - i, score, values: { x: 6 } }));
  const values = [];
  for (let seed = 1; seed <= 200; seed += 1) values.push(propose(history, { random: seeded(seed) }).varied.x);
  const mean = values.reduce((s, x) => s + x, 0) / values.length;
  assert.ok(Math.abs(mean - 6) < 0.6, `rata-rata ${mean}`);
  // sebaran dijepit di lantai rentang/10 = 2; hampir semua sampel dalam 3 sigma
  assert.ok(values.every((x) => Math.abs(x - 6) <= 6.5));
  assert.match(propose(history).basis, /^pusat dari #6, #5, #4/);
});

test("tiga percobaan terakhir gagal: sebaran dilebarkan", () => {
  const history = [
    { no: 4, score: -5, values: { x: 0 } }, { no: 3, score: -5, values: { x: 0 } },
    { no: 2, score: -5, values: { x: 0 } }, { no: 1, score: 3, values: { x: 2 } },
  ];
  assert.match(propose(history).basis, /dilebarkan/);
});

test("randn memakai generator yang diberikan", () => {
  assert.equal(randn(seeded(7)), randn(seeded(7)));
});
