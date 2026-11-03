import assert from "node:assert/strict";
import test from "node:test";
import { draftReloadDecision, syncedRecord, textHash } from "../src/actionDraftSync.js";

const BIN = "/ros2_ws/src/ROBOTIS-OP3/op3_action_module/data/ALPHONSE.bin";
const DRAFT = "pages:\n  - index: 2\n    name: WALKING_READY\n";

test("draft yang belum diedit diganti saat .bin diubah di luar Studio", () => {
  const synced = syncedRecord(DRAFT, BIN, 1000);
  assert.equal(draftReloadDecision({ draftText: DRAFT, hasDraft: true, binPath: BIN, binMtimeMs: 90000, synced }), "reload");
});

test("draft yang SUDAH diedit tidak pernah ditimpa", () => {
  const synced = syncedRecord(DRAFT, BIN, 1000);
  const edited = DRAFT.replace("WALKING_READY", "WALKING_READY_baru");
  assert.equal(draftReloadDecision({ draftText: edited, hasDraft: true, binPath: BIN, binMtimeMs: 90000, synced }), "edited");
});

test("tidak ada yang dimuat ulang kalau .bin tidak lebih baru", () => {
  const synced = syncedRecord(DRAFT, BIN, 1000);
  assert.equal(draftReloadDecision({ draftText: DRAFT, hasDraft: true, binPath: BIN, binMtimeMs: 2500, synced }), "keep");
});

test("draft lama tanpa catatan sinkron, atau untuk .bin lain, dibiarkan", () => {
  assert.equal(draftReloadDecision({ draftText: DRAFT, hasDraft: true, binPath: BIN, binMtimeMs: 90000, synced: null }), "keep");
  const other = syncedRecord(DRAFT, "/x/CHRONUS.bin", 1000);
  assert.equal(draftReloadDecision({ draftText: DRAFT, hasDraft: true, binPath: BIN, binMtimeMs: 90000, synced: other }), "keep");
});

test("editor kosong langsung diisi dari .bin; tanpa waktu .bin menunggu", () => {
  assert.equal(draftReloadDecision({ draftText: "", hasDraft: false, binPath: BIN, binMtimeMs: 5, synced: null }), "load");
  assert.equal(draftReloadDecision({ draftText: DRAFT, hasDraft: true, binPath: BIN, binMtimeMs: null, synced: null }), "wait");
});

test("sidik berubah kalau satu karakter berubah", () => {
  assert.notEqual(textHash(DRAFT), textHash(DRAFT.replace("2", "3")));
  assert.equal(textHash(DRAFT), textHash(String(DRAFT)));
});
