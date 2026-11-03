import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import {
  walkingVersionSlotTaken, seedWalkingVersion, parseStoredWalkingVersions,
} from "../src/walkingVersionStore.js";

const SUMBER = new URL("../src/App.jsx", import.meta.url);

// Slot preset yang dipakai App.jsx. Namanya sengaja sama dengan salah satu versi
// tersimpan operator -- itulah yang dulu membuatnya tertimpa.
const JALAN = { id: "jalan-ke-surga-v1", name: "JALAN KE SURGA" };

const versiOperator = (params) => ({
  id: "b7e1-operator", name: "JALAN KE SURGA",
  createdAt: 1, updatedAt: 2, params,
});

test("semaian tidak menimpa versi operator yang namanya kebetulan sama", () => {
  const punyaOperator = versiOperator({ init_x_offset: -0.02, init_pitch_offset: -0.07 });
  const sesudah = seedWalkingVersion([punyaOperator], {
    ...JALAN,
    params: { init_x_offset: -0.03, init_pitch_offset: -0.12 },
    now: 999,
  });

  assert.equal(sesudah.length, 1);
  assert.equal(sesudah[0], punyaOperator, "objeknya harus yang sama persis, bukan salinan");
  assert.equal(sesudah[0].params.init_x_offset, -0.02);
  assert.equal(sesudah[0].params.init_pitch_offset, -0.07);
  assert.equal(sesudah[0].updatedAt, 2, "updatedAt tidak boleh ikut bergeser");
});

test("menaikkan revisi seed berkali-kali tetap tidak bisa merusak data operator", () => {
  // Inilah bentuk kegagalan yang sebenarnya terjadi: penanda seed diturunkan
  // dari JALAN_WALKING_SEED_REV, jadi tiap kenaikan revisi menjalankan semaian
  // lagi pada browser yang SAMA. Di localStorage robot ini tertinggal tiga
  // revisi penanda berdampingan, artinya ini sudah berjalan dua kali.
  let daftar = [versiOperator({ init_x_offset: -0.034 })];
  for (const rev of ["v1", "v2", "2026-09-02", "2026-09-11"]) {
    daftar = seedWalkingVersion(daftar, {
      ...JALAN, params: { init_x_offset: -0.03, rev }, now: 1000,
    });
  }
  assert.equal(daftar.length, 1);
  assert.equal(daftar[0].params.init_x_offset, -0.034, "nilai operator harus bertahan");
});

test("semaian tetap menambah kalau slotnya memang belum ada", () => {
  const sesudah = seedWalkingVersion([], { ...JALAN, params: { init_x_offset: -0.03 }, now: 42 });
  assert.equal(sesudah.length, 1);
  assert.deepEqual(sesudah[0], {
    id: JALAN.id, name: JALAN.name, createdAt: 42, updatedAt: 42,
    params: { init_x_offset: -0.03 },
  });
});

test("slot dianggap terisi lewat id maupun nama", () => {
  assert.equal(walkingVersionSlotTaken([{ id: JALAN.id, name: "lain" }], JALAN.id, JALAN.name), true);
  assert.equal(walkingVersionSlotTaken([{ id: "lain", name: JALAN.name }], JALAN.id, JALAN.name), true);
  assert.equal(walkingVersionSlotTaken([{ id: "lain", name: "lain" }], JALAN.id, JALAN.name), false);
});

test("semaian tidak menyentuh versi tersimpan lain", () => {
  const lain = [{ id: "a", name: "AMBA", params: {} }, { id: "b", name: "MENCARI CELAH", params: {} }];
  const sesudah = seedWalkingVersion(lain, { ...JALAN, params: {}, now: 1 });
  assert.equal(sesudah.length, 3);
  assert.deepEqual(sesudah.slice(1), lain, "yang lama harus utuh dan urutannya tetap");
});

test("penyimpanan kosong bukan kerusakan", () => {
  for (const raw of [null, undefined, ""]) {
    assert.deepEqual(parseStoredWalkingVersions(raw), { versions: [], rusak: null });
  }
});

test("JSON rusak dilaporkan supaya pemanggil menyelamatkannya, bukan diam-diam jadi kosong", () => {
  // Tanpa ini daftar kosong langsung ditulis balik ke kunci yang sama oleh
  // useEffect pemersist, dan seluruh versi tersimpan operator hilang selamanya.
  const raw = '[{"id":"a","params":{}},{"id":"b"';
  assert.deepEqual(parseStoredWalkingVersions(raw), { versions: [], rusak: raw });
});

test("JSON yang bukan array juga dilaporkan rusak", () => {
  const raw = '{"id":"a"}';
  assert.deepEqual(parseStoredWalkingVersions(raw), { versions: [], rusak: raw });
});

test("entri cacat dibuang tapi teks aslinya tetap diselamatkan", () => {
  const raw = JSON.stringify([
    { id: "a", params: { init_x_offset: -0.02 } },
    { id: "b" },                      // tanpa params
    { params: { init_x_offset: 0 } }, // tanpa id
  ]);
  const { versions, rusak } = parseStoredWalkingVersions(raw);
  assert.equal(versions.length, 1);
  assert.equal(versions[0].id, "a");
  assert.equal(rusak, raw, "sisanya tidak boleh hilang tanpa salinan");
});

test("daftar sehat tidak memicu penyelamatan", () => {
  const raw = JSON.stringify([{ id: "a", name: "AMBA", params: { init_x_offset: -0.03 } }]);
  const { versions, rusak } = parseStoredWalkingVersions(raw);
  assert.equal(versions.length, 1);
  assert.equal(rusak, null);
});

test("App.jsx tidak lagi memakai pola 'refresh in place' yang menimpa slot", () => {
  const teks = readFileSync(SUMBER, "utf8");
  assert.equal(teks.includes("refreshed[existing]"), false,
    "penimpaan slot bernama sama sudah dicabut; jangan dipasang lagi");
  assert.match(teks, /rescueWalkingVersions/,
    "jalur baca wajib menyelamatkan teks yang gagal diurai sebelum tertimpa");
});

// ---------------------------------------------------------------------------
// Pemulihan saat startup, Save, dan penanda "belum disimpan".
import {
  findRestorableVersion, planWalkingSave, walkingDraftUnsaved, sameWalkingParams,
} from "../src/walkingVersionStore.js";
import { fetchWalkingStore, mutateWalkingStore, WALKING_STORE_URL } from "../src/walkingVersionServer.js";
import { WALKING_KEYS } from "../src/walkingParams.js";

// Satu set lengkap 29 field; isinya tidak penting di sini, kelengkapannya yang penting.
const penuh = (overrides = {}) => ({
  ...Object.fromEntries(WALKING_KEYS.map((key) => [key,
    key === "move_aim_on" || key === "balance_enable" ? false : key === "p_gain" ? 36 : 0.01])),
  ...overrides,
});

const MEKKAH = { id: "m", name: "JALAN MENUJU MEKKAH", createdAt: 1, updatedAt: 2, params: penuh({ init_pitch_offset: -0.8 }) };
const AMBA = { id: "a", name: "AMBA", createdAt: 1, updatedAt: 2, params: penuh({ init_x_offset: -0.03 }) };
const DEFAULT = penuh({ init_x_offset: -0.02, init_pitch_offset: -0.07 });

test("startup: versi aktif yang tersimpan dipulihkan, BUKAN default", () => {
  assert.equal(findRestorableVersion({ activeVersionId: "m", versions: [AMBA, MEKKAH] }), MEKKAH);
});

test("startup: default hanya dipakai kalau memang tidak ada versi aktif", () => {
  assert.equal(findRestorableVersion({ activeVersionId: null, versions: [AMBA, MEKKAH] }), null);
  assert.equal(findRestorableVersion({ activeVersionId: "sudah-dihapus", versions: [AMBA] }), null);
  assert.equal(findRestorableVersion(null), null);
});

test("Save menulis ke versi yang sedang terbuka, dengan seluruh isian", () => {
  const params = penuh({ init_pitch_offset: -0.81 });
  const { mode, version } = planWalkingSave({
    versions: [AMBA, MEKKAH], activeVersionId: "m", name: "JALAN MENUJU MEKKAH", params, now: 99, makeId: () => "x",
  });
  assert.equal(mode, "update");
  assert.deepEqual(version, { ...MEKKAH, params, updatedAt: 99 });
});

test("Save dengan nama lain membuat versi BARU; versi yang terbuka tidak diganti nama", () => {
  const { mode, version } = planWalkingSave({
    versions: [AMBA, MEKKAH], activeVersionId: "m", name: "MEKKAH 2", params: MEKKAH.params, now: 99, makeId: () => "baru",
  });
  assert.equal(mode, "new");
  assert.equal(version.id, "baru");
  assert.equal(version.name, "MEKKAH 2");
});

test("Save tidak bisa menimpa versi LAIN lewat namanya", () => {
  assert.throws(() => planWalkingSave({
    versions: [AMBA, MEKKAH], activeVersionId: "m", name: "AMBA", params: MEKKAH.params, now: 99, makeId: () => "x",
  }), /sudah dipakai versi lain/);
  assert.throws(() => planWalkingSave({
    versions: [AMBA, MEKKAH], activeVersionId: "", name: "AMBA", params: DEFAULT, now: 99, makeId: () => "x",
  }), /sudah dipakai versi lain/);
});

test("penanda belum-disimpan membandingkan dengan versi yang terbuka", () => {
  assert.equal(walkingDraftUnsaved({ draft: { ...MEKKAH.params }, activeVersion: MEKKAH, defaults: DEFAULT }), false);
  assert.equal(walkingDraftUnsaved({ draft: { ...MEKKAH.params, init_pitch_offset: -0.79 }, activeVersion: MEKKAH, defaults: DEFAULT }), true);
  assert.equal(walkingDraftUnsaved({ draft: { ...MEKKAH.params, init_pitch_offset: "-" }, activeVersion: MEKKAH, defaults: DEFAULT }), true,
    "angka yang masih diketik dihitung sebagai perubahan");
  // Pembacaan balik float32 sesudah Apply bukan perubahan:
  assert.equal(walkingDraftUnsaved({
    draft: { ...MEKKAH.params, init_pitch_offset: Math.fround(-0.8) }, activeVersion: MEKKAH, defaults: DEFAULT,
  }), false);
  assert.equal(walkingDraftUnsaved({ draft: { ...DEFAULT }, activeVersion: null, defaults: DEFAULT }), false);
});

test("verifikasi sesudah Save itu KETAT, tanpa toleransi", () => {
  assert.equal(sameWalkingParams(MEKKAH.params, { ...MEKKAH.params }), true);
  assert.equal(sameWalkingParams(MEKKAH.params, { ...MEKKAH.params, init_pitch_offset: Math.fround(-0.8) }), false);
  assert.equal(sameWalkingParams(MEKKAH.params, null), false);
});

test("klien penyimpanan mengirim operasi lewat POST berheader, dan tidak menelan galat", async () => {
  const calls = [];
  const ok = async (url, init) => {
    calls.push({ url, init });
    return { ok: true, status: 200, json: async () => ({ versions: [MEKKAH], activeVersionId: "m" }) };
  };
  await mutateWalkingStore({ op: "upsert", version: MEKKAH }, ok);
  assert.equal(calls[0].url, WALKING_STORE_URL);
  assert.equal(calls[0].init.method, "POST");
  assert.equal(calls[0].init.headers["X-Bascorro-Studio"], "1");
  assert.deepEqual(JSON.parse(calls[0].init.body), { op: "upsert", version: MEKKAH });

  const ditolak = async () => ({ ok: false, status: 409, json: async () => ({ error: "Nama sudah dipakai" }) });
  await assert.rejects(mutateWalkingStore({ op: "upsert", version: MEKKAH }, ditolak), /Nama sudah dipakai/);

  // Server tanpa plugin menjawab index.html: harus jadi galat, bukan daftar kosong.
  const html = async () => ({ ok: true, status: 200, json: async () => { throw new SyntaxError("bukan JSON"); } });
  await assert.rejects(fetchWalkingStore(html), /tidak melayani penyimpanan/);
});

// ---------------------------------------------------------------------------
// Pemisahan tegas EDITING / RUNTIME / LAST SAVED / DEFAULT.

function badanFungsi(teks, nama) {
  const awal = teks.indexOf(`const ${nama} = `);
  assert.notEqual(awal, -1, `fungsi ${nama} tidak ditemukan di App.jsx`);
  const buka = teks.indexOf("{", teks.indexOf("=>", awal));
  let dalam = 0;
  for (let i = buka; i < teks.length; i += 1) {
    if (teks[i] === "{") dalam += 1;
    else if (teks[i] === "}" && (dalam -= 1) === 0) return teks.slice(buka, i + 1);
  }
  throw new Error(`badan ${nama} tidak tertutup`);
}

test("Apply tidak pernah menulis hasil baca robot ke isian operator", () => {
  // Dulu applyWalkingWithContext() memanggil setWalkingParams(params) dengan
  // hasil BACA ULANG dari robot. Itu jalur nyata runtime -> editing state:
  // Walking Tuner di tab Tuning memakai fungsi yang sama dengan draft-nya
  // sendiri, jadi Apply di sana diam-diam mengganti isian tab Walking, dan
  // Save berikutnya menuliskannya ke versi yang sedang terbuka.
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "applyWalkingWithContext");
  const panggilan = [...badan.matchAll(/^\s*setWalkingParams\(/gm)];
  assert.deepEqual(panggilan.map((m) => m[0].trim()), [],
    "Apply hanya boleh memperbarui walkingCurrentParams (kolom Current), bukan isian");
  assert.match(badan, /setWalkingCurrentParams\(params\)/,
    "hasil pembacaan tetap harus mendarat di RUNTIME STATE");
});

test("hanya tindakan operator atau snapshot tersimpan yang boleh mengisi editing state", () => {
  // Penjaga daftar-putih: tiap penulis isian harus berasal dari fungsi yang
  // memang dipicu operator (tombol/isian) atau dari data tersimpan. Kalau ada
  // penulis baru muncul di sini, ia wajib diperiksa dulu -- di situlah runtime
  // dulu menyelinap masuk.
  const teks = readFileSync(SUMBER, "utf8");
  const boleh = [
    "updateWalkingParam",        // operator mengetik
    "sendBalanceCommand",        // operator menekan Balance ON/OFF
    "loadWalkingParams",         // operator menekan Load (baris PARAMETER)
    "applyWalkingPreset",        // operator menekan preset
    "saveWalkingVersion",        // hasil Save yang sudah diverifikasi server
    "loadWalkingVersion",        // versi tersimpan
    "selectWalkingVersion",      // memilih versi = memuatnya
    "restoreWalkingDraft",       // snapshot tersimpan, saat Studio dibuka
  ];
  const luar = [];
  for (const m of teks.matchAll(/setWalkingParams\(/g)) {
    const sebelum = teks.slice(0, m.index);
    if (/\/\/[^\n]*$/.test(sebelum)) continue;                 // di dalam komentar
    const fn = [...sebelum.matchAll(/\n {2}const ([A-Za-z0-9_]+) = /g)].pop()?.[1];
    if (!boleh.includes(fn)) luar.push(fn);
  }
  assert.deepEqual(luar, [], `penulis isian di luar daftar putih: ${luar.join(", ")}`);
});

test("snapshot Save kebal terhadap perubahan isian sesudahnya", () => {
  // Spec poin 1: deep copy, bukan reference ke state runtime.
  const isian = penuh({ init_pitch_offset: -0.5 });
  const { version } = planWalkingSave({
    versions: [MEKKAH], activeVersionId: "m", name: "JALAN MENUJU MEKKAH",
    params: { ...isian }, now: 7, makeId: () => "x",
  });
  isian.init_pitch_offset = 999;
  isian.x_move_amplitude = 999;
  assert.equal(version.params.init_pitch_offset, -0.5);
  assert.notEqual(version.params, isian);
  assert.equal(version.params.x_move_amplitude, penuh().x_move_amplitude);
});

test("memperbarui versi tidak mengubah objek versi lama di daftar", () => {
  const sebelum = JSON.stringify(MEKKAH);
  planWalkingSave({
    versions: [AMBA, MEKKAH], activeVersionId: "m", name: "", params: penuh({ period_time: 0.9 }),
    now: 7, makeId: () => "x",
  });
  assert.equal(JSON.stringify(MEKKAH), sebelum, "daftar sumber harus tetap utuh (immutable)");
});

test("runtime yang bergeser tidak mengubah last saved state, hanya menandainya", () => {
  // Spec poin 3: SAVE = A; robot berjalan dan runtime jadi B; A harus tetap A.
  const A = penuh({ x_move_amplitude: 0.016, init_pitch_offset: -0.5 });
  const tersimpan = { ...MEKKAH, params: { ...A } };
  const B = { ...A, x_move_amplitude: 0, init_pitch_offset: -0.07 };   // runtime sesudah penulis lain
  assert.deepEqual(tersimpan.params, A, "nilai tersimpan tidak boleh ikut bergeser");
  // Isian tetap A, jadi penanda "belum disimpan" TIDAK menyala walau runtime B.
  assert.equal(walkingDraftUnsaved({ draft: { ...A }, activeVersion: tersimpan, defaults: DEFAULT }), false);
  // Dan kalau operator memang menarik runtime ke isian (tombol Load), itu terlihat.
  assert.equal(walkingDraftUnsaved({ draft: B, activeVersion: tersimpan, defaults: DEFAULT }), true);
});

// ---------------------------------------------------------------------------
// Acuan pose dipatok sebelum walking_module dinyalakan.
import { walkingPoseFarFromReference, WALKING_BIAS_FIELDS } from "../src/walkingParams.js";
import { WALKING_POSE_REFERENCE } from "../src/walkingPoseReference.js";

const REF = WALKING_POSE_REFERENCE.offsets;

test("enam field yang menentukan walking_bias_, tanpa init_y_offset", () => {
  // computeNeutralLegAngle dipanggil dengan with_y_offset=false saat bias
  // direkam, jadi y TIDAK pernah masuk acuan dan tidak boleh ikut digeser.
  assert.deepEqual([...WALKING_BIAS_FIELDS].sort(), [
    "hip_pitch_offset", "init_pitch_offset", "init_roll_offset",
    "init_x_offset", "init_yaw_offset", "init_z_offset",
  ]);
  assert.equal(WALKING_BIAS_FIELDS.includes("init_y_offset"), false);
});

test("offset yang jauh dari page 2 dilaporkan, yang dekat tidak", () => {
  assert.deepEqual(walkingPoseFarFromReference({ ...REF }, REF), []);
  const jauh = walkingPoseFarFromReference({ ...REF, init_pitch_offset: -0.5, hip_pitch_offset: 0 }, REF);
  assert.deepEqual(jauh.map((x) => x.key).sort(), ["hip_pitch_offset", "init_pitch_offset"]);
  assert.ok(Math.abs(jauh.find((x) => x.key === "init_pitch_offset").selisih - (-0.5 - REF.init_pitch_offset)) < 1e-9);
  // Perubahan kecil tidak mengganggu operator dengan dialog.
  assert.deepEqual(walkingPoseFarFromReference({ ...REF, init_pitch_offset: REF.init_pitch_offset - 0.05 }, REF), []);
});

test("acuan pose diterbitkan SEBELUM walking_module dinyalakan", () => {
  // Kalau urutannya terbalik, biasnya menangkap nilai operator sendiri dan
  // offsetnya meniadakan diri lagi -- persis bug "sesi 2 bentuknya balik".
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "initThenEnableWalking");
  const acuan = badan.indexOf("publishWalkingPoseReference(");
  const nyala = badan.indexOf('data: "walking_module"');
  assert.ok(acuan !== -1, "acuan pose harus diterbitkan di sini");
  assert.ok(nyala !== -1, "modul harus dinyalakan di sini");
  assert.ok(acuan < nyala, "acuan WAJIB mendarat sebelum modul menyala");
});

test("page 2 diputar sebelum acuan dipatok dan walking_module dinyalakan", () => {
  // walking_bias_ merekam goal yang SEDANG dipegang. Tanpa page 2 dulu, sesudah
  // Torque ON yang terekam adalah pose lemas robot, bukan page 2.
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "initThenEnableWalking");
  const page2 = badan.indexOf("playWalkingReadyPage(");
  const acuan = badan.indexOf("publishWalkingPoseReference(");
  assert.ok(page2 !== -1, "page 2 harus diputar di sini");
  assert.ok(page2 < acuan, "page 2 WAJIB selesai sebelum acuan dan modul walking");
  const putar = badanFungsi(readFileSync(SUMBER, "utf8"), "playWalkingReadyPage");
  assert.ok(putar.indexOf('data: "action_module"') < putar.indexOf("WALKING_READY_PAGE_NUM }"));
  assert.match(putar, /actionIsRunningRef\.current/, "ditunggu sampai action_module selesai, bukan cuma jeda");
});

test("Apply & Start menyalakan modul dulu, baru mengirim nilai operator", () => {
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "applyWalkingAndStart");
  const nyala = badan.indexOf("initThenEnableWalking(");
  const kirim = badan.indexOf("applyWalkingWithContext(");
  assert.ok(nyala !== -1 && kirim !== -1);
  assert.ok(nyala < kirim, "kalau nilai operator dikirim dulu, ia jadi acuannya sendiri");
});

test("Start mengirim ulang nilai operator sesudah acuan diterbitkan", () => {
  // initThenEnableWalking menaruh ACUAN di runtime; tanpa kirim ulang, robot
  // akan berjalan memakai acuan itu, bukan versi yang dipilih operator.
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "startWalkingNow");
  const nyala = badan.indexOf("initThenEnableWalking(");
  const kirim = badan.indexOf("applyWalkingWithContext(");
  assert.ok(nyala !== -1 && kirim !== -1, "Start harus menyalakan modul dan mengirim nilainya");
  assert.ok(nyala < kirim, "kirim ulang harus SESUDAH modul menyala");
});
