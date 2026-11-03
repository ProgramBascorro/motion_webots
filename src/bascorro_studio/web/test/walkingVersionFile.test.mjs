import test from "node:test";
import assert from "node:assert/strict";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import {
  createWalkingVersionFile, applyStoreOperation, emptyStoreState, StoreError, STORE_FILE_NAME,
  REFERENCE_FILE_NAME,
} from "../server/walkingVersionFile.js";
import { WALKING_KEYS } from "../src/walkingParams.js";

const tempDir = () => fs.mkdtemp(path.join(os.tmpdir(), "walking-store-"));

// Satu set parameter lengkap (29 field) dengan angka yang sengaja "sulit":
// desimal panjang hasil konversi derajat, angka sangat kecil, dan negatif.
function params(overrides = {}) {
  const base = {
    init_x_offset: -0.039, init_y_offset: 0.028, init_z_offset: 0.0408, init_roll_offset: 0,
    init_pitch_offset: -0.069813170079773, init_yaw_offset: 0, period_time: 0.75, dsp_ratio: 0.4,
    step_fb_ratio: 0.25, soft_start_time: 0.08, x_move_amplitude: 0.016, y_move_amplitude: 0,
    z_move_amplitude: 0.037, angle_move_amplitude: 0, angle_move_amplitude_turn_in: 0,
    move_aim_on: false, balance_enable: false, balance_hip_roll_gain: 0.35, balance_knee_gain: 0.4,
    balance_ankle_roll_gain: 0.7, balance_ankle_pitch_gain: 0.9, y_swap_amplitude: 0.0021,
    z_swap_amplitude: 1e-9, arm_swing_gain: 0, pelvis_offset: 0.05, hip_pitch_offset: 0.14,
    p_gain: 36, i_gain: 0, d_gain: 0,
  };
  return { ...base, ...overrides };
}

const version = (id, name, overrides = {}, updatedAt = 1000) =>
  ({ id, name, createdAt: 500, updatedAt, params: params(overrides) });

function assertIdentical(actual, expected, label) {
  assert.equal(actual.id, expected.id, `${label}: id`);
  assert.equal(actual.name, expected.name, `${label}: nama`);
  assert.equal(actual.createdAt, expected.createdAt, `${label}: createdAt`);
  assert.equal(actual.updatedAt, expected.updatedAt, `${label}: updatedAt`);
  assert.deepEqual(Object.keys(actual.params).sort(), [...WALKING_KEYS].sort(), `${label}: himpunan field`);
  for (const key of WALKING_KEYS) {
    assert.ok(actual.params[key] === expected.params[key],
      `${label}: ${key} berubah ${expected.params[key]} -> ${actual.params[key]}`);
  }
}

test("TEST A+B: Save lalu 'Studio ditutup-dibuka' mengembalikan ke-29 field secara identik", async () => {
  const dir = await tempDir();
  const mekkah = version("mekkah", "JALAN MENUJU MEKKAH", { init_pitch_offset: -0.8, x_move_amplitude: 0.0173 });
  await createWalkingVersionFile({ dataDir: dir }).mutate({ op: "upsert", version: mekkah });

  // Objek penyimpanan BARU di direktori yang sama = proses server baru.
  const reopened = await createWalkingVersionFile({ dataDir: dir }).load();
  assert.equal(reopened.exists, true);
  assert.equal(reopened.activeVersionId, "mekkah", "versi yang terakhir disimpan jadi versi aktif");
  assertIdentical(reopened.versions[0], mekkah, "sesudah dibuka ulang");
});

test("TEST C: lima versi tetap memegang nilainya sendiri-sendiri", async () => {
  const dir = await tempDir();
  const store = createWalkingVersionFile({ dataDir: dir });
  const semua = [
    version("a", "JALAN MENUJU MEKKAH", { init_pitch_offset: -0.29 }),
    version("b", "GESER KANAN", { y_move_amplitude: -0.02 }),
    version("c", "GESER KIRI", { y_move_amplitude: 0.02 }),
    version("d", "MUTAR KANAN", { angle_move_amplitude: -0.3 }),
    version("e", "MUTAR KIRI", { angle_move_amplitude: 0.3 }),
  ];
  for (const item of semua) await store.mutate({ op: "upsert", version: item });

  // Sunting SATU versi; empat lainnya tidak boleh ikut berubah.
  const diubah = version("c", "GESER KIRI", { y_move_amplitude: 0.031 }, 2000);
  await store.mutate({ op: "upsert", version: diubah });

  const { versions } = await createWalkingVersionFile({ dataDir: dir }).load();
  const byId = Object.fromEntries(versions.map((item) => [item.id, item]));
  for (const item of semua) {
    assertIdentical(byId[item.id], item.id === "c" ? diubah : item, item.name);
  }
});

test("TEST D: simpan-tutup-buka berulang tidak pernah kembali ke nilai lama atau default", async () => {
  const dir = await tempDir();
  let terakhir = null;
  for (let putaran = 1; putaran <= 6; putaran += 1) {
    terakhir = version("mekkah", "JALAN MENUJU MEKKAH", {
      init_x_offset: -0.039 - putaran / 1000, init_pitch_offset: -0.29 - putaran / 100,
    }, 1000 + putaran);
    await createWalkingVersionFile({ dataDir: dir }).mutate({ op: "upsert", version: terakhir });
    for (let buka = 0; buka < 3; buka += 1) {
      const { versions } = await createWalkingVersionFile({ dataDir: dir }).load();
      assertIdentical(versions[0], terakhir, `putaran ${putaran}, buka ke-${buka + 1}`);
    }
  }
});

test("poin 9: sunting lalu Save menghasilkan nilai TERBARU, bukan sebelum perubahan", async () => {
  const dir = await tempDir();
  const store = createWalkingVersionFile({ dataDir: dir });
  await store.mutate({ op: "upsert", version: version("m", "JALAN MENUJU MEKKAH", { init_pitch_offset: -0.29 }) });
  const baru = version("m", "JALAN MENUJU MEKKAH", { init_pitch_offset: -0.8 }, 2000);
  await store.mutate({ op: "upsert", version: baru });
  const { versions } = await createWalkingVersionFile({ dataDir: dir }).load();
  assert.equal(versions.length, 1, "menyunting tidak boleh membuat salinan");
  assertIdentical(versions[0], baru, "sesudah disunting");
});

test("Save tidak pernah mengganti nama versi yang sudah ada", async () => {
  const dir = await tempDir();
  const store = createWalkingVersionFile({ dataDir: dir });
  await store.mutate({ op: "upsert", version: version("a", "AMBA") });
  const sebelum = await fs.readFile(path.join(dir, STORE_FILE_NAME), "utf8");
  await assert.rejects(
    store.mutate({ op: "upsert", version: version("a", "JALAN MENUJU MEKKAH") }),
    (error) => error instanceof StoreError && error.status === 409,
  );
  assert.equal(await fs.readFile(path.join(dir, STORE_FILE_NAME), "utf8"), sebelum, "berkas tidak boleh tersentuh");
});

test("versi baru tidak boleh memakai nama versi lain (dulu begitulah satu versi menimpa versi lain)", async () => {
  const store = createWalkingVersionFile({ dataDir: await tempDir() });
  await store.mutate({ op: "upsert", version: version("a", "JALAN MENUJU MEKKAH") });
  await assert.rejects(
    store.mutate({ op: "upsert", version: version("b", "JALAN MENUJU MEKKAH", { init_pitch_offset: 0 }) }),
    (error) => error.status === 409,
  );
  const { versions } = await store.load();
  assert.equal(versions.length, 1);
  assert.equal(versions[0].params.init_pitch_offset, -0.069813170079773);
});

test("nilai tidak sah DITOLAK utuh, tidak diisi default atau dibulatkan", async () => {
  const dir = await tempDir();
  const store = createWalkingVersionFile({ dataDir: dir });
  await store.mutate({ op: "upsert", version: version("a", "AMBA") });
  const sebelum = await fs.readFile(path.join(dir, STORE_FILE_NAME), "utf8");
  const buruk = [
    { ...version("a", "AMBA"), params: params({ x_move_amplitude: "0.016" }) },
    { ...version("a", "AMBA"), params: params({ x_move_amplitude: Number.NaN }) },
    { ...version("a", "AMBA"), params: params({ p_gain: 36.5 }) },
    { ...version("a", "AMBA"), params: params({ balance_enable: 1 }) },
    { ...version("a", "AMBA"), params: { ...params(), bukan_field: 1 } },
    { ...version("a", "AMBA"), params: Object.fromEntries(Object.entries(params()).filter(([k]) => k !== "d_gain")) },
    { ...version("a", "AMBA"), name: "  " },
  ];
  for (const item of buruk) {
    await assert.rejects(store.mutate({ op: "upsert", version: item }), (error) => error.status === 400);
  }
  assert.equal(await fs.readFile(path.join(dir, STORE_FILE_NAME), "utf8"), sebelum);
});

test("hapus menghapus satu versi saja dan melepas status aktifnya", async () => {
  const store = createWalkingVersionFile({ dataDir: await tempDir() });
  await store.mutate({ op: "upsert", version: version("a", "AMBA") });
  await store.mutate({ op: "upsert", version: version("b", "MENCARI CELAH") });
  const state = await store.mutate({ op: "delete", id: "b" });
  assert.deepEqual(state.versions.map((item) => item.id), ["a"]);
  assert.equal(state.activeVersionId, null);
});

test("impor dari browser hanya MENAMBAH, tidak pernah menimpa, dan aman diulang", async () => {
  const store = createWalkingVersionFile({ dataDir: await tempDir() });
  const milikRobot = version("a", "AMBA", { init_x_offset: -0.03 });
  await store.mutate({ op: "upsert", version: milikRobot });
  const dariBrowser = [
    version("a", "AMBA", { init_x_offset: -0.5 }), // id sama, isi beda: TIDAK boleh menimpa
    version("b", "MENCARI CELAH"),
    { id: "c", name: "RUSAK", createdAt: 1, updatedAt: 1, params: { x_move_amplitude: 0 } },
  ];
  const pertama = await store.mutate({ op: "import", versions: dariBrowser });
  const kedua = await store.mutate({ op: "import", versions: dariBrowser });
  assert.equal(pertama.result.imported, 1);
  assert.equal(pertama.result.rejected.length, 1, "yang tidak sah dilaporkan, bukan diam-diam dibuang");
  assert.equal(kedua.result.imported, 0);
  assert.deepEqual(kedua.versions.map((item) => item.id), ["a", "b"]);
  assertIdentical(kedua.versions[0], milikRobot, "versi yang sudah di robot");
});

test("berkas rusak tidak pernah diganti daftar kosong", async () => {
  const dir = await tempDir();
  const file = path.join(dir, STORE_FILE_NAME);
  await fs.writeFile(file, '{"format":"bascorro-studio/walking-versions","versions":[{"id":"a"', "utf8");
  const store = createWalkingVersionFile({ dataDir: dir });
  await assert.rejects(store.load(), (error) => error.status === 500);
  await assert.rejects(store.mutate({ op: "upsert", version: version("b", "BARU") }), (error) => error.status === 500);
  assert.equal(await fs.readFile(file, "utf8"), '{"format":"bascorro-studio/walking-versions","versions":[{"id":"a"');
});

test("setiap Save menyimpan cadangan berkas lama, dan jumlahnya dibatasi", async () => {
  const dir = await tempDir();
  let jam = 1_800_000_000_000;
  const store = createWalkingVersionFile({ dataDir: dir, keepBackups: 3, now: () => (jam += 1000) });
  for (let i = 0; i < 6; i += 1) {
    await store.mutate({ op: "upsert", version: version("a", "AMBA", { init_x_offset: -i / 100 }) });
  }
  const cadangan = (await fs.readdir(path.join(dir, "backup"))).sort();
  assert.equal(cadangan.length, 3);
  const tertua = JSON.parse(await fs.readFile(path.join(dir, "backup", cadangan[0]), "utf8"));
  const terbaru = JSON.parse(await fs.readFile(path.join(dir, "backup", cadangan[2]), "utf8"));
  assert.equal(tertua.versions[0].params.init_x_offset, -0.02);
  assert.equal(terbaru.versions[0].params.init_x_offset, -0.04, "cadangan terbaru = keadaan tepat sebelum Save terakhir");
  const sisa = (await fs.readdir(dir)).filter((name) => name.includes(".tmp-"));
  assert.deepEqual(sisa, [], "tidak boleh ada berkas sementara yang tertinggal");
});

test("dua puluh Save bersamaan tidak saling menimpa", async () => {
  const store = createWalkingVersionFile({ dataDir: await tempDir() });
  await Promise.all(Array.from({ length: 20 }, (_, i) =>
    store.mutate({ op: "upsert", version: version(`v${i}`, `VERSI ${i}`, { init_x_offset: -(i + 1) / 1000 }) })));
  const { versions, revision } = await store.load();
  assert.equal(versions.length, 20);
  assert.equal(revision, 20);
  for (let i = 0; i < 20; i += 1) {
    assert.equal(versions.find((item) => item.id === `v${i}`).params.init_x_offset, -(i + 1) / 1000);
  }
});

test("versi tersimpan tidak berbagi objek dengan data yang dikirim", () => {
  const masuk = version("a", "AMBA");
  const { state } = applyStoreOperation(emptyStoreState(), { op: "upsert", version: masuk });
  masuk.params.init_x_offset = 99;
  assert.equal(state.versions[0].params.init_x_offset, -0.039);
  assert.notEqual(state.versions[0].params, masuk.params);
});

test("pilihan tampilan relatif/asli ikut tersimpan", async () => {
  const dir = await tempDir();
  await createWalkingVersionFile({ dataDir: dir }).mutate({ op: "ui", ui: { relativeDisplay: false } });
  const { ui } = await createWalkingVersionFile({ dataDir: dir }).load();
  assert.equal(ui.relativeDisplay, false);
});

test("penyimpanan yang belum pernah ada dilaporkan kosong, bukan diisi default", async () => {
  const state = await createWalkingVersionFile({ dataDir: await tempDir() }).load();
  assert.equal(state.exists, false);
  assert.deepEqual(state.versions, []);
  assert.equal(state.activeVersionId, null);
});

test("-0 kembali sebagai 0: JSON memang tidak bisa menulis nol bertanda, nilainya tetap identik", async () => {
  // Bukan pembulatan dari penyimpanan ini: JSON.stringify(-0) === "0", dan itu
  // berlaku sama untuk localStorage lama. -0 === 0, float32 robotnya sama, dan
  // (-0).toFixed() di layar juga "0".
  const dir = await tempDir();
  await createWalkingVersionFile({ dataDir: dir }).mutate({
    op: "upsert", version: version("a", "AMBA", { init_roll_offset: -0 }),
  });
  const { versions } = await createWalkingVersionFile({ dataDir: dir }).load();
  assert.ok(versions[0].params.init_roll_offset === -0);
  assert.equal(Math.fround(versions[0].params.init_roll_offset), 0);
});

test("tanda acuan page 2: berkas baru dari Save ditandai, impor tidak, tanda lama terbawa", async () => {
  const dir = await tempDir();
  await fs.writeFile(path.join(dir, REFERENCE_FILE_NAME), JSON.stringify({ id: "page2-uji" }));
  const store = createWalkingVersionFile({ dataDir: dir });
  const lahir = await store.mutate({ op: "upsert", version: version("a", "A") });
  assert.equal(lahir.poseReferenceId, "page2-uji", "angkanya diketik terhadap acuan yang berlaku");

  // Acuan diturunkan ulang: hanya walking_ready_fk.py yang boleh menandai ulang
  // (sambil menggeser angkanya). Save berikutnya membawa tanda LAMA apa adanya.
  await fs.writeFile(path.join(dir, REFERENCE_FILE_NAME), JSON.stringify({ id: "page2-baru" }));
  const sesudah = await store.mutate({ op: "upsert", version: version("b", "B") });
  assert.equal(sesudah.poseReferenceId, "page2-uji");
  assert.equal(JSON.parse(await fs.readFile(path.join(dir, STORE_FILE_NAME), "utf8")).poseReferenceId, "page2-uji");

  const lain = await tempDir();
  await fs.writeFile(path.join(lain, REFERENCE_FILE_NAME), JSON.stringify({ id: "page2-uji" }));
  const impor = await createWalkingVersionFile({ dataDir: lain }).mutate({ op: "import", versions: [version("c", "C")] });
  assert.equal(impor.poseReferenceId, null, "versi dari browser bisa dari acuan mana pun");
});

test("tanpa berkas acuan (server uji), tidak ada tanda dan Save tetap jalan", async () => {
  const dir = await tempDir();
  const state = await createWalkingVersionFile({ dataDir: dir }).mutate({ op: "upsert", version: version("a", "A") });
  assert.equal(state.poseReferenceId, null);
  assert.equal(state.versions.length, 1);
});
