import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

// TERUKUR DI ROBOT 2026-09-09.
//
// Tombol "Torque ON" di Studio memasang batas kecepatan servo
// (profile_velocity/profile_acceleration) dan SENGAJA membiarkannya terpasang.
// Batas itu hidup di RAM servo sampai daya servo dimatikan, sedangkan
// torqueLimitActive cuma state React yang hilang begitu halaman dimuat ulang.
//
// Dulu pelepasannya dipagari `if (torqueLimitActive)`. Jadi urutan
//   Torque ON  ->  reload/buka ulang Studio  ->  Apply & Start
// berjalan dengan batas MASIH terpasang dan tidak ada yang melepasnya.
// Akibatnya, dengan parameter yang sama persis:
//   ayunan pinggul yang diperintahkan 23,21 deg -> yang nyata cuma 5,0 deg (22%)
// yaitu "robot jalan pelan, langkah kecil, padahal nilainya tidak diubah".
//
// Menulis 0 saat tidak ada batas sama sekali tidak berbahaya (0 = nilai
// pabrik = tanpa batas), jadi pelepasannya HARUS tanpa syarat.
const SUMBER = new URL("../src/App.jsx", import.meta.url);

// Komentar dibuang dulu: teks penjelas di bawah ini memang MENYEBUT
// torqueLimitActive, dan pemeriksaan "tidak boleh ada pagar" harus melihat
// kode saja.
const tanpaKomentar = (s) => s.replace(/^[ \t]*\/\/.*$/gm, "");

function badanFungsi(teks, nama) {
  const i = teks.indexOf(`const ${nama} = `);
  assert.ok(i >= 0, `fungsi ${nama} tidak ditemukan`);
  const buka = teks.indexOf("{", i);
  let dalam = 0;
  for (let j = buka; j < teks.length; j += 1) {
    if (teks[j] === "{") dalam += 1;
    else if (teks[j] === "}") {
      dalam -= 1;
      if (dalam === 0) return teks.slice(buka, j + 1);
    }
  }
  throw new Error(`badan ${nama} tidak tertutup`);
}

test("gait start always clears the servo speed limit, unconditionally", () => {
  const teks = readFileSync(SUMBER, "utf8");
  const badan = tanpaKomentar(badanFungsi(teks, "initThenEnableWalking"));
  assert.match(badan, /restoreFullSpeed\(\)/,
    "initThenEnableWalking harus melepas batas kecepatan sebelum gait mulai");
  assert.doesNotMatch(badan, /torqueLimitActive/,
    "pelepasan batas tidak boleh dipagari torqueLimitActive: state itu hilang "
    + "saat halaman dimuat ulang, sementara batasnya masih hidup di RAM servo");
});

test("both walking start paths go through initThenEnableWalking", () => {
  const teks = readFileSync(SUMBER, "utf8");
  for (const nama of ["applyWalkingAndStart", "startWalkingNow"]) {
    // Boleh membawa draft sebagai argumen kedua (acuan pose dipatok di sana);
    // yang wajib cuma: kedua jalur start benar-benar melewatinya.
    assert.match(badanFungsi(teks, nama), /initThenEnableWalking\(context[,)]/,
      `${nama} harus lewat initThenEnableWalking, kalau tidak batasnya tidak dilepas`);
  }
});

test("starting a demo mode clears the limit too", () => {
  const teks = readFileSync(SUMBER, "utf8");
  const badan = tanpaKomentar(badanFungsi(teks, "handleDemoMode"));
  assert.match(badan, /restoreFullSpeed\(/, "demo juga berjalan dengan gait");
  assert.doesNotMatch(badan, /torqueLimitActive/,
    "sama seperti gait: jangan dipagari state yang hilang saat reload");
});

test("restoreFullSpeed really writes zero to both profile registers", () => {
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "restoreFullSpeed");
  assert.match(badan, /\["profile_velocity",\s*0\]/);
  assert.match(badan, /\["profile_acceleration",\s*0\]/);
});

// restoreFullSpeed sekarang ditunggu (await). writeSyncItemsSequential dulu
// keluar diam-diam tanpa memanggil onDone kalau urutannya dibatalkan atau ROS
// putus -- Promise-nya tidak pernah selesai dan Apply & Start menggantung
// selamanya. onDone harus dipanggil di SEMUA jalan keluar.
test("writeSyncItemsSequential always settles its onDone callback", () => {
  const badan = badanFungsi(readFileSync(SUMBER, "utf8"), "writeSyncItemsSequential");
  const keluar = [...tanpaKomentar(badan).matchAll(/return;/g)];
  const dipanggil = [...tanpaKomentar(badan).matchAll(/onDone\?\.\(/g)];
  assert.ok(dipanggil.length >= keluar.length,
    `tiap jalan keluar harus memanggil onDone: ${keluar.length} return, `
    + `${dipanggil.length} panggilan onDone`);
});

test("initThenEnableWalking waits for the limit write to land", () => {
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "initThenEnableWalking"));
  assert.match(badan, /await restoreFullSpeed\(\)/,
    "tanpa await, \"start\" bisa terkirim sebelum profile_velocity mendarat "
    + "dan langkah pertama tetap terkekang");
});

// Syarat "Load dulu" dicabut 2026-09-09. Kalau ada yang memasangnya lagi,
// memilih versi tersimpan lalu Apply & Start kembali ditolak mentah-mentah.
test("Apply no longer refuses when the robot has not been read yet", () => {
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "applyWalkingWithContext"));
  assert.doesNotMatch(badan, /if \(!baseline\)\s*throw/,
    "baseline tidak lagi menentukan isi yang dikirim; memaksa Load cuma "
    + "menghalangi pemakaian versi tersimpan");
  assert.match(badan, /baseline: baseline \?\? draft/,
    "draft dipakai sebagai baseline-nya sendiri saat belum ada pembacaan");
});

// ---------------------------------------------------------------------------
// TORQUE ON MENAHAN DI TEMPAT (2026-09-13).
//
// Keluhan berurutan dari operator: Torque ON "melompat ke suatu nilai", lalu
// sesudah percobaan perbaikan di controller "malah kecepatan dan servonya
// mati". Akarnya: selagi torsi mati, goal controller berhenti di pose lama
// sementara robot melorot, dan modul yang masih memegang sendi terus mengirim
// pose lama itu. Menyalakan torsi berarti menarik robot ke sana -- cepat
// (menyentak) atau pelan (beban penuh berkepanjangan -> overload).
//
// Urutan yang benar: batas kecepatan -> lepas modul (TUNGGU terkonfirmasi) ->
// goal := posisi nyata -> torsi. Test di bawah menjaga urutan itu.
const posisiDi = (badan, pola) => {
  const m = badan.match(pola);
  return m ? m.index : -1;
};

test("Torque ON: batas kecepatan, lepas modul, samakan goal, BARU torsi", () => {
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "handleTorque"));
  const batas = posisiDi(badan, /\["profile_velocity",\s*preset\.velocity\]/);
  const lepas = posisiDi(badan, /data:\s*"none"/);
  const tunggu = posisiDi(badan, /waitForJointsReleased\(/);
  const baca = posisiDi(badan, /readPresentJointsOnce\(/);
  const goal = posisiDi(badan, /setJointStatesPubRef\.current\.publish\(/);
  const torsi = posisiDi(badan, /writeSyncItem\("torque_enable",\s*1\)/);
  for (const [nama, i] of Object.entries({ batas, lepas, tunggu, baca, goal, torsi })) {
    assert.ok(i >= 0, `langkah "${nama}" hilang dari handleTorque`);
  }
  assert.ok(batas < lepas, "batas kecepatan dipasang sebelum apa pun bergerak");
  assert.ok(lepas < tunggu, "pelepasan modul harus ditunggu, bukan ditebak dengan jeda");
  assert.ok(tunggu < baca, "posisi dibaca SESUDAH tidak ada modul yang menimpa goal");
  assert.ok(baca < goal, "goal diisi dari pembacaan yang baru");
  assert.ok(goal < torsi, "torsi WAJIB menyala paling akhir, sesudah goal = posisi nyata");
});

test("Torque ON tidak menyalakan torsi di dalam urutan batas kecepatan", () => {
  // Dulu torque_enable ikut dikirim bersama profile_velocity/acceleration, jadi
  // torsi menyala sementara goal masih pose lama.
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "handleTorque"));
  const batch = badan.match(/writeSyncItemsAwaited\(\[([\s\S]*?)\]\)/);
  assert.ok(batch, "batas kecepatan harus dikirim lewat writeSyncItemsAwaited");
  assert.doesNotMatch(batch[1], /torque_enable/, "torque_enable tidak boleh ada di urutan batas");
  assert.equal((badan.match(/"torque_enable",\s*1/g) || []).length, 1, "torsi dinyalakan tepat SATU kali");
});

test("Torque ON batal TANPA menyalakan torsi kalau pelepasan atau pembacaan gagal", () => {
  // Menyalakan torsi dengan goal lama adalah persis sentakan yang dihindari, jadi
  // kegagalan di tengah jalan harus berhenti sebelum torque_enable.
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "handleTorque"));
  const torsi = posisiDi(badan, /writeSyncItem\("torque_enable",\s*1\)/);
  const cekLepas = posisiDi(badan, /if \(!await waitForJointsReleased\([^)]*\)\) \{[\s\S]*?return;/);
  const cekBaca = posisiDi(badan, /if \(!posisi\) \{[\s\S]*?return;/);
  assert.ok(cekLepas >= 0 && cekLepas < torsi, "gagal melepas modul harus keluar sebelum torsi");
  assert.ok(cekBaca >= 0 && cekBaca < torsi, "gagal membaca posisi harus keluar sebelum torsi");
});

test("pelepasan modul dianggap selesai hanya kalau SEMUA sendi 'none'", () => {
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "waitForJointsReleased"));
  assert.match(badan, /modul\.length === names\.length/, "jumlah sendi yang menjawab harus lengkap");
  assert.match(badan, /modul\.every\(\(nama\) => nama === "none"\)/, "setiap sendi wajib 'none'");
  assert.match(badan, /MODULE_RELEASE_TIMEOUT_MS/, "harus ada batas waktu, bukan menunggu selamanya");
});

test("posisi untuk goal dibaca SEGAR, bukan dari snapshot langganan 2 detik", () => {
  const teks = readFileSync(SUMBER, "utf8");
  const baca = tanpaKomentar(badanFungsi(teks, "readPresentJointsOnce"));
  assert.match(baca, /\/robotis\/present_joint_states/);
  assert.doesNotMatch(baca, /throttle_rate/, "pembacaan untuk goal tidak boleh dibatasi laju");
  const torsi = tanpaKomentar(badanFungsi(teks, "handleTorque"));
  assert.doesNotMatch(torsi, /robotJointNamesRef\.current\s*\)?\s*\.?position/,
    "snapshot nama sendi tidak boleh dipakai sebagai sumber posisi");
});

test("Torque OFF tetap satu tulisan torque_enable 0 (sudah benar menurut operator)", () => {
  const badan = tanpaKomentar(badanFungsi(readFileSync(SUMBER, "utf8"), "handleTorque"));
  const off = badan.match(/if \(!enable\) \{([\s\S]*?)\n    \}/);
  assert.ok(off, "cabang Torque OFF harus ada");
  assert.match(off[1], /writeSyncItem\("torque_enable",\s*0\)/);
  assert.doesNotMatch(off[1], /"none"|setJointStates|profile_/, "Torque OFF tidak boleh ikut berubah");
});
