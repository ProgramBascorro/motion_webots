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
    assert.match(badanFungsi(teks, nama), /initThenEnableWalking\(context\)/,
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
