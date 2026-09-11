// Bagian MURNI dari "Versi Tersimpan" di tab Walking, dipisahkan dari App.jsx
// supaya perilakunya bisa diuji tanpa React, browser, maupun server. App.jsx
// memegang plumbing-nya; penyimpanan sebenarnya ada di disk robot
// (server/walkingVersionFile.js).
import { parametersEqual, WALKING_KEYS } from "./walkingParams.js";

// SEMAIAN PRESET TIDAK BOLEH MENIMPA APA PUN YANG SUDAH TERSIMPAN.
//
// Versi sebelumnya mencari slot dengan NAMA yang sama lalu mengganti isinya
// ("refresh in place"). Akibatnya: operator menyunting "JALAN KE SURGA",
// menyimpannya, menutup Studio -- dan begitu penanda seed-nya tidak cocok lagi,
// muat berikutnya mengembalikan angka hardcode dan pekerjaannya hilang tanpa
// satu pun peringatan.
//
// Penandanya tidak cocok lebih sering daripada dugaan: ia diturunkan dari
// JALAN_WALKING_SEED_REV, jadi setiap kenaikan revisi di kode membuat SEMUA
// browser menimpa slot itu sekali lagi. Jejaknya masih ada di localStorage robot
// ini -- tiga revisi penanda berdampingan (...jalan_ke_surga.v1, .v2,
// .2026-09-02), artinya penimpaan itu sudah terjadi dua kali. Itulah keluhan
// "sudah di-save, keluar lalu masuk lagi, malah berubah".
//
// Sekarang slot yang sudah terisi SELALU menang. Kalau sebuah preset memang
// harus disebarkan ulang ke browser yang sudah terisi, beri nama BARU.
export function walkingVersionSlotTaken(versions, id, name) {
  return versions.some((version) => version.id === id || version.name === name);
}

export function seedWalkingVersion(versions, { id, name, params, now = Date.now() }) {
  if (walkingVersionSlotTaken(versions, id, name)) return versions;
  return [{ id, name, createdAt: now, updatedAt: now, params }, ...versions];
}

// Membaca isi mentah localStorage menjadi { versions, rusak }.
//
// `rusak` berisi teks mentah yang TIDAK seluruhnya bisa dipulihkan. Pemanggil
// WAJIB menyelamatkannya sebelum daftar yang sudah menyusut ditulis balik ke
// kunci yang sama -- lihat rescueWalkingVersions() di App.jsx untuk alasan
// lengkapnya. Teks kosong bukan kerusakan: itu browser yang memang masih baru.
export function parseStoredWalkingVersions(raw) {
  if (raw === null || raw === undefined || raw === "") return { versions: [], rusak: null };
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return { versions: [], rusak: raw };
  }
  if (!Array.isArray(parsed)) return { versions: [], rusak: raw };
  const versions = parsed.filter((item) => item?.id && item?.params);
  return { versions, rusak: versions.length === parsed.length ? null : raw };
}

// ---------------------------------------------------------------------------
// Pemulihan saat Studio dibuka, dan penentuan versi mana yang ditulis Save.
//
// AKAR "sesudah Studio dibuka ulang parameternya kembali ke default": isian
// Walking dulu SELALU dibuat dari WALKING_DEFAULT_PARAMS, dan versi yang terakhir
// dipakai tidak pernah diingat. Versi tersimpan sebenarnya utuh di penyimpanan
// -- yang tidak pernah terjadi adalah memuatnya kembali ke isian. Sekarang versi
// aktif dicatat di penyimpanan robot dan dipulihkan; default hanya dipakai
// kalau memang tidak ada versi aktif.
export function findRestorableVersion(state) {
  if (!state?.activeVersionId) return null;
  return state.versions.find((version) => version.id === state.activeVersionId) || null;
}

// AKAR KEDUA: memilih versi di daftar dulu TIDAK memuatnya ke isian, tapi Save
// menulis isian ke versi yang sedang dipilih. Jadi "pilih JALAN MENUJU MEKKAH
// -> Save" menimpa MEKKAH dengan isi apa pun yang kebetulan ada di isian --
// default sesudah Studio baru dibuka, atau nilai versi lain. Sekarang memilih
// = memuat (App.jsx), dan Save hanya pernah menulis ke versi yang isinya
// sedang terbuka.
//
// Aturan nama, supaya Save tidak pernah menggantikan versi lain:
// - nama kosong atau sama dengan versi yang terbuka -> perbarui versi itu;
// - nama lain -> versi BARU (versi yang terbuka tidak diganti nama);
// - nama yang sudah dipakai versi lain -> ditolak.
export function planWalkingSave({ versions, activeVersionId, name, params, now, makeId }) {
  const active = versions.find((version) => version.id === activeVersionId) || null;
  const wanted = String(name ?? "").trim();
  if (active && (wanted === "" || wanted === active.name)) {
    return { mode: "update", version: { ...active, params, updatedAt: now } };
  }
  const finalName = wanted || `Walking ${new Date(now).toLocaleString()}`;
  if (versions.some((version) => version.name === finalName)) {
    throw new Error(`Nama "${finalName}" sudah dipakai versi lain. Pilih versi itu di daftar dulu `
      + "kalau memang mau menimpanya — Save tidak pernah menulis ke versi yang tidak sedang dibuka.");
  }
  return { mode: "new", version: { id: makeId(), name: finalName, createdAt: now, updatedAt: now, params } };
}

// Apakah isian berbeda dari versi yang sedang dibuka (atau dari default kalau
// belum ada versi yang dibuka). Toleransinya sama dengan Apply (1e-7 relatif),
// supaya pembacaan balik float32 sesudah Apply tidak dianggap "perubahan".
export function walkingDraftUnsaved({ draft, activeVersion, defaults }) {
  return !parametersEqual(draft, activeVersion ? activeVersion.params : defaults, WALKING_KEYS);
}

// Pemeriksaan KETAT sesudah Save: yang dikembalikan server harus sama persis.
export function sameWalkingParams(left, right) {
  return Boolean(left && right) && WALKING_KEYS.every((key) => left[key] === right[key]);
}
