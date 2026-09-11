// Bagian MURNI dari penyimpanan "Versi Tersimpan" di tab Walking, dipisahkan
// dari App.jsx supaya perilakunya bisa diuji tanpa React maupun localStorage.
// App.jsx tetap memegang plumbing-nya: kunci localStorage, penanda seed, dan
// penulisan salinan penyelamat.

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
