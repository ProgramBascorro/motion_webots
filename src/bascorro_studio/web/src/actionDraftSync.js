// Kapan draft Action Editor (di browser) boleh diganti otomatis dengan isi .bin.
//
// Draft dipulihkan dari localStorage tiap Studio dibuka, BUKAN dari .bin. Kalau
// halaman diubah di luar Studio (op3_action_editor di terminal), draft itu basi
// tanpa suara: 24 Okt 2026 operator membuka Studio dan page 2 WALKING_READY masih
// isi sebelum di-teach ulang 22:39, padahal robot sudah memutar yang baru.
//
// Memuat ulang otomatis hanya aman kalau draft BELUM DIEDIT sejak terakhir sama
// dengan .bin -- kalau sudah, memuat ulang = menghapus kerja operator. Jadi saat
// export/apply sukses dicatat sidik isi draft + waktu .bin; saat dibuka, draft
// yang sidiknya masih sama dan .bin-nya lebih baru boleh diganti.

export const SYNCED_DRAFT_KEY = "op3ActionEditorSyncedDraft";

// FNV-1a 32-bit: cukup untuk "isinya masih persis sama", dan kecil -- kuota
// localStorage alamat ini sudah hampir penuh oleh riwayat undo editor.
export function textHash(text) {
  let hash = 0x811c9dc5;
  for (let i = 0; i < text.length; i += 1) {
    hash ^= text.charCodeAt(i);
    hash = Math.imul(hash, 0x01000193) >>> 0;
  }
  return `${text.length}:${hash.toString(16)}`;
}

export function syncedRecord(text, binPath, binMtimeMs) {
  return { hash: textHash(text), path: binPath, mtimeMs: binMtimeMs };
}

// "load"   editor kosong -> isi dari .bin (tidak ada yang tertimpa)
// "reload" .bin lebih baru, draft belum diedit -> ganti dengan .bin
// "edited" .bin lebih baru TAPI draft sudah diedit -> jangan sentuh, beri tahu
// "keep"   draft sama dengan .bin, atau draft lama tanpa catatan sinkron
// "wait"   waktu .bin belum diketahui
export function draftReloadDecision({ draftText, hasDraft, binPath, binMtimeMs, synced }) {
  if (!binMtimeMs) return "wait";
  if (!hasDraft || !draftText || !draftText.trim()) return "load";
  if (!synced || synced.path !== binPath) return "keep";
  if (binMtimeMs <= synced.mtimeMs + 2000) return "keep";
  return textHash(draftText) === synced.hash ? "reload" : "edited";
}
