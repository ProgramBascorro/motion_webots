import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

// App.jsx punya satu fungsi render per halaman (renderWalking, renderDemo, ...)
// dan MASING-MASING mendeklarasikan const lokalnya sendiri -- termasuk beberapa
// yang namanya mirip, seperti walkingButtonBase dan demoButtonBase.
//
// 2026-09-08 sebuah tombol baru di satu halaman memakai walkingButtonBase, yang
// cuma ada di dalam renderWalking. Itu ReferenceError saat render: tab Tendangan
// jadi LAYAR PUTIH total.
//
// `vite build` tidak menangkapnya (sah secara sintaks) dan tes lain juga tidak
// (tidak ada yang merender komponen). Uji ini menutup celah itu dengan
// pemeriksaan statis: tidak boleh ada fungsi render yang memakai nama yang
// dideklarasikan di dalam fungsi render LAIN.
const SUMBER = new URL("../src/App.jsx", import.meta.url);

// Dua bentuk yang dipakai App.jsx: `=> {` (ada const lokal) dan `=> (` (JSX
// langsung). Keduanya harus ikut diperiksa -- yang kedua tidak mendeklarasikan
// apa apa, tapi tetap BISA memakai nama milik fungsi lain.
function blokRender(teks) {
  const blok = new Map();
  const pola = /\n  const (render[A-Za-z0-9_]*) = \([^)]*\) => ([{(])/g;
  let m;
  while ((m = pola.exec(teks)) !== null) {
    const buka = m[2];
    const tutup = buka === "{" ? "}" : ")";
    const i = m.index + m[0].length - 1;
    let dalam = 0;
    for (let j = i; j < teks.length; j += 1) {
      const c = teks[j];
      if (c === buka) dalam += 1;
      else if (c === tutup) {
        dalam -= 1;
        if (dalam === 0) {
          blok.set(m[1], teks.slice(i, j + 1));
          break;
        }
      }
    }
  }
  return blok;
}

// HANYA const tingkat atas fungsi render (indentasi tepat 4 spasi). Yang di
// dalam callback bersarang punya scope-nya sendiri dan namanya sering umum
// ("value", "now"); ikut menghitungnya menghasilkan positif palsu di mana-mana.
const lokalDari = (badan) =>
  new Set([...badan.matchAll(/\n {4}const ([A-Za-z_$][\w$]*)\s*=/g)].map((m) => m[1]));

test("no render function uses a const that belongs to another render function", () => {
  const teks = readFileSync(SUMBER, "utf8");
  const blok = blokRender(teks);
  assert.ok(blok.size >= 5, `harus menemukan fungsi render, dapat ${blok.size}`);

  const lokal = new Map([...blok].map(([nama, badan]) => [nama, lokalDari(badan)]));
  const pelanggaran = [];

  for (const [nama, badan] of blok) {
    const milikSendiri = lokal.get(nama);
    // Lookbehind menolak akses properti: `Date.now()` bukan pemakaian
    // variabel bernama `now`, dan `row.value` bukan pemakaian `value`.
    const dipakai = new Set(
      [...badan.matchAll(/(?<![.\w$])([A-Za-z_$][\w$]*)/g)].map((m) => m[1]));
    for (const [lain, namaLain] of lokal) {
      if (lain === nama) continue;
      for (const kandidat of namaLain) {
        if (milikSendiri.has(kandidat)) continue;   // punya salinan sendiri: aman
        if (dipakai.has(kandidat)) pelanggaran.push(`${nama} memakai ${kandidat} milik ${lain}`);
      }
    }
  }

  assert.deepEqual(pelanggaran, [],
    "identifier dari fungsi render lain akan jadi ReferenceError dan layar putih:\n  "
    + pelanggaran.join("\n  "));
});

test("JSX braces and tags in App.jsx stay balanced", () => {
  const teks = readFileSync(SUMBER, "utf8");
  const blok = blokRender(teks);
  for (const [nama, badan] of blok) {
    // blokRender() sudah memasangkan kurung kurawal; kalau tidak seimbang,
    // badannya tidak akan pernah ditutup dan panjangnya membengkak ke akhir
    // berkas. Ini penjaga murah supaya kegagalan itu terbaca sebagai nama
    // fungsi, bukan sebagai "Unterminated regular expression" di baris terakhir.
    assert.ok(/[})]$/.test(badan), `${nama}: badan fungsi tidak tertutup rapi`);
    assert.ok(badan.length < teks.length * 0.8, `${nama}: badan fungsi menelan sisa berkas`);
  }
});
