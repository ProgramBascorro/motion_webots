#!/usr/bin/env python3
"""Bandingkan (dan salin) versi walking tersimpan di Bascorro Studio ke preset demo.

KENAPA ADA SKRIP INI
--------------------
Enam gerakan demo di `op3_demo/config/preset_walking.yaml` disetel operator di
Bascorro Studio, lalu disalin TANGAN ke berkas itu. Studio menyimpan versinya di
localStorage browser, bukan di server, jadi tidak ada yang otomatis. Menyalin 29
angka x 6 bagian dengan tangan itu tempat salah ketik bersarang -- dan sudah
pernah kejadian: `init_x_offset: -0.18` (harusnya -0.018) membuat IK kaki gagal
total dan log membanjir "IK produced non-finite leg_angle[2]".

Skrip ini membaca versinya langsung dari profil Firefox, membandingkannya dengan
yaml, DAN memeriksa apakah tiap preset masih terjangkau kaki sebelum menulis.

PAKAI
-----
    python3 scripts/sync_preset_from_studio.py            # lihat dulu (dry run)
    python3 scripts/sync_preset_from_studio.py --write    # tulis

preset_walking.yaml disimbolik-link dari install/, jadi TIDAK perlu colcon build.
Cukup jalankan ulang demo.

CATATAN
-------
* Nilai localStorage Firefox (LSNG) dipadatkan snappy blok mentah; dekompresornya
  ditulis di sini supaya tidak perlu pustaka tambahan.
* Browser MENAHAN tulisan sebentar. Kalau versi yang baru disimpan belum muncul,
  tutup tab Studio (atau Firefox) dulu lalu ulangi.
* Pemeriksaan jangkauan memakai geometri kaki OP3: paha 110,15 + betis 110,00 mm.
  Preset yang tidak terjangkau TIDAK ditulis, kecuali dipaksa dengan --izinkan-mustahil.
"""

import argparse
import json
import math
import os
import re
import sqlite3
import sys
import tempfile
from glob import glob

# nama bagian yaml -> nama versi di Bascorro Studio
PETA = [
    ("jalan", "JALAN MENUJU MEKKAH"),
    ("mencari_celah", "MENCARI CELAH"),
    ("putar_kiri", "Putar Kiri New"),
    ("putar_kanan", "Putar Kanan New"),
    ("geser_kiri", "KIRI MIN NEW"),
    ("geser_kanan", "KANAN MIN NEW"),
]

# op3_kinematics_dynamics: panjang tautan kaki kanan
PAHA_M, BETIS_M = 0.11015, 0.110
JANGKAUAN_M = PAHA_M + BETIS_M

YAML_PRESET = "src/ROBOTIS-OP3-Demo/op3_demo/config/preset_walking.yaml"
KUNCI_LS = "bascorro.walking_versions.v1"


def snappy_decompress(data: bytes) -> bytes:
    """Snappy blok mentah. Firefox memadatkan nilai localStorage dengan ini."""
    pos = shift = size = 0
    while True:
        b = data[pos]
        pos += 1
        size |= (b & 0x7F) << shift
        if not b & 0x80:
            break
        shift += 7
    out = bytearray()
    while pos < len(data):
        tag = data[pos]
        pos += 1
        kind = tag & 0x03
        if kind == 0:  # literal
            n = tag >> 2
            if n >= 60:
                extra = n - 59
                n = int.from_bytes(data[pos:pos + extra], "little")
                pos += extra
            n += 1
            out += data[pos:pos + n]
            pos += n
            continue
        if kind == 1:  # salinan, offset 1 bita
            length = 4 + ((tag >> 2) & 0x07)
            offset = ((tag >> 5) << 8) | data[pos]
            pos += 1
        elif kind == 2:  # salinan, offset 2 bita
            length = (tag >> 2) + 1
            offset = int.from_bytes(data[pos:pos + 2], "little")
            pos += 2
        else:  # salinan, offset 4 bita
            length = (tag >> 2) + 1
            offset = int.from_bytes(data[pos:pos + 4], "little")
            pos += 4
        start = len(out) - offset
        for i in range(length):  # boleh tumpang tindih, jangan diganti slice
            out.append(out[start + i])
    if len(out) != size:
        raise ValueError(f"snappy: panjang {len(out)} != {size}")
    return bytes(out)


def cari_data_sqlite(port: int) -> str:
    pola = [
        os.path.expanduser(f"~/snap/firefox/common/.mozilla/firefox/*/storage/default/"
                           f"http+++localhost+{port}/ls/data.sqlite"),
        os.path.expanduser(f"~/.mozilla/firefox/*/storage/default/"
                           f"http+++localhost+{port}/ls/data.sqlite"),
    ]
    hit = [p for g in pola for p in glob(g)]
    if not hit:
        raise SystemExit(
            f"Tidak menemukan localStorage Firefox untuk localhost:{port}.\n"
            "Buka Bascorro Studio di Firefox sekali, lalu ulangi.")
    return max(hit, key=os.path.getmtime)


def baca_versi(port: int):
    db = cari_data_sqlite(port)
    # disalin dulu: berkasnya bisa terkunci selagi Firefox jalan
    with tempfile.NamedTemporaryFile(suffix=".sqlite", delete=False) as tmp:
        salinan = tmp.name
        tmp.write(open(db, "rb").read())
    try:
        con = sqlite3.connect(salinan)
        row = con.execute("SELECT compression_type, value FROM data WHERE key=?",
                          (KUNCI_LS,)).fetchone()
        if row is None:
            raise SystemExit(f"Kunci {KUNCI_LS} tidak ada di {db}")
        comp, val = row
        teks = (snappy_decompress(val) if comp == 1 else val).decode("utf-8")
    finally:
        os.unlink(salinan)
    return db, {v["name"]: v for v in json.loads(teks)}


def terjangkau(p: dict) -> tuple:
    """Apakah kaki masih sampai pada init pose preset ini, termasuk langkah penuh."""
    z_turun = abs(p["init_z_offset"] - JANGKAUAN_M)
    terjauh = 0.0
    for sx in (0.0, p["x_move_amplitude"], -p["x_move_amplitude"]):
        for sy in (0.0, p["y_move_amplitude"] / 2, -p["y_move_amplitude"] / 2):
            terjauh = max(terjauh, math.hypot(p["init_x_offset"] + sx,
                                              math.hypot(sy - p["init_y_offset"] / 2, z_turun)))
    return terjauh <= JANGKAUAN_M, terjauh


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="tulis perubahannya")
    ap.add_argument("--port", type=int, default=5173, help="port Studio (bawaan 5173)")
    ap.add_argument("--izinkan-mustahil", action="store_true",
                    help="tulis walaupun preset di luar jangkauan kaki (JANGAN)")
    args = ap.parse_args()

    akar = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(akar)

    db, versi = baca_versi(args.port)
    print(f"localStorage : {db}\n")

    try:
        import yaml
    except ImportError:
        raise SystemExit("butuh PyYAML: pip install pyyaml")
    baris = open(YAML_PRESET, encoding="utf-8").read().split("\n")
    doc = yaml.safe_load("\n".join(baris))

    norm = lambda v: v if isinstance(v, bool) else float(v)
    rencana, ditolak, total = {}, [], 0

    for sec, nama in PETA:
        if nama not in versi:
            print(f"{sec:16} versi '{nama}' TIDAK ADA di Studio, dilewati")
            continue
        s = versi[nama]["params"]
        y = doc[sec]
        beda = [(k, y.get(k), s[k]) for k in sorted(s)
                if k not in y or norm(y[k]) != norm(s[k])]

        ok, butuh = terjangkau(s)
        tanda = "" if ok else f"  <- DI LUAR JANGKAUAN: butuh {butuh*1000:.1f} mm dari {JANGKAUAN_M*1000:.1f} mm"
        print(f"{sec:16} {len(beda)} beda{tanda}")
        for k, a, b in beda:
            print(f"    {k:30} {str(a):>10} -> {b}")
        if not ok and not args.izinkan_mustahil:
            if beda:
                ditolak.append(sec)
            continue
        if beda:
            rencana[sec] = {k: b for k, _, b in beda}
            total += len(beda)
        print()

    if ditolak:
        print(f"\nDITOLAK (tidak ditulis): {', '.join(ditolak)}")
        print("Perbaiki angkanya di Studio dulu. Paling sering: init_x_offset kelebihan "
              "satu angka nol, mis. -0.18 padahal maksudnya -0.018.")

    if not rencana:
        print("\nTidak ada yang perlu ditulis.")
        return 1 if ditolak else 0

    if not args.write:
        print(f"\n(dry run) {total} nilai siap ditulis; tambahkan --write")
        return 0

    cur = None
    sisa = {s: dict(v) for s, v in rencana.items()}
    for i, ln in enumerate(baris):
        m = re.match(r"^([a-z_]+):\s*$", ln)
        if m and m.group(1) in dict(PETA):
            cur = m.group(1)
            continue
        if cur in sisa:
            km = re.match(r"^(\s+)([a-z_]+):\s*(-?[\d.]+|true|false)\s*$", ln)
            if km and km.group(2) in sisa[cur]:
                nilai = sisa[cur].pop(km.group(2))
                if isinstance(nilai, bool):
                    nilai = "true" if nilai else "false"
                baris[i] = f"{km.group(1)}{km.group(2)}: {nilai}"

    hilang = {s: v for s, v in sisa.items() if v}
    if hilang:
        raise SystemExit(f"baris tidak ketemu di yaml (field baru?): {hilang}")

    open(YAML_PRESET, "w", encoding="utf-8").write("\n".join(baris))
    print(f"\nditulis: {YAML_PRESET} ({total} nilai)")
    print("preset_walking.yaml disimbolik-link dari install/, jadi tidak perlu colcon build.")
    return 1 if ditolak else 0


if __name__ == "__main__":
    sys.exit(main())
