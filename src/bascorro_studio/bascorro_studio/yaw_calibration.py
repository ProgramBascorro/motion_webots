"""Kalibrasi yaw kompas OpenCR dari titik rujukan yang DIREKAM operator.

Tidak ada rumus yang diandaikan. Operator memutar robot ke lima posisi fisik
(0, kiri 90, kiri 180, kanan 90, kanan 180) dan merekam apa yang dilaporkan
sensor di tiap posisi. Pemetaan dihitung SEPENUHNYA dari rekaman itu, lalu
dipakai open_cr_module untuk mengubah heading kompas menjadi yaw.

Konvensi hasil: KIRI (berlawanan jarum jam) = POSITIF, KANAN = NEGATIF,
rentang (-180, +180], titik 0 = arah saat titik 0 direkam.

Dua sumber sinyal, dan kenapa bukan yaw firmware
------------------------------------------------
Yaw firmware TIDAK dipakai sebagai masukan pemetaan: ia = heading dikurangi
referensi yang dikunci SEKALI, 3 detik setelah OpenCR menyala, dan besar
referensi itu bergantung urutan penyalaan OpenCR vs servo (terukur meleset ~30
derajat antar-boot). Pemetaan di atasnya akan diam-diam salah pada boot
berikutnya.

Yang dipakai salah satu dari dua sinyal mentah, DIREKAM BERSAMAAN di tiap titik
supaya bisa dibandingkan dengan bukti, bukan dengan anggapan:

  "gyro"   -- yaw hasil integrasi gyro. Mulus, tanda konsisten, melayang
              ~0,38 derajat/menit.

              PENTING: integratornya MULAI DARI NOL setiap OpenCR menyala, jadi
              nolnya justru per-boot -- penyakit yang sama dengan yaw firmware,
              bukan kebal terhadapnya (catatan lama di berkas ini keliru soal
              ini, dan akibatnya terukur: h0 337,70 dari sesi perekaman membuat
              yaw start di +24,6 derajat setelah power-cycle, bukan 0).

              Karena itu h0 yang ditulis ke berkas HANYA sah dalam sesi yang
              sama dengan perekaman. Saat start, open_cr_module menambatkan
              ulang titik nol ke arah yang dihadapi robot saat itu; yang dipakai
              dari berkas adalah BENTUK kurvanya (jarak antar simpul = skala
              tiap kuadran), dan itu memang yang diukur kalibrasi ini.
  "compass"-- heading magnetometer. Absolut dan tidak melayang secara prinsip,
              TAPI di robot ini terukur: +33 derajat dalam 300 detik saat DIAM,
              melompat +-10 derajat per 0,2 detik saat gait, dan pada satu
              putaran kiri >180 derajat hanya berubah ~70 derajat ke arah yang
              SALAH (medan servo). Lihat memori kalibrasi-yaw-kompas-opencr.

Bawaannya "gyro" karena bukti di atas. Kalau kompas suatu saat diperbaiki
(kabel daya dirutekan ulang, hard-iron dikalibrasi ulang), rekam ulang dan
pilih "compass" -- struktur pemetaannya identik, hanya sinyal masukannya beda.

Bentuk pemetaan
---------------
1. h0 = heading saat titik 0 direkam.
2. Arah putar sensor DIBACA dari titik kiri 90: kalau heading bergerak ke arah
   negatif saat robot diputar ke kiri, tandanya dibalik. Tidak diandaikan.
3. Tiap rekaman diubah ke posisi melingkar p = (arah * (heading - h0)) mod 360,
   sehingga 0 -> 0, kiri 90 -> ~90, belakang -> ~180, kanan 90 -> ~270.
4. Titik simpul (p -> target) dirangkai linear sepotong-sepotong mengelilingi
   lingkaran. Ini meluruskan arah, skala, DAN galat yang bergantung arah
   (hard-iron -- "0 benar, 180 meleset") sekaligus, karena tiap kuadran punya
   potongan sendiri.
5. Kiri 180 dan kanan 180 adalah arah fisik yang SAMA (menghadap belakang).
   Keduanya dipakai sebagai simpul di sisi masing-masing dan dipetakan ke 180;
   di situlah sambungan +180/-180 berada, jadi mendekat dari kiri memberi
   +179 dan dari kanan memberi -179 -- tanpa loncatan di tempat lain.
"""

import math
import os
from typing import Dict, List, Optional, Tuple

# Urutan tampilan. Kunci penyimpanan memakai bentuk teks dari bilangan bulat ini.
TARGETS: Tuple[int, ...] = (0, 90, 180, -90, -180)

TARGET_LABEL = {
    0: "Posisi awal",
    90: "Kiri 90",
    180: "Kiri 180",
    -90: "Kanan 90",
    -180: "Kanan 180",
}

# Rekaman lebih kasar dari ini berarti robot masih bergerak saat direkam.
# Nilainya TETAP disimpan apa adanya -- operator yang memutuskan rekam ulang.
SPREAD_WARN_DEG = 2.0

# Kurang dari ini berarti robot praktis tidak berputar dari titik 0.
MIN_TURN_DEG = 10.0

DEFAULT_TXT_PATH = "/ros2_ws/src/ROBOTIS-OP3/open_cr_module/config/yaw_calibration.txt"

# Nama sinyal masukan. Harus sama persis dengan yang dibaca yaw_calibration.h.
SOURCES = ("gyro", "compass")
DEFAULT_SOURCE = "gyro"

# Kunci rekaman per titik: nilai mentah tiap sumber, direkam bersamaan.
SOURCE_FIELD = {"gyro": "gyro", "compass": "heading"}


class CalibrationError(Exception):
    """Rekaman tidak bisa dijadikan pemetaan. Pesannya untuk operator."""


def calibration_txt_path() -> str:
    # Satu variabel lingkungan dengan bawaan yang sama di C++ (open_cr_module),
    # supaya penulis dan pembaca tidak mungkin menunjuk berkas yang berbeda.
    return os.environ.get("OP3_YAW_CALIB_FILE", DEFAULT_TXT_PATH)


def wrap_pm180(angle: float) -> float:
    """Rapikan ke (-180, +180]. +180 dipertahankan positif (sisi kiri)."""
    r = math.fmod(angle + 180.0, 360.0)
    if r < 0.0:
        r += 360.0
    r -= 180.0
    return 180.0 if r == -180.0 else r


def wrap360(angle: float) -> float:
    r = math.fmod(angle, 360.0)
    return r + 360.0 if r < 0.0 else r


def circular_mean_deg(values: List[float]) -> Tuple[float, float]:
    """Rerata melingkar dan sebaran (derajat).

    Rerata aritmetika salah untuk sudut: 359 dan 1 rerata aritmetikanya 180,
    padahal keduanya berjarak 2 derajat di sekitar 0. Heading kompas sering
    berada tepat di sambungan 0/360, jadi ini wajib.
    """
    if not values:
        raise ValueError("tidak ada sampel")
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    mean = wrap360(math.degrees(math.atan2(s, c)))
    r = math.hypot(s, c) / len(values)
    spread = math.degrees(math.sqrt(-2.0 * math.log(r))) if r < 1.0 else 0.0
    return mean, spread


def raw_values(points: Dict[int, dict], source: str) -> Dict[int, float]:
    """Ambil sinyal mentah `source` dari rekaman titik. Titik tanpa nilai untuk
    sumber itu sengaja DILEWATI, supaya pesan galatnya menyebut titik mana."""
    field = SOURCE_FIELD[source]
    out = {}
    for t, rec in points.items():
        v = rec.get(field)
        if v is not None:
            out[int(t)] = float(v)
    return out


def compute_mapping(headings: Dict[int, float], source: str = DEFAULT_SOURCE) -> dict:
    """Hitung pemetaan dari lima nilai mentah terekam. Melempar CalibrationError."""
    if source not in SOURCES:
        raise CalibrationError(f"Sumber tidak dikenal: {source}")
    missing = [t for t in TARGETS if t not in headings]
    if missing:
        names = ", ".join(TARGET_LABEL[t] for t in missing)
        raise CalibrationError(f"Titik belum direkam: {names}")

    h0 = headings[0]
    rel = {t: wrap_pm180(headings[t] - h0) for t in TARGETS}

    # Arah putar sensor dibaca dari rekaman kiri 90 -- bukan diandaikan.
    if abs(rel[90]) < MIN_TURN_DEG:
        raise CalibrationError(
            f"Titik kiri 90 hanya {rel[90]:+.1f} derajat dari titik 0: robot "
            "tampaknya tidak berputar. Rekam ulang titik kiri 90.")
    sign = 1 if rel[90] > 0.0 else -1

    # Kanan 90 harus di sisi yang berlawanan dengan kiri 90.
    if sign * rel[-90] >= 0.0:
        raise CalibrationError(
            f"Kanan 90 ({rel[-90]:+.1f}) searah dengan kiri 90 ({rel[90]:+.1f}) "
            "relatif titik 0. Salah satu titik direkam di sisi yang keliru.")
    if abs(rel[-90]) < MIN_TURN_DEG:
        raise CalibrationError(
            f"Titik kanan 90 hanya {rel[-90]:+.1f} derajat dari titik 0: robot "
            "tampaknya tidak berputar. Rekam ulang titik kanan 90.")

    pos = {t: wrap360(sign * (headings[t] - h0)) for t in TARGETS}
    pos[0] = 0.0
    p90, p270 = pos[90], pos[-90]
    # Kiri 180 dan kanan 180 adalah arah fisik yang sama; derau menentukan mana
    # yang sedikit lebih kecil. Urutkan supaya simpulnya tetap naik.
    pb_lo, pb_hi = sorted((pos[180], pos[-180]))

    if not (0.0 < p90 < pb_lo <= pb_hi < p270 < 360.0):
        raise CalibrationError(
            "Urutan titik mengelilingi lingkaran tidak runtut "
            f"(0 -> kiri90 {p90:.1f} -> belakang {pb_lo:.1f}/{pb_hi:.1f} -> "
            f"kanan90 {p270:.1f} -> 360). Biasanya berarti robot diputar lebih "
            "dari yang dimaksud, atau titik direkam tertukar.")

    knots = [
        (0.0, 0.0),
        (p90, 90.0),
        (pb_lo, 180.0),
        (pb_hi, 180.0),
        (p270, 270.0),
        (360.0, 360.0),
    ]
    # Kedua rekaman "belakang" adalah arah fisik yang sama, jadi jarak di antara
    # keduanya adalah ukuran ketidakulangan sensor: makin besar, makin tidak bisa
    # dipercaya. Dipakai untuk membandingkan gyro vs kompas dengan angka.
    return {"source": source, "h0_deg": h0, "sign": sign, "knots": knots,
            "back_gap_deg": pb_hi - pb_lo}


def map_heading(cal: dict, heading: float, trim: float = 0.0) -> float:
    """Heading kompas -> yaw terkoreksi di (-180, +180].

    HARUS identik dengan applyYawCalibration() di open_cr_module
    (include/open_cr_module/yaw_calibration.h).
    """
    p = wrap360(cal["sign"] * (heading - cal["h0_deg"]))
    knots = cal["knots"]
    t = 0.0
    for (p0, t0), (p1, t1) in zip(knots, knots[1:]):
        if p0 <= p <= p1:
            t = t0 if p1 - p0 <= 1e-9 else t0 + (t1 - t0) * (p - p0) / (p1 - p0)
            break
    return wrap_pm180(t - trim)


def display_for_target(target: int, value: float) -> float:
    """Tampilkan sambungan belakang di sisi target: -180 bukan +180 untuk kanan."""
    if target < 0 and value > 90.0:
        return value - 360.0
    return value


def write_calibration_txt(cal: dict, path: str) -> None:
    """Tulis berkas yang dibaca open_cr_module. Atomik: tmp lalu ganti nama."""
    lines = [
        "# Kalibrasi yaw kompas OpenCR -- DIBUAT OTOMATIS oleh Bascorro Studio",
        "# (bascorro_studio/yaw_calibration.py). Jangan disunting tangan; rekam",
        "# ulang lewat kartu Kalibrasi Yaw. Hapus berkas ini untuk kembali ke",
        "# perilaku lama (yaw firmware + YAW_OFFSET_DEG_DEFAULT).",
        "#",
        "# Konvensi: kiri = positif, kanan = negatif.",
        "# h0_deg: nilai mentah saat titik 0 direkam. Untuk sumber gyro angka ini",
        "#         HANYA sah di sesi perekaman -- integrator gyro mulai dari nol",
        "#         tiap OpenCR menyala. open_cr_module menambatkan ulang titik nol",
        "#         ke arah saat start; yang dipakai dari sini bentuk kurvanya.",
        "# source : sinyal masukan -- gyro (yaw integrasi gyro) atau compass",
        "#          (heading magnetometer). open_cr_module memilih sinyalnya",
        "#          berdasarkan baris ini.",
        "# knot <p> <t> : p = sign*(mentah-h0) mod 360, t = target 0..360.",
        "version 1",
        f"source {cal.get('source', DEFAULT_SOURCE)}",
        f"h0_deg {cal['h0_deg']:.6f}",
        f"sign {int(cal['sign']):+d}",
    ]
    for p, t in cal["knots"]:
        lines.append(f"knot {p:.6f} {t:.6f}")
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    os.replace(tmp, path)


def build_table(points: Dict[int, dict], cal: Optional[dict],
                verify: Optional[Dict[int, dict]] = None) -> List[dict]:
    """Baris tabel hasil untuk operator.

    points[t] berisi 'heading' (masukan pemetaan) dan 'yaw_old' (yaw yang
    ditampilkan sistem LAMA di posisi itu). Kolom 'terkoreksi' diambil dari
    rekaman VERIFIKASI kalau ada -- itulah galat yang sebenarnya. Tanpa
    verifikasi, kolom itu adalah pemetaan yang dikenakan pada rekamannya
    sendiri, dan galatnya NOL MENURUT KONSTRUKSI; field 'source' membedakannya.
    """
    rows = []
    source = (cal or {}).get("source", DEFAULT_SOURCE)
    field = SOURCE_FIELD.get(source, "gyro")
    yaw_old_0 = points.get(0, {}).get("yaw_old")
    for t in TARGETS:
        rec = points.get(t)
        row = {"target": t, "label": TARGET_LABEL[t]}
        if rec is None:
            rows.append(row)
            continue
        row["source"] = source
        row["raw"] = rec.get(field)
        row["heading"] = rec.get("heading")
        row["gyro"] = rec.get("gyro")
        row["spread"] = rec.get("spread_" + field, rec.get("spread"))
        if yaw_old_0 is not None and rec.get("yaw_old") is not None:
            old_rel = display_for_target(t, wrap_pm180(rec["yaw_old"] - yaw_old_0))
            row["yaw_old_rel"] = old_rel
            row["err_old"] = wrap_pm180(old_rel - t)
        ver = (verify or {}).get(t)
        if ver is not None and ver.get("yaw_new") is not None:
            corrected = display_for_target(t, ver["yaw_new"])
            row["basis"] = "verifikasi"
        elif cal is not None and row["raw"] is not None:
            corrected = display_for_target(t, map_heading(cal, row["raw"]))
            row["basis"] = "rekaman"
        else:
            rows.append(row)
            continue
        row["yaw_new"] = corrected
        row["err_new"] = wrap_pm180(corrected - t)
        rows.append(row)
    return rows
