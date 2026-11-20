#!/usr/bin/env python3
"""Periksa mutu set frame kalibrasi INT8, sebelum ia merusak sebuah model.

Ada karena pernah terjadi: 300 frame kalibrasi terekam dengan lensa tertutup
(kecerahan rata-rata ~7 dari 255). Kuantisasi pasca-latih menentukan rentang
aktivasi tiap lapisan dengan MENJALANKAN model pada frame-frame itu, jadi set
yang hitam mengajarkan rentang mendekati nol; begitu melihat adegan nyata yang
terang, aktivasinya terpotong. Model INT8 yang lahir dari situ tetap terlihat
wajar pada bola besar, tapi pada geometri kamera robot ia meloloskan 0 dari
547 frame di ambang 0,5 -- lawan 326 untuk FP32.

Kegagalan seperti itu senyap: tidak ada yang error, modelnya cuma buta. Karena
itu pemeriksaan ini dipasang di DUA tempat -- saat merekam dan saat
mengkuantisasi -- sebab berkas kalibrasi bisa datang dari mana saja.

Dipakai oleh scripts/capture_calib_frames.py dan scripts/quantize_ball_int8.py.
"""

# Di bawah ini praktis gelap: bukan "agak kurang cahaya", tapi lensa tertutup
# atau ruangan tanpa lampu.
DARK_MEAN = 20.0
# Sebagian kecil frame gelap itu wajar (bayangan, kepala menunduk). Sepersepuluh
# set ke atas berarti perekamannya yang salah, bukan adegannya.
MAX_DARK_FRACTION = 0.10
# Simpangan baku kecerahan ANTAR-frame. Di bawah ini, seluruh set adalah satu
# adegan diam: 300 salinan satu kondisi, bukan 300 sampel.
MIN_SPREAD = 6.0


def frame_brightness(img):
    """Kecerahan rata-rata satu frame BGR, 0..255."""
    return float(img.mean())


def check(means):
    """Nilai daftar kecerahan rata-rata per frame.

    Mengembalikan (ok, laporan) -- laporan adalah daftar baris siap cetak.
    """
    lines = []
    if not means:
        return False, ["Tidak ada frame untuk diperiksa."]

    n = len(means)
    avg = sum(means) / n
    spread = (sum((m - avg) ** 2 for m in means) / n) ** 0.5
    dark = sum(1 for m in means if m < DARK_MEAN)
    dark_frac = dark / n

    lines.append("Mutu set kalibrasi: %d frame, kecerahan rata-rata %.1f/255, "
                 "sebaran %.1f, gelap %d (%.0f%%)"
                 % (n, avg, spread, dark, 100 * dark_frac))

    ok = True
    if dark_frac > MAX_DARK_FRACTION:
        ok = False
        lines.append("")
        lines.append("SET INI DITOLAK -- TERLALU GELAP.")
        lines.append("  %.0f%% frame di bawah kecerahan %.0f. Lensa tertutup, "
                     "atau ruangan gelap." % (100 * dark_frac, DARK_MEAN))
        lines.append("  Kuantisasi dengan set ini menghasilkan model yang "
                     "cepat tapi buta -- dan tidak ada pesan error yang akan")
        lines.append("  memberitahu Anda. Nyalakan lampu, buka penutup lensa, "
                     "lalu rekam ulang.")
    elif spread < MIN_SPREAD:
        ok = False
        lines.append("")
        lines.append("SET INI DITOLAK -- SATU ADEGAN DIAM.")
        lines.append("  Sebaran kecerahan antar-frame cuma %.1f. Kamera tidak "
                     "digerakkan, atau kondisinya tidak berubah." % spread)
        lines.append("  Kalibrasi butuh RAGAM: bola dekat sampai jauh "
                     "(terutama yang jauh dan kecil), berbagai sudut,")
        lines.append("  berbagai pencahayaan, plus beberapa frame tanpa bola.")

    return ok, lines


def check_dir(path):
    """Periksa satu folder JPEG/PNG. Mengembalikan (ok, laporan)."""
    import os
    import cv2

    names = sorted(f for f in os.listdir(path)
                   if f.lower().endswith((".jpg", ".jpeg", ".png")))
    means = []
    for n in names:
        img = cv2.imread(os.path.join(path, n))
        if img is not None:
            means.append(frame_brightness(img))
    return check(means)
