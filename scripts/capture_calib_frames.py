#!/usr/bin/env python3
"""Rekam frame dari kamera robot untuk kalibrasi kuantisasi INT8.

Kenapa dari kamera robot, bukan dari dataset latih: kuantisasi pasca-latih
(PTQ) menentukan rentang nilai tiap lapisan dengan MENJALANKAN model pada
contoh masukan nyata. Contoh yang dipakai harus semirip mungkin dengan yang
akan dilihat model saat bertugas -- pencahayaan lapangan, jarak bola, sudut
kepala, derau sensor, artefak MJPEG. Frame dari kamera ini sendiri memenuhi
itu lebih baik daripada dataset latih yang direkam dengan kamera lain.
(Dataset latihnya juga tidak ada di mesin ini; metadata model menunjuk
`F:\\Robosoccer\\...` di mesin Windows.)

Yang perlu Anda lakukan saat merekam: PASTIKAN LAMPU MENYALA dan lensa tidak
tertutup, lalu gerakkan kamera menyapu lapangan dengan bola di berbagai JARAK,
berbagai SUDUT, dan kalau bisa berbagai PENCAHAYAAN. Sertakan juga beberapa
frame tanpa bola.

Perbanyak bola yang JAUH DAN KECIL. Itu kasus pertama yang hilang ketika
kuantisasi meleset, dan justru kasus itulah yang dipakai robot untuk menemukan
bola sebelum mendekat. Bola besar di depan hidung tetap terdeteksi oleh model
yang sudah rusak sekalipun, jadi set yang isinya bola dekat semua tidak
memberitahu apa-apa.

Skrip ini MENOLAK set yang gelap atau yang isinya satu adegan diam, dan keluar
dengan kode 1 -- lihat scripts/calib_quality.py untuk sebabnya.

Kamera harus BEBAS: hentikan `usb_cam` dulu, perangkat V4L2 tidak bisa dibuka
dua kali.

Pemakaian:
    docker exec -it op3 python3 /ros2_ws/scripts/capture_calib_frames.py
    scripts/capture_calib_frames.py                 # 300 frame, 2 FPS
    scripts/capture_calib_frames.py --count 500 --fps 3
    scripts/capture_calib_frames.py --out /path/lain

Frame disimpan sebagai JPEG kualitas 95 pada resolusi kamera apa adanya
(640x360). Jangan diubah ukurannya di sini -- pra-proses letterbox dilakukan
oleh skrip kuantisasi supaya sama persis dengan yang dilakukan Ultralytics
saat inferensi.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_quality  # noqa: E402

WS = os.environ.get("OP3_WS") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(
    WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector/model/ball_model/calib_frames")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--count", type=int, default=300, help="jumlah frame (bawaan 300)")
    ap.add_argument("--fps", type=float, default=2.0,
                    help="frame per detik yang DISIMPAN (bawaan 2 -- pelan, "
                         "supaya frame berurutan tidak kembar)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("cv2 tidak ada. Jalankan di dalam container: "
              "docker exec -it op3 python3 /ros2_ws/scripts/capture_calib_frames.py",
              file=sys.stderr)
        return 1

    os.makedirs(args.out, exist_ok=True)

    idx = args.device
    if isinstance(idx, str) and idx.startswith("/dev/video"):
        try:
            idx = int(idx.replace("/dev/video", ""))
        except ValueError:
            pass
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("Gagal membuka %s. Kamera dipakai node lain? "
              "Hentikan usb_cam dulu." % args.device, file=sys.stderr)
        return 1

    period = 1.0 / max(0.1, args.fps)
    print("Merekam %d frame ke %s" % (args.count, args.out))
    print("GERAKKAN kamera: bola di berbagai jarak, sudut, dan pencahayaan.")
    print("Sertakan beberapa frame TANPA bola. Ctrl-C untuk berhenti lebih awal.\n")

    saved = 0
    means = []
    try:
        next_at = time.monotonic()
        while saved < args.count:
            ok, frame = cap.read()
            if not ok:
                continue
            now = time.monotonic()
            if now < next_at:
                continue
            next_at = now + period
            path = os.path.join(args.out, "calib_%04d.jpg" % saved)
            cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            means.append(calib_quality.frame_brightness(frame))
            saved += 1
            if saved % 10 == 0 or saved == args.count:
                sys.stdout.write("\r  %d/%d frame" % (saved, args.count))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n  dihentikan.")
    finally:
        cap.release()

    print("\nSelesai: %d frame di %s" % (saved, args.out))
    if saved < 100:
        print("PERINGATAN: di bawah 100 frame biasanya terlalu sedikit untuk "
              "kalibrasi yang stabil.")

    # Diperiksa DI SINI, saat Anda masih berdiri di depan kamera dan bisa
    # merekam ulang -- bukan nanti setelah modelnya terlanjur dikuantisasi dan
    # gagal senyap di lapangan. Lihat scripts/calib_quality.py.
    ok, lines = calib_quality.check(means)
    print()
    for line in lines:
        print(line)
    if not ok:
        print("\nSet TIDAK dipakai apa adanya. Perbaiki kondisinya, lalu "
              "jalankan ulang perintah yang sama.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
