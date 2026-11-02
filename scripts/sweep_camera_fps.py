#!/usr/bin/env python3
"""Pilih FPS kamera (= batas pajan = batas blur) dari DETEKSI BOLA, bukan mata.

Kamera ALPHONSE dipakai dengan auto-exposure FIRMWARE (lihat komentar di
op3_ball_detector/launch/yolo_ball_detector_openvino.launch.py: mode manual
terukur jauh lebih gelap). Kenop trade-off yang dipatuhi firmware tinggal laju
frame: dengan exposure_dynamic_framerate=0, pajan tidak boleh melebihi periode
frame, dan anti-flicker 50 Hz menguncinya ke kelipatan 10 ms.

    20 FPS -> pajan <= 50 ms : paling terang, paling buram saat kepala menyapu
    24 FPS -> pajan <= 40 ms : yang dipakai sekarang
    30 FPS -> pajan <= 30 ms : sedikit lebih gelap, blur 25% lebih pendek

Skrip ini merekam frame dari kamera pada tiap laju BERGANTIAN (beberapa ronde,
supaya perubahan cahaya / posisi bola tidak menguntungkan satu laju saja), lalu
menjalankan bobot detektor yang sama dengan demo dan melaporkan per laju:
frame berbola, conf rata-rata, conf MINIMUM (yang membuat pelacak kehilangan
bola adalah frame terburuk), luma lapangan, dan ketajaman (variansi Laplacian).

SYARAT: tidak ada yang sedang memakai kamera (matikan stack / usb_cam dulu --
`fuser /dev/video0` harus kosong). Jalankan di dalam container `op3`:

    docker exec -it op3 bash -lc 'export YOLO_AUTOINSTALL=false; \\
        python3 /ros2_ws/scripts/sweep_camera_fps.py --label diam_2m'

Skenario yang disarankan (satu pemanggilan per skenario, label berbeda):
    bola diam 1 m / 2 m / 4 m, kepala diam      -> --label diam_2m
    kepala/kamera disapu kiri-kanan pelan-cepat -> --label sapu
Frame disimpan di --out/<label>/<fps>/ supaya bisa diulang tanpa merekam lagi
(--evaluate-only).
"""

import argparse
import os
import subprocess
import sys
import time

WS = os.environ.get("OP3_WS", "/ros2_ws")
PKG = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector")
DEFAULT_MODEL = os.path.join(PKG, "model/ball_gawang_v3_288x480_int8_openvino_model")


def ctl(dev, control):
    """Satu kontrol per panggilan: satu yang ditolak menggagalkan semuanya."""
    r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", control],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print("  [peringatan] v4l2-ctl %s: %s" % (control, r.stderr.strip()))


def read_ctl(dev, name):
    r = subprocess.run(["v4l2-ctl", "-d", dev, "-C", name],
                       capture_output=True, text=True)
    parts = r.stdout.split()
    return parts[-1] if parts else "?"


def luma_field(img):
    """Luma rata-rata 2/3 bawah frame -- sama dengan frame_stats[0] detektor."""
    import numpy as np
    sub = img[::8, ::8].astype(np.float32)
    luma = sub[..., 0] * 0.114 + sub[..., 1] * 0.587 + sub[..., 2] * 0.299
    return float(luma[luma.shape[0] // 3:, :].mean())


def sharpness(img):
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def capture(args, out_dir):
    import cv2
    rates = [int(r) for r in args.fps.split(",")]
    for rnd in range(args.rounds):
        for fps in rates:
            cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise SystemExit("Tidak bisa membuka %s -- masih dipakai usb_cam? "
                                 "Cek: fuser %s" % (args.device, args.device))
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            # Sama persis dengan launch detektor.
            ctl(args.device, "auto_exposure=3")
            ctl(args.device, "exposure_dynamic_framerate=0")
            ctl(args.device, "power_line_frequency=%d" % args.power_line)
            t_end = time.time() + args.settle
            while time.time() < t_end:          # biarkan AE konvergen
                cap.read()
            d = os.path.join(out_dir, str(fps))
            os.makedirs(d, exist_ok=True)
            t0 = time.time()
            n = 0
            while n < args.frames:
                ok, frame = cap.read()
                if not ok:
                    break
                cv2.imwrite(os.path.join(d, "r%02d_%04d.jpg" % (rnd, n)), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                n += 1
            real = n / max(1e-6, time.time() - t0)
            print("  ronde %d  %2d FPS: %d frame, laju nyata %.1f, pajan %s (x100 us)"
                  % (rnd + 1, fps, n, real, read_ctl(args.device, "exposure_time_absolute")))
            cap.release()


def evaluate(args, out_dir):
    import cv2
    import numpy as np
    from ultralytics import YOLO

    model = YOLO(args.model, task="detect")
    names = {str(v).lower(): int(k) for k, v in dict(model.names).items()}
    want = names.get(args.cls.lower())
    if want is None:
        raise SystemExit("kelas '%s' tidak ada di model (ada: %s)" % (args.cls, sorted(names)))
    imgsz = tuple(int(x) for x in args.imgsz.split(","))

    rates = sorted((d for d in os.listdir(out_dir) if d.isdigit()), key=int)
    if not rates:
        raise SystemExit("Tidak ada frame di %s" % out_dir)
    print("\nModel: %s  kelas=%s  conf>=%.2f  imgsz=%s" %
          (os.path.basename(args.model), args.cls, args.conf, imgsz))
    print("%-6s %8s %10s %9s %9s %8s %10s %10s" %
          ("FPS", "frame", "berbola", "conf avg", "conf MIN", "luma", "tajam", "tajam bola"))
    for fps in rates:
        d = os.path.join(out_dir, fps)
        files = sorted(f for f in os.listdir(d) if f.endswith(".jpg"))
        hits, confs, lumas, sharp, ball_sharp = 0, [], [], [], []
        for i, f in enumerate(files):
            img = cv2.imread(os.path.join(d, f))
            if img is None:
                continue
            if i == 0:
                model.predict(img, imgsz=imgsz, verbose=False)   # pemanasan
            lumas.append(luma_field(img))
            sharp.append(sharpness(img))
            r = model.predict(img, imgsz=imgsz, conf=args.conf, iou=0.45,
                              max_det=10, verbose=False)[0]
            idx = [j for j in range(len(r.boxes)) if int(r.boxes.cls[j]) == want]
            if idx:
                hits += 1
                j = max(idx, key=lambda k: float(r.boxes.conf[k]))
                confs.append(float(r.boxes.conf[j]))
                x1, y1, x2, y2 = (int(v) for v in r.boxes.xyxy[j].tolist())
                crop = img[max(0, y1):y2, max(0, x1):x2]
                if crop.size and min(crop.shape[:2]) >= 4:
                    ball_sharp.append(sharpness(crop))
        n = len(files)
        print("%-6s %8d %5d (%3.0f%%) %9.3f %9.3f %8.1f %10.0f %10s" % (
            fps, n, hits, 100.0 * hits / max(1, n),
            np.mean(confs) if confs else 0.0, min(confs) if confs else 0.0,
            np.mean(lumas) if lumas else 0.0, np.mean(sharp) if sharp else 0.0,
            "%.0f" % np.mean(ball_sharp) if ball_sharp else "-"))
    print("\nCara membaca: pilih laju dengan frame berbola TERBANYAK dan conf MIN")
    print("tertinggi pada skenario 'sapu'; skenario diam cuma memastikan laju itu")
    print("tidak kalah saat bola jauh (lebih gelap = conf bola jauh turun duluan).")
    print("Ganti framerate di op3_ball_detector/config/camera_param.yaml (30/24/20 saja).")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="nama skenario, mis. diam_2m / sapu")
    ap.add_argument("--fps", default="20,24,30",
                    help="laju yang dibandingkan; HANYA yang ditawarkan kamera "
                         "(30/24/20/15/10 di 640x360)")
    ap.add_argument("--rounds", type=int, default=3, help="ronde bergantian per laju")
    ap.add_argument("--frames", type=int, default=40, help="frame per laju per ronde")
    ap.add_argument("--settle", type=float, default=3.0, help="detik untuk AE konvergen")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--power-line", type=int, default=1, help="1 = 50 Hz (PLN)")
    ap.add_argument("--out", default=os.path.join(WS, "log/camera_fps_sweep"))
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--imgsz", default="288,480", help="tinggi,lebar ekspor model")
    ap.add_argument("--cls", default="ball")
    ap.add_argument("--conf", type=float, default=0.30,
                    help="sama dengan confidence_threshold detektor")
    ap.add_argument("--evaluate-only", action="store_true",
                    help="jangan rekam; nilai frame yang sudah tersimpan")
    args = ap.parse_args()

    out_dir = os.path.join(args.out, args.label)
    try:
        if not args.evaluate_only:
            print("Merekam '%s' ke %s" % (args.label, out_dir))
            capture(args, out_dir)
    finally:
        if not args.evaluate_only:
            # Kembalikan ke keadaan yang sama dengan launch detektor.
            ctl(args.device, "auto_exposure=3")
            ctl(args.device, "exposure_dynamic_framerate=0")
            ctl(args.device, "power_line_frequency=%d" % args.power_line)
    evaluate(args, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
