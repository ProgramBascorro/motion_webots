#!/usr/bin/env python3
"""Bandingkan dua model bola: kecepatan DAN akurasi, berdampingan.

Dibuat untuk memutuskan apakah model INT8 layak dipakai, tapi berguna untuk
perbandingan model apa pun (ekspor baru, ukuran masukan baru, bobot baru).

Kenapa keduanya harus diukur bersama: kuantisasi INT8 SELALU lebih cepat dan
SELALU berpotensi kurang akurat. Mengukur salah satunya saja menghasilkan
keputusan yang salah. Skrip ini menolak menyimpulkan dari kecepatan sendirian.

Dua gerbang yang dipakai:
  * KECEPATAN -- CPU robot ini (i5-10210U) ber-AVX2 TANPA VNNI, jadi INT8
    biasanya cuma 1,3-1,8x, bukan 2x. Di bawah ~1,3x, percepatannya tidak
    sepadan dengan risiko akurasi.
  * AKURASI -- jumlah frame terdeteksi tidak boleh turun, dan conf minimum
    tidak boleh mendekati confidence_threshold (0,5). Conf minimum lebih
    penting daripada rata-rata: yang membuat pelacak kehilangan bola adalah
    frame terburuk, bukan frame rata-rata.

Pemakaian (di dalam container `op3`, dengan YOLO_AUTOINSTALL=false):
    scripts/compare_ball_models.py --video /ros2_ws/tes_infer.mp4
    scripts/compare_ball_models.py --frames <folder gambar>
    scripts/compare_ball_models.py --b <dir model lain>
"""

import argparse
import os
import sys
import time

WS = os.environ.get("OP3_WS", "/ros2_ws")
MODEL_DIR = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector/model")


def load_frames(video, frames_dir, limit):
    import cv2
    imgs = []
    if frames_dir:
        names = sorted(f for f in os.listdir(frames_dir)
                       if f.lower().endswith((".jpg", ".jpeg", ".png")))
        for n in names[:limit]:
            img = cv2.imread(os.path.join(frames_dir, n))
            if img is not None:
                imgs.append(img)
    else:
        cap = cv2.VideoCapture(video)
        while len(imgs) < limit:
            ok, f = cap.read()
            if not ok:
                break
            imgs.append(f)
        cap.release()
    return imgs


def run(path, imgs, imgsz, conf, iou):
    import numpy as np
    from ultralytics import YOLO
    model = YOLO(path, task="detect")
    model.predict(imgs[0], imgsz=imgsz, verbose=False)      # pemanasan
    hits, confs, rads = 0, [], []
    t0 = time.perf_counter()
    for img in imgs:
        r = model.predict(img, imgsz=imgsz, conf=conf, iou=iou,
                          max_det=10, verbose=False)[0]
        if len(r.boxes):
            hits += 1
            i = int(r.boxes.conf.argmax())
            confs.append(float(r.boxes.conf[i]))
            x1, y1, x2, y2 = r.boxes.xyxy[i].tolist()
            rads.append(max(x2 - x1, y2 - y1) / 2)
    ms = (time.perf_counter() - t0) / len(imgs) * 1000.0
    return {
        "ms": ms, "hits": hits, "n": len(imgs),
        "conf_mean": float(np.mean(confs)) if confs else 0.0,
        "conf_min": float(min(confs)) if confs else 0.0,
        "rad_mean": float(np.mean(rads)) if rads else 0.0,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", default=os.path.join(MODEL_DIR, "ball_yolov8n_384x640_openvino_model"),
                    help="model acuan (bawaan: FP32 384x640)")
    ap.add_argument("--b", default=os.path.join(MODEL_DIR, "ball_yolov8n_384x640_int8_openvino_model"),
                    help="model pembanding (bawaan: INT8 384x640)")
    ap.add_argument("--video", default=os.path.join(WS, "tes_infer.mp4"))
    ap.add_argument("--frames", default="", help="pakai folder gambar, bukan video")
    ap.add_argument("--limit", type=int, default=30)
    ap.add_argument("--imgsz", default="384,640",
                    help="tinggi,lebar -- harus sama dengan bentuk ekspor")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--speed-gate", type=float, default=1.3)
    args = ap.parse_args()

    imgsz = [int(x) for x in str(args.imgsz).split(",")]
    imgsz = tuple(imgsz) if len(imgsz) == 2 else imgsz[0]

    for d in (args.a, args.b):
        if not os.path.exists(d):
            print("Tidak ditemukan: %s" % d, file=sys.stderr)
            return 1

    imgs = load_frames(args.video, args.frames, args.limit)
    if not imgs:
        print("Tidak ada frame untuk diuji.", file=sys.stderr)
        return 1
    h, w = imgs[0].shape[:2]
    print("Sumber uji: %d frame, %dx%d\n" % (len(imgs), w, h))

    ra = run(args.a, imgs, imgsz, args.conf, args.iou)
    rb = run(args.b, imgs, imgsz, args.conf, args.iou)

    print("%-8s %10s %10s %10s %10s %10s" %
          ("MODEL", "ms/frame", "deteksi", "conf avg", "conf MIN", "radius"))
    for tag, r in (("A (acuan)", ra), ("B (baru)", rb)):
        print("%-8s %10.1f %6d/%-3d %10.3f %10.3f %10.1f" %
              (tag, r["ms"], r["hits"], r["n"], r["conf_mean"], r["conf_min"], r["rad_mean"]))
    print("\nA = %s" % os.path.basename(args.a))
    print("B = %s" % os.path.basename(args.b))

    speedup = ra["ms"] / rb["ms"] if rb["ms"] else 0.0
    print("\n%-22s %.2fx (gerbang: >= %.2fx)" % ("Percepatan:", speedup, args.speed_gate))

    ok_speed = speedup >= args.speed_gate
    ok_hits = rb["hits"] >= ra["hits"]
    ok_conf = rb["conf_min"] >= args.conf + 0.10        # marjin di atas ambang

    print("%-22s %s" % ("Gerbang kecepatan:", "LULUS" if ok_speed else "GAGAL"))
    print("%-22s %s (%d vs %d)" % ("Gerbang deteksi:",
                                   "LULUS" if ok_hits else "GAGAL", rb["hits"], ra["hits"]))
    print("%-22s %s (min %.3f, ambang %.2f)" % ("Gerbang conf minimum:",
                                                "LULUS" if ok_conf else "GAGAL",
                                                rb["conf_min"], args.conf))
    print()
    if ok_speed and ok_hits and ok_conf:
        print("PUTUSAN: B layak dipakai.")
        return 0
    if not ok_speed:
        print("PUTUSAN: TOLAK B -- percepatannya tidak sepadan dengan risiko akurasi.")
    else:
        print("PUTUSAN: TOLAK B -- lebih cepat, tapi akurasinya turun.")
    print("Satu berkas uji tidak cukup untuk memutuskan menerima; ulangi dengan")
    print("--frames dari kamera robot pada kondisi lapangan sebenarnya.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
