#!/usr/bin/env python3
"""Bandingkan dua model bola: kecepatan DAN akurasi, berdampingan.

Dibuat untuk memutuskan apakah model INT8 layak dipakai, tapi berguna untuk
perbandingan model apa pun (ekspor baru, ukuran masukan baru, bobot baru).

Kenapa keduanya harus diukur bersama: kuantisasi INT8 SELALU lebih cepat dan
SELALU berpotensi kurang akurat. Mengukur salah satunya saja menghasilkan
keputusan yang salah. Skrip ini menolak menyimpulkan dari kecepatan sendirian.

Dua gerbang yang dipakai:
  * KECEPATAN -- CPU robot ini (i5-1340P) PUNYA VNNI, jadi INT8 harusnya di
    atas 2x. Gerbangnya tetap dipasang di 1,3x sebagai lantai: di bawah itu,
    percepatannya tidak sepadan dengan risiko akurasi.
  * AKURASI -- jumlah frame yang lolos ambang produksi (0,5) tidak boleh
    turun, dan keyakinan rata-ratanya tidak boleh merosot di bawah model
    acuan. Keduanya diukur RELATIF terhadap model A. Lantai mutlak pernah
    dicoba dan dibuang: pada 200 frame beragam selalu ada satu deteksi yang
    duduk persis di atas ambang, sehingga lantai itu menggagalkan model acuan
    sendiri -- ia mengukur adanya frame sulit, bukan mutu model.

DIUKUR PADA GEOMETRI KAMERA ROBOT (--eval-size, bawaan 640x360). Ini bukan
detail: versi pertama skrip ini mengukur 30 frame pertama tes_infer.mp4 pada
ukuran aslinya (850x478), dan di situ bolanya jauh lebih besar daripada yang
pernah dilihat robot. Model INT8 yang rusak LULUS gerbang ini, lalu di robot
tidak mendeteksi apa pun. Diperkecil ke 640x360, model yang sama meloloskan
0 dari 547 frame sementara FP32 meloloskan 326. Mengukur pada ukuran asli
berarti mengukur kasus yang tidak pernah terjadi.

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
MODEL_DIR = os.path.join(
    WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector/model/ball_model")


def spread(items, limit):
    """Ambil `limit` unsur TERSEBAR MERATA, bukan `limit` unsur pertama.

    Frame berurutan di awal video adalah satu adegan yang sama: satu jarak,
    satu sudut, satu pencahayaan. Mengambil 200 frame pertama berarti
    mengukur satu kondisi 200 kali, bukan mengambil sampel.
    """
    n = len(items)
    if n <= limit:
        return list(items)
    return [items[(i * n) // limit] for i in range(limit)]


def load_frames(video, frames_dir, limit, eval_size):
    import cv2
    imgs = []
    if frames_dir:
        names = sorted(f for f in os.listdir(frames_dir)
                       if f.lower().endswith((".jpg", ".jpeg", ".png")))
        for n in spread(names, limit):
            img = cv2.imread(os.path.join(frames_dir, n))
            if img is not None:
                imgs.append(img)
    else:
        cap = cv2.VideoCapture(video)
        raw = []
        while True:
            ok, f = cap.read()
            if not ok:
                break
            raw.append(f)
        cap.release()
        imgs = spread(raw, limit)

    if eval_size:
        # Perkecil ke geometri kamera robot supaya bola berukuran sebesar yang
        # benar-benar dilihat robot. INTER_AREA, sama dengan yang dipakai
        # pipeline v4l2/letterbox saat mengecilkan.
        w, h = eval_size
        imgs = [cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA)
                if (im.shape[1], im.shape[0]) != (w, h) else im
                for im in imgs]
    return imgs


def run(path, imgs, imgsz, conf, iou, raw_conf):
    """Jalankan satu model pada semua frame.

    Inferensi dilakukan pada ambang RENDAH (raw_conf), lalu "hits" dihitung
    pada ambang produksi (conf). Kenapa dua ambang: kalau inferensinya sendiri
    sudah disaring di 0,5, maka conf terkecil yang tersisa selalu tepat di atas
    0,5 -- angkanya tidak mengukur apa pun. Dengan ambang rendah, sebaran
    keyakinan terlihat utuh dan MARJIN di atas ambang produksi bisa dibandingkan
    antar-model.
    """
    import numpy as np
    from ultralytics import YOLO
    model = YOLO(path, task="detect")
    model.predict(imgs[0], imgsz=imgsz, verbose=False)      # pemanasan
    hits, confs, rads = 0, [], []
    t0 = time.perf_counter()
    for img in imgs:
        r = model.predict(img, imgsz=imgsz, conf=raw_conf, iou=iou,
                          max_det=10, verbose=False)[0]
        if not len(r.boxes):
            continue
        i = int(r.boxes.conf.argmax())
        c = float(r.boxes.conf[i])
        if c < conf:                       # tidak lolos ambang produksi
            continue
        hits += 1
        confs.append(c)
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
    ap.add_argument("--limit", type=int, default=200,
                    help="jumlah frame yang diuji, diambil TERSEBAR MERATA "
                         "sepanjang sumber (bawaan 200)")
    ap.add_argument("--eval-size", default="640x360",
                    help="perkecil tiap frame ke geometri kamera robot "
                         "sebelum inferensi (bawaan 640x360). 'none' untuk "
                         "mengukur pada ukuran asli -- lihat docstring: itulah "
                         "cara meloloskan model yang sebenarnya rusak.")
    ap.add_argument("--imgsz", default="384,640",
                    help="tinggi,lebar -- harus sama dengan bentuk ekspor")
    # Membandingkan dua ekspor dengan BENTUK berbeda (640x640 lawan 384x640)
    # tidak bisa memakai satu --imgsz: OpenVINO menolak tensor yang bentuknya
    # tidak sama dengan yang dibakar saat ekspor. Kosong = ikut --imgsz.
    ap.add_argument("--imgsz-a", default="", help="bentuk khusus untuk model A")
    ap.add_argument("--imgsz-b", default="", help="bentuk khusus untuk model B")
    ap.add_argument("--conf", type=float, default=0.5,
                    help="ambang produksi, harus sama dengan "
                         "confidence_threshold di YAML detektor")
    ap.add_argument("--raw-conf", type=float, default=0.10,
                    help="ambang inferensi mentah; harus jauh di "
                         "bawah --conf supaya marjin terlihat")
    ap.add_argument("--conf-drop", type=float, default=0.05,
                    help="seberapa jauh conf rata-rata B boleh "
                         "turun di bawah A (bawaan 0,05)")
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--speed-gate", type=float, default=1.3)
    args = ap.parse_args()

    def parse_imgsz(spec):
        vals = [int(x) for x in str(spec).split(",")]
        return tuple(vals) if len(vals) == 2 else vals[0]

    imgsz = parse_imgsz(args.imgsz)
    imgsz_a = parse_imgsz(args.imgsz_a) if args.imgsz_a else imgsz
    imgsz_b = parse_imgsz(args.imgsz_b) if args.imgsz_b else imgsz

    if str(args.eval_size).strip().lower() in ("", "none", "0"):
        eval_size = None
    else:
        ew, eh = (int(v) for v in str(args.eval_size).lower().split("x"))
        eval_size = (ew, eh)

    for d in (args.a, args.b):
        if not os.path.exists(d):
            print("Tidak ditemukan: %s" % d, file=sys.stderr)
            return 1

    imgs = load_frames(args.video, args.frames, args.limit, eval_size)
    if not imgs:
        print("Tidak ada frame untuk diuji.", file=sys.stderr)
        return 1
    h, w = imgs[0].shape[:2]
    print("Sumber uji: %d frame tersebar merata, %dx%d%s\n" % (
        len(imgs), w, h,
        "" if eval_size else "  (UKURAN ASLI -- bola lebih besar daripada "
                             "yang dilihat robot, hasilnya optimistis)"))

    ra = run(args.a, imgs, imgsz_a, args.conf, args.iou, args.raw_conf)
    rb = run(args.b, imgs, imgsz_b, args.conf, args.iou, args.raw_conf)

    print("%-8s %10s %10s %10s %10s %10s" %
          ("MODEL", "ms/frame", "deteksi", "conf avg", "conf MIN", "radius"))
    for tag, r in (("A (acuan)", ra), ("B (baru)", rb)):
        print("%-8s %10.1f %6d/%-3d %10.3f %10.3f %10.1f" %
              (tag, r["ms"], r["hits"], r["n"], r["conf_mean"], r["conf_min"], r["rad_mean"]))
    print("\nA = %s  (imgsz %s)" % (os.path.basename(args.a), imgsz_a))
    print("B = %s  (imgsz %s)" % (os.path.basename(args.b), imgsz_b))

    speedup = ra["ms"] / rb["ms"] if rb["ms"] else 0.0
    print("\n%-22s %.2fx (gerbang: >= %.2fx)" % ("Percepatan:", speedup, args.speed_gate))

    ok_speed = speedup >= args.speed_gate
    ok_hits = rb["hits"] >= ra["hits"]
    # Dibandingkan terhadap A, bukan terhadap lantai mutlak. Lantai mutlak
    # ("conf minimum harus >= 0,60") ditolak setelah diukur: pada 200 frame
    # yang beragam SELALU ada satu deteksi yang duduk persis di atas ambang,
    # jadi lantai itu menggagalkan model acuan sekalipun -- ia mengukur
    # keberadaan frame sulit, bukan mutu model.
    ok_conf = rb["conf_mean"] >= ra["conf_mean"] - args.conf_drop

    print("%-22s %s" % ("Gerbang kecepatan:", "LULUS" if ok_speed else "GAGAL"))
    print("%-22s %s (%d vs %d)" % ("Gerbang deteksi:",
                                   "LULUS" if ok_hits else "GAGAL", rb["hits"], ra["hits"]))
    print("%-22s %s (avg %.3f vs %.3f, boleh turun %.2f)" % (
        "Gerbang conf rata2:", "LULUS" if ok_conf else "GAGAL",
        rb["conf_mean"], ra["conf_mean"], args.conf_drop))
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
