#!/usr/bin/env python3
"""Gerbang akurasi + kecepatan model GAWANG, dinilai terhadap LABEL, bukan tebakan.

Pasangan dari compare_ball_models.py. Bedanya: skrip bola cuma menghitung
"berapa frame ada kotaknya" karena dulu tidak ada label sama sekali. Untuk
gawang ada set berlabel (val+test dataset ball_gawang_v3, 226 gambar, satu
kotak gawang per gambar), jadi di sini kotaknya benar-benar dicocokkan dengan
label lewat IoU. Hitungan "ada kotak" saja tidak bisa membedakan kotak gawang
yang benar dari kotak yang meleset ke tiang lampu.

Tiga hal diukur per model, dan ketiganya diperlukan:
  * AP50 / AP50-95 pada set berlabel -- kualitas keseluruhan, tak bergantung
    ambang.
  * Presisi, recall dan conf MINIMUM pada ambang kerja (--conf) -- yang
    sungguh dialami node. Conf minimum penting karena gawang yang hilang di
    frame terburuk-lah yang membuat pemakainya kehilangan arah.
  * Deteksi palsu pada frame kamera ROBOT yang TIDAK berisi gawang (--neg) --
    set berlabel semuanya dari Roboflow, bukan dari kamera ini, jadi sisi
    "jangan melihat gawang di dinding" harus diuji dengan frame robot sendiri.
    Kecepatan diukur di pass ini juga, karena frame 640x360 inilah yang
    sungguh masuk ke node.

Pemakaian (container `op3` atau container --rm dari image yang sama):
    YOLO_AUTOINSTALL=false python3 scripts/eval_goal_model.py \\
        --model <dir IR A> --model <dir IR B> [--imgsz 288,480] [--conf 0.40]

Bawaan --images/--labels/--neg menunjuk ke model/eval_gawang_v3 dan dua
folder frame kalibrasi robot (keduanya tanpa gawang). Model ditaruh di
model/ yang di-.gitignore, jadi folder itu cuma ada di mesin ini.
"""

import argparse
import os
import sys
import time

WS = os.environ.get("OP3_WS", "/ros2_ws")
MODEL_DIR = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector/model")
DEFAULT_EVAL = os.path.join(MODEL_DIR, "eval_gawang_v3")
DEFAULT_NEG = [os.path.join(MODEL_DIR, "calib_frames_demo_2026-09-18"),
               os.path.join(MODEL_DIR, "calib_frames")]
SWEEP = (0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60)


def list_images(folder):
    return sorted(f for f in os.listdir(folder)
                  if f.lower().endswith((".jpg", ".jpeg", ".png")))


def load_labels(path, cls_id, w, h):
    """Kotak berkelas cls_id dari berkas label YOLO, dalam piksel xyxy."""
    boxes = []
    if not os.path.isfile(path):
        return boxes
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) < 5 or int(float(parts[0])) != cls_id:
                continue
            cx, cy, bw, bh = (float(v) for v in parts[1:5])
            boxes.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                          (cx + bw / 2) * w, (cy + bh / 2) * h))
    return boxes


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1])
             - inter)
    return inter / union if union > 0 else 0.0


def match(preds, gts, thr):
    """preds: [(img, conf, box)], gts: {img: [box]}.

    Mengembalikan daftar (conf, tp) terurut conf menurun -- tiap label hanya
    boleh dipakai SATU prediksi, yang ber-conf tertinggi lebih dulu.
    """
    used = {k: [False] * len(v) for k, v in gts.items()}
    out = []
    for img, conf, box in sorted(preds, key=lambda p: -p[1]):
        best, best_j = thr, -1
        for j, gt in enumerate(gts.get(img, [])):
            if used[img][j]:
                continue
            v = iou(box, gt)
            if v >= best:
                best, best_j = v, j
        if best_j >= 0:
            used[img][best_j] = True
        out.append((conf, best_j >= 0))
    return out


def average_precision(matched, n_gt):
    """AP interpolasi 101 titik (gaya COCO) dari keluaran match()."""
    import numpy as np
    if not matched or n_gt == 0:
        return 0.0
    tp = np.cumsum([1.0 if t else 0.0 for _, t in matched])
    fp = np.cumsum([0.0 if t else 1.0 for _, t in matched])
    recall = tp / n_gt
    precision = tp / np.maximum(tp + fp, 1e-9)
    # amplop menurun dari kanan, lalu cuplik di 101 titik recall
    for i in range(len(precision) - 2, -1, -1):
        precision[i] = max(precision[i], precision[i + 1])
    points = np.linspace(0.0, 1.0, 101)
    idx = np.searchsorted(recall, points, side="left")
    return float(np.mean([precision[i] if i < len(precision) else 0.0
                          for i in idx]))


def at_threshold(matched, n_gt, conf):
    tp = sum(1 for c, t in matched if c >= conf and t)
    fp = sum(1 for c, t in matched if c >= conf and not t)
    tp_confs = [c for c, t in matched if c >= conf and t]
    return {
        "tp": tp, "fp": fp, "fn": n_gt - tp,
        "p": tp / (tp + fp) if (tp + fp) else 0.0,
        "r": tp / n_gt if n_gt else 0.0,
        "conf_min": min(tp_confs) if tp_confs else 0.0,
    }


def evaluate(path, imgsz, args, cls_name):
    import cv2
    from ultralytics import YOLO

    model = YOLO(path, task="detect")
    names = list_images(args.images)
    first = cv2.imread(os.path.join(args.images, names[0]))
    model.predict(first, imgsz=imgsz, verbose=False)          # pemanasan
    lookup = {str(v).lower(): int(k) for k, v in dict(model.names).items()}
    if cls_name.lower() not in lookup:
        raise SystemExit("kelas '%s' tidak ada di %s (ada: %s)"
                         % (cls_name, path, sorted(lookup)))
    cls_id = lookup[cls_name.lower()]

    # --- set berlabel: conf rendah supaya kurva PR lengkap -----------------
    preds, gts = [], {}
    for n in names:
        img = cv2.imread(os.path.join(args.images, n))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = os.path.splitext(n)[0]
        gts[n] = load_labels(os.path.join(args.labels, stem + ".txt"),
                             cls_id, w, h)
        r = model.predict(img, imgsz=imgsz, conf=0.001, iou=args.iou,
                          max_det=10, classes=[cls_id], verbose=False)[0]
        for k in range(len(r.boxes)):
            preds.append((n, float(r.boxes.conf[k]),
                          tuple(r.boxes.xyxy[k].tolist())))
    n_gt = sum(len(v) for v in gts.values())
    m50 = match(preds, gts, 0.50)
    ap50 = average_precision(m50, n_gt)
    ap5095 = sum(average_precision(match(preds, gts, t / 100.0), n_gt)
                 for t in range(50, 100, 5)) / 10.0

    # --- frame robot TANPA gawang: deteksi palsu + kecepatan nyata ----------
    neg_hits, neg_frames, neg_max = 0, 0, 0.0
    neg_confs = []
    elapsed = 0.0
    for folder in args.neg:
        for n in list_images(folder)[:args.neg_limit]:
            img = cv2.imread(os.path.join(folder, n))
            if img is None:
                continue
            t0 = time.perf_counter()
            r = model.predict(img, imgsz=imgsz, conf=0.05, iou=args.iou,
                              max_det=10, classes=[cls_id], verbose=False)[0]
            elapsed += time.perf_counter() - t0
            neg_frames += 1
            top = max((float(c) for c in r.boxes.conf), default=0.0)
            neg_confs.append(top)
            neg_max = max(neg_max, top)
            if top >= args.conf:
                neg_hits += 1
    return {
        "path": path, "n_img": len(gts), "n_gt": n_gt,
        "ap50": ap50, "ap5095": ap5095, "m50": m50,
        "op": at_threshold(m50, n_gt, args.conf),
        "neg_frames": neg_frames, "neg_hits": neg_hits, "neg_max": neg_max,
        "neg_confs": neg_confs,
        "ms": elapsed / neg_frames * 1000.0 if neg_frames else 0.0,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", action="append", required=True,
                    help="direktori IR (boleh diulang; yang pertama = acuan)")
    ap.add_argument("--imgsz", default="288,480",
                    help="tinggi,lebar -- WAJIB sama dengan bentuk ekspor IR")
    ap.add_argument("--images", default=os.path.join(DEFAULT_EVAL, "images"))
    ap.add_argument("--labels", default=os.path.join(DEFAULT_EVAL, "labels"))
    ap.add_argument("--neg", action="append", default=None,
                    help="folder frame robot TANPA gawang (boleh diulang)")
    ap.add_argument("--neg-limit", type=int, default=200,
                    help="maksimum frame per folder --neg")
    ap.add_argument("--class", dest="cls", default="gawang")
    ap.add_argument("--conf", type=float, default=0.40, help="ambang kerja")
    ap.add_argument("--iou", type=float, default=0.45, help="IoU NMS")
    args = ap.parse_args()
    args.neg = args.neg or DEFAULT_NEG

    sz = [int(v) for v in args.imgsz.split(",")]
    imgsz = tuple(sz) if len(sz) == 2 else sz[0]
    for d in args.model + [args.images, args.labels] + args.neg:
        if not os.path.exists(d):
            print("Tidak ditemukan: %s" % d, file=sys.stderr)
            return 1

    results = [evaluate(p, imgsz, args, args.cls) for p in args.model]

    print("\nKelas '%s', imgsz %s, ambang kerja %.2f" % (args.cls, args.imgsz, args.conf))
    print("Set berlabel: %d gambar / %d kotak  |  negatif: %d frame robot\n"
          % (results[0]["n_img"], results[0]["n_gt"], results[0]["neg_frames"]))
    print("%-3s %8s %7s %8s %6s %6s %9s %10s %9s" %
          ("", "ms/frm", "AP50", "AP50-95", "P", "R", "conf MIN",
           "palsu@neg", "neg max"))
    for i, r in enumerate(results):
        o = r["op"]
        print("%-3s %8.1f %7.3f %8.3f %6.3f %6.3f %9.3f %6d/%-4d %8.3f" %
              (chr(65 + i), r["ms"], r["ap50"], r["ap5095"], o["p"], o["r"],
               o["conf_min"], r["neg_hits"], r["neg_frames"], r["neg_max"]))
    print()
    for i, r in enumerate(results):
        print("%s = %s" % (chr(65 + i), os.path.basename(r["path"].rstrip("/"))))

    # Sapuan ambang untuk model TERAKHIR -- bahan memilih confidence_threshold.
    last = results[-1]
    print("\nSapuan ambang (%s):" % chr(64 + len(results)))
    print("%7s %6s %6s %4s %4s %10s" % ("ambang", "P", "R", "FP", "FN", "palsu@neg"))
    for t in SWEEP:
        o = at_threshold(last["m50"], last["n_gt"], t)
        neg = sum(1 for c in last["neg_confs"] if c >= t)
        print("%7.2f %6.3f %6.3f %4d %4d %6d/%-4d" %
              (t, o["p"], o["r"], o["fp"], o["fn"], neg, last["neg_frames"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
