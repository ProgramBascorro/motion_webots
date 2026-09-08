#!/usr/bin/env python3
"""Kuantisasi INT8 model bola (OpenVINO IR) dengan NNCF, kalibrasi dari frame kamera robot.

JALANKAN DI CONTAINER SEKALI-PAKAI, JANGAN di container `op3`:

    docker run --rm --entrypoint /bin/bash \\
      -v /home/alphonse/motion_webots:/ros2_ws \\
      op3-webots-ros2:humble -lc '
        pip3 install --no-cache-dir nncf &&
        python3 /ros2_ws/scripts/quantize_ball_int8.py'

Alasannya: `nncf` menyeret numpy 2, dan numpy 2 MERUSAK `cv_bridge` di ROS
Humble. Pernah terjadi di mesin ini. Container `--rm` membuang instalasinya
begitu selesai, sehingga image dan container `op3` tetap bersih. Verifikasi
sesudahnya:
    docker exec op3 bash -lc 'source /opt/ros/humble/setup.bash && \\
      python3 -c "import numpy,cv_bridge;print(numpy.__version__)"'   # 1.26.4

Kenapa NNCF langsung, bukan `yolo export int8=True data=...`: yang terakhir
menuntut dataset YAML berlabel lengkap. Dataset latihnya ada di mesin Windows
(`F:\\Robosoccer\\...`), tidak di sini. NNCF cuma butuh gambar -- dan gambar
dari kamera robot ini justru lebih mewakili kondisi kerja sebenarnya.

CATATAN KECEPATAN: i5-10210U ini AVX2 TANPA VNNI. INT8 di AVX2 biasanya cuma
1,3-1,8x, bukan 2x yang didapat CPU ber-VNNI. Ukur dulu; kalau di bawah ~1,3x,
percepatannya tidak sepadan dengan risiko akurasi.
"""

import argparse
import os
import shutil
import sys

WS = os.environ.get("OP3_WS", "/ros2_ws")
MODEL_DIR = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_ball_detector/model")
DEFAULT_SRC = os.path.join(MODEL_DIR, "ball_yolov8n_384x640_openvino_model")
DEFAULT_DST = os.path.join(MODEL_DIR, "ball_yolov8n_384x640_int8_openvino_model")
DEFAULT_CALIB = os.path.join(MODEL_DIR, "calib_frames")


def letterbox(img, new_shape, color=(114, 114, 114)):
    """Salinan setia LetterBox milik Ultralytics untuk backend non-PyTorch.

    Penting: `auto=False` (backend ekspor punya bentuk masukan STATIS), jadi
    gambar di-pad sampai new_shape PENUH, bukan sampai kelipatan stride.
    Pra-proses kalibrasi HARUS sama persis dengan pra-proses inferensi -- kalau
    beda, rentang nilai yang dipelajari NNCF salah dan akurasinya jatuh tanpa
    pesan error apa pun.
    """
    import cv2
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    return cv2.copyMakeBorder(img, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=color)


def preprocess(img, shape_hw):
    """BGR uint8 HWC -> NCHW float32 [0,1] RGB, persis seperti Ultralytics."""
    import numpy as np
    lb = letterbox(img, shape_hw)
    rgb = lb[:, :, ::-1]                       # BGR -> RGB
    chw = rgb.transpose(2, 0, 1)               # HWC -> CHW
    arr = np.ascontiguousarray(chw, dtype=np.float32) / 255.0
    return arr[None]                           # tambah dimensi batch


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=DEFAULT_SRC, help="direktori IR FP32")
    ap.add_argument("--dst", default=DEFAULT_DST, help="direktori keluaran INT8")
    ap.add_argument("--calib", default=DEFAULT_CALIB, help="folder frame kalibrasi")
    ap.add_argument("--subset", type=int, default=300,
                    help="berapa frame dipakai kalibrasi (bawaan 300)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    import openvino as ov
    import nncf

    xmls = [f for f in os.listdir(args.src) if f.endswith(".xml")]
    if not xmls:
        print("Tidak ada .xml di %s" % args.src, file=sys.stderr)
        return 1
    xml = os.path.join(args.src, xmls[0])

    core = ov.Core()
    model = core.read_model(xml)
    shape = list(model.input(0).partial_shape.get_shape())   # [1,3,H,W]
    shape_hw = (shape[2], shape[3])
    print("Model  : %s" % xml)
    print("Masukan: %s  (tinggi x lebar = %d x %d)" % (shape, shape_hw[0], shape_hw[1]))

    files = sorted(
        os.path.join(args.calib, f) for f in os.listdir(args.calib)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    )
    if not files:
        print("Tidak ada gambar di %s -- jalankan scripts/capture_calib_frames.py dulu"
              % args.calib, file=sys.stderr)
        return 1
    files = files[:args.subset]
    print("Kalibrasi: %d frame dari %s" % (len(files), args.calib))
    if len(files) < 100:
        print("PERINGATAN: di bawah 100 frame biasanya terlalu sedikit.")

    def gen():
        for path in files:
            img = cv2.imread(path)
            if img is None:
                continue
            yield preprocess(img, shape_hw)

    calibration_dataset = nncf.Dataset(gen())

    print("\nMenjalankan kuantisasi pasca-latih (bisa beberapa menit)...")
    quantized = nncf.quantize(
        model,
        calibration_dataset,
        preset=nncf.QuantizationPreset.MIXED,
        subset_size=len(files),
    )

    if os.path.isdir(args.dst):
        shutil.rmtree(args.dst)
    os.makedirs(args.dst, exist_ok=True)
    base = os.path.splitext(os.path.basename(xml))[0]
    out_xml = os.path.join(args.dst, base + ".xml")
    ov.save_model(quantized, out_xml)

    # metadata.yaml membawa nama kelas + imgsz; Ultralytics membacanya saat
    # memuat model. Tanpa ini, kelasnya jadi "0,1,2,..." dan target_classes
    # ["ball"] tidak akan cocok dengan apa pun.
    meta = os.path.join(args.src, "metadata.yaml")
    if os.path.isfile(meta):
        shutil.copy2(meta, os.path.join(args.dst, "metadata.yaml"))
        print("metadata.yaml disalin.")
    else:
        print("PERINGATAN: metadata.yaml tidak ada di sumber -- nama kelas akan hilang.")

    fp32 = sum(os.path.getsize(os.path.join(args.src, f)) for f in os.listdir(args.src))
    int8 = sum(os.path.getsize(os.path.join(args.dst, f)) for f in os.listdir(args.dst))
    print("\nSelesai: %s" % args.dst)
    print("Ukuran : FP32 %.1f MB -> INT8 %.1f MB" % (fp32 / 1e6, int8 / 1e6))
    print("\nLangkah berikutnya WAJIB: ukur kecepatan DAN akurasi sebelum dipakai.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
