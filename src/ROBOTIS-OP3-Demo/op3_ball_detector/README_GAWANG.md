# Detektor gawang — OpenVINO INT8 (op3_ball_detector)

Node **mandiri** `yolo_gawang_detector.py`: mendeteksi gawang dengan bobot
`ball_gawang_v3` (YOLOv8n, kelas `{0: ball, 1: gawang}`) dan hanya meneruskan
kelas `gawang`. **Belum disambung ke demo mana pun** dan tidak menyentuh jalur
bola (`/ball_detector_node/*`, `/vision/yolo/ball_center`).

| topik | tipe | isi |
|---|---|---|
| `/gawang_detector_node/detections` | `vision_msgs/Detection2DArray` | semua gawang, piksel gambar asli (pusat + ukuran kotak, `class_id`, `score`). Terbit tiap frame, **kosong** kalau tidak ada gawang |
| `/gawang_detector_node/gawang_center` | `geometry_msgs/PointStamped` | gawang ber-conf tertinggi, x,y dalam [0,1] (konvensi sama dengan `ball_center`), `z` = conf. Hanya saat terlihat |
| `/gawang_detector_node/fps` | `std_msgs/Float32` | laju proses nyata |
| `/gawang_detector_node/image_out` | `sensor_msgs/Image` | gambar beranotasi + FPS; hanya dihitung kalau ada penonton |

Masukan: `image_in` (bawaan `/usb_cam_node/image_raw/compressed`) dan `enable`
(`std_msgs/Bool`).

## Menjalankan — satu perintah

```bash
bash ~/motion_webots/scripts/gawang_detector.sh     # dari host
bash /ros2_ws/scripts/gawang_detector.sh            # dari DALAM container op3
```

Skrip itu menyamakan `ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` dengan kamera yang
sedang hidup, menghentikan detektor gawang lama kalau ada, lalu menjalankan
detektor + `rqt_image_view` (inti 3,7). Tutup jendela rqt atau Ctrl-C =
semuanya berhenti. Argumen launch boleh ditambahkan, mis. `rqt:=false` atau
`confidence_threshold:=0.45`.

## Menjalankan manual (di dalam container `op3`)

**`ROS_DOMAIN_ID` DAN `ROS_LOCALHOST_ONLY` harus sama dengan stack yang
memegang kamera.** `docker exec ... bash -lc` memberi domain 42 dari
`/etc/profile.d/op3-ros.sh`, sedangkan stack dari `./script.sh` bisa memakai
domain lain (30 Okt: domain **1**, localhost-only **1**). Satu saja yang beda,
node tidak melihat kamera: peringatan `NO IMAGE`, rqt hitam, dan launch
`use_usb_cam:=auto` mencoba menyalakan usb_cam kedua yang langsung mati
(`/dev/video0` sibuk). Cek milik stack (nama proses terpotong 15 huruf, maka
`usb_cam_node_ex`):

```bash
docker exec op3 bash -c 'tr "\0" "\n" < /proc/$(pgrep -o -x usb_cam_node_ex)/environ | grep -E "^ROS_(DOMAIN_ID|LOCALHOST_ONLY)="'
```

```bash
# terminal 1 -- detektor, menumpang kamera yang sudah hidup (sesuaikan angkanya)
docker exec -it op3 bash -lc 'export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1; \
  source /ros2_ws/install/setup.bash && \
  ros2 launch op3_ball_detector yolo_gawang_detector.launch.py'

# terminal 2 -- lihat hasilnya (env yang SAMA; inti 3,7 supaya tidak merebut YOLO)
docker exec -it op3 bash -lc 'export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1; \
  source /ros2_ws/install/setup.bash && \
  taskset -c 3,7 ros2 run rqt_image_view rqt_image_view /gawang_detector_node/image_out'

# ubah ambang selagi hidup
ros2 param set /gawang_detector_node/yolo_gawang_detector confidence_threshold 0.45
```

`use_usb_cam:=auto` (bawaan) menyalakan kamera sendiri hanya kalau belum ada
yang menerbitkan `image_topic`; kalau launch ini yang menyalakannya, rem laju
kamera ikut dilepas (`exposure_dynamic_framerate=0`).

## Optimasi yang dipakai

Sama dengan detektor bola (`PANDUAN_OPTIMASI_FPS_DETEKSI.md`):

- IR OpenVINO **INT8** dengan masukan **persegi panjang 288x480** (16:9, tanpa
  bar padding).
- Kalibrasi INT8 **berisi gawang** + dekode kotak kepala Detect tetap FP32
  (`quantize_ball_int8.py --ignore-head`) — lihat tabel di bawah.
- Antrean gambar kedalaman 1 best-effort; `detection_rate` 100 Hz (≥ 3x
  laju kamera 24 FPS); `plot()` dilewati tanpa penonton.
- Inti CPU: YOLO `taskset 0-2` (sibling 4,5,6 dikosongkan), usb_cam 3,7;
  `YOLO_AUTOINSTALL=false` di environment node.

## Model dan bukti akurasinya

`model/ball_gawang_v3_288x480_int8_calgawang_openvino_model`. `.pt`-nya
identik dengan yang dipakai detektor bola (sha256 `d7dde576…`); bedanya hanya
kuantisasi. Diukur 2026-10-30 dengan `scripts/eval_goal_model.py` pada 226
gambar berlabel (val+test dataset `ball_gawang_v3`) + 400 frame kamera robot
tanpa gawang, ambang 0,40:

| 288x480 | AP50 | AP50-95 | P | R | palsu (400 frame robot) |
|---|---|---|---|---|---|
| FP32 | 1,000 | 0,910 | 1,000 | 0,991 | 0 |
| INT8 milik detektor bola | 0,999 | 0,811 | 0,996 | 0,982 | 0 |
| INT8 kalibrasi gawang | 1,000 | 0,801 | 1,000 | 0,991 | 0 |
| **INT8 kalibrasi gawang + kepala FP32 (aktif)** | **1,000** | **0,902** | **1,000** | **0,991** | **0** |

INT8 1,42x lebih cepat dari FP32. AP50-95 yang membedakan: kuantisasi biasa
membuat tepi kotak (= posisi tiang) kurang pas. Kelas bola pada model yang
sama tidak dirugikan (tes_infer 60 frame: 59/60, conf min 0,344 → 0,380).

Uji hidup 30 Okt, berdampingan dengan detektor bola di inti 0-2: **21,05 FPS**
rata-rata (kamera 22,8–24), inferensi rata-rata 35 ms, umur frame ~160 ms,
0/934 deteksi palsu. Ongkosnya: detektor bola turun 23,9 → 20,2 FPS selama
keduanya hidup. Jalur keluaran diuji dengan 8 gambar berlabel lewat topik uji:
8/8 terdeteksi, pusat kotak meleset ≤ 4 px dari label.

**Batas bukti:** set berlabel seluruhnya dari Roboflow, bukan dari kamera
ALPHONSE, dan frame negatif tidak berisi lapangan. Gawang sungguhan lewat
kamera robot **belum pernah** diuji.

## Membuat ulang model (folder `model/` di-.gitignore)

Sumber: `~/Downloads/ball_gawang_v3_yolo.zip` (dataset) — 260 gambar gawang
kalibrasi ada di `model/calib_frames_gawang_rf_2026-10-30`, set uji berlabel di
`model/eval_gawang_v3`.

```bash
# kuantisasi -- WAJIB container --rm (nncf menyeret numpy 2)
docker run --rm --entrypoint /bin/bash -v /home/alphonse/motion_webots:/ros2_ws \
  op3-webots-ros2:humble -lc '
  M=/ros2_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/model
  pip3 install --no-cache-dir nncf &&
  python3 /ros2_ws/scripts/quantize_ball_int8.py \
    --src $M/ball_gawang_v3_288x480_openvino_model \
    --dst $M/ball_gawang_v3_288x480_int8_calgawang_openvino_model \
    --calib $M/calib_frames_gawang_rf_2026-10-30 --calib $M/calib_frames_demo_2026-09-18 \
    --calib $M/calib_frames --calib /ros2_ws/tes_infer.mp4 --video-frames 60 \
    --subset 720 --ignore-head'

# gerbang akurasi berlabel + kecepatan
docker run --rm --entrypoint /bin/bash -e YOLO_AUTOINSTALL=false \
  -v /home/alphonse/motion_webots:/ros2_ws op3-webots-ros2:humble -lc '
  M=/ros2_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/model
  python3 /ros2_ws/scripts/eval_goal_model.py \
    --model $M/ball_gawang_v3_288x480_openvino_model \
    --model $M/ball_gawang_v3_288x480_int8_calgawang_openvino_model'
```

Begitu ada rekaman gawang dari kamera robot (`scripts/capture_calib_frames.py`
di lapangan), tambahkan sebagai `--calib` lain dan ulangi kedua langkah.
