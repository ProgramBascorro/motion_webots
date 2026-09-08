# Landmark detector — OpenVINO (op3_ball_detector)

Menjalankan ekspor **OpenVINO INT8** dari
`model/landmark_deploy/best_landmark_v8n_int8_openvino_model` dan menerbitkan
**semua landmark** yang terlihat, lengkap dengan label kelas dan skornya:

| id | kelas | artinya |
|---|---|---|
| 0 | `L` | simpangan garis bentuk L (pojok lapangan / pojok kotak) |
| 1 | `T` | simpangan bentuk T (garis tengah bertemu garis samping, mulut kotak) |
| 2 | `X` | perpotongan bentuk X |
| 3 | `goalpost` | tiang gawang |
| 4 | `center_circle` | lingkaran tengah |

Ini adalah **umpan landmark mentah** — node ini sengaja tidak menafsirkan
geometrinya. Saudaranya, `yolo_goalline_detector.py`, memakai model yang sama
tapi menjawab satu pertanyaan spesifik (di mana garis gawang). Keduanya boleh
jalan bersamaan.

## Cara pakai (di dalam container `op3`)

```bash
# menumpang kamera yang sudah hidup (detektor bola biasanya sudah memegangnya)
docker exec -it op3 bash -lc \
  'ros2 launch op3_ball_detector yolo_landmark_detector.launch.py'

# nyalakan sekalian kameranya
docker exec -it op3 bash -lc \
  'ros2 launch op3_ball_detector yolo_landmark_detector.launch.py use_usb_cam:=true'

# hanya kelas yang berkaitan dengan gawang
docker exec -it op3 bash -lc \
  "ros2 launch op3_ball_detector yolo_landmark_detector.launch.py \
     target_classes:=\"['goalpost','center_circle']\""

# lihat hasilnya
docker exec -it op3 bash -lc 'ros2 run rqt_image_view rqt_image_view /landmark_detector_node/image_out'
docker exec -it op3 bash -lc 'ros2 topic echo /landmark_detector_node/detections'
```

Hanya **satu** proses boleh memegang `/dev/video0` — jangan pakai
`use_usb_cam:=true` kalau `usb_cam` sudah jalan.

## Files

| File | Purpose |
| --- | --- |
| `scripts/yolo_landmark_detector.py` | Node-nya. |
| `config/yolo_landmark_detector_params.yaml` | Semua parameter (**sumber kebenaran**, termasuk `model_path` dan `imgsz`). |
| `launch/yolo_landmark_detector.launch.py` | Menjalankan node (opsional + `usb_cam`), dengan `taskset` + `OMP_NUM_THREADS`. |
| `model/landmark_deploy/` | Hasil ekstrak `landmark_deploy.zip`: IR OpenVINO INT8, `.pt`, `.onnx`, dan daftar kelasnya. |
| `scripts/detector_fps.py` | Penghitung FPS bersama — dipakai node ini, detektor bola, dan detektor garis gawang. |

Jangan tertukar dengan **`config/yolo_landmark_detector_openvino_params.yaml`**:
itu profil lama untuk node generik `yolo_field_detector.py` memakai bobot yang
sama. Ketiganya bisa hidup berdampingan; hanya
`yolo_landmark_detector_params.yaml` yang milik node ini.

## Topics (di bawah namespace `landmark_detector_node`)

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| sub | `image_in` | `CompressedImage` / `Image` | Stream kamera (di-remap di launch). |
| sub | `enable` | `std_msgs/Bool` | Jeda/lanjutkan deteksi. |
| pub | `detections` | `vision_msgs/Detection2DArray` | Semua landmark, **piksel**. `.id` dan `.results[0].hypothesis.class_id` = nama kelas, `.score` = keyakinan, `.bbox.center.position` = titik tengah, `.bbox.size_x/y` = ukuran. |
| pub | `landmark_count` | `std_msgs/Int32` | Jumlah landmark di frame ini. |
| pub | `fps` | `std_msgs/Float32` | Laju proses langsung. |
| pub | `image_out` | `sensor_msgs/Image` | Gambar beranotasi. Overlay hanya digambar kalau ada yang subscribe. |

## Laporan FPS

Sama polanya dengan detektor lain di paket ini — **dalam FPS, bukan milidetik**:

1. **Saat start**: `Model ceiling on this machine: 35.0 FPS (28.6 ms/frame over
   5 warm frames)`. Inferensi pertama (kompilasi IR ~200 ms) dibuang supaya
   angkanya jujur. Kalau plafon ini di bawah `detection_rate`, node bilang
   terus terang.
2. **Selama jalan**, tiap `fps_log_period` detik — beserta isi layarnya:
   `FPS 10.0 (avg 9.7) | infer 30.0 ms | loop 38.5 ms | frames 150 | L x1, goalpost x2`
3. **Saat berhenti**, ringkasan + **rekap per kelas**, yang justru paling
   berguna untuk menilai layak-tidaknya model di lapangan tertentu:

```
===== landmark detector =====
frames processed : 277
elapsed          : 28.39 s
AVERAGE FPS      : 9.72
min / max FPS    : 8.43 / 10.04
target rate      : 10.0 Hz  (achieved 97%)
inference        : avg   40.6 ms  p50   37.3  p95   63.9  max  125.1
preprocess       : avg    4.0 ms  p50    3.7  p95    6.3  max   10.4
postprocess      : avg    1.9 ms  p50    1.8  p95    3.2  max    6.4
loop total       : avg   47.7 ms  p50   44.5  p95   72.1  max  134.5
landmark seen    : 277 / 277 frames (100.0%)
per class        : (name: total boxes / frames seen)
  goalpost       :   495 boxes /   277 frames (100.0%)
  L              :   390 boxes /   271 frames (97.8%)
  T              :     2 boxes /     2 frames (0.7%)
=============================
```

## Kalau tidak ada hasil sama sekali

Sejak node ini punya pengawas alur gambar, kasus itu **memberi tahu dirinya
sendiri** dalam 3 detik:

```
[INFO] Listening for images on "/usb_cam_node/image_raw/compressed"
[WARN] NO IMAGE on "/usb_cam_node/image_raw/compressed" after 3 s — nothing is
       publishing there, so this detector has nothing to look at. Either start
       the camera together with it (use_usb_cam:=true), or point image_topic:=
       at a topic that has a publisher. Check with: ros2 topic hz ...
```

Sejak `use_usb_cam` default-nya **`auto`**, launch ini memeriksa dulu apakah
sudah ada yang mem-publish `image_topic`; kalau belum, kameranya dinyalakan
sendiri. Keputusannya dicetak di baris pertama:

```
[camera] starting the usb_cam node (auto: nothing publishes "..." yet)
[camera] NOT starting the usb_cam node (auto: "..." already has a publisher)
```

Jadi menjalankan launch ini sendirian langsung bekerja, dan menjalankannya di
samping detektor bola tidak berebut `/dev/video0` — hanya **satu** proses yang
boleh memegangnya. Paksa dengan `use_usb_cam:=true` atau `:=false` bila perlu.

Perlu diingat: `ros2 topic list` **menampilkan** topik kamera walau tak ada yang
mem-publish — node ini sendiri yang membuatnya muncul karena ber-subscribe ke
situ. Yang membuktikan ada datanya adalah `ros2 topic hz <topik>`.

Kalau aliran gambar berhenti di tengah jalan, pengawas juga bilang:
`Image flow STOPPED on "..." : nothing for 4.2 s after 913 frames.`

### Gambar masuk tapi hasilnya tetap nol

Node memeriksa tingkat kecerahan frame pertama dan mengeluh kalau gelap:

```
[INFO] First image received on "...": 1280x720 (mean level 0.0/255)
[WARN] Image on "..." is almost BLACK (mean 0.0/255): there is nothing in it
       for the model to find. Check the lens cap, the room lights, and where
       the head is pointing.
```

`mean level` itu angka yang paling cepat menjawab "salah model atau salah
kamera". Pemandangan nyata biasanya 60–160. Di bawah 16 berarti sensornya
memang tidak menerima cahaya — bukan modelnya yang buta.

Kalau perlu memastikan device-nya benar:

```bash
docker exec op3 bash -lc 'v4l2-ctl -d /dev/video0 --info'   # Card type = kameranya
```

Pada robot ini kameranya **Logitech C920 di `/dev/video0`**. `/dev/video1` milik
kamera yang sama tapi node metadata — tidak bisa dipakai capture, jadi jangan
mengarahkan `video_device` ke situ.


## Angka terukur di robot ini (NUC10i5, OpenVINO INT8, 640 px)

| | FPS | Inferensi |
|---|---|---|
| Plafon model (mesin senggang) | **35–39** | 22–29 ms |
| Node + kamera asli, `detection_rate: 10` | **9,7–9,9** (97–99% target) | 38–41 ms |

Modelnya sanggup ~35 FPS, tapi `detection_rate` sengaja ditahan **10 Hz**:
detektor bola dan kontroler walking memperebutkan core yang sama.

## Yang perlu diketahui

- **`imgsz` wajib 640.** IR ini statis `[1,3,640,640]`; angka lain ditolak
  OpenVINO. Ekspor ulang model untuk mengubahnya.
- **Direktori OpenVINO butuh `.xml` DAN `.bin`.** Kalau salah satunya hilang,
  node berhenti dengan pesan yang menyebutkan file mana yang kurang — bukan
  error ultralytics yang membingungkan.
- **Model ini mudah salah kira di dalam ruangan.** Rekap per kelas di atas
  diambil dengan kamera menghadap lab tanpa gawang, dan tetap melaporkan 495
  `goalpost`. Itu soal data latih. Naikkan `confidence_threshold` di lapangan
  sungguhan.
- **`YOLO_AUTOINSTALL=false`** sudah disetel di launch file. Kalau menjalankan
  skripnya manual, ekspor sendiri variabel itu.
