# YOLO Ball Detector (op3_ball_detector)

A YOLO-based ball detector that is a **drop-in replacement** for the classic
Hough-circle `ball_detector_node`. It runs an [Ultralytics](https://docs.ultralytics.com)
YOLO model on the camera stream and republishes detections in the exact format
the existing `op3_demo` soccer pipeline expects.

## Files

| File | Purpose |
| --- | --- |
| `scripts/yolo_ball_detector.py` | The detector node (Python / rclpy). Loads `.pt`, `.onnx`, **or** an OpenVINO dir. |
| `launch/yolo_ball_detector.launch.py` | Brings up `usb_cam` + the detector (PyTorch `.pt`). |
| `launch/yolo_ball_detector_onnx.launch.py` | Same, but runs an `.onnx` model via onnxruntime. |
| `launch/yolo_ball_detector_openvino.launch.py` | Same, but runs an OpenVINO model (fastest on Intel CPU). |
| `config/yolo_ball_detector_params.yaml` | Tunable parameters for the `.pt` setup (**single source of truth**, incl. `model_path`). |
| `config/yolo_ball_detector_onnx_params.yaml` | Same, for the `.onnx` setup (`model_path` → `model/*.onnx`). |
| `config/yolo_ball_detector_openvino_params.yaml` | Same, for the OpenVINO setup (`model_path` → `model/*_openvino_model/`). |
| `model/ball_yolov8n*` | **Current ball weights** (`.pt`, `.onnx`, `_openvino_model/`) — single class, class 0 = `ball`, trained/exported at imgsz **640**. |
| `model/v8n*`, `model/v8s*` | Previous ball weights (class 0 = `Bola-baru`, exported at 320). Kept for comparison. |
| `model/v8n_lapangan_best*` | Field/environment weights — used by `yolo_field_detector.py`, **not** by the ball detector. |
| `model/best_landmark_v8n*` | Landmark weights (L/T/X/goalpost/center_circle) — used by the **goal-line detector**, see `README_GOALLINE.md`. |

> **Landmark / garis gawang?** Keduanya punya dokumennya sendiri:
> [`README_LANDMARK.md`](README_LANDMARK.md) — `yolo_landmark_detector.py`,
> umpan landmark mentah 5 kelas lewat OpenVINO; dan
> [`README_GOALLINE.md`](README_GOALLINE.md) — `yolo_goalline_detector.py`
> plus program benchmark FPS mandiri `goalline_fps_benchmark.py`.

## Topics (under the `ball_detector_node` namespace)

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| sub | `image_in` | `CompressedImage` / `Image` | Camera stream (remapped in launch). |
| sub | `enable` | `std_msgs/Bool` | Pause/resume detection. |
| pub | `circle_set` | `op3_ball_detector_msgs/CircleSetStamped` | `x,y` normalized `[-1,1]`, `z` = radius in **pixels** — same as the C++ detector. |
| pub | `image_out` | `sensor_msgs/Image` | Annotated debug image. |
| pub | `fps` | `std_msgs/Float32` | Live processing rate (sliding window). |

Because it publishes on `/ball_detector_node/circle_set`, the `op3_demo`
`ball_tracker` / `ball_follower` work unchanged.

## Laporan FPS (bukan milidetik)

Node ini dulu mencetak `latency: pre=... infer=... post=... ms`. Angka itu
menjawab "satu frame berapa mahal", **bukan** "loop-nya dapat berapa frame per
detik" — dua hal yang langsung berpisah begitu kamera, `detection_rate`, atau
frame yang terlewat ikut bermain. Sekarang yang dilaporkan adalah **FPS**, dan
milidetiknya tetap ada sebagai penjelas di belakangnya:

1. **Saat start**: `Model ceiling on this machine: 21.4 FPS (46.7 ms/frame over
   5 warm frames)` — inferensi pertama (kompilasi IR) dibuang. Kalau plafon ini
   di bawah `detection_rate`, node bilang terus terang.
2. **Selama jalan**, tiap `fps_log_period` detik (default 5 s), dan terus-
   menerus di topik `fps`:
   `FPS 12.0 (avg 11.7) | infer 45.1 ms | loop 49.7 ms | frames 267 | no ball`
3. **Saat berhenti** (Ctrl-C termasuk), blok ringkasan:

```
===== ball detector =====
frames processed : 317
elapsed          : 26.83 s
AVERAGE FPS      : 11.78
min / max FPS    : 10.83 / 12.05
target rate      : 12.0 Hz  (achieved 98%)
inference        : avg   48.0 ms  p50   46.2  p95   61.0  max  124.8
preprocess       : avg    3.2 ms  p50    3.0  p95    4.7  max    8.5
postprocess      : avg    0.6 ms  p50    0.6  p95    1.0  max    1.8
loop total       : avg   52.9 ms  p50   51.1  p95   66.4  max  136.0
ball seen        : 0 / 317 frames (0.0%)
=========================
```

Node ini juga mengawasi alur gambar: kalau tidak ada yang mem-publish ke
`image_in`, dalam 3 detik ia memperingatkan `NO IMAGE on "..."` beserta cara
memeriksanya — bukan diam seperti dulu, yang tak bisa dibedakan dari "bolanya
memang tidak ketemu".

Penghitungnya ada di `scripts/detector_fps.py` dan dipakai bersama oleh
detektor bola, landmark, dan garis gawang — supaya tidak pernah ada dua cara
menghitung FPS di paket ini. Parameter barunya: `fps_log_period`,
`fps_window_sec`, `startup_benchmark_frames`.

## Dependencies

`ultralytics` (and `torch`) have **no rosdep key**. In the project Docker image
they are **already baked in** — the project `Dockerfile` installs CPU `torch`/
`torchvision` plus `requirements_yolo.txt`, so when you build/run via
`./script.sh` nothing extra is needed inside the container.

If you ever run outside that image, install them manually in the runtime
environment (CPU-only example):

```bash
apt-get update && apt-get install -y python3-pip   # if pip is missing
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r src/ROBOTIS-OP3-Demo/op3_ball_detector/requirements_yolo.txt
```

> **IMPORTANT:** keep `numpy < 2`. ROS Humble's `cv_bridge` is compiled against
> NumPy 1.x and fails with `_ARRAY_API not found` under NumPy 2.x. The
> `requirements_yolo.txt` pins `numpy<2` and a NumPy-1.x-compatible
> `opencv-python` for this reason.

> **ONNX / OpenVINO:** `requirements_yolo.txt` also installs `onnxruntime` and
> `openvino` (both CPU, both compatible with `numpy<2` — `openvino` declares
> `numpy<2.5,>=1.16.6`). Without them ultralytics auto-pip-installs them
> unpinned on first load — lost on container recreate, and the unpinned install
> can drag NumPy 2 back in. `torch` is still needed even for ONNX/OpenVINO
> inference (ultralytics imports it).

`rclpy`, `cv_bridge` and OpenCV/numpy from ROS are declared in `package.xml`.

In the project image this is already persistent (baked into the `Dockerfile`).
If you installed manually into a running container instead, make it persistent
with `docker commit <container> <new-image>` or add the `pip install` steps to
the `Dockerfile` and rebuild.

## Build

Build the package the usual way (inside the project container):

```bash
colcon build --packages-up-to op3_ball_detector
source install/setup.bash
```

## Run

```bash
# usb_cam + YOLO detector (everything read from the YAML)
ros2 launch op3_ball_detector yolo_ball_detector.launch.py

# Use an already-running camera publishing a raw Image:
ros2 launch op3_ball_detector yolo_ball_detector.launch.py \
    use_usb_cam:=false image_topic:=/usb_cam_node/image_raw use_compressed:=false

# One-off override of the YAML (e.g. GPU + a different weight file):
ros2 launch op3_ball_detector yolo_ball_detector.launch.py \
    device:=cuda:0 model_path:=model/ball_yolov8n_best.pt
```

### ONNX model

Use the ONNX launch + ONNX param file instead (runs through onnxruntime,
CPUExecutionProvider):

```bash
# default model from config/yolo_ball_detector_onnx_params.yaml
# (model/ball_yolov8n.onnx exported at imgsz 640)
ros2 launch op3_ball_detector yolo_ball_detector_onnx.launch.py
```

It is the same node and publishes the same topics — only the weights/backend
differ. `imgsz` in the ONNX YAML **must** match the size the model was exported
with (`ball_yolov8n.onnx` is `640`), or onnxruntime rejects the input shape.
To use a different size, re-export and update `imgsz`:
`yolo export model=model/ball_yolov8n_best.pt format=onnx imgsz=NNN`.

### OpenVINO model (fastest on Intel CPU)

Use the OpenVINO launch + param file. The model is a **directory**
(`.xml`+`.bin`), exported from the `.pt`:

```bash
# one-time export (needs the openvino package; see Dependencies)
yolo export model=model/ball_yolov8n_best.pt format=openvino imgsz=640
# -> model/ball_yolov8n_openvino_model/  (already shipped in this repo)
#
# Export at 320 instead for ~4x less CPU (accepts a smaller ball on screen):
# yolo export model=model/ball_yolov8n_best.pt format=openvino imgsz=320
# then set imgsz: 320 in config/yolo_ball_detector_openvino_params.yaml.

ros2 launch op3_ball_detector yolo_ball_detector_openvino.launch.py
```

On Intel CPUs OpenVINO auto-tunes threading ("LATENCY mode") and is both faster
and more stable than onnxruntime, which oversubscribes all cores. **Safe with
`numpy<2`:** `openvino` declares `numpy<2.5,>=1.16.6`, so it does not pull NumPy 2.

### Performance levers

Rough CPU cost on the i5-1340P (single class, imgsz 320 unless noted):

| Backend / model | ms/frame | notes |
| --- | --- | --- |
| ONNX v8s @ 640 | ≈120 | the old default |
| ONNX v8n @ 640 | ≈74 | |
| ONNX v8n @ 416 | ≈36 | |
| ONNX v8n @ 320 | ≈23–59 | varies — thread oversubscription |
| OpenVINO v8n @ 320 | ≈26 | fastest + stable |
| OpenVINO ball_yolov8n @ 640 | ≈42 | previous default — kept as fallback in `model/ball_yolov8n_openvino_model` |
| **OpenVINO ball_yolov8n @ 384x640** | **≈27** | **current default** — rectangular, matches the 640x360 camera |

## Optimalisasi FPS 2026-09-01 (NUC10i5FNH, i5-10210U)

Keluhan awal: deteksi bersamaan dengan demo cuma memberi **8–15 FPS**. Ada
**tiga** sebab terpisah, dan semuanya sudah diukur — jangan tebak ulang.

### 1. Sibling Hyper-Threading, bukan jumlah inti

4 inti fisik / 8 utas, pasangan `0↔4 1↔5 2↔6 3↔7`. Dua utas satu pasangan
berbagi unit eksekusi:

| kondisi | ms/frame |
| --- | --- |
| YOLO di 0-3, inti 4-7 sepi | 40,3 |
| YOLO di 0-3, inti 4-7 sibuk | **84,2** (2,1x lebih lambat) |

Default lama `cpu_affinity: 0-3` **tidak melindungi apa pun** — ia cuma
melarang YOLO memakai 4-7, sementara proses lain bebas mendarat di sana.
Sekarang: YOLO di `0-2`, node pendamping di `3,7` (`aux_cpu_affinity` di launch
detektor, `aux_launch_prefix` di `demo.launch.xml`).
Pesaing terbesar bukan node ROS melainkan **desktop** (Firefox 39%+19%,
VS Code ~35%) — kurung dengan `scripts/op3_docker.sh match-mode on`.

Terukur dengan pesaing terkurung: median 63 → 48 ms (15,8 → 20,9 FPS),
p95 114 → 59 ms (**8,7 → 16,9 FPS**).

### 2. Letterbox membuang 44% komputasi

Kamera 640x360, tapi `imgsz` persegi mem-padding jadi 640x640 → hanya
230400/409600 piksel berisi gambar. Bentuk **384x640** mengikuti rasio kamera;
skala letterbox tetap 1,0 di kedua kasus, jadi bola **tidak diperkecil sedikit
pun**. Diatur lewat parameter baru `imgsz_hw: [384, 640]`.

Uji regresi di `tes_infer.mp4` (30 frame, resolusi asli 478x850):

| model | deteksi | conf rata2 | conf min | ms/frame |
| --- | --- | --- | --- | --- |
| 640x640 lama | 30/30 | 0,868 | 0,837 | 55,0 |
| **384x640 baru** | **30/30** | **0,869** | **0,848** | **30,7** |

Kualitas identik (min conf malah naik), 1,79x lebih cepat.

### 3. `detection_rate` beradu-layang dengan laju kamera ← paling tak terduga

Executor rclpy satu utas. Kalau `detection_rate` terlalu dekat dengan laju
kamera, timer menyala saat frame belum tiba lalu keluar; frame tiba beberapa
milidetik kemudian dan menunggu SATU periode timer penuh.

| detection_rate | FPS nyata (kamera 20 FPS) |
| --- | --- |
| 25 Hz | 13,4 (33% frame hangus) |
| **60 Hz** | **19,4** (mentok di kamera) |

Tik kosong harganya cuma satu lock + satu tes None, jadi ini praktis gratis.
**Patokan: setel minimal 3x laju kamera.** Sesudah kamera naik ke 24 FPS,
`detection_rate` ikut naik ke **100 Hz**: periode kamera 41,7 ms dikurangi
inferensi ~30 ms menyisakan 11,7 ms, jadi timer harus lebih cepat dari ~85 Hz
supaya tidak ada frame yang terlewat.

### 4. Pajan kamera membatasi FPS di SUMBERNYA ← paling sering salah didiagnosis

Gejalanya identik dengan masalah CPU (FPS turun), padahal CPU tidak terlibat
sama sekali. Kamera UVC boleh **memperlambat dirinya sendiri** supaya sensor
sempat memajan lebih lama di tempat gelap:

```
FPS maksimum = 1 / waktu_pajan
```

Terukur saat FPS "misterius" turun ke 14-15 sambil webcam digerakkan:

```
exposure_time_absolute     = 666  (satuan 100 us) -> pajan 66,6 ms -> 15,0 FPS
exposure_dynamic_framerate = 1    (bawaannya 0!)  -> izin memperlambat diri
gain                       = 255  (mentok, tak ada cadangan cahaya lain)
```

**15,0 itu persis angka yang dilaporkan.** Tidak ada optimasi komputasi yang
bisa menambah frame yang memang tidak pernah dikirim sensor.

Perbaikannya `v4l2-ctl -c exposure_dynamic_framerate=0`, dipasang otomatis oleh
`yolo_ball_detector_openvino.launch.py` (2 detik setelah `usb_cam` start).
`usb_cam` **tidak punya** parameter untuk kontrol ini — ia cuma mengenal
`autoexposure`/`exposure`, jadi harus lewat `v4l2-ctl`.
Ini **tidak** mematikan auto-exposure: kamera tetap menyesuaikan cahaya, cuma
tidak boleh lagi melanggar laju yang diminta.

Hasil sesudahnya: pajan 666 -> **416** (41,6 ms, tepat periode 24 FPS),
FPS **23,5-24,0 mantap**.

**Laju yang tersedia terikat perangkat keras.** Di 640x360 kamera ini cuma
menawarkan 30 / 24 / 20 / 15 / 10 / 7,5 / 5 — **25 TIDAK ADA**. Jangan menulis
25 di `camera_param.yaml`; ia akan diam-diam jatuh ke mode terdekat.
Periksa dengan `v4l2-ctl -d /dev/video0 --list-formats-ext`.

#### Trade-off cahaya — dan batas pengetahuan kita

Laju frame membatasi pajan, jadi menaikkan FPS **pasti** memotong cahaya di
adegan gelap, dan `gain` sudah mentok 255 sehingga sensor tidak bisa menebusnya:

| laju | batas pajan | cahaya vs 66,6 ms |
|---|---|---|
| 15 FPS | 66,6 ms | 100% (acuan) |
| 20 FPS | 50,0 ms | 75% (-0,41 EV) |
| **24 FPS** | **41,6 ms** | **62% (-0,68 EV)** |
| 30 FPS | 33,3 ms | 50% (-1,0 EV) |

**Tapi batas itu hanya menggigit kalau adegannya cukup gelap.** Terukur pada
pencahayaan ruangan biasa, auto-exposure berhenti di 333 (33,3 ms) — di bawah
batas 41,6 ms — sehingga kecerahan sebelum/sesudah identik (148,3 vs 147,7
dari 255). Jadi pada cahaya cukup, 24 FPS **gratis**.

Angka -0,68 EV di tabel adalah **kasus terburuk hasil hitungan, bukan hasil
ukur**: ruangan saat pengujian terlalu terang untuk memancing kondisi itu.
Verifikasi di lapangan yang sebenarnya: bandingkan `conf` deteksi bola pada
posisi & pencahayaan yang sama sebelum/sesudah. Kalau conf mendekati
`confidence_threshold: 0.5`, turunkan `framerate` ke 20.0 atau tambah
penerangan lapangan — penerangan adalah satu-satunya jalan yang menaikkan FPS
**dan** akurasi sekaligus.

Cek keadaan kamera kapan saja lewat `scripts/op3_docker.sh doctor` (bagian 1),
yang melaporkan pajan, batas FPS yang tersirat, dan status remnya.

### 5. Di bawah beban demo penuh, semuanya berubah

Angka 24 FPS di atas didapat dengan detektor + kamera SAJA. Demo penuh
menambah `op3_manager` (modul jalan, IK 125 Hz), `op_demo_node`, dan
`head_tracking`. Terukur pada demo sungguhan: inferensi **55-84 ms**
(rata-rata 11,6 FPS) — dua sampai tiga kali lebih lambat.

Model dan kamera TIDAK bersalah di sini; log mencatat model 384x640 yang benar
termuat, dan pajan kamera sudah 33,3 ms (sanggup 30 FPS). Yang terjadi murni
perebutan CPU.

Terukur, model 384x640, 45 detik berkelanjutan:

| kondisi | inferensi | setara FPS |
|---|---|---|
| tanpa pesaing | 30,1 ms | 33 |
| 3 pesaing BEBAS di 0-7 | 47,7 ms | 21 |
| 3 pesaing DIKURUNG ke 3,7 | **33,6 ms** | 30 |

Mengurung pesaing memulihkan hampir seluruh kerugian.

#### Jebakan: match mode TIDAK mengurung alat debug di dalam container

`scripts/op3_docker.sh match-mode on` bekerja lewat cgroup `user.slice`, jadi
ia hanya mengurung **sesi desktop host** (Firefox, VS Code, GNOME). `rqt`,
`rviz`, atau `ros2 topic echo` yang dijalankan **di dalam container** ada di
cgroup docker dan **tidak tersentuh** — mereka tetap bebas mendarat di inti
4, 5, 6 dan memperlambat detektor.

Saat demo sungguhan: **tutup `rqt`/`rviz`**, atau jalankan dengan
`taskset -c 3,7 rqt`. Terukur, `rqt` yang menganggur pun memakai 7,8% CPU
tanpa dipatok.

(Yang TIDAK bermasalah: `rqt` sebagai *pelanggan* `image_out`. Diuji langsung —
memasang pelanggan di tengah run tidak menurunkan FPS sama sekali, karena
`_publish_annotated` memang murah. Yang mahal adalah rqt sebagai *pemakai CPU*.)

#### Cari sendiri siapa pencurinya

```bash
# jalankan SELAMA demo berlangsung, di terminal kedua
./script.sh --cpu-thieves
```
`scripts/op3_cpu_thieves.py` mencuplik 30 detik dan melaporkan proses mana yang
duduk di inti **sibling** milik detektor (4,5,6) berikut CPU yang mereka pakai
di sana, plus penanda mana yang tidak dipatok. Ini menjawab pertanyaan yang
tidak bisa dijawab `top`/`htop`: `taskset -c 0-2` pada detektor **tidak**
mencadangkan inti 0-2 untuknya — ia cuma melarang detektor keluar dari sana.

#### Panas ikut bermain (koreksi terhadap catatan lama)

Baris "thermal throttling" di tabel bawah ditulis berdasarkan 4,5 detik sejak
boot, saat mesin sedang santai. **Itu tidak berlaku di bawah beban nyata.**
Terukur: 45 detik inferensi berkelanjutan sendirian menambah 4,1 detik
throttling paket (9% waktu); dengan sibling ikut sibuk naik jadi 10,6 detik
(24%). Pantau dengan:
```bash
cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_total_time_ms
```
Kalau tambahannya besar selama demo, memangkas komputasi (INT8) menolong dua
kali: lebih cepat DAN lebih dingin.

### Sudah diperiksa dan DISINGKIRKAN (ada angkanya, jangan diulang)

| dugaan | hasil ukur | putusan |
| --- | --- | --- |
| decode JPEG mahal | 2,67 ms (4% anggaran) | biarkan compressed |
| `results.plot()` | 0,11 ms, sudah digerbang jumlah subscriber | biarkan |
| NMS mahal | 0,62 ms | biarkan |
| CPU tidak naik clock | stabil 3300–3400 MHz | governor tidak perlu diubah |
| utas OpenVINO kurang | sudah LATENCY + 4 utas | biarkan |
| thermal throttling | 4,5 detik sejak boot, mesin santai | **tidak lagi berlaku di bawah beban demo** — lihat bagian 5 |
| `reshape()` IR jadi 384x640 | ditolak, grid anchor dibakar jadi konstanta | wajib re-export |

Sisa jalur kalau masih kurang: **INT8** (`format=openvino int8=True` + data
kalibrasi) untuk ~1,5-2x lagi pada AVX2 (CPU ini tidak punya VNNI), dan
menaikkan `framerate` kamera di atas 20.

> **Awas saat ekspor ulang:** ekspor OpenVINO lewat ONNX, dan itu memicu
> Ultralytics AutoUpdate memasang `onnx` → menyeret **numpy 2** → **merusak
> `cv_bridge`**. Jangan `docker exec op3`. Pakai container sekali-pakai supaya
> instalasinya ikut mati:
> ```bash
> docker run --rm --entrypoint /bin/bash \
>   -v /home/alphonse/motion_webots:/ros2_ws \
>   -w /ros2_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/model \
>   op3-webots-ros2:humble -lc '
>     pip3 install --no-cache-dir onnx==1.16.2 onnxslim
>     yolo export model=ball_yolov8n_best.pt format=openvino imgsz=384,640'
> ```
> (`--entrypoint /bin/bash` perlu: entrypoint image ini menelan argumen pada
> `docker run`, walau normal lewat `docker exec`.)
> Lalu pastikan container tetap bersih:
> `docker exec op3 bash -lc 'source /opt/ros/humble/setup.bash && python3 -c "import numpy,cv_bridge;print(numpy.__version__)"'` → harus `1.26.4`.

### Changing the model

Edit `model_path` in the matching param YAML
(`yolo_ball_detector_params.yaml` for `.pt`,
`yolo_ball_detector_onnx_params.yaml` for `.onnx`,
`yolo_ball_detector_openvino_params.yaml` for OpenVINO) and **rebuild**
(`colcon build --packages-select op3_ball_detector`) so the change lands in
`install/`. The value may be an absolute path, or relative to the package share
dir — e.g. `model/v8s.pt` resolves to
`<share>/op3_ball_detector/model/v8s.pt`. The launch args below default to empty
and only override the YAML when you pass them, so the YAML always wins otherwise.

Run the node standalone:

```bash
ros2 run op3_ball_detector yolo_ball_detector.py --ros-args \
    --params-file install/op3_ball_detector/share/op3_ball_detector/config/yolo_ball_detector_params.yaml
```

## Launch arguments

| Arg | Default | Description |
| --- | --- | --- |
| `use_usb_cam` | `true` | Also start the `usb_cam` node. |
| `image_topic` | `/usb_cam_node/image_raw/compressed` | Camera topic mapped to `image_in`. |
| `use_compressed` | `""` (use YAML) | Override only: `true` = `CompressedImage`, `false` = raw `Image`. |
| `model_path` | `""` (use YAML) | Override only: abs path or relative-to-share weights. |
| `device` | `""` (use YAML) | Override only: `cpu` or e.g. `cuda:0`. |

> The last three default to empty = **use the YAML**. They override the YAML
> only when you pass a non-empty value, so editing the YAML always takes effect.

See `config/yolo_ball_detector_params.yaml` for the full parameter list
(confidence/IoU thresholds, `imgsz`, `target_classes`, `detection_rate`, …).
