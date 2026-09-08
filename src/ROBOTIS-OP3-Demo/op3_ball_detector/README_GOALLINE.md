# Goal-line detector — garis gawang (op3_ball_detector)

Menjalankan model **`best_landmark_v8n`** dari `model/landmark_deploy.zip`
(5 kelas: `L`, `T`, `X`, `goalpost`, `center_circle`) dan mengubah kotak-kotak
YOLO itu menjadi **satu jawaban geometris**: di mana garis gawang berada di
dalam gambar. Setiap kali dijalankan, program **melaporkan FPS yang didapat**.

Node ini berdiri sendiri — tidak menyentuh pipeline bola, jadi aman dijalankan
berbarengan dengan `yolo_ball_detector.py` di kamera yang sama.

## Cara pakai cepat (di dalam container `op3`)

```bash
# 1) Ukur dulu: berapa FPS model ini di mesin ini? (tanpa ROS, dari video)
docker exec -it op3 bash -lc \
  'cd /ros2_ws && ros2 run op3_ball_detector goalline_fps_benchmark.py \
     --source recording_20260128_044704.mp4 --conf 0.35'

# 2) Jalankan di robot, menumpang kamera yang sudah hidup
docker exec -it op3 bash -lc \
  'ros2 launch op3_ball_detector yolo_goalline_detector.launch.py'

# 3) Nyalakan sekalian kameranya (kalau detektor bola belum jalan)
docker exec -it op3 bash -lc \
  'ros2 launch op3_ball_detector yolo_goalline_detector.launch.py use_usb_cam:=true'

# 4) Lihat hasilnya
docker exec -it op3 bash -lc 'ros2 run rqt_image_view rqt_image_view /goalline_detector_node/image_out'
docker exec -it op3 bash -lc 'ros2 topic echo /goalline_detector_node/fps'
```

Tekan `Ctrl-C` untuk berhenti — ringkasan FPS lengkap dicetak saat itu juga.

## Files

| File | Purpose |
| --- | --- |
| `scripts/yolo_goalline_detector.py` | Node ROS 2. Kamera → garis gawang + laporan FPS. |
| `scripts/goalline_fps_benchmark.py` | Program mandiri (tanpa ROS): video/kamera/folder gambar → laporan FPS. |
| `scripts/goalline_core.py` | Geometri garis gawang + penggambaran. Dipakai **kedua** program di atas, supaya hasilnya tidak pernah beda. Bukan node. |
| `scripts/detector_fps.py` | `FpsMeter` — penghitung FPS bersama untuk detektor bola, landmark, dan garis gawang. Bukan node. |
| `config/yolo_goalline_detector_params.yaml` | Semua parameter node (**satu-satunya sumber kebenaran**, termasuk `model_path` dan `imgsz`). |
| `launch/yolo_goalline_detector.launch.py` | Menjalankan node (opsional + `usb_cam`), lengkap dengan `taskset`/`OMP_NUM_THREADS`. |
| `model/best_landmark_v8n_openvino_model/` | Bobot OpenVINO INT8 (`.xml` + `.bin` + `metadata.yaml`) hasil ekstrak `landmark_deploy.zip`. **Ini yang dipakai secara default.** |
| `model/best_landmark_v8n.pt`, `.onnx` | Bobot yang sama dalam format lain, untuk membandingkan backend. |
| `model/landmark_export.yaml` | Daftar kelas apa adanya dari pelatihan. |

## Bagaimana garis gawangnya dihitung

Model tidak punya kelas "garis gawang" — jadi garisnya **direkonstruksi**:

1. Setiap kotak `goalpost` menyumbang **kakinya**: titik tengah sisi bawah
   kotak, tempat tiang menyentuh tanah. Hanya titik ini yang dijamin berada
   di atas garis gawang.
2. Sebuah garis dicocokkan ke kaki-kaki itu dengan **total least squares**
   (SVD), bukan regresi `y = ax + b` — gawang yang dilihat dari depan tampak
   tegak di layar dan akan membuat regresi biasa meledak.
3. Simpangan `L`/`T`/`X` yang jaraknya ≤ `landmark_band_px` dari garis itu
   ikut ditarik masuk lalu garis dicocokkan ulang. Simpangan yang memang ada
   di garis gawang memanjangkan segmen ke arah pojok lapangan dan menstabilkan
   sudutnya.
4. Hasil ditolak kalau titik-titik pendukungnya tidak cukup segaris
   (`min_linearity`) atau segmennya lebih pendek dari `min_length_px` —
   penjaga terhadap dua deteksi tiang yang bertumpuk.
5. Ujung-ujung segmen dihaluskan (EMA, `smoothing`) terhadap frame sebelumnya,
   dan ketika satu frame gagal, garis lama **ditahan** selama `hold_sec` alih-
   alih berkedip hilang.

`width_px` adalah jarak **tiang-ke-tiang** (lebar gawang), sedangkan
`length_px` adalah panjang segmen yang terlihat — bisa lebih panjang kalau ada
simpangan yang ikut dipakai.

## Topics (di bawah namespace `goalline_detector_node`)

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| sub | `image_in` | `CompressedImage` / `Image` | Stream kamera (di-remap di launch). |
| sub | `enable` | `std_msgs/Bool` | Jeda/lanjutkan deteksi. |
| pub | `goal_line` | `geometry_msgs/PolygonStamped` | **Piksel.** `[0]`,`[1]` = ujung segmen; `[2]`,`[3]` = kaki tiang (hanya ada kalau dua tiang terlihat). Kosong = tidak ketemu. |
| pub | `goal_center` | `geometry_msgs/PointStamped` | `x,y` **ternormalisasi `[-1,1]`** seperti detektor bola (-1 kiri/atas, +1 kanan/bawah); `z` = lebar gawang dalam piksel. Ini yang dipakai loop pengarah kepala/badan. |
| pub | `goal_found` | `std_msgs/Bool` | Estimasi valid di frame ini atau tidak. |
| pub | `detections` | `vision_msgs/Detection2DArray` | Semua kotak landmark mentah + label + skor (piksel). |
| pub | `fps` | `std_msgs/Float32` | Laju proses langsung (jendela geser). |
| pub | `image_out` | `sensor_msgs/Image` | Gambar beranotasi. Overlay hanya digambar **kalau ada yang subscribe**, jadi gratis saat pertandingan. |

## Laporan FPS — tiga tempat

1. **Saat start**, sebelum frame kamera pertama datang, model diukur pada
   beberapa frame kosong:
   `Model ceiling on this machine: 36.9 FPS (27.1 ms/frame over 5 warm frames)`.
   Inferensi pertama (kompilasi IR OpenVINO, ~200 ms) sengaja dibuang supaya
   angkanya jujur. Kalau plafon ini di bawah `detection_rate`, node bilang
   terus terang bahwa lajunya tidak akan tercapai.
2. **Selama jalan**, tiap `fps_log_period` detik:
   `FPS 26.1 (avg 25.8) | infer 21.9 ms | loop 26.3 ms | frames 482 | center=(514,618) width=832px angle=-3.6deg ...`
   Nilai yang sama diterbitkan terus-menerus di topik `fps`.
3. **Saat berhenti** (termasuk `Ctrl-C`), blok ringkasan:

```
===== goal-line detector =====
frames processed : 482
elapsed          : 18.44 s
AVERAGE FPS      : 26.08
min / max FPS    : 24.19 / 27.96
target rate      : 10.0 Hz  (achieved 100%)
inference        : avg   23.4 ms  p50   21.9  p95   33.3  max   39.0
preprocess       : avg    3.0 ms  p50    2.8  p95    3.9  max    6.1
postprocess      : avg    1.0 ms  p50    1.0  p95    1.4  max    3.1
loop total       : avg   27.8 ms  p50   26.3  p95   38.2  max   43.5
goal line found  : 45 / 482 frames (9.3%)
==============================
```

## Program mandiri: `goalline_fps_benchmark.py`

Deteksi yang **persis sama** (kode geometri yang sama), tapi membaca berkas
video, indeks kamera, atau folder gambar — dan tanpa ROS sama sekali. Gunanya
menjawab "model ini dapat berapa FPS di mesin ini?" sebelum dipasang ke robot.

```bash
# klip rekaman, headless
ros2 run op3_ball_detector goalline_fps_benchmark.py --source klip.mp4

# kamera robot, 300 frame, simpan video beranotasi + CSV per frame
ros2 run op3_ball_detector goalline_fps_benchmark.py --source 0 \
  --max-frames 300 --save out.mp4 --csv fps.csv

# bandingkan backend: OpenVINO INT8 (default) vs PyTorch .pt vs ONNX
ros2 run op3_ball_detector goalline_fps_benchmark.py --source klip.mp4 \
  --model model/best_landmark_v8n.pt
```

`--help` menampilkan seluruh saklarnya (ambang, geometri, batas frame/detik).
Skripnya juga bisa dipanggil langsung dari `scripts/` tanpa `ros2 run`.

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


## Angka terukur di mesin ini (NUC10i5, container `op3`, OpenVINO INT8, 640 px)

| Kondisi | FPS | Inferensi |
| --- | --- | --- |
| OpenVINO INT8, benchmark offline, mesin senggang | **35,5** | 22 ms |
| OpenVINO INT8, dipin ke core 0–3 sambil ada beban lain | **26,1** | 23 ms |
| PyTorch `.pt` (bobot yang sama, core 0–3) | 12,9 | 73 ms |
| Node ROS + kamera palsu (decode video 720p ikut memakan core yang sama) | 6–12 | 35–110 ms |

Jadi **OpenVINO INT8 hampir 3x lebih cepat dari `.pt`** untuk bobot yang sama —
itulah kenapa YAML-nya menunjuk ke direktori OpenVINO. Bandingkan juga dengan
model bola 640 px yang 43 ms/frame: model landmark INT8 ini **dua kali lebih
murah**. Meski begitu,
`detection_rate` default sengaja ditahan di **10 Hz**: detektor bola dan
kontroler walking memperebutkan core yang sama (lihat
`reference_yolo_starves_servo_bus`).

## Yang perlu diketahui sebelum percaya hasilnya

- **Butuh dua tiang.** Dengan `min_goalposts: 2` (default), garis baru muncul
  kalau kedua tiang terlihat. Log akan menyebutkan alasannya apa adanya:
  `no goal line (only 1 goalpost(s), need 2)`.
- **Model ini mudah salah kira di dalam ruangan.** Pada klip uji
  `recording_20260128_044704.mp4` (rekaman lab, gawang tidak ada di frame),
  rangka robot yang hitam dan tegak terdeteksi sebagai `goalpost` dengan
  keyakinan 0,70. Itu soal data latih, bukan soal kode. Naikkan
  `confidence_threshold` di lapangan sungguhan, dan pakai `goal_found` +
  `width_px` sebagai penyaring akal sehat.
- **`imgsz` wajib 640.** IR OpenVINO model ini statis `[1,3,640,640]`; angka
  lain langsung ditolak OpenVINO. Untuk mengubahnya, ekspor ulang modelnya.
- **Jangan biarkan ultralytics memasang paket sendiri.** Launch file sudah
  menyetel `YOLO_AUTOINSTALL=false`; kalau menjalankan skripnya manual,
  ekspor sendiri variabel itu — memuat `.onnx` tanpa penjaga ini menarik
  NumPy 2 dan mematikan `cv_bridge` untuk semua node di workspace.
