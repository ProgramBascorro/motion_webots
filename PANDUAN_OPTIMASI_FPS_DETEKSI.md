# Menaikkan FPS deteksi bola di robot OP3 ber-mini-PC Intel

**Untuk Claude di robot lain.** Dokumen ini memindahkan strategi yang dipakai di
ALPHONSE (Intel NUC10i5FNH, i5-10210U) dari **7–11 FPS** saat demo penuh
menjadi **23,3 FPS**, terukur pada demo sungguhan.

Semua angka di sini **hasil ukur di ALPHONSE**, bukan perkiraan. Angka di robot
Anda **akan berbeda**. Dokumen ini memberi Anda *metode dan alatnya*; angkanya
dipakai sebagai pembanding, bukan target yang harus dikejar.

---

## ATURAN NOMOR SATU: ukur dulu, jangan tebak

Gejala "FPS deteksi rendah" punya **lima** sebab yang berbeda total, dan
kelimanya terlihat sama dari luar. Di ALPHONSE, tiga dugaan yang paling
masuk akal ternyata **salah** dan baru ketahuan setelah diukur:

| Dugaan yang wajar | Kenyataan di ALPHONSE |
|---|---|
| "CPU-nya kurang kuat" | Inferensi cuma 17 ms; CPU menganggur separuh waktu |
| "detektornya belum terintegrasi ke demo" | Sudah, sejak awal — log membuktikannya |
| "`rqt` yang berlangganan gambar bikin berat" | Diuji langsung: **nol** pengaruh ke FPS |
| "governor CPU perlu diubah ke performance" | Clock stabil 3300–3400 MHz; tidak perlu |
| "thermal throttling bukan masalah" | **Awalnya benar, lalu jadi salah** di bawah beban nyata |

Jangan mengulang tebakan-tebakan itu. Ikuti urutan diagnosis di bawah.

---

## Langkah 1 — Cari tahu sebabnya yang mana (15 menit)

Jalankan **demo penuh**, lalu di terminal kedua kumpulkan empat angka ini.
Jangan lewati satu pun; masing-masing menyingkirkan satu sebab.

### 1a. Berapa FPS-nya, dan berapa lama inferensinya?

```bash
# baca baris log detektor yang muncul tiap 5 detik, atau:
ros2 topic echo /ball_detector_node/fps
```
Baris lognya berbentuk:
```
FPS 23.5 (avg 23.8) | infer 16.1 ms | loop 19.2 ms | frames 3829 | no ball
```
Bandingkan `1000/loop_ms` dengan `FPS`:

- **`1000/loop_ms` JAUH di atas FPS** → detektornya menganggur. Sebabnya
  **bukan** komputasi. Lanjut ke 1b dan 1c.
- **`1000/loop_ms` ≈ FPS** → detektornya memang mentok komputasi. Lanjut ke 1d.

### 1b. Apakah KAMERA yang membatasi? (paling sering terlewat)

```bash
v4l2-ctl -d /dev/video0 -C exposure_dynamic_framerate,exposure_time_absolute,gain
```

**Rumusnya mutlak: `FPS maksimum = 1 / waktu_pajan`.**
`exposure_time_absolute` satuannya 100 µs, jadi `666` = 66,6 ms = **maksimum
15,0 FPS**, tidak peduli sekuat apa CPU-nya.

Kalau `exposure_dynamic_framerate = 1`, kamera **boleh memperlambat dirinya
sendiri** supaya sensor sempat memajan lebih lama di tempat gelap. Inilah
sebabnya FPS turun **saat kamera digerakkan** — mengarah ke area lebih gelap.
→ Perbaikan di Langkah 3.

### 1c. Siapa yang mencuri CPU dari detektor?

```bash
./script.sh --cpu-thieves          # atau: scripts/op3_cpu_thieves.py
```
Jalankan **selama demo berlangsung**, **dari host** (bukan di dalam container —
kalau dari dalam, proses desktop tidak akan terlihat sama sekali).

Alat ini menjawab pertanyaan yang tidak bisa dijawab `top`/`htop`: proses mana
yang duduk di inti **sibling Hyper-Threading** milik detektor. Baca Langkah 2
untuk memahami kenapa itu yang penting. → Perbaikan di Langkah 2.

### 1d. Apakah panas?

```bash
cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_total_time_ms
# catat, jalankan demo 60 detik, catat lagi
```
Kalau tambahannya > 20% durasi demo, panas ikut bermain. → Perbaikan di
Langkah 5 (yang juga menyembuhkan ini).

> **Peringatan tentang counter ini:** nilainya **kumulatif sejak boot**.
> Membacanya sekali tidak berarti apa-apa — yang bermakna hanya SELISIH
> sebelum/sesudah beban. Di ALPHONSE saya sempat menyimpulkan "panas bukan
> masalah" dari satu bacaan saat mesin santai; kesimpulan itu terbukti salah
> begitu diukur di bawah beban.

---

## Langkah 2 — Partisi inti CPU (gratis, tanpa rebuild)

### Kenapa ini penting, dan kenapa `taskset` saja tidak cukup

CPU seri-U Intel (i5-10210U dan sejenisnya) punya **4 inti FISIK / 8 utas**.
Pasangan Hyper-Threading di Linux biasanya `0↔4  1↔5  2↔6  3↔7` — **verifikasi
dulu, jangan diasumsikan**:

```bash
for c in 0 1 2 3 4 5 6 7; do
  echo -n "cpu$c: "; cat /sys/devices/system/cpu/cpu$c/topology/thread_siblings_list
done
```

Dua utas dalam satu pasangan **berbagi unit eksekusi**. Mereka bukan dua mesin,
melainkan satu mesin yang dipakai bergantian. Terukur di ALPHONSE:

```
YOLO di inti 0-3, inti 4-7 sepi ....  40,3 ms
YOLO di inti 0-3, inti 4-7 sibuk ...  84,2 ms     <- 2,1x lebih lambat
```

**Jebakan besar:** `taskset -c 0-2` pada detektor **TIDAK mencadangkan** inti
0-2 untuknya. Ia cuma melarang detektor keluar dari sana. Proses lain tetap
bebas masuk — dan lebih buruk lagi, bebas menempati sibling-nya.

Konsekuensinya, default lama `cpu_affinity: 0-3` (satu utas per inti fisik)
**tidak melindungi apa pun**: median `0-3` dan `0-7` praktis identik
(40,7 vs 40,9 ms). Yang menentukan bukan afinitas detektor, melainkan **siapa
yang menempati sibling-nya**.

### Peta yang dipakai

| Inti fisik | Utas | Penghuni |
|---|---|---|
| P0, P1, P2 | 0,1,2 (+ sibling 4,5,6 **dikosongkan**) | detektor bola SAJA |
| P3 | 3,7 | usb_cam, op_demo_node, head_tracking, desktop |

Detektor butuh **3 inti**. Dengan 2 inti ia terlalu lambat (terukur ~44 ms di
384x640) — jangan diperas lebih jauh.

### Yang perlu diubah

**`op3_ball_detector/launch/yolo_ball_detector_openvino.launch.py`**
```python
DeclareLaunchArgument('cpu_affinity',     default_value='0-2')
DeclareLaunchArgument('aux_cpu_affinity', default_value='3,7')
```
Diterapkan lewat `prefix='taskset -c %s'` pada node (pola sudah ada di berkas).

**`op3_demo/launch/demo.launch.xml`**
```xml
<arg name="aux_launch_prefix" default="taskset -c 3,7"/>
...
<node pkg="op3_ball_localization" exec="head_tracking_node"
      launch-prefix="$(var aux_launch_prefix)"> ...
<node pkg="op3_demo" exec="op_demo_node"
      launch-prefix="$(var aux_launch_prefix)"> ...
```
`launch-prefix` adalah atribut resmi launch XML (`Node.parse` mewarisi
`ExecuteProcess.parse`) — sudah diverifikasi bekerja.

**Desktop** — pesaing terbesar, dan `taskset` di berkas launch tidak
menyentuhnya sama sekali:
```bash
./script.sh --match-mode on      # systemctl set-property --runtime user.slice AllowedCPUs=3,7
```

> **JEBAKAN:** `match-mode` bekerja lewat cgroup `user.slice`, jadi ia hanya
> mengurung **desktop host**. `rqt`/`rviz` yang dijalankan **di dalam
> container** ada di cgroup docker dan **tidak tersentuh**. Tutup alat debug
> saat demo sungguhan, atau jalankan `taskset -c 3,7 rqt`.

### Hasil terukur

| Kondisi | inferensi | p95 |
|---|---|---|
| 3 pesaing BEBAS | 47,7 ms | 114 ms (8,7 FPS) |
| 3 pesaing DIKURUNG ke 3,7 | **33,6 ms** | 59 ms (16,9 FPS) |

### Soal mematok `op3_manager` — JANGAN, kecuali terbukti perlu

Pengukuran di ALPHONSE menunjukkan `op3_manager` adalah pencuri **nomor satu**:
17,6% CPU, **seluruhnya** di inti sibling, tidak dipatok. Penjadwal menaruhnya
di sana karena 4,5,6 **terlihat menganggur** — kernel tidak tahu unit
eksekusinya sudah terpakai.

**Meski begitu, di ALPHONSE ia TIDAK dipatok, dan itu keputusan yang benar.**
Sesudah Langkah 5 (INT8), FPS sudah mentok laju kamera; mematoknya cuma
memangkas inferensi ~17 → ~15 ms **tanpa menambah FPS satu pun**, sambil
mengambil risiko pada loop bus servo 125 Hz. Patok hanya kalau FPS masih di
bawah target **sesudah** semua langkah lain, dan verifikasi kesehatan bus servo
sesudahnya.

---

## Langkah 3 — Lepas rem laju kamera

Kalau Langkah 1b menunjukkan `exposure_dynamic_framerate = 1`:

**`op3_ball_detector/launch/yolo_ball_detector_openvino.launch.py`** — tambahkan
aksi yang berjalan **sesudah** `usb_cam` membuka perangkat:

```python
RegisterEventHandler(OnProcessStart(
    target_action=usb_cam_node,
    on_start=[TimerAction(period=2.0, actions=[ExecuteProcess(
        cmd=['bash', '-c',
             'v4l2-ctl -d %s -c exposure_dynamic_framerate=0 || true' % camera_device],
    )])]),
    condition=IfCondition(LaunchConfiguration('use_usb_cam')))
```

Kenapa di berkas launch, bukan di skrip start container: kontrol ini menempel
pada perangkat dan **kembali ke bawaan begitu webcam dicabut-pasang**. Di
launch, ia dipasang ulang setiap kali detektor jalan.

`usb_cam` **tidak punya** parameter untuk kontrol ini — ia cuma mengenal
`autoexposure`/`exposure`. Harus lewat `v4l2-ctl`.

Ini **tidak** mematikan auto-exposure: kamera tetap menyesuaikan cahaya, cuma
tidak boleh lagi melanggar laju yang diminta.

### Pilih laju kamera — CEK DULU apa yang ditawarkan perangkat keras

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```
Kamera ALPHONSE menawarkan **30 / 24 / 20 / 15 / 10 / 7,5 / 5** di 640x360.
**25 FPS tidak ada.** Menulis angka yang tidak ada di `camera_param.yaml` akan
diam-diam jatuh ke mode terdekat. Jangan menulis angka "bulat" tanpa mengecek.

**Laju kamera harus DI ATAS target FPS**, bukan sama dengan. Kalau target
lantai 20 dan kamera disetel 20, hasil sempurnanya 20,0 dan sentakan sekecil
apa pun menjatuhkannya ke 19.

### Trade-off cahaya — buka jujur ke operator

Laju frame membatasi waktu pajan, jadi menaikkan FPS **memotong cahaya** di
adegan gelap:

| laju | batas pajan | cahaya relatif |
|---|---|---|
| 15 FPS | 66,6 ms | 100% |
| 20 FPS | 50,0 ms | 75% (−0,41 EV) |
| 24 FPS | 41,6 ms | 62% (−0,68 EV) |
| 30 FPS | 33,3 ms | 50% (−1,0 EV) |

Cek `gain` dulu: kalau sudah mentok (255 di ALPHONSE), sensor **tidak bisa**
menebus cahaya yang hilang.

**Tapi batas itu hanya menggigit kalau adegannya cukup gelap.** Terukur pada
pencahayaan ruangan biasa, auto-exposure berhenti di 33,3 ms — di bawah batas
41,6 ms — sehingga kecerahan sebelum/sesudah identik (148,3 vs 147,7 dari 255).
Pada cahaya cukup, kenaikan laju ini **gratis**.

> **Jangan pakai baris `mean level` dari pesan `First image received`** untuk
> menilai kecerahan: itu frame PERTAMA, sebelum auto-exposure konvergen. Di
> ALPHONSE sempat terbaca 7,2/255 padahal steady state-nya 147,7 — nyaris saya
> laporkan sebagai kerusakan berat.

---

## Langkah 4 — `detection_rate` harus JAUH di atas laju kamera

Ini yang paling tidak terduga dan paling murah diperbaiki.

Node memisahkan inferensi dari kedatangan frame: timer menyala pada
`detection_rate`, mengambil gambar terbaru, lalu memprosesnya. Executor rclpy
**satu utas**. Kalau laju timer terlalu dekat dengan laju kamera, keduanya
beradu-layang:

```
timer menyala -> _latest_image masih None -> keluar
5 ms kemudian frame tiba, lalu cuma duduk menunggu
tik timer berikutnya baru satu periode lagi   <- frame telat penuh
```

Terukur (kamera 20 FPS, inferensi ~30 ms):

| `detection_rate` | FPS nyata |
|---|---|
| 25 Hz | **13,4** (33% frame hangus) |
| 60 Hz | **19,4** (mentok kamera) |

**Aritmetika untuk memilih angkanya:**
```
periode_kamera - inferensi > periode_timer
contoh 24 FPS: 41,7 − 30 = 11,7 ms  ->  timer harus > 85 Hz
```
ALPHONSE memakai **100 Hz**. Tik yang menyala saat belum ada frame baru harganya
cuma satu lock + satu tes None — praktis gratis. **Patokan aman: ≥ 4x laju
kamera**, dan hitung ulang setiap kali laju kamera diubah.

---

## Langkah 5 — Pangkas komputasi modelnya

Dua tindakan terpisah. Yang pertama gratis dan tanpa risiko; yang kedua butuh
verifikasi akurasi.

### 5a. Ekspor ulang PERSEGI PANJANG mengikuti rasio kamera (gratis)

Kamera 640x360, tapi `imgsz` persegi memaksa Ultralytics mem-padding jadi
640x640: hanya `230400/409600 = 56%` piksel berisi gambar. **44% komputasi
habis memproses bar abu-abu.**

Bentuk 384x640 mengikuti rasio kamera (360 dibulatkan ke kelipatan stride 32).
Skala letterbox tetap 1,0 di kedua kasus, jadi bola **tidak diperkecil sedikit
pun** — ini murni membuang padding, bukan menukar akurasi dengan kecepatan.

```bash
yolo export model=ball_yolov8n_best.pt format=openvino imgsz=384,640
```
Terukur: **42 → 27 ms (1,73x)**, deteksi 30/30 sama persis, conf min malah naik
sedikit (0,848 vs 0,837).

> Node butuh dukungan `imgsz` non-persegi. Di repo ini sudah ada parameter
> `imgsz_hw: [384, 640]` di `yolo_ball_detector.py`. Kalau robot Anda belum
> punya, port dulu.

**HITUNG UNTUK KAMERA ANDA.** Kalau kameranya 1280x720, bentuk yang tepat
**bukan** 384x640. Rumusnya: pertahankan rasio kamera, bulatkan ke kelipatan 32.

> **`reshape()` saat runtime TIDAK BISA.** Grid anchor sudah dibakar jadi
> konstanta waktu ekspor; OpenVINO menolak dengan
> `Argument shapes are inconsistent`. Harus ekspor ulang.

### 5b. Kuantisasi INT8 (inilah yang menyelamatkan kasus demo penuh)

Ini yang membuat detektor **tetap masuk jatah walau CPU diperebutkan**:

| kondisi (45 detik berkelanjutan) | FP32 | INT8 | jatah 24 FPS |
|---|---|---|---|
| tanpa pesaing | 30,1 ms | **16,9 ms** | 41,7 ms |
| **3 pesaing bebas (= demo)** | 47,7 ms ✗ | **28,4 ms** ✓ | 41,7 ms |
| 3 pesaing dikurung | 33,6 ms | **18,5 ms** | 41,7 ms |

**Efek panasnya jauh lebih besar dari dugaan** — ini bonus yang tidak
direncanakan: run 45 detik menambah throttling paket **4100 ms (FP32) → 5 ms
(INT8)**.

Rantai alatnya (ada di `scripts/`, port apa adanya):

```bash
# 1. rekam frame kalibrasi DARI KAMERA ROBOT, di lapangan, bola berbagai jarak
scripts/capture_calib_frames.py --count 300

# 2. kuantisasi — WAJIB di container --rm, lihat peringatan di bawah
docker run --rm --entrypoint /bin/bash -v <repo>:/ros2_ws <image> -lc '
  pip3 install --no-cache-dir nncf &&
  python3 /ros2_ws/scripts/quantize_ball_int8.py'

# 3. gerbang kecepatan DAN akurasi
scripts/compare_ball_models.py
```

> **WAJIB container `--rm`, jangan `docker exec` ke container kerja.**
> `nncf` (dan ekspor ONNX) menyeret **numpy 2**, dan numpy 2 **merusak
> `cv_bridge`** di ROS Humble. Ini sudah pernah terjadi di ALPHONSE dan
> melumpuhkan seluruh pipeline. Container `--rm` membuang instalasinya begitu
> selesai. Verifikasi sesudahnya:
> ```bash
> docker exec <container> bash -lc 'source /opt/ros/humble/setup.bash && \
>   python3 -c "import numpy,cv_bridge;print(numpy.__version__)"'   # harus 1.26.4
> ```

> **Jebakan `docker run`:** entrypoint image ini **menelan argumen** pada
> `docker run` (walau normal lewat `docker exec`). Wajib pakai
> `--entrypoint /bin/bash`. Gejalanya: `[docker-entrypoint] launching:` kosong,
> exit 0, dan tidak ada yang terjadi.

**Gerbang kecepatan:** CPU seri-U ini **AVX2 tanpa VNNI** (cek `grep vnni
/proc/cpuinfo` — kosong berarti tidak punya). INT8 di AVX2 cuma **1,3–1,8x**,
bukan 2x. Kalau hasil ukurnya di bawah ~1,3x, **batalkan** — tidak sepadan
dengan risiko akurasi.

**Gerbang akurasi — jangan pernah dilewati.** Kuantisasi SELALU lebih cepat dan
SELALU berpotensi kurang akurat; mengukur salah satunya saja menghasilkan
keputusan yang salah. Di ALPHONSE: FP32 30/30 conf min 0,848; INT8 30/30 conf
min 0,843 — praktis tidak berubah. **Conf MINIMUM lebih penting daripada
rata-rata**: yang membuat pelacak kehilangan bola adalah frame terburuk.

**Kalibrasi harus dari kamera robot itu sendiri**, di kondisi lapangan
sebenarnya. Kuantisasi mempelajari rentang nilai dari contoh yang diberikan;
kalibrasi di ruangan lalu dipakai di lapangan terang adalah risiko nyata.

---

## Urutan yang disarankan

1. **Langkah 1** — diagnosis. Jangan lewati.
2. **Langkah 3 + 4** — kamera dan `detection_rate`. Gratis, tanpa risiko,
   sering memberi lompatan terbesar.
3. **Langkah 5a** — ekspor persegi panjang. Gratis, tanpa risiko akurasi.
4. **Langkah 2** — partisi inti. Gratis, tapi perlu diuji di robot.
5. **Langkah 5b** — INT8. Paling ampuh untuk kasus demo penuh, tapi satu-satunya
   yang **wajib** diverifikasi akurasinya.

---

## Verifikasi akhir

Jalankan **demo penuh** (bukan detektor sendirian — angkanya beda jauh):

1. **FPS**: `ros2 topic echo /ball_detector_node/fps`. **Nilai MIN, bukan
   rata-rata** — keluhan operator selalu ada di ekornya.
2. **Pencuri CPU**: `./script.sh --cpu-thieves` selama demo.
3. **Panas**: selisih `package_throttle_total_time_ms` sebelum/sesudah.
4. **Akurasi deteksi**: bola diam, bandingkan `conf` pada posisi & pencahayaan
   yang sama sebelum/sesudah.
5. **Bus servo tidak turun**: laju `present_joint_states` tetap, tidak ada
   `first bulk read fail` baru di `~/.ros/log`. **Wajib** kalau Anda mematok
   `op3_manager`.

---

## Nilai final di ALPHONSE (pembanding, bukan target)

```
kamera        : 640x360 @ 24 fps, exposure_dynamic_framerate=0
model         : ball_yolov8n_384x640_int8_openvino_model
imgsz_hw      : [384, 640]
detection_rate: 100.0 Hz
cpu_affinity  : 0-2      (detektor)
aux_cpu_affin.: 3,7      (usb_cam, demo, head_tracking)
op3_manager   : TIDAK dipatok  (sengaja)
match-mode    : tidak wajib menyala
ultralytics 8.4.82 · openvino 2026.2 · ROS 2 Humble
```

Hasil pada demo sungguhan: **23,3 FPS**, inferensi 15,7–17,7 ms, throttling 0 ms
per 30 detik. Lonjakan inferensi ke 63 ms sempat terjadi dan **FPS tidak turun**
— itulah gunanya headroom.

---

## Kalau robot Anda berbeda

- **Kamera bukan 640x360** → hitung ulang bentuk ekspor di 5a (rasio kamera,
  kelipatan 32). Ingat radius bola diterbitkan dalam **piksel gambar asli**
  (`yolo_ball_detector.py::_extract_circles`), jadi pemakainya harus mengikuti
  lebar kamera.
- **CPU bukan 4c/8t** → peta inti di Langkah 2 berubah total. Baca
  `thread_siblings_list`, jangan menyalin `0-2` / `3,7` mentah-mentah.
- **CPU punya VNNI** (`grep vnni /proc/cpuinfo` ada isinya) → INT8 akan lebih
  dari 2x; gerbang kecepatan pasti lolos.
- **Tidak ada Hyper-Threading** → Langkah 2 sebagian besar tidak berlaku;
  `op3_cpu_thieves.py` akan mengatakannya sendiri.
- Untuk perbedaan CHRONUS vs ALPHONSE di luar soal FPS, baca
  `PANDUAN_PORTING_KE_CHRONUS.md` di akar repo yang sama.
