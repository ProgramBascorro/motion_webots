# Panduan Memindahkan Upgrade ALPHONSE ke CHRONUS

Dokumen ini ditulis untuk **Claude yang bekerja di robot CHRONUS**, supaya bisa
memasang semua perbaikan yang sudah terbukti jalan di ALPHONSE tanpa mengulang
kesalahan yang sudah pernah dibayar mahal di sini.

Bahasanya sengaja sederhana. Kalau ada istilah yang harus dipakai, dijelaskan
sekali lalu dipakai konsisten.

- **Ditulis:** 27 Agustus 2026
- **Cabang sumber:** `ALPHONSE_NEW` (commit terakhir `eb0a45a`)
- **Cabang tujuan:** `CHRONUS_NEW`
- **Repo:** `https://github.com/ProgramBascorro/motion_webots.git`

---

## 0. Baca ini dulu — tiga aturan yang tidak boleh dilanggar

### Aturan 1: JANGAN `git merge ALPHONSE_NEW`

Per hari ini kedua cabang berbeda di **99 berkas, 8410 baris tambah, 2726 baris
hapus**. CHRONUS punya kerjaannya sendiri (simulasi Webots, bridge domain,
editor aksi, `WalkingSimPreview.jsx`, dan lain-lain) yang **tidak ada** di
ALPHONSE. Merge buta akan menimpa kerjaan itu atau melahirkan konflik yang
tidak bisa dinilai satu per satu.

Cara yang benar: **ambil per-upgrade**, satu per satu, dengan cek sebelum dan
sesudah. Bagian 3 dan 4 memandu itu.

### Aturan 2: Beberapa hal di ALPHONSE HARAM ikut ke CHRONUS

Robotnya beda perangkat keras. Salah menyalin satu baris ini bisa membuat
CHRONUS tidak bisa boot sama sekali. Rinciannya di bagian 2. Ringkasnya:

| Jangan disalin | Kenapa |
|---|---|
| Jalur serial `/dev/serial/by-id/...FT3WKHM6...` | Itu nomor seri U2D2 milik ALPHONSE. CHRONUS memakai `/dev/ttyOP3`. |
| Baris port OpenCR yang terpisah di `OP3.robot` | OpenCR ALPHONSE rusak di bus TTL, jadi dipindah ke USB. OpenCR CHRONUS masih normal di bus servo. |
| Semua angka bawaan `kick_*` di `demo.launch.xml` | Angka itu hasil ukur di ALPHONSE, dalam keadaan **digantung 29,5 cm**. |
| Sudut sapuan kepala `scan_tilt_*` | Hasil permintaan lapangan di ALPHONSE, bukan kebenaran umum. |

### Aturan 3: Yang sudah pernah diukur, jangan ditebak ulang

Ada tiga hal yang di ALPHONSE **sudah dua kali ditebak salah** dan dua kali
memakan waktu robot. Prosedur mengukurnya ada di bagian 7. Kalau ragu, ukur —
jangan baca komentar yaml, jangan menyimpulkan dari kesan gerakan, dan jangan
membandingkan foto dari dua run yang berbeda.

---

## 1. Apa saja yang mau dipindahkan (peta besar)

Ada **8 paket upgrade**. Diurutkan dari yang paling murah dan paling aman:

| # | Paket | Untuk mengatasi | Risiko |
|---|---|---|---|
| A | Pemisahan domain ROS 2 | Robot lain di LAN mengacak-acak modul sendi kita | Sangat rendah |
| B | Penangkap crash + perbaikan SIGSEGV | Node mati diam-diam, menyamar jadi 3 bug lain | Sangat rendah |
| C | Debounce tombol OpenCR | Tombol START/STOP kadang jalan kadang tidak | Rendah |
| D | Batas sendi `head_tilt` | Kepala tidak bisa menunduk cukup dalam | Sedang (mekanis) |
| E | Kamera 640x360 @ 15 FPS | Deteksi bola berumur 1,2 detik; kepala tak mengunci | Rendah |
| F | Logika tendang lengkap | Robot tidak pernah menendang | Sedang (paling besar) |
| G | Perampingan bulk read servo | Loop kontrol lambat / bacaan servo hilang | Sedang (menyentuh controller) |
| H | `y_offset` jadi knob lebar kaki | Parameter ada tapi tidak berpengaruh | Rendah |

Paket F yang paling besar dan paling banyak diminta operator. Tapi **kerjakan A
sampai E dulu** — kalau tidak, F akan gagal karena sebab yang bukan salah F.

---

## 2. Beda lingkungan ALPHONSE vs CHRONUS

Ini tabel paling penting di dokumen ini. **Periksa dulu di robot, jangan
percaya tabel ini bulat-bulat** — bisa saja sudah berubah sejak ditulis.

| Hal | ALPHONSE | CHRONUS (per 27-08-2026) | Akibatnya |
|---|---|---|---|
| Port servo | `/dev/serial/by-id/usb-FTDI_..._FT3WKHM6-if00-port0` | `/dev/ttyOP3` | Pertahankan punya CHRONUS |
| OpenCR (ID 200) | Port **terpisah**, lewat micro-USB CDC | **Satu bus** dengan servo (susunan asli OP3) | Lihat paket G |
| Kamera | 640x360 @ 15 FPS | 1280x720 @ 30 FPS | Lihat paket E |
| `ROS_DOMAIN_ID` | 42 + `ROS_LOCALHOST_ONLY=1` | Belum diatur (artinya 0) | Lihat paket A |
| Arg `kick_*` di launch | 11 argumen | **Tidak ada sama sekali** | Lihat paket F |
| `ball_center_topic` di detektor YOLO | Ada (diambil DARI CHRONUS) | **Sudah ada** | Tidak perlu diapa-apakan |
| Posisi robot saat ditera | **Digantung 29,5 cm** dari lantai | Belum diketahui | Angka `kick_camera_height` WAJIB diukur ulang |

### Cara memeriksa perangkat keras CHRONUS sendiri

```bash
# di HOST, bukan di container
ls -l /dev/serial/by-id/            # nama port yang sebenarnya ada
lsusb -t                            # OpenCR menempel di hub mana
```

Kalau `/dev/ttyOP3` ada dan `OP3.robot` cuma punya SATU baris port — berarti
OpenCR CHRONUS masih di bus servo, dan itu susunan yang sehat. Jangan diubah.

---

## 3. Blok cek: apa yang sudah ada, apa yang belum

Jalankan ini **di CHRONUS**, dari akar repo. Tidak mengubah apa pun, cuma
membaca. Aman diulang kapan saja.

```bash
#!/usr/bin/env bash
# cek_upgrade.sh — jawab "sudah / belum" untuk tiap paket upgrade
cek() {  # cek "nama" "berkas" "pola"
  printf "%-44s " "$1"
  if [ ! -f "$2" ]; then echo "BERKAS TIDAK ADA"; return; fi
  if grep -q -- "$3" "$2"; then echo "sudah"; else echo "BELUM"; fi
}

echo "== A. domain ROS 2 =="
cek "domain di docker-entrypoint" docker-entrypoint.sh "ROS_DOMAIN_ID"
cek "domain di op3_docker.sh"     scripts/op3_docker.sh "OP3_ROS_DOMAIN_ID"

echo "== B. crash handler =="
cek "penangkap crash" src/ROBOTIS-OP3-Demo/op3_demo/src/demo_node.cpp "op_demo_crash.log"

echo "== C. tombol =="
cek "debounce tombol" src/ROBOTIS-OP3/open_cr_module/src/open_cr_module.cpp "publishAllowed"

echo "== D. batas kepala =="
cek "head_tilt_max_deg" src/ROBOTIS-OP3/op3_head_control_module/src/head_control_module.cpp "head_tilt_max_deg"

echo "== E. kamera =="
grep -nE "image_width|framerate" src/ROBOTIS-OP3-Demo/op3_ball_detector/config/camera_param.yaml

echo "== F. logika tendang =="
cek "jendela pan"     src/ROBOTIS-OP3-Demo/op3_demo/include/op3_demo/ball_follower.h "kick_pan_right_min_deg_"
cek "ambang radius"   src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/ball_follower.cpp "kick_ball_radius_px_"
cek "tanda head_tilt" src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/ball_follower.cpp "head_tilt_sign_"
cek "niat operator"   src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/soccer_demo.cpp "soccer_requested_"
printf "%-44s %s\n" "jumlah arg kick di launch" \
  "$(grep -c '<arg name=\"kick_' src/ROBOTIS-OP3-Demo/op3_demo/launch/demo.launch.xml)"

echo "== G. bus servo =="
cek "OP3_SENSOR_DIV" src/ROBOTIS-Framework/robotis_controller/src/robotis_controller/robotis_controller.cpp "OP3_SENSOR_DIV"
cek "OP3_PROFILE"    src/ROBOTIS-Framework/robotis_controller/src/robotis_controller/robotis_controller.cpp "OP3_PROFILE"
printf "%-44s %s\n" "jumlah port di OP3.robot" \
  "$(sed -n '/\[ port info \]/,/\[ device info \]/p' src/ROBOTIS-OP3/op3_manager/config/OP3.robot | grep -c '^/dev/')"

echo "== H. walking =="
cek "with_y_offset" src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp "with_y_offset"
```

Hasil di CHRONUS saat dokumen ini ditulis: **semuanya "BELUM"**, kecuali
`ball_center_topic` di detektor YOLO yang justru asalnya dari CHRONUS.

### Melihat isi perubahan aslinya

```bash
git fetch origin ALPHONSE_NEW
git log --oneline origin/CHRONUS_NEW..origin/ALPHONSE_NEW     # daftar commit
git show <hash>                                               # isi satu commit
git diff origin/CHRONUS_NEW..origin/ALPHONSE_NEW -- <berkas>  # beda satu berkas
```

---

## 4. Paket upgrade, satu per satu

Format tiap paket sama: **Gejala → Sebab → Perbaikan → Berkas → Cara terapkan
→ Cara verifikasi → Jebakan**.

---

### Paket A — Pisahkan domain ROS 2

**Commit ALPHONSE:** `34f8573`

**Gejala.** Di tengah demo, kepala atau kaki "kehilangan parameter dan memakai
yang lain". Log `head_tracking_node` penuh baris
`kepala nyangkut di 'none' ... direbut kembali`, berulang selamanya. Log manager
menampilkan `Walking Disable` dan `base_module` muncul tanpa ada yang meminta.

**Sebab.** ROS 2 bawaannya `ROS_DOMAIN_ID=0` dengan DDS multicast ke seluruh
LAN. Robot **lain** di jaringan yang sama juga di domain 0, jadi kedua graph
menyatu. Pesan `/robotis/enable_ctrl_module` dari robot sebelah memindahkan
modul sendi di robot kita.

**Cara membuktikan (30 detik, tanpa menyentuh robot).** Matikan SEMUA node lokal
(manager, demo, tracker), lalu:

```bash
ros2 topic echo /robotis/enable_ctrl_module
```

Kalau masih ada pesan masuk padahal tidak ada node lokal — itu robot lain.

> Jangan percaya `ros2 node list` untuk ini: node asing sering tidak muncul di
> sana. Jangan percaya `ros2 topic info -v` juga: daemon menyimpan publisher
> hantu dari proses yang sudah mati.

**Perbaikan.** `ROS_DOMAIN_ID=42` + `ROS_LOCALHOST_ONLY=1`, dipasang di
**tiga jalan masuk** karena masing-masing melewati yang lain:

| jalan masuk | yang membacanya |
|---|---|
| `docker-entrypoint.sh` | PID 1 container |
| `/root/.bashrc` | `docker exec -it op3 bash` |
| `/etc/profile.d/op3-ros.sh` | `docker exec op3 bash -lc '...'` |
| `docker run -e ...` | container yang dibuat berikutnya |

> **Jebakan yang memakan waktu di sini:** `bash -lc` itu shell **login**. Ia
> membaca `/etc/profile`, **BUKAN** `.bashrc`. Karena itu memasang di `.bashrc`
> saja terlihat "sudah dipasang tapi tetap kosong".

**Cara terapkan.** Ambil `docker-entrypoint.sh` dan bagian env di
`scripts/op3_docker.sh` dari ALPHONSE. Angka domainnya bebas asal **bukan 0** —
kalau CHRONUS dan ALPHONSE dipakai bersamaan di satu ruangan, beri mereka angka
**berbeda** (misal ALPHONSE 42, CHRONUS 43).

**Verifikasi.**

```bash
scripts/op3_docker.sh doctor        # bagian 6 memeriksa isolasi ini
docker exec op3 bash -lc 'echo $ROS_DOMAIN_ID $ROS_LOCALHOST_ONLY'
```

**Jebakan.** Kalau CHRONUS memang perlu satu graph dengan PC lain (misal
Bascorro Studio di laptop terpisah), beri mereka domain yang **sama** dan
matikan `ROS_LOCALHOST_ONLY` **di kedua sisi**. Jangan kembali ke domain 0.

---

### Paket B — Penangkap crash + perbaikan SIGSEGV klok THROTTLE

**Commit ALPHONSE:** `df507e7`

**Gejala.** Tiga gejala yang kelihatannya tidak berhubungan, padahal satu sebab:

1. Tombol START "berhasil" (robot mulai jalan), lalu **tidak ada tombol yang
   berpengaruh lagi**.
2. Robot **tidak pernah menendang**.
3. Robot **terus berjalan dan tidak bisa dihentikan**.

**Sebab.** Pola ini:

```cpp
RCLCPP_INFO_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000, "...");   // SALAH
```

Makro THROTTLE menyimpan **referensi** ke klok, lalu memakainya lagi di
pernyataan berikutnya di dalam makro. `*rclcpp::Clock::make_shared()` membuat
`shared_ptr` **sementara** yang mati di akhir pernyataan pertama. Pemakaian
berikutnya membaca memori yang sudah dibebaskan → SIGSEGV.

Node mati ~0,5 detik setelah start. `walking_module` masih memegang amplitudo
terakhir, jadi robot terus berjalan — persis seperti "bug logika".

**Perbaikan.**

| kelas | pakai |
|---|---|
| turunan `rclcpp::Node` | `*this->get_clock()` |
| bukan Node (misal `BallFollower`) | anggota `rclcpp::Clock::SharedPtr log_clock_`, di-init di konstruktor |

Ditambah: `demo_node.cpp` memasang handler SIGSEGV/SIGABRT/SIGBUS/SIGFPE yang
menulis jejak tumpukan ke stderr **dan** ke `/tmp/op_demo_crash.log`. Ini
penting karena stderr node yang diluncurkan `ros2 launch` tidak tersimpan di
mana pun, dan di container **tidak ada `gdb`**.

**Cara terapkan.** Salin blok handler dari `demo_node.cpp` ALPHONSE, lalu sisir
seluruh repo CHRONUS:

```bash
grep -rn "make_shared()" --include=*.cpp --include=*.hpp | grep THROTTLE
```

Setiap yang ketemu harus diperbaiki.

**Verifikasi.** Sebelum menuduh logika demo apa pun, **pastikan dulu nodenya
masih hidup**:

```bash
pgrep -a op_demo_node
grep "process has died" ~/.ros/log/<run>/launch.log
cat /tmp/op_demo_crash.log
```

**Jebakan.** Bug ini kadang **tidak langsung crash** (memori bebasnya belum
ditimpa), jadi bisa lolos saat diuji di meja lalu mati di robot.

---

### Paket C — Debounce tombol OpenCR

**Commit ALPHONSE:** `db25a73` (akar), `75b4e1f` (akibatnya di demo)

**Gejala.** Tombol START bekerja, tombol STOP tidak. Atau kadang jalan kadang
tidak, tanpa pola.

**Sebab, dua lapis.**

*Lapis hulu (yang sebenarnya):* `open_cr_module` tidak punya debounce.
`handleButton()` menerbitkan `"start"` tiap tepi-lepas, tombolnya mekanis, dan
dibaca tiap 8 ms. Kontak yang memantul = **beberapa** `"start"` dari **satu**
tekanan. `"start"` itu toggle, jadi dua pesan = mati lalu nyala lagi. Jumlah
pantulan tidak tetap — makanya kadang berhasil.

*Lapis hilir:* demo memakai `on_following_ball_` sebagai penanda "sedang
jalan". Itu salah, karena `handleKick()` menolkannya sendiri, dan `handleKick()`
memblokir loop demo lebih dari 4 detik — tekanan saat menendang baru diproses
sesudahnya, saat flag sudah false, jadi tekanan "matikan" malah MENYALAKAN.

**Perbaikan.**
1. `publishAllowed()`: satu nama tombol maksimal sekali per **400 ms**, dihitung
   dari **penerbitan terakhir** (bukan dari tekanan) supaya rentetan pantulan
   tidak lolos satu per satu.
2. Ganti penanda jadi `soccer_requested_` = niat operator, bukan keadaan mesin.
3. `stopSoccerMode()` juga membatalkan `restart_soccer_` — kalau tidak,
   `process()` menyalakannya lagi sendiri satu tick kemudian.

**Verifikasi.** Semua sudah dicetak ke log; bedakan gejalanya begini:

| yang terlihat di log | artinya |
|---|---|
| tidak ada baris `TOMBOL` sama sekali | pesan tidak sampai (kabel / OpenCR / domain) |
| `dibuang: pantulan kontak (N ms ...)` | debounce bekerja, tekanan ganda ditolak |
| `TOMBOL 'start' ... demo TIDAK aktif` | belum masuk mode soccer |
| `tombol ... perpindahan mode masih diproses` | kena gerbang `apply_desired` |
| `TOMBOL ... JALAN (tekan = matikan)` lalu `Stop Soccer Demo` | jalur benar |

---

### Paket D — Batas sendi `head_tilt`

**Commit ALPHONSE:** `e43eecb`

**Gejala.** Kepala tidak mau menunduk lebih dari 30 derajat, walaupun yaml
minta lebih. Sapuan yang seharusnya melihat lantai malah berhenti di tengah.

**Sebab.** `head_control_module.cpp` memakai batas bawaan ROBOTIS
`max_angle_ head_tilt = +30 deg`, `min = -75 deg`. Batas itu dibuat untuk
konvensi asli OP3 (**negatif = menunduk**, jatah 75 derajat ke bawah). Di
ALPHONSE konvensinya **terbalik** (positif = menunduk), jadi jatah menunduknya
tinggal 30 derajat. Lebih jahat lagi: `checkAngleLimit()` **memotong diam-diam**,
tidak menolak — jadi tidak ada error apa pun yang muncul.

**Perbaikan.** Jatahnya ditukar: `max = +75`, `min = -30`, dan dijadikan
parameter ROS `head_tilt_max_deg` / `head_tilt_min_deg` supaya bisa ditera tanpa
compile ulang.

**PENTING untuk CHRONUS.** Cek dulu konvensinya sendiri (prosedur di bagian 7).
Kalau ternyata CHRONUS memakai konvensi **asli** (negatif = menunduk), maka
batasnya **jangan** ditukar — cukup jadikan parameter saja.

Petunjuk awal: `head_tracking.yaml` CHRONUS punya `scan_tilt_down_rad: 0.55`
dengan komentar *"looking toward floor"*. Angka positif untuk melihat lantai
berarti CHRONUS **kemungkinan besar sama terbaliknya** dengan ALPHONSE. Tapi
komentar yaml **pernah salah di ALPHONSE**, jadi tetap ukur.

**Jebakan.** Jangkauan +75 derajat itu **belum pernah dipakai** sebelumnya.
Coba dengan robot **di dudukan dulu**. Kalau mentok atau berbunyi, turunkan
`head_tilt_max_deg` bertahap. Jangan langsung di lantai.

---

### Paket E — Kamera 640x360 @ 15 FPS

**Commit ALPHONSE:** `a77992d`

**Gejala.** Kepala mendeteksi bola tapi **tidak pernah mengunci** — melesat
lewat bola lalu mengayun bolak-balik.

**Sebab, dan ini yang mengejutkan: usb_cam, bukan YOLO.** Detektor jalan 8 Hz
dan kelihatan sehat, tapi tiap deteksi sudah berumur **1241 ms**. Kepala
mengejar posisi bola satu detik yang lalu; pada laju sapuan 65 deg/s itu meleset
78 derajat.

`camera_param.yaml` memakai 1280x720 @ 30 FPS `mjpeg2rgb`. usb_cam harus
membongkar MJPEG 720p lalu memampatkannya lagi untuk topik `/compressed`, 30
kali sedetik = **73 % CPU**, sementara YOLO sudah memakan 3,2 dari 8 inti.
Frame menumpuk dan stempel waktunya menua.

**Aturan yang harus diingat: laju topik TIDAK memberi tahu keterlambatan.**
Selalu ukur umur stempel:

```python
st  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
age = self.get_clock().now().nanoseconds * 1e-9 - st
```

**Perbaikan, dua bagian.**
1. Kamera **640x360 @ 15 FPS**. Pertahankan rasio **16:9** — pindah ke 640x480
   akan mengubah bidang pandang dan membuat `fov_width_deg` / `fov_height_deg`
   di `head_tracking.yaml` tidak sahih lagi. Modelnya sendiri mengecilkan gambar
   ke 640 px, jadi 720p memang mubazir.
2. **Antrean langganan gambar kedalaman 1**, bukan `qos_profile_sensor_data`
   (kedalaman 5). Detektor hanya memakai gambar terbaru, dan executor rclpy satu
   utas: selama inferensi tidak ada callback yang jalan, jadi antrean 5
   benar-benar terisi lalu diolah dari yang **paling lama**. Ini saja menyumbang
   ~0,25 detik.

**Hasil terukur di ALPHONSE:**

| tahap | sebelum | sesudah |
|---|---|---|
| gambar kamera sampai ke pelanggan | 996 ms | **170 ms** |
| deteksi bola | 1241 ms | **297 ms** |
| laju deteksi | 2,8 Hz | 8,8–12,6 Hz |
| CPU usb_cam | 73 % | **13 %** |

**Jebakan.** `camera_info.yaml` masih menyatakan 1280x720 — itu berkas
kalibrasi dan **tidak** ikut berubah. Aman untuk jalur YOLO; yang memakai
intrinsik cuma detektor Hough C++ lama.

**Kaitan ke paket F:** kalau CHRONUS tetap di 1280 px, argumen
`kick_image_width_px` **wajib** diisi 1280. Radius bola yang diterbitkan detektor
dalam piksel gambar ASLI, jadi ambang radius ikut berlipat kalau lebarnya beda.

---

### Paket F — Logika tendang lengkap

**Commit ALPHONSE:** `75b4e1f`, `d345bb6`, `7d50053`, `e43eecb`, `34f8573`,
`44d6139`, `146956f`, `a2d6c09`, `4fd436d`, `9e30300`, `d4b45c7`, `bfdd3f2`,
`eb0a45a`

Ini paket terbesar. Di ALPHONSE ada **delapan penghalang berlapis**, masing-masing
menyembunyikan yang berikutnya. Diurutkan supaya CHRONUS bisa memeriksa satu per
satu, bukan menebak.

#### F.1 — Rumus jarak memakai arah KEPALA, bukan posisi BOLA

Rumus asli:

```cpp
distance_to_ball = CAMERA_HEIGHT * tan(M_PI*0.5 + current_tilt_ - hip_pitch_offset_ - ball_size);
```

`current_tilt_` itu sudut sendi kepala. Rumus ini mengukur "seberapa jauh titik
yang **dipandang** kepala", bukan "seberapa jauh **bola**". Selama bola terkunci
di tengah frame keduanya sama — justru saat bola mendekat kaki keduanya berbeda
jauh (bola turun ke tepi bawah gambar sementara kepala masih menyusul, atau
sudah mentok di batas sendi). Setengah-FOV tegak 21,6 deg, jadi selisihnya bisa
lebih dari 20 derajat.

**Perbaikan:**

```cpp
const double tilt_for_geometry = head_tilt_sign_ * current_tilt_ + y_angle;
```

`y_angle` = `BallTracker::getTiltOfBall()`, sudah dalam konvensi rumus.

#### F.2 — Tanda `head_tilt` terbalik

Lihat bagian 7. **Ukur, jangan tebak.** `head_tilt_sign_` = `-1.0` untuk robot
berkonvensi terbalik, `+1.0` untuk konvensi asli.

#### F.3 — Batas sendi kepala memotong diam-diam

Ini paket D. Tanpa itu ambang tendang tidak pernah bisa dicapai.

#### F.4 — `kick_distance` itu juga JARAK BERHENTI

Ini jebakan paling berbahaya di seluruh paket, karena arahnya berlawanan dengan
intuisi.

`calcFootstep()` diberi `(jarak_bola − kick_distance)`, jadi robot **berhenti
mendekat** begitu jarak bola sama dengan angka itu. Dipasang 0,54 m, robot
berhenti satu setengah langkah dari bola — bola tak pernah sampai di depan kaki,
jendela servo tak pernah tercapai, tendangan tak pernah terjadi.

> **Menaikkan `kick_distance` supaya "lebih mudah menendang" justru membuatnya
> mustahil.**

Juga: angka yang diukur operator dengan meteran itu jarak **lurus**
kamera-ke-bola, sedangkan angka di kode jarak **tanah**. Konversinya:

```
jarak_tanah = sqrt(jarak_lurus² − (tinggi_kamera − jari_jari_bola)²)
```

#### F.5 — Jendela servo `head_pan` terlalu sempit

Pemicu tambahan atas permintaan operator: "kalau kepala sudah memutar ke kanan
sampai servo sekitar 157 derajat dan bola di kanan, tendang kaki kanan".

```
servo_deg = 180 + head_pan(deg)
```

(XM430/MX-28: 0 rad = nilai 2048 = 180 derajat, 4096 langkah = 360 derajat.
Berlaku **hanya kalau** offset `head_pan` di `offset.yaml` = 0 — periksa dulu.)

Jendela selebar 2 derajat **tidak pernah kena**, karena kepala bergerak dalam
lompatan, bukan sapuan mulus. Di ALPHONSE akhirnya jadi:

```
kanan 152..178 deg     kiri 193..208 deg
```

Angka itu **hasil ukur di ALPHONSE**, 21 sampel dari 2 run. CHRONUS harus
mengukur sendiri: jalankan demo, baca `servo head_pan NNN deg` di baris `TENDANG`
pada log, kumpulkan sepuluhan sampel, baru pasang jendelanya.

Ada **zona mati** antara kedua jendela (178..193 di ALPHONSE). Itu disengaja: di
situ bola ada di depan tengah badan, bukan di depan salah satu kaki.

#### F.6 — Pemicu paling andal: JARI-JARI BOLA dalam piksel

Ini yang akhirnya paling banyak dipakai, dan yang paling sedikit asumsinya.
Tidak bergantung tanda `head_tilt`, `hip_pitch`, ataupun apakah kepala menunjuk
ke bola. Bola dekat selalu besar di gambar.

Ambangnya **dihitung sendiri**, tidak diisi tangan:

```
fokus  = (kick_image_width_px / 2) / tan(kick_fov_width_deg)
miring = akar(kick_distance² + (kick_camera_height − kick_ball_real_radius_m)²)
ambang = fokus × kick_ball_real_radius_m / miring
```

Kenapa harus dihitung, bukan angka tetap: besar bola di gambar ditentukan jarak
**lurus** kamera-ke-bola, dan itu ikut berubah saat kamera naik. Terukur di
ALPHONSE: bola di lantai tepat di posisi kaki = **64,7 px** saat digantung
29,5 cm, tapi **101,3 px** saat berdiri. Angka tetap pasti salah di salah satu
keadaan.

Dengan rumus ini, cukup ubah `kick_camera_height` saat robot pindah posisi dan
ambangnya ikut benar sendiri.

> **Jebakan yang pernah kena:** argumen ketiga `processFollowing()` bernama
> `ball_size` dan dipakai **di dalam `tan()`** — satuannya RADIAN. Mengoper
> radius piksel ke situ berarti 45 radian dan merusak total perhitungan jarak.
> Radius piksel harus lewat argumen keempat yang terpisah.

#### F.7 — Publisher sekali-pakai membuang pesan pertama

`playMotion()` membuat publisher baru tiap dipanggil lalu langsung `publish()`.
DDS butuh waktu untuk saling menemukan (discovery **asinkron**), jadi pesan
pertama hilang. Akibatnya halaman aksi tendangan **tidak pernah jalan**, padahal
lognya bilang sudah dikirim.

**Aturan umum:** di ROS 2, **jangan pernah** `create_publisher` lalu langsung
`publish` di tempat. Selalu publisher anggota kelas.

#### F.8 — Argumen launch tanpa titik desimal membunuh node

Mengetik `kick_pan_left_min_deg:=193` (tanpa `.0`) membuat ROS menyimpulkan tipe
**integer**. Tidak cocok dengan `declare_parameter` bertipe double, exception
tidak tertangkap, node **mati seketika** (exit −6).

Dua perbaikan, pakai dua-duanya:
1. `type="float"` pada tiap `<param>` di launch XML.
   **Pengenal tipenya `float`, BUKAN `double`** — launch XML ROS 2 menolak
   `double` dengan `ValueError: Got invalid type identifier`.
2. `try/catch` di pembaca parameter: kalau tipenya salah, coba lagi sebagai
   `int64_t`, dan kalau tetap gagal pakai nilai bawaan sambil mencetak peringatan.

> **Jebakan XML:** komentar XML **tidak boleh mengandung `--`**. Satu tanda hubung
> ganda di dalam komentar membuat seluruh berkas launch gagal diurai
> (`ParseError: not well-formed`). Selalu periksa setelah menyunting:
>
> ```bash
> python3 -c "import xml.etree.ElementTree as ET; ET.parse('demo.launch.xml'); print('ok')"
> ```

#### Daftar argumen launch yang harus ada

```
kick_camera_height        tinggi kamera dari LANTAI (m) — WAJIB diukur ulang
kick_distance             ambang tendang = jarak berhenti, jarak TANAH (m)
kick_pan_max_distance     pagar jarak untuk jalur jendela pan (m)
kick_ball_radius_px       0 = hitung sendiri (pakai ini); negatif = matikan jalur
kick_pan_right_min_deg    jendela servo kaki kanan
kick_pan_right_max_deg
kick_pan_left_min_deg     jendela servo kaki kiri
kick_pan_left_max_deg
kick_ball_real_radius_m   0.11 untuk bola ukuran 5
kick_fov_width_deg        SETENGAH bukaan mendatar; samakan dengan fov_width_deg
kick_image_width_px       samakan dengan image_width di camera_param.yaml
```

#### Cara verifikasi di robot

Saat start, log harus mencetak empat baris ini:

```
Kick radius bola dihitung sendiri: NN.N px (dari kick_distance ... tinggi kamera ...)
Kick geometry: camera_height=... kick_distance=... head_tilt_sign=...
Kick pan window: kanan A..B deg, kiri C..D deg ... maks N.NN m, butuh N siklus
Kick radius bola: >= NN.N px (aktif)
```

Kalau baris pertama tidak muncul, berarti `kick_ball_radius_px` masih terisi
manual di suatu tempat.

Saat demo jalan, tiap detik keluar baris keadaan:

```
jarak bola N.NNN m (ambang N.NNN) | servo head_pan NNN.N deg (kanan A-B, kiri C-D)
  | head_tilt +NN.N deg | bola dlm gambar +NN.N deg | radius NN.N px (ambang NN)
  | bola x +NN.N deg | siap: <alasan>
```

Kolom `siap:` yang paling berguna — ia menyebutkan sendiri kenapa belum
menendang, misal `jendela pan cocok tapi bola masih jauh`.

Saat menendang:

```
TENDANG kaki KANAN -- pemicu: radius bola | servo head_pan 161.8 deg | jarak 0.930 m | radius 61.7 px
```

> **Cara membaca label `pemicu` — mudah disalahpahami.** Labelnya
> **berprioritas**, bukan saling meniadakan:
> `radius_ready ? "radius bola" : (pan_ready ? "jendela pan" : "jarak")`.
> Jadi `pemicu: radius bola` **tidak** berarti jendela pan gagal — kalau
> keduanya terpenuhi, radius yang menang label. Label `jendela pan` hanya muncul
> saat radius belum cukup **atau** bola melenceng lebih dari 25 derajat di gambar.

**Hasil di ALPHONSE:** 11 tendangan dalam satu run 15 menit, dua kaki, dua jalur
pemicu, 0 error.

---

### Paket G — Perampingan bulk read servo

**Commit ALPHONSE:** `2b76d84`

**Bagian ini paling perlu penilaian sendiri.** Ada dua hal di dalamnya, dan
**hanya satu** yang cocok untuk CHRONUS.

#### G.1 — Perampingan `BULK READ ITEMS` — AMAN, salin saja

`OP3.robot` ALPHONSE hanya membaca `present_position` per servo.

Alasannya: `GroupBulkRead` memakai alamat item **pertama** sebagai alamat awal
(132 = `present_position`). Gain p/i/d ada di alamat 80/82/84 — **di bawah** 132
— jadi `isAvailable()` untuk gain **selalu false** dan nilainya **tidak pernah
terbaca**. Yang ia lakukan cuma memanjangkan balasan tiap servo dari 4 byte jadi
10.

Bedanya nyata di kabel: `20 × (11 + 10) = 420` byte jadi `20 × (11 + 4) = 300`
byte. Round-trip terukur **3,79 ms → 2,81 ms**.

Gain tetap dibaca sekali saat init lewat pembacaan per-item biasa, jadi tidak
ada yang hilang. Ini juga **tidak** mengubah peta indirect address, jadi **tidak
perlu torque off**.

#### G.2 — Transaksi port sensor di awal siklus — TIDAK PERLU untuk CHRONUS

Ini obat untuk masalah yang **khusus ALPHONSE**: OpenCR-nya rusak di bus TTL,
jadi dipindah ke micro-USB. Perangkat full-speed 12 Mbit itu menempel di hub USB
**yang sama** dengan U2D2, dan host wajib memakai *split transaction* untuk
perangkat 12 Mbit di balik hub high-speed. Slot itu diambil dari jatah yang juga
dipakai U2D2 — kalau transaksi OpenCR jatuh bersamaan dengan 420 byte balasan 20
servo, sebagian byte itu **HILANG**, bukan telat.

Terukur (persentase bulk read servo yang berhasil):

| OpenCR disentuh tiap | tidak disentuh | 8 siklus | 4 | 2 | 1 | 1 tapi di AWAL siklus |
|---|---|---|---|---|---|---|
| berhasil | 100 % | 90 % | 78 % | 54 % | 3 % | **100 %** |

**CHRONUS tidak punya masalah ini** — OpenCR-nya di bus servo, jadi himpunan
port-sensor-terpisahnya kosong dan kodenya jadi tidak berbuat apa-apa. Salinnya
tidak merusak, tapi juga tidak menolong. Kalau mau tetap disalin (supaya kedua
robot satu kode), pastikan `sensor_only_ports` benar-benar kosong di CHRONUS.

Yang **tetap berguna** untuk CHRONUS dari commit ini: pemroses `OP3_PROFILE=1`
yang mencetak `[PROF]` tiap 2 detik (rx_ok per port, byte tersedia, lama tiap
fase). Mati total tanpa env itu, jadi aman dibawa.

> **Jebakan timeout — umpan balik positif.** Bulk read itu dipipa: permintaan
> dikirim di akhir `process()`, balasan dibaca di awal `process()` berikutnya.
> Jadi waktu menjawab = `control_cycle − lama process()`. Timeout **besar**
> membuat: Rx gagal → menunggu lama → permintaan berikutnya terlambat → jeda
> menyusut → gagal lagi. Terukur dengan 3 ms: mulai sehat 20 % lalu **runtuh ke
> 0 % dalam 8 detik dan tidak pernah pulih**. Timeout Rx harus **1 ms**.

**Alat bantu:** `scripts/op3_bus_loop.py` menjalankan pola yang persis sama
dengan `RobotisController::process()` tapi **tanpa ROS**. Berguna untuk
memisahkan "bus/servo rusak" dari "penjadwalan USB". Belum ada di CHRONUS;
salin kalau perlu mendiagnosis.

---

### Paket H — `y_offset` jadi knob lebar kaki

**Commit ALPHONSE:** `29ea099`

**Gejala.** Parameter `y_offset` ada di `param.yaml` dan di Bascorro Studio,
tapi mengubahnya tidak berpengaruh apa-apa.

**Sebab.** `computeNeutralLegAngle()` dipakai dua pihak dengan kebutuhan
berlawanan, dan keduanya memakai pose yang sama persis. Penurunan
`walking_bias_` menghitung `bias = pose_page2 − IK(neutral)`, lalu gait memakai
`bias + IK(gait)`. Apa pun yang masuk ke neutral **dikurangkan lagi** dari gait,
jadi selama `y_offset` ada di kedua sisi ia meniadakan dirinya sendiri.

**Perbaikan.** Bedakan kedua pemakai lewat argumen `with_y_offset`: penurunan
bias memakai `false` (supaya `y_offset` tidak batal sendiri), pelacakan idle
memakai `true` (supaya lebar kaki saat berdiri sama dengan saat berjalan).

**Jebakan.** Kalau hanya dikeluarkan dari neutral tanpa membedakan pemakai,
lahir gejala kedua: lebar kaki hanya berlaku saat gait jalan, jadi stance
melebar tepat saat start dan merapat lagi ke tengah saat stop.

> **Peringatan umum tentang parameter walking:** angka yang sama disimpan di
> **TIGA tempat** yang gampang menyimpang — `param.yaml`, `initialize()` di
> `.cpp`, dan nilai bawaan di `App.jsx`. Kalau mengubah satu, ubah ketiganya.
> Satuan juga beda: yaml pakai derajat/milidetik, UI Bascorro pakai
> radian/detik.

---

## 5. Urutan pengerjaan yang disarankan

Kerjakan berurutan. Jangan lompat — tiap fase membersihkan sebab yang bisa
menyamar jadi kegagalan fase berikutnya.

**Fase 1 — bikin lingkungannya jujur dulu (setengah hari)**
1. Paket A (domain ROS 2). Tanpa ini, semua pengukuran berikutnya bisa tercemar
   robot sebelah.
2. Paket B (crash handler). Tanpa ini, node yang mati menyamar jadi bug logika.
3. Jalankan `scripts/op3_docker.sh doctor` sampai bersih.

**Fase 2 — masukan yang benar (setengah hari)**
4. Paket C (tombol). Supaya start/stop bisa dipercaya saat menguji.
5. Paket E (kamera). Supaya deteksi tidak basi.
6. Ukur konvensi `head_tilt` (bagian 7). **Sebelum** menyentuh paket D dan F.

**Fase 3 — gerakan (satu hari)**
7. Paket D (batas kepala) — **robot di dudukan**, naikkan bertahap.
8. Paket H (walking `y_offset`) kalau memang dibutuhkan.

**Fase 4 — tendangan (satu hari, paling banyak coba-coba)**
9. Paket F, urut F.1 sampai F.8.
10. Ukur jendela `head_pan` CHRONUS sendiri (kumpulkan ~10 sampel per kaki).
11. Setel `kick_camera_height` sesuai posisi robot saat itu.

**Fase 5 — kalau masih ada gejala bus**
12. Paket G.1 (perampingan bulk read). G.2 hanya kalau OpenCR CHRONUS ternyata
    juga pindah ke USB.

---

## 6. Jebakan lingkungan & build

Ini yang paling sering memakan waktu, dan tidak satu pun berhubungan dengan
logika robot.

### 6.1 Workspace hanya jalan DI DALAM container

Seluruh `install/` dibangun dengan `--symlink-install` **di dalam** container,
tempat repo di-mount sebagai `/ros2_ws`. Di host semua symlink itu menggantung.

**Jebakannya diam-diam:** `source install/setup.bash` di host **berhasil, exit
0**. colcon cuma melewati paket yang `local_setup.bash`-nya tidak ketemu, dan
hanya mencetak `not found:` yang lewat begitu saja. Dari 73 paket hanya ~4 yang
lolos. Akibatnya `ros2 run` menjawab `Package 'xxx' not found`, seolah paketnya
lenyap.

**Selalu jalankan `scripts/op3_docker.sh doctor` dulu.**

### 6.2 Rebuild: `--packages-above`, BUKAN `--packages-select`

Kalau mengubah **header**, paket yang bergantung padanya harus ikut dibangun
ulang. Kalau tidak, mereka memakai tata-letak kelas yang **lama** → gejalanya
mengerikan dan tanpa petunjuk: proses diam di 100 % CPU, atau mati tepat setelah
satu baris log, **tanpa pesan error sama sekali**.

```bash
colcon build --packages-above <paket> --symlink-install
```

Selalu `source /opt/ros/humble/setup.bash` dulu, kalau tidak colcon menjawab
`ModuleNotFoundError: ament_package`.

### 6.3 "No executable found" = masalah izin, bukan berkas hilang

Skrip Python di `op3_action_editor` tersimpan `100644` di git → `ros2 run`
menolak menjalankannya. `chmod +x` saja, tidak perlu rebuild (install/ symlink
ke sumber).

### 6.4 Nama proses terpotong

Nama proses terpotong 15 karakter, jadi `pgrep -x op3_action_editor` **selalu
kosong**. Pakai `pgrep -a` atau `ps -eo cmd`.

### 6.5 JANGAN `pkill -f` berantai

`pkill -f` yang polanya kena shell-nya sendiri akan **membunuh shell itu**
(exit 137). Sudah terjadi dua kali dalam satu sesi. Cara aman: daftar PID
dengan `ps`, lalu bunuh per-PID, lalu verifikasi dengan `ps` lagi.

### 6.6 Jangan serial mentah saat manager hidup

Alat seperti `op3_ping.py` atau `op3_bus_loop.py` **berebut bus** dengan
`op3_manager`. Bulk read jadi rusak dan `present_joint_states` berubah jadi
−3.14159 semua. Matikan manager dulu, atau pakai topik ROS saja.

### 6.7 Ultralytics AutoUpdate merusak pin NumPy

Memuat berkas `.onnx` di container memicu Ultralytics memasang `onnx` + NumPy 2,
dan itu **mematikan `cv_bridge`**. Cegah dengan:

```bash
export YOLO_AUTOINSTALL=false
```

### 6.8 Dua demo jalan bersamaan

Dua `head_tracking_node` akan berebut kepala dan gejalanya terlihat seperti
kepala rusak. Cek dengan `ps`, **bukan** `ros2 node list` (daemon menyimpan
node hantu).

---

## 7. Yang HARUS diukur, tidak boleh ditebak

### 7.1 Tanda `head_tilt` — mana yang menunduk

Di ALPHONSE ini **sudah dua kali disimpulkan keliru** dan dua kali memakan waktu
robot. Salah tanda = tendangan tidak akan pernah terpicu.

**Satu-satunya cara yang sah** — dalam **SATU run**, berselang beberapa detik:

```bash
# 1. hentikan pelacak supaya kepala tidak melawan
ros2 topic pub --times 3 /ball_tracker/command std_msgs/msg/String "{data: stop}"

# 2. perintahkan sudut MUTLAK, tunggu ~3 detik, ambil satu frame
ros2 topic pub --times 3 /robotis/head_control/set_joint_states \
  sensor_msgs/msg/JointState "{name: [head_pan, head_tilt], position: [0.0, -0.45]}"
#    ambil gambar dari /usb_cam_node/image_raw/compressed

# 3. ulangi dengan position: [0.0, 0.45]
#    ambil gambar lagi

# 4. LIHAT kedua gambarnya
```

Hasil di ALPHONSE: `−25,8 deg` → langit-langit; `+25,8 deg` → rumput dan bola.
Jadi **positif = menunduk**, dan `head_tilt_sign` = `−1.0`.

**Yang TIDAK boleh dipakai sebagai bukti:**
- komentar di `head_tracking.yaml` — pernah salah;
- kesan arah gerak kepala saat `scan_tilt_*` diubah — pernah salah;
- membandingkan foto dari **dua run berbeda** — inilah yang dulu membuat tanda
  dibalik secara keliru. Di antara dua run bolanya dipindah, jadi "dua-duanya
  kelihatan lantai" tidak membuktikan apa pun.

### 7.2 Tinggi kamera

`kick_camera_height` bukan konstanta robot. Ia ikut posisi robot:

```
berdiri di lantai     : ~0.56 m
digantung 29,5 cm     : ~0.855 m
```

Selisihnya besar: bola di jarak tanah 0,40 m butuh `head_tilt` +46,5 deg kalau
berdiri, tapi +56,9 deg kalau digantung.

### 7.3 Jendela servo `head_pan`

Jangan salin angka ALPHONSE. Jalankan demo, kumpulkan sepuluhan baris `TENDANG`,
baca kolom `servo head_pan NNN deg`, baru tentukan jendelanya. Beri margin
beberapa derajat karena kepala bergerak dalam lompatan.

Periksa juga `offset.yaml` — rumus `servo_deg = 180 + head_pan_deg` hanya sahih
kalau offset `head_pan` = 0.

### 7.4 Nomor seri U2D2

Nomor seri di jalur `by-id` **berpindah-pindah** antar robot dan antar colokan.
Selalu `ls /dev/serial/by-id` di host dulu. Kalau CHRONUS pakai `/dev/ttyOP3`,
biarkan begitu — itu lebih tahan banting.

---

## 8. Masalah yang mungkin muncul saat menerapkan

Tabel ini disusun dari gejala, bukan dari sebab — karena gejalanya yang lebih
dulu terlihat.

| Gejala | Kemungkinan sebab | Cek pertama |
|---|---|---|
| Node demo mati tanpa pesan, exit −11 | klok sementara di THROTTLE (paket B) | `cat /tmp/op_demo_crash.log` |
| Node demo mati, exit −6, tepat saat start | argumen launch integer vs double (F.8) | log baris `InvalidParameterType` |
| Launch gagal diurai | ada `--` di dalam komentar XML | jalankan `ET.parse()` |
| `Got invalid type identifier` | ditulis `type="double"` | ganti jadi `type="float"` |
| Robot jalan terus tak bisa distop | node demo sudah mati (paket B) | `pgrep -a op_demo_node` |
| Tombol start jalan, stop tidak | debounce belum ada (paket C) | cari baris `TOMBOL` di log |
| Tidak pernah menendang | `kick_distance` terlalu besar (F.4) | bandingkan `jarak bola` vs ambang di log |
| Menendang tapi halaman aksi salah | publisher sekali-pakai (F.7) | log `(penerima: 0)` |
| Menendang cuma kalau bola diangkat | ambang radius terlalu tinggi (F.6) | baca baris `dihitung sendiri` |
| Kepala mendeteksi tapi tak mengunci | deteksi basi (paket E) | ukur umur `header.stamp` |
| Kepala/kaki kehilangan parameter | robot lain di LAN (paket A) | `echo` topik saat semua node lokal mati |
| Proses diam 100 % CPU, tanpa error | ABI basi (6.2) | rebuild `--packages-above` |
| `Package not found` | shell di host, bukan container (6.1) | `op3_docker.sh doctor` |
| `first bulk read fail!!` lalu crash | satu servo diam / konektor longgar | `scripts/op3_ping.py` |
| Semua sendi jadi −3.14159 | serial mentah bentrok manager (6.6) | matikan manager dulu |
| Kartu dashboard Studio semua "-" | agen studio mati, bukan ROS | `/tmp/bascorro_studio_logs/` |

### Kalau `op3_manager` mati

Baca `~/.ros/log`. Di ALPHONSE, dari 43 run: **20 mati SAAT START** (biasanya
`first bulk read fail!!`), cuma **2 yang mati di tengah**. Exit −6 yang disertai
`signal_handler` biasanya crash **saat shutdown**, bukan saat demo — periksa
apakah jejaknya ada di jalur keluar proses (`__libc_start_main` → `on_exit` →
destruktor) sebelum menuduh logika demo.

### Satu servo diam bisa menjatuhkan semuanya

`GroupBulkRead::rxPacket()` berhenti di servo pertama yang gagal, jadi **satu**
servo yang diam menjatuhkan **semua 20**. Bedakan:
- diam di **semua** baud = sambungan atau daya;
- kepanasan = masih menjawab ping, cuma torsinya mati.

Cek pertama selalu `scripts/op3_ping.py`, bukan menebak-nebak konfigurasi.

---

## 9. Yang TIDAK ikut dan yang belum selesai

Supaya tidak dicari-cari:

**Tidak ikut ke repo (masih lokal di ALPHONSE, belum di-commit):**
- Magnetometer / yaw di `open_cr_module` + `OPEN-CR.device`
- Perbaikan bocor `std::thread` di `head_control_module`
- `boot_init_page` di include manager pada `demo.launch.xml`
- Beberapa setelan walking (`MAX_FB_STEP`, `hip_pitch_offset`, `setWalkingParam`
  tanpa argumen balance)

**Belum tuntas di ALPHONSE, jadi jangan berharap sempurna di CHRONUS:**
- Bulk read servo ALPHONSE rata-rata **87 %**, bukan 100 %. Sisanya rebutan USB.
  Langkah berikutnya bersifat **fisik**: pindahkan U2D2 ke port yang mendarat di
  root hub lain. (Tidak relevan untuk CHRONUS selama OpenCR-nya di bus servo.)
- `op_demo_node` masih **segfault saat shutdown** (bukan saat demo). Jejaknya
  seluruhnya di jalur keluar proses. Tidak berbahaya, tapi menambah catatan di
  `/tmp/op_demo_crash.log`.
- Jangkauan `head_tilt` +75 derajat belum diuji sampai batas mekanisnya.

**Jangan diulang — sudah dicoret dengan bukti di ALPHONSE:**
- Servo kepala ID19/20 sehat (ikut perintah 125 Hz, simpangan maks 4 hitungan).
- Bus elektris bersih (0 error CRC dari 400 transaksi).
- `latency_timer` FTDI sudah ditangani otomatis oleh `op3_docker.sh`.
- Teori "sync write menabrak bulk read half-duplex" — **DIBANTAH**.
- Teori "YOLO meruntuhkan bus servo" — **DIBANTAH** (SIGSTOP tidak berpengaruh).
- Memindah colokan USB — dicoba, tidak menolong.

---

## 10. Catatan cara kerja

Untuk Claude yang mengerjakan ini:

1. **Kerjakan bertahap dan uji di robot tiap tahap.** Delapan penghalang di
   paket F ditemukan satu per satu justru karena tiap perbaikan diuji sendiri.
   Kalau lima diterapkan sekaligus lalu gagal, tidak ada cara tahu yang mana.

2. **Tulis alasannya di komentar kode, bukan cuma angkanya.** Hampir semua
   waktu yang hilang di ALPHONSE terjadi karena ada angka tanpa penjelasan, lalu
   orang berikutnya (termasuk saya sendiri) menebak maksudnya dan menebak salah.

3. **Kalau sebuah angka hasil ukur, tulis kapan dan bagaimana diukurnya.**
   Kalau tebakan, tulis "belum ditera".

4. **Commit dengan identitas yang benar.** Konfigurasi git di mesin ini masih
   placeholder, jadi identitasnya harus dipaksa tiap commit:

   ```bash
   git -c user.name=ProgramBascorro -c user.email=motionbascorro@gmail.com \
       commit -m "..."
   ```

   Stage berkas **satu per satu dengan jalur eksplisit**, jangan `git add -A` —
   working tree biasanya berisi kerjaan operator yang belum siap masuk.

5. **Jangan menambah publisher atau logika init di `op3_manager.cpp`.** Di
   perangkat keras sungguhan itu memicu stack smashing.

6. **Kalau perilaku robot bertentangan dengan yang tertulis di kode**, curigai
   binary basi lebih dulu. Rebuild dengan `--packages-above`, lalu baca ulang.
