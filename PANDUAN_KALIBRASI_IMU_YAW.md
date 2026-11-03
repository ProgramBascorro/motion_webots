# Panduan Kalibrasi Yaw IMU (Magnetometer OpenCR) — ROBOTIS OP3 ROS 2

Dibuat dari implementasi di robot ALPHONSE (17–18 Sep 2026), untuk dipasang ulang
di robot lain dengan stack ROS 2 (Humble) + op3_manager + OpenCR yang mirip.

**Isi file ini HANYA kalibrasi yaw IMU.** Tidak ada logika demo di sini.

- Bagian A: latar belakang dan temuan (kenapa kalibrasi perlu, dan bentuknya).
- Bagian B: arsitektur yang harus ada di robot.
- Bagian C: **PROMPT siap tempel** untuk Claude di robot lain.
- Bagian D: cara operator menjalankan kalibrasi.
- Bagian E: pemeriksaan hasil dan pemecahan masalah.

---

## A. Latar belakang

### A1. Masalahnya
Yaw dari OpenCR berasal dari **magnetometer** (kompas), dan firmware memakai
referensi 0° = arah robot saat OpenCR dinyalakan. Di ALPHONSE, badan robot
(servo, rangka besi, kabel baterai) sangat mengganggu medan magnet, sehingga
heading **tidak linier**. Hasil ukur saat robot diputar 90° setiap langkah:

| Sesi | Terbaca per 90° nyata | Keterangan |
|---|---|---|
| 17 Sep (rekaman 4 titik pertama) | +30,7 / −78,8 / +20,4 | **tidak monoton** (berbalik arah) → tidak bisa dikoreksi tabel |
| 17 Sep (5 titik, sesi kedua) | 43,7 / 44,9 / 235,6 / 35,8 | monoton, total 360 → **bisa dikoreksi** |
| 18 Sep (5 titik, alat interaktif) | 47,4 / 50,3 / 204,5 / 57,7 | monoton, total 360 → **bisa dikoreksi** |

Tanpa koreksi, putaran robot 90° bisa terbaca 36° sampai 236°. Target yaw di
demo (mis. ±10°) jadi tidak berarti.

### A2. Bentuk koreksi yang dipakai
- Robot diputar manual ke **KIRI** di 5 titik: **0°, 90°, 180°, 270°, 360°**
  (titik 5 kembali tepat ke titik 1).
- Di tiap titik dicatat **heading kompas ABSOLUT mentah** (`mag_heading`),
  **bukan** yaw relatif. Alasannya, titik 0 yaw firmware berubah setiap OpenCR
  dinyalakan, jadi tabel berbasis yaw relatif akan salah di nyala berikutnya.
- Titik 0 tabel = **rata-rata melingkar titik 1 dan titik 5**.
- Tabel 4 baris: `heading_terukur[0..3] → arah_nyata [0, 90, 180, 270]`.
- Koreksi saat jalan: **interpolasi linier melingkar** antar titik tabel.
  Heading dikoreksi → difilter → dikurangi titik nol → diterbitkan sebagai yaw.
- Hasil di ALPHONSE: kelima titik rekaman terpetakan ke 358,3 / 90,0 / 180,0 /
  270,0 / 1,4°. Sisa ±1,4° adalah batas pengulangan sensor itu sendiri.

### A3. Syarat tabel dianggap sah
1. Minimal 3 titik (dipakai 4), jumlah `measured` = jumlah `true`.
2. Setiap bentang maju (dari titik ke titik berikutnya, mod 360) **> 0**, dan
   **total bentang = 360° (±1°)**. Kalau total 720/1080, robot diputar ke
   KANAN atau urutan titik tertukar.
3. Setiap bentang **≥ 5°**. Kalau lebih kecil, kompas hampir tidak ikut
   berputar.
4. **|titik 5 − titik 1| ≤ 15°** (peringatan > 5°).

Kalau tidak sah, **jangan disimpan**. Kalau sampai tersimpan juga, modul harus
otomatis kembali ke yaw firmware mentah.

### A4. Kapan TIDAK bisa dikalibrasi dengan tabel
Kalau heading **tidak monoton** (bolak-balik saat robot diputar searah), atau
seluruh putaran 360° hanya terbaca rentang kecil (mis. 80°), artinya offset
**hard-iron** magnetometer sudah tidak cocok. Tabel apa pun tidak bisa
memetakannya. Solusinya kalibrasi hard-iron di firmware dulu (lihat E3), baru
ulangi 5 titik.

---

## B. Arsitektur yang harus ada

```
OpenCR firmware (magnetometer)
   register: yaw (48), mag_heading (57, 0..3600 = 0,1°), mag_status (61)
        │  bulk read op3_manager (OP3.robot: item mag_heading WAJIB ada)
        ▼
open_cr_module (di dalam op3_manager)
   ├─ terbitkan /robotis/open_cr/mag_heading   (Float32, deg, MENTAH, hanya mag_status 2/3)
   ├─ kalau yaw_calib_enable: yaw = tabel(mag_heading)  [interp. linier melingkar]
   ├─ titik nol: awal sensor siap, dan setiap pesan di /robotis/open_cr/reset_yaw
   └─ terbitkan /robotis/open_cr/imu  (orientation yaw = relatif titik nol, terkoreksi)
        ▲
        │ parameter dari config/yaw_calibration.yaml (dibaca SEKALI saat start)
        │
scripts/calib_imu_yaw.py (interaktif, SPASI tiap titik) → menulis yaw_calibration.yaml
```

| Komponen | Isi |
|---|---|
| Firmware OpenCR | versi yaw magnetometer: register `mag_heading` (57), `mag_status` (61), `imu_control` (50, bit4 = kalibrasi hard-iron 20 s) |
| `OPEN-CR.device` | item `mag_heading` (addr 57, 2 byte, R) dan `mag_status` (61) ada di tabel kontrol |
| `OP3.robot` | baris sensor open-cr: tambah `mag_heading` di daftar bulk read (…, yaw, **mag_heading**, mag_status) |
| `open_cr_module` | parameter `yaw_calib_*`, fungsi `correctHeadingDeg()`, topik `mag_heading`, topik `reset_yaw` |
| `op3_manager/config/yaw_calibration.yaml` | tabel kalibrasi (satu-satunya sumber) |
| Semua cara start op3_manager | wajib memuat `yaw_calibration.yaml` |
| `scripts/calib_imu_yaw.py` | alat interaktif perekam 5 titik + penulis yaml + cadangan |
| Menu / pintasan | `./script.sh` → "Calibration" → `op3_docker.sh imu-calib` → `docker exec -it … /docker-entrypoint.sh python3 …` |

### B1. Parameter
| Parameter (`open_cr_module`) | Contoh ALPHONSE 18 Sep | Arti |
|---|---|---|
| `yaw_calib_enable` | `true` | `false` = yaw firmware mentah seperti dulu |
| `yaw_calib_measured_heading_deg` | `[211.44, 258.88, 309.22, 153.70]` | heading mentah di 0/90/180/270° nyata |
| `yaw_calib_true_deg` | `[0.0, 90.0, 180.0, 270.0]` | arah nyata tiap titik (naik) |

**Angka di atas khusus ALPHONSE, jangan disalin ke robot lain.** Setiap robot
wajib merekam tabelnya sendiri.

### B2. Konstanta alat kalibrasi
| Konstanta | Nilai | Arti |
|---|---|---|
| `SAMPLE_SEC` | 3,0 s | lama membaca tiap titik |
| `MIN_SAMPLES` | 20 | sampel minimum per titik |
| `STILL_GYRO_RAD_S` | 0,05 rad/s | \|gyro z\| maksimum agar dianggap diam |
| `STILL_DRIFT_DEG` | 2,0° | pergeseran rata-rata paruh 1 vs paruh 2 jendela |
| `NOISY_JITTER_DEG` | 8,0° | ambang peringatan getaran kompas |
| `MIN_SPAN_DEG` | 5,0° | bentang minimum antar titik |
| `CLOSURE_WARN_DEG` / `CLOSURE_REJECT_DEG` | 5° / 15° | selisih titik 5 vs 1 |
| `--init-timeout` | 90 s | batas tunggu pose INIT kalau alat menyalakan stack sendiri |

---

## C. PROMPT (tempel ke Claude di robot lain)

````
Pasang KALIBRASI YAW IMU (magnetometer OpenCR) 5 titik di stack ROBOTIS OP3 ROS 2 robot ini. Hanya kalibrasi
IMU, jangan ubah logika demo/walking/action apa pun.

LANGKAH 0 - PELAJARI DULU, JANGAN MENEBAK
- Baca open_cr_module (process(), publishIMU(), handleButton), OP3.robot (baris sensor open-cr + daftar bulk
  read), robotis_device/devices/sensor/OPEN-CR.device, op3_manager.launch.py, dan SEMUA cara op3_manager
  dinyalakan di robot ini (launch file, profil tmux, skrip start manager, Studio). Cari juga
  docker-entrypoint/env: ROS_DOMAIN_ID dan ROS_LOCALHOST_ONLY stack yang sebenarnya (baca /proc/<pid>/environ
  dari proses op3_manager, jangan mengandalkan `bash -lc`).
- Pastikan firmware OpenCR punya yaw magnetometer: register mag_heading (addr 57, uint16, 0,1 deg, 0..3600),
  mag_status (61: 0 tidak ada magnetometer, 1 settling, 2 OK, 3 kalibrasi hard-iron), imu_control (50, bit4 =
  kalibrasi hard-iron 20 s, bit5 = arah sekarang = 0). Kalau firmware tidak punya register ini, BERHENTI dan
  laporkan: kalibrasi ini butuh firmware yaw magnetometer.
- JANGAN membuka port serial OpenCR/U2D2 secara langsung selama op3_manager hidup (bulk read rusak). Kalau
  harus membaca register mentah, pastikan dulu tidak ada proses op3_manager (pgrep). Selama manager hidup,
  baca lewat topik ROS saja.

1. DEVICE + BULK READ
- OPEN-CR.device harus punya item mag_heading (57, 2 byte, R, unsigned) dan mag_status (61). Tambahkan kalau
  belum ada.
- OP3.robot baris sensor open-cr: tambahkan `mag_heading` ke daftar bulk read, urut alamat (..., yaw,
  mag_heading, mag_status). Periksa bahwa sensor tanpa indirect address dibaca sebagai SATU blok kontinu dari
  item pertama sampai item terakhir. Kalau blok sudah mencakup alamat 57, item ini tidak menambah byte.

2. open_cr_module
a) Parameter (declare di konstruktor, try/catch; gagal = kalibrasi MATI):
   yaw_calib_enable (bool, bawaan false), yaw_calib_measured_heading_deg (double[]),
   yaw_calib_true_deg (double[]).
   Validasi saat start: ukuran >= 3 dan sama; tiap bentang maju (mod 360) > 0 untuk true dan measured;
   total bentang measured = 360 +-1. Tidak sah -> log ERROR + kalibrasi MATI (pakai yaw firmware mentah).
   Sah -> log WARN "KALIBRASI YAW AKTIF (N titik, mag_heading -> arah nyata): <measured->true, ...>".
b) correctHeadingDeg(h): untuk tiap i (j = i+1 mod N), meas_span = fmod(m[j]-m[i]+720,360),
   off = fmod(h-m[i]+720,360). Kalau off <= meas_span -> kembalikan
   fmod(t[i] + fmod(t[j]-t[i]+720,360) * off / meas_span + 720, 360). Tidak ada yang cocok -> h apa adanya.
c) Di process(), HANYA saat mag_status 2 atau 3:
   - yaw_rad = yaw firmware; kalau kalibrasi aktif, yaw_rad = correctHeadingDeg(mag_heading/10) dalam rad.
     Pada sampel sah PERTAMA, isi state filter dan yaw_zero_offset_ dengan nilai ini, supaya yaw awal = 0 dan
     filter tidak merayap dari 0.
   - result_["yaw"] = lowPassFilterAngle(0.4, yaw_rad, previous) (filter sudut melingkar yang sudah ada).
   - Kalau ada permintaan reset (flag atomic dari callback): yaw_zero_offset_ = result_["yaw"], log WARN
     "YAW DIRESET".
   mag_status bukan 2/3: yaw = 0 dan state filter di-nol-kan (perilaku lama).
d) publishIMU(): kalau mag_status 2/3, yaw terbit = atan2(sin(yaw - offset), cos(yaw - offset)).
e) Topik baru (publisher/subscription ANGGOTA kelas, dibuat bersama publisher lain di thread antrean modul):
   - /robotis/open_cr/mag_heading (std_msgs/Float32): mag_heading/10 MENTAH, tanpa tabel, hanya mag_status 2/3.
     Dipakai alat kalibrasi.
   - /robotis/open_cr/reset_yaw (std_msgs/String, isi bebas = alasan): set flag atomic; offset diambil di loop
     kontrol (satu thread dengan result_), jangan di callback.
f) Header: std::atomic<bool> yaw_zero_request_, double yaw_zero_offset_, bool yaw_zero_initialized_, vektor
   tabel, fungsi correctHeadingDeg. #include <atomic>, <vector>. Perubahan header = ABI berubah -> build
   `colcon build --packages-above open_cr_module` (JANGAN --packages-select saja).

3. SATU BERKAS TABEL, DIMUAT SEMUA JALUR START
- Buat op3_manager/config/yaw_calibration.yaml berkunci `/**:` -> `ros__parameters:` (supaya sampai ke node
  open_cr_module di dalam proses op3_manager), isi awal yaw_calib_enable: false (belum dikalibrasi).
- Pastikan CMake op3_manager meng-install folder config.
- Muat berkas ini di SEMUA cara op3_manager dinyalakan:
  - op3_manager.launch.py: tambahkan os.path.join(get_package_share_directory('op3_manager'), 'config',
    'yaw_calibration.yaml') ke list parameters Node (sesudah dict parameter yang ada; import os);
  - setiap perintah `ros2 run op3_manager op3_manager --ros-args ...` (profil tmux, skrip start manager):
    tambahkan `--params-file <workspace>/src/.../op3_manager/config/yaw_calibration.yaml` (path src, jadi tidak
    perlu build ulang sesudah kalibrasi).
  Cari jalur start lain dengan grep "op3_manager op3_manager" dan "op3_manager.launch". Satu jalur yang
  terlewat = kalibrasi diam-diam tidak terpakai.

4. ALAT INTERAKTIF scripts/calib_imu_yaw.py (python3, rclpy, jalan di DALAM container / env ROS stack)
- Kalau op3_manager belum hidup: luncurkan launch yang SAMA dengan perintah demo robot ini (mis. `ros2 launch
  op3_demo demo.launch.xml`) sebagai subprocess (start_new_session, log ke /tmp/imu_calib_stack.log), tunggu
  "Succeed to start page 2" (pose INIT; batas 90 s; gagal kalau "process has died" untuk op3_manager), dan
  matikan lagi di akhir (SIGINT 12 s -> SIGTERM 5 s -> SIGKILL, ke process group). Kalau manager sudah hidup:
  pakai apa adanya, JANGAN dimatikan, dan beri tahu bahwa tabel baru berlaku sesudah manager start ulang.
  Opsi --no-launch, --output, --sample-sec (3,0), --init-timeout (90).
- Wajib terminal (isatty); tombol tanpa Enter (termios cbreak, flush input, kembalikan di finally).
- Langganan: /robotis/open_cr/mag_heading (Float32), /robotis/open_cr/mag_status (UInt8), /robotis/open_cr/imu
  (Imu, pakai |angular_velocity.z|). Tunggu heading mengalir maks 30 s; gagal -> jelaskan (manager lama tanpa
  topik ini, atau mag_status belum 2).
- 5 titik, putar ke KIRI (berlawanan jarum jam dilihat dari atas):
  1) 0 deg: hadapkan ke arah acuan (mis. gawang lawan), tandai kaki di lantai;
  2) 90 kiri; 3) 180 (membelakangi acuan); 4) 270 (= 90 kanan dari acuan);
  5) 360 = kembali TEPAT ke tanda titik 1.
  Tiap titik tampilkan heading mentah live + mag_status; SPASI = rekam, q/Esc = batal (tidak ada yang disimpan).
- Rekam SAMPLE_SEC detik. "Diam" dinilai dari GYRO: max |gyro z| <= 0,05 rad/s DAN pergeseran rata-rata
  melingkar paruh 1 vs paruh 2 <= 2,0 deg. JANGAN pakai sebaran heading sebagai syarat diam: di beberapa arah
  kompas bergetar sendiri +-4..7 deg walau robot diam, dan syarat sebaran akan menolak titik itu selamanya.
  Sampel < 20 -> ulang. Nilai titik = rata-rata melingkar; tampilkan getaran RMS (peringatan > 8 deg: jauhkan
  logam/kabel/laptop) dan perubahan dari titik sebelumnya.
- Bangun tabel: p0 = rata-rata melingkar titik 1 & 5; tabel = [p0, t2, t3, t4], true = [0, 90, 180, 270].
  Tolak (TIDAK disimpan, tabel lama tidak disentuh) kalau: total bentang != 360 +-1 (diputar ke kanan / urutan
  salah), ada bentang < 5 deg, atau |titik5 - titik1| > 15 deg. Peringatan kalau > 5 deg.
- Simpan: cadangkan berkas lama ke config/yaw_calibration_history/yaw_calibration_YYYYmmdd_HHMMSS.yaml, tulis
  atomik (tmp + fsync + os.replace), kembalikan pemilik berkas/folder ke uid:gid folder config (alat jalan
  sebagai root di container, repo milik user host). Isi yaml: komentar berisi 5 heading mentah, selisih titik
  5 vs 1, bentang per 90 deg, tanggal; lalu `/**: ros__parameters:` yaw_calib_enable: true,
  yaw_calib_measured_heading_deg: [4 angka], yaw_calib_true_deg: [0.0, 90.0, 180.0, 270.0].
- Kode keluar: 0 tersimpan, 1 gagal, 2 dibatalkan, 3 tabel ditolak.
- Pintasan menu (kalau robot punya script.sh/tmux/op3_docker.sh): tambah item "Calibration" -> di host
  `docker exec -it <container> /docker-entrypoint.sh python3 /ros2_ws/scripts/calib_imu_yaw.py` (lewat
  entrypoint supaya ROS_DOMAIN_ID/LOCALHOST sama dengan stack), di dalam container langsung python3.

5. TITIK NOL
- Tabel hanya meluruskan SKALA sudut. Titik 0 tetap: saat sensor pertama siap sesudah manager start, dan setiap
  pesan /robotis/open_cr/reset_yaw. Pemakai lama yaw (/robotis/open_cr/imu) tidak perlu diubah. Kalau demo
  memakai reset (mis. tombol demo mengirim reset_yaw), reset otomatis memakai yaw yang SUDAH terkoreksi.

6. UJI (wajib, laporkan hasil apa adanya)
- Tanpa robot: jalankan alat dengan penerbit palsu (mag_heading, mag_status=2, imu gyro 0) di ROS_DOMAIN_ID
  terpisah, --no-launch --output /tmp/uji.yaml, lewat pty. Umpan data contoh ALPHONSE 17 Sep (heading 211.74,
  256.15, 301.05, 176.65, 213.10) harus menghasilkan tabel [212.42, 256.15, 301.05, 176.65]. Uji juga
  penolakan: urutan dibalik (putar kanan) -> ditolak total != 360.
- Di robot: jalankan alat, putar 5 titik. Sesudah op3_manager start ulang, log harus berisi "KALIBRASI YAW
  AKTIF". Cek: reset_yaw -> yaw 0; putar robot 90 kiri -> yaw terbaca sekitar +90; 180 -> sekitar +/-180;
  kembali -> sekitar 0.
- Jangan tinggalkan launch uji sendiri berjalan sesudah selesai (dua launch = manager gagal membuka port).
````

---

## D. Cara operator menjalankan kalibrasi

1. Pastikan robot di lokasi pakai (lapangan) dan konfigurasi akhir: OpenCR,
   baterai, dan kabel **sudah di posisi final**. Jauhkan laptop, besi, dan
   kabel daya dari robot.
2. Jalankan `./script.sh` → **Calibration** (atau
   `scripts/op3_docker.sh imu-calib`). Kalau stack belum hidup, alat
   menyalakan robot ke pose INIT demo.
3. **Jangan tekan tombol demo** (D60) selama kalibrasi.
4. Titik 1: hadapkan robot ke arah acuan, tandai kaki di lantai, lepaskan
   robot, tekan **SPASI**.
5. Titik 2–4: putar **90° ke KIRI** setiap kali (pakai garis/tanda di lantai),
   lepaskan, tekan **SPASI**. Kalau muncul "Belum diam", tunggu sebentar lalu
   tekan SPASI lagi.
6. Titik 5: putar ke kiri sampai **tepat kembali ke tanda titik 1**, lalu
   tekan SPASI.
7. Baca hasilnya:
   - **TERSIMPAN PERMANEN**: selesai. Tabel berlaku saat op3_manager start
     berikutnya.
   - **TIDAK DISIMPAN**: baca alasannya (lihat E1), perbaiki, ulangi.
8. Jalankan demo seperti biasa. Log manager harus menampilkan `KALIBRASI YAW AKTIF`.

**Ulangi kalibrasi** setiap: pindah lapangan/lokasi, OpenCR/baterai/kabel
dipindah, firmware OpenCR di-flash ulang, atau yaw terasa meleset lagi.

---

## E. Pemeriksaan dan pemecahan masalah

### E1. Pesan penolakan alat
| Pesan | Penyebab | Tindakan |
|---|---|---|
| bentang tidak berjumlah 360 (720/1080) | diputar ke KANAN, atau urutan tertukar | ulangi, semua putaran ke KIRI |
| bentang X→Y hanya … | kompas hampir tidak berubah | cek hard-iron (E3), jauhkan logam |
| titik 5 berbeda … dari titik 1 | tidak kembali ke tanda awal, atau medan berubah | tandai lantai lebih teliti, ulangi |
| tidak ada data di mag_heading | manager lama tanpa topik, mag_status belum 2 | build ulang `--packages-above open_cr_module`, restart manager, tunggu mag_status 2 |
| Belum diam: gyro / heading bergeser | robot masih bergerak | lepaskan robot, tunggu 2–3 s |

### E2. Tabel tersimpan tapi tidak terpakai
- Log manager **tidak** menampilkan `KALIBRASI YAW AKTIF` → jalur start
  manager tidak memuat `yaw_calibration.yaml` (cek semua jalur di C-3), atau
  log `tabel kalibrasi yaw tidak sah` → rekam ulang.
- Manager sudah hidup saat kalibrasi → tabel baru berlaku setelah manager
  **start ulang**.
- Nilai `yaw_calib_enable: false` → kalibrasi sengaja dimatikan.

### E3. Heading tidak monoton / hampir tidak bergerak → kalibrasi hard-iron dulu
Gejala: robot diputar 360° tetapi heading hanya bergerak di rentang kecil, atau
bolak-balik. Di ALPHONSE rekaman 4 titik pertama: 246 → 277 → 198 → 219°.
1. Matikan op3_manager (port OpenCR harus bebas).
2. Tulis `imu_control` (addr 50) = 16 (bit4) ke OpenCR ID 200.
3. Dalam **20 detik**, putar robot pelan-pelan 360° penuh (1,5–2 putaran,
   sedikit dimiringkan) dengan baterai dan servo terpasang seperti saat
   dipakai. `mag_status` = 3 selama proses.
4. Firmware menyimpan offset hard-iron ke EEPROM (`mag_x/y/z_offset` 6–11,
   `mag_cali_valid` 12 = 0x5A).
5. Ulangi kalibrasi 5 titik.

### E4. Jebakan yang pernah terjadi
- **Baca serial mentah saat manager hidup** → bulk read rusak. Selama manager
  hidup, pakai topik ROS saja.
- **`ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` beda** antara shell dan stack →
  topik terlihat kosong. Ambil env dari `/proc/<pid>/environ` proses manager.
- **Syarat "diam" dari sebaran heading** menolak titik tertentu selamanya
  (kompas bergetar ±4–7° di sebagian arah). Pakai gyro.
- **Tabel berbasis yaw relatif** rusak setelah OpenCR nyala ulang. Wajib pakai
  `mag_heading` absolut.
- **Build hanya `open_cr_module`** saat header berubah → op3_manager memakai
  layout kelas lama (hang/crash tanpa pesan). Pakai `--packages-above`.
- **Dua launch sekaligus** → manager kedua gagal membuka port → tidak ada init.
