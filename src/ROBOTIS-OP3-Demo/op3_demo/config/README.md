# Preset dan alur demo ALPHONSE

## Preset jalan 7 September 2026

`preset_walking.yaml` bagian `jalan` memuat 29 field versi Studio
**JALAN MENUJU MEKKAH**, sesuai tiga screenshot 19:44:00, 19:44:44, dan 19:45:01.
Dibandingkan bagian `jalan` sebelumnya, hanya `p_gain` berubah dari 16 ke 36.
Lima preset mencari/putar/geser tidak diubah. File gerakan `ALPHONSE.bin`, page 2,
serta acuan netral `op3_walking_module/config/param.yaml` tidak diubah oleh
pembaruan ini.

Nama versi dalam browser Studio bukan sumber konfigurasi otomatis demo.
Demo membaca YAML ini saat node mulai; restart demo diperlukan setelah preset
diubah. Jangan menekan Apply/Start di Studio bersamaan dengan demo yang berjalan:
keduanya dapat menulis parameter walking yang sama.

Semua field memakai satuan `WalkingParam`: meter, radian, dan detik.
Langkah maju 0,016 m dan angkat kaki 0,037 m adalah batas atas; ramp demo tetap
menaikkan amplitudo secara bertahap. Gain, pose, timing, dan balance mengikuti
preset. `balance_enable` tetap `false`, sesuai screenshot, bukan diaktifkan
otomatis.

## Urutan satu siklus

1. Launch menyiapkan Walking Ready, page 2. Gerakan soccer menunggu tombol Start.
2. Jika bola tidak terlihat, gunakan `mencari_celah` (**MENCARI CELAH**) setelah
   batas kehilangan bola yang sudah ada, 0,8 detik.
3. Setelah bola ditemukan saat mencari, hentikan putaran, tunggu 0,5 detik,
   lalu dekati dengan `jalan` (**JALAN MENUJU MEKKAH**).
4. Saat bola masuk area persiapan tendangan, rem dan periksa yaw saat diam.
   Target default 0° ±15°; yaw positif memakai **Putar Kiri New**, negatif
   memakai **Putar Kanan New**. Pemetaan ini tidak ditukar otomatis.
5. Walaupun yaw sudah sesuai dari awal, jarak dan posisi kaki tetap diperiksa.
   Bola yang masih jauh harus didekati lagi dengan `jalan`, bukan langsung
   ditendang hanya karena arah bola masuk jendela kaki. Jika arah bola belum
   masuk jendela kaki, geser dulu; maju lurus tidak boleh mengejar bola yang
   terlalu menyamping.
6. Pilih kaki terdekat, lalu sesuaikan lateral menggunakan **KIRI MIN NEW** atau
   **KANAN MIN NEW**. Jika jarak bertambah selama menggeser, kembali ke tahap
   maju. Pergantian tahap memakai jeda 0,5 detik yang sudah ada.
7. Saat jarak dan jendela kaki sudah sesuai, nolkan amplitudo, tunggu gait reda,
   lalu periksa ulang yaw dan bola. Jika bola menjauh saat jeda, maju lagi.
8. Tepat sebelum aksi, verifikasi kembali jarak, kaki terpilih, dan yaw.
   Mainkan page 105 untuk kanan atau 104 untuk kiri; siklus kemudian berulang.

Jeda sebelum tendangan tidak lagi memblokir loop demo, sehingga pembaruan
deteksi, pengecekan jatuh, dan Stop tetap diproses selama jeda tersebut.
Ini tidak menjadikan aksi tendangan yang sudah dimulai sebagai pengendali
tertutup atau menjamin robot tidak jatuh.

## Pagar sebelum tendangan

- Frame bola dan posisi kepala harus baru, maksimal 0,5 detik sejak diterima.
  Ini batas umur data pada demo, bukan jaminan latensi kamera/detektor.
- Jarak dihitung dari sinar menuju **pusat bola**, termasuk posisi bola di
  dalam gambar; data terus diperbarui saat memutar dan reposisi, bukan hanya
  saat mengejar. Median memakai `kick_distance_filter_n` (default 5).
- Jarak harus valid dan tidak lebih besar dari `kick_distance` sebelum aksi.
  Jendela pan saja tidak pernah cukup untuk mengizinkan tendangan.
  Pemeriksaan terakhir memakai median dan sampel terbaru, agar median tidak
  menutupi bola yang baru saja menjauh.
- Yaw harus segar dan valid jika penyelarasan diaktifkan. Jatah ulang yang
  habis tidak melonggarkan toleransi.
- Timeout reposisi atau data bola tidak valid menghentikan reposisi dan
  membatalkan tendangan itu, lalu demo mencari/mendekati ulang selama Start
  masih aktif. Stop membatalkan siklus dan tendangan yang masih menunggu.

## Jarak tendang: kalibrasi awal kanan

Pengamatan kamera pertama beserta perbandingan ukuran manual dicatat dalam
[KALIBRASI_TENDANG_2026-09-07.md](KALIBRASI_TENDANG_2026-09-07.md).
Tinggi vertikal 52 cm, diameter bola 13,4 cm, dan posisi kontak kanan telah
dikonfirmasi operator. Jarak horizontal acuannya 14,6 cm; kalibrasi satu titik
belum menggantikan uji kontak fisik pada pose walking yang sudah berhenti.

Nilai demo dan referensi tab Tendangan Studio:

| Parameter | Nilai | Makna |
|---|---:|---|
| `kick_distance` | 0,15 m | Ambang awal berdasarkan posisi kontak kanan |
| `kick_pan_max_distance` | 0,20 m | Area persiapan penyelarasan lewat jendela pan; bukan izin aksi |
| Jendela kanan | 152–178° | Arah bola dalam konvensi sudut servo |
| Jendela kiri | 193–208° | Arah bola dalam konvensi sudut servo |
| `kick_camera_height` | 0,52 m | Tinggi vertikal kamera yang diukur operator |
| `kick_ball_real_radius_m` | 0,067 m | Setengah diameter bola 13,4 cm |
| `kick_camera_pitch_offset_deg` | 30,84° | Koreksi efektif rotasi sinar kamera dari fitting 265 pasangan |

Model saat ini memperkirakan jarak horizontal:

`jarak = (tinggi_kamera - jari_jari_bola) / tan(sudut_tunduk_ke_pusat_bola)`

Angka 0,15 m bukan celah antara ujung sepatu dan permukaan bola, dan bukan
jarak miring kamera-ke-bola. Kepala dibaca dari `/robotis/goal_joint_states`,
bukan encoder aktual; kemiringan badan, tinggi kamera, lensa, dan keterlambatan
servo bisa menambah galat. Kamera saja tidak cukup menetapkan jarak kontak
tendangan yang aman tanpa pembanding fisik.

Offset kamera ditambahkan ke `hip_pitch_offset - head_tilt_sign × head_tilt`
pada rotasi sinar 3D; arah bola untuk jendela kaki ikut terkoreksi. Field ini
ada di Studio dan disimpan ke launch tanpa mengubah parameter walking.
Nilai tak berhingga atau di luar ±60° ditolak. Perubahan geometri mengosongkan
median lama agar tidak tercampur dengan model baru. Launch lain yang tidak
memberikan offset tetap memakai 0° untuk kompatibilitas; kalibrasi ini hanya
diterapkan pada `demo.launch.xml`. Jendela kiri belum divalidasi secara fisik.

Prosedur pengukuran bersama operator:

1. Robot dipegang dengan kaki menapak dan pose sama seperti sebelum tendangan;
   tangan dan benda lain jauh dari lintasan kaki. Jangan mengubah torque/pose
   atau menjalankan halaman tendangan untuk sekadar mengambil data kamera.
2. Operator menempatkan bola diam di posisi kontak yang diinginkan untuk kaki
   kanan, lalu kiri. Ukur celah ujung kaki–permukaan bola, diameter bola, dan
   tinggi kamera. Bila menguji rumus jarak, ukur juga jarak horizontal dari
   proyeksi kamera di lantai ke pusat bola; jangan menukar acuan pengukuran.
3. Baca tampilan kamera dan telemetri beberapa detik pada setiap posisi:
   `jarak bola`, `servo head_pan`, `head_tilt`, `bola dlm gambar`, serta yaw.
   Catat rentangnya, tidak hanya satu frame. Perekaman ini hanya pengamatan;
   jangan menerbitkan perintah Start atau aksi dari alat pengukur.
4. Cocokkan estimasi dengan meteran sebelum menyepakati `kick_distance` dan
   jendela kaki. Uji tendangan pertama hanya atas persetujuan operator,
   dengan robot diamankan. Jangan menaikkan ambang hanya agar tendangan lolos.

## Build dan menjalankan

Semua perintah ROS dijalankan **dalam container `op3`**. Perintah demo adalah
`ros2 launch op3_demo demo.launch.xml`, bukan `ros2 run ...launch.xml`.

```bash
bash scripts/op3_docker.sh doctor
docker exec op3 bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-above op3_demo'
```

Header ikut berubah; jangan mengganti build di atas dengan `--packages-select`.
Jangan menjalankan launch kedua jika demo atau manager sudah aktif. Setelah
operator menghentikan sesi lama dan mengamankan robot, buka shell container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch op3_demo demo.launch.xml
```

Jika manager sengaja dijalankan terpisah, gunakan `with_manager:=false` pada
launch demo agar tidak ada dua proses yang berebut serial.

## Uji regresi tanpa hardware

`kick_alignment_test` menguji kebijakan jarak/yaw dan state machine
`BallFollower`, termasuk bola menjauh setelah putar, saat geser, dan saat jeda;
timeout, data basi, pemilihan kaki, dan kecocokan 29 field screenshot.
Tes memakai domain ROS 77 serta publisher `/op3_demo_alignment_test/...`,
bukan topik perintah robot. Tes tidak menjalankan `self_test_node`.

```bash
docker exec op3 bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select op3_demo --ctest-args -R kick_alignment_test --output-on-failure && colcon test-result --test-result-base build/op3_demo --verbose'
```
