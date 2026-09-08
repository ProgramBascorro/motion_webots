# Pengamatan jarak tendang, 7 September 2026

Status: ukuran operator sudah dikonfirmasi dan kalibrasi satu titik untuk
posisi kontak **tendangan kanan** sudah diterapkan pada konfigurasi demo dan
Studio. Belum ada uji tendangan fisik atau validasi kontak kaki kiri.

## Rekaman langsung

Pengamatan 12 detik sekitar 22:13 WIB memakai `scripts/observe_kick_geometry.py`.
Alat hanya berlangganan sensor dan membaca layanan parameter; tidak menerbitkan
perintah kepala, walking, pergantian modul, atau aksi.

Artefak lokal, relatif terhadap workspace:

- `log/kick_geometry/capture-ucazl9n4/measurement.json`: parameter, deteksi,
  sampel kepala/IMU, timestamp, dan hasil hitungan.
- `log/kick_geometry/capture-ucazl9n4/middle.jpg`: gambar asli pertengahan rekaman,
  640 × 360 piksel; bola dan ujung kaki terlihat.
- `first.jpg` dan `last.jpg` di folder yang sama: gambar awal dan akhir.

281 pesan deteksi diterima dan semuanya berisi bola. Terdapat 265 pasangan
deteksi–sudut kepala dengan selisih timestamp maksimal 100 ms. Sudut kepala
dipasangkan ke waktu gambar yang dibawa deteksi, bukan waktu pesan diterima.

Hasil berikut adalah **baseline sebelum kalibrasi**, bukan konfigurasi baru:

| Besaran | Median | Rentang |
|---|---:|---:|
| Estimasi jarak horizontal, sudut kepala goal | 50,09 cm | 50,00–50,16 cm |
| Estimasi jarak horizontal, sudut kepala present | 50,24 cm | 50,15–50,31 cm |
| Sudut tunduk sinar ke pusat bola, memakai goal | 41,94° | 41,90–41,99° |
| Arah bola dalam konvensi servo | 171,95° | 171,92–171,99° |
| Radius deteksi | 81,48 px | 81,27–81,70 px |
| Head pan goal | −0,703° | tetap selama rekaman |
| Head tilt goal | −29,883° | tetap selama rekaman |
| Yaw IMU | +45,42° | +44,28–+46,56° |

Menurut jendela yang terpasang, 265/265 pasangan masuk sisi kanan (152–178°).
Ini klasifikasi arah menurut model sebelum kalibrasi; keberhasilan kontak kaki ke bola
belum diuji. Yaw juga masih di luar target 0° ±15° pada rekaman ini.

Angka 50,09 cm adalah **estimasi model**, bukan pengukuran meteran. Sebarannya
yang sempit membuktikan kestabilan pembacaan pada pose ini, bukan ketepatan
absolut geometri robot.

## Parameter dan kualitas data

Parameter runtime lama yang dibaca saat perekaman:

- `kick_camera_height = 0.56 m`
- `kick_ball_real_radius_m = 0.11 m` (diameter asumsi 22 cm)
- `kick_distance = 0.30 m`
- `kick_pan_max_distance = 0.40 m`
- `kick_head_tilt_sign = 1.0`
- `hip_pitch_offset = 0.1396 rad`

Walking runtime masih berisi pose/timing netral: `init_x_offset = -0.02 m`,
`init_pitch_offset = -0.07 rad`, periode 0,8 s, dan amplitudo x/y/sudut nol.
Pengamatan ini belum memvalidasi geometri pada pose berjalan baru dengan
`init_x_offset = -0.03 m`, `init_pitch_offset = -0.12 rad`, periode 0,77 s.

Selisih waktu terima terhadap timestamp gambar: median 826,8 ms untuk gambar
dan 850,9 ms untuk deteksi. Selisih ini perlu ditelusuri sebagai keterlambatan
pipeline atau perbedaan acuan clock sebelum dipakai untuk bola bergerak.
Ini bukan pengukuran langsung latensi fisik kamera. Pada rekaman ini bola dan
sudut kepala stabil.

`camera_info` masih menyebut 1280 × 720, sedangkan gambar yang diterima
640 × 360. Validitas matriks kalibrasi tersebut belum terbukti. Hitungan observer
mengikuti FOV yang memang dipakai `BallTracker` (setengah FOV horizontal 35,2°,
vertikal 21,6°); matriks `camera_info` tidak menggantikan model demo.

## Mencocokkan ukuran manual operator

Konfirmasi operator:

- 52 cm adalah tinggi kamera **tegak ke lantai**, bukan jarak miring ke kaki.
- Diameter bola 13,4 cm, sehingga radius fisiknya **6,7 cm**, bukan 11 cm.
- Posisi bola pada rekaman merupakan acuan kontak tendangan **kanan**.
- Ukuran sebelumnya: kamera ke bawah bola 54 cm, ke permukaan bola sekitar
  40 cm. Hitungan memakai 54 cm sebagai jarak lurus ke titik kontak bola–lantai;
  tinggi dan jarak ini harus berawal dari pusat optik kamera yang sama.

Jarak horizontal dari proyeksi kamera di lantai ke pusat bola:

`sqrt(0.54² - 0.52²) = 0.145602 m`, sekitar **14,6 cm**.

Beda tinggi kamera–pusat bola adalah `0.52 - 0.067 = 0.453 m`.
Geometri ini memprediksi jarak ke permukaan terdekat
`sqrt(0.145602² + 0.453²) - 0.067 = 0.408825 m`, sekitar **40,9 cm**.
Itu mendekati ukuran permukaan sekitar 40 cm, tetapi bukan kecocokan persis;
ukuran permukaan tidak dipakai untuk memaksa hasil kalibrasi.

Mengganti tinggi dan radius saja menghasilkan sekitar **50,4 cm** pada
rekaman: perubahan beda tinggi 0,450 → 0,453 m justru hampir saling meniadakan.
Karena itu model orientasi sinar kamera perlu dikoreksi, bukan menaikkan
ambang tendangan agar angka yang salah lolos.

Kalibrasi memasukkan `kick_camera_pitch_offset_deg` pada rotasi sinar:

`pitch = hip_pitch_offset - kick_head_tilt_sign × head_tilt + offset_deg × π/180`

Pitch tersebut diterapkan sebelum rotasi pan, sehingga **jarak dan arah bola
dikoreksi bersama**. Offset bukan perintah sudut kepala atau parameter gait.
Skrip mencocokkan sinar dari setiap deteksi dengan depresi referensi
`atan2(0.453, 0.145602) = 72.18168°`, lalu mengambil median offset.
Hasil dari 265 pasangan: **30,83937°**, dibulatkan menjadi **30,84°**.

Hasil pemutaran ulang dengan tinggi/radius/offset baru:

| Besaran | Median | Rentang |
|---|---:|---:|
| Jarak horizontal dari head goal | 14,560 cm | 14,515–14,599 cm |
| Jarak horizontal dari head present | 14,633 cm | 14,588–14,672 cm |
| Arah bola dalam konvensi servo, goal | 161,193° | 161,117–161,278° |
| Sampel di jendela kanan | 265/265 | Jendela tetap 152–178° |

Replay memakai data yang juga dipakai untuk fitting; ini pemeriksaan
konsistensi, **bukan validasi independen**. Sebaran sempit bukan jaminan akurasi
milimeter. Jarak 54 cm sangat dekat dengan tinggi 52 cm: bila masing-masing
ukuran bergeser ±0,5 cm berlawanan arah, hasil horizontal berkisar 10,3–17,8 cm.
Pengukuran horizontal langsung lebih baik untuk menyempurnakan ambang.

## Keputusan konfigurasi

Nilai baru pada `launch/demo.launch.xml` dan referensi Studio:

| Parameter | Nilai |
|---|---:|
| `kick_camera_height` | 0,52 m |
| `kick_ball_real_radius_m` | 0,067 m |
| `kick_camera_pitch_offset_deg` | 30,84° |
| `kick_distance` | 0,15 m |
| `kick_pan_max_distance` | 0,20 m |

Ambang awal **15 cm** membulatkan titik acuan 14,6 cm; ini bukan jarak celah
ujung kaki–kulit bola. Batas 20 cm memulai penyelarasan 5 cm lebih awal,
bukan mengizinkan aksi pada 20 cm. Jendela kaki kanan/kiri dan parameter
walking tidak diubah untuk kalibrasi ini. Yaw rekaman +45,4° tetap wajib
dikoreksi ke 0° ±15° sebelum aksi; kalibrasi jarak tidak melewati pagar yaw.

Offset ini adalah **koreksi efektif satu titik**, bukan hasil pengukuran
mekanis kemiringan dudukan kamera. Ia dapat menyerap galat pose, FOV, titik
nol servo, dan tinggi. Matriks `camera_info` belum tervalidasi dan pose rekaman
belum sama dengan pose walking baru yang sudah berhenti. Periksa kembali
pada pose tersebut, posisi kanan/kiri, serta beberapa jarak sebelum demo
bergerak bebas. Jangan menganggap kontak kaki kiri sudah terkalibrasi hanya
karena kedua kaki memakai rumus yang sama.

Tidak ada parameter live, pose kepala, torque, gait, atau aksi yang dikirim
saat fitting. Binary/parameter baru perlu dimuat melalui restart demo yang
dikendalikan operator, dengan robot dipegang atau diamankan. Backend Studio
juga perlu restart dan halaman browser dimuat ulang agar field baru tersedia.
Jangan menekan Start sampai pose dan bola telah diperiksa.

## Mengulang pengamatan

Kamera dan manager sudah harus aktif. Samakan konfigurasi koneksi dengan
proses robot; pada sesi ini domain 42 dan `ROS_LOCALHOST_ONLY=0`.

```bash
docker exec op3 bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 && python3 scripts/observe_kick_geometry.py --seconds 12'
```

Skrip mencetak lokasi rekaman baru dan ringkasan pengukuran. Tes perhitungan
serta pemasangan timestamp dapat dijalankan tanpa ROS/hardware:

```bash
python3 -m unittest discover -s src/ROBOTIS-OP3-Demo/op3_demo/test -p test_kick_geometry_observer.py -v
```

Mengulang fitting yang sama, sepenuhnya offline dan tanpa ROS:

```bash
python3 scripts/observe_kick_geometry.py \
  --replay log/kick_geometry/capture-ucazl9n4/measurement.json \
  --camera-height 0.52 --ball-diameter 0.134 --floor-contact-slant 0.54
```

Perintah mencetak hasil JSON; tidak menimpa rekaman atau mengubah robot.
Observer tetap bisa membaca node lama yang belum punya parameter offset,
dan mencatat parameter tersebut sebagai tidak tersedia (model lama offset 0).
