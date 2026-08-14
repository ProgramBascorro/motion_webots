# PLAN




# Robotic Code OP3

![alt text](./images/image.png)

## How to run ( OLD ) 

build
```bash
❯ colcon build --continue-on-error
```


action editor
```bash
ros2 run op3_action_editor webots_executor.py
```


rqt_image_view 
```bash
ros2 run rqt_image_view rqt_image_view
```


ros2 debug vision

```bash
ros2 launch soccer_vision soccer_vision.launch.py publish_debug_image:=true
```

## How to run ( NEWW )

```bash
chmod +x ./run/run_vision_and_webots.sh
./run/run_vision_and_webots.sh
```

### TODO

[ ] Docker
[ ] Lokalisasi
    OTW UKF https://chatgpt.com/share/69209904-ab40-8010-be8c-09a715ca9bb4
[ ] Game controler
[ ] run_kill_all.sh : masih belum nutup
[ ] webots : kalau di refresh masih missconnect 
[ ] vission prototype : cv2 --> yolo /src/soccer_vision/launch/soccer_vision.launch.py


# DOCKER

## BUILD

Image yang dipakai robot ini (Jetson, arm64):

```bash
docker build -t motion_webots_new-op3_ros2:latest .
```

Tag itu bukan pilihan estetika: image dengan nama itu (dibangun 2026-07-04) yang
punya `torch(cpu)`, `ultralytics`, dan wheel `openvino` hasil adaptasi arm64 di
`Dockerfile`. Image lama `op3-webots-ros2:humble` (2026-06-30) lahir sebelum blok
pip YOLO masuk, jadi `import torch` di dalamnya gagal dan `op3_ball_detector`
tidak jalan. `scripts/op3_docker.sh` dan `docker-compose.yml` memilih dengan
urutan yang sama; paksa yang lain dengan `OP3_IMAGE=...`.

Build ini makan puluhan menit di Jetson (OpenCV dikompilasi dari source dengan
`-j2` supaya tidak OOM di RAM 7.4 GB). Workspace-nya sendiri **tidak** ikut
di-build ke dalam image — `install/` datang dari bind mount host, jadi perubahan
C++ cukup `colcon build --symlink-install --merge-install` di dalam container.

## RUN

```bash
scripts/op3_docker.sh up       # nyalakan container
scripts/op3_docker.sh shell    # buka shell di dalamnya (boleh berkali-kali)
scripts/op3_docker.sh doctor   # periksa colokan, port, isi container, kesegaran build
scripts/op3_docker.sh down     # matikan
```

**Satu container, satu definisi.** `scripts/op3_docker.sh` adalah satu-satunya
tempat container didefinisikan. `./script.sh` (menu Container: start / start +
shell / stop) dan `docker-compose.yml` sekarang memakai container yang sama
(`op3`) dan hanya mendelegasikan ke script itu. Dulu ada **tiga** definisi yang
saling bertentangan: menu tmux memakai `-it --rm` dan me-mount `src` read-only
(sehingga action editor tidak bisa menyimpan `.bin`), sementara compose
me-mount `src`, `build`, `install`, dan `log` terpisah-pisah dengan nama image
sendiri. Kalau perlu mengubah salah satu, ubah bersama.

Jangan lagi memakai `docker run` manual untuk robot asli. Perintah lama di sini
memakai `--device=/dev/ttyUSB0`, dan itu sumber masalah yang berulang:

- Docker menyalin major:minor perangkat **sekali** saat container start lalu
  membekukannya. Adapter FTDI FT232H re-enumerate tiap kali dicabut/di-reset dan
  bisa pindah antara `ttyUSB0`/`ttyUSB1`, sehingga node di dalam container jadi
  basi. Gejalanya bisa berupa `PORT [...] SETUP ERROR!`, atau — yang jauh lebih
  menipu — tanpa error sama sekali: manager jalan, gait berputar,
  `goal_joint_states` berayun, tapi robot diam dan `present_joint_states` beku
  bit-for-bit karena fd-nya sudah mati. `scripts/op3_docker.sh` memakai
  `--privileged -v /dev:/dev`, jadi `/dev` container mengikuti host secara live.
  (`--privileged` saja tidak cukup: container dapat devtmpfs sendiri yang tidak
  diisi udev — terbukti di robot ini, `/dev/ttyUSB0` muncul tapi symlink
  `/dev/ttyOP3` tidak.)
- `-it --rm ... bash` membuat PID 1 adalah shell-mu: menutup terminal mengirim
  SIGHUP, container mati (exit 129) dan `--rm` menghapusnya berikut manager,
  rosbridge, dan studio yang sedang jalan. Script menjalankannya detached dengan
  `sleep infinity`, jadi umur container tidak terikat satu jendela terminal.

### Port serial robot ini

Servo ID 1–20 **dan** sub controller ID 200 ada di satu bus TTL yang sama, lewat
adapter FTDI FT232H (`0403:6014`) — OpenCR-nya di-flash `opencr_op3` standar yang
melayani ID 200 di `DXL_PORT = Serial3`. Nomor `ttyUSB`-nya **tidak** dipegang
mati: `op3_manager.launch.py` dan `op3_action_editor/scripts/executor.py`
memindai `/dev/ttyUSB0-9`, memakai yang benar-benar bisa dibuka, lalu menulis
salinan `OP3.robot` dengan port itu. Jadi `OP3.robot` boleh saja menyebut
`ttyUSB0` sementara adapternya sedang di `ttyUSB1`.

```bash
OP3_DEVICE=/dev/ttyUSB1 scripts/op3_docker.sh doctor   # paksa satu port
scripts/op3_docker.sh shell -c 'python3 scripts/op3_ping.py'   # ada yang menjawab?
```

Bus DXL bermaster tunggal: `op3_manager` dan `op3_action_editor` tidak boleh
memegang port bersamaan — paketnya akan saling merusak dan baru ketahuan jauh
kemudian sebagai `first bulk read fail!!`. `doctor` bagian **2** dan **5**
menyebutkan nama proses yang sedang memegang port.

### Menghentikan stack, dan kenapa port DXL pernah tetap terkunci

```bash
./script.sh                    # menu -> "Exit tmux session"  (sesi mati + port dilepas)
./script.sh --exit             # sama, tanpa menu
./script.sh --free-port        # cuma lepaskan port, stack yang jalan dibiarkan
```

Dulu berhenti cuma berarti `tmux kill-session`, dan itu tidak cukup. Gejalanya:
`op3_manager` berikutnya menghitung mundur 30 detik lalu menyerah dengan
`/dev/ttyUSB0 is in use by op3_action_edit(pid ...) - giving up`, dan action
editor berikutnya menolak start — padahal di tmux sudah tidak ada apa-apa lagi.

Dua sebab, keduanya sekarang ditutup di `scripts/op3_tmux/lib/reap.sh`:

- `op3_action_editor` memasang handler sendiri untuk SIGINT/SIGTERM/SIGQUIT
  (`op3_action_editor/src/main.cpp:103-106`) yang memanggil `tcsetattr()` lalu
  `system("clear")`. Begitu terminalnya hilang, handler itu menggantung: proses
  tetap hidup dan tetap memegang `/dev/ttyUSBx`. Cuma SIGKILL yang menembusnya.
- proses yang sudah pindah *process group* tidak terjangkau SIGHUP pane mana pun.
  Paling sering: `ros2 run op3_action_editor executor.py` dijalankan tangan dari
  `op3_docker.sh shell` lalu di-Ctrl-C — `executor.py` mati, `ros2 run` dan node
  C++ di bawahnya dipungut init container jadi PPID 1 dan terus jalan.

Karena itu berhenti sekarang dua langkah: bunuh pohon proses tiap pane lewat
PPID (presisi, tanpa menebak nama proses), baru `kill-session`; lalu sapu siapa
pun yang masih memegang port DXL, dari mana pun asalnya. Yang disapu diberi
SIGTERM dulu, SIGKILL menyusul buat yang bandel (`PROC_TREE_GRACE_SEC`, default
2 detik).

Sapuan portnya sengaja memindai `ttyUSB0-9` **tanpa** memeriksa perangkatnya
masih ada atau tidak. Justru yang sudah tidak ada yang penting: adapter FTDI
re-enumerate (`ttyUSB0` → `ttyUSB2`), node lamanya terhapus, dan proses yang
menggantung tetap memegang fd ke inode lama — kernel menuliskannya sebagai
`/dev/ttyUSB0 (deleted)`. Kalau nama lama itu tidak ikut dicari, persis proses
yang paling perlu dibersihkan yang lolos.

Sumber yatim ditutup di dua tempat, lewat helper bersama
`scripts/lib/proc_tree.sh`:

- **`executor.py`** membunuh anak-cucunya di blok `finally`, dan memasang
  handler SIGHUP/SIGTERM supaya blok itu benar-benar kebagian jalan. Tanpa
  handler itu python mati seketika saat pane ditutup — dan menutup pane, bukan
  Ctrl-C, adalah jalur yang paling sering dipakai.
- **`scripts/bascorro_studio.sh`** membunuh pohon proses tiap background job,
  bukan cuma pembungkus `ros2 run`-nya, dan trap-nya dipasang di
  `EXIT INT TERM HUP` (trap `EXIT` saja tidak jalan kalau bash mati oleh sinyal).
  Gejala lamanya: studio menyala tapi **semua tombol Walking mati** — rosbridge
  sisa run sebelumnya masih menempel di 9090, atau asset server di 8001,
  sehingga yang baru gagal bind dan tidak pernah naik. Di UI itu cuma terlihat
  sebagai `Disconnected` di pojok kiri bawah.

SIGKILL tetap tidak bisa ditangkap siapa pun; `./script.sh --free-port` adalah
jaring pengaman terakhirnya.

### Kamera

Tidak ada `--device=/dev/video0` di mana pun, dan itu disengaja. Cabang
ALPHONSE_NEW menuliskannya satu per satu (`devices:` di compose,
`--device=/dev/video0` di menu tmux); di sini kamera ikut lewat `-v /dev:/dev`
yang sama dengan bus DXL. Dua bedanya nyata di robot ini:

- `devices:` **menolak start** kalau perangkatnya belum ada. Robot ini sering
  hidup tanpa kamera tercolok (kalibrasi servo, action editor, webots), dan
  container yang gagal start membawa serta manager, rosbridge, dan studio.
- nomor `/dev/videoN` geser persis seperti `ttyUSB`. Dengan bind, kamera yang
  dicolok **setelah** container jalan langsung terlihat di dalamnya; dengan
  `devices:` container harus dibuat ulang dulu.

`doctor` bagian **1** menyebut kamera yang ada di host dan bagian **5** memeriksa
apakah kamera itu tembus ke dalam container. Tanpa kamera, `op3_ball_detector`
dan `op3_yolo_vision` tetap jalan tapi topiknya sepi — node-nya tidak mengeluh.

### Membaca isi halaman action tanpa membuka editor

```bash
python3 scripts/action_page_fk.py            # page 2 (WALKING_READY)
python3 scripts/action_page_fk.py --page 3   # INIT_BARU
python3 scripts/action_page_fk.py --page 1 --raw
```

Berguna karena action editor memegang port DXL, jadi tidak bisa dibuka
bersamaan dengan `op3_manager` cuma untuk melihat sebuah page. Script ini
membaca `motion_4095_ORION.bin` langsung, mencetak nilai tiap sendi, lalu
menghitung FK-nya jadi `x/y/z_offset` dan menjajarkannya dengan `param.yaml`.

Angka FK-nya sendiri **jangan** disalin ke `param.yaml`: script ini menganggap
raw 2048 == sendi lurus, dan kalibrasi nol servo robot ini tidak begitu, jadi
`z_offset`-nya keluar dari rentang masuk akal. Yang dipakai robot ini adalah
mekanisme capture/bias di `op3_walking_module` — ia menangkap pose page 2 saat
runtime, jadi tidak butuh FK maupun kalibrasi nol yang benar. Script ini
mencetak peringatan itu sendiri kalau hasilnya di luar rentang.

(Di CHRONUS_NEW berkas ini bernama `scripts/init_baru_fk.py` dan nilai sendinya
disalin tangan ke dalam script. Di sini namanya diganti karena page 2 sudah
bernama WALKING_READY dan INIT_BARU pindah ke page 3, dan nilainya dibaca dari
bin supaya tidak bisa basi.)

### `ros2 run ...` menjawab "Package not found"

Hampir selalu artinya kamu berada **di host**, bukan di dalam container. Seluruh
`install/` dibangun dengan `--symlink-install` di dalam container, jadi isinya
menunjuk ke `/ros2_ws/...` yang tidak ada di host — `source install/setup.bash`
tetap berhasil (exit 0) tapi diam-diam melewati hampir semua paket, sehingga
paketnya seolah lenyap. Jalankan `scripts/op3_docker.sh shell` dulu.

Dua hal yang membuatnya kambuh, keduanya sudah ditutup di script:

- **Setelah PC reboot** container hilang. Sekarang dipasang
  `--restart unless-stopped`, jadi ia hidup lagi sendiri. `down` tetap dihitung
  "stopped" dan tidak akan bangkit sendiri.
- **`docker exec` melewati ENTRYPOINT**, jadi shell polos lahir tanpa ROS sama
  sekali dan `ros2` bahkan tidak ada di PATH. `shell` sekarang membungkusnya
  dengan `/docker-entrypoint.sh`, yang men-source `/opt/ros/humble` **dan**
  `/ros2_ws/install`. `scripts/container_bashrc` ikut di-mount ke
  `/root/.bashrc`, jadi `docker exec -it op3 bash` manual pun tetap beres.

`doctor` bagian **6** menguji ulang semua ini dari shell baru (termasuk modul
python YOLO yang dipakai `op3_ball_detector`), dan bagian **7** menangkap paket
yang dibangun sebelum `robotis_controller.h` terakhir berubah — kelas bug yang
tidak bergejala jelas (action editor berhenti tepat setelah "Load offsets..."
lalu berputar 100% CPU tanpa satu pun pesan error).
