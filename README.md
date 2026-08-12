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

sudo docker build -t op3-webots-ros2:humble .

## RUN

```bash
scripts/op3_docker.sh up       # nyalakan container
scripts/op3_docker.sh shell    # buka shell di dalamnya (boleh berkali-kali)
scripts/op3_docker.sh doctor   # periksa colokan, node perangkat, kecocokan OP3.robot
scripts/op3_docker.sh down     # matikan
```

**Satu container, satu definisi.** `scripts/op3_docker.sh` adalah satu-satunya
tempat container didefinisikan. `./script.sh` (menu Container: start / start +
shell / stop) dan `docker-compose.yml` sekarang memakai container yang sama
(`op3`) dan hanya mendelegasikan ke script itu. Dulu ada **tiga** definisi yang
saling bertentangan: menu tmux memakai `-it --rm` dan me-mount `src` read-only
(sehingga action editor tidak bisa menyimpan `.bin`), sementara compose hanya
me-mount `./src` tanpa `build/` dan `install/` (sehingga semua `ros2 run`
menjawab "Package not found"). Kalau perlu mengubah salah satu, ubah bersama.

Jangan lagi memakai `docker run` manual untuk robot asli. Perintah lama di sini
memakai `--device=/dev/ttyUSB0`, dan itu sumber masalah yang berulang:

- Docker menyalin major:minor perangkat **sekali** saat container start lalu
  membekukannya. U2D2 (FTDI FT232H) re-enumerate tiap kali dicabut/di-reset dan
  bisa pindah antara `ttyUSB0`/`ttyUSB1`, sehingga node di dalam container jadi
  basi. Gejalanya bisa berupa `PORT [...] SETUP ERROR!`, atau — yang jauh lebih
  menipu — tanpa error sama sekali: manager jalan, gait berputar,
  `goal_joint_states` berayun, tapi robot diam dan `present_joint_states` beku
  bit-for-bit karena fd-nya sudah mati. `scripts/op3_docker.sh` memakai
  `--privileged -v /dev:/dev`, jadi `/dev` container mengikuti host secara live —
  termasuk symlink udev `/dev/ttyOP3` yang dipakai `OP3.robot`. (`--privileged`
  saja tidak cukup — tanpa bind mount, `/dev/ttyUSB*` tidak muncul di dalam.)
- `-it --rm ... bash` membuat PID 1 adalah shell-mu: menutup terminal mengirim
  SIGHUP, container mati (exit 129) dan `--rm` menghapusnya berikut manager,
  rosbridge, dan studio yang sedang jalan. Script menjalankannya detached dengan
  `sleep infinity`, jadi umur container tidak terikat satu jendela terminal.

### Port serial robot ini (CHRONUS)

`/dev/ttyOP3` adalah symlink buatan udev ke U2D2 (FTDI `0403:6014`). Satu port
itu membawa servo ID 1–20 **dan** sub controller ID 200, karena OpenCR-nya
di-flash `opencr_op3` standar yang melayani ID 200 di `DXL_PORT = Serial3`.
`/dev/ttyACM0` (micro-USB OpenCR) hanya konsol debug/flash dan tidak pernah
membalas paket DXL — kalau `ttyOP3` sampai menunjuk ke sana, semua 20 joint akan
`does NOT respond!!`. Symlink-nya butuh rule di host, sekali saja:

```bash
sudo cp scripts/99-op3-opencr.rules /etc/udev/rules.d/99-op3-opencr.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`scripts/op3_docker.sh doctor` bagian **2** memeriksa rule ini dan ke mana
`ttyOP3` menunjuk, jadi tidak perlu menebak-nebak.

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
  `/ros2_ws/install`. `up` juga menanam auto-source di `.bashrc` container,
  jadi `docker exec -it op3 bash` manual pun tetap beres.

`doctor` bagian **6** menguji ulang semua ini dari shell baru, dan bagian **7**
menangkap paket yang dibangun sebelum `robotis_controller.h` terakhir berubah —
kelas bug yang tidak bergejala jelas (action editor berhenti tepat setelah
"Load offsets..." lalu berputar 100% CPU tanpa satu pun pesan error).
