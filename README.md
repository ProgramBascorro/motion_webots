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

Jangan lagi memakai `docker run` manual untuk robot asli. Perintah lama di sini
memakai `--device=/dev/ttyUSB0`, dan itu sumber masalah yang berulang:

- Docker menyalin major:minor perangkat **sekali** saat container start lalu
  membekukannya. U2D2 (FTDI FT232H) sering re-enumerate dan berpindah antara
  `ttyUSB0`/`ttyUSB1`, sehingga node di dalam container jadi basi. Gejalanya bisa
  berupa `PORT [...] SETUP ERROR!`, atau — yang jauh lebih menipu — tanpa error
  sama sekali: manager jalan, gait berputar, `goal_joint_states` berayun, tapi
  robot diam dan `present_joint_states` beku bit-for-bit karena fd-nya sudah mati.
  `scripts/op3_docker.sh` memakai `--privileged -v /dev:/dev`, jadi `/dev`
  container mengikuti host secara live. (`--privileged` saja tidak cukup —
  tanpa bind mount, `/dev/ttyUSB*` dan `/dev/serial` tidak muncul di dalam.)
- `-it --rm ... bash` membuat PID 1 adalah shell-mu: menutup terminal mengirim
  SIGHUP, container mati (exit 129) dan `--rm` menghapusnya berikut manager,
  rosbridge, dan studio yang sedang jalan. Script menjalankannya detached dengan
  `sleep infinity`, jadi umur container tidak terikat satu jendela terminal.

Kalau ada yang aneh dengan port/serial, jalankan `scripts/op3_docker.sh doctor`
sebelum menebak-nebak — dia memeriksa colokan di host, menghitung USB disconnect
30 menit terakhir, mencocokkan by-id di `OP3.robot` dengan yang benar-benar
tercolok, dan mencoba membuka port itu dari dalam container.
