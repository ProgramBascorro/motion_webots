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

INPUT_GID=$(getent group input | cut -d: -f3)
DIALOUT_GID=$(getent group dialout | cut -d: -f3)

sudo docker run -it --rm \
  --net=host --ipc=host \
  --device=/dev/ttyUSB0 \
  --device=/dev/input \
  --device=/dev/uinput \
  --group-add "${INPUT_GID}" \
  --group-add "${DIALOUT_GID}" \
  --ulimit rtprio=99 --ulimit memlock=-1 \
  --cap-add SYS_NICE --cap-add SYS_RESOURCE \
  -v "$(pwd)":/ros2_ws \
  -w /ros2_ws \
  op3-webots-ros2:humble \
  bash
