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
