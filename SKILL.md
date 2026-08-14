# ROBOTIS OP3 ROS Packages Skill

Use this skill when helping with ROBOTIS OP3 / ROBOTIS-OP3 ROS packages, especially `op3_manager`, action pages, walking, head control, tuning tools, topics, services, launch files, and debugging package-level integration.

Primary source:
- ROBOTIS e-Manual: https://emanual.robotis.com/docs/en/platform/op3/robotis_ros_packages/
- Source markdown: https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/op3/robotis_ros_packages.md

## How to use this skill

When answering OP3 questions:

1. Identify which package/module is involved: `op3_manager`, `op3_action_module`, `op3_base_module`, `op3_head_control_module`, `op3_walking_module`, `op3_action_editor`, `op3_walking_tuner`, `op3_offset_tuner`, `op3_ball_detector`, or related message packages.
2. Prefer ROS interfaces first: topics, services, parameters, launch files, and config paths.
3. Warn when a tool directly controls hardware. Do not recommend running multiple direct-control tools at the same time unless the official docs allow it.
4. For real robot issues, separate software/package problems from hardware problems: USB device, OpenCR power, Dynamixel baudrate, servo ID, torque, sensor read, and `.robot` config.
5. Give commands that are copy-pasteable for ROS 2, but mention if a command may differ between ROS 1 docs and ROS 2 Jazzy/Humble ports.
6. For code agents, prefer creating a team-owned wrapper/bridge package instead of editing vendor ROBOTIS packages directly.

## Fast decision guide

- Need to play kick, stand-up, or custom motion: use `op3_action_module` through `op3_manager`; publish page numbers or call the relevant service.
- Need to edit action pages: use `op3_action_editor`; do not run it concurrently with `op3_manager` controlling the robot.
- Need walking parameter tuning: use `op3_walking_tuner` / walking module interfaces.
- Need head pan/tilt tracking: use `op3_head_control_module`.
- Need init pose / base pose: use `op3_base_module`.
- Need camera ball detection: use `op3_ball_detector`; check parameters, camera topic, and OpenCV/camera launch.
- Getting `first bulk read fail`: check USB/OpenCR/Dynamixel power/baud/robot file before blaming behavior code.

## Safety and reliability rules for agents

- Treat OP3 hardware commands as potentially dangerous. Keep robot supported when testing standing, walking, kick, and stand-up pages.
- Confirm the robot is physically safe before torque-on, walking, or action playback.
- Avoid editing vendor code unless necessary. Prefer wrapper packages like `bascorro_motion_bridge`.
- When suggesting changes to `.robot`, `.yaml`, or action files, mention backing up the original file first.
- If the user is on ROS 2 Jazzy/Humble, do not blindly copy ROS 1-only command syntax.

---

## Detailed reference

# AGENTS.md — ROBOTIS OP3 ROS Packages Notes for Coding Agents

Source documentation:
- ROBOTIS e-Manual: https://emanual.robotis.com/docs/en/platform/op3/robotis_ros_packages/
- Source markdown: https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/op3/robotis_ros_packages.md
- Raw source path: `ROBOTIS-GIT/emanual/docs/en/platform/op3/robotis_ros_packages.md`

Purpose: give LLM/coding agents a compact, repo-ready reference for working with ROBOTIS OP3 ROS 2 packages. Prefer the official e-Manual/source for exact wording and updates.

---

## High-level architecture

ROBOTIS OP3 uses `op3_manager` to control ROBOTIS Framework on the robot. The manager loads robot description/config files, applies offsets, loads sensor and motion modules, starts a timer, then exchanges data with DYNAMIXEL and OpenCR at the frequency configured in the `.robot` file.

Typical `op3_manager` flow:

```cpp
// initialize robot
controller->initialize(g_robot_file, g_init_file);

// load offset
if (g_offset_file != "")
  controller->loadOffset(g_offset_file);

// add sensor module
controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

// add motion modules
controller->addMotionModule((MotionModule*) ActionModule::getInstance());
controller->addMotionModule((MotionModule*) BaseModule::getInstance());
controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
controller->addMotionModule((MotionModule*) WalkingModule::getInstance());

// start timer
controller->startTimer();
```

Important: `op3_manager` has direct control of the robot. Do not run other direct-control programs like `op3_action_editor`, `op3_offset_tuner`, or `op3_walking_tuner` at the same time unless the docs/tool explicitly allow it.

---

## Main ROS packages and modules

### `op3_action_module`

Manages OP3 action/motion pages. It is compiled as a library and loaded by `op3_manager`. Actions contain joint angles for each time frame to define full motions.

Subscribed topics:

- `/robotis/action/page_num` — `std_msgs/msg/Int32`
  - `1 ~ 255`: play action page
  - `-1`: stop action
  - `-2`: brake action
- `/robotis/action/start_action` — `op3_action_module_msgs/msg/StartAction`
  - Plays a page and can apply specified joint names/angles to the action execution result.

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`
- `/robotis/movement_done` — `std_msgs/msg/String`
  - `action`: action finished
  - `action_failed`: action failed

Service:

- `/robotis/action/is_running` — `op3_action_module_msgs/srv/IsRunning`

Parameter:

- `/action file path` — string, default `op3_action_module/data/ALPHONSE.bin`

Agent notes:

- For kick/stand-up/custom motions, prefer publishing an action page number through `/robotis/action/page_num` or wrapping it behind a semantic motion bridge.
- Action page files are binary. Use `op3_action_editor` to inspect or edit.

---

### `op3_base_module`

Manages the initial/default posture. It is loaded by `op3_manager`.

Subscribed topics:

- `/robotis/base/ini_pose` — `std_msgs/msg/String`
  - Moves OP3 to the initial default posture. This can execute even when the base module is inactive.

Published topics:

- `/robotis/enable_ctrl_module` — `std_msgs/msg/String`
  - Activates `op3_base_module` and moves OP3 to initial posture.
- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`

Data file:

- `/op3_base_module/data/ini_pose.yaml`

YAML fields:

- `mov_time` — estimated time to target points, seconds
- `via_num` — number of waypoints
- `via_time` — time between waypoints
- `via_pose` — joint angles for waypoints, degrees
- `tar_pose` — target joint angles for initial posture

---

### `op3_head_control_module`

Controls OP3 head motion. Loaded by `op3_manager`.

Subscribed topics:

- `/robotis/head_control/scan_command` — `std_msgs/msg/String`
  - Requests a looking-around scan motion.
- `/robotis/head_control/set_joint_states` — `sensor_msgs/msg/JointState`
  - Sets head joint angles.
- `/robotis/head_control/set_joint_states_offset` — `sensor_msgs/msg/JointState`
  - Applies offsets relative to current head angles.

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`

Agent notes:

- For soccer tracking, avoid fighting control loops. Let head tracking aim at the ball and let body yaw reduce head pan toward zero.

---

### `op3_walking_module`

Controls OP3 walking. Loaded by `op3_manager`.

Subscribed topics:

- `/robotis/walking/command` — `std_msgs/msg/String`
  - Starts/stops walking.
- `/robotis/walking/set_params` — `op3_walking_module_msgs/msg/WalkingParam`
  - Updates walking parameters.

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`

Service:

- `/robotis/walking/get_params` — `op3_walking_module_msgs/msg/GetWalkingParam`

Parameter:

- `/walking_param_path` — string, default `op3_walking_module/config/param.yaml`

Data file:

- `/op3_walking_module/config/param.yaml`

Important walking parameters:

- `x_offset` — body/foot offset front-back, meters
- `y_offset` — left-right offset, meters
- `z_offset` — up-down offset, meters
- `roll_offset` — roll offset, degrees
- `pitch_offset` — pitch offset, degrees
- `yaw_offset` — yaw offset, degrees
- `hip_pitch_offset` — hip pitch offset, DYNAMIXEL position values / degrees
- `period_time` — time for two full steps, left + right, milliseconds
- `dsp_ratio` — double-support phase ratio; time both feet touch ground vs. single-support phase
- `foot_height` — foot lift height, meters
- `swing_right_left` — lateral body swing, meters
- `swing_top_down` — vertical body swing, meters
- `pelvis_offset` — pelvis roll offset / hip roll value, degrees
- `arm_swing_gain` — arm swing gain relative to step length
- `balance_hip_roll_gain` — gyro roll gain
- `balance_knee_gain` — gyro pitch gain
- `balance_ankle_roll_gain` — gyro roll gain
- `balance_ankle_pitch_gain` — gyro pitch gain
- `p_gain`, `i_gain`, `d_gain` — listed as not implemented in this documentation

Agent notes:

- Tune walking conservatively. Change one parameter at a time.
- `period_time`, `dsp_ratio`, `foot_height`, `swing_right_left`, `swing_top_down`, and balance gains strongly affect stability.

---

### `op3_direct_control_module`

Allows direct joint control.

Subscribed topics:

- `/robotis/direct_control/set_joint_states` — `sensor_msgs/msg/JointState`

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`

Parameters:

- `/robotis/direct_control/default_moving_time` — double, default `0.5`
- `/robotis/direct_control/default_moving_angle` — double, default `30`
- `/robotis/direct_control/check_collision` — bool, default `true`

Agent notes:

- Use this carefully on the real robot. Prefer semantic motion commands or action pages for known safe poses.

---

### `op3_tuning_module`

Used for motion tuning, offsets, and gain adjustment.

Subscribed topics:

- `/robotis/tuning_module/tuning_pose` — `std_msgs/msg/String`
- `/robotis/tuning_module/joint_offset_data` — `op3_tuning_module_msgs/msg/JointOffsetData`
- `/robotis/tuning_module/joint_gain_data` — `op3_tuning_module_msgs/msg/JointOffsetData`
- `/robotis/tuning_module/torque_enable` — `op3_tuning_module_msgs/msg/JointTorqueOnOffArray`
- `/robotis/tuning_module/command` — `std_msgs/msg/String`

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`
- `/robotis/enable_ctrl_module` — `std_msgs/msg/String`
- `/robotis/sync_write_item` — `robotis_controller_msgs/msg/SyncWriteItem`
- `/robotis/enable_offset` — `std_msgs/msg/Bool`

Services:

- `/robotis/tuning_module/get_present_joint_offset_data` — `op3_tuning_module_msgs/msg/GetPresentJointOffsetData`
- `/robotis/set_present_ctrl_modules` — `robotis_controller_msgs/msg/SetModule`
- `/robotis/load_offset` — `robotis_controller_msgs/msg/LoadOffset`

Parameters:

- `offset_file_path` — string, default `~/data/tune_pose.yaml`
- `init_file_path` — string, default `~/data/offset.yaml`

---

### `open_cr_module`

Sensor module for OpenCR. Provides gyro, acceleration, button, and LED IO interface.

Published topics:

- `/robotis/status` — `robotis_controller_msgs/msg/StatusMsg`
- `/robotis/open_cr/imu` — `sensor_msgs/msg/Imu`
- `/robotis/open_cr/button` — `std_msgs/msg/String`

Agent notes:

- Button input is used by default demo programs.
- IMU is important for fall detection, balancing, and future walking stabilization.

---

## `op3_manager`

Run command from docs:

```bash
sudo bash
ros2 launch op3_manager op3_manager.launch.py
```

Launch parameters:

- `gazebo` — bool, default `false`
- `gazebo_robot_name` — string, default `""`
  - Example: if `op3`, joint states topic may be `/op3/joint_states`.
- `offset_file_path` — string, path to offset data and initial posture data
- `robot_file_path` — string, path to `.robot` description file
- `init_file_path` — string, path to joint initialization YAML
- `device_name` — string, default `/dev/ttyUSB0`
- `baud_rate` — int, default `2000000`

Agent notes:

- If `/dev/ttyUSB0` changes, prefer a udev rule or stable symlink rather than hardcoding a moving USB path.
- On real robot, stop `op3_demo`, action editor, offset tuner, and walking tuner before launching manager if they directly access hardware.

---

## Other packages

### `op3_balance_control`

Library for balance control to improve walking. The documentation says it is not currently implemented because of FT/IMU sensor requirements.

### `op3_localization`

ROS node for localization. Publishes TF from `/world` to `/body_link`.

Subscribed topics:

- `/robotis/pelvis_pose` — `geometry_msgs/msg/PoseStamped`
- `/robotis/pelvis_pose_reset` — `std_msgs/msg/String`

---

## Message packages

### `op3_action_module_msgs`

- Message: `op3_action_module_msgs/msg/StartAction`
- Service: `op3_action_module_msgs/srv/IsRunning`

### `op3_walking_module_msgs`

- Message: `WalkingParam.msg`
- Services:
  - `GetWalkingParam.srv`
  - `SetWalkingParam.srv`

### `op3_tuning_module_msgs`

- Messages:
  - `JointOffsetData.msg`
  - `JointOffsetPositionData.msg`
  - `JointTorqueOnOff.msg`
  - `JointTorqueOnOffArray.msg`
- Service:
  - `GetPresentJointOffsetData.srv`

### `op3_offset_tuner_msgs`

- Messages:
  - `JointOffsetData.msg`
  - `JointOffsetPositionData.msg`
  - `JointTorqueOnOff.msg`
  - `JointTorqueOnOffArray.msg`
- Service:
  - `GetPresentJointOffsetData.srv`

---

## `op3_ball_detector`

Package for OP3 vision demo. Uses OpenCV to search for a ball with a specific color.

Run with USB camera:

```bash
ros2 launch op3_ball_detector ball_detector_from_usb_cam.launch.py
```

Required utility package:

```bash
sudo apt-get install v4l-utils
```

Subscribed topics:

- `~/enable` — `std_msgs/msg/Bool`
  - True starts ball search; False stops it.
- `~/image_in` — `sensor_msgs/msg/Image`
- `~/cameraInfo_in` — `sensor_msgs/msg/CameraInfo`

Published topics:

- `~/image_out` — `sensor_msgs/msg/Image`
- `~/camera_info` — `sensor_msgs/msg/CameraInfo`
- `~/circle_set` — `ball_detector/circleSetStamped`
  - `header`: `std_msgs/msg/Header`
  - `circles`: `geometry_msgs/msg/Point`
    - `x`: center x in image coordinates
    - `y`: center y in image coordinates
    - `z`: detected radius

Parameters:

- `/yaml_path`
- `/gaussian_blur_size` — int
- `/gaussian_blur_sigma` — double
- `/canny_edge_th` — double
- `/hough_accum_resolution` — double
- `/min_circle_dist` — double
- `/hough_accum_th` — double
- `/min_radius`, `/max_radius` — int
- `/filter_h_min`, `/filter_h_max` — int HSV hue range
- `/filter_s_min`, `/filter_s_max` — int HSV saturation range
- `/filter_v_min`, `/filter_v_max` — int HSV value range
- `/use_second_filter` — bool
- `/filter_debug` — bool

Example YAML from docs:

```yaml
gaussian_blur_size: 7
gaussian_blur_sigma: 2
canny_edge_th: 100
hough_accum_resolution: 1
min_circle_dist: 100
hough_accum_th: 28
min_radius: 20
max_radius: 300
filter_h_min: 350
filter_h_max: 15
filter_s_min: 200
filter_s_max: 255
filter_v_min: 60
filter_v_max: 255
use_second_filter: false
filter2_h_min: 30
filter2_h_max: 355
filter2_s_min: 0
filter2_s_max: 40
filter2_v_min: 200
filter2_v_max: 255
ellipse_size: 2
filter_debug: false
```

Agent notes:

- Be careful with ROS 2 parameter types. For example, `canny_edge_th` is documented as `double`; use `100.0` if the launch system enforces type strictly.
- For newer YOLO/OpenVINO pipelines, treat this detector as a reference or fallback, not necessarily the final soccer perception system.

---

## `op3_demo`

Basic demonstrations: soccer, face tracking, and action sequence.

Run:

```bash
ros2 launch op3_demo demo.launch.xml
```

Buttons:

- Mode button:
  - short press in ready mode: switch next demo (`soccer > vision > action`)
  - long press during demo: return to demo ready mode
- Start button:
  - short press: play selected demo
  - during demo: pause/resume
- User button: available for user-defined features
- Reset button: cuts off power to all connected DYNAMIXEL actuators

Subscribed topics:

- `/robotis/open_cr/button` — `std_msgs/msg/String`

Published topics:

- `/robotis/base/ini_pose` — `std_msgs/msg/String`
- `/robotis/sync_write_item` — `std_msgs/msg/String`
- `/play_sound_file` — `std_msgs/msg/String`

Demos:

- Soccer Demo — searches for colored ball and plays with it
- Vision Demo — detects and follows faces
- Action Demo — plays predefined actions while speaking

---

## `op3_description`

Contains ROBOTIS OP3 URDF model assets.

Package structure:

- `doc` — joint and link documentation
- `launch` — RViz launch files
- `stl` — OP3 STL files
- `src` — ROS node for RViz imaginary gripper joint
- `urdf` — URDF and xacro files

Agent notes:

- For custom robot modeling, start from OP3 URDF/xacro and swap meshes/inertias/joint transforms carefully.
- STL alone is only geometry; URDF/xacro must define links, joints, inertial data, and frames.

---

## `op3_gazebo`

Gazebo simulation package.

Package structure:

- `config` — ROS controller config for Gazebo
- `launch` — Gazebo simulation launch files
- `worlds` — simulation environments

---

## `op3_action_editor`

Tool for creating/editing binary action files used by `op3_action_module`.

Run:

```bash
ros2 run op3_action_editor executor.py
```

Do not run while these are running:

- `op3_manager`
- `op3_offset_tuner`
- `op3_walking_tuner`

Action file facts:

- Default action file is under `op3_action_module/data`.
- Binary action file contains 256 pages.
- Each page can store up to 7 stages/steps, `STP0` to `STP6`.
- `STP7` is the current position of connected DYNAMIXELs.
- `----` means torque is disabled or the servo is not connected/readable.
- `Next` links an action to another page.
- `Exit` links to a safe recovery page if action exits in unstable state.
- `Play Count` sets how many times the page plays.
- `PauseTime` is the pause duration for the step.
- `Time(x 8msec)` means each time unit is 8 ms.

Default action file examples:

| Page | Title | Description | Pages |
|---:|---|---|---:|
| 1 | `walki_init` | Initial standing pose | 1 |
| 2 | `hello` | Greeting | 1 |
| 3 | `thank_you` | Thank you | 1 |
| 4 | `yes` | Yes | 1 |
| 5 | `no` | No | 1 |
| 6 | `fighting` | Fighting | 1 |
| 7 | `clap` | Clap | 2 |
| 9 | `S_H_RE` | Ready for shaking hands | 1 |
| 10 | `S_H` | Shaking hands | 1 |
| 11 | `S_H_END` | Move to initial pose from shaking-hands ready pose | 1 |
| 12 | `scanning` | Looking around | 1 |
| 13 | `ceremony` | Ceremony | 1 |

Commands:

- `exit` — exit program
- `re` — refresh screen
- `b` — previous page
- `n` — next page
- `page [index]` — go to page
- `list` — list pages
- `new` — clear current page
- `copy [index]` — copy page `[index]` into current page
- `set [value]` — set selected actuator position value
- `save` — save changes; docs mention saved file `motion_4096.bin` in `op3_action_module/data`
- `play` — play current page motion
- `name` — rename current page
- `i` — insert current position `STP7` into `STP0`, shifting existing steps right
- `i [index]` — insert `STP7` into `STP[index]`, shifting from that index right
- `m [index] [index2]` — move data from `[index2]` to `[index]`
- `d [index]` — delete `STP[index]`, shifting later steps left
- `on/off` — torque on/off all connected DYNAMIXELs
- `on/off [id1] [id2] ...` — torque on/off selected IDs

Agent notes for Yuda's common issue:

- `i` is insert, not duplicate. It inserts `STP7` into `STP0` and shifts old steps.
- To duplicate an existing step without using current servo pose, use move/copy behavior carefully inside the editor or edit via exported/binary tooling if available. In the built-in command list, `copy [page]` copies a page, while `m [dst] [src]` moves step data.
- Make small incremental position/time/pause changes when testing real-robot motions.

---

## `op3_gui_demo`

GUI app for controlling OP3: module settings, walking tuner, head joint control, and action playback.

Run:

```bash
ros2 launch op3_gui_demo op3_demo.launch.py
```

Subscribed topics:

- `/robotis/status` — `std_msgs/msg/String`
- `/robotis/present_joint_ctrl_modules` — `robotis_controller_msgs/msg/JointCtrlModule`
- `/robotis/head_control/present_joint_states` — `sensor_msgs/msg/JointState`

Published topics:

- `/robotis/base/ini_pose` — `std_msgs/msg/String`
- `/robotis/enable_ctrl_module` — `std_msgs/msg/String`
- `/robotis/sync_write_item` — `std_msgs/msg/String`
- `/robotis/head_control/set_joint_states_offset` — `sensor_msgs/msg/JointState`
- `/play_sound_file` — `std_msgs/msg/String`
- `/robotis/walking/command` — `std_msgs/msg/String`
- `/robotis/walking/set_params` — `op3_walking_module_msgs/msg/WalkingParam`
- `/robotis/action/page_num` — `std_msgs/msg/Int32`

Services:

- `/robotis/get_present_joint_ctrl_modules` — `robotis_controller_msgs/msg/GetJointModule`
- `/robotis/walking/get_params` — `op3_walking_module_msgs/msg/GetWalkingParam`

Parameter:

- `/demo_config` — default `/op3_gui_demo/config/demo_config.yaml`

---

## `op3_tuner_client`

GUI node for offset and gain adjustment with `op3_manager`.

Run:

```bash
ros2 launch op3_tuner_client op3_tuner_client.launch.xml
```

Published topics:

- `/robotis/tuning_module/tuning_pose` — `std_msgs/msg/String`
- `/robotis/tuning_module/joint_offset_data` — `op3_tuning_module_msgs/msg/JointOffsetData`
- `/robotis/tuning_module/joint_gain_data` — `op3_tuning_module_msgs/msg/JointOffsetData`
- `/robotis/tuning_module/torque_enable` — `op3_tuning_module_msgs/msg/JointTorqueOnOffArray`
- `/robotis/tuning_module/command` — `std_msgs/msg/String`

Service:

- `/robotis/tuning_module/get_present_joint_offset_data` — `op3_tuning_module_msgs/msg/GetPresentJointOffsetData`

---

## `op3_offset_tuner_server`

Offset tuner server. Directly controls the robot, so do not run with `op3_manager`, `op3_action_editor`, or `op3_walking_tuner`.

Run:

```bash
ros2 launch op3_offset_tuner_server op3_offset_tuner_server.launch.xml
```

Subscribed topics:

- `/robotis/base/send_tra` — `std_msgs/msg/String`
- `/robotis/offset_tuner/joint_offset_data` — `op3_offset_tuner_msgs/msg/JointOffsetData`
- `/robotis/offset_tuner/torque_enable` — `op3_offset_tuner_msgs/msg/JointTorqueOnOffArray`
- `/robotis/offset_tuner/command` — `std_msgs/msg/String`
  - `save`: save current offset to YAML
  - `ini_pose`: take initial posture for offset tuning

Service:

- `robotis/offset_tuner/get_present_joint_offset_data` — `op3_offset_tuner_msgs/msg/GetPresentJointOffsetData`

Parameters:

- `/offset_path` — string, YAML path to save offsets
- `/robot_file_path` — string, `.robot` file path
- `/init_file_path` — string, joint initialization file path

---

## `op3_offset_tuner_client`

GUI node for adjusting offset values with `op3_offset_tuner_server`.

Published topics:

- `/robotis/offset_tuner/joint_offset_data` — `op3_offset_tuner_msgs/msg/JointOffsetData`
- `/robotis/offset_tuner/torque_enable` — `op3_offset_tuner_msgs/msg/JointTorqueOnOffArray`
- `/robotis/offset_tuner/command` — `std_msgs/msg/String`

Service:

- `/robotis/offset_tuner/get_present_joint_offset_data` — `op3_offset_tuner_msgs/msg/GetPresentJointOffsetData`

---

## Safety and debugging heuristics for agents

1. Never suggest running two direct hardware controllers simultaneously.
2. For `/dev/ttyUSB*` instability, suggest stable udev symlinks.
3. For DYNAMIXEL bulk read failures, check USB path, baud rate, OpenCR power, servo power, cable integrity, and whether another process is holding the port.
4. For ROS 2 parameter errors, check exact declared types. Use floats for documented `double` params.
5. For action tuning, change joint positions and time/pause values incrementally.
6. For custom code, put team-owned behavior in separate packages and use vendor packages behind a small motion bridge.
7. For soccer behavior, use a finite-state machine: idle/search/track/approach/kick/recover.
8. Prefer semantic commands such as `init_pose`, `kick_left`, `kick_right`, `walk_velocity`, and `head_track` over scattering vendor topic calls across many nodes.

