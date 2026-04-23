# op3_joy_teleop

Gamepad teleop bridge for OP3 walking module (ROS 2 Humble).

## Build

```bash
cd /home/farhan/Projects/motion_webots_farhan_coba
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch op3_joy_teleop op3_joy_teleop.launch.py
```

## Quick verification

```bash
ros2 topic echo /joy --once
ros2 topic list | grep robotis
ros2 topic echo /robotis/walking/command
ros2 topic echo /robotis/head_control/set_joint_states_offset
ros2 topic echo /robotis/walking/set_params | grep angle_move_amplitude -n
ros2 topic pub --once /op3_joy_teleop/command std_msgs/msg/String "{data: refresh_params}"
ros2 topic echo /op3_joy_teleop/status
ros2 topic info /robotis/action/page_num
ros2 topic echo /robotis/movement_done
```

## Head/Camera Control

Use the right stick (default) to pan/tilt the head. Center with the configured button.

Verify topics and mappings:

```bash
ros2 topic list | grep head_control
ros2 topic echo /joint_states --once
ros2 topic echo /joy --once
```

If `/robotis/head_control/set_joint_states_offset` is missing, the node falls back to
`/robotis/direct_control/set_joint_states` for absolute head positions.

The node auto-assigns head joints to `head_control_module` at startup and after enabling
walking. Set `auto_enable_head_module: false` in the config to disable this behavior.

## Controls (default mapping)

- Walk: left stick axes 0/1 + deadman L1 (button 6)
- Stop: disabled by default (set `stop_button` to enable)
- Init pose: long-press button 2 (configurable); requires deadman released by default
- Head: right stick axes 2/3
- Heading hold: tap X (button 3) to toggle (yaw forced to 0)
- Gear cycle: long-press X (>=0.5s) cycles slow/normal/fast
- Turbo: hold R1 (button 7)
- Recenter head: long-press A (button 0)
- Turn left/right: L2/R2 (buttons 8/9 by default; can be axes if configured)
- Kick mode: long-press Y (button 4), then L2 = left kick, R2 = right kick

Turning is disabled by default (`axis_yaw: -1`). Set `axis_yaw` if you want yaw control.
Turning from triggers is controlled by `enable_turning` and `turn_*` parameters.

Init pose publishes `ini_pose` on `/robotis/base/ini_pose`. Adjust
`init_pose_button`/`init_pose_longpress_sec` as needed; set `init_pose_button: -1`
to disable.

With `stop_button` disabled, release the deadman or wait for `joy_timeout` to stop.

Direct control is not used for turning; yaw always goes through `angle_move_amplitude`.
If you need head direct control in a special sim, set `allow_direct_control_fallback: true`
and ensure the `op3_direct_control_module` is enabled.

## Runtime Teleop Commands

The node accepts simple string commands on `/op3_joy_teleop/command` and publishes
JSON status snapshots on `/op3_joy_teleop/status`. This lets Bascorro Studio refresh
the teleop walking baseline after you tune or load a Walking version.

Useful commands:

- `refresh_params`: reload the baseline from `/robotis/walking/get_params`
- `start`: enable `walking_module` and publish walking `start`
- `stop`: publish walking `stop` and zero x/y/yaw amplitudes
- `zero_params`: publish zero x/y/yaw amplitudes while keeping the current baseline
- `heading_hold_toggle`, `heading_hold_on`, `heading_hold_off`
- `gear_next`, or `gear_slow`, `gear_normal`, `gear_fast`

Optional config:

- `auto_refresh_baseline_sec`: periodically reload baseline params; `0.0` disables it
- `refresh_baseline_button`: joystick button index for manual baseline reload; `-1` disables it

## Kick Mode

Kick mode avoids input conflicts by repurposing L2/R2 only while it is active.
Long-press Y (>=0.5s) to enter kick mode for ~2 seconds, then press L2 (left kick) or
R2 (right kick). Kicks require deadman released and switch to `action_module` temporarily.

Diagnostics:

```bash
ros2 topic info /robotis/action/page_num
ros2 topic echo /robotis/movement_done
```

## Troubleshooting

- If walking does not respond, verify axis/button mappings in `config/op3_joy_teleop.yaml`.
- Check gamepad permissions (e.g., `/dev/input/js0` access) if `/joy` is empty.
- Ensure the walking module is enabled (`/robotis/enable_ctrl_module` publishes `walking_module`).
- If the robot does not move, confirm `/robotis/walking/get_params` is available and OP3 is in a walking-ready state.
- If head control is inverted or unresponsive, confirm `axis_head_pan/axis_head_tilt` and `head_sign_*` settings.
