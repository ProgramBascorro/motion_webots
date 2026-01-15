import copy
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, String

from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
from robotis_controller_msgs.msg import JointCtrlModule


class Op3JoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__("op3_joy_teleop")

        self._axis_x = int(self.declare_parameter("axis_x", 1).value)
        self._axis_y = int(self.declare_parameter("axis_y", 0).value)
        self._axis_yaw = int(self.declare_parameter("axis_yaw", -1).value)
        self._deadman_button = int(self.declare_parameter("deadman_button", 6).value)
        self._stop_button = int(self.declare_parameter("stop_button", 1).value)
        self._init_pose_button = int(
            self.declare_parameter("init_pose_button", 2).value
        )
        self._init_pose_longpress_sec = float(
            self.declare_parameter("init_pose_longpress_sec", 1.0).value
        )
        self._require_deadman_released_for_init_pose = bool(
            self.declare_parameter("require_deadman_released_for_init_pose", True).value
        )
        self._auto_stop_before_init_pose = bool(
            self.declare_parameter("auto_stop_before_init_pose", True).value
        )

        self._max_x = float(self.declare_parameter("max_x", 0.02).value)
        self._max_y = float(self.declare_parameter("max_y", 0.015).value)
        self._max_yaw = float(self.declare_parameter("max_yaw", 0.25).value)
        self._joy_sign_x = float(self.declare_parameter("joy_sign_x", -1.0).value)
        self._joy_sign_y = float(self.declare_parameter("joy_sign_y", 1.0).value)
        self._joy_sign_yaw = float(self.declare_parameter("joy_sign_yaw", 1.0).value)
        self._deadzone = float(self.declare_parameter("deadzone", 0.05).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 30.0).value)
        self._dry_run = bool(self.declare_parameter("dry_run", False).value)
        self._joy_timeout = float(self.declare_parameter("joy_timeout", 0.5).value)

        self._smoothing_mode = str(
            self.declare_parameter("smoothing_mode", "accel_limit").value
        )
        self._accel_limit_x = float(
            self.declare_parameter("accel_limit_x", 0.06).value
        )
        self._accel_limit_y = float(
            self.declare_parameter("accel_limit_y", 0.05).value
        )
        self._accel_limit_yaw = float(
            self.declare_parameter("accel_limit_yaw", 0.8).value
        )
        self._lowpass_alpha = float(
            self.declare_parameter("lowpass_alpha", 0.25).value
        )

        self._gear_scales = list(
            self.declare_parameter("gear_scales", [0.5, 1.0, 1.5]).value
        )
        self._gear_names = list(
            self.declare_parameter("gear_names", ["slow", "normal", "fast"]).value
        )
        self._heading_hold_button = int(
            self.declare_parameter("heading_hold_button", 3).value
        )
        self._gear_longpress_sec = float(
            self.declare_parameter("gear_longpress_sec", 0.5).value
        )
        self._turbo_button = int(self.declare_parameter("turbo_button", 7).value)
        self._turbo_scale = float(self.declare_parameter("turbo_scale", 1.5).value)

        self._enable_turning = bool(
            self.declare_parameter("enable_turning", True).value
        )
        self._turn_left_button = int(
            self.declare_parameter("turn_left_button", 9).value
        )
        self._turn_right_button = int(
            self.declare_parameter("turn_right_button", 8).value
        )
        self._turn_left_axis = int(
            self.declare_parameter("turn_left_axis", -1).value
        )
        self._turn_right_axis = int(
            self.declare_parameter("turn_right_axis", -1).value
        )
        self._trigger_axis_threshold = float(
            self.declare_parameter("trigger_axis_threshold", 0.2).value
        )
        self._turn_max_yaw = float(
            self.declare_parameter("turn_max_yaw", 0.25).value
        )

        self._enable_kicks = bool(self.declare_parameter("enable_kicks", True).value)
        self._left_kick_page = int(self.declare_parameter("left_kick_page", 120).value)
        self._right_kick_page = int(self.declare_parameter("right_kick_page", 121).value)
        self._kick_mode_button = int(
            self.declare_parameter("kick_mode_button", 4).value
        )
        self._kick_mode_longpress_sec = float(
            self.declare_parameter("kick_mode_longpress_sec", 0.5).value
        )
        self._kick_mode_window_sec = float(
            self.declare_parameter("kick_mode_window_sec", 2.0).value
        )
        self._require_deadman_released_for_kick = bool(
            self.declare_parameter("require_deadman_released_for_kick", True).value
        )
        self._auto_stop_before_kick = bool(
            self.declare_parameter("auto_stop_before_kick", True).value
        )
        self._kick_apply_mode_delay_sec = float(
            self.declare_parameter("kick_apply_mode_delay_sec", 0.2).value
        )
        self._kick_timeout_sec = float(
            self.declare_parameter("kick_timeout_sec", 5.0).value
        )
        self._kick_cooldown_sec = float(
            self.declare_parameter("kick_cooldown_sec", 1.0).value
        )
        self._action_module_name = str(
            self.declare_parameter("action_module_name", "action_module").value
        )
        self._walking_module_name = str(
            self.declare_parameter("walking_module_name", "walking_module").value
        )

        self._axis_head_pan = int(self.declare_parameter("axis_head_pan", 2).value)
        self._axis_head_tilt = int(self.declare_parameter("axis_head_tilt", 3).value)
        self._head_sign_pan = float(self.declare_parameter("head_sign_pan", 1.0).value)
        self._head_sign_tilt = float(self.declare_parameter("head_sign_tilt", -1.0).value)
        self._head_pan_speed = float(self.declare_parameter("head_pan_speed", 1.2).value)
        self._head_tilt_speed = float(self.declare_parameter("head_tilt_speed", 1.0).value)
        self._head_pan_min = float(self.declare_parameter("head_pan_min", -1.8).value)
        self._head_pan_max = float(self.declare_parameter("head_pan_max", 1.8).value)
        self._head_tilt_min = float(self.declare_parameter("head_tilt_min", -0.8).value)
        self._head_tilt_max = float(self.declare_parameter("head_tilt_max", 0.8).value)
        self._head_pan_joint = str(self.declare_parameter("head_pan_joint", "head_pan").value)
        self._head_tilt_joint = str(self.declare_parameter("head_tilt_joint", "head_tilt").value)
        self._head_center_button = int(
            self.declare_parameter("head_center_button", 0).value
        )
        self._head_center_longpress_sec = float(
            self.declare_parameter("head_center_longpress_sec", 0.5).value
        )
        self._auto_enable_head_module = bool(
            self.declare_parameter("auto_enable_head_module", True).value
        )
        self._head_module_name = str(
            self.declare_parameter("head_module_name", "head_control_module").value
        )
        self._allow_direct_control_fallback = bool(
            self.declare_parameter("allow_direct_control_fallback", False).value
        )

        self._dpad_axis_x = int(self.declare_parameter("dpad_axis_x", -1).value)
        self._dpad_axis_y = int(self.declare_parameter("dpad_axis_y", -1).value)
        self._dpad_step_x = float(self.declare_parameter("dpad_step_x", 0.2).value)
        self._dpad_step_y = float(self.declare_parameter("dpad_step_y", 0.2).value)
        self._dpad_button_up = int(self.declare_parameter("dpad_button_up", -1).value)
        self._dpad_button_down = int(
            self.declare_parameter("dpad_button_down", -1).value
        )
        self._dpad_button_left = int(
            self.declare_parameter("dpad_button_left", -1).value
        )
        self._dpad_button_right = int(
            self.declare_parameter("dpad_button_right", -1).value
        )

        self._enable_pub = self.create_publisher(
            String, "/robotis/enable_ctrl_module", 10
        )
        self._command_pub = self.create_publisher(
            String, "/robotis/walking/command", 10
        )
        self._param_pub = self.create_publisher(
            WalkingParam, "/robotis/walking/set_params", 10
        )
        self._head_offset_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states_offset", 10
        )
        self._head_direct_pub = None
        if self._allow_direct_control_fallback:
            self._head_direct_pub = self.create_publisher(
                JointState, "/robotis/direct_control/set_joint_states", 10
            )
        self._joint_ctrl_pub = self.create_publisher(
            JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10
        )
        self._action_page_pub = self.create_publisher(
            Int32, "/robotis/action/page_num", 10
        )
        self._init_pose_pub = self.create_publisher(
            String, "/robotis/base/ini_pose", 10
        )

        self._param_client = self.create_client(
            GetWalkingParam, "/robotis/walking/get_params"
        )
        self._baseline_params: Optional[WalkingParam] = None
        self._baseline_request_in_flight = False

        self._deadman_active = False
        self._stop_pressed = False
        self._last_joy: Optional[Joy] = None
        self._last_log_times: Dict[str, float] = {}
        self._last_joy_time: Optional[float] = None
        self._joy_timed_out = False
        self._last_walk_update_time = time.monotonic()
        self._smoothed_x = 0.0
        self._smoothed_y = 0.0
        self._smoothed_yaw = 0.0
        self._heading_hold = False
        self._turn_src = "none"
        self._yaw_target = 0.0
        if self._gear_scales:
            self._gear_index = min(1, len(self._gear_scales) - 1)
        else:
            self._gear_index = 0
        self._turbo_active = False
        self._x_pressed = False
        self._x_press_time = 0.0
        self._x_longpress_triggered = False
        self._head_center_pressed = False
        self._head_center_press_time = 0.0
        self._head_center_triggered = False
        self._head_center_request = False
        self._init_pose_pressed = False
        self._init_pose_press_time = 0.0
        self._init_pose_triggered = False
        self._init_pose_request = False
        self._head_pan = 0.0
        self._head_tilt = 0.0
        self._head_initialized = False
        self._last_head_update_time = time.monotonic()
        self._last_head_publish_time = 0.0
        self._head_backend = "head_control"
        self._last_head_module_pub_time = 0.0

        self._kick_mode_pressed = False
        self._kick_mode_press_time = 0.0
        self._kick_mode_active = False
        self._kick_mode_until = 0.0
        self._kick_stage = "idle"
        self._kick_start_time = 0.0
        self._kick_apply_time = 0.0
        self._kick_page = 0
        self._kick_last_done = ""
        self._last_kick_time = 0.0

        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10
        )
        self.create_subscription(
            String, "/robotis/movement_done", self._movement_done_callback, 10
        )

        if not self._dry_run:
            self._enable_pub.publish(String(data="walking_module"))
        else:
            self._log_throttled("dry_startup", "dry_run enabled; not publishing startup enable")

        self.create_timer(1.0, self._try_fetch_baseline)

        if self._publish_rate <= 0.0:
            self.get_logger().warn("publish_rate <= 0; defaulting to 30.0")
            self._publish_rate = 30.0
        self.create_timer(1.0 / self._publish_rate, self._publish_loop)
        self.create_timer(2.0, self._update_head_backend)
        self._validate_gears()

    def _joy_callback(self, msg: Joy) -> None:
        self._last_joy = msg
        now = time.monotonic()
        self._last_joy_time = now
        if self._joy_timed_out:
            self._joy_timed_out = False

        deadman = self._get_button(msg, self._deadman_button)
        stop = self._get_button(msg, self._stop_button)
        x_pressed = self._get_button(msg, self._heading_hold_button)
        head_center_pressed = self._get_button(msg, self._head_center_button)
        init_pose_pressed = self._get_button(msg, self._init_pose_button)
        kick_mode_pressed = self._get_button(msg, self._kick_mode_button)
        self._turbo_active = self._get_button(msg, self._turbo_button)

        if stop and not self._stop_pressed:
            self._publish_stop_zero()

        if deadman and not self._deadman_active:
            self._publish_enable_and_start()

        if not deadman and self._deadman_active:
            self._publish_stop_zero()

        self._update_heading_hold_and_gear(x_pressed, now)
        self._update_head_center_longpress(head_center_pressed, now)
        self._update_init_pose_longpress(init_pose_pressed, now)
        self._update_kick_mode(kick_mode_pressed, now)
        self._handle_kick_trigger(now)

        self._deadman_active = deadman
        self._stop_pressed = stop

    def _try_fetch_baseline(self) -> None:
        if self._baseline_params is not None or self._baseline_request_in_flight:
            return
        if not self._param_client.service_is_ready():
            self._log_throttled(
                "wait_params",
                "Waiting for /robotis/walking/get_params service...",
                1.0,
            )
            return

        request = GetWalkingParam.Request()
        request.get_param = True
        future = self._param_client.call_async(request)
        self._baseline_request_in_flight = True
        future.add_done_callback(self._handle_baseline_response)

    def _handle_baseline_response(self, future) -> None:
        self._baseline_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to get walking params: {exc}")
            return
        if response is None:
            self.get_logger().warn("Walking params response was empty")
            return
        self._baseline_params = response.parameters
        self.get_logger().info("Loaded baseline walking parameters")

    def _publish_loop(self) -> None:
        self._handle_joy_timeout()
        self._update_kick_state()
        self._handle_init_pose_request()
        if not self._is_kick_blocking():
            self._publish_walking()
            self._publish_head()
        self._log_debug_state()

    def _publish_walking(self) -> None:
        if not self._deadman_active:
            return

        if self._baseline_params is None:
            self._log_throttled(
                "no_baseline",
                "Baseline walking params not available yet; skipping publish",
                1.0,
            )
            return

        raw_x = self._get_axis(self._last_joy, self._axis_x)
        raw_y = self._get_axis(self._last_joy, self._axis_y)
        raw_yaw = self._get_axis(self._last_joy, self._axis_yaw)

        target_x = self._joy_sign_x * self._max_x * self._apply_deadzone(raw_x)
        target_y = self._joy_sign_y * self._max_y * self._apply_deadzone(raw_y)
        target_yaw = self._joy_sign_yaw * self._max_yaw * self._apply_deadzone(raw_yaw)

        if self._enable_turning:
            target_yaw, self._turn_src = self._get_turning_yaw()
            if self._turn_src != "none":
                target_yaw *= self._turn_max_yaw
            else:
                self._turn_src = "none"
        else:
            self._turn_src = "none"

        dpad_x, dpad_y = self._get_dpad_inputs()
        if dpad_x != 0.0:
            target_x += dpad_x * self._max_x * self._dpad_step_x
        if dpad_y != 0.0:
            target_y += dpad_y * self._max_y * self._dpad_step_y

        if self._heading_hold:
            target_yaw = 0.0
            self._turn_src = "none"

        self._yaw_target = target_yaw

        scale = self._get_current_scale()
        target_x *= scale
        target_y *= scale
        target_yaw *= scale

        now = time.monotonic()
        dt = max(0.0, now - self._last_walk_update_time)
        self._last_walk_update_time = now

        x, y, yaw = self._apply_smoothing(target_x, target_y, target_yaw, dt)
        self._smoothed_x = x
        self._smoothed_y = y
        self._smoothed_yaw = yaw

        if self._dry_run:
            self._log_throttled(
                "dry_run",
                f"dry_run x={x:.4f} y={y:.4f} yaw={yaw:.4f}",
                1.0,
            )
            return

        params = copy.deepcopy(self._baseline_params)
        params.x_move_amplitude = x
        params.y_move_amplitude = y
        params.angle_move_amplitude = yaw
        self._param_pub.publish(params)

    def _publish_head(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_head_update_time)
        self._last_head_update_time = now

        raw_pan = self._get_axis(self._last_joy, self._axis_head_pan)
        raw_tilt = self._get_axis(self._last_joy, self._axis_head_tilt)
        axis_pan = self._apply_deadzone(raw_pan)
        axis_tilt = self._apply_deadzone(raw_tilt)
        if self._head_center_request:
            self._head_pan = 0.0
            self._head_tilt = 0.0
            self._head_center_request = False
            if self._dry_run:
                self._log_throttled(
                    "head_center",
                    "dry_run head center pan=0.0 tilt=0.0",
                    1.0,
                )
                return
            self._publish_head_center(self._head_pan, self._head_tilt)
            return

        if axis_pan == 0.0 and axis_tilt == 0.0:
            return

        delta_pan = self._head_sign_pan * axis_pan * self._head_pan_speed * dt
        delta_tilt = self._head_sign_tilt * axis_tilt * self._head_tilt_speed * dt
        if delta_pan == 0.0 and delta_tilt == 0.0:
            return

        self._head_pan = self._clamp(
            self._head_pan + delta_pan, self._head_pan_min, self._head_pan_max
        )
        self._head_tilt = self._clamp(
            self._head_tilt + delta_tilt, self._head_tilt_min, self._head_tilt_max
        )

        if self._dry_run:
            self._log_throttled(
                "head_dry",
                (
                    f"dry_run head pan={self._head_pan:.3f} "
                    f"tilt={self._head_tilt:.3f} backend={self._head_backend}"
                ),
                1.0,
            )
            return

        if self._head_backend == "head_control":
            self._publish_head_offset(delta_pan, delta_tilt)
        elif self._allow_direct_control_fallback:
            if now - self._last_head_publish_time >= 1.0 / 15.0:
                self._publish_head_direct(self._head_pan, self._head_tilt)
                self._last_head_publish_time = now

        self._log_throttled(
            "head_state",
            (
                f"head pan={self._head_pan:.3f} tilt={self._head_tilt:.3f} "
                f"backend={self._head_backend}"
            ),
            1.0,
        )

    def _publish_enable_and_start(self) -> None:
        if self._dry_run:
            self._log_throttled("dry_start", "dry_run start requested", 1.0)
            return
        self._enable_pub.publish(String(data=self._walking_module_name))
        self._publish_head_module_assignment()
        self._command_pub.publish(String(data="start"))

    def _publish_stop_zero(self) -> None:
        if self._dry_run:
            self._log_throttled("dry_stop", "dry_run stop requested", 1.0)
            return
        self._command_pub.publish(String(data="stop"))
        self._publish_zero_params()
        self._deadman_active = False

    def _publish_zero_params(self) -> None:
        if self._baseline_params is None:
            self._log_throttled(
                "no_baseline_zero",
                "Baseline walking params not available; cannot zero params",
                1.0,
            )
            return
        params = copy.deepcopy(self._baseline_params)
        params.x_move_amplitude = 0.0
        params.y_move_amplitude = 0.0
        params.angle_move_amplitude = 0.0
        self._param_pub.publish(params)

    def _handle_init_pose_request(self) -> None:
        if not self._init_pose_request:
            return
        if self._is_kick_blocking():
            return
        if self._require_deadman_released_for_init_pose and self._deadman_active:
            self._log_throttled(
                "init_pose_deadman",
                "Release deadman before init pose",
                1.0,
            )
            self._init_pose_request = False
            return
        if self._auto_stop_before_init_pose:
            self._publish_stop_zero()
        if self._dry_run:
            self._log_throttled("init_pose", "dry_run init pose requested", 1.0)
        else:
            self._init_pose_pub.publish(String(data="ini_pose"))
        self._init_pose_request = False

    def _publish_head_offset(self, delta_pan: float, delta_tilt: float) -> None:
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [delta_pan, delta_tilt]
        self._head_offset_pub.publish(msg)

    def _publish_head_direct(self, pan: float, tilt: float) -> None:
        if self._head_direct_pub is None:
            return
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        self._head_direct_pub.publish(msg)

    def _publish_head_center(self, pan: float, tilt: float) -> None:
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        if self._head_backend == "head_control":
            self._head_offset_pub.publish(msg)
        elif self._allow_direct_control_fallback:
            if self._head_direct_pub is not None:
                self._head_direct_pub.publish(msg)

    def _publish_head_module_assignment(self) -> None:
        if not self._auto_enable_head_module:
            return
        if self._dry_run:
            self._log_throttled(
                "head_module",
                "dry_run head module assignment requested",
                1.0,
            )
            return
        now = time.monotonic()
        if now - self._last_head_module_pub_time < 1.5:
            return
        msg = JointCtrlModule()
        msg.joint_name = [self._head_pan_joint, self._head_tilt_joint]
        msg.module_name = [self._head_module_name, self._head_module_name]
        self._joint_ctrl_pub.publish(msg)
        self._last_head_module_pub_time = now
        self._log_throttled(
            "head_module_pub",
            (
                f"Published head module assignment to {self._head_module_name} for "
                f"{self._head_pan_joint}, {self._head_tilt_joint}"
            ),
            2.0,
        )

    def _update_head_backend(self) -> None:
        topics = self.get_topic_names_and_types()
        has_head_control = any(
            name == "/robotis/head_control/set_joint_states_offset" for name, _ in topics
        )
        if has_head_control:
            self._head_backend = "head_control"
        elif self._allow_direct_control_fallback:
            self._head_backend = "direct_control"
        else:
            self._head_backend = "head_control"
        if self._head_backend == "head_control":
            self._publish_head_module_assignment()

    def _joint_states_callback(self, msg: JointState) -> None:
        if self._head_initialized:
            return
        pan, tilt, found = self._extract_head_positions(msg)
        if found:
            self._head_pan = pan
            self._head_tilt = tilt
            self._head_initialized = True
            if self._joint_state_sub is not None:
                self.destroy_subscription(self._joint_state_sub)
                self._joint_state_sub = None

    def _extract_head_positions(self, msg: JointState) -> Tuple[float, float, bool]:
        if not msg.name or not msg.position:
            return 0.0, 0.0, False
        pan = None
        tilt = None
        for name, position in zip(msg.name, msg.position):
            if name == self._head_pan_joint:
                pan = float(position)
            elif name == self._head_tilt_joint:
                tilt = float(position)
        if pan is None or tilt is None:
            return 0.0, 0.0, False
        return pan, tilt, True

    def _get_axis(self, msg: Optional[Joy], index: int) -> float:
        if msg is None or index < 0 or index >= len(msg.axes):
            return 0.0
        return float(msg.axes[index])

    def _get_button(self, msg: Optional[Joy], index: int) -> bool:
        if msg is None or index < 0 or index >= len(msg.buttons):
            return False
        return bool(msg.buttons[index])

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def _log_throttled(self, key: str, message: str, period_sec: float = 1.0) -> None:
        now = time.monotonic()
        last_time = self._last_log_times.get(key, 0.0)
        if now - last_time >= period_sec:
            self._last_log_times[key] = now
            self.get_logger().info(message)

    def _handle_joy_timeout(self) -> None:
        if self._joy_timeout <= 0.0:
            return
        now = time.monotonic()
        if self._last_joy_time is None:
            return
        if now - self._last_joy_time > self._joy_timeout:
            if not self._joy_timed_out:
                self._joy_timed_out = True
                self._publish_stop_zero()
                self._smoothed_x = 0.0
                self._smoothed_y = 0.0
                self._smoothed_yaw = 0.0

    def _update_heading_hold_and_gear(self, pressed: bool, now: float) -> None:
        if pressed and not self._x_pressed:
            self._x_pressed = True
            self._x_press_time = now
            self._x_longpress_triggered = False

        if pressed and self._x_pressed and not self._x_longpress_triggered:
            if now - self._x_press_time >= self._gear_longpress_sec:
                self._cycle_gear()
                self._x_longpress_triggered = True

        if not pressed and self._x_pressed:
            if not self._x_longpress_triggered:
                self._heading_hold = not self._heading_hold
            self._x_pressed = False

    def _update_head_center_longpress(self, pressed: bool, now: float) -> None:
        if pressed and not self._head_center_pressed:
            self._head_center_pressed = True
            self._head_center_press_time = now
            self._head_center_triggered = False

        if pressed and self._head_center_pressed and not self._head_center_triggered:
            if now - self._head_center_press_time >= self._head_center_longpress_sec:
                self._head_center_request = True
                self._head_center_triggered = True

        if not pressed and self._head_center_pressed:
            self._head_center_pressed = False

    def _update_init_pose_longpress(self, pressed: bool, now: float) -> None:
        if self._init_pose_button < 0:
            return
        if pressed and not self._init_pose_pressed:
            self._init_pose_pressed = True
            self._init_pose_press_time = now
            self._init_pose_triggered = False

        if pressed and self._init_pose_pressed and not self._init_pose_triggered:
            if now - self._init_pose_press_time >= self._init_pose_longpress_sec:
                self._init_pose_request = True
                self._init_pose_triggered = True

        if not pressed and self._init_pose_pressed:
            self._init_pose_pressed = False

    def _apply_smoothing(
        self, target_x: float, target_y: float, target_yaw: float, dt: float
    ) -> Tuple[float, float, float]:
        if dt <= 0.0:
            return self._smoothed_x, self._smoothed_y, self._smoothed_yaw

        if self._smoothing_mode == "lowpass":
            alpha = max(0.0, min(1.0, self._lowpass_alpha))
            x = self._smoothed_x + alpha * (target_x - self._smoothed_x)
            y = self._smoothed_y + alpha * (target_y - self._smoothed_y)
            yaw = self._smoothed_yaw + alpha * (target_yaw - self._smoothed_yaw)
            return x, y, yaw

        x = self._limit_rate(self._smoothed_x, target_x, self._accel_limit_x, dt)
        y = self._limit_rate(self._smoothed_y, target_y, self._accel_limit_y, dt)
        yaw = self._limit_rate(self._smoothed_yaw, target_yaw, self._accel_limit_yaw, dt)
        return x, y, yaw

    @staticmethod
    def _limit_rate(current: float, target: float, accel_limit: float, dt: float) -> float:
        max_delta = max(0.0, accel_limit) * dt
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + max_delta * (1.0 if delta > 0.0 else -1.0)

    def _get_current_scale(self) -> float:
        if not self._gear_scales:
            return 1.0
        gear_scale = float(self._gear_scales[self._gear_index])
        if self._turbo_active:
            return gear_scale * self._turbo_scale
        return gear_scale

    def _cycle_gear(self) -> None:
        if not self._gear_scales:
            return
        self._gear_index = (self._gear_index + 1) % len(self._gear_scales)

    def _validate_gears(self) -> None:
        if not self._gear_scales:
            return
        if not self._gear_names or len(self._gear_names) != len(self._gear_scales):
            self._gear_names = [f"gear_{i}" for i in range(len(self._gear_scales))]

    def _get_dpad_inputs(self) -> Tuple[float, float]:
        dpad_x = 0.0
        dpad_y = 0.0
        if self._deadman_active:
            if self._dpad_axis_x >= 0:
                dpad_x = self._get_axis(self._last_joy, self._dpad_axis_x)
                dpad_x = max(-1.0, min(1.0, dpad_x))
            if self._dpad_axis_y >= 0:
                dpad_y = self._get_axis(self._last_joy, self._dpad_axis_y)
                dpad_y = max(-1.0, min(1.0, dpad_y))

            if self._dpad_button_left >= 0 and self._get_button(
                self._last_joy, self._dpad_button_left
            ):
                dpad_x = -1.0
            if self._dpad_button_right >= 0 and self._get_button(
                self._last_joy, self._dpad_button_right
            ):
                dpad_x = 1.0
            if self._dpad_button_up >= 0 and self._get_button(
                self._last_joy, self._dpad_button_up
            ):
                dpad_y = 1.0
            if self._dpad_button_down >= 0 and self._get_button(
                self._last_joy, self._dpad_button_down
            ):
                dpad_y = -1.0
        return dpad_x, dpad_y

    def _log_debug_state(self) -> None:
        if self._gear_names and self._gear_scales:
            gear_name = self._gear_names[self._gear_index]
            gear_scale = self._gear_scales[self._gear_index]
        else:
            gear_name = "n/a"
            gear_scale = 1.0
        now = time.monotonic()
        kick_remaining = 0.0
        if self._kick_mode_active:
            kick_remaining = max(0.0, self._kick_mode_until - now)
        kick_elapsed = 0.0
        if self._kick_stage != "idle":
            kick_elapsed = max(0.0, now - self._kick_start_time)
        self._log_throttled(
            "debug",
            (
                f"walking={self._deadman_active} timed_out={self._joy_timed_out} "
                f"gear={gear_name} scale={gear_scale:.2f} turbo={self._turbo_active} "
                f"heading_hold={self._heading_hold} "
                f"turn_src={self._turn_src} yaw_target={self._yaw_target:.3f} yaw={self._smoothed_yaw:.3f} "
                f"x={self._smoothed_x:.3f} y={self._smoothed_y:.3f} "
                f"head_pan={self._head_pan:.3f} head_tilt={self._head_tilt:.3f} "
                f"head_module_auto={self._auto_enable_head_module} backend={self._head_backend} "
                f"kick_mode={self._kick_mode_active} kick_remain={kick_remaining:.1f} "
                f"kick_stage={self._kick_stage} kick_elapsed={kick_elapsed:.1f} "
                f"kick_page={self._kick_page} last_done={self._kick_last_done}"
            ),
            1.0,
        )

    def _update_kick_mode(self, pressed: bool, now: float) -> None:
        if not self._enable_kicks:
            self._kick_mode_active = False
            return
        if pressed and not self._kick_mode_pressed:
            self._kick_mode_pressed = True
            self._kick_mode_press_time = now
        if pressed and self._kick_mode_pressed:
            if not self._kick_mode_active and now - self._kick_mode_press_time >= self._kick_mode_longpress_sec:
                self._kick_mode_active = True
                self._kick_mode_until = now + self._kick_mode_window_sec
        if not pressed and self._kick_mode_pressed:
            self._kick_mode_pressed = False

        if self._kick_mode_active and now >= self._kick_mode_until:
            self._kick_mode_active = False

    def _handle_kick_trigger(self, now: float) -> None:
        if not self._enable_kicks:
            return
        if not self._kick_mode_active:
            return
        if self._kick_stage != "idle":
            return
        if self._require_deadman_released_for_kick and self._deadman_active:
            return
        if now - self._last_kick_time < self._kick_cooldown_sec:
            return

        kick_side = self._get_kick_side()
        if kick_side == "none":
            return

        if self._auto_stop_before_kick:
            self._publish_stop_zero()

        if not self._dry_run:
            self._enable_pub.publish(String(data=self._action_module_name))
        else:
            self._log_throttled("kick_mode", "dry_run action module enable", 1.0)

        self._kick_stage = "switching"
        self._kick_start_time = now
        self._kick_apply_time = now + self._kick_apply_mode_delay_sec
        self._kick_page = self._left_kick_page if kick_side == "left" else self._right_kick_page
        self._kick_mode_active = False

    def _update_kick_state(self) -> None:
        if not self._enable_kicks:
            return
        now = time.monotonic()
        if self._kick_mode_active and now >= self._kick_mode_until:
            self._kick_mode_active = False
        if self._kick_stage == "idle":
            return
        if self._kick_stage == "switching" and now >= self._kick_apply_time:
            if not self._dry_run:
                self._action_page_pub.publish(Int32(data=self._kick_page))
            else:
                self._log_throttled("kick_page", f"dry_run page_num {self._kick_page}", 1.0)
            self._kick_stage = "waiting"

        if self._kick_stage == "waiting":
            if now - self._kick_start_time >= self._kick_timeout_sec:
                self._kick_last_done = "timeout"
                self._finish_kick()

    def _movement_done_callback(self, msg: String) -> None:
        if self._kick_stage == "idle":
            return
        if msg.data in {"action", "action_failed"}:
            self._kick_last_done = msg.data
            self._finish_kick()

    def _finish_kick(self) -> None:
        if not self._dry_run:
            self._enable_pub.publish(String(data=self._walking_module_name))
        else:
            self._log_throttled("kick_restore", "dry_run walking module restore", 1.0)
        self._publish_head_module_assignment()
        self._kick_stage = "idle"
        self._last_kick_time = time.monotonic()

    def _get_kick_side(self) -> str:
        left_button = (
            self._turn_left_button >= 0
            and self._get_button(self._last_joy, self._turn_left_button)
        )
        right_button = (
            self._turn_right_button >= 0
            and self._get_button(self._last_joy, self._turn_right_button)
        )
        if left_button or right_button:
            if left_button and right_button:
                return "none"
            return "left" if left_button else "right"

        left_axis = 0.0
        right_axis = 0.0
        if self._turn_left_axis >= 0:
            left_axis = self._get_axis(self._last_joy, self._turn_left_axis)
        if self._turn_right_axis >= 0:
            right_axis = self._get_axis(self._last_joy, self._turn_right_axis)

        left_active = abs(left_axis) > self._trigger_axis_threshold
        right_active = abs(right_axis) > self._trigger_axis_threshold
        if left_active or right_active:
            if left_active and right_active:
                return "none"
            return "left" if left_active else "right"

        return "none"

    def _is_kick_blocking(self) -> bool:
        return self._kick_mode_active or self._kick_stage != "idle"

    def _get_turning_yaw(self) -> Tuple[float, str]:
        if not self._deadman_active:
            return 0.0, "none"

        left_button = (
            self._turn_left_button >= 0
            and self._get_button(self._last_joy, self._turn_left_button)
        )
        right_button = (
            self._turn_right_button >= 0
            and self._get_button(self._last_joy, self._turn_right_button)
        )
        if left_button or right_button:
            if left_button and right_button:
                return 0.0, "buttons"
            return (-1.0 if left_button else 1.0), "buttons"

        left_axis = 0.0
        right_axis = 0.0
        if self._turn_left_axis >= 0:
            left_axis = self._get_axis(self._last_joy, self._turn_left_axis)
        if self._turn_right_axis >= 0:
            right_axis = self._get_axis(self._last_joy, self._turn_right_axis)

        left_active = abs(left_axis) > self._trigger_axis_threshold
        right_active = abs(right_axis) > self._trigger_axis_threshold
        if left_active or right_active:
            if left_active and right_active:
                return 0.0, "axes"
            return (-1.0 if left_active else 1.0), "axes"

        return 0.0, "none"


def main() -> None:
    rclpy.init()
    node = Op3JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
