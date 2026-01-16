import copy
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32, String

from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
from robotis_controller_msgs.msg import JointCtrlModule


class BallLocalizer(Node):
    STATE_SCAN = "scan"
    STATE_SEARCH = "search"
    STATE_APPROACH = "approach"
    STATE_PRE_KICK = "pre_kick"
    STATE_KICKING = "kicking"
    STATE_ARRIVED = "arrived"

    def __init__(self) -> None:
        super().__init__("op3_ball_localizer")

        self._ball_topic = str(
            self.declare_parameter("ball_topic", "/vision/yolo/balls").value
        )
        self._output_frame = str(
            self.declare_parameter("output_frame", "base").value
        )
        self._publish_rate = float(
            self.declare_parameter("publish_rate", 10.0).value
        )
        self._status_log_enabled = bool(
            self.declare_parameter("status_log_enabled", True).value
        )
        self._status_log_period_sec = float(
            self.declare_parameter("status_log_period_sec", 1.0).value
        )
        self._scan_pan_min = float(
            self.declare_parameter("scan_pan_min", -1.2).value
        )
        self._scan_pan_max = float(
            self.declare_parameter("scan_pan_max", 1.2).value
        )
        self._scan_tilt_down = float(
            self.declare_parameter("scan_tilt_down", -0.6).value
        )
        self._scan_tilt_up = float(
            self.declare_parameter("scan_tilt_up", -0.25).value
        )
        self._scan_step = float(self.declare_parameter("scan_step", 0.3).value)
        self._scan_dwell_sec = float(
            self.declare_parameter("scan_dwell_sec", 0.4).value
        )
        self._scan_timeout = float(
            self.declare_parameter("scan_timeout", 6.0).value
        )
        self._lost_timeout = float(
            self.declare_parameter("lost_timeout", 0.8).value
        )
        self._arrive_distance = float(
            self.declare_parameter("arrive_distance", 0.25).value
        )
        self._arrive_hysteresis = float(
            self.declare_parameter("arrive_hysteresis", 0.05).value
        )
        self._pre_kick_distance = float(
            self.declare_parameter("pre_kick_distance", 0.22).value
        )
        self._kick_distance = float(
            self.declare_parameter("kick_distance", 0.18).value
        )
        self._approach_x_gain = float(
            self.declare_parameter("approach_x_gain", 0.04).value
        )
        self._approach_max_x = float(
            self.declare_parameter("approach_max_x", 0.02).value
        )
        self._approach_yaw_gain = float(
            self.declare_parameter("approach_yaw_gain", 1.2).value
        )
        self._approach_max_yaw = float(
            self.declare_parameter("approach_max_yaw", 0.25).value
        )
        self._pre_kick_max_x = float(
            self.declare_parameter("pre_kick_max_x", self._approach_max_x).value
        )
        self._pre_kick_max_yaw = float(
            self.declare_parameter("pre_kick_max_yaw", self._approach_max_yaw).value
        )
        self._approach_yaw_only_threshold = float(
            self.declare_parameter("approach_yaw_only_threshold", 0.6).value
        )
        self._min_forward_x = float(
            self.declare_parameter("min_forward_x", 0.05).value
        )
        self._search_yaw = float(self.declare_parameter("search_yaw", 0.2).value)
        self._search_reverse_period = float(
            self.declare_parameter("search_reverse_period", 0.0).value
        )
        self._require_ball_in_front = bool(
            self.declare_parameter("require_ball_in_front", True).value
        )
        self._min_valid_x = float(
            self.declare_parameter("min_valid_x", 0.01).value
        )
        self._head_pan_joint = str(
            self.declare_parameter("head_pan_joint", "head_pan").value
        )
        self._head_tilt_joint = str(
            self.declare_parameter("head_tilt_joint", "head_tilt").value
        )
        self._head_pan_min = float(
            self.declare_parameter("head_pan_min", -1.8).value
        )
        self._head_pan_max = float(
            self.declare_parameter("head_pan_max", 1.8).value
        )
        self._head_tilt_min = float(
            self.declare_parameter("head_tilt_min", -0.8).value
        )
        self._head_tilt_max = float(
            self.declare_parameter("head_tilt_max", 0.8).value
        )
        self._head_track_enable = bool(
            self.declare_parameter("head_track_enable", True).value
        )
        self._head_track_pan_gain = float(
            self.declare_parameter("head_track_pan_gain", 1.0).value
        )
        self._head_track_tilt_mode = str(
            self.declare_parameter("head_track_tilt_mode", "geometry").value
        ).lower()
        self._head_track_tilt = float(
            self.declare_parameter("head_track_tilt", self._scan_tilt_down).value
        )
        self._pre_kick_tilt = float(
            self.declare_parameter("pre_kick_tilt", self._scan_tilt_down).value
        )
        self._head_track_tilt_far = float(
            self.declare_parameter("head_track_tilt_far", self._head_track_tilt).value
        )
        self._head_track_tilt_near = float(
            self.declare_parameter("head_track_tilt_near", self._pre_kick_tilt).value
        )
        self._head_track_tilt_far_distance = float(
            self.declare_parameter("head_track_tilt_far_distance", 0.45).value
        )
        self._head_track_tilt_near_distance = float(
            self.declare_parameter("head_track_tilt_near_distance", 0.15).value
        )
        self._head_track_tilt_max_step = float(
            self.declare_parameter("head_track_tilt_max_step", 0.2).value
        )
        self._camera_height = float(
            self.declare_parameter("camera_height", 0.46).value
        )
        self._camera_forward_offset = float(
            self.declare_parameter("camera_forward_offset", 0.0).value
        )
        self._head_track_tilt_offset = float(
            self.declare_parameter("head_track_tilt_offset", 0.0).value
        )
        self._head_track_min_distance = float(
            self.declare_parameter("head_track_min_distance", 0.05).value
        )
        self._head_track_deadzone = float(
            self.declare_parameter("head_track_deadzone", 0.05).value
        )
        self._head_track_max_step = float(
            self.declare_parameter("head_track_max_step", 0.4).value
        )
        self._kick_enable = bool(
            self.declare_parameter("kick_enable", True).value
        )
        self._kick_left_page = int(
            self.declare_parameter("kick_left_page", 120).value
        )
        self._kick_right_page = int(
            self.declare_parameter("kick_right_page", 121).value
        )
        self._kick_bearing_deadzone = float(
            self.declare_parameter("kick_bearing_deadzone", 0.08).value
        )
        self._kick_center_prefer = str(
            self.declare_parameter("kick_center_prefer", "right").value
        ).lower()
        self._kick_hold_still_sec = float(
            self.declare_parameter("kick_hold_still_sec", 0.4).value
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
        self._auto_enable_head_module = bool(
            self.declare_parameter("auto_enable_head_module", True).value
        )
        self._head_module_name = str(
            self.declare_parameter("head_module_name", "head_control_module").value
        )
        self._auto_enable_walking_module = bool(
            self.declare_parameter("auto_enable_walking_module", True).value
        )
        self._walking_module_name = str(
            self.declare_parameter("walking_module_name", "walking_module").value
        )
        self._dry_run = bool(self.declare_parameter("dry_run", False).value)

        self._scan_step = abs(self._scan_step)
        if self._scan_pan_min > self._scan_pan_max:
            self._scan_pan_min, self._scan_pan_max = (
                self._scan_pan_max,
                self._scan_pan_min,
            )
        if self._scan_tilt_down > self._scan_tilt_up:
            self._scan_tilt_down, self._scan_tilt_up = (
                self._scan_tilt_up,
                self._scan_tilt_down,
            )
        if self._kick_distance > self._pre_kick_distance:
            self.get_logger().warn(
                "kick_distance > pre_kick_distance; clamping pre_kick_distance"
            )
            self._pre_kick_distance = self._kick_distance
        if self._kick_bearing_deadzone < 0.0:
            self._kick_bearing_deadzone = abs(self._kick_bearing_deadzone)
        self._pre_kick_max_x = abs(self._pre_kick_max_x)
        self._pre_kick_max_yaw = abs(self._pre_kick_max_yaw)
        if self._head_track_tilt_mode not in {"geometry", "distance", "fixed"}:
            self.get_logger().warn(
                f"Unknown head_track_tilt_mode '{self._head_track_tilt_mode}', using geometry"
            )
            self._head_track_tilt_mode = "geometry"
        self._head_track_tilt_far_distance = abs(self._head_track_tilt_far_distance)
        self._head_track_tilt_near_distance = abs(self._head_track_tilt_near_distance)
        if self._head_track_tilt_far_distance < self._head_track_tilt_near_distance:
            self.get_logger().warn(
                "head_track_tilt_far_distance < head_track_tilt_near_distance; swapping"
            )
            self._head_track_tilt_far_distance, self._head_track_tilt_near_distance = (
                self._head_track_tilt_near_distance,
                self._head_track_tilt_far_distance,
            )
        self._head_track_tilt_max_step = abs(self._head_track_tilt_max_step)
        self._camera_height = abs(self._camera_height)
        self._head_track_min_distance = max(
            0.01, abs(self._head_track_min_distance)
        )

        self._head_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states", 10
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
        self._action_page_pub = self.create_publisher(
            Int32, "/robotis/action/page_num", 10
        )
        self._joint_ctrl_pub = self.create_publisher(
            JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10
        )

        self._param_client = self.create_client(
            GetWalkingParam, "/robotis/walking/get_params"
        )
        self._baseline_params: Optional[WalkingParam] = None
        self._baseline_request_in_flight = False

        self._state = self.STATE_SCAN
        self._scan_start_time = time.monotonic()
        self._search_start_time = 0.0
        self._search_direction = 1.0
        self._last_search_flip_time = 0.0
        self._walking_active = False
        self._head_center_sent = False
        self._head_track_last_pan = 0.0
        self._head_track_last_tilt = self._head_track_tilt_far
        self._pre_kick_start_time = 0.0
        self._kick_ready_since: Optional[float] = None
        self._kick_stage = "idle"
        self._kick_start_time = 0.0
        self._kick_apply_time = 0.0
        self._kick_selected_page = 0
        self._last_kick_time = 0.0
        self._last_ball_distance = 0.0
        self._last_ball_bearing = 0.0

        self._last_ball_time: Optional[float] = None
        self._last_ball_point: Optional[Tuple[float, float, float]] = None

        self._last_scan_step_time = 0.0
        self._scan_pan = self._scan_pan_min
        self._scan_tilt = self._scan_tilt_down
        self._scan_direction = 1.0
        self._scan_tilt_down_active = True

        self._last_log_times = {}
        self._last_head_module_pub_time = 0.0

        self.create_subscription(
            PointCloud2, self._ball_topic, self._ball_callback, 10
        )
        self.create_subscription(
            String, "/robotis/movement_done", self._movement_done_callback, 10
        )

        if self._auto_enable_head_module:
            self._publish_head_module_assignment()

        self.create_timer(1.0, self._try_fetch_baseline)

        if self._publish_rate <= 0.0:
            self.get_logger().warn("publish_rate <= 0; defaulting to 10.0")
            self._publish_rate = 10.0
        self.create_timer(1.0 / self._publish_rate, self._control_loop)

    def _ball_callback(self, msg: PointCloud2) -> None:
        if self._output_frame and msg.header.frame_id:
            if msg.header.frame_id != self._output_frame:
                self._log_throttled(
                    "frame_mismatch",
                    (
                        f"Ball frame {msg.header.frame_id} != {self._output_frame}; "
                        "update yolo output_frame"
                    ),
                    2.0,
                )
                return

        best_point = None
        best_dist = None
        for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            if self._require_ball_in_front and x < self._min_valid_x:
                continue
            dist = math.hypot(x, y)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_point = (float(x), float(y), float(z))

        if best_point is None:
            return

        self._last_ball_point = best_point
        self._last_ball_time = time.monotonic()

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

    def _control_loop(self) -> None:
        now = time.monotonic()
        ball_point = self._get_latest_ball(now)
        distance = None
        bearing = None
        if ball_point is not None:
            distance = math.hypot(ball_point[0], ball_point[1])
            bearing = math.atan2(ball_point[1], ball_point[0])
            self._last_ball_distance = distance
            self._last_ball_bearing = bearing

        self._update_state(now, distance, bearing)

        if self._state == self.STATE_SCAN:
            self._stop_walking()
            self._update_head_scan(now)
        elif self._state == self.STATE_SEARCH:
            self._start_walking()
            self._update_head_scan(now)
            self._publish_search_params(now)
        elif self._state == self.STATE_APPROACH:
            self._start_walking()
            if distance is not None and bearing is not None:
                self._update_head_on_ball(bearing, distance, ball_point)
                self._publish_approach_params(distance, bearing)
        elif self._state == self.STATE_PRE_KICK:
            self._start_walking()
            if distance is not None and bearing is not None:
                self._update_head_on_ball(
                    bearing, distance, ball_point, min_tilt=self._pre_kick_tilt
                )
                self._publish_approach_params(
                    distance,
                    bearing,
                    max_x=self._pre_kick_max_x,
                    max_yaw=self._pre_kick_max_yaw,
                )
        elif self._state == self.STATE_KICKING:
            self._stop_walking()
            self._update_kick_state(now)
        elif self._state == self.STATE_ARRIVED:
            self._stop_walking()
            if distance is not None and bearing is not None:
                self._update_head_on_ball(bearing, distance, ball_point)

        self._log_status(now, distance, bearing, ball_point)

    def _update_state(
        self, now: float, distance: Optional[float], bearing: Optional[float]
    ) -> None:
        ball_visible = distance is not None and bearing is not None

        if self._state == self.STATE_SCAN:
            if ball_visible:
                self._set_state(self.STATE_APPROACH)
            elif now - self._scan_start_time >= self._scan_timeout:
                self._set_state(self.STATE_SEARCH)
        elif self._state == self.STATE_SEARCH:
            if ball_visible:
                self._set_state(self.STATE_APPROACH)
        elif self._state == self.STATE_APPROACH:
            if not ball_visible:
                self._set_state(self.STATE_SEARCH)
                return
            if self._kick_enable and distance <= self._pre_kick_distance:
                self._set_state(self.STATE_PRE_KICK)
            elif (not self._kick_enable) and distance <= self._arrive_distance:
                self._set_state(self.STATE_ARRIVED)
        elif self._state == self.STATE_PRE_KICK:
            if not ball_visible:
                self._set_state(self.STATE_SEARCH)
                return
            if not self._kick_enable:
                self._set_state(self.STATE_ARRIVED)
                return
            if distance > self._pre_kick_distance + self._arrive_hysteresis:
                self._set_state(self.STATE_APPROACH)
                return
            if distance <= self._kick_distance:
                if self._kick_ready_since is None:
                    self._kick_ready_since = now
                if (
                    now - self._kick_ready_since >= self._kick_hold_still_sec
                    and now - self._last_kick_time >= self._kick_cooldown_sec
                ):
                    self._set_state(self.STATE_KICKING)
            else:
                self._kick_ready_since = None
        elif self._state == self.STATE_KICKING:
            return
        elif self._state == self.STATE_ARRIVED:
            if not ball_visible:
                self._set_state(self.STATE_SCAN)
                return
            if distance >= self._arrive_distance + self._arrive_hysteresis:
                self._set_state(self.STATE_APPROACH)

    def _set_state(self, new_state: str) -> None:
        if new_state == self._state:
            return
        self.get_logger().info(f"State {self._state} -> {new_state}")
        self._state = new_state
        now = time.monotonic()
        if new_state == self.STATE_SCAN:
            self._scan_start_time = now
            self._reset_scan_pattern()
            self._head_center_sent = False
        elif new_state == self.STATE_SEARCH:
            self._search_start_time = now
            self._last_search_flip_time = now
            self._search_direction = 1.0
            self._reset_scan_pattern()
            self._head_center_sent = False
        elif new_state == self.STATE_APPROACH:
            self._head_center_sent = False
        elif new_state == self.STATE_PRE_KICK:
            self._head_center_sent = False
            self._pre_kick_start_time = now
            self._kick_ready_since = None
        elif new_state == self.STATE_KICKING:
            self._head_center_sent = False
            self._start_kick_sequence(now)
        elif new_state == self.STATE_ARRIVED:
            self._head_center_sent = False

    def _get_latest_ball(
        self, now: float
    ) -> Optional[Tuple[float, float, float]]:
        if self._last_ball_time is None or self._last_ball_point is None:
            return None
        if now - self._last_ball_time > self._lost_timeout:
            return None
        return self._last_ball_point

    def _update_head_scan(self, now: float) -> None:
        if now - self._last_scan_step_time < self._scan_dwell_sec:
            return
        self._last_scan_step_time = now

        next_pan = self._scan_pan + self._scan_step * self._scan_direction
        next_tilt = (
            self._scan_tilt_down
            if self._scan_tilt_down_active
            else self._scan_tilt_up
        )

        if self._scan_direction > 0 and next_pan >= self._scan_pan_max:
            next_pan = self._scan_pan_max
            self._scan_direction = -1.0
            self._scan_tilt_down_active = False
            next_tilt = self._scan_tilt_up
        elif self._scan_direction < 0 and next_pan <= self._scan_pan_min:
            next_pan = self._scan_pan_min
            self._scan_direction = 1.0
            self._scan_tilt_down_active = True
            next_tilt = self._scan_tilt_down

        self._scan_pan = next_pan
        self._scan_tilt = next_tilt
        self._publish_head_absolute(next_pan, next_tilt)

    def _publish_search_params(self, now: float) -> None:
        if self._search_reverse_period > 0.0:
            if now - self._last_search_flip_time >= self._search_reverse_period:
                self._search_direction *= -1.0
                self._last_search_flip_time = now
        yaw = self._search_yaw * self._search_direction
        self._publish_walking_params(0.0, yaw)

    def _publish_approach_params(
        self,
        distance: float,
        bearing: float,
        max_x: Optional[float] = None,
        max_yaw: Optional[float] = None,
    ) -> None:
        max_x = self._approach_max_x if max_x is None else max_x
        max_yaw = self._approach_max_yaw if max_yaw is None else max_yaw
        yaw = self._clamp(
            bearing * self._approach_yaw_gain,
            -max_yaw,
            max_yaw,
        )

        x_cmd = 0.0
        if abs(bearing) < self._approach_yaw_only_threshold:
            x_cmd = min(distance * self._approach_x_gain, max_x)

        if self._require_ball_in_front and self._last_ball_point is not None:
            if self._last_ball_point[0] < self._min_forward_x:
                x_cmd = 0.0

        self._publish_walking_params(x_cmd, yaw)

    def _publish_walking_params(self, x: float, yaw: float) -> None:
        if self._baseline_params is None:
            self._log_throttled(
                "no_baseline",
                "Baseline walking params not available yet; skipping",
                1.0,
            )
            return
        if self._dry_run:
            self._log_throttled(
                "dry_run",
                f"dry_run walking x={x:.3f} yaw={yaw:.3f}",
                1.0,
            )
            return
        params = copy.deepcopy(self._baseline_params)
        params.x_move_amplitude = x
        params.y_move_amplitude = 0.0
        params.angle_move_amplitude = yaw
        self._param_pub.publish(params)

    def _start_walking(self) -> None:
        if self._walking_active:
            return
        if self._baseline_params is None:
            self._log_throttled(
                "no_baseline_start",
                "Baseline walking params not available; waiting to start walking",
                1.0,
            )
            return
        if self._auto_enable_walking_module and not self._dry_run:
            self._enable_pub.publish(String(data=self._walking_module_name))
        if not self._dry_run:
            self._command_pub.publish(String(data="start"))
        self._walking_active = True

    def _stop_walking(self) -> None:
        if not self._walking_active:
            return
        if not self._dry_run:
            self._command_pub.publish(String(data="stop"))
            self._publish_zero_params()
        self._walking_active = False

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

    def _publish_head_absolute(self, pan: float, tilt: float) -> None:
        pan = self._clamp(pan, self._head_pan_min, self._head_pan_max)
        tilt = self._clamp(tilt, self._head_tilt_min, self._head_tilt_max)
        if self._dry_run:
            self._log_throttled(
                "head_dry",
                f"dry_run head pan={pan:.3f} tilt={tilt:.3f}",
                1.0,
            )
            return
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        self._head_pub.publish(msg)
        self._publish_head_module_assignment()

    def _publish_head_center_once(self) -> None:
        if self._head_center_sent:
            return
        self._publish_head_absolute(0.0, 0.0)
        self._head_center_sent = True

    def _update_head_on_ball(
        self,
        bearing: float,
        distance: Optional[float] = None,
        ball_point: Optional[Tuple[float, float, float]] = None,
        min_tilt: Optional[float] = None,
    ) -> None:
        if self._head_track_enable:
            tilt = self._compute_track_tilt(distance, ball_point)
            if min_tilt is not None:
                tilt = min(tilt, min_tilt)
            self._publish_head_track(bearing, tilt)
        else:
            self._publish_head_center_once()

    def _publish_head_track(self, bearing: float, tilt: float) -> None:
        target_pan = bearing * self._head_track_pan_gain
        if abs(target_pan) < self._head_track_deadzone:
            target_pan = 0.0
        target_pan = self._clamp(target_pan, self._head_pan_min, self._head_pan_max)
        target_tilt = self._clamp(tilt, self._head_tilt_min, self._head_tilt_max)

        if self._head_track_max_step > 0.0:
            delta = target_pan - self._head_track_last_pan
            max_step = self._head_track_max_step
            if abs(delta) > max_step:
                target_pan = self._head_track_last_pan + max_step * (
                    1.0 if delta > 0.0 else -1.0
                )
        if self._head_track_tilt_max_step > 0.0:
            delta_tilt = target_tilt - self._head_track_last_tilt
            max_step = self._head_track_tilt_max_step
            if abs(delta_tilt) > max_step:
                target_tilt = self._head_track_last_tilt + max_step * (
                    1.0 if delta_tilt > 0.0 else -1.0
                )

        if (
            abs(target_pan - self._head_track_last_pan) < 1e-3
            and abs(target_tilt - self._head_track_last_tilt) < 1e-3
        ):
            return

        self._head_track_last_pan = target_pan
        self._head_track_last_tilt = target_tilt
        self._publish_head_absolute(target_pan, target_tilt)

    def _compute_track_tilt(
        self,
        distance: Optional[float],
        ball_point: Optional[Tuple[float, float, float]],
    ) -> float:
        if self._head_track_tilt_mode == "geometry":
            return self._compute_geometry_tilt(ball_point)
        if self._head_track_tilt_mode == "fixed":
            return self._head_track_tilt_far
        return self._compute_distance_tilt(distance)

    def _compute_distance_tilt(self, distance: Optional[float]) -> float:
        if distance is None:
            return self._head_track_tilt_far
        near_dist = self._head_track_tilt_near_distance
        far_dist = self._head_track_tilt_far_distance
        if far_dist <= near_dist:
            return self._head_track_tilt_near
        if distance <= near_dist:
            return self._head_track_tilt_near
        if distance >= far_dist:
            return self._head_track_tilt_far
        ratio = (distance - near_dist) / (far_dist - near_dist)
        return self._head_track_tilt_near + ratio * (
            self._head_track_tilt_far - self._head_track_tilt_near
        )

    def _compute_geometry_tilt(
        self, ball_point: Optional[Tuple[float, float, float]]
    ) -> float:
        if ball_point is None:
            return self._head_track_tilt_far
        dx = ball_point[0] - self._camera_forward_offset
        dy = ball_point[1]
        horizontal = math.hypot(dx, dy)
        horizontal = max(horizontal, self._head_track_min_distance)
        angle_down = math.atan2(self._camera_height, horizontal)
        return self._head_track_tilt_offset - angle_down

    def _publish_head_module_assignment(self) -> None:
        if not self._auto_enable_head_module or self._dry_run:
            return
        now = time.monotonic()
        if now - self._last_head_module_pub_time < 1.5:
            return
        msg = JointCtrlModule()
        msg.joint_name = [self._head_pan_joint, self._head_tilt_joint]
        msg.module_name = [self._head_module_name, self._head_module_name]
        self._joint_ctrl_pub.publish(msg)
        self._last_head_module_pub_time = now

    def _reset_scan_pattern(self) -> None:
        self._scan_pan = self._scan_pan_min
        self._scan_tilt = self._scan_tilt_down
        self._scan_direction = 1.0
        self._scan_tilt_down_active = True
        self._last_scan_step_time = 0.0

    def _start_kick_sequence(self, now: float) -> None:
        if not self._kick_enable:
            return
        if now - self._last_kick_time < self._kick_cooldown_sec:
            self._set_state(self.STATE_PRE_KICK)
            return
        if self._kick_center_prefer not in {"left", "right"}:
            self._kick_center_prefer = "right"

        self._stop_walking()
        if not self._dry_run:
            self._enable_pub.publish(String(data=self._action_module_name))
        else:
            self._log_throttled("kick_mode", "dry_run action module enable", 1.0)

        self._kick_stage = "switching"
        self._kick_start_time = now
        self._kick_apply_time = now + self._kick_apply_mode_delay_sec
        self._kick_selected_page = self._select_kick_page(self._last_ball_bearing)

    def _update_kick_state(self, now: float) -> None:
        if self._kick_stage == "idle":
            return
        if self._kick_stage == "switching" and now >= self._kick_apply_time:
            if not self._dry_run:
                self._action_page_pub.publish(Int32(data=self._kick_selected_page))
            else:
                self._log_throttled(
                    "kick_page",
                    f"dry_run page_num {self._kick_selected_page}",
                    1.0,
                )
            self._kick_stage = "waiting"

        if self._kick_stage == "waiting":
            if now - self._kick_start_time >= self._kick_timeout_sec:
                self._finish_kick("timeout")

    def _movement_done_callback(self, msg: String) -> None:
        if self._state != self.STATE_KICKING:
            return
        if self._kick_stage == "idle":
            return
        if msg.data in {"action", "action_failed"}:
            self._finish_kick(msg.data)

    def _finish_kick(self, result: str) -> None:
        if not self._dry_run:
            self._enable_pub.publish(String(data=self._walking_module_name))
        else:
            self._log_throttled("kick_restore", "dry_run walking module restore", 1.0)
        self._publish_head_module_assignment()
        self._kick_stage = "idle"
        self._last_kick_time = time.monotonic()
        self._walking_active = False
        self._set_state(self.STATE_SCAN)
        self.get_logger().info(f"Kick finished: {result}")

    def _select_kick_page(self, bearing: float) -> int:
        if abs(bearing) < self._kick_bearing_deadzone:
            return (
                self._kick_left_page
                if self._kick_center_prefer == "left"
                else self._kick_right_page
            )
        return self._kick_left_page if bearing > 0.0 else self._kick_right_page

    def _log_throttled(self, key: str, message: str, period_sec: float = 1.0) -> None:
        now = time.monotonic()
        last_time = self._last_log_times.get(key, 0.0)
        if now - last_time >= period_sec:
            self._last_log_times[key] = now
            self.get_logger().info(message)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def _log_status(
        self,
        now: float,
        distance: Optional[float],
        bearing: Optional[float],
        ball_point: Optional[Tuple[float, float, float]],
    ) -> None:
        if not self._status_log_enabled:
            return
        if self._status_log_period_sec <= 0.0:
            return
        if distance is None or bearing is None or ball_point is None:
            message = f"state={self._state} ball=none"
        else:
            message = (
                f"state={self._state} dist={distance:.3f} bearing={bearing:.2f} "
                f"ball=({ball_point[0]:.3f},{ball_point[1]:.3f}) "
                f"arrive={self._arrive_distance:.2f} "
                f"pre_kick={self._pre_kick_distance:.2f} kick={self._kick_distance:.2f} "
                f"head_pan={self._head_track_last_pan:.2f} head_tilt={self._head_track_last_tilt:.2f}"
            )
        self._log_throttled("status", message, self._status_log_period_sec)


def main() -> None:
    rclpy.init()
    node = BallLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
