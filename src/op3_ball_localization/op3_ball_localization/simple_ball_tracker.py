#!/usr/bin/env python3
"""
Simplified ball tracker using 2D bounding boxes + distance estimation.
Works with /vision/yolo/detections and /vision/yolo/ball_center.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from soccer_msgs.msg import BoundingBoxes
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
import copy


class SimpleBallTracker(Node):
    STATE_SCAN = "scan"
    STATE_APPROACH = "approach"
    STATE_ARRIVED = "arrived"

    def __init__(self) -> None:
        super().__init__("simple_ball_tracker")

        # Parameters
        self._detections_topic = str(
            self.declare_parameter("detections_topic", "/vision/yolo/detections").value
        )
        self._ball_center_topic = str(
            self.declare_parameter("ball_center_topic", "/vision/yolo/ball_center").value
        )
        self._publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        # Scan parameters (slower than default)
        self._scan_pan_min = float(self.declare_parameter("scan_pan_min", -1.2).value)
        self._scan_pan_max = float(self.declare_parameter("scan_pan_max", 1.2).value)
        self._scan_tilt_down = float(self.declare_parameter("scan_tilt_down", -0.6).value)
        self._scan_tilt_up = float(self.declare_parameter("scan_tilt_up", -0.25).value)
        self._scan_step = float(self.declare_parameter("scan_step", 0.2).value)  # Slower
        self._scan_dwell_sec = float(self.declare_parameter("scan_dwell_sec", 0.8).value)  # Longer

        # Detection parameters
        self._lost_timeout = float(self.declare_parameter("lost_timeout", 1.5).value)  # Longer

        # Distance estimation (from bbox size)
        self._ball_real_diameter_m = float(self.declare_parameter("ball_real_diameter_m", 0.065).value)
        self._camera_focal_length_px = float(self.declare_parameter("camera_focal_length_px", 500.0).value)
        self._min_bbox_area = float(self.declare_parameter("min_bbox_area", 100.0).value)

        # Approach parameters
        self._arrive_distance = float(self.declare_parameter("arrive_distance", 0.25).value)
        self._approach_x_gain = float(self.declare_parameter("approach_x_gain", 0.05).value)
        self._approach_max_x = float(self.declare_parameter("approach_max_x", 0.02).value)
        self._approach_yaw_gain = float(self.declare_parameter("approach_yaw_gain", 1.5).value)
        self._approach_max_yaw = float(self.declare_parameter("approach_max_yaw", 0.3).value)

        # Head control
        self._head_pan_joint = str(self.declare_parameter("head_pan_joint", "head_pan").value)
        self._head_tilt_joint = str(self.declare_parameter("head_tilt_joint", "head_tilt").value)
        self._head_track_enable = bool(self.declare_parameter("head_track_enable", True).value)
        self._head_track_pan_gain = float(self.declare_parameter("head_track_pan_gain", 1.2).value)
        self._head_track_tilt_fixed = float(self.declare_parameter("head_track_tilt_fixed", -0.5).value)

        # Walking module
        self._auto_enable_walking = bool(self.declare_parameter("auto_enable_walking", True).value)
        self._walking_module_name = str(self.declare_parameter("walking_module_name", "walking_module").value)
        self._head_module_name = str(self.declare_parameter("head_module_name", "head_control_module").value)

        # Publishers
        self._head_pub = self.create_publisher(JointState, "/robotis/head_control/set_joint_states", 10)
        self._enable_pub = self.create_publisher(String, "/robotis/enable_ctrl_module", 10)
        self._command_pub = self.create_publisher(String, "/robotis/walking/command", 10)
        self._param_pub = self.create_publisher(WalkingParam, "/robotis/walking/set_params", 10)

        # Subscribers
        self.create_subscription(BoundingBoxes, self._detections_topic, self._detections_callback, 10)
        self.create_subscription(PointStamped, self._ball_center_topic, self._ball_center_callback, 10)

        # Walking params service
        self._param_client = self.create_client(GetWalkingParam, "/robotis/walking/get_params")
        self._baseline_params: Optional[WalkingParam] = None

        # State
        self._state = self.STATE_SCAN
        self._last_ball_time: Optional[float] = None
        self._last_ball_distance: Optional[float] = None
        self._last_ball_bearing: Optional[float] = None  # From center pixel
        self._last_ball_pixel: Optional[Tuple[float, float]] = None
        self._last_ball_bbox_area: Optional[float] = None

        # Scan state
        self._scan_pan = self._scan_pan_min
        self._scan_tilt = self._scan_tilt_down
        self._scan_direction = 1.0
        self._scan_tilt_down_active = True
        self._last_scan_step_time = 0.0

        # Walking state
        self._walking_active = False
        self._last_walking_enable_time = 0.0

        # Timers
        self.create_timer(1.0, self._try_fetch_baseline)
        self.create_timer(1.0 / self._publish_rate, self._control_loop)

        if self._auto_enable_walking:
            self._enable_walking_module()

        self.get_logger().info("Simple Ball Tracker initialized (2D + distance estimation)")
        self.get_logger().info(f"  Detections: {self._detections_topic}")
        self.get_logger().info(f"  Ball center: {self._ball_center_topic}")

    def _detections_callback(self, msg: BoundingBoxes) -> None:
        """Process bounding box detections and estimate distance."""
        import time

        # Find ball detection
        best_ball = None
        best_area = 0.0

        for bbox in msg.bounding_boxes:
            if bbox.class_id == "ball":
                area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)
                if area > self._min_bbox_area and area > best_area:
                    best_area = area
                    best_ball = bbox

        if best_ball is None:
            return

        # Estimate distance from bbox size
        bbox_height = best_ball.ymax - best_ball.ymin
        if bbox_height > 0:
            # Distance = (real_size * focal_length) / pixel_size
            distance = (self._ball_real_diameter_m * self._camera_focal_length_px) / bbox_height
            distance = max(0.1, min(5.0, distance))  # Clamp to reasonable range

            self._last_ball_distance = distance
            self._last_ball_bbox_area = best_area
            self._last_ball_time = time.monotonic()

    def _ball_center_callback(self, msg: PointStamped) -> None:
        """Process ball center for bearing calculation."""
        x_norm = msg.point.x  # 0-1 normalized
        y_norm = msg.point.y

        if math.isnan(x_norm) or math.isnan(y_norm):
            return

        # Convert normalized coords to bearing
        # x_norm: 0=left, 0.5=center, 1=right
        # Bearing: negative=left, 0=center, positive=right
        x_centered = x_norm - 0.5  # -0.5 to 0.5

        # Estimate bearing (assuming ~60deg FOV)
        fov_rad = math.radians(60.0)
        bearing = x_centered * fov_rad

        self._last_ball_bearing = bearing
        self._last_ball_pixel = (x_norm, y_norm)

    def _try_fetch_baseline(self) -> None:
        """Fetch baseline walking parameters."""
        if self._baseline_params is not None:
            return
        if not self._param_client.service_is_ready():
            return

        request = GetWalkingParam.Request()
        request.get_param = True
        future = self._param_client.call_async(request)
        future.add_done_callback(self._handle_baseline_response)

    def _handle_baseline_response(self, future) -> None:
        try:
            response = future.result()
            if response:
                self._baseline_params = response.parameters
                self.get_logger().info("Loaded baseline walking parameters")
        except Exception as e:
            self.get_logger().warn(f"Failed to get walking params: {e}")

    def _control_loop(self) -> None:
        """Main control loop."""
        import time
        now = time.monotonic()

        # Check if ball is visible
        ball_visible = (
            self._last_ball_time is not None and
            (now - self._last_ball_time) < self._lost_timeout and
            self._last_ball_distance is not None and
            self._last_ball_bearing is not None
        )

        # State machine
        if self._state == self.STATE_SCAN:
            self._stop_walking()
            self._update_head_scan(now)

            if ball_visible:
                self._set_state(self.STATE_APPROACH)

        elif self._state == self.STATE_APPROACH:
            if not ball_visible:
                self._set_state(self.STATE_SCAN)
                return

            self._start_walking()
            self._update_head_track()
            self._publish_approach_params()

            if self._last_ball_distance <= self._arrive_distance:
                self._set_state(self.STATE_ARRIVED)

        elif self._state == self.STATE_ARRIVED:
            self._stop_walking()

            if not ball_visible:
                self._set_state(self.STATE_SCAN)
            elif self._last_ball_distance > self._arrive_distance + 0.05:
                self._set_state(self.STATE_APPROACH)

        # Status logging
        if ball_visible:
            self.get_logger().info(
                f"State={self._state} dist={self._last_ball_distance:.2f}m "
                f"bearing={math.degrees(self._last_ball_bearing):.1f}deg "
                f"area={self._last_ball_bbox_area:.0f}px",
                throttle_duration_sec=2.0
            )

    def _set_state(self, new_state: str) -> None:
        """Change state."""
        if new_state == self._state:
            return
        self.get_logger().info(f"State {self._state} -> {new_state}")
        self._state = new_state

        import time
        if new_state == self.STATE_SCAN:
            self._reset_scan_pattern()
            self._last_scan_step_time = time.monotonic()

    def _update_head_scan(self, now: float) -> None:
        """Update scanning head movement."""
        if now - self._last_scan_step_time < self._scan_dwell_sec:
            return
        self._last_scan_step_time = now

        next_pan = self._scan_pan + self._scan_step * self._scan_direction
        next_tilt = self._scan_tilt_down if self._scan_tilt_down_active else self._scan_tilt_up

        # Check bounds
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
        self._publish_head(next_pan, next_tilt)

    def _update_head_track(self) -> None:
        """Track ball with head."""
        if not self._head_track_enable or self._last_ball_bearing is None:
            return

        pan = self._last_ball_bearing * self._head_track_pan_gain
        tilt = self._head_track_tilt_fixed

        self._publish_head(pan, tilt)

    def _reset_scan_pattern(self) -> None:
        """Reset scan to start position."""
        self._scan_pan = self._scan_pan_min
        self._scan_tilt = self._scan_tilt_down
        self._scan_direction = 1.0
        self._scan_tilt_down_active = True

    def _publish_approach_params(self) -> None:
        """Publish walking parameters for approaching ball."""
        if self._baseline_params is None:
            return

        if self._last_ball_distance is None or self._last_ball_bearing is None:
            return

        # Calculate yaw command
        yaw = max(-self._approach_max_yaw, min(self._approach_max_yaw,
                  self._last_ball_bearing * self._approach_yaw_gain))

        # Calculate forward speed (slower when far, faster when close)
        x_cmd = min(self._last_ball_distance * self._approach_x_gain, self._approach_max_x)

        # Only move forward if roughly aligned
        if abs(self._last_ball_bearing) > 0.5:  # ~30 degrees
            x_cmd = 0.0

        params = copy.deepcopy(self._baseline_params)
        params.x_move_amplitude = x_cmd
        params.y_move_amplitude = 0.0
        params.angle_move_amplitude = yaw
        self._param_pub.publish(params)

    def _publish_head(self, pan: float, tilt: float) -> None:
        """Publish head joint positions."""
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        self._head_pub.publish(msg)

        # Ensure head module is enabled
        import time
        now = time.monotonic()
        if now - self._last_walking_enable_time > 2.0:
            from robotis_controller_msgs.msg import JointCtrlModule
            ctrl_msg = JointCtrlModule()
            ctrl_msg.joint_name = [self._head_pan_joint, self._head_tilt_joint]
            ctrl_msg.module_name = [self._head_module_name, self._head_module_name]
            # Note: would need publisher for this

    def _start_walking(self) -> None:
        """Start walking."""
        if self._walking_active:
            return
        self._command_pub.publish(String(data="start"))
        self._walking_active = True

    def _stop_walking(self) -> None:
        """Stop walking."""
        if not self._walking_active:
            return
        self._command_pub.publish(String(data="stop"))
        if self._baseline_params:
            params = copy.deepcopy(self._baseline_params)
            params.x_move_amplitude = 0.0
            params.y_move_amplitude = 0.0
            params.angle_move_amplitude = 0.0
            self._param_pub.publish(params)
        self._walking_active = False

    def _enable_walking_module(self) -> None:
        """Enable walking module."""
        import time
        self._enable_pub.publish(String(data=self._walking_module_name))
        self._last_walking_enable_time = time.monotonic()


def main():
    rclpy.init()
    node = SimpleBallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
