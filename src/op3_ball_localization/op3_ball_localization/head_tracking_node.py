#!/usr/bin/env python3
"""
Head-only ball tracking node for the OpenVINO YOLO stack.

Uses /vision/yolo/ball_center directly and follows the original OP3 head
tracking convention: the node publishes head joint offsets that drive the ball
toward the center of the camera image.
"""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotis_controller_msgs.msg import JointCtrlModule


class HeadTrackingNode(Node):
    """Head-only ball tracker - no walking or kicking."""

    def __init__(self) -> None:
        super().__init__("head_tracking_node")

        self._ball_center_topic = str(
            self.declare_parameter("ball_center_topic", "/vision/yolo/ball_center").value
        )

        # PID gains copied from the original OP3 ball tracker defaults.
        self._p_gain = float(self.declare_parameter("p_gain", 0.30).value)
        self._i_gain = float(self.declare_parameter("i_gain", 0.0).value)
        self._d_gain = float(self.declare_parameter("d_gain", 0.045).value)

        # Deadzone around image center — don't react to YOLO jitter when the
        # ball is essentially centered already.
        self._center_deadzone = float(
            self.declare_parameter("center_deadzone", 0.10).value
        )
        # EMA smoothing on ball center. 1.0 = no filtering.
        self._input_smoothing = float(
            self.declare_parameter("input_smoothing", 0.4).value
        )

        self._control_rate = float(self.declare_parameter("control_rate", 20.0).value)
        self._lost_timeout = float(self.declare_parameter("lost_timeout", 0.8).value)
        self._scan_enabled = bool(self.declare_parameter("scan_enabled", True).value)
        self._scan_period_sec = float(self.declare_parameter("scan_period_sec", 2.5).value)
        self._scan_pan_rad = float(self.declare_parameter("scan_pan_rad", 0.7).value)
        self._scan_tilt_forward_rad = float(
            self.declare_parameter("scan_tilt_forward_rad", -0.10).value
        )
        self._scan_tilt_down_rad = float(
            self.declare_parameter("scan_tilt_down_rad", -0.35).value
        )
        self._head_enable_retry_sec = float(
            self.declare_parameter("head_enable_retry_sec", 1.5).value
        )
        self._head_enable_boot_sec = float(
            self.declare_parameter("head_enable_boot_sec", 5.0).value
        )

        # Original OP3 tracker FOV values.
        self._fov_width_rad = math.radians(
            float(self.declare_parameter("fov_width_deg", 35.2).value)
        )
        self._fov_height_rad = math.radians(
            float(self.declare_parameter("fov_height_deg", 21.6).value)
        )
        self._min_command_rad = math.radians(
            float(self.declare_parameter("min_command_deg", 1.0).value)
        )
        self._max_offset_pan_rad = abs(
            float(self.declare_parameter("max_offset_pan_rad", 0.25).value)
        )
        self._max_offset_tilt_rad = abs(
            float(self.declare_parameter("max_offset_tilt_rad", 0.20).value)
        )

        self._pan_sign = float(self.declare_parameter("pan_sign", 1.0).value)
        self._tilt_sign = float(self.declare_parameter("tilt_sign", -1.0).value)

        # When False the node starts paused; a "start" on /ball_tracker/command
        # activates it. Used in Demo mode so the head only moves after the user
        # presses the button to start soccer mode.
        self._active = bool(self.declare_parameter("auto_start", True).value)

        self._head_pan_joint = str(
            self.declare_parameter("head_pan_joint", "head_pan").value
        )
        self._head_tilt_joint = str(
            self.declare_parameter("head_tilt_joint", "head_tilt").value
        )
        self._head_module_name = str(
            self.declare_parameter("head_module_name", "head_control_module").value
        )

        self._head_offset_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states_offset", 10
        )
        self._head_abs_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states", 10
        )
        self._enable_pub = self.create_publisher(
            String, "/robotis/enable_ctrl_module", 10
        )
        self._joint_ctrl_pub = self.create_publisher(
            JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10
        )

        self.create_subscription(
            PointStamped,
            self._ball_center_topic,
            self._ball_center_callback,
            10,
        )
        self.create_subscription(
            String,
            "/ball_tracker/command",
            self._tracker_command_callback,
            10,
        )

        self._last_ball_time: Optional[float] = None
        self._last_ball_x: Optional[float] = None
        self._last_ball_y: Optional[float] = None
        self._last_x_norm: float = 0.0
        self._last_y_norm: float = 0.0
        self._prev_pan_error = 0.0
        self._prev_tilt_error = 0.0
        self._pan_error_sum = 0.0
        self._tilt_error_sum = 0.0
        self._last_control_time = time.monotonic()
        self._last_head_enable_time = 0.0
        self._joint_ctrl_modules_sent = False
        self._scan_active = False
        self._last_scan_cmd_time = 0.0
        self._scan_idx = 0

        # Scan sweep: forward-left, forward-right, down-right, down-left.
        # Down positions use slightly narrower pan so the camera looks between
        # the feet — helps find balls rolling up close.
        p = self._scan_pan_rad
        fw = self._scan_tilt_forward_rad
        dn = self._scan_tilt_down_rad
        self._scan_positions = [
            (-p, fw),
            (+p, fw),
            (+p * 0.7, dn),
            (-p * 0.7, dn),
        ]

        self.create_timer(1.0 / self._control_rate, self._control_loop)
        if self._active:
            self._ensure_head_module_enabled(force=True)

        self.get_logger().info(
            f"Head Tracking Node initialized (auto_start={self._active})"
        )
        self.get_logger().info(f"  Ball center topic: {self._ball_center_topic}")
        self.get_logger().info(
            f"  PID gains: P={self._p_gain:.3f}, I={self._i_gain:.3f}, D={self._d_gain:.3f}"
        )
        self.get_logger().info(f"  Scan enabled: {self._scan_enabled}")

    def _tracker_command_callback(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == "start" and not self._active:
            self._active = True
            self._reset_pid()
            self._last_ball_x = None
            self._last_ball_y = None
            self._last_ball_time = None
            self._last_head_enable_time = 0.0
            self._joint_ctrl_modules_sent = False
            self._ensure_head_module_enabled(force=True)
            self.get_logger().info("[CMD] activated by start command")
        elif cmd == "stop" and self._active:
            self._active = False
            self._reset_pid()
            self.get_logger().info("[CMD] deactivated by stop command")

    def _ball_center_callback(self, msg: PointStamped) -> None:
        """Track the latest normalized YOLO center point."""
        x_norm = float(msg.point.x)
        y_norm = float(msg.point.y)

        if math.isnan(x_norm) or math.isnan(y_norm):
            return

        # /vision/yolo/ball_center is normalized to [0, 1]. Convert to the same
        # [-1, 1] image coordinate convention used by the original OP3 tracker:
        # (-1, -1) = top-left, (+1, +1) = bottom-right.
        self._last_x_norm = x_norm
        self._last_y_norm = y_norm
        raw_x = (x_norm * 2.0) - 1.0
        raw_y = (y_norm * 2.0) - 1.0

        a = max(0.0, min(1.0, self._input_smoothing))
        if self._last_ball_x is None or self._last_ball_y is None:
            self._last_ball_x = raw_x
            self._last_ball_y = raw_y
        else:
            self._last_ball_x = a * raw_x + (1.0 - a) * self._last_ball_x
            self._last_ball_y = a * raw_y + (1.0 - a) * self._last_ball_y
        self._last_ball_time = time.monotonic()

    def _control_loop(self) -> None:
        if not self._active:
            return

        now = time.monotonic()

        ball_visible = (
            self._last_ball_time is not None
            and self._last_ball_x is not None
            and self._last_ball_y is not None
            and (now - self._last_ball_time) < self._lost_timeout
        )

        if ball_visible:
            self._track_ball(now)
        else:
            self._reset_pid()
            self._ensure_head_module_enabled(now)  # retry only when idle
            if self._scan_enabled:
                self._ensure_scan_mode(now)

    def _track_ball(self, now: float) -> None:
        if self._last_ball_x is None or self._last_ball_y is None:
            return

        if self._scan_active:
            self._scan_active = False
            self._last_scan_cmd_time = 0.0

        # Deadzone: ball is within the inner box around image center, hold still.
        # Stops YOLO detection jitter from turning into head oscillation when the
        # ball is stationary.
        if (
            abs(self._last_ball_x) < self._center_deadzone
            and abs(self._last_ball_y) < self._center_deadzone
        ):
            self._prev_pan_error = 0.0
            self._prev_tilt_error = 0.0
            return

        # Preserve the original OP3 sign convention.
        pan_error = -math.atan(self._last_ball_x * math.tan(self._fov_width_rad))
        tilt_error = -math.atan(self._last_ball_y * math.tan(self._fov_height_rad))

        delta_time = max(now - self._last_control_time, 1.0 / 240.0)
        self._last_control_time = now

        pan_error_diff = (pan_error - self._prev_pan_error) / delta_time
        tilt_error_diff = (tilt_error - self._prev_tilt_error) / delta_time

        self._pan_error_sum += pan_error
        self._tilt_error_sum += tilt_error

        pan_cmd = (
            pan_error * self._p_gain
            + pan_error_diff * self._d_gain
            + self._pan_error_sum * self._i_gain
        )
        tilt_cmd = (
            tilt_error * self._p_gain
            + tilt_error_diff * self._d_gain
            + self._tilt_error_sum * self._i_gain
        )

        self._prev_pan_error = pan_error
        self._prev_tilt_error = tilt_error

        pan_cmd *= self._pan_sign
        tilt_cmd *= self._tilt_sign

        pan_cmd = max(-self._max_offset_pan_rad, min(self._max_offset_pan_rad, pan_cmd))
        tilt_cmd = max(-self._max_offset_tilt_rad, min(self._max_offset_tilt_rad, tilt_cmd))

        if abs(pan_cmd) < self._min_command_rad and abs(tilt_cmd) < self._min_command_rad:
            return

        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan_cmd, tilt_cmd]
        msg.header.stamp = self.get_clock().now().to_msg()
        self._head_offset_pub.publish(msg)

        self.get_logger().info(
            f"[TRACK] norm=({self._last_x_norm:.2f},{self._last_y_norm:.2f}) "
            f"xy=({self._last_ball_x:.2f},{self._last_ball_y:.2f}) "
            f"offset=({pan_cmd:.3f},{tilt_cmd:.3f})",
            throttle_duration_sec=0.5,
        )

    def _ensure_scan_mode(self, now: Optional[float] = None) -> None:
        if now is None:
            now = time.monotonic()
        # Custom sweep: move head through forward + down positions only (no
        # ceiling-facing poses), dwelling at each so YOLO has time to detect.
        if (now - self._last_scan_cmd_time) < self._scan_period_sec:
            return
        pan, tilt = self._scan_positions[self._scan_idx]
        self._scan_idx = (self._scan_idx + 1) % len(self._scan_positions)

        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        msg.header.stamp = self.get_clock().now().to_msg()
        self._head_abs_pub.publish(msg)
        self._last_scan_cmd_time = now
        self._scan_active = True
        self.get_logger().info(
            f"[SCAN] move to ({math.degrees(pan):+.0f}°, {math.degrees(tilt):+.0f}°)",
            throttle_duration_sec=1.0,
        )

    def _reset_pid(self) -> None:
        self._prev_pan_error = 0.0
        self._prev_tilt_error = 0.0
        self._pan_error_sum = 0.0
        self._tilt_error_sum = 0.0
        self._last_control_time = time.monotonic()

    def _ensure_head_module_enabled(
        self, now: Optional[float] = None, force: bool = False
    ) -> None:
        if now is None:
            now = time.monotonic()

        if not force and (now - self._last_head_enable_time) < self._head_enable_retry_sec:
            return

        # Always safe to re-publish enable_ctrl_module: MotionModule::setModuleEnable
        # is guarded (`if this->enable_ == enable return;`), so repeat enables are
        # no-ops and do NOT reset scan_state_. If op3_manager came up slowly and the
        # first message was lost, subsequent tries will land.
        self._enable_pub.publish(String(data=self._head_module_name))

        # set_joint_ctrl_modules re-runs sync_write setup and can disrupt an
        # in-progress scan, so only publish it at startup (or explicit force).
        if force or not self._joint_ctrl_modules_sent:
            ctrl_msg = JointCtrlModule()
            ctrl_msg.joint_name = [self._head_pan_joint, self._head_tilt_joint]
            ctrl_msg.module_name = [self._head_module_name, self._head_module_name]
            self._joint_ctrl_pub.publish(ctrl_msg)
            self._joint_ctrl_modules_sent = True

        self._last_head_enable_time = now


def main() -> None:
    rclpy.init()
    node = HeadTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
