#!/usr/bin/env python3
"""
Head-only ball tracking node for the OP3 ball detector stack.

Subscribes to the classic /ball_detector_node/circle_set (drop-in for both
the Hough-circle detector and our OpenVINO YOLO detector, which publishes
the same CircleSetStamped topic) and follows the original OP3 head tracking
convention: the node publishes head joint offsets that drive the ball
toward the center of the camera image.

The scan sweep is driven from head_tracking.yaml (forward + down positions
only, no ceiling-facing poses) and uses ABSOLUTE set_joint_states so a new
target immediately replaces whatever the module was executing — this is the
key difference from the internal ball_tracker + head_control_module scan
pattern, which chains scan corners until finishMoving() checks scan_state_.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotis_controller_msgs.msg import JointCtrlModule
from op3_ball_detector_msgs.msg import CircleSetStamped


class HeadTrackingNode(Node):
    """Head-only ball tracker - no walking or kicking."""

    def __init__(self) -> None:
        super().__init__("head_tracking_node")

        # /ball_detector_node/circle_set is what the YOLO detector (and the
        # legacy Hough detector) publishes. Coordinates are already in the
        # [-1, +1] image-frame convention, so no rescale needed downstream.
        self._ball_center_topic = str(
            self.declare_parameter("ball_center_topic", "/ball_detector_node/circle_set").value
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
        # Warmup right after activation: head_control_module drops commands until
        # its first process() cycle populates goal_position_ (enable handshake).
        # During this window re-issue the *current* scan target every
        # scan_warmup_retry_sec so the head starts moving within a fraction of a
        # second of the module becoming ready — instead of idling up to a full
        # scan_period_sec (the ~5 s dead time seen after pressing START).
        self._scan_warmup_sec = float(
            self.declare_parameter("scan_warmup_sec", 6.0).value
        )
        self._scan_warmup_retry_sec = float(
            self.declare_parameter("scan_warmup_retry_sec", 0.4).value
        )
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
            CircleSetStamped,
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
        # Wall-clock of the last activation ("start" command, or boot when
        # auto_start). Drives the post-activation scan warmup (see _ensure_scan_mode).
        self._activated_at: Optional[float] = time.monotonic() if self._active else None

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
            # Restart the scan sweep from the first position and open the warmup
            # window so the first SCAN retries fast until head_control is ready.
            self._activated_at = time.monotonic()
            self._scan_idx = 0
            self._last_scan_cmd_time = 0.0
            self._ensure_head_module_enabled(force=True)
            self.get_logger().info("[CMD] activated by start command")
        elif cmd == "stop" and self._active:
            self._active = False
            self._reset_pid()
            self._scan_active = False
            self._last_scan_cmd_time = 0.0
            # Park head at neutral (pan=0, tilt=forward) immediately so the
            # robot stops mid-scan instead of completing the last SCAN target.
            # Without this, the servos finish executing the last commanded
            # SCAN pose, making the head visually "keep scanning" for a
            # second or two after STOP was pressed.
            msg = JointState()
            msg.name = [self._head_pan_joint, self._head_tilt_joint]
            msg.position = [0.0, self._scan_tilt_forward_rad]
            msg.header.stamp = self.get_clock().now().to_msg()
            self._head_abs_pub.publish(msg)
            self.get_logger().info("[CMD] deactivated by stop command (head parked at neutral)")

    def _ball_center_callback(self, msg: CircleSetStamped) -> None:
        """Track the largest ball reported by the detector.

        CircleSetStamped.circles[i].(x, y) already use the OP3 convention:
        (-1, -1) = image top-left, (+1, +1) = image bottom-right. z is the
        radius in pixels — we pick the biggest one so a stray small false
        positive doesn't hijack the head from the real ball.
        """
        if not msg.circles:
            return

        best = max(msg.circles, key=lambda c: c.z)
        # Radius <= 0 is the "no ball" sentinel the C++ Hough detector uses.
        if best.z <= 0:
            return

        raw_x = float(best.x)
        raw_y = float(best.y)
        if math.isnan(raw_x) or math.isnan(raw_y):
            return

        # Log the value in [0, 1] like the old topic did, for readability.
        self._last_x_norm = (raw_x + 1.0) * 0.5
        self._last_y_norm = (raw_y + 1.0) * 0.5

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
        #
        # During the warmup window right after activation, head_control_module is
        # still finishing its enable handshake and drops commands until its first
        # process() cycle. Re-issue the *current* target at scan_warmup_retry_sec
        # (fast) instead of scan_period_sec so the head starts moving as soon as
        # the module is ready. We do NOT advance _scan_idx during warmup, so the
        # head parks at the first scan pose rather than sweeping too fast to see.
        in_warmup = (
            self._activated_at is not None
            and (now - self._activated_at) < self._scan_warmup_sec
        )
        period = self._scan_warmup_retry_sec if in_warmup else self._scan_period_sec
        if (now - self._last_scan_cmd_time) < period:
            return
        pan, tilt = self._scan_positions[self._scan_idx]
        if not in_warmup:
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
