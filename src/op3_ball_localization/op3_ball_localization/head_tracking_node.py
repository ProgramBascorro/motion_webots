#!/usr/bin/env python3
"""
Standalone head tracking node for ball following (scan only mode).
Subscribes to ball detections and controls head pan/tilt to track the ball.
Includes scanning behavior when ball is lost.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from op3_ball_detector_msgs.msg import CircleSetStamped


class HeadTrackingNode(Node):
    """Head-only tracking node - no walking or kicking."""

    def __init__(self) -> None:
        super().__init__("head_tracking_node")

        # Parameters - PID control
        self._p_gain = float(self.declare_parameter("p_gain", 0.45).value)
        self._d_gain = float(self.declare_parameter("d_gain", 0.045).value)

        # Parameters - Scanning
        self._scan_enabled = bool(self.declare_parameter("scan_enabled", True).value)
        self._scan_pan_min = float(self.declare_parameter("scan_pan_min", -1.2).value)
        self._scan_pan_max = float(self.declare_parameter("scan_pan_max", 1.2).value)
        self._scan_tilt_down = float(self.declare_parameter("scan_tilt_down", -0.6).value)
        self._scan_tilt_up = float(self.declare_parameter("scan_tilt_up", -0.25).value)
        self._scan_step = float(self.declare_parameter("scan_step", 0.2).value)
        self._scan_dwell_sec = float(self.declare_parameter("scan_dwell_sec", 0.8).value)

        # Parameters - Detection
        self._lost_timeout = float(self.declare_parameter("lost_timeout", 1.5).value)
        self._min_ball_radius = float(self.declare_parameter("min_ball_radius", 0.01).value)

        # Parameters - Joint names
        self._head_pan_joint = str(self.declare_parameter("head_pan_joint", "head_pan").value)
        self._head_tilt_joint = str(self.declare_parameter("head_tilt_joint", "head_tilt").value)
        self._head_module_name = str(self.declare_parameter("head_module_name", "head_control_module").value)

        # Parameters - Control rate
        self._control_rate = float(self.declare_parameter("control_rate", 10.0).value)

        # Publishers
        self._head_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states", 10
        )
        self._enable_pub = self.create_publisher(String, "/robotis/enable_ctrl_module", 10)

        # Subscribers
        self.create_subscription(
            CircleSetStamped,
            "/ball_detector_node/circle_set",
            self._ball_callback,
            10
        )

        # PID state
        self._prev_pan_error = 0.0
        self._prev_tilt_error = 0.0

        # Ball tracking state
        self._last_ball_time: Optional[float] = None
        self._last_ball_x: Optional[float] = None  # Normalized [-1, 1]
        self._last_ball_y: Optional[float] = None  # Normalized [-1, 1]

        # Scanning state
        self._scan_pan = self._scan_pan_min
        self._scan_tilt = self._scan_tilt_down
        self._scan_direction = 1.0
        self._scan_tilt_down_active = True
        self._last_scan_step_time = time.monotonic()

        # Current head position (for smooth transitions)
        self._current_pan = 0.0
        self._current_tilt = -0.5

        # Control timer
        self.create_timer(1.0 / self._control_rate, self._control_loop)

        # Enable head control module
        self._enable_head_module()

        self.get_logger().info("Head Tracking Node initialized (scan only mode)")
        self.get_logger().info(f"  P gain: {self._p_gain}, D gain: {self._d_gain}")
        self.get_logger().info(f"  Scan enabled: {self._scan_enabled}")
        self.get_logger().info(f"  Lost timeout: {self._lost_timeout}s")

    def _ball_callback(self, msg: CircleSetStamped) -> None:
        """Process ball detection messages."""
        if len(msg.circles) == 0:
            # No ball detected
            return

        # Get the largest ball (assuming it's the closest/most relevant)
        # CircleSetStamped uses Point objects where:
        # - x, y = normalized center coordinates [-1, 1]
        # - z = normalized radius
        best_ball = None
        best_radius = 0.0

        for circle in msg.circles:
            # Circle is a Point object: x, y = center, z = radius
            radius = circle.z
            if radius > self._min_ball_radius and radius > best_radius:
                best_radius = radius
                best_ball = circle

        if best_ball is None:
            return

        # Update ball state
        # The bridge already provides normalized coordinates [-1, 1]
        # x: -1 (left) to +1 (right)
        # y: -1 (top) to +1 (bottom)
        self._last_ball_x = best_ball.x  # Already normalized [-1, 1]
        self._last_ball_y = -best_ball.y  # Invert so up is positive
        self._last_ball_time = time.monotonic()

        self.get_logger().debug(
            f"Ball detected: x={self._last_ball_x:.2f}, y={self._last_ball_y:.2f}, "
            f"radius={best_ball.z:.3f}",
            throttle_duration_sec=1.0
        )

    def _control_loop(self) -> None:
        """Main control loop - decides between tracking and scanning."""
        now = time.monotonic()

        # Check if ball is visible
        ball_visible = (
            self._last_ball_time is not None and
            (now - self._last_ball_time) < self._lost_timeout
        )

        if ball_visible:
            # Track the ball
            self._track_ball()
        elif self._scan_enabled:
            # Scan for the ball
            self._scan_for_ball(now)
        else:
            # No tracking or scanning - hold position
            pass

    def _track_ball(self) -> None:
        """Use PID control to track the ball with head."""
        if self._last_ball_x is None or self._last_ball_y is None:
            return

        # Calculate errors (how far the ball is from center)
        pan_error = self._last_ball_x  # Positive = ball on right
        tilt_error = self._last_ball_y  # Positive = ball above center

        # PID control (P + D terms)
        pan_cmd = (
            self._p_gain * pan_error +
            self._d_gain * (pan_error - self._prev_pan_error)
        )
        tilt_cmd = (
            self._p_gain * tilt_error +
            self._d_gain * (tilt_error - self._prev_tilt_error)
        )

        # Update previous errors for next iteration
        self._prev_pan_error = pan_error
        self._prev_tilt_error = tilt_error

        # Update current position (incremental control)
        self._current_pan += pan_cmd
        self._current_tilt += tilt_cmd

        # Clamp to safe servo limits (avoid extreme positions)
        self._current_pan = max(-1.0, min(1.0, self._current_pan))
        self._current_tilt = max(-0.7, min(0.3, self._current_tilt))

        # Publish head command
        self._publish_head(self._current_pan, self._current_tilt)

        self.get_logger().info(
            f"[TRACKING] pan={self._current_pan:.2f}, tilt={self._current_tilt:.2f}, "
            f"ball_error=({pan_error:.2f}, {tilt_error:.2f})",
            throttle_duration_sec=1.0
        )

    def _scan_for_ball(self, now: float) -> None:
        """Scan head to search for the ball."""
        # Check if it's time to move to next scan position
        if now - self._last_scan_step_time < self._scan_dwell_sec:
            return

        self._last_scan_step_time = now

        # Calculate next scan position
        next_pan = self._scan_pan + self._scan_step * self._scan_direction
        next_tilt = self._scan_tilt_down if self._scan_tilt_down_active else self._scan_tilt_up

        # Check bounds and reverse direction if needed
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

        # Update scan state
        self._scan_pan = next_pan
        self._scan_tilt = next_tilt
        self._current_pan = next_pan
        self._current_tilt = next_tilt

        # Publish head command
        self._publish_head(next_pan, next_tilt)

        self.get_logger().info(
            f"[SCANNING] pan={next_pan:.2f}, tilt={next_tilt:.2f}, "
            f"dir={'+' if self._scan_direction > 0 else '-'}",
            throttle_duration_sec=1.0
        )

    def _publish_head(self, pan: float, tilt: float) -> None:
        """Publish head joint positions."""
        msg = JointState()
        msg.name = [self._head_pan_joint, self._head_tilt_joint]
        msg.position = [pan, tilt]
        msg.header.stamp = self.get_clock().now().to_msg()
        self._head_pub.publish(msg)

    def _enable_head_module(self) -> None:
        """Enable head control module."""
        msg = String()
        msg.data = self._head_module_name
        self._enable_pub.publish(msg)
        self.get_logger().info(f"Enabled head control module: {self._head_module_name}")


def main():
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
