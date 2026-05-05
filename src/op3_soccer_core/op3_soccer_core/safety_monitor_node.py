#!/usr/bin/env python3
"""Layer 3 — Safety Monitor Node.
Monitors IMU for falls. Publishes /safety/motion_veto=True during recovery.
When fallen: sends stop + getup command to /motion/command.
"""
import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from .config_loader import load_rules

# Safety FSM states
_STATE_STABLE = "stable"
_STATE_UNSTABLE = "unstable"
_STATE_FALLEN_FRONT = "fallen_front"
_STATE_FALLEN_BACK = "fallen_back"
_STATE_RECOVERING = "recovering"


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("op3_safety_monitor")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._fall_pitch_front_deg = float(
            self.declare_parameter("fall_pitch_front_deg",
                                   rules.get("fall_pitch_front_deg", 60.0)).value
        )
        self._fall_pitch_back_deg = float(
            self.declare_parameter("fall_pitch_back_deg",
                                   rules.get("fall_pitch_back_deg", 60.0)).value
        )
        self._fall_hold_sec = float(self.declare_parameter(
            "fall_hold_sec", rules.get("fall_hold_sec", 0.3)).value)
        self._fall_cooldown_sec = float(self.declare_parameter(
            "fall_cooldown_sec", rules.get("fall_cooldown_sec", 4.0)).value)
        self._recover_pitch_deg = float(self.declare_parameter(
            "fall_recover_pitch_deg", rules.get("fall_recover_pitch_deg", 25.0)).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        # RoboCup rule: robot not recovered within 20s → incapable (10s removal penalty)
        self._incapable_timeout_sec = float(self.declare_parameter(
            "incapable_timeout_sec", rules.get("incapable_timeout_sec", 20.0)).value)

        self._state = _STATE_STABLE
        self._fall_start: Optional[float] = None
        self._getup_sent_at: Optional[float] = None
        self._fallen_since: Optional[float] = None   # tracks total time fallen (incapable rule)
        self._incapable_reported = False
        self._imu_pitch = 0.0
        self._imu_roll = 0.0
        self._last_imu_time: Optional[float] = None

        self.create_subscription(Imu, "/robotis_op3/imu", self._imu_cb, 10)

        self._veto_pub = self.create_publisher(Bool, "/safety/motion_veto", 10)
        self._status_pub = self.create_publisher(String, "/safety/status", 10)
        self._motion_pub = self.create_publisher(String, "/motion/command", 10)
        self._gc_fallen_pub = self.create_publisher(Bool, "/game_controller/status/fallen", 10)
        # Signals GC that this robot is incapable (fallen > 20s) → GC removes for 10s
        self._gc_incapable_pub = self.create_publisher(Bool, "/game_controller/status/incapable", 10)

        self.create_timer(1.0 / self._publish_rate, self._update)

        self.get_logger().info("Safety monitor ready")

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._imu_pitch = math.asin(sinp)
        self._last_imu_time = time.monotonic()

    def _update(self) -> None:
        now = time.monotonic()
        pitch_deg = math.degrees(self._imu_pitch)
        blocking = self._state in (_STATE_FALLEN_FRONT, _STATE_FALLEN_BACK, _STATE_RECOVERING)

        if self._state == _STATE_STABLE:
            if pitch_deg > self._fall_pitch_front_deg:
                self._state = _STATE_UNSTABLE
                self._fall_start = now
            elif pitch_deg < -self._fall_pitch_back_deg:
                self._state = _STATE_UNSTABLE
                self._fall_start = now

        elif self._state == _STATE_UNSTABLE:
            if (abs(pitch_deg) < min(self._fall_pitch_front_deg, self._fall_pitch_back_deg)):
                self._state = _STATE_STABLE
                self._fall_start = None
            elif self._fall_start is not None and (now - self._fall_start) >= self._fall_hold_sec:
                if pitch_deg > 0:
                    self._state = _STATE_FALLEN_FRONT
                else:
                    self._state = _STATE_FALLEN_BACK
                self._motion_pub.publish(String(data=json.dumps({"action": "stop"})))
                self.get_logger().warn(f"Fall detected: {self._state}")

        elif self._state in (_STATE_FALLEN_FRONT, _STATE_FALLEN_BACK):
            if self._fallen_since is None:
                self._fallen_since = now
                self._incapable_reported = False
            direction = "front" if self._state == _STATE_FALLEN_FRONT else "back"
            self._motion_pub.publish(
                String(data=json.dumps({"action": "getup", "direction": direction}))
            )
            self._getup_sent_at = now
            self._state = _STATE_RECOVERING

        elif self._state == _STATE_RECOVERING:
            cooldown_elapsed = (
                self._getup_sent_at is not None
                and (now - self._getup_sent_at) >= self._fall_cooldown_sec
            )
            pitch_ok = abs(pitch_deg) < self._recover_pitch_deg
            if cooldown_elapsed and pitch_ok:
                self._state = _STATE_STABLE
                self._getup_sent_at = None
                self._fallen_since = None
                self._incapable_reported = False
                self.get_logger().info("Recovery complete — returning to stable")

        blocking = self._state in (_STATE_FALLEN_FRONT, _STATE_FALLEN_BACK, _STATE_RECOVERING)
        self._veto_pub.publish(Bool(data=blocking))
        self._gc_fallen_pub.publish(Bool(data=blocking))

        # RoboCup incapable player rule: fallen > 20s without recovery → signal GC
        incapable = False
        if blocking and self._fallen_since is not None:
            fallen_duration = now - self._fallen_since
            if fallen_duration >= self._incapable_timeout_sec and not self._incapable_reported:
                self.get_logger().warn(
                    f"Incapable player: fallen {fallen_duration:.1f}s (>{self._incapable_timeout_sec}s)"
                )
                self._incapable_reported = True
            incapable = self._incapable_reported
        self._gc_incapable_pub.publish(Bool(data=incapable))

        status = {
            "state": self._state, "blocking": blocking,
            "pitch_deg": round(pitch_deg, 1), "incapable": incapable,
        }
        self._status_pub.publish(String(data=json.dumps(status)))


def main() -> None:
    rclpy.init()
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
