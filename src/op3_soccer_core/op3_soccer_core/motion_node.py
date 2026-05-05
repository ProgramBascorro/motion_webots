#!/usr/bin/env python3
"""Layer 8 — Motion Node.
The ONLY node in op3_soccer_core that publishes to /robotis/... topics.
All other strategy nodes must send commands here via /motion/command (JSON).
"""
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .config_loader import load_rules
from .walking_interface import WalkingInterface

# Allowed action field values in /motion/command JSON
_ACTION_WALK = "walk"
_ACTION_KICK = "kick"
_ACTION_GETUP = "getup"
_ACTION_STOP = "stop"
_ACTION_ENABLE_MODULE = "enable_module"


class MotionNode(Node):
    def __init__(self) -> None:
        super().__init__("op3_motion")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._max_x = float(self.declare_parameter("max_x", rules.get("walk_max_vx", 0.02)).value)
        self._max_y = float(self.declare_parameter("max_y", rules.get("walk_max_vy", 0.015)).value)
        self._max_yaw = float(self.declare_parameter("max_yaw", rules.get("walk_max_vyaw", 0.25)).value)
        self._kick_cooldown_sec = float(self.declare_parameter(
            "kick_cooldown_sec", rules.get("kick_cooldown_sec", 1.0)).value)
        self._getup_cooldown_sec = float(self.declare_parameter(
            "getup_cooldown_sec", rules.get("getup_cooldown_sec", 2.0)).value)
        self._kick_left_page = int(self.declare_parameter(
            "kick_left_page", rules.get("kick_left_page", 120)).value)
        self._kick_right_page = int(self.declare_parameter(
            "kick_right_page", rules.get("kick_right_page", 121)).value)
        self._getup_front_page = int(self.declare_parameter(
            "getup_front_page", rules.get("getup_front_page", 122)).value)
        self._getup_back_page = int(self.declare_parameter(
            "getup_back_page", rules.get("getup_back_page", 123)).value)
        self._control_rate = float(self.declare_parameter("control_rate", 10.0).value)

        self._safety_veto = False
        self._last_kick_time = 0.0
        self._last_getup_time = 0.0
        self._last_status_warn = 0.0

        self._walker = WalkingInterface(self)

        self.create_subscription(String, "/motion/command", self._command_cb, 10)
        self.create_subscription(Bool, "/safety/motion_veto", self._veto_cb, 10)

        self._status_pub = self.create_publisher(String, "/motion/status", 10)

        self.create_timer(1.0 / self._control_rate, self._tick)

        self.get_logger().info("Motion node ready — sole publisher to /robotis/...")

    def _veto_cb(self, msg: Bool) -> None:
        self._safety_veto = bool(msg.data)

    def _command_cb(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid /motion/command JSON: {msg.data[:80]}")
            return

        if self._safety_veto and cmd.get("action") not in (_ACTION_STOP, _ACTION_GETUP):
            self._walker.stop_walking()
            return

        action = str(cmd.get("action", _ACTION_STOP))

        if action == _ACTION_STOP:
            self._walker.stop_walking()

        elif action == _ACTION_WALK:
            if not self._walker.baseline_ready:
                self._warn_baseline_missing()
                return
            vx = float(cmd.get("vx", 0.0))
            vy = float(cmd.get("vy", 0.0))
            vyaw = float(cmd.get("vyaw", 0.0))
            self._walker.enable_walking()
            self._walker.set_walk(vx, vy, vyaw, self._max_x, self._max_y, self._max_yaw)
            self._walker.start_walking()

        elif action == _ACTION_KICK:
            now = time.monotonic()
            if now - self._last_kick_time < self._kick_cooldown_sec:
                return
            page = int(cmd.get("page", self._kick_right_page))
            self._walker.stop_walking()
            self._walker.kick(page)
            self._last_kick_time = now

        elif action == _ACTION_GETUP:
            now = time.monotonic()
            if now - self._last_getup_time < self._getup_cooldown_sec:
                return
            direction = str(cmd.get("direction", "front"))
            page = self._getup_front_page if direction == "front" else self._getup_back_page
            self._walker.stop_walking()
            self._walker.kick(page)
            self._last_getup_time = now

        elif action == _ACTION_ENABLE_MODULE:
            module = str(cmd.get("module", "walking_module"))
            self._walker.enable_module(module)

        else:
            self.get_logger().warn(f"Unknown motion action: {action}")

    def _tick(self) -> None:
        now = time.monotonic()
        self._walker.tick(now)
        status = {
            "baseline_ready": self._walker.baseline_ready,
            "safety_veto": self._safety_veto,
        }
        self._status_pub.publish(String(data=json.dumps(status)))

    def _warn_baseline_missing(self) -> None:
        now = time.monotonic()
        if now - self._last_status_warn >= 1.0:
            self.get_logger().warn(
                "Walking baseline params not yet loaded — walk command ignored. "
                "Is /robotis/walking/get_params service available?"
            )
            self._last_status_warn = now


def main() -> None:
    rclpy.init()
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
