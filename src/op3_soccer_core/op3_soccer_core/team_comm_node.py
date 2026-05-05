#!/usr/bin/env python3
"""Layer 5 — Team Communication Node.

Two operating modes:

  UDP mode  (use_ros_relay: false, default for real robots)
    Sends JSON broadcast over UDP; receives UDP broadcasts.
    Works on a real WiFi/LAN network.
    May fail on WSL2 same-machine simulation due to broadcast forwarding issues.

  ROS relay mode  (use_ros_relay: true, recommended for Webots simulation)
    Sends JSON as a ROS2 String to a shared relay topic.
    Receives from the same relay topic (self-messages filtered by self_id).
    No UDP needed — works on any machine including WSL2.
    The relay topic (relay_topic param) must be the same for all robots on the
    same team: e.g. "/team1/team_comm/relay".
"""
import json
import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeamCommNode(Node):
    def __init__(self) -> None:
        super().__init__("op3_team_comm")

        self._broadcast_ip  = str(self.declare_parameter("broadcast_ip", "255.255.255.255").value)
        self._udp_port      = int(self.declare_parameter("udp_port", 10030).value)
        self._publish_rate  = float(self.declare_parameter("publish_rate", 10.0).value)
        self._self_id       = int(self.declare_parameter("self_id", 0).value)
        self._ignore_self   = bool(self.declare_parameter("ignore_self", True).value)

        # ROS relay mode — skip UDP entirely, use a shared ROS2 topic
        self._use_ros_relay = bool(self.declare_parameter("use_ros_relay", False).value)
        self._relay_topic   = str(self.declare_parameter(
            "relay_topic", "/team_comm/relay").value)

        # /team_comm/tx → outgoing; /team_comm/rx → incoming
        self._tx_sub = self.create_subscription(
            String, "/team_comm/tx", self._tx_cb, 10)
        self._rx_pub = self.create_publisher(String, "/team_comm/rx", 10)

        if self._use_ros_relay:
            self._relay_pub = self.create_publisher(
                String, self._relay_topic, 10)
            self.create_subscription(
                String, self._relay_topic, self._relay_rx_cb, 10)
            self._sock = None
            self.get_logger().info(
                f"Team comm ready [ROS relay mode] "
                f"(relay={self._relay_topic}, self_id={self._self_id})")
        else:
            self._relay_pub = None
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                pass   # SO_REUSEPORT not available on all platforms
            self._sock.bind(("", self._udp_port))
            self._sock.setblocking(False)
            self.create_timer(1.0 / self._publish_rate, self._poll_rx)
            self.get_logger().info(
                f"Team comm ready [UDP mode] "
                f"(port={self._udp_port}, broadcast={self._broadcast_ip}, "
                f"self_id={self._self_id})")

    # ── TX path ──────────────────────────────────────────────────────────────

    def _tx_cb(self, msg: String) -> None:
        if not msg.data:
            return
        if self._use_ros_relay:
            self._relay_pub.publish(String(data=msg.data))
        else:
            try:
                self._sock.sendto(
                    msg.data.encode("utf-8"),
                    (self._broadcast_ip, self._udp_port))
            except OSError as exc:
                self.get_logger().warn(f"UDP send failed: {exc}")

    # ── RX path — ROS relay mode ─────────────────────────────────────────────

    def _relay_rx_cb(self, msg: String) -> None:
        if self._ignore_self and self._is_self(msg.data):
            return
        self._rx_pub.publish(String(data=msg.data))

    # ── RX path — UDP mode ───────────────────────────────────────────────────

    def _poll_rx(self) -> None:
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warn(f"UDP recv failed: {exc}")
                break
            try:
                payload = data.decode("utf-8")
            except UnicodeDecodeError:
                continue
            if self._ignore_self and self._is_self(payload):
                continue
            self._rx_pub.publish(String(data=payload))

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _is_self(self, payload: str) -> bool:
        if self._self_id <= 0:
            return False
        try:
            return int(json.loads(payload).get("player_id", -1)) == self._self_id
        except (json.JSONDecodeError, ValueError):
            return False


def main() -> None:
    rclpy.init()
    node = TeamCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
