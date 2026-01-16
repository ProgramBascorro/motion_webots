#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String


class WebotsBridge(Node):
    def __init__(self):
        super().__init__("webots_bridge")

        # Joint state from Webots
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/robotis_op3/joint_states",  # Actual Webots topic
            self.joint_state_callback,
            10,
        )

        # Individual joint command publishers for Webots
        self.joint_publishers = {}
        # Order matches OP3.robot IDs (ascending).
        self.joint_names = [
            "r_sho_pitch",
            "l_sho_pitch",
            "r_sho_roll",
            "l_sho_roll",
            "r_el",
            "l_el",
            "r_hip_yaw",
            "l_hip_yaw",
            "r_hip_roll",
            "l_hip_roll",
            "r_hip_pitch",
            "l_hip_pitch",
            "r_knee",
            "l_knee",
            "r_ank_pitch",
            "l_ank_pitch",
            "r_ank_roll",
            "l_ank_roll",
            "head_pan",
            "head_tilt",
        ]

        for joint_name in self.joint_names:
            topic_name = f"/robotis_op3/{joint_name}_position/command"
            self.joint_publishers[joint_name] = self.create_publisher(Float64, topic_name, 10)

        # Action Editor communication
        self.action_command_sub = self.create_subscription(
            String,
            "/webots/action_command",
            self.action_command_callback,
            10,
        )

        self.joint_positions_sub = self.create_subscription(
            Float64MultiArray,
            "/webots/joint_positions",
            self.joint_positions_callback,
            10,
        )

        # Feedback to Action Editor
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",  # Topic that Action Editor expects
            10,
        )

        # Debug publisher
        self.debug_pub = self.create_publisher(String, "/webots_bridge/debug", 10)

        # Current joint states
        self.current_joint_states = JointState()
        self.received_webots_data = False
        self._joint_log_once = False

        # Timer for status updates
        self.timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info("Webots bridge initialized")
        self.get_logger().info("Subscribed to: /robotis_op3/joint_states")
        self.get_logger().info("Publishing to: /joint_states")
        self.get_logger().info("Ready for individual joint commands")

    def joint_state_callback(self, msg: JointState) -> None:
        """Receive joint states from Webots and forward to Action Editor."""
        self.current_joint_states = msg
        self.received_webots_data = True

        # Forward to Action Editor with expected topic name
        self.joint_state_pub.publish(msg)

        if not self._joint_log_once:
            self.get_logger().info(f"Joint states received: {len(msg.name)} joints")
            self._joint_log_once = True

    def action_command_callback(self, msg: String) -> None:
        """Handle action commands from Action Editor."""
        self.get_logger().info(f"Action command: {msg.data}")

        try:
            # Parse action command (expecting JSON format)
            command_data = json.loads(msg.data)

            if "action" in command_data:
                action_type = command_data["action"]
                self.get_logger().info(f"Executing action: {action_type}")

                if action_type == "play":
                    self.handle_play_action(command_data)
                elif action_type == "stop":
                    self.handle_stop_action()

        except json.JSONDecodeError:
            # Simple string command
            self.get_logger().info(f"Simple action: {msg.data}")

    def joint_positions_callback(self, msg: Float64MultiArray) -> None:
        """Handle joint position commands from Action Editor."""
        positions = list(msg.data)
        if not positions:
            return

        count = min(len(positions), len(self.joint_names))
        self.get_logger().info(f"Setting joint positions: {count} joints")

        for idx in range(count):
            joint_name = self.joint_names[idx]
            command_msg = Float64()
            command_msg.data = float(positions[idx])
            self.joint_publishers[joint_name].publish(command_msg)

    def handle_play_action(self, command_data: dict) -> None:
        """Handle play action command."""
        self.get_logger().info("Playing action in Webots")

        if "positions" in command_data:
            positions = command_data["positions"]
            for joint_name, position in positions.items():
                if joint_name in self.joint_publishers:
                    command_msg = Float64()
                    command_msg.data = float(position)
                    self.joint_publishers[joint_name].publish(command_msg)

    def handle_stop_action(self) -> None:
        """Handle stop action command."""
        self.get_logger().info("Stopping action in Webots")

    def publish_status(self) -> None:
        """Publish bridge status."""
        status = (
            f"Bridge OK. Webots data: {self.received_webots_data}. "
            f"Joints: {len(self.current_joint_states.name)}"
        )

        debug_msg = String()
        debug_msg.data = status
        self.debug_pub.publish(debug_msg)

        if self.received_webots_data:
            self.get_logger().info(status)
        else:
            self.get_logger().warning("No Webots data received yet")


def main(args=None) -> None:
    rclpy.init(args=args)

    print("Starting Webots bridge with topic mapping...")
    bridge = WebotsBridge()

    try:
        print("Bridge running - monitoring Webots topics")
        print("Subscribing to: /robotis_op3/joint_states")
        print("Publishing to: /joint_states")
        print("Ready for joint commands")
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("Bridge stopped")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
