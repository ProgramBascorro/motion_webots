#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState

class WebotsBridge(Node):
    def __init__(self):
        super().__init__('webots_bridge')
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/robotis_op3/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.joint_names = [
            'r_sho_pitch', 'l_sho_pitch', 'r_sho_roll', 'l_sho_roll',
            'r_el', 'l_el', 'r_hip_yaw', 'l_hip_yaw',
            'r_hip_roll', 'l_hip_roll', 'r_hip_pitch', 'l_hip_pitch',
            'r_knee', 'l_knee', 'r_ank_pitch', 'l_ank_pitch',
            'r_ank_roll', 'l_ank_roll', 'head_pan', 'head_tilt'
        ]
        
        self.joint_publishers = {}
        for joint_name in self.joint_names:
            topic1 = f'/robotis_op3/{joint_name}_position/command'
            topic2 = f'/robotis/{joint_name}_position/command'

            self.joint_publishers[joint_name] = [
                self.create_publisher(Float64, topic1, 10),
                self.create_publisher(Float64, topic2, 10),
            ]

        self.action_command_sub = self.create_subscription(
            String,
            '/webots/action_command',
            self.action_command_callback,
            10
        )
        
        self.joint_positions_sub = self.create_subscription(
            Float64MultiArray,
            '/webots/joint_positions',
            self.joint_positions_callback,
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.debug_pub = self.create_publisher(String, '/webots_bridge/debug', 10)
        
        self.current_joint_states = JointState()
        self.received_webots_data = False
        self.timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('🌉 Webots Bridge initialized')
        self.get_logger().info('📡 Listening: /robotis_op3/joint_states')
        self.get_logger().info('📤 Publishing: /joint_states')
        self.get_logger().info('🎯 Joint commands ready')

    def joint_state_callback(self, msg: JointState):
        self.current_joint_states = msg
        self.received_webots_data = True
        self.joint_state_pub.publish(msg)
        self.get_logger().info(f'📊 Updated: {len(msg.name)} joints')

    def action_command_callback(self, msg: String):
        self.get_logger().info(f'🎬 Command received: {msg.data}')

    def joint_positions_callback(self, msg: Float64MultiArray):
        positions = list(msg.data)
        count = min(len(positions), len(self.joint_names))
        self.get_logger().info(f'🎯 Moving {count} joints')

        for i in range(count):
            joint_name = self.joint_names[i]
            data = Float64()
            data.data = float(positions[i])
            for publisher in self.joint_publishers[joint_name]:
                publisher.publish(data)

        self.get_logger().info('✅ Positions sent to Webots')

    def publish_status(self):
        msg = String()
        msg.data = f"Bridge OK | Webots data={self.received_webots_data}"
        self.debug_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = WebotsBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("🛑 Bridge stopped")
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
