#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from soccer_msgs.msg import RobotState

class SimpleRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        self.state_pub = self.create_publisher(RobotState, 'state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.state = RobotState()
        self.state.status = RobotState.STATUS_READY
        
        self.get_logger().info('Simple Robot State Publisher started')
    
    def publish_state(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.header.frame_id = 'base_link'
        self.state_pub.publish(self.state)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()