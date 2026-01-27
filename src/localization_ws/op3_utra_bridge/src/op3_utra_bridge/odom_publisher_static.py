#!/usr/bin/env python3
"""
Static odom frame publisher
Publishes odom→base_link transform for AMCL
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StaticOdomPublisher(Node):
    def __init__(self):
        super().__init__('static_odom_publisher')
        
        self.br = TransformBroadcaster(self)
        
        # Create timer to publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_odom)
        
        self.get_logger().info("Static odom publisher started")
    
    def publish_odom(self):
        """Publish static odom→base_link transform"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Identity transform (robot at origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = StaticOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()