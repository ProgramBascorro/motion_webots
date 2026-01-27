#!/usr/bin/env python3
"""
Odometry Path Publisher
Converts PoseWithCovarianceStamped to Path for visualization trail
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')
        
        # Parameters
        self.declare_parameter('max_path_length', 500)
        self.max_path_length = self.get_parameter('max_path_length').value
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Publisher for path
        self.path_pub = self.create_publisher(
            Path,
            'odom_path',
            10
        )
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'odom_combined',
            self.odom_callback,
            10
        )
        
        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_path)
        
        self.get_logger().info(
            f'Odometry Path Publisher started (max length: {self.max_path_length})'
        )
    
    def odom_callback(self, msg: PoseWithCovarianceStamped):
        """Add new pose to path"""
        # Create PoseStamped from odometry
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Add to path
        self.path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # Update path header timestamp
        self.path.header.stamp = msg.header.stamp
    
    def publish_path(self):
        """Publish current path"""
        if len(self.path.poses) > 0:
            self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()