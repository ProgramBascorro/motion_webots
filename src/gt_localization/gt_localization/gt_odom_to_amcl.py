#!/usr/bin/env python3
"""
Ground Truth Odometry to AMCL-compatible Odometry
Converts Webots ground truth to proper nav_msgs/Odometry on /odom topic
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class GTOdomToAMCL(Node):
    """
    Republish ground truth odometry to AMCL-compatible format
    
    Input:  /ground_truth/odom (nav_msgs/Odometry)
    Output: /odom (nav_msgs/Odometry)
            TF: odom → base_link
    """
    
    def __init__(self):
        super().__init__('gt_odom_to_amcl')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        
        # TF broadcaster
        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber to ground truth
        self.gt_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.gt_callback,
            10
        )
        
        # Publisher for AMCL
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        # Statistics
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info('='*60)
        self.get_logger().info('Ground Truth Odometry Republisher Started')
        self.get_logger().info(f'  Input:  /ground_truth/odom')
        self.get_logger().info(f'  Output: /odom')
        self.get_logger().info(f'  Frames: {self.odom_frame} → {self.base_frame}')
        self.get_logger().info(f'  Publish TF: {self.publish_tf_flag}')
        self.get_logger().info('='*60)
    
    def gt_callback(self, gt_msg: Odometry):
        """
        Convert ground truth to AMCL-compatible odometry
        
        Args:
            gt_msg: Ground truth odometry from Webots
        """
        # Create AMCL-compatible odometry message
        odom_msg = Odometry()
        
        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Pose (position + orientation)
        odom_msg.pose.pose = gt_msg.pose.pose
        
        # Covariance (low uncertainty for ground truth)
        # Position: 1mm std dev
        odom_msg.pose.covariance[0] = 0.000001  # x
        odom_msg.pose.covariance[7] = 0.000001  # y
        odom_msg.pose.covariance[14] = 0.000001  # z
        # Orientation: 0.01 rad (~0.5°) std dev
        odom_msg.pose.covariance[21] = 0.0001  # roll
        odom_msg.pose.covariance[28] = 0.0001  # pitch
        odom_msg.pose.covariance[35] = 0.0001  # yaw
        
        # Twist (velocity)
        odom_msg.twist.twist = gt_msg.twist.twist
        
        # Twist covariance
        odom_msg.twist.covariance[0] = 0.001  # vx
        odom_msg.twist.covariance[7] = 0.001  # vy
        odom_msg.twist.covariance[35] = 0.001  # vyaw
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform
        if self.publish_tf_flag:
            self.publish_transform(odom_msg)
        
        # Statistics
        self.msg_count += 1
        current_time = self.get_clock().now()
        
        # Log every 5 seconds
        if (current_time - self.last_log_time).nanoseconds > 5e9:
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(odom_msg.pose.pose.orientation)
            
            self.get_logger().info(
                f'Odometry: x={x:.3f}m, y={y:.3f}m, yaw={math.degrees(yaw):.1f}°  '
                f'({self.msg_count} messages published)'
            )
            
            self.last_log_time = current_time
    
    def publish_transform(self, odom_msg: Odometry):
        """
        Publish TF transform: odom → base_link
        
        Args:
            odom_msg: Odometry message
        """
        t = TransformStamped()
        
        # Header
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Translation
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        # Broadcast
        self.tf_broadcaster.sendTransform(t)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GTOdomToAMCL()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()