#!/usr/bin/env python3
"""
OP3 Static TF Publisher
Publishes static transforms for ROBOTIS OP3 robot
Handles: base_link -> head_link -> cam_link
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class OP3StaticTFPublisher(Node):
    """Publishes static TF tree for OP3 robot"""
    
    def __init__(self):
        super().__init__('op3_static_tf_publisher')
        
        # Create static broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)
        
        # Declare parameters
        self.declare_parameter('robot_name', 'op3')
        self.declare_parameter('camera_height', 0.475)
        self.declare_parameter('camera_tilt', -20.0)  # degrees
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        camera_height = self.get_parameter('camera_height').value
        camera_tilt_deg = self.get_parameter('camera_tilt').value
        
        # Publish transforms
        self.publish_transforms(camera_height, camera_tilt_deg)
        
        self.get_logger().info(f'OP3 static TF tree published for {self.robot_name}')
        self.get_logger().info(f'Camera height: {camera_height}m, tilt: {camera_tilt_deg}°')
    
    def publish_transforms(self, camera_height, camera_tilt_deg):
        """
        Publish all static transforms
        
        TF Tree:
        odom -> base_link -> head_link -> cam_link
        """
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # Transform 1: base_link -> head_link
        # Head is at torso height
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'head_link'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.40  # OP3 torso to head ~40cm
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)
        
        # Transform 2: head_link -> cam_link
        # Camera is forward and above head pivot, tilted down
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'head_link'
        t2.child_frame_id = 'cam_link'
        t2.transform.translation.x = 0.08   # 8cm forward
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.075  # 7.5cm above head pivot
        
        # Camera tilt (rotation around Y axis)
        camera_tilt_rad = math.radians(camera_tilt_deg)
        # Convert to quaternion (rotation around Y axis)
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = math.sin(camera_tilt_rad / 2.0)
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = math.cos(camera_tilt_rad / 2.0)
        transforms.append(t2)
        
        # Publish all transforms
        self.broadcaster.sendTransform(transforms)
        
        # Log transform details
        self.get_logger().info('Published transforms:')
        self.get_logger().info('  base_link -> head_link: [0, 0, 0.40]')
        self.get_logger().info(f'  head_link -> cam_link: [0.08, 0, 0.075], tilt={camera_tilt_deg}°')

def main(args=None):
    rclpy.init(args=args)
    
    node = OP3StaticTFPublisher()
    
    try:
        # Keep node alive (static transforms don't need spinning)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()