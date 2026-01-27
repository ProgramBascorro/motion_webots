#!/usr/bin/env python3
"""
Noisy Odometry Simulator
Adds realistic noise to ground truth odometry to simulate real robot
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class NoisyOdometrySim(Node):
    """
    Simulate noisy odometry from perfect ground truth
    
    Noise model:
      - Position drift: Proportional to distance traveled
      - Rotation drift: Proportional to rotation
      - Random walk: Accumulating small errors
    
    Typical real robot: ±5-10cm per meter, ±2-5° per 90° turn
    """
    
    def __init__(self):
        super().__init__('noisy_odom_sim')
        
        # Parameters for noise model
        self.declare_parameter('linear_drift_per_meter', 0.05)  # 5cm per meter
        self.declare_parameter('angular_drift_per_radian', 0.05)  # ~3° per 90° turn
        self.declare_parameter('random_walk_std', 0.005)  # 5mm random walk per update
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        self.linear_drift = self.get_parameter('linear_drift_per_meter').value
        self.angular_drift = self.get_parameter('angular_drift_per_radian').value
        self.random_walk = self.get_parameter('random_walk_std').value
        
        # State
        self.last_gt_pose = None
        self.accumulated_drift_x = 0.0
        self.accumulated_drift_y = 0.0
        self.accumulated_drift_yaw = 0.0
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to ground truth
        self.gt_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.gt_callback,
            10
        )
        
        # Publish noisy odometry
        self.noisy_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.msg_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Noisy Odometry Simulator Started')
        self.get_logger().info(f'  Linear drift: {self.linear_drift*100:.1f}cm per meter')
        self.get_logger().info(f'  Angular drift: {math.degrees(self.angular_drift):.1f}° per 90° turn')
        self.get_logger().info(f'  Random walk: {self.random_walk*1000:.1f}mm per update')
        self.get_logger().info('  This simulates REALISTIC humanoid robot odometry!')
        self.get_logger().info('='*60)
    
    def gt_callback(self, gt_msg: Odometry):
        """Add realistic noise to ground truth odometry"""
        
        # Get current GT pose
        gt_x = gt_msg.pose.pose.position.x
        gt_y = gt_msg.pose.pose.position.y
        gt_yaw = self.quaternion_to_yaw(gt_msg.pose.pose.orientation)
        
        # Calculate movement since last update
        if self.last_gt_pose is not None:
            last_x, last_y, last_yaw = self.last_gt_pose
            
            # Distance traveled
            dx = gt_x - last_x
            dy = gt_y - last_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Rotation
            dyaw = self.normalize_angle(gt_yaw - last_yaw)
            
            # Accumulate totals
            self.total_distance += distance
            self.total_rotation += abs(dyaw)
            
            # Add proportional drift
            if distance > 0.001:  # Only if moving
                # Drift perpendicular to motion (typical for step-based odometry)
                motion_angle = math.atan2(dy, dx)
                drift_distance = distance * self.linear_drift
                
                self.accumulated_drift_x += drift_distance * math.cos(motion_angle + math.pi/2)
                self.accumulated_drift_y += drift_distance * math.sin(motion_angle + math.pi/2)
            
            # Add rotation drift
            if abs(dyaw) > 0.01:
                self.accumulated_drift_yaw += dyaw * self.angular_drift
            
            # Add random walk (always present)
            self.accumulated_drift_x += np.random.normal(0, self.random_walk)
            self.accumulated_drift_y += np.random.normal(0, self.random_walk)
            self.accumulated_drift_yaw += np.random.normal(0, self.random_walk * 0.1)
        
        self.last_gt_pose = (gt_x, gt_y, gt_yaw)
        
        # Create noisy odometry
        noisy_msg = Odometry()
        noisy_msg.header.stamp = self.get_clock().now().to_msg()
        noisy_msg.header.frame_id = 'odom'
        noisy_msg.child_frame_id = 'base_link'
        
        # Add accumulated noise
        noisy_x = gt_x + self.accumulated_drift_x
        noisy_y = gt_y + self.accumulated_drift_y
        noisy_yaw = self.normalize_angle(gt_yaw + self.accumulated_drift_yaw)
        
        # Position
        noisy_msg.pose.pose.position.x = noisy_x
        noisy_msg.pose.pose.position.y = noisy_y
        noisy_msg.pose.pose.position.z = gt_msg.pose.pose.position.z
        
        # Orientation
        q = self.yaw_to_quaternion(noisy_yaw)
        noisy_msg.pose.pose.orientation = q
        
        # Covariance (higher for noisy odom)
        noisy_msg.pose.covariance[0] = 0.01  # x: 10cm std dev
        noisy_msg.pose.covariance[7] = 0.01  # y: 10cm std dev
        noisy_msg.pose.covariance[35] = 0.01  # yaw: ~6° std dev
        
        # Velocity (copy from GT, could add noise here too)
        noisy_msg.twist = gt_msg.twist
        noisy_msg.twist.covariance[0] = 0.01
        noisy_msg.twist.covariance[7] = 0.01
        noisy_msg.twist.covariance[35] = 0.01
        
        # Publish
        self.noisy_pub.publish(noisy_msg)
        self.publish_tf(noisy_msg)
        
        # Log occasionally
        self.msg_count += 1
        if self.msg_count % 250 == 0:
            error = math.sqrt(self.accumulated_drift_x**2 + self.accumulated_drift_y**2)
            self.get_logger().info(
                f'Drift: {error*100:.1f}cm positional, {math.degrees(abs(self.accumulated_drift_yaw)):.1f}° angular  '
                f'(traveled {self.total_distance:.1f}m, turned {math.degrees(self.total_rotation):.1f}°)'
            )
    
    def publish_tf(self, odom_msg: Odometry):
        """Publish TF: odom → base_link"""
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = NoisyOdometrySim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()