#!/usr/bin/env python3
"""
IMU Bridge for OP3-UTRA Bridge
Republishes OP3 IMU data with proper timestamp and frame_id
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        
        # Determine source (real robot or simulation)
        self.declare_parameter('use_sim', False)
        use_sim = self.get_parameter('use_sim').value
        
        # Frame ID for IMU data
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        
        # Select appropriate IMU topic
        if use_sim:
            imu_topic = '/robotis_op3/imu'
            self.get_logger().info(f'Using simulation IMU: {imu_topic}')
        else:
            imu_topic = '/robotis/open_cr/imu'
            self.get_logger().info(f'Using real robot IMU: {imu_topic}')
        
        # Publisher for standardized IMU
        self.imu_pub = self.create_publisher(
            Imu,
            'imu/data',
            10
        )
        
        # Subscriber to OP3 IMU
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        
        self.get_logger().info(f'IMU Bridge started, publishing to imu/data with frame {self.target_frame}')
    
    def imu_callback(self, msg: Imu):
        """
        Republish IMU data with proper timestamp and frame_id
        """
        # Create new message
        imu_msg = Imu()
        
        # Copy header
        imu_msg.header = msg.header
        
        # Fix timestamp if it's zero (common in OP3)
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            
        # Set proper frame_id
        if not msg.header.frame_id or msg.header.frame_id == '':
            imu_msg.header.frame_id = self.target_frame
        
        # Copy orientation
        imu_msg.orientation = msg.orientation
        imu_msg.orientation_covariance = msg.orientation_covariance
        
        # Copy angular velocity
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # Copy linear acceleration
        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # Publish corrected message
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()