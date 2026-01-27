#!/usr/bin/env python3
"""
Simple PointCloud to LaserScan Converter
Custom implementation to bypass potential bugs in pointcloud_to_laserscan
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math

class SimplePC2Scan(Node):
    def __init__(self):
        super().__init__('simple_pc2scan')
        
        # Parameters
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0174533)  # ~1 degree
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('scan_height', 0.0)  # Z level to use
        
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_height = self.get_parameter('scan_height').value
        
        # Calculate number of ranges
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/field_scan', 10)
        
        # Subscriber
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/field_point_cloud',
            self.convert_callback,
            10
        )
        
        self.get_logger().info('🔄 Simple PointCloud to LaserScan Converter Started')
        self.get_logger().info(f'   Angle range: [{math.degrees(self.angle_min):.1f}°, {math.degrees(self.angle_max):.1f}°]')
        self.get_logger().info(f'   Number of ranges: {self.num_ranges}')
        self.get_logger().info(f'   Range limits: [{self.range_min:.2f}, {self.range_max:.2f}]m')
        
    def convert_callback(self, pc_msg):
        # Decode pointcloud
        points = []
        point_step = pc_msg.point_step
        
        for i in range(0, len(pc_msg.data), point_step):
            point_data = pc_msg.data[i:i+point_step]
            x, y, z = struct.unpack('fff', bytes(point_data))
            points.append((x, y, z))
        
        if not points:
            self.get_logger().warn('Received empty pointcloud')
            return
        
        # Initialize scan with inf
        ranges = [float('inf')] * self.num_ranges
        intensities = [0.0] * self.num_ranges
        
        # Convert points to scan
        converted_count = 0
        for x, y, z in points:
            # Calculate angle and distance in XY plane
            angle = math.atan2(y, x)
            distance = math.sqrt(x*x + y*y)
            
            # Check if in range
            if angle < self.angle_min or angle > self.angle_max:
                continue
            if distance < self.range_min or distance > self.range_max:
                continue
            
            # Calculate index
            index = int((angle - self.angle_min) / self.angle_increment)
            
            # Bounds check
            if 0 <= index < self.num_ranges:
                # Keep closest point for this angle
                if distance < ranges[index]:
                    ranges[index] = distance
                    converted_count += 1
        
        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = pc_msg.header  # Copy header (frame_id, timestamp)
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        
        # Publish
        self.scan_pub.publish(scan_msg)
        
        # Log statistics (only occasionally)
        if converted_count > 0:
            valid_ranges = [r for r in ranges if math.isfinite(r)]
            self.get_logger().info(
                f'Converted {len(points)} points → {len(valid_ranges)} valid ranges '
                f'(frame: {pc_msg.header.frame_id})',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().warn(
                f'Converted {len(points)} points but got 0 valid ranges!',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = SimplePC2Scan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()