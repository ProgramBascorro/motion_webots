#!/usr/bin/env python3
"""
Particle Cloud Message Converter
Converts nav2_msgs/ParticleCloud to geometry_msgs/PoseArray for RViz

USAGE:
  As standalone: python3 particle_converter.py
  In launch file: Add as Node with executable='particle_converter.py'
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray


class ParticleConverter(Node):
    """
    Converts Nav2 ParticleCloud to standard PoseArray for RViz visualization
    
    Subscribe: /particle_cloud (nav2_msgs/ParticleCloud, BEST_EFFORT)
    Publish: /particle_cloud_viz (geometry_msgs/PoseArray, RELIABLE)
    """
    
    def __init__(self):
        super().__init__('particle_converter')
        
        # QoS for Nav2 messages (match AMCL publisher)
        nav2_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for RViz (standard)
        rviz_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to nav2_msgs/ParticleCloud
        self.sub = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.callback,
            nav2_qos
        )
        
        # Publish as geometry_msgs/PoseArray
        self.pub = self.create_publisher(
            PoseArray,
            '/particle_cloud_viz',
            rviz_qos
        )
        
        self.msg_count = 0
        self.last_particle_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Particle Cloud Converter Started')
        self.get_logger().info('  Subscribe: /particle_cloud (nav2_msgs/ParticleCloud)')
        self.get_logger().info('  Publish:   /particle_cloud_viz (geometry_msgs/PoseArray)')
        self.get_logger().info('  QoS In:    BEST_EFFORT (match AMCL)')
        self.get_logger().info('  QoS Out:   RELIABLE (match RViz)')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for particle cloud messages...')
    
    def callback(self, nav2_msg: ParticleCloud):
        """
        Convert nav2_msgs/ParticleCloud to geometry_msgs/PoseArray
        
        Args:
            nav2_msg: Input ParticleCloud message from AMCL
        """
        
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header = nav2_msg.header
        
        # Extract poses from particles
        # ParticleCloud contains: particles[].pose (Pose), particles[].weight (float)
        # PoseArray just needs: poses[] (Pose)
        for particle in nav2_msg.particles:
            pose_array.poses.append(particle.pose)
        
        # Publish converted message
        self.pub.publish(pose_array)
        
        # Statistics
        self.msg_count += 1
        particle_count = len(pose_array.poses)
        
        # Log on first message
        if self.msg_count == 1:
            self.get_logger().info(
                f'✅ First message converted! {particle_count} particles'
            )
        
        # Log every 50 messages or if particle count changes significantly
        if (self.msg_count % 50 == 0 or 
            abs(particle_count - self.last_particle_count) > 100):
            self.get_logger().info(
                f'Converted {self.msg_count} messages, {particle_count} particles'
            )
            self.last_particle_count = particle_count


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ParticleConverter()
        
        # Log connection status after 2 seconds
        import time
        time.sleep(2)
        
        if node.msg_count == 0:
            node.get_logger().warn(
                'No messages received yet. Check:'
            )
            node.get_logger().warn('  1. Is AMCL running? (ros2 node list | grep amcl)')
            node.get_logger().warn('  2. Is AMCL publishing? (ros2 topic hz /particle_cloud)')
            node.get_logger().warn('  3. Has initial pose been set?')
        
        # Spin
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n✅ Particle converter shutting down...')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()