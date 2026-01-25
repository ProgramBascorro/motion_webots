#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import math

class GTOdomNode(Node):
    def __init__(self):
        super().__init__('gt_odom_node')

        self.sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        self.pose2d_pub = self.create_publisher(Pose2D, '/gt_pose_2d', 10)

        self.get_logger().info("GT Localization Node with YAW started.")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(q)

        pose2d = Pose2D()
        pose2d.x = x
        pose2d.y = y
        pose2d.theta = yaw
        self.pose2d_pub.publish(pose2d)

        self.get_logger().info(
            f"GT Pose2D -> x={x:.4f}, y={y:.4f}, yaw={math.degrees(yaw):.2f} deg"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GTOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
