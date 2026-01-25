#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller import Supervisor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class GTSupervisor(Node):
    def __init__(self):
        super().__init__("gt_supervisor_node")

        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.robot_node = self.supervisor.getFromDef("OP3")
        if self.robot_node is None:
            self.get_logger().error("Cannot find DEF OP3 in world file")
            return

        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Ground truth supervisor started.")

    def update(self):
        if self.supervisor.step(self.timestep) == -1:
            return

        pos = self.robot_node.getPosition()        # [x, y, z]
        rot = self.robot_node.getOrientation()    # 3x3 matrix (row-major)

        # yaw dari rotation matrix
        yaw = math.atan2(rot[3], rot[0])

        # Quaternion (z, w cukup untuk planar)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        now = self.get_clock().now().to_msg()

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GTSupervisor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
