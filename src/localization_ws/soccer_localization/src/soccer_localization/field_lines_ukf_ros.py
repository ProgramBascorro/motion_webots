#!/usr/bin/env python3
import copy
import os
import time
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import scipy
import sensor_msgs_py.point_cloud2 as pcl2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from soccer_localization.field import Field
from soccer_localization.field_lines_ukf import FieldLinesUKF

from soccer_common import Transformation

# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
from soccer_msgs.msg import RobotState


class FieldLinesUKFROS(FieldLinesUKF, Node):
    def __init__(self, map=Field()):
        # Initialize Node first
        Node.__init__(self, 'soccer_localization')
        
        # Then initialize FieldLinesUKF
        FieldLinesUKF.__init__(self)
        
        self.ukf_lock = Lock()

        # QoS Profile for subscriptions and publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions with ROS2 syntax
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "odom_combined",
            self.odom_callback,
            qos_profile
        )
        
        self.field_point_cloud_subscription = self.create_subscription(
            PointCloud2,
            "field_point_cloud",
            self.field_point_cloud_callback,
            qos_profile
        )
        
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "initialpose",
            self.initial_pose_callback,
            qos_profile
        )
        
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            "state",
            self.robot_state_callback,
            qos_profile
        )

        # Create publishers with ROS2 syntax
        self.field_point_cloud_transformed_publisher = self.create_publisher(
            PointCloud2,
            "field_point_cloud_transformed",
            qos_profile
        )
        
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "amcl_pose",
            qos_profile
        )

        self.map = map

        self.initial_pose = Transformation(pos_theta=[0, 0, 0])
        self.ukf.x = self.initial_pose.pos_theta

        self.odom_t_previous = None

        # TF2 Broadcaster
        self.br = TransformBroadcaster(self)
        
        # ROS2 time handling
        self.timestamp_last = self.get_clock().now()

        self.robot_state = RobotState()

        self.get_logger().info("Soccer Localization UKF initiated")

    def robot_state_callback(self, robot_state: RobotState):
        with self.ukf_lock:
            self.robot_state = robot_state

            if self.robot_state.status in [RobotState.STATUS_PENALIZED, RobotState.STATUS_DISCONNECTED]:
                self.initial_pose = Transformation(pos_theta=[0, 0, 0])
                self.ukf.x = np.array([0, 0, 0])
                self.ukf.P = np.diag([0.0004, 0.0004, 0.002])
                self.odom_t_previous = None

    def odom_callback(self, pose_msg: PoseWithCovarianceStamped):
        with self.ukf_lock:
            if self.robot_state.status not in [
                RobotState.STATUS_LOCALIZING,
                RobotState.STATUS_READY,
                RobotState.STATUS_WALKING,
            ]:
                return

            if self.odom_t_previous is None:
                self.odom_t_previous = Transformation(pose_with_covariance_stamped=pose_msg)
                self.odom_t_previous.orientation_euler = [self.odom_t_previous.orientation_euler[0], 0, 0]  # Needed to remove non yaw values
                return
            
            odom_t = Transformation(pose_with_covariance_stamped=pose_msg)
            odom_t.orientation_euler = [odom_t.orientation_euler[0], 0, 0]  # Needed to remove non yaw values

            diff_transformation: Transformation = scipy.linalg.inv(self.odom_t_previous) @ odom_t
            
            # ROS2 time handling
            dt_nsec = (odom_t.timestamp.sec - self.odom_t_previous.timestamp.sec) * 1e9 + \
                      (odom_t.timestamp.nanosec - self.odom_t_previous.timestamp.nanosec)
            dt_secs = dt_nsec * 1e-9
            
            # ===== FIX START: Validate dt to prevent numerical issues =====
            MIN_DT = 0.01  # 10ms minimum (prevents too frequent updates)
            MAX_DT = 1.0   # 1s maximum (cap large gaps)
            
            if dt_secs < MIN_DT:
                # Too frequent, skip this update to prevent numerical instability
                return
            
            if dt_secs > MAX_DT:
                # Too long gap, cap it
                self.get_logger().warn(
                    f"Large dt detected: {dt_secs:.2f}s, capping to {MAX_DT}s", 
                    throttle_duration_sec=1.0
                )
                dt_secs = MAX_DT
            
            if dt_secs <= 0:
                return
            # ===== FIX END =====

            if self.robot_state.status == RobotState.STATUS_LOCALIZING:
                self.ukf.Q = self.Q_localizing
                self.ukf.R = self.R_localizing
            elif self.robot_state.status == RobotState.STATUS_READY:
                self.ukf.Q = self.Q_ready
                self.ukf.R = self.R_ready
            else:
                self.ukf.Q = self.Q_walking
                self.ukf.R = self.R_walking

            # ===== FIX START: Error handling for predict =====
            try:
                self.predict(u=diff_transformation.pos_theta / dt_secs, dt=dt_secs)
            except np.linalg.LinAlgError as e:
                self.get_logger().error(
                    f"UKF predict failed (LinAlgError): {e}", 
                    throttle_duration_sec=1.0
                )
                # Reset covariance to safe values
                self.ukf.P = np.diag([0.1, 0.1, 0.05])
                return
            except Exception as e:
                self.get_logger().error(
                    f"UKF predict failed (unexpected): {e}", 
                    throttle_duration_sec=1.0
                )
                return
            # ===== FIX END =====

            self.odom_t_previous = odom_t

            self.broadcast_tf_position(pose_msg.header.stamp)
            self.publish_amcl_pose(timestamp=pose_msg.header.stamp)

            return odom_t

    def field_point_cloud_callback(self, point_cloud_msg: PointCloud2):
        with self.ukf_lock:
            if self.robot_state.status not in [
                RobotState.STATUS_LOCALIZING,
                RobotState.STATUS_READY,
                RobotState.STATUS_WALKING,
            ]:
                return None, None, None

            stamp = point_cloud_msg.header.stamp
            point_cloud = pcl2.read_points_list(point_cloud_msg)
            point_cloud_array = np.array(point_cloud)
            current_transform = Transformation(pos_theta=self.ukf.x)

            iterations = 3
            if self.robot_state.status == RobotState.STATUS_LOCALIZING:
                iterations = 10
            tt = self.map.matchPointsWithMapIterative(
                current_transform, point_cloud_array, iterations, localizing=self.robot_state.status == RobotState.STATUS_LOCALIZING
            )

            if tt is not None:
                (offset_transform, transform_confidence) = tt
                vo_transform = current_transform @ offset_transform
                vo_pos_theta = vo_transform.pos_theta
                self.update(vo_pos_theta, transform_confidence)
                self.broadcast_tf_position(timestamp=stamp)
                self.broadcast_vo_transform_debug(vo_transform, point_cloud_msg)
                self.publish_amcl_pose(timestamp=stamp)

                return point_cloud_array, vo_transform, vo_pos_theta
            return None, None, None

    def broadcast_vo_transform_debug(self, vo_transform: Transformation, point_cloud: PointCloud2):
        # Create TransformStamped message for TF2
        t = TransformStamped()
        t.header.stamp = point_cloud.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = f"{os.environ.get('ROS_NAMESPACE', 'robot1')}/odom_vo"
        
        # Set translation
        t.transform.translation.x = vo_transform.position[0]
        t.transform.translation.y = vo_transform.position[1]
        t.transform.translation.z = vo_transform.position[2]
        
        # Set rotation
        t.transform.rotation.x = vo_transform.quaternion[0]
        t.transform.rotation.y = vo_transform.quaternion[1]
        t.transform.rotation.z = vo_transform.quaternion[2]
        t.transform.rotation.w = vo_transform.quaternion[3]
        
        self.br.sendTransform(t)
        
        point_cloud.header.frame_id = f"{os.environ.get('ROS_NAMESPACE', '/robot1').replace('/', '')}/odom_vo"
        self.field_point_cloud_transformed_publisher.publish(point_cloud)

    def broadcast_tf_position(self, timestamp):
        if self.odom_t_previous is None:
            # ROS2 throttle logging
            self.get_logger().warn("Odom not published", throttle_duration_sec=1.0)
            return

        # Prevent rebroadcasting same or older timestamp
        timestamp_ros2 = rclpy.time.Time.from_msg(timestamp)
        if timestamp_ros2 <= self.timestamp_last:
            return
        else:
            self.timestamp_last = timestamp_ros2

        world_to_odom = Transformation(pos_theta=self.ukf.x) @ scipy.linalg.inv(self.odom_t_previous)

        # Create TransformStamped message for TF2
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "world"
        t.child_frame_id = f"{os.environ.get('ROS_NAMESPACE', 'robot1')}/odom"
        
        # Set translation
        t.transform.translation.x = world_to_odom.position[0]
        t.transform.translation.y = world_to_odom.position[1]
        t.transform.translation.z = world_to_odom.position[2]
        
        # Set rotation
        t.transform.rotation.x = world_to_odom.quaternion[0]
        t.transform.rotation.y = world_to_odom.quaternion[1]
        t.transform.rotation.z = world_to_odom.quaternion[2]
        t.transform.rotation.w = world_to_odom.quaternion[3]
        
        self.br.sendTransform(t)

    def publish_amcl_pose(self, timestamp):
        amcl_pose = Transformation(pos_theta=self.ukf.x, pose_theta_covariance_array=self.ukf.P).pose_with_covariance_stamped
        amcl_pose.header.stamp = timestamp
        self.amcl_pose_publisher.publish(amcl_pose)

    def initial_pose_callback(self, pose_stamped: PoseWithCovarianceStamped):
        with self.ukf_lock:
            self.initial_pose = Transformation(pose_with_covariance_stamped=pose_stamped)
            self.ukf.x = self.initial_pose.pos_theta
            self.ukf.P = self.initial_pose.pose_theta_covariance_array
            self.odom_t_previous = None
            self.get_logger().info(f"Reinitialized with x = {self.ukf.x} and P = {np.diag(self.ukf.P)}")
            self.broadcast_tf_position(pose_stamped.header.stamp)