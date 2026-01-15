import threading
from typing import Optional

import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from soccer_common import Transformation
from soccer_localization.field import Field
from soccer_localization.field_lines_ukf import FieldLinesUKF


class FieldLinesUkfNode(Node):
    def __init__(self):
        super().__init__("field_lines_ukf")

        self.ukf_lock = threading.Lock()

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("field_point_cloud_topic", "/field_point_cloud")
        self.declare_parameter("field_point_cloud_map_topic", "/field_point_cloud_transformed")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("pose_topic", "/localization/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("vo_frame", "odom_vo")
        self.declare_parameter("publish_vo_debug", True)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_covariance", [0.0004, 0.0004, 0.002])
        self.declare_parameter("match_iterations", 3)

        self.declare_parameter("distance_point_threshold", 5.0)
        self.declare_parameter("min_points_threshold", 40)
        self.declare_parameter("max_detected_line_parallel_offset_error", 0.1)
        self.declare_parameter("max_detected_line_perpendicular_offset_error", 0.3)
        self.declare_parameter("offset_movement_limit", 0.2)
        self.declare_parameter("max_detected_line_parallel_offset_error_localizing", 0.5)
        self.declare_parameter("max_detected_line_perpendicular_offset_error_localizing", 0.5)
        self.declare_parameter("offset_movement_limit_localizing", 0.5)

        field_params = {
            "distance_point_threshold": float(self.get_parameter("distance_point_threshold").value),
            "min_points_threshold": int(self.get_parameter("min_points_threshold").value),
            "max_detected_line_parallel_offset_error": float(
                self.get_parameter("max_detected_line_parallel_offset_error").value
            ),
            "max_detected_line_perpendicular_offset_error": float(
                self.get_parameter("max_detected_line_perpendicular_offset_error").value
            ),
            "offset_movement_limit": float(self.get_parameter("offset_movement_limit").value),
            "max_detected_line_parallel_offset_error_localizing": float(
                self.get_parameter("max_detected_line_parallel_offset_error_localizing").value
            ),
            "max_detected_line_perpendicular_offset_error_localizing": float(
                self.get_parameter("max_detected_line_perpendicular_offset_error_localizing").value
            ),
            "offset_movement_limit_localizing": float(self.get_parameter("offset_movement_limit_localizing").value),
        }

        self.map = Field(params=field_params, logger=self.get_logger())
        self.filter = FieldLinesUKF()

        initial_pose = self.get_parameter("initial_pose").value
        initial_cov = self.get_parameter("initial_covariance").value
        if len(initial_pose) == 3:
            self.filter.ukf.x = np.array(initial_pose, dtype=float)
        if len(initial_cov) == 3:
            self.filter.ukf.P = np.diag([float(initial_cov[0]), float(initial_cov[1]), float(initial_cov[2])])

        self.odom_t_previous: Optional[Transformation] = None
        self.timestamp_last = None

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.vo_frame = self.get_parameter("vo_frame").value
        self.publish_vo_debug = bool(self.get_parameter("publish_vo_debug").value)

        self.tf_broadcaster = TransformBroadcaster(self)

        odom_topic = self.get_parameter("odom_topic").value
        cloud_topic = self.get_parameter("field_point_cloud_topic").value
        initialpose_topic = self.get_parameter("initialpose_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        cloud_map_topic = self.get_parameter("field_point_cloud_map_topic").value

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, cloud_topic, self.field_point_cloud_callback, 1)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, initialpose_topic, self.initial_pose_callback, 1
        )
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.cloud_map_pub = self.create_publisher(PointCloud2, cloud_map_topic, 1)

        self.get_logger().info("Field line UKF localization ready")

    def odom_callback(self, msg: Odometry):
        with self.ukf_lock:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose

            if self.odom_t_previous is None:
                self.odom_t_previous = Transformation(pose_with_covariance_stamped=pose_msg)
                yaw = self.odom_t_previous.orientation_euler[0]
                self.odom_t_previous.orientation_euler = [yaw, 0.0, 0.0]
                return

            odom_t = Transformation(pose_with_covariance_stamped=pose_msg)
            yaw = odom_t.orientation_euler[0]
            odom_t.orientation_euler = [yaw, 0.0, 0.0]

            diff_transformation: Transformation = scipy.linalg.inv(self.odom_t_previous) @ odom_t
            dt_secs = self._delta_seconds(self.odom_t_previous.timestamp, odom_t.timestamp)
            if dt_secs <= 0.0:
                return

            self.predict(u=diff_transformation.pos_theta / dt_secs, dt=dt_secs)

            self.odom_t_previous = odom_t

            self.broadcast_tf_position(msg.header.stamp)
            self.publish_pose(msg.header.stamp)

    def field_point_cloud_callback(self, point_cloud_msg: PointCloud2):
        with self.ukf_lock:
            points = point_cloud2.read_points_numpy(
                point_cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            if points.size == 0:
                return

            point_cloud_array = np.atleast_2d(np.asarray(points, dtype=float))
            current_transform = Transformation(pos_theta=self.filter.ukf.x)

            iterations = int(self.get_parameter("match_iterations").value)
            tt = self.map.matchPointsWithMapIterative(current_transform, point_cloud_array, iterations, localizing=False)

            if tt is None:
                return

            offset_transform, transform_confidence = tt
            vo_transform = current_transform @ offset_transform
            vo_pos_theta = vo_transform.pos_theta
            self.update(vo_pos_theta, transform_confidence)
            self.broadcast_tf_position(point_cloud_msg.header.stamp)
            self.publish_pose(point_cloud_msg.header.stamp)

            if self.publish_vo_debug:
                self.broadcast_vo_transform_debug(vo_transform, point_cloud_array, point_cloud_msg.header.stamp)

    def broadcast_vo_transform_debug(self, vo_transform: Transformation, points: np.ndarray, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.vo_frame
        t.transform.translation.x = float(vo_transform.position[0])
        t.transform.translation.y = float(vo_transform.position[1])
        t.transform.translation.z = float(vo_transform.position[2])
        q = vo_transform.quaternion
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

        rot = vo_transform.matrix[0:2, 0:2]
        trans = vo_transform.position[0:2]
        pts_xy = (rot @ points[:, 0:2].T).T + trans
        pts_map = np.column_stack((pts_xy, points[:, 2]))
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=self._make_header(stamp, self.map_frame),
            points=pts_map.tolist(),
        )
        self.cloud_map_pub.publish(cloud_msg)

    def broadcast_tf_position(self, stamp):
        if self.odom_t_previous is None:
            return

        if self.timestamp_last is not None and self._time_from_msg(stamp) <= self.timestamp_last:
            return
        self.timestamp_last = self._time_from_msg(stamp)

        world_to_odom = Transformation(pos_theta=self.filter.ukf.x) @ scipy.linalg.inv(self.odom_t_previous)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(world_to_odom.position[0])
        t.transform.translation.y = float(world_to_odom.position[1])
        t.transform.translation.z = float(world_to_odom.position[2])
        q = world_to_odom.quaternion
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self, stamp):
        pose_msg = Transformation(
            pos_theta=self.filter.ukf.x, pose_theta_covariance_array=self.filter.ukf.P
        ).pose_with_covariance_stamped
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        self.pose_pub.publish(pose_msg)

    def initial_pose_callback(self, pose_stamped: PoseWithCovarianceStamped):
        with self.ukf_lock:
            self.filter.ukf.x = Transformation(pose_with_covariance_stamped=pose_stamped).pos_theta
            self.filter.ukf.P = Transformation(pose_with_covariance_stamped=pose_stamped).pose_theta_covariance_array
            self.odom_t_previous = None
            self.broadcast_tf_position(pose_stamped.header.stamp)

    def predict(self, u, dt):
        self.filter.predict(u=u, dt=dt)

    def update(self, z, transform_confidence):
        self.filter.update(z, transform_confidence)

    @staticmethod
    def _delta_seconds(t0, t1) -> float:
        return (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9

    @staticmethod
    def _time_from_msg(stamp):
        return rclpy.time.Time.from_msg(stamp)

    @staticmethod
    def _make_header(stamp, frame_id: str):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header


def main():
    rclpy.init()
    node = FieldLinesUkfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
