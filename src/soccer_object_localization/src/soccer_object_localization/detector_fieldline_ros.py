#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge

from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_localization.detector_fieldline import DetectorFieldline


class FieldLineDetectorNode(Node):
    def __init__(self):
        super().__init__("fieldline_detector")

        self.declare_parameter("image_topic", "/robotis_op3/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/robotis_op3/camera/camera_info")
        self.declare_parameter("debug_image_topic", "/vision/lines/debug")
        self.declare_parameter("point_cloud_topic", "/field_point_cloud")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("camera_frame", "cam_link")
        self.declare_parameter("point_cloud_max_distance", 5.0)
        self.declare_parameter("point_cloud_spacing", 30)
        self.declare_parameter("publish_debug_image", True)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        point_cloud_topic = self.get_parameter("point_cloud_topic").value
        base_frame = self.get_parameter("base_frame").value
        camera_frame = self.get_parameter("camera_frame").value

        self.bridge = CvBridge()
        self.detector = DetectorFieldline(
            camera=CameraCalculationsRos(
                node=self,
                base_frame=base_frame,
                camera_frame=camera_frame,
                camera_info_topic=camera_info_topic,
            )
        )
        self.detector.point_cloud_max_distance = float(self.get_parameter("point_cloud_max_distance").value)
        self.detector.point_cloud_spacing = int(self.get_parameter("point_cloud_spacing").value)

        self.image_pub = self.create_publisher(Image, debug_image_topic, 1)
        self.cloud_pub = self.create_publisher(PointCloud2, point_cloud_topic, 1)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.base_frame = base_frame
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.get_logger().info("Field line detector ready")

    def image_callback(self, img: Image):
        if not self.detector.camera.reset_position(timestamp=img.header.stamp, skip_if_not_found=True):
            return

        image = self.bridge.imgmsg_to_cv2(img, desired_encoding="rgb8")
        lines_only = self.detector.image_filter(image)

        if self.publish_debug_image and self.image_pub.get_subscription_count() > 0:
            img_out = self.bridge.cv2_to_imgmsg(lines_only)
            img_out.header = img.header
            self.image_pub.publish(img_out)

        points3d = self.detector.img_to_points(lines_only)
        if not points3d:
            return

        header = Header()
        header.stamp = img.header.stamp
        header.frame_id = self.base_frame
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points3d)
        self.cloud_pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = FieldLineDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
