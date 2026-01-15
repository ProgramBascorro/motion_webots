import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from soccer_common.transformation import Transformation
from soccer_object_detection.camera.camera_calculations import CameraCalculations


class CameraCalculationsRos(CameraCalculations):
    def __init__(self, node, base_frame: str = "base", camera_frame: str = "cam_link", camera_info_topic: str = "/robotis_op3/camera/camera_info"):
        super().__init__()
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.camera_info_topic = camera_info_topic
        self.camera_info = CameraInfo()
        self.node.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def camera_info_callback(self, camera_info: CameraInfo):
        self.camera_info = camera_info

    def reset_position(self, timestamp=None, skip_if_not_found=False) -> bool:
        if timestamp is None:
            timestamp = rclpy.time.Time()

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                timestamp,
                timeout=Duration(seconds=0.1),
            ):
                return False

            transform_stamped = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, timestamp)
            trans = transform_stamped.transform.translation
            rot = transform_stamped.transform.rotation

            self.pose = Transformation(position=[trans.x, trans.y, trans.z], quaternion=[rot.x, rot.y, rot.z, rot.w])
            return True
        except Exception as exc:  # noqa: BLE001
            if not skip_if_not_found:
                self.node.get_logger().warn(f"Failed to lookup {self.base_frame} -> {self.camera_frame}: {exc}")
            return False
