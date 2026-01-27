#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # parameters
        self.declare_parameter('input_image_topic', '/robotis_op3/camera/image_raw')
        self.declare_parameter('output_image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_cfg', 'config/camera.yaml')

        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        cfg_path = self.get_parameter('camera_cfg').get_parameter_value().string_value

        # resolve package-relative path (when camera_cfg is a substitution it becomes _Substitution)
        if hasattr(cfg_path, 'perform') is False and not os.path.isabs(cfg_path):
            # probably a normal string path
            pass

        # allow package substitution objects: try to evaluate to string via str()
        try:
            cfg_path_str = str(cfg_path)
        except Exception:
            cfg_path_str = cfg_path

        # if not absolute, make it relative to package share (we assume user passed full path via launch)
        if not os.path.isabs(cfg_path_str):
            pkg_dir = os.path.dirname(__file__)
            cfg_path = os.path.join(pkg_dir, '..', cfg_path_str)
        else:
            cfg_path = cfg_path_str

        self.camera_info = self._load_camera_info(cfg_path)
        self.bridge = CvBridge()

        self.pub_image = self.create_publisher(Image, out_topic, 10)
        self.pub_caminfo = self.create_publisher(CameraInfo, cam_info_topic, 10)

        self.sub_image = self.create_subscription(Image, input_topic, self.image_cb, 10)

        self.get_logger().info(f'CameraNode started. Subscribing: {input_topic} -> Publishing: {out_topic}, {cam_info_topic}')

    def _load_camera_info(self, path):
        cam_info = CameraInfo()
        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f)
            cam_info.width = cfg.get('image_width', 640)
            cam_info.height = cfg.get('image_height', 480)
            cam_info.distortion_model = cfg.get('distortion_model', 'plumb_bob')
            cam_info.d = cfg.get('distortion_coefficients', {}).get('data', [0.0,0.0,0.0,0.0,0.0])
            cam_info.k = cfg.get('camera_matrix', {}).get('data', [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0])
            cam_info.r = cfg.get('rectification_matrix', {}).get('data', [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0])
            cam_info.p = cfg.get('projection_matrix', {}).get('data', [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0])
        except Exception as e:
            self.get_logger().warn(f'Could not load camera config {path}: {e}. Using defaults.')
            cam_info.width = 640
            cam_info.height = 480
            cam_info.distortion_model = 'plumb_bob'
            cam_info.d = [0.0]*5
            cam_info.k = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
            cam_info.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
            cam_info.p = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0]
        return cam_info

    def image_cb(self, msg: Image):
        # Use ROS time for header stamp (especially in simulation)
        now = self.get_clock().now().to_msg()
        out = Image()
        out.header.stamp = now
        out.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data

        # publish image and camera info
        self.pub_image.publish(out)

        caminfo = self.camera_info
        caminfo.header.stamp = now
        caminfo.header.frame_id = out.header.frame_id
        self.pub_caminfo.publish(caminfo)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
