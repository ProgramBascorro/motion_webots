#!/usr/bin/env python3
"""
camera_info_publisher.py — Publish /camera_info dari file YAML kalibrasi
=========================================================================
Masalah: Webots OP3 tidak otomatis publish /robotis_op3/camera/camera_info
dengan data kalibrasi kita. Node ini membaca camera.yaml hasil kalibrasi
dan publish ke topik yang dibutuhkan oleh image_proc/rectify_node.

PENGGUNAAN:
  python3 camera_info_publisher.py

  Atau via launch file (sudah termasuk di localization_v10.launch.py):
  Node(package='soccer_object_localization',
       executable='camera_info_publisher', ...)

TOPIK:
  Publish: /robotis_op3/camera/camera_info (sensor_msgs/CameraInfo)
  Subscribe: /robotis_op3/camera/image_raw (untuk sync timestamp)

KONFIGURASI:
  Parameter 'camera_yaml_path': path ke file YAML hasil kalibrasi
  Default: ~/ros2_ws/src/soccer_object_localization/config/camera.yaml
  Fallback: ~/.ros/camera_info/camera.yaml

PENTING: Jika kalibrasi belum dilakukan, node ini akan publish
  CameraInfo dengan nilai default pinhole (fx=fy=900, cx=640, cy=360)
  yang cukup untuk pengujian awal, tapi tidak untuk akurasi penuh.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


DEFAULT_FX = 900.0   # default focal length (estimasi dari parameter kita sebelumnya)
DEFAULT_FY = 900.0
DEFAULT_CX = 640.0   # cx = image_width / 2
DEFAULT_CY = 360.0   # cy = image_height / 2


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Parameter
        self.declare_parameter('camera_yaml_path', '')
        self.declare_parameter('image_width',  1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('camera_frame_id', 'cam_link')

        yaml_path = self.get_parameter('camera_yaml_path').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.frame = self.get_parameter('camera_frame_id').value

        # Load kalibrasi
        self.cam_info = self._load_or_default(yaml_path)

        # Pub / Sub
        self.pub_info = self.create_publisher(
            CameraInfo, '/robotis_op3/camera/camera_info', 10)

        self.sub_img = self.create_subscription(
            Image, '/robotis_op3/camera/image_raw',
            self._cb_image, qos_profile_sensor_data)

        # Juga publish periodik (backup jika image_raw lambat)
        self.timer = self.create_timer(0.1, self._cb_timer)

        self.get_logger().info('CameraInfoPublisher started')
        self.get_logger().info(
            f'  fx={self.cam_info.k[0]:.1f}  fy={self.cam_info.k[4]:.1f}  '
            f'cx={self.cam_info.k[2]:.1f}  cy={self.cam_info.k[5]:.1f}')

    def _cb_image(self, msg):
        """Publish camera_info dengan timestamp sinkron dengan image_raw."""
        self.cam_info.header.stamp    = msg.header.stamp
        self.cam_info.header.frame_id = self.frame
        self.pub_info.publish(self.cam_info)

    def _cb_timer(self):
        """Publish periodik agar downstream tidak menunggu."""
        self.cam_info.header.stamp    = self.get_clock().now().to_msg()
        self.cam_info.header.frame_id = self.frame
        self.pub_info.publish(self.cam_info)

    def _load_or_default(self, yaml_path):
        info = CameraInfo()
        info.width  = self.img_w
        info.height = self.img_h
        info.distortion_model = 'plumb_bob'

        # Cari file YAML
        paths_to_try = []
        if yaml_path:
            paths_to_try.append(yaml_path)
        paths_to_try += [
            os.path.expanduser('~/soccer_object_localization/config/camera.yaml'),
            os.path.expanduser('~/.ros/camera_info/camera.yaml'),
            os.path.expanduser('~/calib_images/calibration_result.yaml'),
        ]

        for path in paths_to_try:
            if os.path.exists(path):
                try:
                    with open(path) as f:
                        data = yaml.safe_load(f)
                    K = data['camera_matrix']['data']
                    D = data['distortion_coefficients']['data']
                    P = data.get('projection_matrix', {}).get('data', None)
                    R = data.get('rectification_matrix', {}).get('data', None)

                    info.k = K
                    info.d = D
                    info.p = P if P else [K[0],0,K[2],0, 0,K[4],K[5],0, 0,0,1,0]
                    info.r = R if R else [1,0,0, 0,1,0, 0,0,1]

                    self.get_logger().info(f'Loaded calibration: {path}')
                    return info
                except Exception as e:
                    self.get_logger().warn(f'Gagal load {path}: {e}')

        # Default: perkiraan pinhole tanpa distorsi
        self.get_logger().warn(
            'Camera.yaml tidak ditemukan! Menggunakan nilai default.\n'
            '  Jalankan calibrate_camera_offline.py untuk kalibrasi formal.\n'
            f'  fx=fy={DEFAULT_FX}, cx={DEFAULT_CX}, cy={DEFAULT_CY}')
        info.k = [DEFAULT_FX, 0.0, DEFAULT_CX,
                  0.0, DEFAULT_FY, DEFAULT_CY,
                  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.p = [DEFAULT_FX, 0.0, DEFAULT_CX, 0.0,
                  0.0, DEFAULT_FY, DEFAULT_CY, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return info


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()