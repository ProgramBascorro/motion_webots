#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')

        self.declare_parameter('image_topic', '/image_raw')
        topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10)

        self.get_logger().info(f"Viewing images from: {topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Utrabot Vision Viewer", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
