#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessing(Node):
    def __init__(self):
        super().__init__('camera_processing')
        self.declare_parameter('input_image_topic', '/image_raw')
        self.declare_parameter('output_image_topic', '/image_processed')
        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Image, input_topic, self.cb_image, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.bridge = CvBridge()
        self.get_logger().info(f'CameraProcessing started. Subscribing: {input_topic} -> Publishing: {output_topic}')

    def cb_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge convert error: {e}')
            return

        # example processing: grayscale + Canny (replace with your field-line detector)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        vis = cv2.addWeighted(cv_image, 0.8, edges_colored, 0.8, 0)

        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
