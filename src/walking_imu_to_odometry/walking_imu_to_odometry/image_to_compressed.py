#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class ImageToCompressed(Node):
    def __init__(self):
        super().__init__('image_to_compressed')
        self.get_logger().info('✅ image_to_compressed node started')

        # subscribe ke raw image Webots
        self.sub = self.create_subscription(
            Image,
            '/robotis_op3/camera/image_raw',
            self.image_cb,
            10
        )

        # publish ke topic yang dibutuhkan ball_detector
        self.pub = self.create_publisher(
            CompressedImage,
            '/robotis_op3/camera/image_raw/compressed',
            10
        )

    def image_cb(self, msg: Image):
        # convert ROS Image -> numpy BGR
        try:
            if msg.encoding not in ['rgb8', 'bgr8']:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            # buat array dari data
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width, -1)

            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # kompres ke jpeg
            ok, enc = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                self.get_logger().warn('Failed to encode image')
                return

            comp = CompressedImage()
            comp.header = msg.header
            comp.format = 'jpeg'
            comp.data = enc.tobytes()

            self.pub.publish(comp)
        except Exception as e:
            self.get_logger().error(f'Error in image_cb: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageToCompressed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
