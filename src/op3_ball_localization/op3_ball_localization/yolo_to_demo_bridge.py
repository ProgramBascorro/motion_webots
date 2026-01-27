#!/usr/bin/env python3
"""
YOLO to OP3 Demo Bridge
Converts YOLO detections to CircleSetStamped format for op3_demo compatibility.
"""

import rclpy
from rclpy.node import Node
from soccer_msgs.msg import BoundingBoxes
from op3_ball_detector_msgs.msg import CircleSetStamped, Circle2D
from geometry_msgs.msg import Point


class YoloToDemoBridge(Node):
    def __init__(self):
        super().__init__('yolo_to_demo_bridge')

        # Subscribe to YOLO detections
        self.create_subscription(
            BoundingBoxes,
            '/vision/yolo/detections',
            self.detections_callback,
            10
        )

        # Publish in CircleSetStamped format (what op3_demo expects)
        self.circle_pub = self.create_publisher(
            CircleSetStamped,
            '/ball_detector_node/circle_set',
            10
        )

        # Parameters for conversion
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('YOLO to Demo Bridge initialized')
        self.get_logger().info('  Listening: /vision/yolo/detections')
        self.get_logger().info('  Publishing: /ball_detector_node/circle_set')

    def detections_callback(self, msg: BoundingBoxes):
        """Convert YOLO bounding boxes to circles for op3_demo."""
        # Find the ball detection
        ball_bbox = None
        best_confidence = 0.0

        for bbox in msg.bounding_boxes:
            if bbox.class_id == "ball" and bbox.probability > best_confidence:
                ball_bbox = bbox
                best_confidence = bbox.probability

        # Create CircleSetStamped message
        circle_msg = CircleSetStamped()
        circle_msg.header = msg.header

        if ball_bbox is not None:
            # Convert bbox to circle
            circle = Circle2D()

            # Calculate center
            cx = (ball_bbox.xmin + ball_bbox.xmax) / 2.0
            cy = (ball_bbox.ymin + ball_bbox.ymax) / 2.0

            # Calculate diameter (average of width and height)
            width = ball_bbox.xmax - ball_bbox.xmin
            height = ball_bbox.ymax - ball_bbox.ymin
            diameter = (width + height) / 2.0

            # Normalize to [-1, 1] range (op3_demo format)
            # x: -1 (left) to +1 (right)
            # y: -1 (top) to +1 (bottom)
            norm_x = (cx / self.image_width) * 2.0 - 1.0
            norm_y = (cy / self.image_height) * 2.0 - 1.0

            # Size as fraction of image
            norm_diameter = diameter / max(self.image_width, self.image_height)

            circle.center = Point(x=norm_x, y=norm_y, z=0.0)
            circle.z = norm_diameter  # Size indicator

            circle_msg.circles = [circle]

            self.get_logger().debug(
                f'Ball detected: center=({norm_x:.2f}, {norm_y:.2f}), '
                f'size={norm_diameter:.3f}, conf={best_confidence:.2f}'
            )

        # Publish (even if empty, to indicate no ball found)
        self.circle_pub.publish(circle_msg)


def main():
    rclpy.init()
    node = YoloToDemoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
