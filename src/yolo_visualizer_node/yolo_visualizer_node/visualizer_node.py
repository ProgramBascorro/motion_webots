#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters # For synchronizing topics

class YoloVisualizerNode(Node):
    def __init__(self):
        super().__init__('yolo_visualizer_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/robotis_op3/camera/image_raw')
        self.declare_parameter('input_detections_topic', '/yolo/detections')
        self.declare_parameter('output_image_topic', '/yolo/image_detections')
        self.declare_parameter('queue_size', 10)

        # Get Param Values
        image_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('input_detections_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Publisher for annotated image
        self.image_pub = self.create_publisher(RosImage, output_topic, 10)

        # Subscribers using Message Filters for approximate time synchronization
        self.image_sub = message_filters.Subscriber(self, RosImage, image_topic)
        self.detections_sub = message_filters.Subscriber(self, Detection2DArray, detections_topic)

        # Time Synchronizer
        # Adjust slop parameter (seconds) based on expected time difference
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detections_sub], queue_size, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info(f"Visualizer node started. Syncing '{image_topic}' and '{detections_topic}'. Publishing to '{output_topic}'.")


    def synced_callback(self, img_msg: RosImage, det_msg: Detection2DArray):
        """Callback for synchronized image and detection messages."""
        self.get_logger().debug("Received synchronized messages.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Draw detections
        for detection in det_msg.detections:
            # Extract BBox info (center, size)
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y

            # Convert to top-left (pt1) and bottom-right (pt2) coordinates for cv2.rectangle
            pt1_x = int(center_x - size_x / 2)
            pt1_y = int(center_y - size_y / 2)
            pt2_x = int(center_x + size_x / 2)
            pt2_y = int(center_y + size_y / 2)

            # Draw rectangle
            cv2.rectangle(cv_image, (pt1_x, pt1_y), (pt2_x, pt2_y), (0, 255, 0), 2) # Green box, thickness 2

            # Prepare label text (Class ID and Score)
            label = "Detection" # Default
            score_str = ""
            if detection.results: # Check if hypothesis exists
                hypothesis = detection.results[0].hypothesis
                label = hypothesis.class_id if hypothesis.class_id else "Unknown"
                score_str = f"{hypothesis.score:.2f}"

            # Draw label background and text
            text = f"{label}: {score_str}"
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(cv_image, (pt1_x, pt1_y - text_height - baseline), (pt1_x + text_width, pt1_y), (0, 255, 0), -1) # Filled green background
            cv2.putText(cv_image, text, (pt1_x, pt1_y - baseline // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA) # Black text

        # Convert annotated CV image back to ROS Image message
        try:
            annotated_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            # Ensure the header matches the input image header for timestamp/frame_id
            annotated_img_msg.header = img_msg.header
            self.image_pub.publish(annotated_img_msg)
        except CvBridgeError as e:
             self.get_logger().error(f"CV Bridge Error (publishing): {e}")
        except Exception as e:
             self.get_logger().error(f"Error publishing annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
