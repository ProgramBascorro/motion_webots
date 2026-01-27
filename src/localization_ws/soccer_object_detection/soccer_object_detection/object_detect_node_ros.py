#!/usr/bin/env python3
"""
Object Detection Node ROS2 - Compatible with existing UTRA code
Bridges ObjectDetectionNode with ROS2 Humble
"""

import os
from os.path import expanduser
from argparse import ArgumentParser

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import torch
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# Import custom modules
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_detection.object_detect_node import ObjectDetectionNode, bcolors
from soccer_msgs.msg import BoundingBoxes

# Set default namespace if not exists
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"


class ObjectDetectionNodeRos(ObjectDetectionNode, Node):
    """
    ROS2 bridge for object detection
    Input: 480x640x4 bgra8 -> Output: detections with bounding boxes
    """

    def __init__(self, model_path):
        # Initialize ROS2 Node first
        Node.__init__(self, "object_detector")
        
        # Initialize ObjectDetectionNode
        ObjectDetectionNode.__init__(self, model_path)
        
        # Declare and get parameters
        self.declare_parameter("cover_horizon_up_threshold", 30)
        self.cover_horizon_up_threshold = self.get_parameter("cover_horizon_up_threshold").value
        
        self.declare_parameter("ball_confidence_threshold", 0.75)
        self.CONFIDENCE_THRESHOLD = self.get_parameter("ball_confidence_threshold").value

        # Log CUDA availability
        if torch.cuda.is_available():
            self.get_logger().info(
                f"{bcolors.OKGREEN}Using CUDA for object detection{bcolors.ENDC}"
            )
        else:
            self.get_logger().info(
                f"{bcolors.WARNING}CUDA not available, using CPU{bcolors.ENDC}"
            )

        # Get robot name from namespace
        self.robot_name = self.get_namespace().strip("/")  # Remove leading '/'
        if not self.robot_name:
            self.robot_name = "robot1"
            self.get_logger().warn(f"No namespace set, using default: {self.robot_name}")

        # Initialize camera calculations
        self.camera = CameraCalculationsRos(self, self.robot_name)
        self.camera.reset_position()

        # CV Bridge for image conversion
        self.br = CvBridge()

        # Publishers
        self.pub_ball = self.create_publisher(
            PoseStamped, 
            f"{self.robot_name}/ball", 
            10
        )
        
        self.pub_ball_pixel = self.create_publisher(
            Float32MultiArray, 
            f"{self.robot_name}/ball_pixel", 
            10
        )
        
        self.pub_detection = self.create_publisher(
            Image, 
            f"{self.robot_name}/detection_image", 
            10
        )
        
        self.pub_boundingbox = self.create_publisher(
            BoundingBoxes, 
            f"{self.robot_name}/object_bounding_boxes", 
            10
        )

        # Subscriber - using sensor QoS for camera images
        self.image_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Object Detection Node initialized for robot: {self.robot_name}"
        )
        self.get_logger().info(f"Model loaded from: {model_path}")
        self.get_logger().info(f"Subscribed to: /camera/image_raw")
        self.get_logger().info(f"Publishing to: {self.robot_name}/ball, {self.robot_name}/detection_image")

    def callback(self, msg: Image):
        """
        Main callback for processing incoming camera images
        
        Args:
            msg: ROS2 Image message (bgra8 format, 480x640x4 pixels)
        """
        try:
            # Convert ROS image to OpenCV format
            image = self.br.imgmsg_to_cv2(msg)
            
            # Reset camera position with current timestamp
            self.camera.reset_position(timestamp=msg.header.stamp)

            # Run object detection
            detection_image, bbs_msg = self.get_model_output(image)

            # Set header for bounding boxes message
            bbs_msg.header = msg.header

            # Publish bounding boxes if detections exist
            if len(bbs_msg.bounding_boxes) > 0:
                self.pub_boundingbox.publish(bbs_msg)
                self.get_logger().debug(
                    f"Published {len(bbs_msg.bounding_boxes)} detections",
                    throttle_duration_sec=1.0
                )

            # Publish debug/detection image
            detection_msg = self.br.cv2_to_imgmsg(detection_image, encoding="bgr8")
            detection_msg.header = msg.header
            self.pub_detection.publish(detection_msg)

            # Process ball detections (class "0")
            for box in bbs_msg.bounding_boxes:
                if box.data == "0":  # Ball class
                    self.process_ball_detection(box, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def process_ball_detection(self, box, header):
        """
        Process ball detection and publish position
        
        Args:
            box: BoundingBox message containing ball detection
            header: Original image header for timestamp and frame_id
        """
        try:
            # Calculate bounding box corners
            boundingBoxes = [
                [box.xmin, box.ymin],  # Top-left
                [box.xmax, box.ymax]   # Bottom-right
            ]

            # Publish ball pixel location (image coordinates)
            ball_pixel = Float32MultiArray()
            ball_pixel.data = [
                (box.xmin + box.xmax) / 2.0,  # Center X
                (box.ymin + box.ymax) / 2.0   # Center Y
            ]
            self.pub_ball_pixel.publish(ball_pixel)

            # Calculate ball position in 3D space
            ball_pos = self.camera.calculate_ball_from_bounding_boxes(boundingBoxes)

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = header.stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose = ball_pos.pose
            
            self.pub_ball.publish(pose_msg)
            
            self.get_logger().debug(
                f"Ball detected at: x={pose_msg.pose.position.x:.2f}, "
                f"y={pose_msg.pose.position.y:.2f}, "
                f"z={pose_msg.pose.position.z:.2f}",
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f"Error processing ball detection: {str(e)}")


def main(args=None):
    """Main entry point for the node"""
    rclpy.init(args=args)

    # Default model path
    model_path_default = os.path.join(
        expanduser("~"),
        "ros2_ws/src/soccerbot/soccer_perception/soccer_object_detection/models/half_5.pt"
    )

    # Parse command line arguments
    parser = ArgumentParser()
    parser.add_argument(
        "--model",
        dest="model_path",
        default=model_path_default,
        help="Path to YOLO model file (.pt)"
    )
    args, unknown = parser.parse_known_args()

    # Verify model exists
    if not os.path.exists(args.model_path):
        print(f"ERROR: Model file not found at: {args.model_path}")
        print("Please specify correct model path using --model argument")
        return

    # Create and spin node
    try:
        node = ObjectDetectionNodeRos(args.model_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down object detection node...")
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()