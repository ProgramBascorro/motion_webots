#!/usr/bin/env python3
"""
Fieldline Detector ROS2 Node for Humble
Detects field lines from camera images and publishes as point clouds
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Import custom modules
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_localization.detector_fieldline import DetectorFieldline

# Set default namespace if not exists
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"


class DetectorFieldlineRos(DetectorFieldline, Node):
    """
    ROS2 bridge for detecting fieldlines
    Processes camera images to detect field lines and publishes as point clouds
    """

    def __init__(self):
        # Initialize ROS2 Node first
        Node.__init__(self, "detector_fieldline")
        
        # Initialize DetectorFieldline
        DetectorFieldline.__init__(self)
        
        # Declare parameters
        self.declare_parameter("point_cloud_max_distance", 5.0)
        self.declare_parameter("point_cloud_spacing", 30)
        self.declare_parameter("ground_truth", False)
        self.declare_parameter("publish_point_cloud", True)
        
        # Get parameters
        self.point_cloud_max_distance = self.get_parameter("point_cloud_max_distance").value
        self.point_cloud_spacing = self.get_parameter("point_cloud_spacing").value
        self.ground_truth = self.get_parameter("ground_truth").value
        self.publish_point_cloud = self.get_parameter("publish_point_cloud").value
        
        # Get robot name from namespace
        self.robot_name = self.get_namespace().strip("/")
        if not self.robot_name:
            self.robot_name = "robot1"
            self.get_logger().warn(f"No namespace set, using default: {self.robot_name}")
        
        # Initialize camera calculations
        self.camera = CameraCalculationsRos(self, self.robot_name)
        self.camera.reset_position()
        
        # CV Bridge for image conversion
        self.br = CvBridge()
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS profile for sensor data (best effort)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            sensor_qos
        )
        
        # Publishers
        self.image_publisher = self.create_publisher(
            Image,
            "camera/line_image",
            10
        )
        
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            "field_point_cloud",
            10
        )
        
        self.get_logger().info(
            f"Fieldline detector initialized for robot: {self.robot_name}"
        )
        self.get_logger().info(
            f"Point cloud settings: max_distance={self.point_cloud_max_distance}m, "
            f"spacing={self.point_cloud_spacing}"
        )

    def image_callback(self, img: Image):
        """
        Main callback for processing incoming camera images
        
        Args:
            img: ROS2 Image message (rgb8 format)
        """
        try:
            t_start = time.time()
            
            # Reset camera position with current timestamp
            if self.ground_truth:
                if not self.publish_point_cloud:
                    self.camera.reset_position(timestamp=img.header.stamp)
                else:
                    # For ground truth, use world frame
                    self.camera.reset_position(
                        timestamp=img.header.stamp,
                        camera_frame="/camera_gt"
                    )
            else:
                self.camera.reset_position(timestamp=img.header.stamp)
            
            # Convert ROS image to OpenCV format
            image = self.br.imgmsg_to_cv2(img, desired_encoding="rgb8")
            
            # Process image to detect field lines
            lines_only = self.image_filter(image, debug=False)
            
            # Publish line detection image if there are subscribers
            if self.image_publisher.get_subscription_count() > 0:
                img_out = self.br.cv2_to_imgmsg(lines_only)
                img_out.header = img.header
                self.image_publisher.publish(img_out)
            
            # Publish point cloud
            self.pub_pointcloud(lines_only, img.header.stamp)
            
            # Log processing time
            t_end = time.time()
            self.get_logger().debug(
                f"Fieldline detection rate: {t_end - t_start:.4f}s",
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def pub_pointcloud(self, lines_only, stamp):
        """
        Publish detected field lines as a point cloud
        
        Args:
            lines_only: Binary image with detected lines
            stamp: ROS timestamp for the point cloud
        """
        try:
            # Convert line pixels to 3D points
            points3d = self.img_to_points(lines_only)
            
            if not self.publish_point_cloud:
                return
            
            if self.point_cloud_publisher.get_subscription_count() == 0:
                return
            
            # Publish straight base link transform if available
            if hasattr(self.camera, 'pose_base_link_straight'):
                try:
                    t = TransformStamped()
                    t.header.stamp = stamp
                    t.header.frame_id = f"{self.robot_name}/odom"
                    t.child_frame_id = f"{self.robot_name}/base_footprint_straight"
                    
                    # Set translation
                    pos = self.camera.pose_base_link_straight.position
                    t.transform.translation.x = float(pos[0])
                    t.transform.translation.y = float(pos[1])
                    t.transform.translation.z = float(pos[2])
                    
                    # Set rotation
                    quat = self.camera.pose_base_link_straight.quaternion
                    t.transform.rotation.x = float(quat[0])
                    t.transform.rotation.y = float(quat[1])
                    t.transform.rotation.z = float(quat[2])
                    t.transform.rotation.w = float(quat[3])
                    
                    self.tf_broadcaster.sendTransform(t)
                    
                except Exception as e:
                    self.get_logger().warn(
                        f"Failed to publish base_footprint_straight transform: {e}",
                        throttle_duration_sec=5.0
                    )
            
            # Create point cloud message
            header = Header()
            header.stamp = stamp
            
            # Determine frame_id based on ground truth mode
            if self.ground_truth:
                if not self.publish_point_cloud:
                    header.frame_id = f"{self.robot_name}/base_footprint_straight"
                else:
                    header.frame_id = "world"
            else:
                header.frame_id = f"{self.robot_name}/base_footprint_straight"
            
            # Create and publish point cloud
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            self.point_cloud_publisher.publish(point_cloud_msg)
            
            self.get_logger().debug(
                f"Published {len(points3d)} field line points",
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing point cloud: {str(e)}")


def main(args=None):
    """Main entry point for the node"""
    rclpy.init(args=args)
    
    try:
        node = DetectorFieldlineRos()
        
        node.get_logger().info("Fieldline detector node started")
        node.get_logger().info("Waiting for camera images...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down fieldline detector node...")
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()