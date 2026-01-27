#!/usr/bin/env python3
"""
Hybrid Field Line Detector
Supports both simulator (static) and real robot (dynamic) modes
"""

import time
import numpy as np
import cv2
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs_py.point_cloud2 as pcl2
from std_msgs.msg import Header

from .camera_pose_provider import CameraPoseProvider
from .static_camera_pose import StaticCameraPose
from .dynamic_camera_pose import DynamicCameraPose


class DetectorFieldlineHybrid(Node):
    """
    Hybrid field line detector supporting both modes:
    - Static mode: For simulator (fast, simple)
    - Dynamic mode: For real robot (accurate, robust)
    
    Mode selection via parameter: use_dynamic_tf
    """
    
    def __init__(self):
        super().__init__("detector_fieldline")
        
        # ===== PARAMETERS =====
        self.declare_parameters(
            namespace='',
            parameters=[
                # Mode selection
                ('use_dynamic_tf', False),
                
                # Camera parameters
                ('camera.focal_length', 900.0),
                ('camera.image_width', 1280),
                ('camera.image_height', 720),
                ('camera.height', 0.475),
                ('camera.tilt', -0.349),
                ('camera.offset_x', 0.08),
                ('camera.offset_y', 0.0),
                
                # Frame names
                ('frames.camera', 'cam_link'),
                ('frames.base', 'base_link'),
                ('frames.world', 'odom'),
                
                # Detection parameters
                ('detection.white_threshold', 200),
                ('detection.min_line_length', 20),
                ('detection.max_line_gap', 10),
                
                # Point cloud parameters
                ('point_cloud.max_distance', 5.0),
                ('point_cloud.spacing', 30),
                ('point_cloud.min_points', 5),
                
                # Publishing
                ('publish.debug_image', True),
                ('publish.point_cloud', True),
            ]
        )
        
        # Get parameters
        self.use_dynamic_tf = self.get_parameter('use_dynamic_tf').value
        
        # Camera parameters
        focal_length = self.get_parameter('camera.focal_length').value
        img_width = self.get_parameter('camera.image_width').value
        img_height = self.get_parameter('camera.image_height').value
        
        # Build camera matrix
        self.camera_matrix = np.array([
            [focal_length, 0, img_width / 2],
            [0, focal_length, img_height / 2],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Detection parameters
        self.white_threshold = self.get_parameter('detection.white_threshold').value
        self.min_line_length = self.get_parameter('detection.min_line_length').value
        self.max_line_gap = self.get_parameter('detection.max_line_gap').value
        
        # Point cloud parameters
        self.max_distance = self.get_parameter('point_cloud.max_distance').value
        self.spacing = self.get_parameter('point_cloud.spacing').value
        self.min_points = self.get_parameter('point_cloud.min_points').value
        
        # Publishing flags
        self.publish_debug = self.get_parameter('publish.debug_image').value
        self.publish_pc = self.get_parameter('publish.point_cloud').value
        
        # ===== CAMERA POSE PROVIDER =====
        self.camera_pose: CameraPoseProvider
        
        if self.use_dynamic_tf:
            self.get_logger().info("🤖 DYNAMIC MODE: Real robot with TF tracking")
            self.camera_pose = DynamicCameraPose(
                node=self,
                camera_matrix=self.camera_matrix,
                base_frame=self.get_parameter('frames.base').value,
                camera_frame=self.get_parameter('frames.camera').value,
                world_frame=self.get_parameter('frames.world').value
            )
        else:
            self.get_logger().info("🎮 STATIC MODE: Simulator with fixed camera")
            self.camera_pose = StaticCameraPose(
                camera_matrix=self.camera_matrix,
                camera_height=self.get_parameter('camera.height').value,
                camera_tilt=self.get_parameter('camera.tilt').value,
                camera_offset_x=self.get_parameter('camera.offset_x').value,
                camera_offset_y=self.get_parameter('camera.offset_y').value,
                frame_id=self.get_parameter('frames.camera').value
            )
        
        # Log camera pose provider info
        info = self.camera_pose.get_info()
        self.get_logger().info(f"Camera pose provider: {info}")
        
        # ===== ROS INTERFACES =====
        self.br = CvBridge()
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            sensor_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            sensor_qos
        )
        
        # Publishers
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                'camera/line_image',
                10
            )
        
        if self.publish_pc:
            self.point_cloud_pub = self.create_publisher(
                PointCloud2,
                'field_point_cloud',
                10
            )
        
        # Statistics
        self.frame_count = 0
        self.processing_times = []
        
        # Create timer for statistics logging
        self.create_timer(10.0, self.log_statistics)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Hybrid Field Line Detector initialized!")
        self.get_logger().info(f"Mode: {'DYNAMIC (Real Robot)' if self.use_dynamic_tf else 'STATIC (Simulator)'}")
        self.get_logger().info(f"Frame ID: {self.camera_pose.get_frame_id()}")
        self.get_logger().info(f"Parameters: threshold={self.white_threshold}, "
                              f"max_dist={self.max_distance}m, spacing={self.spacing}px")
        self.get_logger().info("Waiting for camera images...")
        self.get_logger().info("=" * 60)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera info (optional, for future calibration)"""
        pass
    
    def image_callback(self, img: Image):
        """Main image processing callback"""
        try:
            t_start = time.time()
            
            # Check if camera pose provider is ready
            if not self.camera_pose.is_ready():
                self.get_logger().warn(
                    "Camera pose provider not ready, skipping frame",
                    throttle_duration_sec=5.0
                )
                return
            
            # Convert ROS image to OpenCV
            cv_image = self.br.imgmsg_to_cv2(img, desired_encoding="bgr8")
            
            # Detect white lines
            line_mask = self.detect_white_lines(cv_image)
            
            # Publish debug image
            if self.publish_debug and self.debug_image_pub.get_subscription_count() > 0:
                debug_img = self.br.cv2_to_imgmsg(line_mask, encoding="mono8")
                debug_img.header = img.header
                self.debug_image_pub.publish(debug_img)
            
            # Convert to 3D points
            points_3d = self.project_to_3d(line_mask, img.header.stamp)
            
            # Publish point cloud
            if self.publish_pc and len(points_3d) >= self.min_points:
                self.publish_point_cloud(points_3d, img.header.stamp)
            
            # Update statistics
            t_end = time.time()
            processing_time = (t_end - t_start) * 1000  # ms
            self.processing_times.append(processing_time)
            self.frame_count += 1
            
            # Log occasionally
            if self.frame_count % 100 == 0:
                avg_time = np.mean(self.processing_times[-100:])
                self.get_logger().info(
                    f"Processed {self.frame_count} frames, "
                    f"avg time: {avg_time:.1f}ms, "
                    f"points: {len(points_3d)}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def detect_white_lines(self, image: np.ndarray) -> np.ndarray:
        """
        Detect white field lines in image
        
        Args:
            image: BGR image
            
        Returns:
            Binary mask of detected lines
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Threshold for white pixels
        _, white_mask = cv2.threshold(
            gray,
            self.white_threshold,
            255,
            cv2.THRESH_BINARY
        )
        
        # Morphological operations to clean up noise
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        return white_mask
    
    def project_to_3d(self, line_mask: np.ndarray, 
                     timestamp) -> List[Tuple[float, float, float]]:
        """
        Project detected line pixels to 3D points
        
        Args:
            line_mask: Binary mask of line pixels
            timestamp: Image timestamp for synchronized TF lookup
            
        Returns:
            List of 3D points [(x, y, z), ...]
        """
        points_3d = []
        
        # Get white pixel coordinates
        white_pixels = np.where(line_mask > 0)
        num_pixels = len(white_pixels[0])
        
        if num_pixels == 0:
            return points_3d
        
        # Sample pixels according to spacing
        for i in range(0, num_pixels, self.spacing):
            v = white_pixels[0][i]  # row (y in image)
            u = white_pixels[1][i]  # col (x in image)
            
            # Project to 3D
            point_3d = self.camera_pose.project_pixel_to_ground(u, v, timestamp)
            
            if point_3d is not None:
                # Check distance
                distance = np.sqrt(point_3d[0]**2 + point_3d[1]**2)
                if distance <= self.max_distance:
                    points_3d.append(point_3d)
        
        return points_3d
    
    def publish_point_cloud(self, points_3d: List[Tuple[float, float, float]],
                           stamp):
        """
        Publish point cloud
        
        Args:
            points_3d: List of 3D points
            stamp: ROS timestamp
        """
        try:
            if self.point_cloud_pub.get_subscription_count() == 0:
                return
            
            # Create header
            header = Header()
            header.stamp = stamp
            header.frame_id = self.camera_pose.get_frame_id()
            
            # Create point cloud
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points_3d)
            self.point_cloud_pub.publish(point_cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing point cloud: {e}")
    
    def log_statistics(self):
        """Log performance statistics"""
        if self.frame_count == 0:
            return
        
        avg_time = np.mean(self.processing_times)
        max_time = np.max(self.processing_times)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Statistics after {self.frame_count} frames:")
        self.get_logger().info(f"  Avg processing time: {avg_time:.1f}ms")
        self.get_logger().info(f"  Max processing time: {max_time:.1f}ms")
        self.get_logger().info(f"  Avg FPS: {1000/avg_time:.1f}")
        
        # Provider-specific stats
        info = self.camera_pose.get_info()
        if info['type'] == 'dynamic':
            self.get_logger().info(f"  TF success rate: {info['tf_success_rate']}")
        
        self.get_logger().info("=" * 60)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = DetectorFieldlineHybrid()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()