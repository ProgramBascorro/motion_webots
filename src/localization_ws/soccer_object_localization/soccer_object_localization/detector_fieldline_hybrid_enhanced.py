#!/usr/bin/env python3
"""
FIXED Enhanced Hybrid Field Line Detector
TUNED for soccer field - removes goal nets, horizon, noise
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
    FIXED Enhanced hybrid field line detector
    - Better ROI masking (removes goal nets!)
    - Tuned Hough parameters (less false positives)
    - Grass color removal (HSV-based)
    """
    
    def __init__(self):
        super().__init__("detector_fieldline")
        
        # ===== PARAMETERS =====
        self.declare_parameters(
            namespace='',
            parameters=[
                # Mode selection
                ('use_dynamic_tf', False),
                ('use_enhanced_detection', True),
                
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
                
                # Detection parameters - Basic
                ('detection.white_threshold', 180),     # Higher - more strict!
                ('detection.min_line_length', 30),      # Longer lines only
                ('detection.max_line_gap', 15),         # Smaller gaps
                
                # Detection parameters - Enhanced (TUNED!)
                ('detection.canny_low', 80),            # Higher - less noise
                ('detection.canny_high', 200),          # Higher - less noise
                ('detection.hough_threshold', 80),      # Higher - more strict
                ('detection.hough_rho', 1),
                ('detection.hough_theta', 0.017453),
                ('detection.line_thickness', 2),        # Thinner
                
                # ROI parameters (CRITICAL!)
                ('detection.use_roi', True),
                ('detection.roi_top_cut', 0.45),        # Cut top 45% (goal net!)
                ('detection.roi_bottom_cut', 0.10),     # Cut bottom 10% (noise)
                
                # Grass removal (NEW!)
                ('detection.remove_grass', True),
                ('detection.grass_h_low', 35),          # Green hue range
                ('detection.grass_h_high', 85),
                ('detection.grass_s_low', 40),          # Saturation range
                
                # Point cloud parameters
                ('point_cloud.max_distance', 5.0),
                ('point_cloud.spacing', 15),            # Medium density
                ('point_cloud.min_points', 5),
                
                # Publishing
                ('publish.debug_image', True),
                ('publish.point_cloud', True),
            ]
        )
        
        # Get parameters
        self.use_dynamic_tf = self.get_parameter('use_dynamic_tf').value
        self.use_enhanced = self.get_parameter('use_enhanced_detection').value
        
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
        
        # Detection parameters - Basic
        self.white_threshold = self.get_parameter('detection.white_threshold').value
        self.min_line_length = self.get_parameter('detection.min_line_length').value
        self.max_line_gap = self.get_parameter('detection.max_line_gap').value
        
        # Detection parameters - Enhanced
        self.canny_low = self.get_parameter('detection.canny_low').value
        self.canny_high = self.get_parameter('detection.canny_high').value
        self.hough_threshold = self.get_parameter('detection.hough_threshold').value
        self.hough_rho = self.get_parameter('detection.hough_rho').value
        self.hough_theta = self.get_parameter('detection.hough_theta').value
        self.line_thickness = self.get_parameter('detection.line_thickness').value
        
        # ROI parameters
        self.use_roi = self.get_parameter('detection.use_roi').value
        self.roi_top_cut = self.get_parameter('detection.roi_top_cut').value
        self.roi_bottom_cut = self.get_parameter('detection.roi_bottom_cut').value
        
        # Grass removal parameters
        self.remove_grass = self.get_parameter('detection.remove_grass').value
        self.grass_h_low = self.get_parameter('detection.grass_h_low').value
        self.grass_h_high = self.get_parameter('detection.grass_h_high').value
        self.grass_s_low = self.get_parameter('detection.grass_s_low').value
        
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
        self.hough_line_counts = []
        
        # Create timer for statistics logging
        self.create_timer(10.0, self.log_statistics)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔥 FIXED Enhanced Field Line Detector!")
        self.get_logger().info(f"Mode: {'DYNAMIC' if self.use_dynamic_tf else 'STATIC'}")
        self.get_logger().info(f"Detection: {'ENHANCED (Tuned)' if self.use_enhanced else 'SIMPLE'}")
        self.get_logger().info(f"ROI: Top cut {self.roi_top_cut*100:.0f}%, Bottom cut {self.roi_bottom_cut*100:.0f}%")
        self.get_logger().info(f"Grass removal: {self.remove_grass}")
        self.get_logger().info("=" * 60)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera info"""
        pass
    
    def image_callback(self, img: Image):
        """Main image processing callback"""
        try:
            t_start = time.time()
            
            if not self.camera_pose.is_ready():
                self.get_logger().warn(
                    "Camera pose provider not ready",
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
            processing_time = (t_end - t_start) * 1000
            self.processing_times.append(processing_time)
            self.frame_count += 1
            
            if self.frame_count % 100 == 0:
                avg_time = np.mean(self.processing_times[-100:])
                self.get_logger().info(
                    f"Frames: {self.frame_count}, "
                    f"time: {avg_time:.1f}ms, "
                    f"points: {len(points_3d)}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def detect_white_lines(self, image: np.ndarray) -> np.ndarray:
        """Detect white field lines"""
        if self.use_enhanced:
            return self.detect_white_lines_enhanced(image)
        else:
            return self.detect_white_lines_simple(image)
    
    def detect_white_lines_simple(self, image: np.ndarray) -> np.ndarray:
        """Simple thresholding (original)"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, white_mask = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        return white_mask
    
    def detect_white_lines_enhanced(self, image: np.ndarray) -> np.ndarray:
        """
        FIXED Enhanced detection with proper ROI and grass removal
        
        Key fixes:
        1. Remove grass FIRST (HSV color filtering)
        2. Apply strict ROI (remove goal nets and horizon)
        3. Use Canny + Hough on cleaned image
        4. Stricter thresholds (less false positives)
        """
        h, w = image.shape[:2]
        
        # Step 1: Remove grass using HSV color space
        if self.remove_grass:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Grass mask (green hue, high saturation)
            grass_mask = cv2.inRange(
                hsv,
                (self.grass_h_low, self.grass_s_low, 0),
                (self.grass_h_high, 255, 255)
            )
            
            # Invert: keep non-grass
            non_grass_mask = cv2.bitwise_not(grass_mask)
            
            # Apply mask to image
            image_filtered = cv2.bitwise_and(image, image, mask=non_grass_mask)
        else:
            image_filtered = image.copy()
        
        # Step 2: Apply ROI BEFORE processing
        roi_mask = self.create_roi_mask(h, w)
        image_filtered = cv2.bitwise_and(image_filtered, image_filtered, mask=roi_mask)
        
        # Step 3: Convert to grayscale
        gray = cv2.cvtColor(image_filtered, cv2.COLOR_BGR2GRAY)
        
        # Step 4: Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Step 5: Simple threshold (stricter!)
        _, simple_thresh = cv2.threshold(
            blurred,
            self.white_threshold,
            255,
            cv2.THRESH_BINARY
        )
        
        # Step 6: Canny edge detection
        edges = cv2.Canny(
            blurred,
            self.canny_low,
            self.canny_high,
            apertureSize=3
        )
        
        # Step 7: Hough line transform (stricter parameters!)
        lines = cv2.HoughLinesP(
            edges,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        # Step 8: Draw detected lines
        line_mask = np.zeros_like(gray)
        line_count = 0
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Filter: Only nearly horizontal/vertical lines (field lines!)
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                
                # Line angle check: horizontal or vertical within 30 degrees
                if dx > 0:
                    angle = np.arctan(dy / dx) * 180 / np.pi
                    if angle < 30 or angle > 60:  # Keep horizontal-ish or vertical-ish
                        cv2.line(line_mask, (x1, y1), (x2, y2), 255, self.line_thickness)
                        line_count += 1
                else:
                    cv2.line(line_mask, (x1, y1), (x2, y2), 255, self.line_thickness)
                    line_count += 1
        
        self.hough_line_counts.append(line_count)
        
        # Step 9: Combine Hough lines with simple threshold
        combined = cv2.bitwise_or(line_mask, simple_thresh)
        
        # Step 10: Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=1)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return combined
    
    def create_roi_mask(self, h: int, w: int) -> np.ndarray:
        """
        Create ROI mask to focus on field region
        
        Removes:
        - Top portion (goal nets, horizon, sky)
        - Bottom portion (robot body, noise)
        - Side portions (optional)
        """
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        
        # Calculate ROI boundaries
        top_cut = int(h * self.roi_top_cut)
        bottom_cut = int(h * (1.0 - self.roi_bottom_cut))
        
        # Keep only middle vertical band
        roi_mask[top_cut:bottom_cut, :] = 255
        
        return roi_mask
    
    def project_to_3d(self, line_mask: np.ndarray, timestamp) -> List[Tuple[float, float, float]]:
        """Project detected pixels to 3D points"""
        points_3d = []
        
        white_pixels = np.where(line_mask > 0)
        num_pixels = len(white_pixels[0])
        
        if num_pixels == 0:
            return points_3d
        
        for i in range(0, num_pixels, self.spacing):
            v = white_pixels[0][i]
            u = white_pixels[1][i]
            
            point_3d = self.camera_pose.project_pixel_to_ground(u, v, timestamp)
            
            if point_3d is not None:
                distance = np.sqrt(point_3d[0]**2 + point_3d[1]**2)
                if distance <= self.max_distance:
                    points_3d.append(point_3d)
        
        return points_3d
    
    def publish_point_cloud(self, points_3d: List[Tuple[float, float, float]], stamp):
        """Publish point cloud"""
        try:
            if self.point_cloud_pub.get_subscription_count() == 0:
                return
            
            header = Header()
            header.stamp = stamp
            header.frame_id = self.camera_pose.get_frame_id()
            
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points_3d)
            self.point_cloud_pub.publish(point_cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing point cloud: {e}")
    
    def log_statistics(self):
        """Log statistics"""
        if self.frame_count == 0:
            return
        
        avg_time = np.mean(self.processing_times)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📊 Stats: {self.frame_count} frames")
        self.get_logger().info(f"  Avg time: {avg_time:.1f}ms, FPS: {1000/avg_time:.1f}")
        
        if self.use_enhanced and len(self.hough_line_counts) > 0:
            avg_lines = np.mean(self.hough_line_counts[-100:])
            self.get_logger().info(f"  Avg Hough lines: {avg_lines:.1f}")
        
        self.get_logger().info("=" * 60)


def main(args=None):
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