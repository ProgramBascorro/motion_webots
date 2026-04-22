#!/usr/bin/env python3
"""
Filtered Dynamic Camera Pose Provider
Smooths camera pose from TF to reduce jitter during robot movement
"""

import numpy as np
from typing import Optional, Tuple
from rclpy.duration import Duration

from .dynamic_camera_pose import DynamicCameraPose


class FilteredDynamicCameraPose(DynamicCameraPose):
    """
    Dynamic camera pose with exponential moving average filtering
    
    Applies smoothing to:
    - Position (low-pass filter)
    - Orientation (SLERP quaternion interpolation)
    
    This reduces jitter from rapid head movements while maintaining
    accuracy for slower robot motion.
    """
    
    def __init__(self, node, camera_matrix, base_frame, camera_frame, world_frame,
                 position_alpha=0.8, rotation_alpha=0.9):
        """
        Initialize filtered dynamic camera pose
        
        Args:
            node: ROS2 node
            camera_matrix: 3x3 camera intrinsic matrix
            base_frame: Base link frame name
            camera_frame: Camera frame name
            world_frame: World/odom frame name
            position_alpha: Smoothing factor for position (0-1, higher = more smoothing)
                           0.0 = no smoothing (raw TF)
                           1.0 = maximum smoothing (very slow response)
                           0.7 = recommended (balances stability and responsiveness)
            rotation_alpha: Smoothing factor for rotation (0-1, higher = more smoothing)
                           0.8 = recommended (rotations need more smoothing)
        """
        super().__init__(node, camera_matrix, base_frame, camera_frame, world_frame)
        
        # Smoothing parameters
        self.position_alpha = position_alpha
        self.rotation_alpha = rotation_alpha
        
        # Previous state for filtering
        self.prev_position = None
        self.prev_quaternion = None
        
        # Statistics
        self.filter_initialized = False
        
        node.get_logger().info(
            f"Filtered dynamic camera pose initialized: "
            f"pos_alpha={position_alpha}, rot_alpha={rotation_alpha}"
        )
    
    def _get_transform_filtered(self, timestamp):
        """
        Get transform from TF with filtering
        
        Returns:
            (translation, quaternion) tuple or None if lookup fails
        """
        try:
            # CRITICAL FIX: Use time=0 for latest available transform
            # This avoids "frame does not exist" errors at startup
            from rclpy.time import Time
            
            # Get raw transform from TF (use latest available)
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time(),  # Use Time() instead of timestamp for "latest"
                timeout=Duration(seconds=0.05)  # Shorter timeout
            )
            
            # Extract raw position and quaternion
            raw_position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            raw_quaternion = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            # Apply filtering
            if self.prev_position is None or not self.filter_initialized:
                # First frame: initialize filter with current values
                filtered_position = raw_position
                filtered_quaternion = raw_quaternion
                self.filter_initialized = True
            else:
                # Apply exponential moving average to position
                filtered_position = (
                    self.position_alpha * self.prev_position +
                    (1.0 - self.position_alpha) * raw_position
                )
                
                # Apply SLERP to quaternion (spherical interpolation)
                filtered_quaternion = self._slerp_quaternion(
                    self.prev_quaternion,
                    raw_quaternion,
                    1.0 - self.rotation_alpha  # t parameter (0 = prev, 1 = current)
                )
            
            # Store for next iteration
            self.prev_position = filtered_position
            self.prev_quaternion = filtered_quaternion
            
            return filtered_position, filtered_quaternion
            
        except Exception as e:
            # Better error message
            if "does not exist" in str(e):
                # Frame not in TF tree yet (common at startup)
                # This is normal, just wait
                pass
            else:
                # Other TF error
                self.node.get_logger().debug(
                    f"TF lookup failed: {e}",
                    throttle_duration_sec=5.0
                )
            return None
    
    def project_pixel_to_ground(self, u: int, v: int, timestamp) -> Optional[Tuple[float, float, float]]:
        """
        Project pixel to ground plane using filtered camera pose
        
        Args:
            u: Pixel x coordinate
            v: Pixel y coordinate
            timestamp: ROS timestamp for TF lookup
            
        Returns:
            (x, y, z) point in world frame, or None if projection fails
        """
        # Get filtered transform
        transform_result = self._get_transform_filtered(timestamp)
        if transform_result is None:
            return None
        
        filtered_position, filtered_quaternion = transform_result
        
        # Convert quaternion to rotation matrix
        rotation_matrix = self._quaternion_to_rotation_matrix(filtered_quaternion)
        
        # Pixel to camera coordinates (using intrinsic matrix)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Ray in camera frame (normalized)
        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy
        z_cam = 1.0
        
        ray_camera = np.array([x_cam, y_cam, z_cam])
        ray_camera = ray_camera / np.linalg.norm(ray_camera)
        
        # Transform ray to world frame
        ray_world = rotation_matrix @ ray_camera
        
        # Ground plane intersection (z = 0)
        camera_pos = filtered_position
        
        # Ray equation: P = camera_pos + t * ray_world
        # Solve for t where z = 0
        if abs(ray_world[2]) < 1e-6:
            return None  # Ray parallel to ground
        
        t = -camera_pos[2] / ray_world[2]
        
        if t < 0:
            return None  # Intersection behind camera
        
        # Compute intersection point
        point_world = camera_pos + t * ray_world
        
        return (float(point_world[0]), float(point_world[1]), 0.0)
    
    def _slerp_quaternion(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        Spherical linear interpolation between two quaternions
        
        Args:
            q1: First quaternion [x, y, z, w]
            q2: Second quaternion [x, y, z, w]
            t: Interpolation parameter (0 = q1, 1 = q2)
            
        Returns:
            Interpolated quaternion
        """
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Compute dot product
        dot = np.dot(q1, q2)
        
        # If quaternions are close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # If dot product negative, negate q2 (shorter path)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # Clamp dot product
        dot = np.clip(dot, -1.0, 1.0)
        
        # Compute angle
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        
        # Compute SLERP coefficients
        if abs(sin_theta) < 1e-6:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        w1 = np.sin((1.0 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        # Interpolate
        result = w1 * q1 + w2 * q2
        
        return result / np.linalg.norm(result)
    
    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to 3x3 rotation matrix"""
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        
        return R
    
    def is_ready(self) -> bool:
        """
        Check if camera pose provider is ready
        
        Returns:
            True if TF chain is available, False otherwise
        """
        try:
            from rclpy.time import Time
            # Try to get latest transform (no timeout)
            self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.01)  # Very short timeout
            )
            return True
        except Exception:
            # TF not available yet
            return False
    
    def reset_filter(self):
        """Reset filter state"""
        self.prev_position = None
        self.prev_quaternion = None
        self.filter_initialized = False
        self.node.get_logger().info("Filtered camera pose: filter reset")
    
    def get_info(self) -> dict:
        """Get information about camera pose provider"""
        base_info = super().get_info()
        base_info.update({
            'type': 'filtered_dynamic',
            'position_alpha': self.position_alpha,
            'rotation_alpha': self.rotation_alpha,
            'filter_initialized': self.filter_initialized
        })
        return base_info