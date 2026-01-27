#!/usr/bin/env python3
"""
Dynamic Camera Pose Provider
For real robot - tracks actual camera pose via TF tree
Handles head movement, body tilt, walking dynamics
"""

import numpy as np
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

from .camera_pose_provider import CameraPose, CameraPoseProvider


class DynamicCameraPose(CameraPoseProvider):
    """
    Dynamic camera pose provider for real robot
    
    Features:
    - Tracks actual camera pose via TF tree
    - Handles head pan/tilt movements
    - Accounts for body tilt during walking
    - Synchronized timestamps for accurate projection
    
    Advantages:
    - Accurate for real robot
    - Handles all dynamic movements
    - Robust to mechanical variations
    
    Disadvantages:
    - Slower (TF lookups required)
    - Depends on TF tree being published
    - More complex debugging
    """
    
    def __init__(self,
                 node: Node,
                 camera_matrix: np.ndarray,
                 base_frame: str = 'base_link',
                 camera_frame: str = 'camera_link',
                 world_frame: str = 'odom',
                 tf_timeout: float = 0.1,
                 use_latest_tf: bool = False):
        """
        Args:
            node: ROS2 node for TF listener
            camera_matrix: 3x3 intrinsic matrix
            base_frame: Robot base frame
            camera_frame: Camera frame
            world_frame: World/odom frame
            tf_timeout: Timeout for TF lookups (seconds)
            use_latest_tf: Use latest available TF (for real-time)
        """
        super().__init__(camera_matrix)
        
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tf_timeout = tf_timeout
        self.use_latest_tf = use_latest_tf
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # Cache for last known pose
        self.last_pose: Optional[CameraPose] = None
        self.last_transform_time = None
        
        # Statistics
        self.tf_lookup_success = 0
        self.tf_lookup_failure = 0
        
        # Wait for TF tree to be available
        self._wait_for_tf()
    
    def _wait_for_tf(self, timeout: float = 5.0):
        """Wait for TF tree to become available"""
        self.node.get_logger().info(
            f"Waiting for TF: {self.base_frame} → {self.camera_frame}..."
        )
        
        start_time = self.node.get_clock().now()
        
        while rclpy.ok():
            try:
                # Try to lookup transform
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.node.get_logger().info("TF tree available!")
                return True
            except (LookupException, ExtrapolationException):
                # Check timeout
                elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    self.node.get_logger().warn(
                        f"TF tree not available after {timeout}s, continuing anyway..."
                    )
                    return False
                
                # Wait a bit
                rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return False
    
    def get_camera_pose(self, timestamp=None) -> Optional[CameraPose]:
        """
        Get current camera pose from TF tree
        
        Args:
            timestamp: ROS timestamp to lookup (None = latest)
            
        Returns:
            CameraPose object or None if TF lookup fails
        """
        try:
            # Determine which timestamp to use
            if timestamp is None or self.use_latest_tf:
                tf_time = Time()  # Latest available
            else:
                tf_time = timestamp
            
            # Lookup transform
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                tf_time,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            
            # Extract pose
            pose = self._transform_to_pose(transform)
            
            # Update cache
            self.last_pose = pose
            self.last_transform_time = transform.header.stamp
            self.tf_lookup_success += 1
            
            return pose
            
        except (LookupException, ExtrapolationException) as e:
            self.tf_lookup_failure += 1
            
            # Log warning (throttled)
            if self.tf_lookup_failure % 100 == 1:
                self.node.get_logger().warn(
                    f"TF lookup failed ({self.tf_lookup_failure} times): {e}",
                    throttle_duration_sec=5.0
                )
            
            # Return last known pose if available
            return self.last_pose
    
    def _transform_to_pose(self, transform: TransformStamped) -> CameraPose:
        """Convert TF transform to CameraPose"""
        # Extract position
        position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        # Extract quaternion
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        
        # Create pose
        pose = CameraPose(
            position=position,
            orientation=quaternion,
            timestamp=transform.header.stamp
        )
        
        return pose
    
    def project_pixel_to_ground(self, u: int, v: int,
                                timestamp=None) -> Optional[Tuple[float, float, float]]:
        """
        Project pixel to ground plane using current camera pose
        
        Args:
            u: pixel x coordinate
            v: pixel y coordinate  
            timestamp: ROS timestamp for synchronized TF lookup
            
        Returns:
            (x, y, z) in base frame, or None if projection fails
        """
        # Get current camera pose
        pose = self.get_camera_pose(timestamp)
        if pose is None:
            return None
        
        # Normalize pixel coordinates
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        
        # Ray direction in camera frame
        ray_camera = np.array([x_norm, y_norm, 1.0])
        
        # Transform ray to base frame using rotation matrix
        R = pose.get_rotation_matrix()
        ray_base = R @ ray_camera
        
        # Camera position in base frame
        camera_pos = pose.position
        
        # Intersect ray with ground plane (z = 0 in base frame)
        # Parametric ray: P = camera_pos + t * ray_base
        # Ground plane: z = 0
        # Solve: camera_pos[2] + t * ray_base[2] = 0
        
        if abs(ray_base[2]) < 1e-6:
            return None  # Ray parallel to ground
        
        t = -camera_pos[2] / ray_base[2]
        
        if t <= 0:
            return None  # Intersection behind camera
        
        # Calculate 3D point
        point_3d = camera_pos + t * ray_base
        
        return tuple(point_3d)
    
    def is_ready(self) -> bool:
        """Check if TF tree is available"""
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
            return True
        except:
            return False
    
    def get_frame_id(self) -> str:
        """Return base frame for point cloud"""
        return self.base_frame
    
    def get_info(self) -> dict:
        """Get provider configuration and statistics"""
        success_rate = 0
        if self.tf_lookup_success + self.tf_lookup_failure > 0:
            success_rate = 100 * self.tf_lookup_success / (
                self.tf_lookup_success + self.tf_lookup_failure
            )
        
        return {
            'type': 'dynamic',
            'base_frame': self.base_frame,
            'camera_frame': self.camera_frame,
            'world_frame': self.world_frame,
            'tf_success_rate': f"{success_rate:.1f}%",
            'last_pose': str(self.last_pose) if self.last_pose else 'None'
        }