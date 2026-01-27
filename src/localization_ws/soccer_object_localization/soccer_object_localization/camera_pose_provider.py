#!/usr/bin/env python3
"""
Camera Pose Provider - Abstract Base Class
Defines interface for camera pose providers
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional, List
import numpy as np


class CameraPose:
    """
    Data class for camera pose
    """
    def __init__(self, 
                 position: np.ndarray, 
                 orientation: np.ndarray,
                 timestamp: Optional[float] = None):
        """
        Args:
            position: [x, y, z] in meters
            orientation: [roll, pitch, yaw] in radians OR quaternion [x, y, z, w]
            timestamp: ROS timestamp (optional)
        """
        self.position = np.array(position, dtype=np.float64)
        
        if len(orientation) == 3:
            # Euler angles
            self.roll = orientation[0]
            self.pitch = orientation[1]
            self.yaw = orientation[2]
            self.quaternion = self._euler_to_quaternion(orientation)
        elif len(orientation) == 4:
            # Quaternion
            self.quaternion = np.array(orientation, dtype=np.float64)
            euler = self._quaternion_to_euler(orientation)
            self.roll = euler[0]
            self.pitch = euler[1]
            self.yaw = euler[2]
        else:
            raise ValueError("Orientation must be 3 (euler) or 4 (quaternion) elements")
        
        self.timestamp = timestamp
    
    @staticmethod
    def _euler_to_quaternion(euler):
        """Convert euler angles (roll, pitch, yaw) to quaternion"""
        roll, pitch, yaw = euler
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return np.array([qx, qy, qz, qw])
    
    @staticmethod
    def _quaternion_to_euler(quat):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        qx, qy, qz, qw = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def get_rotation_matrix(self):
        """Get 3x3 rotation matrix from quaternion"""
        qx, qy, qz, qw = self.quaternion
        
        R = np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        return R
    
    def __str__(self):
        return (f"CameraPose(pos=[{self.position[0]:.3f}, {self.position[1]:.3f}, "
                f"{self.position[2]:.3f}], euler=[{self.roll:.3f}, {self.pitch:.3f}, "
                f"{self.yaw:.3f}])")


class CameraPoseProvider(ABC):
    """
    Abstract base class for camera pose providers
    
    Implementations:
    - StaticCameraPose: For simulator (fixed camera mount)
    - DynamicCameraPose: For real robot (TF tree tracking)
    """
    
    def __init__(self, camera_matrix: np.ndarray):
        """
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
        """
        self.camera_matrix = camera_matrix
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
    
    @abstractmethod
    def get_camera_pose(self, timestamp=None) -> Optional[CameraPose]:
        """
        Get current camera pose
        
        Args:
            timestamp: ROS timestamp (for dynamic providers)
            
        Returns:
            CameraPose object or None if unavailable
        """
        pass
    
    @abstractmethod
    def project_pixel_to_ground(self, u: int, v: int, 
                                timestamp=None) -> Optional[Tuple[float, float, float]]:
        """
        Project pixel (u, v) to 3D point on ground plane
        
        Args:
            u: pixel x coordinate
            v: pixel y coordinate
            timestamp: ROS timestamp (for dynamic providers)
            
        Returns:
            (x, y, z) in camera/base frame, or None if projection fails
        """
        pass
    
    def project_pixels_to_ground(self, pixels: List[Tuple[int, int]], 
                                 timestamp=None) -> List[Tuple[float, float, float]]:
        """
        Project multiple pixels to ground plane
        
        Args:
            pixels: List of (u, v) pixel coordinates
            timestamp: ROS timestamp
            
        Returns:
            List of (x, y, z) 3D points
        """
        points_3d = []
        for u, v in pixels:
            point = self.project_pixel_to_ground(u, v, timestamp)
            if point is not None:
                points_3d.append(point)
        return points_3d
    
    @abstractmethod
    def is_ready(self) -> bool:
        """
        Check if provider is ready to provide poses
        
        Returns:
            True if ready, False otherwise
        """
        pass
    
    @abstractmethod
    def get_frame_id(self) -> str:
        """
        Get the frame ID for published point clouds
        
        Returns:
            Frame ID string (e.g., 'cam_link', 'base_link')
        """
        pass