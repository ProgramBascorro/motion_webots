#!/usr/bin/env python3
"""
Static Camera Pose Provider
For simulator - assumes fixed camera mount
Fast and simple, no TF lookups needed
"""

import numpy as np
from typing import Optional, Tuple
from .camera_pose_provider import CameraPose, CameraPoseProvider


class StaticCameraPose(CameraPoseProvider):
    """
    Static camera pose provider for simulator
    
    Assumptions:
    - Camera is rigidly mounted on robot
    - No head movement
    - No body tilt during walking
    - Perfect static transform
    
    Advantages:
    - Very fast (no TF lookups)
    - Simple to debug
    - Perfect for simulator
    
    Disadvantages:
    - Not accurate for real robot
    - Cannot handle dynamic movements
    """
    
    def __init__(self, 
                 camera_matrix: np.ndarray,
                 camera_height: float = 0.475,
                 camera_tilt: float = -0.349,
                 camera_offset_x: float = 0.08,
                 camera_offset_y: float = 0.0,
                 frame_id: str = 'cam_link'):
        """
        Args:
            camera_matrix: 3x3 intrinsic matrix
            camera_height: Height above ground (meters)
            camera_tilt: Tilt angle (radians, negative = looking down)
            camera_offset_x: Forward offset from base (meters)
            camera_offset_y: Side offset from base (meters)
            frame_id: Frame ID for point cloud
        """
        super().__init__(camera_matrix)
        
        self.camera_height = camera_height
        self.camera_tilt = camera_tilt
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self._frame_id = frame_id
        
        # Create static pose
        self.static_pose = CameraPose(
            position=[camera_offset_x, camera_offset_y, camera_height],
            orientation=[0.0, camera_tilt, 0.0]  # roll, pitch, yaw
        )
        
        # Precompute rotation matrices for efficiency
        self._rotation_matrix = self.static_pose.get_rotation_matrix()
        
        # Precompute trig values
        self._cos_tilt = np.cos(camera_tilt)
        self._sin_tilt = np.sin(camera_tilt)
    
    def get_camera_pose(self, timestamp=None) -> Optional[CameraPose]:
        """
        Get camera pose (always returns same static pose)
        
        Args:
            timestamp: Ignored for static provider
            
        Returns:
            Static CameraPose object
        """
        return self.static_pose
    
    def project_pixel_to_ground(self, u: int, v: int, 
                                timestamp=None) -> Optional[Tuple[float, float, float]]:
        """
        Project pixel to ground plane using static geometry
        
        Coordinate systems:
        - Image: u (col, left→right), v (row, top→bottom), origin top-left
        - Camera (before tilt): X right, Y down, Z forward
        - Camera (after tilt): rotated around X axis by camera_tilt (negative = looking down)
        - World/ROS: X forward, Y left, Z up
        - Ground plane: Z = 0
        
        Args:
            u: pixel x coordinate (column)
            v: pixel y coordinate (row)
            timestamp: Ignored for static provider
            
        Returns:
            (x, y, z) in world frame, or None if projection fails
        """
        # 1. Normalize pixel coordinates to [-1, 1] range
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        
        # 2. Create ray in camera frame (before tilt)
        # In standard camera convention: X right, Y down, Z forward
        # Normalized image plane is at Z = 1
        ray_cam = np.array([x_norm, y_norm, 1.0])
        
        # 3. Apply camera tilt (rotation around X axis)
        # Rotation matrix for rotation around X axis:
        # [1      0           0     ]
        # [0  cos(θ)   -sin(θ) ]
        # [0  sin(θ)    cos(θ) ]
        
        cos_tilt = self._cos_tilt
        sin_tilt = self._sin_tilt
        
        # Apply rotation
        ray_tilted = np.array([
            ray_cam[0],  # X unchanged
            ray_cam[1] * cos_tilt - ray_cam[2] * sin_tilt,  # Y rotated
            ray_cam[1] * sin_tilt + ray_cam[2] * cos_tilt   # Z rotated
        ])
        
        # 4. Check if ray points forward and downward
        if ray_tilted[2] <= 0:  # Z component (forward)
            return None  # Ray points backward
        
        if ray_tilted[1] <= 0:  # Y component (down in camera frame)
            return None  # Ray points upward (won't hit ground)
        
        # 5. Find intersection with ground plane
        # Camera is at height h above ground
        # Ground plane: z_world = 0
        # Camera frame Z axis (after tilt) points forward and down
        # 
        # We need to find where ray hits z = 0
        # The camera Y axis (after tilt) has a component pointing down
        # 
        # Convert to world frame:
        # - Camera X → World Y (right → left in world)
        # - Camera Y → World -Z (down → down in world)  
        # - Camera Z → World X (forward → forward in world)
        
        # In world frame, ground is at Z = 0
        # Camera is at height h (Z = h in world)
        # Ray direction in world frame:
        ray_world_x = ray_tilted[2]   # forward
        ray_world_y = -ray_tilted[0]  # left (camera right → world left)
        ray_world_z = -ray_tilted[1]  # down (camera down → world down)
        
        # Parametric ray equation: P = P_cam + t * ray_world
        # Camera position: (offset_x, offset_y, h)
        # Ground plane: Z = 0
        # Solve: h + t * ray_world_z = 0
        
        if ray_world_z >= 0:  # Ray points up in world frame
            return None
        
        t = -self.camera_height / ray_world_z
        
        if t <= 0:
            return None  # Intersection behind camera
        
        # 6. Calculate 3D point in world frame
        point_x = self.camera_offset_x + t * ray_world_x
        point_y = self.camera_offset_y + t * ray_world_y
        point_z = 0.0  # On ground
        
        return (point_x, point_y, point_z)
    
    def is_ready(self) -> bool:
        """Static provider is always ready"""
        return True
    
    def get_frame_id(self) -> str:
        """Return frame ID for point cloud"""
        return self._frame_id
    
    def get_info(self) -> dict:
        """Get provider configuration info"""
        return {
            'type': 'static',
            'camera_height': self.camera_height,
            'camera_tilt_deg': np.degrees(self.camera_tilt),
            'camera_offset': [self.camera_offset_x, self.camera_offset_y],
            'frame_id': self._frame_id
        }