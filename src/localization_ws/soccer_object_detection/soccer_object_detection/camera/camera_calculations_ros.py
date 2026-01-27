#!/usr/bin/env python3
"""
Camera Calculations ROS2 - Compatible with existing UTRA code
Handles TF transformations and camera position calculations
"""

import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

# Import custom modules
from soccer_object_detection.camera.camera_calculations import CameraCalculations
from soccer_common.transformation import Transformation


class CameraCalculationsRos(CameraCalculations):
    """
    ROS2 wrapper for camera calculations with TF2 support
    Handles camera pose estimation and coordinate transformations
    """

    def __init__(self, node, robot_name: str):
        """
        Initialize camera calculations with ROS2 node
        
        Args:
            node: ROS2 node instance for logging and subscriptions
            robot_name: Name of the robot (used for TF frame naming)
        """
        super().__init__()
        
        self.node = node
        self.robot_name = robot_name
        
        # Initialize camera pose
        self.pose_base_link_straight = Transformation()
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        # Camera info subscription
        self.camera_info_subscription = self.node.create_subscription(
            CameraInfo,
            "camera/camera_info",
            self.camera_info_callback,
            10
        )
        
        # Store initialization time
        self.init_time = node.get_clock().now()
        
        self.node.get_logger().info(
            f"Camera calculations initialized for robot: {robot_name}"
        )

    def camera_info_callback(self, camera_info: CameraInfo):
        """
        Callback for camera info subscription
        Stores camera intrinsic parameters
        
        Args:
            camera_info: CameraInfo message from camera driver
        """
        if not hasattr(self, 'camera_info') or self.camera_info is None:
            self.node.get_logger().info("Camera info received and stored")
        
        self.camera_info = camera_info

    def reset_position(
        self, 
        timestamp=None, 
        camera_frame="camera", 
        skip_if_not_found=False
    ):
        """
        Reset camera position using TF2 transforms
        Calculates camera pose relative to robot base
        
        Args:
            timestamp: Specific time for TF lookup (None = latest)
            camera_frame: Name of camera frame in TF tree
            skip_if_not_found: If True, skip waiting for transform
        """
        try:
            # Use current time if timestamp not provided
            if timestamp is None:
                timestamp = Time()

            # Primary method: Get transform from head to left_foot
            # This gives us the camera height and orientation
            source_frame = "head"
            target_frame = "left_foot"
            
            if self.tf_buffer.can_transform(
                source_frame, 
                target_frame, 
                timestamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            ):
                # Lookup transform
                transform_stamped = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    timestamp
                )
                
                # Extract translation and rotation
                trans = transform_stamped.transform.translation
                rot = transform_stamped.transform.rotation
                
                # Create transformation with Z height and rotation
                self.pose = Transformation(
                    position=[0, 0, trans.z],
                    quaternion=[rot.x, rot.y, rot.z, rot.w]
                )
                
                # Apply orientation correction
                # Multiply roll by -1 to correct camera orientation
                euler = self.pose.orientation_euler
                euler[0] *= -1  # Flip roll
                self.pose.orientation_euler = euler
                
                self.node.get_logger().debug(
                    f"Camera pose updated: height={trans.z:.3f}m",
                    throttle_duration_sec=5.0
                )
                
            else:
                self.node.get_logger().warn(
                    f"Cannot transform {source_frame} to {target_frame}",
                    throttle_duration_sec=5.0
                )

        except Exception as e:
            self.node.get_logger().warn(
                f"Failed to get camera transform: {str(e)}",
                throttle_duration_sec=5.0
            )

    def reset_position_alternative(
        self,
        timestamp=None,
        from_world_frame=False,
        camera_frame="camera"
    ):
        """
        Alternative method for camera pose calculation
        Uses odom or world frame transformations
        
        NOTE: This is kept for reference but not currently used
        Uncomment and adapt as needed for your TF tree structure
        
        Args:
            timestamp: Specific time for TF lookup
            from_world_frame: Use world frame instead of odom
            camera_frame: Name of camera frame
        """
        if timestamp is None:
            timestamp = Time()
        
        try:
            if from_world_frame:
                # Method 1: Direct world to camera transform
                source_frame = "world"
                target_frame = f"{self.robot_name}/{camera_frame}"
                
                if self.tf_buffer.can_transform(
                    source_frame,
                    target_frame,
                    timestamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                ):
                    transform_stamped = self.tf_buffer.lookup_transform(
                        source_frame,
                        target_frame,
                        timestamp
                    )
                    
                    trans = transform_stamped.transform.translation
                    rot = transform_stamped.transform.rotation
                    
                    self.pose = Transformation(
                        position=[trans.x, trans.y, trans.z],
                        quaternion=[rot.x, rot.y, rot.z, rot.w]
                    )
                    return
                    
            else:
                # Method 2: Relative to robot odom frame
                odom_frame = f"{self.robot_name}/odom"
                base_frame = f"{self.robot_name}/base_footprint"
                camera_full_frame = f"{self.robot_name}/{camera_frame}"
                
                # Get base_footprint transform
                if self.tf_buffer.can_transform(
                    odom_frame,
                    base_frame,
                    timestamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                ):
                    base_transform = self.tf_buffer.lookup_transform(
                        odom_frame,
                        base_frame,
                        timestamp
                    )
                    
                    trans = base_transform.transform.translation
                    rot = base_transform.transform.rotation
                    
                    world_to_base_link = Transformation(
                        position=[trans.x, trans.y, trans.z],
                        quaternion=[rot.x, rot.y, rot.z, rot.w]
                    )
                    
                    # Zero out pitch and roll to get straight base
                    euler = world_to_base_link.orientation_euler
                    euler[0] = 0  # Roll
                    euler[1] = 0  # Pitch
                    world_to_base_link.orientation_euler = euler
                    self.pose_base_link_straight = world_to_base_link
                    
                    # Get camera transform
                    if self.tf_buffer.can_transform(
                        odom_frame,
                        camera_full_frame,
                        timestamp,
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    ):
                        camera_transform = self.tf_buffer.lookup_transform(
                            odom_frame,
                            camera_full_frame,
                            timestamp
                        )
                        
                        trans = camera_transform.transform.translation
                        rot = camera_transform.transform.rotation
                        
                        world_to_camera = Transformation(
                            position=[trans.x, trans.y, trans.z],
                            quaternion=[rot.x, rot.y, rot.z, rot.w]
                        )
                        
                        # Calculate relative camera pose
                        # camera_to_base_link = inv(world_to_base) @ world_to_camera
                        import numpy as np
                        camera_to_base_link = (
                            np.linalg.inv(world_to_base_link.matrix) @ 
                            world_to_camera.matrix
                        )
                        
                        self.pose = Transformation(matrix=camera_to_base_link)
                        return
                        
        except Exception as e:
            self.node.get_logger().error(
                f"Failed alternative camera transform: {str(e)}"
            )


def main(args=None):
    """
    Standalone main function for testing
    Not normally used - CameraCalculationsRos is instantiated by ObjectDetectionNodeRos
    """
    rclpy.init(args=args)
    
    # This would require a proper node implementation
    # Currently CameraCalculationsRos expects to be given a node instance
    print("CameraCalculationsRos is not meant to be run standalone")
    print("It should be instantiated by ObjectDetectionNodeRos")
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()