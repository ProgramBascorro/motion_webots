#!/usr/bin/env python3
"""
Complete Localization Pipeline for OP3 Soccer Robot
Includes: Field line detection → PointCloud → LaserScan → AMCL
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete localization pipeline"""
    
    # Package directories
    pkg_dir = get_package_share_directory('soccer_object_localization')
    
    # Config files
    config_file = os.path.join(pkg_dir, 'config', 'op3_sim.yaml')
    
    # Arguments
    use_dynamic_tf_arg = DeclareLaunchArgument(
        'use_dynamic_tf',
        default_value='true',
        description='Use dynamic TF (true for Webots, false for static)'
    )
    
    # ===== NODE 1: Field Line Detector =====
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf': LaunchConfiguration('use_dynamic_tf'),
                'detection.white_threshold': 210,
                'point_cloud.spacing': 30,
            }
        ],
        remappings=[
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    # ===== NODE 2: PointCloud to LaserScan =====
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            # Frame configuration
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            
            # Height filtering (only ground plane)
            'min_height': -0.1,
            'max_height': 0.1,
            
            # Scan parameters
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # +180 degrees
            'angle_increment': 0.0174533,  # ~1 degree
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 5.0,
            
            # Use_inf: publish inf for no returns
            'use_inf': True,
            
            # Concurrency level
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/field_point_cloud'),
            ('scan', '/field_scan'),
        ]
    )
    
    return LaunchDescription([
        use_dynamic_tf_arg,
        detector_node,
        pc_to_scan_node,
    ])