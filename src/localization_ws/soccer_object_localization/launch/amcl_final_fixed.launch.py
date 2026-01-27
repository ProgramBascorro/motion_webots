#!/usr/bin/env python3
"""
FINAL FIXED: Complete Localization with Simple Converter
KEY FIX: Explicit scan topic remapping for AMCL
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_loc = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')
    
    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='180',
        description='White detection threshold'
    )
    
    # Static odom publisher
    odom_publisher = Node(
        package='op3_utra_bridge',
        executable='odom_publisher_static.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # Static TF tree
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # Field Line Detector
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf': False,
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'point_cloud.spacing': 20,
                'point_cloud.max_distance': 5.0,
                'camera.focal_length': 790.38,
                'camera.height': 0.48,
                'camera.tilt': -0.3491,
            }
        ],
        remappings=[
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    # Simple PointCloud to LaserScan Converter
    simple_pc2scan_node = Node(
        package='soccer_object_localization',
        executable='simple_pc2scan',
        name='simple_pc2scan',
        output='screen',
        parameters=[{
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'range_min': 0.3,
            'range_max': 5.0,
            'scan_height': 0.0,
        }]
    )
    
    # Map Server
    map_file = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )
    
    # Map Server Lifecycle Manager
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # AMCL - WITH EXPLICIT SCAN REMAPPING
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        remappings=[
            ('scan', '/field_scan'),  # CRITICAL: Map /scan to /field_scan!
        ],
        parameters=[{
            # Frame IDs
            'odom_frame_id': 'odom',
            'base_frame_id': 'cam_link',
            'global_frame_id': 'map',
            
            # Particle filter
            'min_particles': 500,
            'max_particles': 2000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            
            # Motion model - low noise (no real odometry)
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.0001,
            'alpha2': 0.0001,
            'alpha3': 0.0001,
            'alpha4': 0.0001,
            'alpha5': 0.0001,
            
            # Update thresholds - frequent updates
            'update_min_d': 0.05,
            'update_min_a': 0.1,
            'resample_interval': 1,
            
            # Laser model
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 0.3,
            'laser_max_range': 5.0,
            'laser_min_range': 0.3,
            'laser_max_beams': 60,
            'laser_z_hit': 0.95,
            'laser_z_rand': 0.05,
            'laser_sigma_hit': 0.2,
            
            # Initial pose
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 1.57,
            
            # Transform
            'transform_tolerance': 0.2,
            'tf_broadcast': True,
            'always_reset_initial_pose': False,
            'first_map_only': False,
        }]
    )
    
    # AMCL Lifecycle Manager
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl']
        }]
    )
    
    # Map→Odom static transform
    map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        white_threshold_arg,
        odom_publisher,
        static_tf_publisher,
        map_to_odom_publisher,
        detector_node,
        simple_pc2scan_node,
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
    ])