#!/usr/bin/env python3
"""
MINIMAL WORKING AMCL SYSTEM
Absolutely minimal configuration - only essentials
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_loc = get_package_share_directory('soccer_object_localization')
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(pkg_loc, 'maps', 'soccer_field.yaml'),
            'use_sim_time': False
        }]
    )
    
    # Map lifecycle
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Detector (your existing one)
    detector = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        output='screen',
        parameters=[{
            'detection.white_threshold': 170,
            'point_cloud.max_distance': 5.0,
            'point_cloud.min_points': 50,
        }],
        remappings=[
            ('/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )
    
    # Simple converter
    converter = Node(
        package='soccer_object_localization',
        executable='simple_pc2scan.py',
        name='simple_pc2scan',
        output='screen',
        parameters=[{
            'range_min': 0.3,
            'range_max': 5.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'num_readings': 360,
        }]
    )
    
    # AMCL - MINIMAL CONFIG
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # Frames - CRITICAL!
            'base_frame_id': 'cam_link',  # Match your scan frame!
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            
            # Particles - START SMALL
            'min_particles': 500,
            'max_particles': 1000,
            
            # Update - LESS SENSITIVE
            'update_min_d': 0.1,  # 10cm
            'update_min_a': 0.2,  # ~11 degrees
            
            # Odometry - DON'T TRUST (we have static odom)
            'alpha1': 0.001,
            'alpha2': 0.001,
            'alpha3': 0.001,
            'alpha4': 0.001,
            'alpha5': 0.001,
            
            # Laser - FORGIVING
            'laser_max_beams': 60,
            'laser_min_range': 0.3,
            'laser_max_range': 5.0,
            'laser_likelihood_max_dist': 0.5,
            
            # Initial pose
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0,
            
            # Transform
            'transform_tolerance': 1.0,
            'tf_broadcast': True,
        }],
        remappings=[
            ('scan', 'field_scan'),  # Use our converted scan
        ]
    )
    
    # AMCL lifecycle
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl']
        }]
    )
    
    # Static transforms - base_link → head_link → cam_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        arguments=[
            '0', '0', '0.395',  # Translation (x, y, z)
            '0', '0', '0', '1',  # Rotation (qx, qy, qz, qw)
            'base_link', 'head_link'
        ]
    )
    
    static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher2',
        output='screen',
        arguments=[
            '0.08', '0', '0.08',  # Translation
            '0', '-0.174', '0', '0.985',  # Rotation (~20 degrees pitch)
            'head_link', 'cam_link'
        ]
    )
    
    return LaunchDescription([
        map_server,
        map_lifecycle,
        detector,
        converter,
        static_tf,
        static_tf2,
        amcl,
        amcl_lifecycle,
    ])