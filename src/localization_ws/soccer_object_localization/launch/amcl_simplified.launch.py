#!/usr/bin/env python3
"""
Simplified AMCL Launch - Uses cam_link frame directly
No TF transform in pointcloud_to_laserscan
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_loc = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')
    
    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold', default_value='170')
    
    # Static odom
    odom_pub = Node(
        package='op3_utra_bridge',
        executable='odom_publisher_static.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # Static TF
    static_tf = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # Map to odom
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Detector
    detector = Node(
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
    
    # PointCloud to LaserScan - SIMPLIFIED!
    pc_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            # NO TRANSFORM! Use original frame
            'target_frame': '',  # Empty = use input frame (cam_link)
            'transform_tolerance': 1.0,
            
            # NO HEIGHT FILTERING!
            'min_height': -100.0,
            'max_height': 100.0,
            
            # Scan config
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0349066,  # 2 degrees (faster)
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 5.0,
            'use_inf': True,
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/field_point_cloud'),
            ('scan', '/field_scan'),
        ]
    )
    
    # Map server
    map_file = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')
    map_server = Node(
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
    
    # AMCL - Using cam_link!
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # CRITICAL: Use cam_link as base!
            'odom_frame_id': 'odom',
            'base_frame_id': 'cam_link',  # Changed!
            'global_frame_id': 'map',
            
            # Particles
            'min_particles': 500,
            'max_particles': 2000,
            
            # Motion model (minimal, no real odom)
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.0001,
            'alpha2': 0.0001,
            'alpha3': 0.0001,
            'alpha4': 0.0001,
            'alpha5': 0.0001,
            
            # Update frequently
            'update_min_d': 0.05,
            'update_min_a': 0.1,
            'resample_interval': 1,
            
            # Laser
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 0.5,
            'laser_max_range': 5.0,
            'laser_min_range': 0.3,
            'laser_max_beams': 30,
            
            # Initial pose
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0,
            
            # TF
            'transform_tolerance': 0.5,
            'tf_broadcast': True,
        }],
        remappings=[('scan', '/field_scan')]
    )
    
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
    
    # Add delays for TF propagation
    pc_to_scan_delayed = TimerAction(
        period=2.0,
        actions=[pc_to_scan]
    )
    
    return LaunchDescription([
        white_threshold_arg,
        odom_pub,
        static_tf,
        map_to_odom,
        detector,
        pc_to_scan_delayed,  # Delayed start
        map_server,
        map_lifecycle,
        amcl,
        amcl_lifecycle,
    ])