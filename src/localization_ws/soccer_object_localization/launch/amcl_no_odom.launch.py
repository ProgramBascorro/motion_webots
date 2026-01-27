#!/usr/bin/env python3
"""
Complete Localization Pipeline WITHOUT Odometry
For testing AMCL with vision-only input
Uses static transforms instead of dynamic odometry
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate localization pipeline without odometry requirement"""
    
    # Package directories
    pkg_loc = get_package_share_directory('soccer_object_localization')
    
    # Config files
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')
    amcl_config = os.path.join(pkg_loc, 'config', 'amcl_no_odom.yaml')
    
    # Arguments
    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='180',
        description='White detection threshold'
    )
    
    # ===== NODE 1: Static odom frame =====
    odom_publisher = Node(
        package='op3_utra_bridge',
        executable='odom_publisher_static.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # ===== NODE 2: Static TF tree =====
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # ===== NODE 3: Field Line Detector (STATIC MODE) =====
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_hybrid',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                # STATIC MODE - no TF lookup
                'use_dynamic_tf': False,
                
                # Detection parameters
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'point_cloud.spacing': 20,
                'point_cloud.max_distance': 5.0,
                
                # Camera parameters (from camera_info)
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
    
    # ===== NODE 4: PointCloud to LaserScan =====
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            # CRITICAL FIX: Transform to base_link!
            'target_frame': 'base_link',  # NOT cam_link!
            'transform_tolerance': 0.2,
            
            # Height filtering - RELAXED for tilted camera
            'min_height': -0.5,  # Allow points below base_link
            'max_height': 0.5,   # Allow points above base_link
            
            # Scan parameters
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,  # 1 degree
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 5.0,
            'use_inf': True,
            
            # Concurrency
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/field_point_cloud'),
            ('scan', '/field_scan'),
        ]
    )
    
    # ===== NODE 5: Map Server =====
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
    
    # ===== NODE 6: Map Server Lifecycle Manager =====
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
    
    # ===== NODE 7: AMCL (Modified for no odometry) =====
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # TF frames
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            
            # Particle filter - MORE particles for vision-only
            'min_particles': 500,
            'max_particles': 2000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            
            # Motion model - VERY LOW noise since no real odometry
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.0001,
            'alpha2': 0.0001,
            'alpha3': 0.0001,
            'alpha4': 0.0001,
            'alpha5': 0.0001,
            
            # Update thresholds - UPDATE MORE FREQUENTLY
            'update_min_d': 0.05,  # Update every 5cm movement
            'update_min_a': 0.1,   # Update every ~6 degree rotation
            'resample_interval': 1,
            
            # Laser model
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 0.3,
            'laser_max_range': 5.0,
            'laser_min_range': 0.3,
            'laser_max_beams': 60,  # Use 60 beams from scan
            'laser_z_hit': 0.95,
            'laser_z_rand': 0.05,
            'laser_sigma_hit': 0.2,
            
            # Initial pose at field center
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            
            # Transform tolerances
            'transform_tolerance': 0.2,
            'tf_broadcast': True,
            
            # CRITICAL: Always update on scan
            'always_reset_initial_pose': False,
            'first_map_only': False,
        }],
        remappings=[
            ('scan', '/field_scan'),
        ]
    )
    
    # ===== NODE 8: AMCL Lifecycle Manager =====
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
    
    # ===== NODE 9: Map→Odom Static Transform =====
    # AMCL will publish map→odom based on localization
    # But initially, set them equal
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
        pc_to_scan_node,
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
    ])