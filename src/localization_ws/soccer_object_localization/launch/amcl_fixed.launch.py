#!/usr/bin/env python3
"""
AMCL Launch File - FIXED VERSION
Fixes pointcloud_to_laserscan height filtering issue
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('soccer_object_localization')
    map_file = os.path.join(pkg_dir, 'maps', 'soccer_field.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # ================================================================
    # 1. Map Server
    # ================================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ================================================================
    # 2. Lifecycle Manager (to activate map_server)
    # ================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # ================================================================
    # 3. PointCloud to LaserScan - FIXED PARAMETERS
    # ================================================================
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/field_point_cloud'),
            ('scan', '/field_scan')
        ],
        parameters=[{
            # CRITICAL FIX: Remove target_frame to keep original frame
            'target_frame': '',  # Empty = use source frame (cam_link)
            
            # CRITICAL FIX: Expand height range to accept all Z values
            'min_height': -10.0,  # Accept all Z coordinates
            'max_height': 10.0,
            
            # Scan parameters
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # +180 degrees
            'angle_increment': 0.0174533,  # ~1 degree
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 10.0,
            
            # Transform settings
            'transform_tolerance': 1.0,  # Increase tolerance
            
            # Concurrency
            'concurrency_level': 0,  # Single-threaded for stability
            
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ================================================================
    # 4. AMCL - Using cam_link as base_frame
    # ================================================================
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # Frame IDs - CRITICAL: use cam_link as base
            'base_frame_id': 'cam_link',  # Changed from base_link!
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            
            # Scan topic
            'scan_topic': '/field_scan',
            
            # Initial pose (center of field)
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            
            # Particle filter parameters
            'min_particles': 500,
            'max_particles': 2000,
            
            # Odometry model (all zero since no odometry)
            'odom_model_type': 'diff',
            'odom_alpha1': 0.0,  # No rotation noise
            'odom_alpha2': 0.0,  # No rotation noise
            'odom_alpha3': 0.0,  # No translation noise
            'odom_alpha4': 0.0,  # No translation noise
            'odom_alpha5': 0.0,
            
            # Laser model
            'laser_model_type': 'likelihood_field',
            'laser_max_range': 8.0,
            'laser_min_range': 0.1,
            
            'laser_likelihood_max_dist': 2.0,
            'laser_z_hit': 0.95,
            'laser_z_rand': 0.05,
            
            # Update parameters - AGGRESSIVE for vision-only
            'update_min_d': 0.01,      # Update every 1cm
            'update_min_a': 0.05,      # Update every ~3 degrees
            'resample_interval': 1,     # Resample every update
            
            # Recovery
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            
            # Performance
            'transform_tolerance': 1.0,
            'always_reset_initial_pose': False,
            
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ================================================================
    # 5. Static Transform: odom -> map (since no odometry)
    # ================================================================
    static_tf_odom_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_map',
        arguments=[
            '0', '0', '0',      # x, y, z
            '0', '0', '0', '1', # qx, qy, qz, qw
            'map', 'odom'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ================================================================
    # Delayed start for pc_to_scan (wait for TF)
    # ================================================================
    pc_to_scan_delayed = TimerAction(
        period=2.0,  # Wait 2 seconds for TF to stabilize
        actions=[pc_to_scan_node]
    )
    
    # ================================================================
    # Launch Description
    # ================================================================
    return LaunchDescription([
        use_sim_time_arg,
        
        # Core nodes
        map_server,
        lifecycle_manager,
        static_tf_odom_map,
        
        # Delayed pointcloud conversion
        pc_to_scan_delayed,
        
        # AMCL
        amcl_node,
    ])