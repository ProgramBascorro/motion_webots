#!/usr/bin/env python3
"""
Phase 3A: Odometry-Only Localization for OP3
Runs UKF with only odometry and IMU (no vision)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Webots) or real robot'
    )
    
    # Get package directories
    op3_utra_bridge_dir = get_package_share_directory('op3_utra_bridge')
    
    # Include bridges (odometry, IMU, robot state)
    bridges_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_utra_bridge_dir, 'launch', 'bridges.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim')
        }.items()
    )
    
    # Localization node (UKF with odometry only)
    localization_node = Node(
        package='soccer_localization',
        executable='main',
        name='localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim'),
            # Input topics
            'odom_topic': '/odom_combined',
            'state_topic': '/state',
            'imu_topic': '/imu/data',
            'field_lines_topic': '/field_point_cloud',  # Empty for now
            # Process noise (tune these for odometry-only)
            'process_noise_x': 0.1,
            'process_noise_y': 0.1,
            'process_noise_theta': 0.05,
            # Measurement noise
            'odom_noise_x': 0.05,
            'odom_noise_y': 0.05,
            'odom_noise_theta': 0.02,
            # Initial uncertainty
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_theta': 0.0,
            'initial_cov': 0.1,
        }],
        remappings=[
            ('/pose', '/amcl_pose'),  # Output pose
        ]
    )
    
    # Static transform: world -> odom (for now, they're the same)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_arg,
        bridges_launch,
        localization_node,
        static_tf_world_odom,
    ])