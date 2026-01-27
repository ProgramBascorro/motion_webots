#!/usr/bin/env python3
"""
Phase 3B: Full Localization with Vision for OP3
Runs UKF with odometry, IMU, and field line detection
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
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera/vision processing'
    )
    
    # Get package directories
    op3_utra_bridge_dir = get_package_share_directory('op3_utra_bridge')
    soccer_obj_loc_dir = get_package_share_directory('soccer_object_localization')
    
    # Camera topic based on sim or real
    # Simulation: /robotis_op3/camera/image_raw
    # Real robot: /camera/image_raw or check actual topic
    
    # Include bridges (odometry, IMU, robot state)
    bridges_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_utra_bridge_dir, 'launch', 'bridges.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim')
        }.items()
    )
    
    # Field line detector
    fieldline_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(soccer_obj_loc_dir, 'launch', 'fieldline_detector.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim')
        }.items()
    )
    
    # Localization node (UKF with vision)
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
            'field_lines_topic': '/field_point_cloud',
            # Process noise (lower with vision correction)
            'process_noise_x': 0.05,
            'process_noise_y': 0.05,
            'process_noise_theta': 0.03,
            # Measurement noise
            'odom_noise_x': 0.05,
            'odom_noise_y': 0.05,
            'odom_noise_theta': 0.02,
            'vision_noise_x': 0.1,   # Field line measurement noise
            'vision_noise_y': 0.1,
            'vision_noise_theta': 0.05,
            # Initial pose
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_theta': 0.0,
            'initial_cov': 0.5,  # Higher uncertainty initially
            # Vision weight
            'use_vision': True,
            'vision_weight': 0.7,  # 70% trust in vision
        }],
        remappings=[
            ('/pose', '/amcl_pose'),
        ]
    )
    
    # Static transforms
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )
    
    # Camera bridge (if needed to remap topics)
    camera_bridge = Node(
        package='op3_utra_bridge',
        executable='camera_topic_bridge.py',
        name='camera_bridge',
        parameters=[{
            'use_sim': LaunchConfiguration('use_sim'),
            'sim_camera_topic': '/robotis_op3/camera/image_raw',
            'real_camera_topic': '/camera/image_raw',
            'output_topic': '/camera/image',
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_camera'))
    )
    
    return LaunchDescription([
        use_sim_arg,
        use_camera_arg,
        bridges_launch,
        fieldline_detector_launch,
        localization_node,
        static_tf_world_odom,
    ])