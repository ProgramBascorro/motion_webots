#!/usr/bin/env python3
"""
FINAL CORRECTED: Complete Localization - Launch File 5 with TF Fixed
KEY FIXES:
1. Added gt_odom_node and gt_odom_to_amcl (from launch file 4)
2. This creates TF chain: map → odom → base_link → head_link → cam_link
3. Enhanced detector with optimized parameters
4. Complete particle cloud support
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
        default_value='165',
        description='White detection threshold'
    )
    
    # Static TF tree (base_link → head_link → cam_link)
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # ===== GROUND TRUTH ODOMETRY (CRITICAL FOR TF!) =====
    # These nodes create the odom → base_link transform
    # WITHOUT these, TF chain is broken!
    
    gt_odom_node = Node(
        package='gt_localization',
        executable='gt_odom_node',
        name='gt_odom_node',
        output='screen',
    )
    
    gt_odom_to_amcl_node = Node(
        package='gt_localization',
        executable='gt_odom_to_amcl',
        name='gt_odom_to_amcl',
        output='screen',
    )
    
    # Field Line Detector - ENHANCED VERSION
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                # ===== STATIC MODE =====
                'use_dynamic_tf': False,
                
                # ===== STATIC CAMERA PARAMETERS =====
                'camera.height': 0.475,
                'camera.tilt': -0.349,
                'camera.offset_x': 0.08,
                'camera.offset_y': 0.0,
                'camera.focal_length': 900.0,
                'camera.image_width': 1280,
                'camera.image_height': 720,
                
                # ===== DETECTION PARAMETERS =====
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced': True,
                'detection.roi_top_cut': 0.35,
                'detection.roi_bottom_cut': 0.08,
                'detection.min_line_length': 15,
                'detection.max_line_gap': 25,
                'detection.canny_low': 60,
                'detection.canny_high': 180,
                'detection.hough_threshold': 60,
                'detection.remove_grass': True,
                'detection.grass_h_low': 35,
                'detection.grass_h_high': 85,
                'detection.grass_s_low': 40,
                
                # ===== POINT CLOUD =====
                'point_cloud.spacing': 12,
                'point_cloud.max_distance': 5.5,
                'point_cloud.min_points': 5,
                
                # ===== PUBLISHING =====
                'publish.debug_image': True,
                'publish.point_cloud': True,
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
    
    # AMCL - CORRECTED CONFIGURATION
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            # ===== FRAME IDS (CORRECTED!) =====
            'odom_frame_id': 'odom',
            'base_frame_id': 'cam_link',      # FIXED: was cam_link
            'global_frame_id': 'map',
            
            # ===== SCAN TOPIC (EXPLICIT!) =====
            'scan_topic': 'field_scan',
            
            # ===== PARTICLE FILTER =====
            'min_particles': 1000,
            'max_particles': 3000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            
            # ===== MOTION MODEL =====
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.000001,
            'alpha2': 0.000001,
            'alpha3': 0.000001,
            'alpha4': 0.000001,
            'alpha5': 0.000001,
            
            # ===== UPDATE THRESHOLDS =====
            'update_min_d': 0.01,
            'update_min_a': 0.01,
            'resample_interval': 1,
            
            # ===== LASER MODEL =====
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 0.5,
            'laser_max_range': 5.0,
            'laser_min_range': 0.3,
            'laser_max_beams': 60,
            'laser_z_hit': 0.5,
            'laser_z_rand': 0.5,
            'laser_sigma_hit': 0.2,
            
            # ===== INITIAL POSE =====
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            
            # ===== TRANSFORM =====
            'transform_tolerance': 1.0,
            'tf_broadcast': True,
            'always_reset_initial_pose': False,
            'first_map_only': False,
            
            # ===== PARTICLE CLOUD =====
            'save_pose_rate': 2.0,
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
    
    # Particle Cloud Converter
    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )
    
    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        gt_odom_node,              # ADDED! (from launch file 4)
        gt_odom_to_amcl_node,      # ADDED! (from launch file 4)
        detector_node,
        simple_pc2scan_node,
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
        particle_converter_node,
    ])