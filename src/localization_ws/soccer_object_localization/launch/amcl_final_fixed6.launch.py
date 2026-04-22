#!/usr/bin/env python3
"""
FINAL CORRECTED: Complete Localization with Filtered Dynamic Camera
KEY FIXES:
1. FilteredDynamicCameraPose with proper smoothing parameters
2. Frame names configured correctly
3. AMCL: base_frame_id = base_link (NOT cam_link!)
4. AMCL: explicit scan_topic = field_scan
5. Optimized detector parameters
6. Particle cloud converter included
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
    
    # Static TF tree
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # Field Line Detector - FILTERED DYNAMIC MODE
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                # ===== DYNAMIC TF MODE (for moving camera) =====
                'use_dynamic_tf': False,
                
                # ===== FRAME NAMES (CRITICAL!) =====
                'frames.camera': 'cam_link',
                'frames.base': 'base_link',
                'frames.world': 'odom',
                
                # ===== SMOOTHING PARAMETERS (prevents jitter!) =====
                'camera.position_alpha': 0.7,    # Position smoothing
                'camera.rotation_alpha': 0.8,    # Rotation smoothing (yaw stability!)
                
                # ===== DETECTION PARAMETERS =====
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced': True,
                
                # ROI (optimized for all field lines)
                'detection.roi_top_cut': 0.35,        # Keeps goal line visible
                'detection.roi_bottom_cut': 0.08,     # Keeps sidelines
                
                # Hough parameters (tuned)
                'detection.min_line_length': 15,
                'detection.max_line_gap': 25,
                'detection.canny_low': 60,
                'detection.canny_high': 180,
                'detection.hough_threshold': 60,
                'detection.line_thickness': 2,
                
                # Grass removal
                'detection.remove_grass': True,
                'detection.grass_h_low': 35,
                'detection.grass_h_high': 85,
                'detection.grass_s_low': 40,
                
                # ===== POINT CLOUD PARAMETERS =====
                'point_cloud.spacing': 12,           # Dense sampling
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
            # ===== FRAME IDS (CRITICAL FIX!) =====
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',      # FIXED: was cam_link!
            'global_frame_id': 'map',
            
            # ===== SCAN TOPIC (EXPLICIT!) =====
            'scan_topic': 'field_scan',        # ADDED: explicit topic name
            
            # ===== PARTICLE FILTER =====
            'min_particles': 1000,
            'max_particles': 3000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            
            # ===== MOTION MODEL (ignore odometry) =====
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
            
            # ===== PARTICLE CLOUD PUBLISHING =====
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
    
    # Particle Cloud Converter (for RViz visualization)
    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    gt_odom_to_amcl_node = Node(
        package='gt_localization',
        executable='gt_odom_to_amcl',
        name='gt_odom_to_amcl',
        output='screen',
    )

    gt_odom_node = Node(
        package='gt_localization',
        executable='gt_odom_node',
        name='gt_odom_node',
        output='screen',
    )
    
    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        detector_node,
        simple_pc2scan_node,
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
        particle_converter_node,
        gt_odom_to_amcl_node,
        gt_odom_node,
    ])