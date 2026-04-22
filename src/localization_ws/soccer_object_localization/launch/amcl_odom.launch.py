#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v2
Fix utama: nama joint disesuaikan dengan Webots OP3
  l_hip_pitch, l_knee, l_ank_pitch (bukan l_hip_p, l_knee_p, l_ank_p)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc     = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')

    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold', default_value='165',
        description='White detection threshold'
    )

    # ════════════════════════════════════════════════════════
    # t = 0s
    # ════════════════════════════════════════════════════════

    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # Static odom→base_link sementara sebelum joint odom aktif
    static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_baselink',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    map_file = os.path.join(pkg_loc, 'maps', 'soccer_field.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name':    'map',
            'frame_id':      'map'
        }]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    # ════════════════════════════════════════════════════════
    # t = 2s: Joint Odom — nama joint sudah difix
    # ════════════════════════════════════════════════════════

    joint_odom_node = Node(
        package='soccer_object_localization',
        executable='joint_odom_node',
        name='joint_odom_node',
        output='screen',
        parameters=[{
            'base_frame':      'base_link',
            'odom_frame':      'odom',
            'use_imu_yaw':     True,
            'pose_cov_xy':     0.15,
            'pose_cov_yaw':    0.1,
            'smoothing_alpha': 0.5,
            'step_scale':      0.8,   # tune: naik kalau odom kurang jauh
            'dual_stance_avg': True,
        }]
    )

    # ════════════════════════════════════════════════════════
    # t = 3s: Detector + PC2Scan
    # ════════════════════════════════════════════════════════

    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf':            False,
                'camera.height':             0.475,
                'camera.tilt':              -0.349,
                'camera.offset_x':           0.08,
                'camera.offset_y':           0.0,
                'camera.focal_length':       900.0,
                'camera.image_width':        1280,
                'camera.image_height':       720,
                'detection.white_threshold': LaunchConfiguration('white_threshold'),
                'detection.use_enhanced':    True,
                'detection.roi_top_cut':     0.35,
                'detection.roi_bottom_cut':  0.08,
                'detection.min_line_length': 15,
                'detection.max_line_gap':    25,
                'detection.canny_low':       60,
                'detection.canny_high':      180,
                'detection.hough_threshold': 60,
                'detection.remove_grass':    True,
                'detection.grass_h_low':     35,
                'detection.grass_h_high':    85,
                'detection.grass_s_low':     40,
                'point_cloud.spacing':       12,
                'point_cloud.max_distance':  5.5,
                'point_cloud.min_points':    5,
                'publish.debug_image':       False,
                'publish.point_cloud':       True,
            }
        ],
        remappings=[
            ('/camera/image_raw',   '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )

    simple_pc2scan_node = Node(
        package='soccer_object_localization',
        executable='simple_pc2scan',
        name='simple_pc2scan',
        output='screen',
        parameters=[{
            'angle_min':       -3.14159,
            'angle_max':        3.14159,
            'angle_increment':  0.0174533,
            'range_min':        0.3,
            'range_max':        5.0,
            'scan_height':      0.0,
            'target_frame':     'odom',
        }]
    )

    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # t = 5s: AMCL
    # ════════════════════════════════════════════════════════

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'odom_frame_id':   'odom',
            'base_frame_id':   'cam_link',
            'global_frame_id': 'map',
            'scan_topic':      'field_scan',

            'min_particles': 300,
            'max_particles': 1000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,

            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.05,
            'alpha2': 0.05,
            'alpha3': 0.02,
            'alpha4': 0.02,
            'alpha5': 0.05,

            'update_min_d':      0.05,
            'update_min_a':      0.05,
            'resample_interval': 3,

            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            36,
            'laser_z_hit':                0.5,
            'laser_z_rand':               0.5,
            'laser_sigma_hit':            0.2,

            'set_initial_pose':           True,
            'initial_pose.x':             0.0,
            'initial_pose.y':             0.0,
            'initial_pose.z':             0.0,
            'initial_pose.yaw':           0.0,

            'transform_tolerance':        1.0,
            'tf_broadcast':               True,
            'always_reset_initial_pose':  False,
            'first_map_only':             False,
            'save_pose_rate':             2.0,
        }]
    )

    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['amcl']}]
    )

    # ════════════════════════════════════════════════════════
    # Staggered startup
    # ════════════════════════════════════════════════════════

    delayed_jointodom = TimerAction(
        period=2.0, actions=[joint_odom_node])

    delayed_perception = TimerAction(
        period=3.0,
        actions=[detector_node, simple_pc2scan_node, particle_converter_node])

    delayed_amcl = TimerAction(
        period=5.0,
        actions=[amcl_node, lifecycle_manager_amcl])

    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        static_odom_baselink,
        map_server_node,
        lifecycle_manager_map,
        delayed_jointodom,
        delayed_perception,
        delayed_amcl,
    ])