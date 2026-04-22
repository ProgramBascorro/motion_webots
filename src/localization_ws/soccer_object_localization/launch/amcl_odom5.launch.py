#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v12

Stack lengkap:
  joint_odom_node      → /odom + TF odom→base_link (10Hz)
  odom_constraint_node → /initialpose tiap 3s (repin partikel ke odom)
  pose_filter_node     → filter /amcl_pose, output /robot_pose
  AMCL                 → alpha kecil, partikel sedikit
  Perception stack     → detector → pc2scan → particle_converter

Cara pakai:
  1. Launch sistem
  2. Klik 2D Pose Estimate di RViz SEKALI di posisi robot
  3. odom_constraint dan pose_filter aktif setelah pose di-set
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
        'white_threshold', default_value='165')

    # ════════════════════════════════════════════════════════
    # t = 0s
    # ════════════════════════════════════════════════════════

    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
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

    # ════════════════════════════════════════════════════════
    # t = 1s
    # ════════════════════════════════════════════════════════

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )
    delayed_lifecycle_map = TimerAction(
        period=1.0, actions=[lifecycle_manager_map])

    # ════════════════════════════════════════════════════════
    # t = 2s: Odometry stack
    # ════════════════════════════════════════════════════════

    joint_odom_node = Node(
        package='soccer_object_localization',
        executable='joint_odom_node',
        name='joint_odom_node',
        output='screen',
        parameters=[{
            'base_frame':         'base_link',
            'odom_frame':         'odom',
            'use_imu_yaw':        True,
            'step_scale':          0.8,
            'stance_l_min':        1.0,
            'stance_r_min':        1.0,
            'cov_xy':              0.02,
            'cov_yaw':             0.01,
            'publish_rate':        10.0,
            # r_fx sudah di-negate di joint_odom_node.py
            # forward_sign=-1.0 diperlukan karena konvensi FK OP3
            # (kombinasi r_fx=-fk() + forward_sign=-1.0 = maju positif)
            'forward_sign':       -1.0,
            'imu_yaw_offset_deg':  0.0,
        }]
    )

    odom_constraint_node = Node(
        package='soccer_object_localization',
        executable='odom_constraint_node',
        name='odom_constraint_node',
        output='screen',
        parameters=[{
            'constraint_interval': 3.0,
            'cov_xy':              0.04,
            'cov_yaw':             0.02,
            'min_move':            0.02,
            # Covariance RViz default=0.25, node kita=0.04
            # threshold=0.15 → RViz > threshold → diterima sebagai user set
            'rviz_cov_threshold':  0.15,
        }]
    )

    pose_filter_node = Node(
        package='soccer_object_localization',
        executable='pose_filter_node',
        name='pose_filter_node',
        output='screen',
        parameters=[{
            'max_jump_xy':   0.3,
            'max_jump_yaw':  0.5,
            'ema_alpha_xy':  0.2,
            'ema_alpha_yaw': 0.25,
            'amcl_timeout':  2.0,
            'max_amcl_cov':  1.0,
            'max_velocity':  0.2,
            'publish_tf':    True,
        }]
    )

    # ════════════════════════════════════════════════════════
    # t = 3s: Perception
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
                'frames.camera':             'cam_link',
                'frames.base':               'cam_link',
                'frames.world':              'odom',
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
                'point_cloud.max_distance':  4.0,
                'point_cloud.min_points':    8,
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
            'angle_increment':  0.0349,
            'range_min':        0.2,
            'range_max':        4.0,
            'scan_height':      0.05,
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
            'odom_frame_id':              'odom',
            'base_frame_id':              'cam_link',
            'global_frame_id':            'map',
            'scan_topic':                 'field_scan',
            'min_particles':               100,
            'max_particles':               300,
            'recovery_alpha_slow':          0.0,
            'recovery_alpha_fast':          0.0,
            'robot_model_type':            'nav2_amcl::DifferentialMotionModel',
            'alpha1':                       0.001,
            'alpha2':                       0.001,
            'alpha3':                       0.0005,
            'alpha4':                       0.0005,
            'alpha5':                       0.001,
            'update_min_d':                 0.02,
            'update_min_a':                 0.03,
            'resample_interval':            3,
            'laser_model_type':            'likelihood_field',
            'laser_likelihood_max_dist':    0.2,
            'laser_max_range':              4.0,
            'laser_min_range':              0.2,
            'laser_max_beams':              30,
            'laser_z_hit':                  0.95,
            'laser_z_rand':                 0.05,
            'laser_sigma_hit':              0.1,
            'set_initial_pose':             False,
            'transform_tolerance':          1.5,
            'tf_broadcast':                 True,
            'always_reset_initial_pose':    False,
            'first_map_only':               False,
            'save_pose_rate':               0.5,
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

    delayed_odom = TimerAction(
        period=2.0,
        actions=[joint_odom_node, odom_constraint_node, pose_filter_node]
    )
    delayed_perception = TimerAction(
        period=3.0,
        actions=[detector_node, simple_pc2scan_node, particle_converter_node]
    )
    delayed_amcl = TimerAction(
        period=5.0,
        actions=[amcl_node, lifecycle_manager_amcl]
    )

    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        map_server_node,
        delayed_lifecycle_map,
        delayed_odom,
        delayed_perception,
        delayed_amcl,
    ])