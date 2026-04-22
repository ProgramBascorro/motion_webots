#!/usr/bin/env python3
"""
Launch file dengan VIO menggantikan gt_odom_to_amcl
AMCL config: base_frame_id = cam_link (sesuai verifikasi)

Perubahan dari versi sebelumnya:
- max_particles     : 2000 → 1500
- resample_interval : 1    → 2
- laser_max_beams   : 60   → 36
- VIO smoothing_alpha    : 0.85  → 0.5   (lebih responsif)
- VIO min_visual_motion  : 0.02  → 0.01  (lebih sensitif)
- VIO: tambah parameter baru (orig_image_width/height, use_vp_yaw, vp_every_n_frames)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc     = get_package_share_directory('soccer_object_localization')
    config_file = os.path.join(pkg_loc, 'config', 'op3_sim.yaml')

    white_threshold_arg = DeclareLaunchArgument(
        'white_threshold',
        default_value='165',
        description='White detection threshold'
    )

    # ── Static TF tree (base_link → head_link → cam_link) ──
    static_tf_publisher = Node(
        package='op3_utra_bridge',
        executable='op3_static_transforms.py',
        name='static_tf_publisher',
        output='screen'
    )

    # ── VIO Node ────────────────────────────────────────────
    vio_node = Node(
        package='soccer_object_localization',
        executable='vio_node_light',
        name='vio_node',
        output='screen',
        parameters=[{
            # Frame config - HARUS match dengan AMCL
            'camera_frame':  'cam_link',
            'base_frame':    'cam_link',
            'odom_frame':    'odom',

            # Camera params
            'focal_length':       900.0,
            'camera_height':      0.475,
            'camera_tilt':       -0.349,
            'orig_image_width':   1280,   # resolusi asli kamera
            'orig_image_height':  720,    # VIO akan resize ke 320x240 internal

            # Smoothing — DITURUNKAN dari 0.85 agar lebih responsif
            'smoothing_alpha':    0.5,
            'min_visual_motion':  0.01,   # DITURUNKAN dari 0.02

            # Motion detection
            'motion_threshold':   0.005,

            # Covariance — naikkan agar AMCL lebih percaya scan
            'pose_cov_xy':    0.5,
            'pose_cov_yaw':   0.2,
            'twist_cov_xy':   0.5,
            'twist_cov_yaw':  0.2,

            # VP yaw stabilization (hemat CPU: hitung tiap 5 frame)
            'use_vp_yaw':        True,
            'vp_every_n_frames': 5,
        }],
        remappings=[
            ('/robotis_op3/camera/image_raw', '/robotis_op3/camera/image_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )

    # ── Field Line Detector ─────────────────────────────────
    # TIDAK ADA PERUBAHAN — deteksi sudah baik, spacing tetap 12
    detector_node = Node(
        package='soccer_object_localization',
        executable='detector_fieldline_enhanced2',
        name='detector_fieldline',
        output='screen',
        parameters=[
            config_file,
            {
                'use_dynamic_tf': False,

                'camera.height':    0.475,
                'camera.tilt':     -0.349,
                'camera.offset_x':  0.08,
                'camera.offset_y':  0.0,
                'camera.focal_length': 900.0,

                'frames.camera': 'cam_link',
                'frames.base':   'cam_link',
                'frames.world':  'odom',

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

                'point_cloud.spacing':      12,   # TIDAK DIUBAH
                'point_cloud.max_distance': 5.5,
                'point_cloud.min_points':   5,

                'publish.debug_image':  True,
                'publish.point_cloud':  True,
            }
        ],
        remappings=[
            ('/camera/image_raw',   '/robotis_op3/camera/image_raw'),
            ('/camera/camera_info', '/robotis_op3/camera/camera_info'),
        ]
    )

    # ── Simple PC to LaserScan ──────────────────────────────
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
        }]
    )

    # ── Map Server ──────────────────────────────────────────
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

    # ── AMCL ────────────────────────────────────────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'odom_frame_id':   'odom',
            'base_frame_id':   'cam_link',   # VERIFIED
            'global_frame_id': 'map',
            'scan_topic':      'field_scan',

            'min_particles': 500,
            'max_particles': 1500,           # 2000 → 1500

            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,

            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.1,
            'alpha2': 0.1,
            'alpha3': 0.05,
            'alpha4': 0.05,
            'alpha5': 0.1,

            'update_min_d':      0.05,
            'update_min_a':      0.05,
            'resample_interval': 2,          # 1 → 2

            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            36,  # 60 → 36
            'laser_z_hit':                0.5,
            'laser_z_rand':               0.5,
            'laser_sigma_hit':            0.2,

            'set_initial_pose':  True,
            'initial_pose.x':    0.0,
            'initial_pose.y':    0.0,
            'initial_pose.z':    0.0,
            'initial_pose.yaw':  0.0,

            'transform_tolerance': 1.0,
            'tf_broadcast':        True,
            'save_pose_rate':      2.0,
        }]
    )

    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['amcl']}]
    )

    # ── Particle Converter ──────────────────────────────────
    particle_converter_node = Node(
        package='soccer_object_localization',
        executable='particle_converter',
        name='particle_converter',
        output='screen',
    )

    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        vio_node,
        detector_node,
        simple_pc2scan_node,
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
        particle_converter_node,
    ])