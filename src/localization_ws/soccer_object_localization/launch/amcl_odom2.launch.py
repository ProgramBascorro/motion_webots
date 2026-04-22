#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v7 - STABLE
- Hapus global_relocalization_node (belum terdaftar di setup.py → crash)
- Tambahkan stance_knee_threshold sebagai parameter launch
  (dari log: knee=L2.478R-2.478rad, threshold lama 0.20 tidak pernah match)
- Fix frames.* eksplisit
- Fix lifecycle_manager_map delay 1s
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

    # ════════════════════════════════════════════════════════
    # t = 1s: Lifecycle Map
    # delay 1s agar map_server selesai load yaml terlebih dahulu
    # ════════════════════════════════════════════════════════

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    delayed_lifecycle_map = TimerAction(period=1.0, actions=[lifecycle_manager_map])

    # ════════════════════════════════════════════════════════
    # t = 2s: Joint Odom
    # stance_threshold parameter diexpose — bisa di-tune dari luar
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
            'pose_cov_xy':     0.3,
            'pose_cov_yaw':    0.1,
            'smoothing_alpha': 0.5,
            'step_scale':      0.8,
            'dual_stance_avg': True,
            # Stance threshold — tunable
            # Dari log: knee berjalan ~1.0-2.5 rad
            # 1.0 = deteksi semua posisi sebagai stance (loose)
            # Naikkan ke 1.5-2.0 setelah tau pola walking robot
            'stance_l_min': 1.0,
            'stance_r_min': 1.0,
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
            'min_particles':   500,
            'max_particles':   2000,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.2,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.1,
            'alpha2': 0.1,
            'alpha3': 0.05,
            'alpha4': 0.05,
            'alpha5': 0.1,
            'update_min_d':      0.01,
            'update_min_a':      0.01,
            'resample_interval': 2,
            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            60,
            'laser_z_hit':               0.8,
            'laser_z_rand':              0.2,
            'laser_sigma_hit':           0.1,
            'set_initial_pose':           True,
            'initial_pose.x':             0.0,
            'initial_pose.y':             0.0,
            'initial_pose.z':             0.0,
            'initial_pose.yaw':           0.0,
            'transform_tolerance':        3.0,   # lebih toleran terhadap delay TF
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

    delayed_jointodom  = TimerAction(period=2.0, actions=[joint_odom_node])
    delayed_perception = TimerAction(
        period=3.0,
        actions=[detector_node, simple_pc2scan_node, particle_converter_node]
    )
    delayed_amcl = TimerAction(
        period=5.0, actions=[amcl_node, lifecycle_manager_amcl]
    )

    return LaunchDescription([
        white_threshold_arg,
        static_tf_publisher,
        static_odom_baselink,
        map_server_node,
        delayed_lifecycle_map,    # t=1s
        delayed_jointodom,        # t=2s
        delayed_perception,       # t=3s
        delayed_amcl,             # t=5s
        # global_relocalization_node DIHAPUS
        # Daftarkan ke setup.py dulu sebelum ditambah kembali
    ])