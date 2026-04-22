#!/usr/bin/env python3
"""
amcl_jointodom.launch.py v3
Fix konvergensi lambat AMCL dengan joint odom.

Strategi:
1. max_particles dinaikkan sementara (500→2000) saat startup untuk convergence cepat
   lalu AMCL sendiri akan kurangi ke min setelah konvergen (adaptive particle filter)
2. recovery_alpha_fast dinaikkan — agar AMCL lebih agresif recovery saat lost
3. laser_z_hit dinaikkan, laser_z_rand diturunkan — lebih percaya scan match
4. laser_sigma_hit dikecilkan — scan match lebih ketat (presisi lebih tinggi)
5. update_min_d/a dikecilkan — update lebih sering saat awal (bantu konvergensi)
6. resample_interval: 1 saat awal agar cepat konvergen
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
    # t = 2s: Joint Odom
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
            'pose_cov_xy':     0.1,    # lebih ketat → AMCL lebih percaya odom
            'pose_cov_yaw':    0.05,   # yaw dari IMU cukup akurat
            'smoothing_alpha': 0.5,
            'step_scale':      0.8,
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
    # t = 5s: AMCL — dioptimalkan untuk konvergensi awal
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

            # ── Particles ──────────────────────────────────
            # min besar → banyak partikel tersebar di awal → cepat temukan posisi
            # max cukup untuk coverage seluruh lapangan
            # AMCL adaptive: otomatis kurangi partikel setelah konvergen
            'min_particles': 500,
            'max_particles': 2000,

            # recovery agresif → kalau lost, langsung scatter ulang
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.2,    # dinaikkan dari 0.1

            # ── Motion model ───────────────────────────────
            # alpha sedikit lebih besar → partikel lebih menyebar
            # mengkompensasi ketidakpastian joint odom
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.1,
            'alpha2': 0.1,
            'alpha3': 0.05,
            'alpha4': 0.05,
            'alpha5': 0.1,

            # ── Update threshold ───────────────────────────
            # lebih kecil = update lebih sering = konvergensi lebih cepat
            # tapi lebih berat — ok untuk fase awal
            'update_min_d':      0.02,   # 2cm (lebih sering dari sebelumnya)
            'update_min_a':      0.02,   # ~1.1°
            'resample_interval': 1,      # resample setiap update untuk konvergensi cepat

            # ── Laser model ────────────────────────────────
            # z_hit tinggi + z_rand rendah = lebih percaya scan match
            # sigma_hit kecil = matching lebih ketat/presisi
            'laser_model_type':          'likelihood_field',
            'laser_likelihood_max_dist':  0.5,
            'laser_max_range':            5.0,
            'laser_min_range':            0.3,
            'laser_max_beams':            36,
            'laser_z_hit':                0.7,    # naik dari 0.5
            'laser_z_rand':               0.3,    # turun dari 0.5
            'laser_sigma_hit':            0.15,   # turun dari 0.2 (lebih presisi)

            # ── Initial pose ───────────────────────────────
            'set_initial_pose':           True,
            'initial_pose.x':             0.0,
            'initial_pose.y':             0.0,
            'initial_pose.z':             0.0,
            'initial_pose.yaw':           0.0,

            # ── Transform ──────────────────────────────────
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

    delayed_jointodom = TimerAction(period=2.0, actions=[joint_odom_node])

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
        static_odom_baselink,
        map_server_node,
        lifecycle_manager_map,
        delayed_jointodom,
        delayed_perception,
        delayed_amcl,
    ])