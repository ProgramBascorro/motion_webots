#!/usr/bin/env python3
"""
op3_full_bringup.launch.py
Master launch file untuk menjalankan seluruh sistem OP3:

Urutan startup:
1. t=0s   → Webots simulator (op3_webots_ros2 robot_launch.py)
2. t=5s   → op3_manager (op3_simulation.launch.py)  ← delay 5 detik
3. t=5s   → op3_gui_demo (op3_demo.launch.py)        ← delay 5 detik

Alasan delay:
- Webots butuh waktu untuk load world & URDF robot
- op3_manager perlu koneksi ke controller yang sudah siap
- Tanpa delay, op3_manager gagal konek dan crash

Letakkan file ini di:
~/basbot/src/op3_bringup/launch/op3_full_bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────
    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='5.0',
        description='Delay (detik) sebelum op3_manager & gui_demo dijalankan, '
                    'memberi waktu Webots untuk load world'
    )

    startup_delay = LaunchConfiguration('startup_delay')

    # ── Package directories ───────────────────────────────────
    pkg_webots   = get_package_share_directory('op3_webots_ros2')
    pkg_manager  = get_package_share_directory('op3_manager')
    pkg_gui_demo = get_package_share_directory('op3_gui_demo')

    # ── 1. Webots (langsung, t=0) ─────────────────────────────
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'robot_launch.py')
        )
    )

    log_webots_start = LogInfo(msg='[op3_bringup] ▶ Memulai Webots simulator...')

    # ── 2. op3_manager (setelah delay) ───────────────────────
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manager, 'launch', 'op3_simulation.launch.py')
        )
    )

    log_manager_start = LogInfo(
        msg='[op3_bringup] ▶ Memulai op3_manager (setelah delay)...'
    )

    # ── 3. op3_gui_demo (setelah delay, bersamaan dengan manager) ──
    gui_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gui_demo, 'launch', 'op3_demo.launch.py')
        )
    )

    log_gui_start = LogInfo(
        msg='[op3_bringup] ▶ Memulai op3_gui_demo (setelah delay)...'
    )

    # ── Bundel node yang di-delay ────────────────────────────
    delayed_nodes = TimerAction(
        period=startup_delay,
        actions=[
            log_manager_start,
            manager_launch,
            log_gui_start,
            gui_demo_launch,
        ]
    )

    return LaunchDescription([
        startup_delay_arg,
        log_webots_start,
        webots_launch,       # t = 0s
        delayed_nodes,       # t = 5s (default)
    ])
