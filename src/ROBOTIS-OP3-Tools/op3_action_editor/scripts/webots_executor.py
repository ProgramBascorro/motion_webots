#!/usr/bin/env python3

import subprocess
import rclpy
import time
import os
import shutil
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ActionEditorExecutor(Node):
    def __init__(self):
        super().__init__('op3_action_editor_executor')
        self.get_logger().info('Starting Action Editor with Webots simulation (FIXED VERSION)')

def main(args=None):
    rclpy.init(args=args)
    node = ActionEditorExecutor()

    WORLD_PATH = "/home/mesmer/Simulasi/motion_webots/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds/robotis_op3_extern.wbt"


    print("\n🚀 Starting Webots with custom world:")
    print("   ", WORLD_PATH)

    webots_proc = None
    try:
        webots_proc = subprocess.Popen(
            [
                'ros2', 'launch',
                'op3_webots_ros2', 'robot_launch.py',
                'world:=/home/mesmer/Simulasi/motion_webots/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds/robotis_op3_extern.wbt'
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        print("⏳ Waiting for Webots to initialize...")
        time.sleep(10)

        if webots_proc.poll() is not None:
            stdout, stderr = webots_proc.communicate()
            print("❌ Webots failed to start")
            print("STDOUT:\n", stdout.decode())
            print("STDERR:\n", stderr.decode())
            return 1

        print("✅ Webots started successfully")

    except Exception as e:
        print(f"❌ ERROR: Failed to start Webots: {e}")
        return 1

    print("\n🔧 Starting OpenCR simulator...")
    open_cr_proc = None
    try:
        open_cr_proc = subprocess.Popen(
            ['ros2', 'launch', 'open_cr_module', 'open_cr.launch.py', 'use_dummy_data:=true'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        print("✅ OpenCR simulator started")
    except Exception as e:
        print(f"⚠️  Failed to start OpenCR simulator: {e}")

    print("\n🌉 Starting Webots Bridge...")
    bridge_proc = None
    try:
        bridge_proc = subprocess.Popen(
            ['ros2', 'run', 'op3_action_editor', 'bridge_webots.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(3)
        print("✅ Webots Bridge started")
    except Exception as e:
        print(f"❌ Failed to start Webots Bridge: {e}")

    proc_player = None
    try:
        print("\n🔊 Starting audio player...")
        proc_player = subprocess.Popen(
            ['ros2', 'run', 'ros_mpg321_player', 'ros_mpg321_player'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print("✅ Audio player started")
    except Exception as e:
        print(f"⚠️  Audio player failed: {e}")

    print("\n🎯 Starting Action Editor...")

    package = "op3_action_editor"
    executable = "op3_action_editor"

    robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/motion_4095.bin'
    device_name_default = '/dev/null'

    params = [
        '--ros-args',
        '-p', 'gazebo:=true',   
        '-p', f'gazebo_robot_name:={robot_name_default}',
        '-p', f'offset_file_path:={offset_file_path_default}',
        '-p', f'robot_file_path:={robot_file_path_default}',
        '-p', f'init_file_path:={init_file_path_default}',
        '-p', f'action_file_path:={action_file_path_default}',
        '-p', f'device_name:={device_name_default}'
    ]

    proc_editor = None
    try:
        proc_editor = subprocess.Popen(
            ['ros2', 'run', package, executable] + params
        )
        print("✅ Action editor started")
    except Exception as e:
        print(f"❌ Error starting action editor: {e}")
        return 1

    print("\n🟢 SYSTEM READY")
    print("• Webots running with custom world")
    print("• OP3 uses <extern> controller")
    print("• Supervisor runs separately")
    print("• Camera should be ACTIVE again")
    print("Press Ctrl+C to exit")

    try:
        while True:
            if webots_proc and webots_proc.poll() is not None:
                print("❌ Webots has terminated")
                break
            if proc_editor and proc_editor.poll() is not None:
                print("❌ Action editor terminated")
                break
            if bridge_proc and bridge_proc.poll() is not None:
                print("❌ Bridge terminated")
                break
            time.sleep(2)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down by user request")

    finally:
        print("\n🧹 Cleaning up processes...")
        for proc, name in [
            (proc_editor, "Action Editor"),
            (bridge_proc, "Webots Bridge"),
            (open_cr_proc, "OpenCR Simulator"),
            (proc_player, "Audio Player"),
            (webots_proc, "Webots")
        ]:
            if proc and proc.poll() is None:
                print(f"  Terminating {name}...")
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()

    rclpy.shutdown()
    print("✅ All processes terminated cleanly.")
    return 0

if __name__ == '__main__':
    main()