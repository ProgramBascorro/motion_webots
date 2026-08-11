#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
import rclpy
import time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ActionEditorExecutor(Node):
    def __init__(self):
        super().__init__('op3_action_editor_executor')


def has_ros_package(package_name: str) -> bool:
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', package_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def ensure_action_file(action_file_path: str, default_path: str) -> None:
    if os.path.isfile(action_file_path):
        return
    if not os.path.isfile(default_path):
        print(f"Default action file not found: {default_path}", file=sys.stderr)
        return
    os.makedirs(os.path.dirname(action_file_path), exist_ok=True)
    shutil.copyfile(default_path, action_file_path)

def main(args=None):
    rclpy.init(args=args)
    node = ActionEditorExecutor()

    # Define your package and executable
    package = "op3_action_editor"
    executable = "op3_action_editor"

    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    share_data_path = get_package_share_directory('op3_action_module') + '/data/motion_4095_ros1_lama.bin'
    # Resolve symlink so we read/write directly to the workspace source file.
    # When --symlink-install is used, this resolves to src/.../motion_4095_ros1_lama.bin,
    # so edits survive rebuilds.
    action_file_path_default = os.path.realpath(share_data_path)
    action_file_path = os.environ.get('OP3_ACTION_FILE', '').strip() or action_file_path_default
    # device_name is NOT the servo bus -- main.cpp opens it only to write the
    # power-on register of the sub controller (ID 200, the OpenCR). This board's
    # OpenCR is deaf on the TTL bus, so ID 200 answers over its own USB CDC port
    # only; pointing this at the U2D2 gives "Error Set port" (or "Failed to turn
    # on the Power of DXLs!" when the U2D2 happens to be plugged in). The raw
    # /dev/ttyUSB0 path was doubly wrong: the FTDI keeps re-enumerating between
    # ttyUSB0 and ttyUSB1, so only the by-id path is stable. Servos still come
    # from the port lines in OP3.robot, untouched by this.
    device_name_default = os.environ.get('OP3_SUB_CONTROLLER_DEVICE', '').strip() \
        or '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'

    if action_file_path != action_file_path_default:
        ensure_action_file(action_file_path, action_file_path_default)
    if not os.path.isfile(action_file_path):
        print(f"Action file not found: {action_file_path}", file=sys.stderr)
        return 1

    # Define any parameters or arguments
    params = [
        '--ros-args',
        '-p', f'gazebo:={gazebo_default}',
        '-p', f'gazebo_robot_name:={gazebo_robot_name_default}',
        '-p', f'offset_file_path:={offset_file_path_default}',
        '-p', f'robot_file_path:={robot_file_path_default}',
        '-p', f'init_file_path:={init_file_path_default}',
        '-p', f'action_file_path:={action_file_path}',
        '-p', f'device_name:={device_name_default}'
    ]

    proc_player = None
    if has_ros_package('ros_mpg321_player'):
        try:
            proc_player = subprocess.Popen(
                ['ros2', 'run', 'ros_mpg321_player', 'ros_mpg321_player'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            print(f"Failed to run ros_mpg321_player: {e}", file=sys.stderr)
    else:
        print("ros_mpg321_player is not installed; starting without sound support.", file=sys.stderr)

    try:
        # Run the node in the same terminal
        proc_editor = subprocess.Popen(['ros2', 'run', package, executable] + params)
    except subprocess.CalledProcessError as e:
        print(f"Error while running op3_action_editor: {e}")
        if proc_player is not None and proc_player.poll() is None:
            proc_player.kill()
        return 1

    while True:
        if proc_editor.poll() is not None:
            break

        time.sleep(1)

    if proc_player is not None and proc_player.poll() is None:
        proc_player.kill()

    if proc_editor.poll() is None:
        proc_editor.kill()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
