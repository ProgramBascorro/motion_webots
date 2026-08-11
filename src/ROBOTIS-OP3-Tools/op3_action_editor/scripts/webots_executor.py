#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory


def ensure_action_file(action_file_path: str, default_path: str) -> None:
    if os.path.isfile(action_file_path):
        return
    if not os.path.isfile(default_path):
        print(f"Default action file not found: {default_path}", file=sys.stderr)
        return
    os.makedirs(os.path.dirname(action_file_path), exist_ok=True)
    shutil.copyfile(default_path, action_file_path)


def main() -> int:
    package = "op3_action_editor"
    executable = "op3_action_editor"

    gazebo_default = True
    gazebo_robot_name_default = "robotis_op3"

    offset_file_path_default = get_package_share_directory("op3_manager") + "/config/offset.yaml"
    robot_file_path_default = get_package_share_directory("op3_manager") + "/config/OP3.robot"
    init_file_path_default = get_package_share_directory("op3_manager") + "/config/dxl_init_OP3.yaml"
    default_action_file_path = (
        get_package_share_directory("op3_action_module") + "/data/motion_4095_ros1_lama.bin"
    )
    action_file_path = os.environ.get("OP3_ACTION_FILE", "").strip() or default_action_file_path
    device_name_default = "/dev/null"

    if action_file_path != default_action_file_path:
        ensure_action_file(action_file_path, default_action_file_path)
    if not os.path.isfile(action_file_path):
        print(f"Action file not found: {action_file_path}", file=sys.stderr)
        return 1

    params = [
        "--ros-args",
        "-p",
        f"gazebo:={gazebo_default}",
        "-p",
        f"gazebo_robot_name:={gazebo_robot_name_default}",
        "-p",
        f"offset_file_path:={offset_file_path_default}",
        "-p",
        f"robot_file_path:={robot_file_path_default}",
        "-p",
        f"init_file_path:={init_file_path_default}",
        "-p",
        f"action_file_path:={action_file_path}",
        "-p",
        f"device_name:={device_name_default}",
    ]

    cmd = ["ros2", "run", package, executable] + params
    print("Starting op3_action_editor (Webots connect-only).")
    print(f"Action file: {action_file_path}")
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
