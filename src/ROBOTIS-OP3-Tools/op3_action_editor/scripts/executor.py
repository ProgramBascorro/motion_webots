#!/usr/bin/env python3

import os
import shutil
import stat
import subprocess
import sys
import rclpy
import time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

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

def ensure_opencr_port(link_path: str = '/dev/ttyOP3', max_minor: int = 15) -> None:
    """Point ``link_path`` at whichever /dev/ttyUSB* the OpenCR is currently on.

    OP3.robot references a single stable port name (/dev/ttyOP3). But inside the
    docker container the serial node is a fixed --device from container-start time,
    so after the board re-enumerates (unplug/re-flash, or swapping between two OpenCR
    boards) it lands on a different ttyUSB minor and the old node/symlink goes stale
    -> "Error opening serial port". We run as root in the privileged container, so
    recreate the raw ttyUSB nodes and repoint the symlink at the port that actually
    opens. Idempotent; safe on every launch. Degrades quietly off-hardware.
    """
    TTY_MAJOR = 188
    for minor in range(max_minor + 1):
        dev = f'/dev/ttyUSB{minor}'
        if not os.path.exists(dev):
            try:
                os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(TTY_MAJOR, minor))
            except OSError:
                pass
    live = None
    for minor in range(max_minor + 1):
        dev = f'/dev/ttyUSB{minor}'
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            continue
        os.close(fd)
        live = dev
        break
    if live is None:
        print(f'ensure_opencr_port: no live /dev/ttyUSB* found; leaving {link_path} as-is',
              file=sys.stderr)
        return
    try:
        if os.path.realpath(link_path) == live:
            return
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(live, link_path)
        print(f'ensure_opencr_port: {link_path} -> {live}', file=sys.stderr)
    except OSError as e:
        print(f'ensure_opencr_port: could not set {link_path}: {e}', file=sys.stderr)


def resolve_action_file_default() -> str:
    """Default action file = the version-controlled workspace src copy that the op3
    stack and Bascorro Studio web read (via OP3_ACTION_FILE). Editing the same file
    keeps the editor and the running stack from drifting apart. Falls back to the
    installed share copy if the src tree is not available."""
    share_copy = get_package_share_directory('op3_action_module') + '/data/motion_4095_CHRONUS.bin'
    try:
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix('op3_action_module')))
        src_copy = os.path.join(ws_root, 'src/ROBOTIS-OP3/op3_action_module/data/motion_4095_CHRONUS.bin')
        if os.path.isfile(src_copy):
            return src_copy
    except Exception:
        pass
    return share_copy


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
    action_file_path_default = resolve_action_file_default()
    action_file_path = os.environ.get('OP3_ACTION_FILE', '').strip() or action_file_path_default
    # Both power-on (device_name) and the controller (OP3.robot) open this one
    # stable name; ensure_opencr_port() makes it track the board plugged in now.
    device_name_default = '/dev/ttyOP3'
    if not gazebo_default:
        ensure_opencr_port()

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
