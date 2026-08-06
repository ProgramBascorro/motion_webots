#!/usr/bin/env python3

import os
import shutil
import stat
import subprocess
import sys
import termios
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

SUB_CONTROLLER_ID = 200
FIRST_SERVO_ID = 1


def dxl_ping(dev: str, dxl_id: int, baud_flag: int = termios.B2000000) -> bool:
    """True if ``dxl_id`` answers a protocol 2.0 ping on ``dev``.

    Pure stdlib on purpose: this runs before the workspace is fully up and the
    container has no pyserial. Baud is nominal on a CDC port (the OpenCR bridge
    ignores it) and mandatory on an FTDI one, so 2 Mbps works for both.
    """
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 3, 0, 0x01])
    crc = 0
    for byte in body:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    packet = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = baud_flag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        os.write(fd, packet)
        reply = b''
        deadline = time.time() + 0.3
        while time.time() < deadline and len(reply) < 14:
            try:
                reply += os.read(fd, 64)
            except BlockingIOError:
                time.sleep(0.002)
        return len(reply) >= 14
    except (OSError, termios.error):
        return False
    finally:
        os.close(fd)


def _point_link(link_path: str, target: str) -> None:
    """Repoint ``link_path`` at ``target``, quietly and idempotently."""
    try:
        if os.path.realpath(link_path) == target:
            return
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(target, link_path)
        print(f'ensure_opencr_port: {link_path} -> {target}', file=sys.stderr)
    except OSError as e:
        print(f'ensure_opencr_port: could not set {link_path}: {e}', file=sys.stderr)


def ensure_opencr_port(servo_link: str = '/dev/ttyOP3',
                       sub_link: str = '/dev/ttyOpenCR',
                       max_minor: int = 15) -> None:
    """Point the two stable port names at whatever is actually plugged in now.

    OP3.robot references stable names, but inside the docker container the serial
    node is a fixed --device from container-start time, so after a board
    re-enumerates (unplug/re-flash, or swapping boards) it lands on a different
    minor and the old node/symlink goes stale -> "Error opening serial port". We run
    as root in the privileged container, so recreate the raw device nodes and
    repoint the symlinks at the ports that actually answer. Idempotent; safe on
    every launch; degrades quietly off-hardware.

    Two names because on CHRONUS the buses are split. Stock opencr_op3 serves the
    sub controller (ID 200 -- IMU, button, buzzer, DXL power) on DXL_PORT = Serial3,
    i.e. the TTL bus alongside the servos. This robot's OpenCR cannot: its TTL
    transceiver is deaf, so it runs opencr_op3_usb and serves ID 200 over its
    micro-USB. Hence servo_link = the TTL adapter, sub_link = the OpenCR CDC.

    Pick by probing, not by liveness -- who answers decides, so the same code keeps
    working if a healthy OpenCR is ever put back on the TTL bus. In that case both
    roles land on one port, and sub_link is deliberately left alone: pointing two
    PortHandlers at one tty would put two fds on it. OP3.robot must move the sensor
    back onto ttyOP3 for that wiring.
    """
    candidates = []
    for major, prefix in ((188, 'ttyUSB'), (166, 'ttyACM')):
        for minor in range(max_minor + 1):
            dev = f'/dev/{prefix}{minor}'
            if not os.path.exists(dev):
                try:
                    os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(major, minor))
                except OSError:
                    pass
            candidates.append(dev)

    live = []
    for dev in candidates:
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            continue
        os.close(fd)
        live.append(dev)

    if not live:
        print(f'ensure_opencr_port: no live serial port found; leaving {servo_link} '
              f'and {sub_link} as-is', file=sys.stderr)
        return

    servo_port = None
    sub_port = None
    for dev in live:
        if servo_port is None and dxl_ping(dev, FIRST_SERVO_ID):
            servo_port = dev
        if sub_port is None and dxl_ping(dev, SUB_CONTROLLER_ID):
            sub_port = dev
        if servo_port is not None and sub_port is not None:
            break

    if servo_port is None:
        servo_port = live[0]
        print(f'ensure_opencr_port: no servo answered on {", ".join(live)}; '
              f'falling back to {servo_port}', file=sys.stderr)
    _point_link(servo_link, servo_port)

    if sub_port is None:
        print(f'ensure_opencr_port: OpenCR not answering (ID {SUB_CONTROLLER_ID} '
              f'silent on {", ".join(live)}) -- no IMU, no button. Check it is '
              f'powered and flashed with opencr_op3_usb (ID 200 over micro-USB).',
              file=sys.stderr)
    elif sub_port == servo_port:
        print(f'ensure_opencr_port: ID {SUB_CONTROLLER_ID} answers on {sub_port}, the '
              f'same port as the servos -- stock opencr_op3 wiring. Leaving {sub_link} '
              f'alone; move the OPEN-CR sensor line in OP3.robot back onto {servo_link}.',
              file=sys.stderr)
    else:
        print(f'ensure_opencr_port: OpenCR found on {sub_port} '
              f'(ID {SUB_CONTROLLER_ID}) -- IMU available', file=sys.stderr)
        _point_link(sub_link, sub_port)


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
    # device_name is only the port used to power on sub controller ID 200, which
    # lives on the OpenCR's own port here; the servos come from OP3.robot. Both
    # names are resolved by ensure_opencr_port() just below.
    device_name_default = '/dev/ttyOpenCR'
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
