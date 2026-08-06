import os
import stat
import struct
import sys
import termios
import time

import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

SUB_CONTROLLER_ID = 200
FIRST_SERVO_ID = 1


def _dxl_answers(dev, dxl_id):
    """True if ``dxl_id`` replies to a protocol 2.0 ping on ``dev``."""
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id]) + struct.pack('<H', 3) + bytes([0x01])
    crc = 0
    for byte in body:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    packet = body + struct.pack('<H', crc)
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B2000000
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


def ensure_ports(servo_link='/dev/ttyOP3', sub_link='/dev/ttyOpenCR', max_minor=15):
    """Resolve both stable port names by probing who actually answers.

    Mirrors ensure_opencr_port() in op3_action_editor's executor.py -- the manager
    needs the same treatment because it is usually the first thing started, and
    /dev/ttyOpenCR has no udev rule installed on this host. Inside the container the
    serial nodes are fixed --device entries from container-start time, so recreate
    them and repoint the links at whatever is live now. Idempotent; quiet off-hardware.

    The servos are on the U2D2 and the sub controller ID 200 on the OpenCR's CDC,
    because this board's TTL transceiver is dead (see OP3.robot). If a healthy
    OpenCR is ever put back on the TTL bus, both answer on one port; sub_link is
    then left alone rather than putting two PortHandlers on one tty.
    """
    candidates = []
    for major, prefix in ((188, 'ttyUSB'), (166, 'ttyACM')):
        for minor in range(max_minor + 1):
            dev = '/dev/%s%d' % (prefix, minor)
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
        print('op3_manager.launch: no live serial port found', file=sys.stderr)
        return

    servo_port = next((d for d in live if _dxl_answers(d, FIRST_SERVO_ID)), None)
    sub_port = next((d for d in live if _dxl_answers(d, SUB_CONTROLLER_ID)), None)

    def point(link, target):
        try:
            if os.path.realpath(link) == target:
                return
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(target, link)
            print('op3_manager.launch: %s -> %s' % (link, target), file=sys.stderr)
        except OSError as e:
            print('op3_manager.launch: could not set %s: %s' % (link, e), file=sys.stderr)

    point(servo_link, servo_port or live[0])

    if sub_port is None:
        print('op3_manager.launch: ID %d silent on %s -- no IMU, no button'
              % (SUB_CONTROLLER_ID, ', '.join(live)), file=sys.stderr)
    elif sub_port == servo_port:
        print('op3_manager.launch: ID %d shares the servo port (%s) -- stock '
              'opencr_op3 wiring; move the OPEN-CR sensor line in OP3.robot back '
              'onto %s' % (SUB_CONTROLLER_ID, sub_port, servo_link), file=sys.stderr)
    else:
        point(sub_link, sub_port)


def generate_launch_description():
    ensure_ports()

    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    # Same stable name OP3.robot uses. This is the port op3_manager writes the DXL
    # power and RGB LED through (sub controller ID 200), so it must follow the
    # OpenCR rather than being pinned to a ttyUSB minor.
    device_name_default = '/dev/ttyOP3'

    return LaunchDescription([
        Node(
            package='op3_manager',
            executable='op3_manager',
            # name='op3_manager',
            output='screen',
            parameters=[{
                'angle_unit': 30.0,
                'gazebo': gazebo_default,
                'gazebo_robot_name': gazebo_robot_name_default,
                'offset_file_path': offset_file_path_default,
                'robot_file_path': robot_file_path_default,
                'init_file_path': init_file_path_default,
                'device_name': device_name_default
            }]
        )
        # Node(
        #     package='op3_localization',
        #     executable='op3_localization',
        #     name='op3_localization',
        #     output='screen'
        # )
    ])
