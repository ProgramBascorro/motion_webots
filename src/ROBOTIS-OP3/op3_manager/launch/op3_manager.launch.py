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


def _usb_serial_ports():
    """Every USB serial tty the kernel actually has, adapters (ttyUSB*) first.

    Enumerated from sysfs rather than guessed: /sys/class/tty lists a name only
    once its driver is bound, and carries the real major:minor. Inside a container
    started with --device the node can be missing from /dev although the kernel has
    the tty, so recreate exactly that one -- we are root in a privileged container
    and device numbers are global. Nothing is ever created for a tty the kernel
    does not have, so no phantom nodes are left behind in the shared /dev.
    """
    ports = []
    for name in os.listdir('/sys/class/tty'):
        if not name.startswith(('ttyUSB', 'ttyACM')):
            continue
        try:
            with open('/sys/class/tty/%s/dev' % name) as handle:
                major, minor = (int(part) for part in handle.read().strip().split(':'))
        except (OSError, ValueError):
            continue
        dev = '/dev/%s' % name
        if not os.path.exists(dev):
            try:
                os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(major, minor))
            except OSError:
                continue
        ports.append(dev)
    # Adapters before CDC ports, then in number order (length first, so ttyUSB2
    # stays ahead of ttyUSB10).
    ports.sort(key=lambda dev: (not dev.startswith('/dev/ttyUSB'), len(dev), dev))
    return ports


def _openable(dev):
    """True if ``dev`` can be opened -- a node with no driver bound cannot."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    os.close(fd)
    return True


def ensure_ports(servo_link='/dev/ttyOP3', sub_link='/dev/ttyOP3'):
    """Resolve the port names by probing who actually answers.

    Mirrors ensure_opencr_port() in op3_action_editor's executor.py -- the manager
    needs the same treatment because it is usually the first thing started. A board
    that re-enumerates lands on a different minor, so the stable name has to follow
    the port that answers. Idempotent; quiet off-hardware.

    Stock opencr_op3 wiring here: sub controller ID 200 answers on the TTL bus
    alongside the servos, so both names are the same port and only one link is ever
    rewritten. The two arguments stay separate because a board flashed with
    opencr_op3_usb serves ID 200 on its micro-USB instead; the probe reports that
    mismatch rather than papering over it.

    A port nobody answered on is never adopted -- adopting one only turns a missing
    adapter into twenty "JOINT[...] does NOT respond!!" lines further downstream.
    """
    live = [dev for dev in _usb_serial_ports() if _openable(dev)]

    if not live:
        print('op3_manager.launch: no live serial port found', file=sys.stderr)
        return

    servo_port = next((d for d in live if _dxl_answers(d, FIRST_SERVO_ID)), None)
    sub_port = next((d for d in live if _dxl_answers(d, SUB_CONTROLLER_ID)), None)

    def point(link, target):
        try:
            if link == target or os.path.realpath(link) == target:
                return
            if os.path.exists(link) and not os.path.islink(link):
                print('op3_manager.launch: %s is a real device node, not a symlink; '
                      'leaving it alone (wanted %s)' % (link, target), file=sys.stderr)
                return
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(target, link)
            print('op3_manager.launch: %s -> %s' % (link, target), file=sys.stderr)
        except OSError as e:
            print('op3_manager.launch: could not set %s: %s' % (link, e), file=sys.stderr)

    if servo_port is None:
        print('op3_manager.launch: no servo answered on %s -- the DXL bus adapter '
              '(U2D2/FTDI, /dev/ttyUSB*) is not reachable, so %s is left as it is. '
              'Check the U2D2 cable and that the robot is powered; the OpenCR '
              'micro-USB alone cannot drive the servos.'
              % (', '.join(live), servo_link), file=sys.stderr)
    else:
        point(servo_link, servo_port)

    if sub_port is None:
        print('op3_manager.launch: ID %d silent on %s -- no IMU, no button'
              % (SUB_CONTROLLER_ID, ', '.join(live)), file=sys.stderr)
    elif sub_port == servo_port:
        pass  # stock wiring: one port carries both, nothing else to point
    elif servo_port is None:
        print('op3_manager.launch: ID %d answers on %s but no servo does -- the '
              'OpenCR is up, the servo bus is not.'
              % (SUB_CONTROLLER_ID, sub_port), file=sys.stderr)
    elif sub_link == servo_link:
        print('op3_manager.launch: ID %d answers on %s, not on the servo port %s -- '
              'this OpenCR serves the sub controller over its own port, so the '
              'OPEN-CR sensor line in OP3.robot needs that port, not %s'
              % (SUB_CONTROLLER_ID, sub_port, servo_port, servo_link), file=sys.stderr)
    else:
        point(sub_link, sub_port)


def generate_launch_description():
    ensure_ports()

    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    # Same name OP3.robot uses. Two parameters, one port: device_name is the servo
    # port (the startup torque check reads joint ID 1 through it) and ID 200 rides
    # the same TTL bus under stock opencr_op3, so the DXL power-on and RGB LED
    # writes go there too. They stay separate for a board flashed with
    # opencr_op3_usb, which serves ID 200 on its own micro-USB port instead.
    device_name_default = '/dev/ttyOP3'
    sub_controller_device_name_default = '/dev/ttyOP3'

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
                'device_name': device_name_default,
                'sub_controller_device_name': sub_controller_device_name_default
            }]
        )
        # Node(
        #     package='op3_localization',
        #     executable='op3_localization',
        #     name='op3_localization',
        #     output='screen'
        # )
    ])
