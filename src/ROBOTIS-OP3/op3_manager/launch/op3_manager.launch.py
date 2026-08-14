import os
import re
import tempfile
import atexit
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _can_open_serial(dev):
    """True if the serial device can be opened right now (exists and not busy)."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        return True
    except OSError:
        return False


def detect_dynamixel_device(preferred='/dev/ttyUSB0'):
    """Find the OpenCR/Dynamixel serial port regardless of its ttyUSB number.

    The adapter enumerates as /dev/ttyUSB0../dev/ttyUSB9 depending on plug order,
    and the udev symlink /dev/ttyOP3 is not recreated inside Docker. So scan the
    ttyUSB0-9 range and pick a usable port. Set OP3_DEVICE to force a specific one."""
    override = os.environ.get('OP3_DEVICE', '').strip()
    if override:
        return override
    candidates = [f'/dev/ttyUSB{i}' for i in range(10) if os.path.exists(f'/dev/ttyUSB{i}')]
    for dev in candidates:
        if _can_open_serial(dev):
            return dev
    if os.path.exists('/dev/ttyOP3'):
        return '/dev/ttyOP3'
    return candidates[0] if candidates else preferred


def robot_file_with_device(src_robot_path, device):
    """Return a robot-config path whose ports match `device`.

    RobotisController opens the ports listed in OP3.robot, so those must point at the
    detected device too. Write a patched copy to a temp path instead of mutating the
    version-controlled file (falls back to the original on error)."""
    try:
        with open(src_robot_path) as f:
            content = f.read()
        patched = re.sub(r'/dev/tty[A-Za-z0-9_]+', device, content)
        if patched == content:
            return src_robot_path
        fd, tmp_path = tempfile.mkstemp(prefix='OP3_', suffix='.robot')
        with os.fdopen(fd, 'w') as f:
            f.write(patched)
        atexit.register(lambda: os.path.exists(tmp_path) and os.remove(tmp_path))
        return tmp_path
    except OSError:
        return src_robot_path


def generate_launch_description():
    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'

    # Auto-detect the OpenCR/Dynamixel port (ttyUSB0-9) so the demo starts whatever
    # number the adapter enumerated as, then patch the robot config to that port.
    # (device_name is used for power-on; OP3.robot ports are opened by the controller.)
    device_name_default = detect_dynamixel_device()
    robot_file_path_default = robot_file_with_device(robot_file_path_default, device_name_default)
    print(f"[op3_manager.launch] using Dynamixel device: {device_name_default}")

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
