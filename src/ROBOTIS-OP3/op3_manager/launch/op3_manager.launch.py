import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
