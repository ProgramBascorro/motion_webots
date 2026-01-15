import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory

#from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    
    ld = LaunchDescription()

    package_dir = get_package_share_directory('op3_webots_ros2')
    gain_file_path_default = package_dir + '/resource/op3_pid_gain_default.yaml'

    # Load robot description (URDF)
    op3_description_path = FindPackageShare('op3_description')
    op3_urdf_path = PathJoinSubstitution([op3_description_path, 'urdf', 'robotis_op3.urdf.xacro'])
    op3_description_content = ParameterValue(Command(['xacro ', op3_urdf_path]), value_type=str)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'robotis_op3_extern.wbt')
    )

    ld.add_action(webots)
    ld.add_action(Node(
        package='op3_webots_ros2',
        executable='op3_extern_controller',
        output='screen',
        parameters=[{'gain_file_path': gain_file_path_default}]
    ))

    # robot_state_publisher for TF tree (base -> ... -> cam_link)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': op3_description_content}],
        remappings=[('/joint_states', '/robotis_op3/joint_states')],
    ))

    return ld