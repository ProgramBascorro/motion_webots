import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory

#from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    ld = LaunchDescription()

    package_dir = get_package_share_directory('op3_webots_ros2')
    gain_file_path_default = package_dir + '/resource/op3_pid_gain_default.yaml'
    op3_description_path = FindPackageShare('op3_description')
    op3_urdf_path = PathJoinSubstitution([op3_description_path, 'urdf', 'robotis_op3.urdf.xacro'])
    op3_description_content = ParameterValue(Command(['xacro ', op3_urdf_path]), value_type=str)
    enable_rsp = LaunchConfiguration('enable_robot_state_publisher')

    ld.add_action(DeclareLaunchArgument(
        'enable_robot_state_publisher',
        default_value='true',
        description='Enable robot_state_publisher for OP3 TF tree.'
    ))

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
    ld.add_action(Node(
        package='op3_webots_ros2',
        executable='op3_gc_bridge',
        output='screen',
        parameters=[{
            'gc_host': '127.0.0.1',
            'control_port': 3838,
            'status_port': 3939,
            'team_number': 1,
            'player_number': 1,
            'rx_period_ms': 20,
            'tx_period_ms': 500
        }]
    ))
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': op3_description_content}],
        remappings=[('/joint_states', '/robotis_op3/joint_states')],
        condition=IfCondition(enable_rsp)
    ))

    return ld
    # return LaunchDegiscription([
    #     webots,
    #     my_robot_driver,
    #     launch.actions.RegisterEventHandler(
    #         event_handler=launch.event_handlers.OnProcessExit(
    #             target_action=webots,
    #             on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
    #         )
    #     )
    # ])
