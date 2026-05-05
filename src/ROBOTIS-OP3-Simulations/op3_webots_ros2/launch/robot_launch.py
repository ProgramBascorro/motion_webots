import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    package_dir = get_package_share_directory('op3_webots_ros2')
    world_file = os.path.join(package_dir, 'worlds', 'robotis_op3_extern.wbt')
    gain_file_path_default = package_dir + '/resource/op3_pid_gain_default.yaml'
    op3_description_path = FindPackageShare('op3_description')
    op3_urdf_path = PathJoinSubstitution([op3_description_path, 'urdf', 'robotis_op3.urdf.xacro'])
    op3_description_content = ParameterValue(Command(['xacro ', op3_urdf_path]), value_type=str)
    enable_rsp = LaunchConfiguration('enable_robot_state_publisher')
    controller_delay = LaunchConfiguration('controller_startup_delay')

    ld.add_action(DeclareLaunchArgument(
        'enable_robot_state_publisher',
        default_value='true',
        description='Enable robot_state_publisher for OP3 TF tree.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'controller_startup_delay',
        default_value='35.0',
        description='Seconds to wait after Webots starts before connecting op3_extern_controller. '
                    'Webots IPC socket takes ~15s to appear; 35s gives a safe margin. '
                    'Reduce to 15.0 if your machine is fast.'
    ))

    # Use ExecuteProcess instead of WebotsLauncher to avoid WSL2 detection that
    # tries to launch webots.exe (Windows binary) when running on WSL2 Linux.
    # /usr/local/bin/webots is the native Linux Webots R2025a binary.
    ld.add_action(ExecuteProcess(
        cmd=['/usr/local/bin/webots', '--batch', '--mode=realtime', world_file],
        output='screen',
        name='webots',
    ))

    # op3_extern_controller calls wb_robot_init() which has a ~50s retry window.
    # Delay its start so Webots has time to create the robot IPC socket (~15s).
    ld.add_action(TimerAction(
        period=controller_delay,
        actions=[
            Node(
                package='op3_webots_ros2',
                executable='op3_extern_controller',
                output='screen',
                parameters=[{'gain_file_path': gain_file_path_default}]
            ),
        ]
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
