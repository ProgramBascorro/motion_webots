import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory

#from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    
    ld = LaunchDescription()

    package_dir = get_package_share_directory('op3_webots_ros2')
    gain_file_path_default = package_dir + '/resource/op3_pid_gain_default.yaml'

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
