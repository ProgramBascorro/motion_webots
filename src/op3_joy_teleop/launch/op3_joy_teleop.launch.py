import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("op3_joy_teleop")
    default_config = os.path.join(package_share, "config", "op3_joy_teleop.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to op3_joy_teleop YAML config",
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        "joy_deadzone",
        default_value="0.05",
        description="Deadzone for joy_node",
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        "joy_autorepeat_rate",
        default_value="30.0",
        description="Autorepeat rate for joy_node",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "deadzone": LaunchConfiguration("joy_deadzone"),
                "autorepeat_rate": LaunchConfiguration("joy_autorepeat_rate"),
            }
        ],
    )

    teleop_node = Node(
        package="op3_joy_teleop",
        executable="op3_joy_teleop",
        name="op3_joy_teleop",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription(
        [config_arg, joy_deadzone_arg, joy_autorepeat_arg, joy_node, teleop_node]
    )
