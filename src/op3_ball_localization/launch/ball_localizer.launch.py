import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("op3_ball_localization")
    default_config = os.path.join(package_share, "config", "ball_localizer.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to op3_ball_localization YAML config",
    )

    node = Node(
        package="op3_ball_localization",
        executable="ball_localizer",
        name="op3_ball_localizer",
        parameters=[LaunchConfiguration("config")],
        output="screen",
    )

    return LaunchDescription([config_arg, node])
