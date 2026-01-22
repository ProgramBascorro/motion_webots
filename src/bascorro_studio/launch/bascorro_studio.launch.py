from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    assets_port = LaunchConfiguration("assets_port")
    assets_dir = LaunchConfiguration("assets_dir")
    create_backup = LaunchConfiguration("create_backup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("assets_port", default_value="8001"),
            DeclareLaunchArgument(
                "assets_dir", default_value="/tmp/bascorro_studio_assets"
            ),
            DeclareLaunchArgument("create_backup", default_value="false"),
            Node(
                package="bascorro_studio",
                executable="asset_server",
                name="bascorro_studio_assets",
                arguments=["--port", assets_port, "--dir", assets_dir],
            ),
            Node(
                package="bascorro_studio",
                executable="apply_node",
                name="bascorro_studio_apply",
                parameters=[{"create_backup": create_backup}],
            ),
            Node(
                package="bascorro_studio",
                executable="studio_agent",
                name="bascorro_studio_agent",
            ),
        ]
    )
