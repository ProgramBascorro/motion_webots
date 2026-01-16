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
                "assets_dir", default_value="/tmp/op3_action_web_assets"
            ),
            DeclareLaunchArgument("create_backup", default_value="false"),
            Node(
                package="op3_action_web",
                executable="asset_server",
                name="op3_action_web_assets",
                arguments=["--port", assets_port, "--dir", assets_dir],
            ),
            Node(
                package="op3_action_web",
                executable="apply_node",
                name="op3_action_web_apply",
                parameters=[{"create_backup": create_backup}],
            ),
        ]
    )
