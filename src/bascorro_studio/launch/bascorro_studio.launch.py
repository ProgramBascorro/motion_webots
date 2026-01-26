from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    assets_port = LaunchConfiguration("assets_port")
    assets_dir = LaunchConfiguration("assets_dir")
    create_backup = LaunchConfiguration("create_backup")
    terminal_port = LaunchConfiguration("terminal_port")
    terminal_credential = LaunchConfiguration("terminal_credential")

    return LaunchDescription(
        [
            DeclareLaunchArgument("assets_port", default_value="8001"),
            DeclareLaunchArgument(
                "assets_dir", default_value="/tmp/bascorro_studio_assets"
            ),
            DeclareLaunchArgument("create_backup", default_value="false"),
            DeclareLaunchArgument("terminal_port", default_value="7681"),
            DeclareLaunchArgument("terminal_credential", default_value=""),
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
            Node(
                package="bascorro_studio",
                executable="terminal_server",
                name="bascorro_studio_terminal",
                parameters=[
                    {"port": terminal_port},
                    {"credential": terminal_credential},
                ],
            ),
        ]
    )
