"""Full 8-layer strategy stack — Phase 4+. Do NOT run soccer_brain alongside this."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('op3_soccer_core')
    rules_file = os.path.join(pkg_share, 'config', 'rc_hl_kidsize.yaml')

    team_number = LaunchConfiguration('team_number')
    player_number = LaunchConfiguration('player_number')
    is_goalkeeper = LaunchConfiguration('is_goalkeeper')

    return LaunchDescription([
        DeclareLaunchArgument('team_number', default_value='1'),
        DeclareLaunchArgument('player_number', default_value='2'),
        DeclareLaunchArgument('is_goalkeeper', default_value='false'),

        # Layer 1 — GC State Adapter
        Node(
            package='op3_soccer_core',
            executable='gc_state_adapter',
            output='screen',
            parameters=[{
                'team_number': team_number,
                'player_number': player_number,
                'is_goalkeeper': is_goalkeeper,
            }]
        ),

        # Layer 2 — Compliance
        Node(
            package='op3_soccer_core',
            executable='op3_compliance',
            output='screen',
            parameters=[{
                'team_number': team_number,
                'player_number': player_number,
                'rules_file': rules_file,
            }]
        ),

        # Layer 3 — Safety Monitor
        Node(
            package='op3_soccer_core',
            executable='op3_safety_monitor',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Layer 4 — Tactical
        Node(
            package='op3_soccer_core',
            executable='op3_tactical',
            output='screen',
            parameters=[{
                'team_number': team_number,
                'player_number': player_number,
                'is_goalkeeper': is_goalkeeper,
                'rules_file': rules_file,
            }]
        ),

        # Layer 5 — Team Comm + Coordinator
        Node(
            package='op3_soccer_core',
            executable='op3_team_comm',
            output='screen',
            parameters=[{'self_id': player_number}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_team_coordinator',
            output='screen',
            parameters=[{
                'team_number': team_number,
                'player_id': player_number,   # team_coordinator_node uses player_id, not player_number
                'rules_file': rules_file,
            }]
        ),

        # Layer 6 — Planner (A* + APF)
        Node(
            package='op3_soccer_core',
            executable='op3_planner',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Layer 7 — Perception + Localization + Ball Estimator
        Node(
            package='op3_soccer_core',
            executable='op3_perception',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_localization',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_ball_estimator',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Layer 8 — Motion (ONLY node that publishes to /robotis/...)
        Node(
            package='op3_soccer_core',
            executable='op3_motion',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Webots Walking Bridge — replaces op3_manager for Webots simulation.
        # Translates /robotis/* walking/action commands → direct joint position
        # commands consumed by op3_extern_controller.
        Node(
            package='op3_soccer_core',
            executable='op3_webots_walking_bridge',
            name='webots_walking_bridge',
            output='screen',
            # Set gait_flip_* to true if robot walks wrong direction after testing:
            #   gait_flip_hip:=true   → robot was walking backward
            #   gait_flip_knee:=true  → robot was falling forward/backward
            #   gait_flip_roll:=true  → robot was falling sideways
        ),
    ])
