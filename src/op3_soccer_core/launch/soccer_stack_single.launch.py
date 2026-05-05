"""Single-robot full strategy stack — Layers 1-8 (Phases 1-7)."""
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
            package='op3_soccer_core', executable='gc_state_adapter',
            name='gc_state_adapter', output='screen',
            parameters=[{
                'team_number': team_number,
                'player_number': player_number,
                'is_goalkeeper': is_goalkeeper,
            }]
        ),

        # Layer 2 — Compliance
        Node(
            package='op3_soccer_core', executable='op3_compliance',
            name='compliance_node', output='screen',
            parameters=[{
                'rules_file': rules_file,
                'team_number': team_number,
                'player_number': player_number,
            }]
        ),

        # Layer 3 — Safety Monitor
        Node(
            package='op3_soccer_core', executable='op3_safety_monitor',
            name='safety_monitor_node', output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Layer 4 — Tactical FSM
        Node(
            package='op3_soccer_core', executable='op3_tactical',
            name='tactical_node', output='screen',
            parameters=[{
                'rules_file': rules_file,
                'player_number': player_number,
                'team_number': team_number,
                'is_goalkeeper': is_goalkeeper,
            }]
        ),

        # Layer 5 — Team Comm + Coordinator
        Node(
            package='op3_soccer_core', executable='op3_team_comm',
            name='team_comm_node', output='screen',
            parameters=[{'self_id': player_number}]
        ),
        Node(
            package='op3_soccer_core', executable='op3_team_coordinator',
            name='team_coordinator_node', output='screen',
            parameters=[{
                'player_id': player_number,
                'team_number': team_number,
            }]
        ),

        # Layer 6 — Planner
        Node(
            package='op3_soccer_core', executable='op3_planner',
            name='planner_node', output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # Layer 7 — Perception + Localization + Ball Estimator
        Node(
            package='op3_soccer_core', executable='op3_ball_estimator',
            name='ball_estimator_node', output='screen',
            parameters=[{
                'camera_focal_length_px': 1109.0,
                'camera_fov_deg': 60.0,
            }]
        ),
        Node(
            package='op3_soccer_core', executable='op3_perception',
            name='perception_node', output='screen',
            parameters=[{
                'camera_focal_length_px': 1109.0,
                'camera_fov_deg': 60.0,
            }]
        ),
        Node(
            package='op3_soccer_core', executable='op3_localization',
            name='localization_node', output='screen',
        ),

        # Layer 8 — Motion (sole publisher to /robotis/...)
        Node(
            package='op3_soccer_core', executable='op3_motion',
            name='motion_node', output='screen',
            parameters=[{'rules_file': rules_file}]
        ),
    ])
