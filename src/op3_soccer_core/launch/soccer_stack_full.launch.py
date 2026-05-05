"""Full-stack launch: Webots + YOLO vision + soccer strategy + FSM designer UI.

Usage:
  ros2 launch op3_soccer_core soccer_stack_full.launch.py
  ros2 launch op3_soccer_core soccer_stack_full.launch.py player_number:=2 team_number:=1

Optional flags:
  use_webots:=true       Launch Webots simulator (default: true)
  use_yolo:=true         Launch YOLO vision pipeline (default: true)
  use_fsm_designer:=true Launch web UI at http://localhost:8080 (default: false)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    soccer_core_share = get_package_share_directory('op3_soccer_core')
    yolo_share = get_package_share_directory('op3_yolo_vision')
    rules_file = os.path.join(soccer_core_share, 'config', 'rc_hl_kidsize.yaml')
    yolo_params_file = os.path.join(yolo_share, 'config', 'yolo.yaml')

    team_number = LaunchConfiguration('team_number')
    player_number = LaunchConfiguration('player_number')
    is_goalkeeper = LaunchConfiguration('is_goalkeeper')
    use_webots = LaunchConfiguration('use_webots')
    use_yolo = LaunchConfiguration('use_yolo')
    use_fsm_designer = LaunchConfiguration('use_fsm_designer')
    gait_flip_hip   = LaunchConfiguration('gait_flip_hip')
    gait_flip_knee  = LaunchConfiguration('gait_flip_knee')
    gait_flip_roll  = LaunchConfiguration('gait_flip_roll')

    return LaunchDescription([
        # === Launch arguments ===
        DeclareLaunchArgument('team_number', default_value='1',
                              description='Team number (1 or 2)'),
        DeclareLaunchArgument('player_number', default_value='2',
                              description='Player number (1=GK, 2=Striker, 3=Support, 4=Defender)'),
        DeclareLaunchArgument('is_goalkeeper', default_value='false'),
        DeclareLaunchArgument('use_webots', default_value='true',
                              description='Launch Webots simulator'),
        DeclareLaunchArgument('use_yolo', default_value='true',
                              description='Launch YOLO vision pipeline'),
        DeclareLaunchArgument('use_fsm_designer', default_value='false',
                              description='Launch FSM Designer web UI at :8080'),
        DeclareLaunchArgument('gait_flip_hip',  default_value='false',
                              description='Flip hip-pitch direction (use if robot walks backward)'),
        DeclareLaunchArgument('gait_flip_knee', default_value='false',
                              description='Flip knee direction (use if robot falls fwd/bwd)'),
        DeclareLaunchArgument('gait_flip_roll', default_value='false',
                              description='Flip hip-roll direction (use if robot falls sideways)'),

        # === Webots simulation ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('op3_webots_ros2'), 'launch', 'robot_launch.py'
                ])
            ]),
            condition=IfCondition(use_webots),
        ),

        # === YOLO vision pipeline ===
        Node(
            package='op3_yolo_vision',
            executable='yolo_detector_node.py',
            name='yolo_detector',
            output='screen',
            parameters=[yolo_params_file],
            condition=IfCondition(use_yolo),
        ),

        # === Layer 1: GC State Adapter ===
        Node(
            package='op3_soccer_core',
            executable='gc_state_adapter',
            name='gc_state_adapter',
            output='screen',
            parameters=[{
                'team_number': team_number,
                'player_number': player_number,
                'is_goalkeeper': is_goalkeeper,
            }]
        ),

        # === Layer 2: Compliance ===
        Node(
            package='op3_soccer_core',
            executable='op3_compliance',
            name='compliance_node',
            output='screen',
            parameters=[{
                'rules_file': rules_file,
                'team_number': team_number,
                'player_number': player_number,
            }]
        ),

        # === Layer 3: Safety Monitor ===
        Node(
            package='op3_soccer_core',
            executable='op3_safety_monitor',
            name='safety_monitor_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # === Layer 4: Tactical FSM ===
        Node(
            package='op3_soccer_core',
            executable='op3_tactical',
            name='tactical_node',
            output='screen',
            parameters=[{
                'rules_file': rules_file,
                'player_number': player_number,
                'team_number': team_number,
                'is_goalkeeper': is_goalkeeper,
            }]
        ),

        # === Layer 5: Team Communication + Coordinator ===
        Node(
            package='op3_soccer_core',
            executable='op3_team_comm',
            name='team_comm_node',
            output='screen',
            parameters=[{'self_id': player_number}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_team_coordinator',
            name='team_coordinator_node',
            output='screen',
            parameters=[{
                'player_id': player_number,
                'team_number': team_number,
            }]
        ),

        # === Layer 6: Planner (A* + APF) ===
        Node(
            package='op3_soccer_core',
            executable='op3_planner',
            name='planner_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # === Layer 7: Ball Estimator + Perception + Localization ===
        Node(
            package='op3_soccer_core',
            executable='op3_ball_estimator',
            name='ball_estimator_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_perception',
            name='perception_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),
        Node(
            package='op3_soccer_core',
            executable='op3_localization',
            name='localization_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # === Layer 8: Motion (SINGLE publisher to /robotis/...) ===
        Node(
            package='op3_soccer_core',
            executable='op3_motion',
            name='motion_node',
            output='screen',
            parameters=[{'rules_file': rules_file}]
        ),

        # === Webots Walking Bridge (replaces op3_manager in simulation) ===
        Node(
            package='op3_soccer_core',
            executable='op3_webots_walking_bridge',
            name='webots_walking_bridge',
            output='screen',
            parameters=[{
                'gait_flip_hip':   gait_flip_hip,
                'gait_flip_knee':  gait_flip_knee,
                'gait_flip_roll':  gait_flip_roll,
            }]
        ),

        # === Phase 9: FSM Designer web UI (optional) ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('op3_fsm_designer'), 'launch', 'fsm_designer.launch.py'
                ])
            ]),
            condition=IfCondition(use_fsm_designer),
        ),
    ])
