"""4v4 simulation — Webots GUI + full strategy stack + optional YOLO per robot.

Usage:
  ros2 launch op3_soccer_core soccer_4v4_gui_full_yolo.launch.py
  ros2 launch op3_soccer_core soccer_4v4_gui_full_yolo.launch.py use_yolo:=true

Optional args:
  controller_startup_delay:=35.0   seconds to wait before extern controllers start
  use_yolo:=false                  launch YOLO per robot (default: false)
  yolo_input_size:=416             YOLO input resolution (basic perception default)
  yolo_frame_skip:=2               YOLO frame skip for lighter runtime
  yolo_publish_debug:=false        publish debug image stream from YOLO
  gait_flip_hip:=false             flip hip-pitch direction if robot walks backward
  gait_flip_knee:=false            flip knee direction if robot falls fwd/bwd
  gait_flip_roll:=false            flip hip-roll direction if robot falls sideways

Fixes applied vs v1
--------------------
  * use_yolo default = false (8 YOLO instances crash WSL2 GPU)
  * team_comm uses ROS relay mode (not UDP broadcast) — works on WSL2 same-machine
  * 2 GC bridges (one per team) for proper per-team status return to GameController
  * webots_walking_bridge added per robot with gait sign-flip params
  * joint_states feedback enables correct action-start interpolation

Namespace convention
--------------------
  Per-robot topics : /{team{N}_p{M}}/{topic}
  Per-team topics  : /team{N}/team_comm/relay (team relay bus only)
  Global topics    : /{topic}           (game_controller/state_json)
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Joint names (must match op3_extern_controller.cpp) ─────────────────────
_OP3_JOINTS = [
    "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
    "r_el", "l_el",
    "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
    "head_pan", "head_tilt",
]

# ── Robot configuration ─────────────────────────────────────────────────────
# (team, player, webots_name, x, y, is_goalkeeper, legacy_udp_port)
#   player_number role map: 1=GK, 2=striker, 3=support, 4=defender
_ROBOTS = [
    (1, 1, "T1P1", -4.2,  0.0, True,  10031),
    (1, 2, "T1P2", -0.9,  0.0, False, 10031),
    (1, 3, "T1P3", -1.5,  0.5, False, 10031),
    (1, 4, "T1P4", -2.5, -0.4, False, 10031),
    (2, 1, "T2P1",  4.2,  0.0, True,  10032),
    (2, 2, "T2P2",  0.9,  0.3, False, 10032),
    (2, 3, "T2P3",  1.5, -0.5, False, 10032),
    (2, 4, "T2P4",  2.5,  0.4, False, 10032),
]


# ── Namespace helpers ────────────────────────────────────────────────────────

def _ns(team: int, player: int) -> str:
    """Per-robot namespace string, e.g. 'team1_p2'."""
    return f"team{team}_p{player}"


def _remap(topic: str, ns: str):
    """Remap absolute topic to per-robot namespaced topic."""
    return (topic, f"/{ns}{topic}")


# ── Extern controller ────────────────────────────────────────────────────────

def _extern_controller(team: int, player: int, webots_name: str,
                        delay, gain_file: str) -> TimerAction:
    """Wrap op3_extern_controller in a TimerAction with full topic remaps."""
    ns = _ns(team, player)

    joint_remaps = [
        (f"/robotis_op3/{jn}_position/command",
         f"/{ns}/robotis_op3/{jn}_position/command")
        for jn in _OP3_JOINTS
    ]

    sensor_remaps = [
        _remap("/robotis_op3/joint_states",       ns),
        _remap("/robotis_op3/imu",                ns),
        _remap("/robotis_op3/com",                ns),
        _remap("/robotis_op3/camera/camera_info", ns),
        _remap("/robotis_op3/camera/image_raw",   ns),
        _remap("/odom",                           ns),
    ]

    return TimerAction(
        period=delay,
        actions=[
            Node(
                package="op3_webots_ros2",
                executable="op3_extern_controller",
                name=f"extern_ctrl_{ns}",
                output="screen",
                parameters=[{"gain_file_path": gain_file}],
                additional_env={"WEBOTS_ROBOT_NAME": webots_name},
                remappings=sensor_remaps + joint_remaps,
            )
        ],
    )


# ── Strategy stack (one full stack per robot) ────────────────────────────────

def _strategy_stack(team: int, player: int, is_goalkeeper: bool,
                    rules_file: str, yolo_params_file: str,
                    use_yolo,
                    yolo_input_size,
                    yolo_frame_skip,
                    yolo_publish_debug,
                    gait_flip_hip=False, gait_flip_knee=False,
                    gait_flip_roll=False) -> list:
    """Return the list of strategy + YOLO nodes for one robot."""
    ns = _ns(team, player)

    def r(topic):  # per-robot remap
        return _remap(topic, ns)

    nodes = [

        # ── Layer 1: GC State Adapter ────────────────────────────────────────
        # Subscribes to global /game_controller/state_json (no remap needed).
        Node(
            package="op3_soccer_core",
            executable="gc_state_adapter",
            name=f"gc_state_adapter_{ns}",
            output="screen",
            parameters=[{
                "team_number": team,
                "player_number": player,
                "is_goalkeeper": is_goalkeeper,
            }],
            remappings=[
                r("/game_state"),
                r("/game_context"),
                r("/game_controller/game_state"),  # legacy output
            ],
        ),

        # ── Layer 2: Compliance ──────────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_compliance",
            name=f"compliance_{ns}",
            output="screen",
            parameters=[{
                "team_number": team,
                "player_number": player,
                "rules_file": rules_file,
            }],
            remappings=[
                r("/game_state"),
                r("/ball/estimate"),
                r("/tactical/role"),
                r("/compliance/status"),
                r("/team_comm/rx"),
            ],
        ),

        # ── Layer 3: Safety Monitor ──────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_safety_monitor",
            name=f"safety_{ns}",
            output="screen",
            parameters=[{"rules_file": rules_file}],
            remappings=[
                r("/robotis_op3/imu"),
                r("/safety/motion_veto"),
                r("/safety/status"),
                r("/motion/command"),
                # Keep fallen status isolated per robot in 4v4.
                r("/game_controller/status/fallen"),
            ],
        ),

        # ── Layer 4: Tactical FSM ────────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_tactical",
            name=f"tactical_{ns}",
            output="screen",
            parameters=[{
                "team_number": team,
                "player_number": player,
                "is_goalkeeper": is_goalkeeper,
                "rules_file": rules_file,
            }],
            remappings=[
                r("/game_state"),       r("/game_context"),
                r("/ball/estimate"),    r("/compliance/status"),
                r("/safety/motion_veto"),
                r("/team_coordinator/snapshot"), r("/team_coordinator/ball_owner"),
                r("/motion/status"),
                r("/localization/confidence"), r("/localization/pose"),
                r("/robotis_op3/imu"),
                r("/tactical/intent"),  r("/tactical/role"),
                r("/motion/command"),
                r("/team_comm/tx"),     r("/team_comm/rx"),
            ],
        ),

        # ── Layer 5a: Team Comm (ROS relay mode — no UDP, works on WSL2) ───────
        # use_ros_relay=True: each robot's team_comm publishes/subscribes via a
        # shared ROS2 topic instead of UDP broadcast. Solves WSL2 broadcast issue.
        # relay_topic is per-team so Team1 and Team2 stay isolated.
        Node(
            package="op3_soccer_core",
            executable="op3_team_comm",
            name=f"team_comm_{ns}",
            output="screen",
            parameters=[{
                "self_id":       player,
                "ignore_self":   True,
                "use_ros_relay": True,
                "relay_topic":   f"/team{team}/team_comm/relay",
            }],
            remappings=[
                r("/team_comm/tx"),
                r("/team_comm/rx"),
            ],
        ),

        # ── Layer 5b: Team Coordinator ───────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_team_coordinator",
            name=f"team_coord_{ns}",
            output="screen",
            parameters=[{
                "player_id": player,
                "team_number": team,
                "rules_file": rules_file,
            }],
            remappings=[
                r("/team_comm/rx"),
                r("/ball/estimate"),
                r("/tactical/role"),    r("/tactical/intent"),
                r("/team_coordinator/snapshot"), r("/team_coordinator/ball_owner"),
            ],
        ),

        # ── Layer 6: Planner (A* + APF) ──────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_planner",
            name=f"planner_{ns}",
            output="screen",
            parameters=[{"rules_file": rules_file}],
            remappings=[
                r("/tactical/intent"),
                r("/localization/pose"),
                r("/perception/obstacles"),
                r("/motion/command"),
            ],
        ),

        # ── Layer 7a: Ball Estimator (parameterised topics) ──────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_ball_estimator",
            name=f"ball_est_{ns}",
            output="screen",
            parameters=[{
                "detections_topic":  f"/{ns}/vision/yolo/detections",
                "ball_center_topic": f"/{ns}/vision/yolo/ball_center",
                "output_topic":      f"/{ns}/ball/estimate",
                "rules_file": rules_file,
            }],
        ),

        # ── Layer 7b: Perception ─────────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_perception",
            name=f"perception_{ns}",
            output="screen",
            parameters=[{"rules_file": rules_file}],
            remappings=[
                r("/vision/yolo/detections"),
                r("/perception/obstacles"),
                r("/perception/landmarks"),
            ],
        ),

        # ── Layer 7c: Localization ───────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_localization",
            name=f"localization_{ns}",
            output="screen",
            parameters=[{"rules_file": rules_file}],
            remappings=[
                r("/odom"),
                r("/robotis_op3/imu"),
                r("/perception/landmarks"),
                r("/localization/pose"),
                r("/localization/confidence"),
            ],
        ),

        # ── Layer 8: Motion ──────────────────────────────────────────────────
        Node(
            package="op3_soccer_core",
            executable="op3_motion",
            name=f"motion_{ns}",
            output="screen",
            parameters=[{"rules_file": rules_file}],
            remappings=[
                r("/motion/command"),      r("/safety/motion_veto"),
                r("/motion/status"),
                # WalkingInterface /robotis/* → namespaced, consumed by bridge
                r("/robotis/enable_ctrl_module"),
                r("/robotis/walking/command"),
                r("/robotis/walking/set_params"),
                r("/robotis/action/page_num"),
                r("/robotis/walking/get_params"),
            ],
        ),

        # ── Webots Walking Bridge ────────────────────────────────────────────
        # Translates namespaced /robotis/* → direct joint position commands.
        # gait_flip_* parameters can be set at launch to correct motion direction.
        Node(
            package="op3_soccer_core",
            executable="op3_webots_walking_bridge",
            name=f"walking_bridge_{ns}",
            output="screen",
            parameters=[{
                "joint_cmd_prefix":    f"/{ns}/robotis_op3",
                "ball_estimate_topic": f"/{ns}/ball/estimate",   # namespaced per robot
                "gait_flip_hip":       gait_flip_hip,
                "gait_flip_knee":      gait_flip_knee,
                "gait_flip_roll":      gait_flip_roll,
            }],
            remappings=[
                r("/robotis/enable_ctrl_module"),
                r("/robotis/walking/command"),
                r("/robotis/walking/set_params"),
                r("/robotis/action/page_num"),
                r("/robotis/walking/get_params"),
            ],
        ),

        # ── YOLO Detector ────────────────────────────────────────────────────
        Node(
            package="op3_yolo_vision",
            executable="yolo_detector_node.py",
            name=f"yolo_{ns}",
            output="screen",
            parameters=[
                yolo_params_file,
                {
                    "image_topic":       f"/{ns}/robotis_op3/camera/image_raw",
                    "ball_center_topic": f"/{ns}/vision/yolo/ball_center",
                    # Basic perception defaults for 4v4 runtime stability
                    "use_gpu": False,
                    "use_fp16": False,
                    "dnn_backend": "opencv",
                    "dnn_target": "cpu",
                    "input_size": yolo_input_size,
                    "frame_skip": yolo_frame_skip,
                    "publish_debug": yolo_publish_debug,
                },
            ],
            remappings=[
                r("/vision/yolo/detections"),
                r("/vision/yolo/debug"),
            ],
            condition=IfCondition(use_yolo),
        ),
    ]

    return nodes


# ── Top-level launch description ─────────────────────────────────────────────

def generate_launch_description():
    soccer_core_share = get_package_share_directory("op3_soccer_core")
    op3_webots_share  = get_package_share_directory("op3_webots_ros2")
    yolo_share        = get_package_share_directory("op3_yolo_vision")

    rules_file       = os.path.join(soccer_core_share, "config", "rc_hl_kidsize.yaml")
    yolo_params_file = os.path.join(yolo_share, "config", "yolo.yaml")
    world_file       = os.path.join(op3_webots_share, "worlds",
                                    "robotis_op3_extern_4v4.wbt")
    gain_file        = os.path.join(op3_webots_share, "resource",
                                    "op3_pid_gain_default.yaml")

    controller_delay = LaunchConfiguration("controller_startup_delay")
    use_yolo         = LaunchConfiguration("use_yolo")
    yolo_input_size  = LaunchConfiguration("yolo_input_size")
    yolo_frame_skip  = LaunchConfiguration("yolo_frame_skip")
    yolo_publish_debug = LaunchConfiguration("yolo_publish_debug")

    actions = [
        # ── Arguments ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "controller_startup_delay",
            default_value="35.0",
            description="Seconds to wait after Webots starts before connecting "
                        "extern controllers. Reduce to 15.0 on fast machines.",
        ),
        DeclareLaunchArgument(
            "use_yolo",
            default_value="false",
            description="Launch YOLO detector per robot. "
                        "Set true only on a machine with enough GPU VRAM for 8 instances.",
        ),
        DeclareLaunchArgument(
            "yolo_input_size",
            default_value="416",
            description="YOLO input resolution (basic perception default for 4v4).",
        ),
        DeclareLaunchArgument(
            "yolo_frame_skip",
            default_value="2",
            description="YOLO frame skip. Higher = lighter CPU/GPU usage.",
        ),
        DeclareLaunchArgument(
            "yolo_publish_debug",
            default_value="false",
            description="Publish YOLO debug image stream (/vision/yolo/debug).",
        ),
        DeclareLaunchArgument("gait_flip_hip",   default_value="false",
                              description="Flip hip-pitch sign if robot walks backward"),
        DeclareLaunchArgument("gait_flip_knee",  default_value="false",
                              description="Flip knee sign if robot falls fwd/bwd"),
        DeclareLaunchArgument("gait_flip_roll",  default_value="false",
                              description="Flip hip-roll sign if robot falls sideways"),

        # ── Webots GUI (no --batch so the window appears) ───────────────────
        ExecuteProcess(
            cmd=["/usr/local/bin/webots", "--mode=realtime", world_file],
            output="screen",
            name="webots_4v4",
        ),

        # ── Two GC Bridges (one per team) ───────────────────────────────────
        # Each team gets its own bridge so all 4 robots send status back to GC.
        # Both bridges listen on port 3838 (same GC packet arrives at the OS;
        # with SO_REUSEADDR one bridge per team receives it).
        # Team 1 bridge — receives GC packets, publishes global /game_controller/state_json
        Node(
            package="op3_webots_ros2",
            executable="op3_gc_bridge",
            name="gc_bridge_team1",
            output="screen",
            parameters=[{
                "gc_host":       "127.0.0.1",
                "control_port":  3838,
                "status_port":   3939,
                "team_number":   1,
                "player_number": 1,   # representative player for status return
                "rx_period_ms":  20,
                "tx_period_ms":  500,
            }],
            # Team status source is goalkeeper (team1_p1) for deterministic
            # GameController return data in 4v4.
            remappings=[
                ("/game_controller/status/fallen", "/team1_p1/game_controller/status/fallen"),
                ("/game_controller/status/hl_message", "/team1_p1/game_controller/status/hl_message"),
            ],
        ),
        # Team 2 bridge — sends team 2 status to standard GC return port
        Node(
            package="op3_webots_ros2",
            executable="op3_gc_bridge",
            name="gc_bridge_team2",
            output="screen",
            parameters=[{
                "gc_host":       "127.0.0.1",
                "control_port":  3838,
                "status_port":   3939,
                "team_number":   2,
                "player_number": 1,
                "rx_period_ms":  20,
                "tx_period_ms":  500,
            }],
            # Team status source is goalkeeper (team2_p1) for deterministic
            # GameController return data in 4v4.
            remappings=[
                ("/game_controller/status/fallen", "/team2_p1/game_controller/status/fallen"),
                ("/game_controller/status/hl_message", "/team2_p1/game_controller/status/hl_message"),
            ],
        ),
    ]

    gait_flip_hip  = LaunchConfiguration("gait_flip_hip")
    gait_flip_knee = LaunchConfiguration("gait_flip_knee")
    gait_flip_roll = LaunchConfiguration("gait_flip_roll")

    # ── Per-robot strategy stacks and extern controllers ─────────────────────
    for team, player, webots_name, _x, _y, is_gk, _udp_port in _ROBOTS:
        actions.extend(
            _strategy_stack(team, player, is_gk,
                            rules_file, yolo_params_file,
                            use_yolo,
                            yolo_input_size,
                            yolo_frame_skip,
                            yolo_publish_debug,
                            gait_flip_hip=gait_flip_hip,
                            gait_flip_knee=gait_flip_knee,
                            gait_flip_roll=gait_flip_roll)
        )
        actions.append(
            _extern_controller(team, player, webots_name,
                               controller_delay, gain_file)
        )

    return LaunchDescription(actions)
