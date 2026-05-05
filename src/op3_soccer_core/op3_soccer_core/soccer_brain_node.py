#!/usr/bin/env python3
"""Legacy node — kept for backward compatibility during Phase 1-3 transition.
Replaced by tactical_node.py + compliance_node.py + safety_monitor_node.py in Phase 4.
Do NOT run alongside tactical_node.py.
"""
import json
import math
import time

try:
    import yaml
except ImportError:
    yaml = None
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from soccer_msgs.msg import GameState

from ament_index_python.packages import get_package_share_directory

from .walking_interface import WalkingInterface


ROLE_GOALKEEPER = "goalkeeper"
ROLE_STRIKER = "striker"
ROLE_SUPPORT = "support"
ROLE_DEFENDER = "defender"


class SoccerBrain(Node):
    def __init__(self) -> None:
        self._rules = {}
        super().__init__("op3_soccer_brain")

        self._team_number = int(self.declare_parameter("team_number", 1).value)
        self._player_number = int(self.declare_parameter("player_number", 1).value)
        self._is_goalkeeper = bool(self.declare_parameter("is_goalkeeper", False).value)
        self._role_override = str(self.declare_parameter("role_override", "").value).strip()
        default_rules = self._default_rules_path()
        self._rules_file = str(self.declare_parameter("rules_file", default_rules).value)
        self._load_rules(self._rules_file)
        self._control_rate = float(self.declare_parameter("control_rate", 10.0).value)

        self._max_x = float(self.declare_parameter("max_x", 0.02).value)
        self._max_y = float(self.declare_parameter("max_y", 0.015).value)
        self._max_yaw = float(self.declare_parameter("max_yaw", 0.25).value)
        self._approach_x_gain = float(self.declare_parameter("approach_x_gain", 0.04).value)
        self._approach_yaw_gain = float(self.declare_parameter("approach_yaw_gain", 1.2).value)
        self._yaw_only_threshold = float(self.declare_parameter("yaw_only_threshold", 0.6).value)
        self._search_yaw = float(self.declare_parameter("search_yaw", 0.2).value)

        self._kick_distance = float(self.declare_parameter("kick_distance", 0.18).value)
        self._hold_distance = float(self.declare_parameter("hold_distance", 0.22).value)
        self._hold_soft_sec = float(self.declare_parameter("hold_soft_sec", 4.0).value)
        self._hold_backoff_sec = float(self.declare_parameter("hold_backoff_sec", 0.8).value)

        self._gk_chase_distance = float(self.declare_parameter("gk_chase_distance", 1.0).value)
        self._gk_kick_distance = float(self.declare_parameter("gk_kick_distance", 0.22).value)
        self._defender_engage_distance = float(
            self.declare_parameter("defender_engage_distance", 1.2).value
        )
        self._support_takeover_distance = float(
            self.declare_parameter("support_takeover_distance", 1.0).value
        )

        self._kick_left_page = int(self.declare_parameter("kick_left_page", 120).value)
        self._kick_right_page = int(self.declare_parameter("kick_right_page", 121).value)
        self._getup_front_page = int(self.declare_parameter("getup_front_page", 122).value)
        self._getup_back_page = int(self.declare_parameter("getup_back_page", 123).value)
        self._kick_cooldown_sec = float(self.declare_parameter("kick_cooldown_sec", 1.0).value)

        self._kickoff_tap_duration = float(self.declare_parameter("kickoff_tap_duration", 0.6).value)
        self._kickoff_tap_speed = float(self.declare_parameter("kickoff_tap_speed", 0.01).value)
        self._kickoff_window_sec = float(self.declare_parameter("kickoff_window_sec", 10.0).value)

        self._fall_pitch_front_deg = float(
            self.declare_parameter("fall_pitch_front_deg", 60.0).value
        )
        self._fall_pitch_back_deg = float(
            self.declare_parameter("fall_pitch_back_deg", 60.0).value
        )
        self._fall_hold_sec = float(self.declare_parameter("fall_hold_sec", 0.3).value)
        self._fall_cooldown_sec = float(self.declare_parameter("fall_cooldown_sec", 4.0).value)

        self._rule_min_opponent_distance = float(
            self.declare_parameter("min_opponent_distance", 0.75).value
        )
        rules_min = self._rules.get('opponent_distance_setplay_m')
        if rules_min is not None and self._rule_min_opponent_distance == 0.75:
            self._rule_min_opponent_distance = float(rules_min)

        self._ball_distance: Optional[float] = None
        self._ball_bearing: Optional[float] = None
        self._ball_visible = False
        self._last_ball_time: Optional[float] = None

        self._game_state: Optional[GameState] = None
        self._last_primary_state: Optional[int] = None

        self._kickoff_active = False
        self._kickoff_start_time: Optional[float] = None
        self._kickoff_tap_until: Optional[float] = None
        self._kickoff_tap_done = False

        self._last_kick_time = 0.0

        self._ball_close_since: Optional[float] = None
        self._hold_backoff_until: Optional[float] = None

        self._team_status: Dict[int, Dict] = {}

        self._odom_yaw = 0.0
        self._imu_pitch = 0.0
        self._imu_roll = 0.0
        self._fall_start: Optional[float] = None
        self._last_getup_time = 0.0

        self._walker = WalkingInterface(self)

        self.create_subscription(GameState, "/game_controller/game_state", self._game_state_cb, 10)
        self.create_subscription(Vector3Stamped, "/ball/estimate", self._ball_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/robotis_op3/imu", self._imu_cb, 10)
        self.create_subscription(String, "/team_comm/rx", self._team_comm_cb, 10)
        self._team_comm_pub = self.create_publisher(String, "/team_comm/tx", 10)

        self.create_timer(1.0 / self._control_rate, self._control_loop)

        self.get_logger().info(
            f"Soccer brain ready (team={self._team_number}, player={self._player_number})"
        )

    def _game_state_cb(self, msg: GameState) -> None:
        self._game_state = msg

    def _ball_cb(self, msg: Vector3Stamped) -> None:
        self._ball_distance = msg.vector.x
        self._ball_bearing = msg.vector.y
        self._ball_visible = msg.vector.z > 0.5
        if self._ball_visible:
            self._last_ball_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self._imu_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self._imu_pitch = math.asin(sinp)

    def _team_comm_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        player_id = int(data.get("player_id", -1))
        if player_id <= 0:
            return
        self._team_status[player_id] = data

    def _control_loop(self) -> None:
        now = time.monotonic()
        self._walker.tick(now)
        self._publish_team_comm(now)

        if self._game_state is None:
            return

        self._update_kickoff_state(now)

        if self._handle_fall(now):
            return

        primary = int(self._game_state.gamestate)
        if primary != self._last_primary_state:
            self._last_primary_state = primary

        if primary == GameState.GAMESTATE_INITIAL:
            self._walker.enable_base()
            self._walker.stop_walking()
            return
        if primary == GameState.GAMESTATE_READY:
            self._walker.enable_action()
            self._walker.stop_walking()
            return
        if primary == GameState.GAMESTATE_SET:
            self._walker.stop_walking()
            return
        if primary == GameState.GAMESTATE_FINISHED:
            self._walker.stop_walking()
            return

        if self._game_state.penalty != GameState.PENALTY_NONE:
            self._walker.stop_walking()
            return

        if self._handle_setplay(now):
            return

        role = self._resolve_role()
        if role == ROLE_GOALKEEPER:
            self._do_goalkeeper(now)
        elif role == ROLE_STRIKER:
            self._do_striker(now)
        elif role == ROLE_SUPPORT:
            self._do_support(now)
        else:
            self._do_defender(now)

    def _update_kickoff_state(self, now: float) -> None:
        if self._game_state is None:
            return
        if self._game_state.gamestate == GameState.GAMESTATE_PLAYING:
            if self._kickoff_start_time is None:
                self._kickoff_start_time = now
                self._kickoff_active = bool(self._game_state.has_kick_off)
                self._kickoff_tap_done = False
                self._kickoff_tap_until = None
        else:
            self._kickoff_start_time = None
            self._kickoff_active = False
            self._kickoff_tap_done = False
            self._kickoff_tap_until = None

        if self._kickoff_start_time is not None:
            if (now - self._kickoff_start_time) > self._kickoff_window_sec:
                self._kickoff_active = False

    def _handle_setplay(self, now: float) -> bool:
        if self._game_state is None:
            return False
        if self._game_state.secondary_state == GameState.STATE_NORMAL:
            return False

        setplay_team = int(self._game_state.secondary_state_team)
        if setplay_team == 0:
            setplay_team = self._team_number if self._game_state.has_kick_off else -1
        is_for_us = setplay_team == self._team_number
        role = self._resolve_role()

        if not is_for_us:
            if self._ball_visible and self._ball_distance is not None:
                if self._ball_distance < self._rule_min_opponent_distance:
                    self._walker.enable_walking()
                    self._walker.set_walk(
                        -self._max_x, 0.0, 0.0, self._max_x, self._max_y, self._max_yaw
                    )
                    self._walker.start_walking()
                    return True
            self._walker.stop_walking()
            return True

        if role != ROLE_STRIKER:
            if self._ball_visible and self._ball_distance is not None:
                if self._ball_distance < self._rule_min_opponent_distance:
                    self._walker.enable_walking()
                    self._walker.set_walk(
                        -self._max_x, 0.0, 0.0, self._max_x, self._max_y, self._max_yaw
                    )
                    self._walker.start_walking()
                    return True
            self._walker.stop_walking()
            return True

        return False

    def _handle_fall(self, now: float) -> bool:
        if now - self._last_getup_time < self._fall_cooldown_sec:
            return False
        pitch_deg = math.degrees(self._imu_pitch)
        if abs(pitch_deg) < min(self._fall_pitch_front_deg, self._fall_pitch_back_deg):
            self._fall_start = None
            return False

        if self._fall_start is None:
            self._fall_start = now
            return False
        if now - self._fall_start < self._fall_hold_sec:
            return False

        if pitch_deg > self._fall_pitch_front_deg:
            self._walker.kick(self._getup_front_page)
        elif pitch_deg < -self._fall_pitch_back_deg:
            self._walker.kick(self._getup_back_page)
        self._last_getup_time = now
        self._fall_start = None
        return True

    def _resolve_role(self) -> str:
        if self._role_override:
            return self._role_override
        if self._is_goalkeeper or self._player_number == 1:
            return ROLE_GOALKEEPER
        if self._player_number == 2:
            return ROLE_STRIKER
        if self._player_number == 3:
            role = ROLE_SUPPORT
        else:
            role = ROLE_DEFENDER

        gk_penalized = self._is_role_penalized(ROLE_GOALKEEPER)
        striker_penalized = self._is_role_penalized(ROLE_STRIKER)

        if role == ROLE_SUPPORT and gk_penalized:
            return ROLE_GOALKEEPER
        if role == ROLE_SUPPORT and striker_penalized:
            return ROLE_STRIKER
        return role

    def _is_role_penalized(self, role: str) -> bool:
        for data in self._team_status.values():
            if data.get("role") == role and data.get("penalized"):
                return True
        return False

    def _do_goalkeeper(self, now: float) -> None:
        if not self._ball_visible or self._ball_distance is None or self._ball_bearing is None:
            self._search()
            return
        if self._ball_distance <= self._gk_kick_distance:
            self._kick(now)
            return
        if self._ball_distance <= self._gk_chase_distance:
            self._approach_ball()
            return
        self._walker.stop_walking()

    def _do_striker(self, now: float) -> None:
        if not self._ball_visible or self._ball_distance is None or self._ball_bearing is None:
            self._search()
            return
        if self._handle_hold(now):
            return
        if self._ball_distance <= self._kick_distance:
            if self._kickoff_active and not self._kickoff_tap_done:
                self._do_kickoff_tap(now)
            else:
                self._kick(now)
            return
        self._approach_ball()

    def _do_support(self, now: float) -> None:
        striker_penalized = self._is_role_penalized(ROLE_STRIKER)
        if striker_penalized:
            self._do_striker(now)
            return
        if self._ball_visible and self._ball_distance is not None:
            if self._ball_distance < self._support_takeover_distance:
                self._approach_ball()
                return
        self._walker.stop_walking()

    def _do_defender(self, now: float) -> None:
        if self._ball_visible and self._ball_distance is not None:
            if self._ball_distance < self._defender_engage_distance:
                self._approach_ball()
                return
        self._walker.stop_walking()

    def _search(self) -> None:
        self._walker.enable_walking()
        self._walker.set_walk(
            0.0, 0.0, self._search_yaw, self._max_x, self._max_y, self._max_yaw
        )
        self._walker.start_walking()

    def _approach_ball(self) -> None:
        if self._ball_distance is None or self._ball_bearing is None:
            return
        yaw_cmd = self._approach_yaw_gain * self._ball_bearing
        x_cmd = self._approach_x_gain * self._ball_distance
        if abs(self._ball_bearing) > self._yaw_only_threshold:
            x_cmd = 0.0
        self._walker.enable_walking()
        self._walker.set_walk(x_cmd, 0.0, yaw_cmd, self._max_x, self._max_y, self._max_yaw)
        self._walker.start_walking()

    def _kick(self, now: float) -> None:
        if now - self._last_kick_time < self._kick_cooldown_sec:
            return
        self._walker.stop_walking()
        if self._ball_bearing is None:
            return
        page = self._kick_right_page if self._ball_bearing >= 0 else self._kick_left_page
        self._walker.kick(page)
        self._last_kick_time = now

    def _do_kickoff_tap(self, now: float) -> None:
        if self._kickoff_tap_until is None:
            self._kickoff_tap_until = now + self._kickoff_tap_duration
            self._walker.enable_walking()
        if now < self._kickoff_tap_until:
            self._walker.set_walk(
                self._kickoff_tap_speed, 0.0, 0.0, self._max_x, self._max_y, self._max_yaw
            )
            self._walker.start_walking()
            return
        self._walker.stop_walking()
        self._kickoff_tap_done = True

    def _handle_hold(self, now: float) -> bool:
        if self._ball_distance is None:
            return False
        if self._ball_distance > self._hold_distance:
            self._ball_close_since = None
            self._hold_backoff_until = None
            return False

        if self._ball_close_since is None:
            self._ball_close_since = now
            return False

        if self._hold_backoff_until is not None:
            if now < self._hold_backoff_until:
                self._walker.enable_walking()
                self._walker.set_walk(
                    -self._max_x, 0.0, 0.0, self._max_x, self._max_y, self._max_yaw
                )
                self._walker.start_walking()
                return True
            self._hold_backoff_until = None
            return False

        if (now - self._ball_close_since) > self._hold_soft_sec:
            self._hold_backoff_until = now + self._hold_backoff_sec
            return True
        return False

    def _default_rules_path(self) -> str:
        try:
            share = get_package_share_directory('op3_soccer_core')
            return share + '/config/rc_hl_kidsize.yaml'
        except Exception:
            return ''

    def _load_rules(self, path: str) -> None:
        if not path or yaml is None:
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to load rules file: {exc}')
            return
        rules = data.get('rc_hl_kidsize', data)
        if isinstance(rules, dict):
            self._rules = rules

    def _publish_team_comm(self, now: float) -> None:
        payload = {
            "player_id": self._player_number,
            "team": self._team_number,
            "role": self._resolve_role(),
            "penalized": (
                self._game_state.penalty != GameState.PENALTY_NONE
                if self._game_state else False
            ),
            "ball_seen": bool(self._ball_visible),
            "ball_distance": float(self._ball_distance) if self._ball_distance is not None else -1.0,
            "ball_bearing": float(self._ball_bearing) if self._ball_bearing is not None else 0.0,
            "yaw": float(self._odom_yaw),
            "stamp": now,
        }
        self._team_comm_pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = SoccerBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
