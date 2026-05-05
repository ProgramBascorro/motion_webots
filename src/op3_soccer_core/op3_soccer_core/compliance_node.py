#!/usr/bin/env python3
"""Layer 2 — Compliance Node.
Evaluates RoboCup rule compliance at 10 Hz and publishes /compliance/status.
Does NOT block motion directly — tactical_node reads compliance before publishing intent.
"""
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from soccer_msgs.msg import GameState

from .compliance_evaluator import ComplianceEvaluator
from .config_loader import load_rules


class ComplianceNode(Node):
    def __init__(self) -> None:
        super().__init__("op3_compliance")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._team_number = int(self.declare_parameter("team_number", 1).value)
        self._player_number = int(self.declare_parameter("player_number", 1).value)

        self._center_circle_radius = float(
            self.declare_parameter("center_circle_radius_m",
                                   rules.get("center_circle_radius_m", 0.75)).value
        )
        self._setplay_min_dist = float(
            self.declare_parameter("opponent_distance_setplay_m",
                                   rules.get("opponent_distance_setplay_m", 0.75)).value
        )
        self._executor_role = str(
            self.declare_parameter("setplay_executor_role",
                                   rules.get("setplay_executor_role", "striker")).value
        )
        self._hold_soft_sec = float(self.declare_parameter(
            "max_ball_hold_soft_sec", rules.get("max_ball_hold_soft_sec", 4.0)).value)
        self._hold_hard_sec = float(self.declare_parameter(
            "max_ball_hold_sec", rules.get("max_ball_hold_sec", 5.0)).value)
        # Goalkeeper gets longer hold time per RoboCup rules (6s on ground)
        self._gk_hold_soft_sec = float(self.declare_parameter(
            "gk_ball_hold_soft_sec", rules.get("gk_ball_hold_soft_sec", 5.0)).value)
        self._gk_hold_hard_sec = float(self.declare_parameter(
            "gk_ball_hold_sec", rules.get("gk_ball_hold_sec", 6.0)).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        self._game_state: Optional[GameState] = None
        self._ball_dist: Optional[float] = None
        self._ball_visible = False
        self._ball_close_since: Optional[float] = None
        self._hold_duration = 0.0

        self._kickoff_active = False
        self._kickoff_start: Optional[float] = None
        self._kickoff_tap_done = False
        self._kickoff_window_sec = float(self.declare_parameter(
            "kickoff_window_sec", rules.get("kickoff_window_sec", 10.0)).value)
        self._pre_kick_dist = float(self.declare_parameter(
            "pre_kick_distance", rules.get("pre_kick_distance", 0.22)).value)

        self._current_role = "striker"
        self._current_intent = "stop"

        # GK incapable rule: ball within 0.5m for > 20s without intercept attempt
        self._gk_ball_nearby_dist = float(self.declare_parameter(
            "game_interrupt_distance_m",
            rules.get("game_interrupt_distance_m", 0.5)).value)
        self._gk_incapable_timeout = float(self.declare_parameter(
            "incapable_timeout_sec", rules.get("incapable_timeout_sec", 20.0)).value)
        self._gk_ball_nearby_since: Optional[float] = None
        self._gk_incapable = False

        self._evaluator = ComplianceEvaluator()

        self.create_subscription(GameState, "/game_state", self._game_state_cb, 10)
        self.create_subscription(Vector3Stamped, "/ball/estimate", self._ball_cb, 10)
        self.create_subscription(String, "/tactical/role", self._role_cb, 10)
        self.create_subscription(String, "/team_comm/rx", self._team_comm_cb, 10)
        self.create_subscription(String, "/tactical/intent", self._tactical_intent_cb, 10)

        self._status_pub = self.create_publisher(String, "/compliance/status", 10)
        self._gk_incapable_pub = self.create_publisher(
            String, "/compliance/gk_incapable", 10)

        self.create_timer(1.0 / self._publish_rate, self._evaluate)

        self.get_logger().info(
            f"Compliance node ready (team={self._team_number}, player={self._player_number})"
        )

    def _game_state_cb(self, msg: GameState) -> None:
        prev = self._game_state
        self._game_state = msg
        now = time.monotonic()

        # Track kickoff window
        if msg.gamestate == GameState.GAMESTATE_PLAYING:
            if prev is None or prev.gamestate != GameState.GAMESTATE_PLAYING:
                self._kickoff_start = now
                self._kickoff_active = bool(msg.has_kick_off)
                self._kickoff_tap_done = False
        else:
            self._kickoff_start = None
            self._kickoff_active = False
            self._kickoff_tap_done = False

        if self._kickoff_start is not None:
            if (now - self._kickoff_start) > self._kickoff_window_sec:
                self._kickoff_active = False

    def _ball_cb(self, msg: Vector3Stamped) -> None:
        self._ball_dist = msg.vector.x
        visible = msg.vector.z > 0.5
        now = time.monotonic()

        if visible and not self._ball_visible:
            self._ball_close_since = None

        self._ball_visible = visible

        if visible and self._ball_dist is not None and self._ball_dist <= self._pre_kick_dist:
            if self._ball_close_since is None:
                self._ball_close_since = now
            self._hold_duration = now - self._ball_close_since
        else:
            self._ball_close_since = None
            self._hold_duration = 0.0

    def _role_cb(self, msg: String) -> None:
        self._current_role = msg.data.strip()

    def _team_comm_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Mark kickoff tap done if another robot (striker) signals it
        if data.get("kickoff_tap_done") and data.get("player_id") != self._player_number:
            self._kickoff_tap_done = True

    def _tactical_intent_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        intent = data.get("intent", "stop")
        self._current_intent = intent
        # Once THIS robot publishes kickoff_tap, the tap is done by definition
        if intent == "kickoff_tap" and self._kickoff_active:
            self._kickoff_tap_done = True
        # GK showing effort toward ball clears the incapable timer
        if self._current_role == "goalkeeper" and intent in (
            "intercept_box", "clear_ball", "approach_ball"
        ):
            self._gk_ball_nearby_since = None
            self._gk_incapable = False

    def _check_gk_incapable(self, ball_dist: float) -> None:
        """Track GK idle rule: ball within 0.5m for >20s without intercept → incapable."""
        if self._current_role != "goalkeeper":
            self._gk_ball_nearby_since = None
            self._gk_incapable = False
            return
        now = time.monotonic()
        is_playing = (
            self._game_state is not None
            and self._game_state.gamestate == GameState.GAMESTATE_PLAYING
        )
        if is_playing and ball_dist <= self._gk_ball_nearby_dist:
            if self._gk_ball_nearby_since is None:
                self._gk_ball_nearby_since = now
            elif (now - self._gk_ball_nearby_since) >= self._gk_incapable_timeout:
                if not self._gk_incapable:
                    self.get_logger().warn(
                        f"GK incapable: ball within {self._gk_ball_nearby_dist}m "
                        f"for >{self._gk_incapable_timeout}s without intercept"
                    )
                self._gk_incapable = True
        else:
            self._gk_ball_nearby_since = None
            self._gk_incapable = False

    def _evaluate(self) -> None:
        if self._game_state is None:
            self._publish_status(True, "no_game_state", None, 0.0)
            return

        ball_dist = self._ball_dist if self._ball_dist is not None else 999.0

        # Check kick legality (kickoff guard)
        kick_ok, kick_reason = self._evaluator.check_kick_legal(
            self._game_state,
            self._kickoff_active,
            self._kickoff_tap_done,
            self._center_circle_radius,
            ball_dist,
        )

        # Check set-play distance
        dist_ok, dist_reason = self._evaluator.check_setplay_distance(
            self._game_state,
            self._current_role,
            self._team_number,
            self._executor_role,
            ball_dist,
            self._setplay_min_dist,
        )

        # Check ball hold timer — GK gets longer limit per rules
        is_gk = (self._current_role == "goalkeeper")
        soft = self._gk_hold_soft_sec if is_gk else self._hold_soft_sec
        hard = self._gk_hold_hard_sec if is_gk else self._hold_hard_sec
        hold_ok, hold_reason = self._evaluator.check_ball_hold(
            self._hold_duration,
            soft,
            hard,
        )

        allowed = kick_ok and dist_ok and hold_ok
        if not allowed:
            if not kick_ok:
                reason = kick_reason
                substitute = "dribble_out"
            elif not dist_ok:
                reason = dist_reason
                substitute = "hold_position"
            else:
                reason = hold_reason
                substitute = "force_release"
        else:
            reason = "ok"
            substitute = None

        self._check_gk_incapable(ball_dist)
        self._publish_status(allowed, reason, substitute, self._hold_duration)

    def _publish_status(
        self,
        allowed: bool,
        reason: str,
        substitute: Optional[str],
        hold_sec: float,
    ) -> None:
        status = {
            "allowed": allowed,
            "reason": reason,
            "substitute": substitute,
            "ball_hold_sec": round(hold_sec, 2),
            "kickoff_active": self._kickoff_active,
            "kickoff_tap_done": self._kickoff_tap_done,
            "gk_incapable": self._gk_incapable,
        }
        self._status_pub.publish(String(data=json.dumps(status)))
        if self._gk_incapable:
            self._gk_incapable_pub.publish(String(data=json.dumps({
                "player": self._player_number, "team": self._team_number
            })))


def main() -> None:
    rclpy.init()
    node = ComplianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
