#!/usr/bin/env python3
import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from soccer_msgs.msg import GameState


PRIMARY_STATE_MAP = {
    "INITIAL": GameState.GAMESTATE_INITIAL,
    "READY": GameState.GAMESTATE_READY,
    "SET": GameState.GAMESTATE_SET,
    "PLAYING": GameState.GAMESTATE_PLAYING,
    "FINISHED": GameState.GAMESTATE_FINISHED,
}

SECONDARY_STATE_MAP = {
    "NORMAL": GameState.STATE_NORMAL,
    "PENALTYSHOOT": GameState.STATE_PENALTYSHOOT,
    "OVERTIME": GameState.STATE_OVERTIME,
    "TIMEOUT": GameState.STATE_TIMEOUT,
    "DIRECT_FREEKICK": GameState.STATE_DIRECT_FREEKICK,
    "INDIRECT_FREEKICK": GameState.STATE_INDIRECT_FREEKICK,
    "PENALTYKICK": GameState.STATE_PENALTYKICK,
    "CORNER_KICK": GameState.STATE_CORNER_KICK,
    "GOAL_KICK": GameState.STATE_GOAL_KICK,
    "THROW_IN": GameState.STATE_THROW_IN,
}

TEAM_COLOR_MAP = {
    0: GameState.TEAM_COLOR_BLUE,
    1: GameState.TEAM_COLOR_RED,
}


class GCStateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gc_state_adapter")

        self._team_number = int(self.declare_parameter("team_number", 1).value)
        self._player_number = int(self.declare_parameter("player_number", 1).value)
        self._is_goalkeeper = bool(self.declare_parameter("is_goalkeeper", False).value)

        self._state_sub = self.create_subscription(
            String, "/game_controller/state_json", self._state_callback, 10
        )
        # Canonical topic for all new strategy nodes
        self._state_pub = self.create_publisher(GameState, "/game_state", 10)
        # Legacy alias — keeps soccer_brain_node.py working during transition
        self._state_pub_legacy = self.create_publisher(GameState, "/game_controller/game_state", 10)
        self._context_pub = self.create_publisher(String, "/game_context", 10)

        self.get_logger().info(
            f"GC State Adapter ready (team={self._team_number}, player={self._player_number})"
        )

    def _state_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid GC JSON: {exc}")
            return

        game_state = GameState()
        game_state.header.stamp = self.get_clock().now().to_msg()

        state_name = str(data.get("state", "INITIAL"))
        sec_name = str(data.get("secondary_state", "NORMAL"))
        game_state.gamestate = PRIMARY_STATE_MAP.get(state_name, GameState.GAMESTATE_INITIAL)
        game_state.secondary_state = SECONDARY_STATE_MAP.get(sec_name, GameState.STATE_NORMAL)

        secondary_info = data.get("secondary_state_info", [0, 0, 0, 0])
        if isinstance(secondary_info, list) and len(secondary_info) >= 2:
            game_state.secondary_state_team = int(secondary_info[0])
            game_state.secondary_state_mode = int(secondary_info[1])
        else:
            game_state.secondary_state_team = 0
            game_state.secondary_state_mode = 0

        game_state.first_half = bool(data.get("first_half", 1))
        game_state.seconds_remaining = int(data.get("secs_remaining", 0))
        game_state.secondary_seconds_remaining = int(data.get("secondary_time", 0))

        kicking_team = int(data.get("kicking_team", 0))
        game_state.has_kick_off = (kicking_team == self._team_number)

        teams = data.get("teams", [])
        own_team, rival_team = self._select_teams(teams)

        if own_team is not None:
            game_state.own_score = int(own_team.get("score", 0))
            team_color = int(own_team.get("field_player_colour", 0))
            game_state.team_color = TEAM_COLOR_MAP.get(team_color, GameState.TEAM_COLOR_BLUE)
            game_state.penalty_shot = int(own_team.get("penalty_shot", 0))
            game_state.single_shots = int(own_team.get("single_shots", 0))
            game_state.coach_message = own_team.get("coach_message_hex", "")
        if rival_team is not None:
            game_state.rival_score = int(rival_team.get("score", 0))

        player_info = self._find_player_info(own_team)
        if player_info is not None:
            game_state.penalty = int(player_info.get("penalty", 0))
            game_state.seconds_till_unpenalized = int(player_info.get("secs_till_unpenalised", 0))
        else:
            game_state.penalty = GameState.PENALTY_NONE
            game_state.seconds_till_unpenalized = 0

        game_state.drop_in_team = bool(data.get("drop_in_team", 0))
        game_state.drop_in_time = int(data.get("drop_in_time", 0))

        self._state_pub.publish(game_state)
        self._state_pub_legacy.publish(game_state)

        context = {
            "is_kickoff_for_us": game_state.has_kick_off,
            "is_setplay_for_us": (game_state.secondary_state_team == self._team_number),
            "is_penalized": game_state.penalty != GameState.PENALTY_NONE,
            "state": state_name,
            "secondary": sec_name,
        }
        self._context_pub.publish(String(data=json.dumps(context)))

    def _select_teams(self, teams: Any) -> tuple:
        if not isinstance(teams, list) or len(teams) < 2:
            return None, None

        own_team = None
        rival_team = None
        for team in teams:
            if int(team.get("team_number", -1)) == self._team_number:
                own_team = team
            else:
                rival_team = team

        if own_team is None:
            own_team = teams[0]
            rival_team = teams[1] if len(teams) > 1 else None
        elif rival_team is None and len(teams) > 1:
            rival_team = teams[1] if teams[0] is own_team else teams[0]

        return own_team, rival_team

    def _find_player_info(self, own_team: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if own_team is None:
            return None
        players = own_team.get("players", [])
        if not isinstance(players, list):
            return None
        for player in players:
            if int(player.get("player_index", -1)) == self._player_number:
                return player
        return None


def main() -> None:
    rclpy.init()
    node = GCStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
