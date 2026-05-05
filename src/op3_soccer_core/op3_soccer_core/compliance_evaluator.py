"""Layer 2 — Compliance Evaluator (pure Python, no ROS).
All RoboCup rule checks live here so the logic is testable without a running node.
"""
from soccer_msgs.msg import GameState


class ComplianceEvaluator:
    """Stateless rule checker. All thresholds passed in as arguments."""

    def check_kick_legal(
        self,
        game_state: GameState,
        kickoff_active: bool,
        kickoff_tap_done: bool,
        center_circle_radius: float,
        ball_dist: float,
    ) -> tuple:
        """Returns (allowed: bool, reason: str).
        Blocks direct goal attempts during active kickoff before tap is done.

        Note:
        `ball_dist` here is robot-to-ball distance, not ball position relative to
        field center. It must NOT be compared against center circle radius.
        """
        if not kickoff_active:
            return True, "ok"
        if kickoff_tap_done:
            return True, "kickoff_tap_done"
        return False, "kickoff_direct_goal_guard"

    def check_setplay_distance(
        self,
        game_state: GameState,
        role: str,
        own_team: int,
        executor_role: str,
        ball_dist: float,
        min_dist: float,
    ) -> tuple:
        """Returns (within_legal_distance: bool, reason: str).
        Non-executors must stay > min_dist from ball during set-play.
        """
        if game_state.secondary_state == GameState.STATE_NORMAL:
            return True, "no_setplay"
        is_for_us = (game_state.secondary_state_team == own_team)
        is_executor = (role == executor_role and is_for_us)
        if is_executor:
            return True, "executor"
        if ball_dist < min_dist:
            return False, "setplay_too_close"
        return True, "ok"

    def check_ball_hold(
        self,
        hold_duration: float,
        soft_limit: float,
        hard_limit: float,
    ) -> tuple:
        """Returns (ok: bool, reason: str).
        Soft limit → warn and request backoff. Hard limit → force release.
        """
        if hold_duration >= hard_limit:
            return False, "ball_hold_hard_limit"
        if hold_duration >= soft_limit:
            return False, "ball_hold_soft_limit"
        return True, "ok"

    def is_setplay_executor(
        self,
        game_state: GameState,
        own_player_id: int,
        own_team: int,
        executor_role: str,
        role: str,
    ) -> bool:
        """True if this robot is the designated set-play executor."""
        if game_state.secondary_state == GameState.STATE_NORMAL:
            return False
        is_for_us = (game_state.secondary_state_team == own_team)
        return is_for_us and (role == executor_role)
