"""Layer 4 — Goalkeeper FSM.
States: HOLD_GOAL → INTERCEPT_BOX → CLEAR → RECOVER

HOLD_GOAL now computes a target bearing/distance toward the goal-centre so
tactical_node can navigate the GK to the correct defensive position instead of
freezing in place wherever it spawned.
"""
import math
from ..intent_types import Intent, IntentType
from ..world_model import WorldModel
from .base_role import BaseRole

_S_HOLD_GOAL    = "HOLD_GOAL"
_S_INTERCEPT_BOX = "INTERCEPT_BOX"
_S_CLEAR        = "CLEAR"
_S_RECOVER      = "RECOVER"

# Tolerance: stop navigating when within this distance of goal centre
_AT_POST_DIST_M = 0.20


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class GoalkeeperFSM(BaseRole):
    def __init__(
        self,
        gk_chase_distance: float = 1.0,
        gk_kick_distance: float = 0.22,
        kick_left_page: int = 120,
        kick_right_page: int = 121,
        goal_x: float = -4.0,   # X of goal centre (world frame); -4.0 for team 1
        goal_y: float = 0.0,
    ) -> None:
        self._state = _S_HOLD_GOAL
        self._gk_chase_distance = gk_chase_distance
        self._gk_kick_distance  = gk_kick_distance
        self._kick_left_page    = kick_left_page
        self._kick_right_page   = kick_right_page
        self._goal_x = goal_x
        self._goal_y = goal_y

    @property
    def state_name(self) -> str:
        return self._state

    def update(self, world: WorldModel) -> Intent:
        if world.safety_veto:
            self._state = _S_RECOVER
            return Intent(IntentType.RECOVER, reason="safety_veto")

        if not world.is_playing or world.is_penalized:
            self._state = _S_HOLD_GOAL
            return Intent(IntentType.STOP, reason="not_playing")

        return self._step(world)

    def _goal_intent(self, world: WorldModel) -> Intent:
        """Return HOLD_GOAL intent with target_dist/bearing toward goal centre."""
        dx = self._goal_x - world.pose_x
        dy = self._goal_y - world.pose_y
        dist = math.hypot(dx, dy)
        if dist < _AT_POST_DIST_M:
            return Intent(IntentType.HOLD_GOAL, reason="at_post")
        world_angle = math.atan2(dy, dx)
        bearing = _wrap(world_angle - world.pose_yaw)
        return Intent(IntentType.HOLD_GOAL,
                      target_dist=round(dist, 3),
                      target_bearing=round(bearing, 4),
                      reason="navigate_to_post")

    def _step(self, world: WorldModel) -> Intent:
        if self._state == _S_RECOVER:
            if not world.safety_veto:
                self._state = _S_HOLD_GOAL
            return Intent(IntentType.RECOVER, reason="recovering")

        if self._state == _S_HOLD_GOAL:
            if world.ball_visible and world.ball_dist <= self._gk_chase_distance:
                self._state = _S_INTERCEPT_BOX
                return self._step(world)   # immediate intent from new state
            # Navigate back to goal centre when not chasing
            return self._goal_intent(world)

        if self._state == _S_INTERCEPT_BOX:
            if not world.ball_visible:
                self._state = _S_HOLD_GOAL
                return self._goal_intent(world)
            if world.ball_dist > self._gk_chase_distance + 0.1:
                self._state = _S_HOLD_GOAL
                return self._goal_intent(world)
            if world.ball_dist <= self._gk_kick_distance:
                self._state = _S_CLEAR
                page = (
                    self._kick_right_page
                    if world.ball_bearing >= 0
                    else self._kick_left_page
                )
                return Intent(IntentType.CLEAR_BALL, kick_page=page, reason="kick_to_clear")
            return Intent(
                IntentType.INTERCEPT_BOX,
                target_dist=world.ball_dist,
                target_bearing=world.ball_bearing,
            )

        if self._state == _S_CLEAR:
            self._state = _S_HOLD_GOAL
            return self._goal_intent(world)

        return Intent(IntentType.STOP)
