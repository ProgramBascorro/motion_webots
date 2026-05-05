"""Layer 4 — Defender FSM.
States: GUARD_HALF → INTERCEPT_THREAT → CLEAR_TO_SUPPORT → RECOVER
"""
from ..intent_types import Intent, IntentType
from ..world_model import WorldModel
from .base_role import BaseRole

_S_GUARD_HALF = "GUARD_HALF"
_S_INTERCEPT = "INTERCEPT_THREAT"
_S_CLEAR = "CLEAR_TO_SUPPORT"
_S_RECOVER = "RECOVER"


class DefenderFSM(BaseRole):
    def __init__(
        self,
        defender_engage_distance: float = 1.2,
        kick_distance: float = 0.18,
        kick_left_page: int = 120,
        kick_right_page: int = 121,
    ) -> None:
        self._state = _S_GUARD_HALF
        self._defender_engage_distance = defender_engage_distance
        self._kick_distance = kick_distance
        self._kick_left_page = kick_left_page
        self._kick_right_page = kick_right_page

    @property
    def state_name(self) -> str:
        return self._state

    def update(self, world: WorldModel) -> Intent:
        if world.safety_veto:
            self._state = _S_RECOVER
            return Intent(IntentType.RECOVER, reason="safety_veto")

        if not world.is_playing or world.is_penalized:
            self._state = _S_GUARD_HALF
            return Intent(IntentType.STOP, reason="not_playing")

        # If GK is incapacitated, shift to GK zone
        if self._is_gk_incapacitated(world):
            return Intent(IntentType.GUARD_HALF, reason="covering_gk")

        return self._step(world)

    def _step(self, world: WorldModel) -> Intent:
        if self._state == _S_RECOVER:
            if not world.safety_veto:
                self._state = _S_GUARD_HALF
            return Intent(IntentType.RECOVER, reason="recovering")

        if self._state == _S_GUARD_HALF:
            if world.ball_visible and world.ball_dist < self._defender_engage_distance:
                self._state = _S_INTERCEPT
                return self._step(world)   # immediate intent from new state
            return Intent(IntentType.GUARD_HALF, reason="holding_line")

        if self._state == _S_INTERCEPT:
            if not world.ball_visible:
                self._state = _S_GUARD_HALF
                return Intent(IntentType.GUARD_HALF, reason="ball_lost")
            if world.ball_dist <= self._kick_distance:
                self._state = _S_CLEAR
                page = (
                    self._kick_right_page
                    if world.ball_bearing >= 0
                    else self._kick_left_page
                )
                return Intent(IntentType.CLEAR_TO_SUPPORT, kick_page=page, reason="clearing")
            return Intent(
                IntentType.INTERCEPT_THREAT,
                target_dist=world.ball_dist,
                target_bearing=world.ball_bearing,
            )

        if self._state == _S_CLEAR:
            self._state = _S_GUARD_HALF
            return Intent(IntentType.GUARD_HALF, reason="post_clear")

        return Intent(IntentType.STOP)

    def _is_gk_incapacitated(self, world: WorldModel) -> bool:
        for data in world.team_snapshot.values():
            if data.get("role") == "goalkeeper":
                return bool(data.get("penalized") or data.get("fallen"))
        return False
