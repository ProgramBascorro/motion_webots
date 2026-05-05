"""Layer 4 — Support FSM.
States: SUPPORT_LANE → SECOND_BALL → TAKEOVER_STRIKER → RECEIVE_PASS → RECOVER

When taking over striker role, delegates to StrikerFSM to avoid duplicating
approach/align/kick logic.
"""
from ..intent_types import Intent, IntentType
from ..world_model import WorldModel
from .base_role import BaseRole
from .striker_fsm import StrikerFSM

_S_SUPPORT_LANE = "SUPPORT_LANE"
_S_SECOND_BALL = "SECOND_BALL"
_S_TAKEOVER = "TAKEOVER_STRIKER"
_S_RECEIVE_PASS = "RECEIVE_PASS"
_S_RECOVER = "RECOVER"


class SupportFSM(BaseRole):
    def __init__(
        self,
        support_takeover_distance: float = 1.0,
        kick_distance: float = 0.18,
        pre_kick_distance: float = 0.22,
        kick_left_page: int = 120,
        kick_right_page: int = 121,
        center_circle_radius: float = 0.75,
    ) -> None:
        self._state = _S_SUPPORT_LANE
        self._support_takeover_distance = support_takeover_distance
        # Striker FSM reused when taking over — full approach/align/kick pipeline
        self._striker_fsm = StrikerFSM(
            kick_distance=kick_distance,
            pre_kick_distance=pre_kick_distance,
            kick_left_page=kick_left_page,
            kick_right_page=kick_right_page,
            center_circle_radius=center_circle_radius,
        )

    @property
    def state_name(self) -> str:
        return self._state

    def update(self, world: WorldModel) -> Intent:
        if world.safety_veto:
            self._state = _S_RECOVER
            return Intent(IntentType.RECOVER, reason="safety_veto")

        if not world.is_playing or world.is_penalized:
            self._state = _S_SUPPORT_LANE
            return Intent(IntentType.STOP, reason="not_playing")

        return self._step(world)

    def _step(self, world: WorldModel) -> Intent:
        if self._state == _S_RECOVER:
            if not world.safety_veto:
                self._state = _S_SUPPORT_LANE
            return Intent(IntentType.RECOVER, reason="recovering")

        striker_absent = self._is_striker_absent(world)

        if self._state == _S_SUPPORT_LANE:
            if striker_absent and world.ball_visible and world.ball_dist < self._support_takeover_distance:
                self._state = _S_TAKEOVER
                return Intent(IntentType.TAKEOVER_STRIKER, reason="striker_absent")
            return Intent(IntentType.SUPPORT_LANE, reason="holding_lane")

        if self._state == _S_TAKEOVER:
            if not striker_absent:
                self._state = _S_SUPPORT_LANE
                self._striker_fsm._state = "SEARCH"   # reset striker FSM for next use
                return Intent(IntentType.SUPPORT_LANE, reason="striker_returned")
            # Delegate fully to StrikerFSM (approach + align + kick)
            return self._striker_fsm.update(world)

        if self._state == _S_RECEIVE_PASS:
            if world.ball_visible and world.ball_dist < self._support_takeover_distance:
                self._state = _S_TAKEOVER
                return self._step(world)   # immediate takeover intent
            return Intent(IntentType.RECEIVE_PASS, reason="awaiting_pass")

        if self._state == _S_SECOND_BALL:
            if striker_absent:
                self._state = _S_TAKEOVER
                return self._step(world)   # immediate takeover intent
            return Intent(IntentType.SUPPORT_LANE, reason="second_ball_shadow")

        return Intent(IntentType.STOP)

    def _is_striker_absent(self, world: WorldModel) -> bool:
        for data in world.team_snapshot.values():
            if data.get("role") == "striker":
                if data.get("penalized") or data.get("fallen"):
                    return True
                return False
        return True
