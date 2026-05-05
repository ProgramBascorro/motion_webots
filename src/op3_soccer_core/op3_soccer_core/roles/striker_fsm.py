"""Layer 4 — Striker FSM.
States: SEARCH → APPROACH → ALIGN → KICK → DRIBBLE_OUT → REPOSITION → FORCE_RELEASE → RECOVER
"""
from ..intent_types import Intent, IntentType
from ..world_model import WorldModel
from .base_role import BaseRole

_S_SEARCH = "SEARCH"
_S_APPROACH = "APPROACH"
_S_ALIGN = "ALIGN"
_S_KICK = "KICK"
_S_DRIBBLE_OUT = "DRIBBLE_OUT"
_S_REPOSITION = "REPOSITION"
_S_FORCE_RELEASE = "FORCE_RELEASE"
_S_RECOVER = "RECOVER"

_ALIGN_BEARING_THRESH = 0.15   # rad — fine alignment threshold before kick
_REPOSITION_DIST = 1.5         # m — after kick, move away before re-acquiring


class StrikerFSM(BaseRole):
    def __init__(
        self,
        kick_distance: float = 0.18,
        pre_kick_distance: float = 0.22,
        kick_left_page: int = 120,
        kick_right_page: int = 121,
        center_circle_radius: float = 0.75,
    ) -> None:
        self._state = _S_SEARCH
        self._kick_distance = kick_distance
        self._pre_kick_distance = pre_kick_distance
        self._kick_left_page = kick_left_page
        self._kick_right_page = kick_right_page
        self._center_circle_radius = center_circle_radius

    @property
    def state_name(self) -> str:
        return self._state

    def update(self, world: WorldModel) -> Intent:
        # Safety always wins
        if world.safety_veto:
            self._state = _S_RECOVER
            return Intent(IntentType.RECOVER, reason="safety_veto")

        # Must be playing and not penalized
        if not world.is_playing or world.is_penalized:
            self._state = _S_SEARCH
            return Intent(IntentType.STOP, reason="not_playing")

        # Force release — compliance detected ball-hold violation
        if (
            self._state != _S_FORCE_RELEASE
            and not world.compliance_allowed
            and world.compliance_substitute == "force_release"
        ):
            self._state = _S_FORCE_RELEASE

        return self._step(world)

    def _step(self, world: WorldModel) -> Intent:
        if self._state == _S_RECOVER:
            if not world.safety_veto:
                self._state = _S_SEARCH
            return Intent(IntentType.RECOVER, reason="recovering")

        if self._state == _S_SEARCH:
            if world.ball_visible:
                self._state = _S_APPROACH
                return self._step(world)   # immediate intent from new state
            return Intent(IntentType.SEARCH, reason="no_ball")

        if self._state == _S_APPROACH:
            if not world.ball_visible:
                self._state = _S_SEARCH
                return Intent(IntentType.SEARCH, reason="ball_lost")
            if world.ball_dist <= self._pre_kick_distance:
                self._state = _S_ALIGN
                return self._step(world)   # immediate align intent
            return Intent(
                IntentType.APPROACH_BALL,
                target_dist=world.ball_dist,
                target_bearing=world.ball_bearing,
            )

        if self._state == _S_ALIGN:
            if not world.ball_visible:
                self._state = _S_SEARCH
                return Intent(IntentType.SEARCH, reason="ball_lost_align")
            if world.ball_dist > self._pre_kick_distance + 0.05:
                self._state = _S_APPROACH
                return Intent(IntentType.APPROACH_BALL,
                              target_dist=world.ball_dist,
                              target_bearing=world.ball_bearing)

            bearing_ok = abs(world.ball_bearing) <= _ALIGN_BEARING_THRESH
            dist_ok = world.ball_dist <= self._kick_distance

            if bearing_ok and dist_ok:
                if not world.compliance_allowed:
                    if world.compliance_substitute == "dribble_out":
                        self._state = _S_DRIBBLE_OUT
                        return Intent(IntentType.DRIBBLE_OUT, reason="kickoff_guard")
                    if world.compliance_substitute == "force_release":
                        self._state = _S_FORCE_RELEASE
                        return Intent(IntentType.FORCE_RELEASE, reason="hold_limit")
                    return Intent(IntentType.STOP, reason=world.compliance_reason)

                # Kickoff tap before committing a full kick
                if world.kickoff_active and not world.kickoff_tap_done:
                    return Intent(IntentType.KICKOFF_TAP, reason="kickoff_tap")

                self._state = _S_KICK
                page = (
                    self._kick_right_page
                    if world.ball_bearing >= 0
                    else self._kick_left_page
                )
                return Intent(IntentType.KICK, kick_page=page, reason="aligned")

            return Intent(
                IntentType.ALIGN_TO_KICK,
                target_dist=world.ball_dist,
                target_bearing=world.ball_bearing,
            )

        if self._state == _S_KICK:
            # Kick action is triggered once; next tick we reposition
            self._state = _S_REPOSITION
            return Intent(IntentType.REPOSITION, reason="post_kick")

        if self._state == _S_DRIBBLE_OUT:
            # Exit when compliance allows kicking again:
            # - kickoff_tap_done: compliance confirmed tap executed
            # - kickoff_active expired: kickoff window (15s) elapsed
            # NOTE: world.ball_dist is robot-to-ball, NOT ball-to-center — cannot
            # use it to check if ball exited the center circle.
            if world.kickoff_tap_done or not world.kickoff_active:
                self._state = _S_ALIGN
                return self._step(world)   # immediate align intent
            return Intent(IntentType.DRIBBLE_OUT,
                          target_dist=world.ball_dist,
                          target_bearing=world.ball_bearing)

        if self._state == _S_REPOSITION:
            if not world.ball_visible or world.ball_dist > _REPOSITION_DIST:
                self._state = _S_SEARCH
                return self._step(world)   # immediate search/approach intent
            return Intent(IntentType.REPOSITION, reason="repositioning")

        if self._state == _S_FORCE_RELEASE:
            # Back off until compliance clears
            if world.compliance_allowed:
                self._state = _S_APPROACH
                return Intent(IntentType.APPROACH_BALL,
                              target_dist=world.ball_dist,
                              target_bearing=world.ball_bearing)
            return Intent(IntentType.FORCE_RELEASE, reason="backing_off")

        return Intent(IntentType.STOP)
