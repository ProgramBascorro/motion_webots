"""Layer 4 — Intent types shared by all role FSMs and tactical_node."""
from dataclasses import dataclass, field
from enum import Enum


class IntentType(Enum):
    STOP = "stop"
    SEARCH = "search"
    APPROACH_BALL = "approach_ball"
    ALIGN_TO_KICK = "align_to_kick"
    KICK = "kick"
    DRIBBLE_OUT = "dribble_out"
    FORCE_RELEASE = "force_release"
    HOLD_GOAL = "hold_goal"
    INTERCEPT_BOX = "intercept_box"
    CLEAR_BALL = "clear_ball"
    SUPPORT_LANE = "support_lane"
    RECEIVE_PASS = "receive_pass"
    TAKEOVER_STRIKER = "takeover_striker"
    GUARD_HALF = "guard_half"
    INTERCEPT_THREAT = "intercept_threat"
    CLEAR_TO_SUPPORT = "clear_to_support"
    RECOVER = "recover"
    REPOSITION = "reposition"
    KICKOFF_TAP = "kickoff_tap"


@dataclass
class Intent:
    intent_type: IntentType = IntentType.STOP
    target_dist: float = 0.0
    target_bearing: float = 0.0
    kick_page: int = -1
    reason: str = ""

    def to_dict(self) -> dict:
        return {
            "intent": self.intent_type.value,
            "target_dist": round(self.target_dist, 3),
            "target_bearing": round(self.target_bearing, 3),
            "kick_page": self.kick_page,
            "reason": self.reason,
        }
