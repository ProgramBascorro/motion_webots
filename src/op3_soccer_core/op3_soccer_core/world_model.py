"""Layer 4 — WorldModel dataclass.
Single snapshot passed to every role FSM each control tick.
All sensor/state data comes from tactical_node subscriptions.
"""
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class WorldModel:
    # Game state
    game_state_id: int = 0          # GameState.GAMESTATE_*
    secondary_state_id: int = 0     # GameState.STATE_*
    is_playing: bool = False
    is_penalized: bool = False
    has_kick_off: bool = False
    is_kickoff_for_us: bool = False
    kickoff_active: bool = False
    kickoff_tap_done: bool = False
    is_setplay_active: bool = False
    is_setplay_for_us: bool = False

    # Ball
    ball_dist: float = 999.0
    ball_bearing: float = 0.0
    ball_visible: bool = False
    ball_hold_sec: float = 0.0

    # IMU / body
    imu_pitch: float = 0.0
    imu_roll: float = 0.0

    # Odometry / localization
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0
    loc_confidence: float = 0.0

    # Compliance (from /compliance/status)
    compliance_allowed: bool = True
    compliance_reason: str = "ok"
    compliance_substitute: Optional[str] = None

    # Safety
    safety_veto: bool = False

    # Team snapshot (from /team_coordinator/snapshot or /team_comm/rx)
    team_snapshot: Dict[int, dict] = field(default_factory=dict)
    ball_owner_id: int = -1

    # Motion readiness
    motion_baseline_ready: bool = False

    # Timestamp
    now: float = 0.0
