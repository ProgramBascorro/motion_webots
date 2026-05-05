"""Layer 4 — Tactical Node.

Runs the active role FSM at 10 Hz, builds WorldModel from latest
subscriptions, translates Intent → /motion/command JSON.

Delegation contract:
- Navigation intents (APPROACH_BALL, ALIGN_TO_KICK, GUARD_HALF, etc.) are
  handled by the planner (Layer 6), which publishes to /motion/command.
- Direct-action intents (KICK, STOP, SEARCH, KICKOFF_TAP, etc.) are handled
  here and published directly to /motion/command.
- NEVER both publish to /motion/command for the same intent in the same tick.

Role assignment (from player_number param):
  1 = goalkeeper, 2 = striker, 3 = support, 4 = defender
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32, String

from soccer_msgs.msg import GameState

from .config_loader import load_rules
from .intent_types import Intent, IntentType
from .roles.defender_fsm import DefenderFSM
from .roles.goalkeeper_fsm import GoalkeeperFSM
from .roles.striker_fsm import StrikerFSM
from .roles.support_fsm import SupportFSM
from .world_model import WorldModel

# Intents delegated to planner (Layer 6) — pure navigation only.
# Holding intents (HOLD_GOAL, GUARD_HALF, etc.) are NOT delegated: tactical
# sends stop directly so the robot never walks when planner has no path ready.
# CLEAR_BALL is a direct kick, not navigation — also not delegated.
_PLANNER_INTENTS = frozenset({
    IntentType.APPROACH_BALL,
    IntentType.ALIGN_TO_KICK,
    IntentType.INTERCEPT_THREAT,
    IntentType.INTERCEPT_BOX,
    IntentType.DRIBBLE_OUT,
})

# How old compliance / safety data can be before we default-block
_COMPLIANCE_STALE_SEC = 0.3
_SAFETY_STALE_SEC = 0.5


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class TacticalNode(Node):
    def __init__(self) -> None:
        super().__init__("tactical_node")

        # --- Rules file (rc_hl_kidsize.yaml) ---
        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        # --- Parameters (defaults fall back to rules_file values) ---
        self.declare_parameter("player_number", 2)
        self.declare_parameter("team_number", 1)
        self.declare_parameter("kick_distance", rules.get("kick_distance", 0.18))
        self.declare_parameter("pre_kick_distance", rules.get("pre_kick_distance", 0.22))
        self.declare_parameter("kick_left_page", rules.get("kick_left_page", 120))
        self.declare_parameter("kick_right_page", rules.get("kick_right_page", 121))
        self.declare_parameter("gk_chase_distance", rules.get("gk_chase_distance", 1.0))
        self.declare_parameter("defender_engage_distance", rules.get("defender_engage_distance", 1.2))
        self.declare_parameter("support_takeover_distance", rules.get("support_takeover_distance", 1.0))
        self.declare_parameter("center_circle_radius", rules.get("center_circle_radius_m", 0.75))
        # Motion tuning — read from rules so YAML changes take effect
        self._search_yaw = float(self.declare_parameter(
            "search_yaw", rules.get("search_yaw", 0.8)).value)
        self._yaw_only_threshold = float(self.declare_parameter(
            "yaw_only_threshold", rules.get("yaw_only_threshold", 0.6)).value)
        self._walk_max_vx = float(self.declare_parameter(
            "walk_max_vx", rules.get("walk_max_vx", 0.06)).value)
        self._walk_max_vyaw = float(self.declare_parameter(
            "walk_max_vyaw", rules.get("walk_max_vyaw", 0.40)).value)

        self._player_number: int = self.get_parameter("player_number").value
        self._team_number: int = self.get_parameter("team_number").value

        # --- Role FSMs ---
        kick_left = self.get_parameter("kick_left_page").value
        kick_right = self.get_parameter("kick_right_page").value
        kick_dist = self.get_parameter("kick_distance").value
        pre_kick = self.get_parameter("pre_kick_distance").value

        self._striker_fsm = StrikerFSM(
            kick_distance=kick_dist,
            pre_kick_distance=pre_kick,
            kick_left_page=kick_left,
            kick_right_page=kick_right,
            center_circle_radius=self.get_parameter("center_circle_radius").value,
        )
        # GK goal-post position: team 1 defends left goal (x=-4.5), team 2 right (x=+4.5)
        # GK stands 0.5m in front of goal line
        gk_goal_x = -4.0 if self._team_number == 1 else 4.0
        self._goalkeeper_fsm = GoalkeeperFSM(
            gk_chase_distance=self.get_parameter("gk_chase_distance").value,
            gk_kick_distance=kick_dist,
            kick_left_page=kick_left,
            kick_right_page=kick_right,
            goal_x=gk_goal_x,
            goal_y=0.0,
        )
        self._support_fsm = SupportFSM(
            support_takeover_distance=self.get_parameter("support_takeover_distance").value,
            kick_distance=kick_dist,
            pre_kick_distance=pre_kick,
            kick_left_page=kick_left,
            kick_right_page=kick_right,
            center_circle_radius=self.get_parameter("center_circle_radius").value,
        )
        self._defender_fsm = DefenderFSM(
            defender_engage_distance=self.get_parameter("defender_engage_distance").value,
            kick_distance=kick_dist,
            kick_left_page=kick_left,
            kick_right_page=kick_right,
        )

        self._role_map = {
            1: ("goalkeeper", self._goalkeeper_fsm),
            2: ("striker", self._striker_fsm),
            3: ("support", self._support_fsm),
            4: ("defender", self._defender_fsm),
        }
        role_name, self._active_fsm = self._role_map.get(
            self._player_number, ("striker", self._striker_fsm)
        )
        self._role_name: str = role_name

        # --- Latest subscription snapshots ---
        self._game_state: GameState = GameState()
        self._is_kickoff_for_us: bool = False
        self._is_setplay_for_us: bool = False
        self._gc_penalized: bool = False
        self._ball_dist: float = 999.0
        self._ball_bearing: float = 0.0
        self._ball_visible: bool = False
        self._ball_hold_sec: float = 0.0
        self._compliance: dict = {
            "allowed": True, "reason": "ok", "substitute": None,
            "ball_hold_sec": 0.0, "kickoff_active": False, "kickoff_tap_done": False,
        }
        # Initialize to now so the first tick is not immediately stale.
        # Compliance node sends at 10Hz — within 100ms the real stamp is set.
        self._compliance_stamp: float = time.monotonic()
        self._safety_veto: bool = False
        self._safety_stamp: float = time.monotonic()
        self._imu_pitch: float = 0.0
        self._imu_roll: float = 0.0
        self._pose_x: float = 0.0
        self._pose_y: float = 0.0
        self._pose_yaw: float = 0.0
        self._loc_confidence: float = 0.0
        self._team_snapshot: dict = {}
        self._ball_owner_id: int = -1
        self._motion_ready: bool = False
        self._kickoff_tap_done: bool = False

        # --- Subscribers ---
        # Bug 1 fix: subscribe as GameState msg, not String
        self.create_subscription(GameState, "/game_state", self._cb_game_state, 10)
        # Bug 7 fix: consume /game_context for kickoff/setplay/penalized context
        self.create_subscription(String, "/game_context", self._cb_game_context, 10)
        self.create_subscription(Vector3Stamped, "/ball/estimate", self._cb_ball, 10)
        self.create_subscription(String, "/compliance/status", self._cb_compliance, 10)
        self.create_subscription(Bool, "/safety/motion_veto", self._cb_safety, 10)
        self.create_subscription(String, "/team_coordinator/snapshot", self._cb_snapshot, 10)
        # Bug 4 fix: subscribe directly to ball_owner topic
        self.create_subscription(Int32, "/team_coordinator/ball_owner", self._cb_ball_owner, 10)
        self.create_subscription(String, "/team_comm/rx", self._cb_team_rx, 10)
        self.create_subscription(String, "/motion/status", self._cb_motion_status, 10)
        # Bug 5 fix: subscribe to /localization/confidence
        self.create_subscription(Float32, "/localization/confidence", self._cb_loc_confidence, 10)
        # Subscribe to localization pose (field frame, better than raw /odom)
        self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self._cb_pose, 10
        )
        # IMU for imu_pitch/roll in WorldModel
        self.create_subscription(Imu, "/robotis_op3/imu", self._cb_imu, 10)

        # --- Publishers ---
        self._pub_intent = self.create_publisher(String, "/tactical/intent", 10)
        self._pub_role = self.create_publisher(String, "/tactical/role", 10)
        self._pub_motion = self.create_publisher(String, "/motion/command", 10)
        self._pub_team_tx = self.create_publisher(String, "/team_comm/tx", 10)

        # --- 10 Hz control loop ---
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"TacticalNode started: player={self._player_number} "
            f"team={self._team_number} role={self._role_name}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_game_state(self, msg: GameState) -> None:
        self._game_state = msg

    def _cb_game_context(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._is_kickoff_for_us = bool(data.get("is_kickoff_for_us", False))
            self._is_setplay_for_us = bool(data.get("is_setplay_for_us", False))
            self._gc_penalized = bool(data.get("is_penalized", False))
        except json.JSONDecodeError:
            pass

    def _cb_ball(self, msg: Vector3Stamped) -> None:
        self._ball_dist = float(msg.vector.x)
        self._ball_bearing = float(msg.vector.y)
        self._ball_visible = bool(msg.vector.z > 0.5)

    def _cb_compliance(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._compliance = data
            self._compliance_stamp = time.monotonic()
            self._ball_hold_sec = float(data.get("ball_hold_sec", 0.0))
            # Bug 8 fix: read kickoff_tap_done from compliance status
            self._kickoff_tap_done = bool(data.get("kickoff_tap_done", False))
        except (json.JSONDecodeError, KeyError):
            pass

    def _cb_safety(self, msg: Bool) -> None:
        self._safety_veto = msg.data
        self._safety_stamp = time.monotonic()

    def _cb_snapshot(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._team_snapshot = data.get("players", {})
            # Also read ball_owner_id from snapshot (fallback if /ball_owner topic is slow)
            owner = data.get("ball_owner_id", -1)
            if self._ball_owner_id == -1 and owner > 0:
                self._ball_owner_id = owner
        except json.JSONDecodeError:
            pass

    def _cb_ball_owner(self, msg: Int32) -> None:
        # Bug 4 fix: direct update from team coordinator
        self._ball_owner_id = msg.data

    def _cb_team_rx(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
            player_id = int(packet.get("player_id", -1))
            if player_id > 0:
                self._team_snapshot[player_id] = packet
        except (json.JSONDecodeError, ValueError, KeyError):
            pass

    def _cb_motion_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._motion_ready = data.get("baseline_ready", False)
        except json.JSONDecodeError:
            pass

    def _cb_loc_confidence(self, msg: Float32) -> None:
        # Bug 5 fix: update loc_confidence from localization node
        self._loc_confidence = float(msg.data)

    def _cb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cb_imu(self, msg: Imu) -> None:
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self._imu_pitch = math.asin(sinp)

    # ------------------------------------------------------------------
    # Control tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()

        compliance_stale = (now - self._compliance_stamp) > _COMPLIANCE_STALE_SEC
        safety_stale = (now - self._safety_stamp) > _SAFETY_STALE_SEC

        gs = self._game_state
        is_playing = (gs.gamestate == GameState.GAMESTATE_PLAYING)
        is_penalized = (gs.penalty != GameState.PENALTY_NONE) or self._gc_penalized
        has_kick_off = gs.has_kick_off
        is_setplay = gs.secondary_state in (
            GameState.STATE_DIRECT_FREEKICK, GameState.STATE_INDIRECT_FREEKICK,
            GameState.STATE_PENALTYKICK, GameState.STATE_CORNER_KICK,
            GameState.STATE_GOAL_KICK, GameState.STATE_THROW_IN,
        )

        world = WorldModel(
            game_state_id=int(gs.gamestate),
            secondary_state_id=int(gs.secondary_state),
            is_playing=is_playing,
            is_penalized=is_penalized,
            has_kick_off=has_kick_off,
            is_kickoff_for_us=self._is_kickoff_for_us,
            # Use compliance window so kickoff_active expires after kickoff_window_sec
            kickoff_active=(
                self._compliance.get("kickoff_active", self._is_kickoff_for_us)
                if not compliance_stale else self._is_kickoff_for_us
            ),
            kickoff_tap_done=self._kickoff_tap_done,
            is_setplay_active=is_setplay,
            is_setplay_for_us=self._is_setplay_for_us,
            ball_dist=self._ball_dist,
            ball_bearing=self._ball_bearing,
            ball_visible=self._ball_visible,
            ball_hold_sec=self._ball_hold_sec,
            imu_pitch=self._imu_pitch,
            imu_roll=self._imu_roll,
            pose_x=self._pose_x,
            pose_y=self._pose_y,
            pose_yaw=self._pose_yaw,
            loc_confidence=self._loc_confidence,
            compliance_allowed=(False if compliance_stale
                                else self._compliance.get("allowed", True)),
            compliance_reason=("stale" if compliance_stale
                               else self._compliance.get("reason", "ok")),
            compliance_substitute=(None if compliance_stale
                                   else self._compliance.get("substitute")),
            safety_veto=(self._safety_veto if not safety_stale else False),
            team_snapshot=self._team_snapshot,
            ball_owner_id=self._ball_owner_id,
            motion_baseline_ready=self._motion_ready,
            now=now,
        )

        intent: Intent = self._active_fsm.update(world)

        # Publish role and intent for monitoring / planner consumption
        self._pub_role.publish(String(data=self._role_name))
        intent_dict = intent.to_dict()
        intent_dict["role"] = self._role_name
        intent_dict["fsm_state"] = self._active_fsm.state_name
        self._pub_intent.publish(String(data=json.dumps(intent_dict)))

        # Bug 8 fix: include kickoff_tap_done in team broadcast
        broadcast = {
            "player_id": self._player_number,
            "team": self._team_number,
            "role": self._role_name,
            "penalized": world.is_penalized,
            "fallen": world.safety_veto,
            "ball_seen": world.ball_visible,
            "ball_dist": round(world.ball_dist, 3),
            "ball_bearing": round(world.ball_bearing, 3),
            "ball_confidence": 1.0 if world.ball_visible else 0.0,
            # Bug 3 fix: use correct IntentType.value names
            "committed": intent.intent_type in (
                IntentType.APPROACH_BALL, IntentType.ALIGN_TO_KICK,
                IntentType.KICK, IntentType.DRIBBLE_OUT,
                IntentType.INTERCEPT_BOX, IntentType.INTERCEPT_THREAT,
            ),
            "kickoff_tap_done": world.kickoff_tap_done,
            "playbook": "none",
            "pose_x": round(world.pose_x, 3),
            "pose_y": round(world.pose_y, 3),
            "pose_yaw": round(world.pose_yaw, 3),
            "loc_confidence": round(world.loc_confidence, 2),
            "stamp": round(now, 2),
        }
        self._pub_team_tx.publish(String(data=json.dumps(broadcast)))

        # Bug 9 fix: only publish motion cmd for non-planner intents
        cmd = self._intent_to_motion(intent, world)
        if cmd is not None:
            self._pub_motion.publish(String(data=json.dumps(cmd)))

    # ------------------------------------------------------------------
    # Intent → motion command translation (direct-action intents only)
    # Bug 9: Planner-handled intents return None — planner publishes to /motion/command
    # ------------------------------------------------------------------

    def _intent_to_motion(self, intent: Intent, world: WorldModel):
        t = intent.intent_type

        # Delegated to planner (Layer 6) — do NOT publish motion command here
        if t in _PLANNER_INTENTS:
            return None

        if t == IntentType.STOP:
            return {"action": "stop"}

        if t == IntentType.RECOVER:
            return {"action": "stop"}

        if t == IntentType.SEARCH:
            # Rotate in place to scan for ball — rate from rules (search_yaw)
            return {"action": "walk", "vx": 0.0, "vy": 0.0,
                    "vyaw": round(self._search_yaw, 3)}

        if t in (IntentType.KICK, IntentType.CLEAR_TO_SUPPORT, IntentType.CLEAR_BALL):
            return {"action": "kick", "page": intent.kick_page}

        if t == IntentType.KICKOFF_TAP:
            return {"action": "walk", "vx": 0.04, "vy": 0.0, "vyaw": 0.0}

        if t == IntentType.FORCE_RELEASE:
            return {"action": "walk", "vx": -0.04, "vy": 0.0, "vyaw": 0.0}

        if t == IntentType.HOLD_GOAL:
            # GK: navigate toward goal centre if not already there
            if intent.target_dist > 0.15:
                vx = min(self._walk_max_vx * 0.7, intent.target_dist * 0.25)
                # If bearing is large, turn first (yaw_only_threshold)
                if abs(intent.target_bearing) > self._yaw_only_threshold:
                    vx = 0.0
                vyaw = max(-self._walk_max_vyaw,
                           min(self._walk_max_vyaw, 1.0 * intent.target_bearing))
                return {"action": "walk", "vx": round(vx, 4), "vy": 0.0,
                        "vyaw": round(vyaw, 4)}
            return {"action": "stop"}

        if t in (IntentType.GUARD_HALF, IntentType.SUPPORT_LANE,
                 IntentType.RECEIVE_PASS, IntentType.REPOSITION):
            # Defensive positioning — stop until planner goal-zone is ready
            return {"action": "stop"}

        if t == IntentType.TAKEOVER_STRIKER:
            # Transition frame — stop until FSM settles to APPROACH_BALL next tick
            return {"action": "stop"}

        # Default: stop for any unhandled intent
        return {"action": "stop"}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TacticalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
