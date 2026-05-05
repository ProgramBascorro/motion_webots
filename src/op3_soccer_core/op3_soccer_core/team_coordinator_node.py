"""Layer 5 — Team Coordinator Node.

Aggregates player snapshots from /team_comm/rx, arbitrates ball ownership,
and publishes a unified /team_coordinator/snapshot + /team_coordinator/ball_owner.
"""
import json
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3Stamped

from .config_loader import load_rules


# Lower ownership_score() = stronger claim on ball.
# score = ball_dist / role_weight  → higher weight = lower score for same dist.
# Striker should claim the ball first in open play; GK only when ball is in box.
ROLE_WEIGHT = {
    "striker":    4.0,   # strongest claim — primary attacker
    "support":    2.5,   # second attacker
    "defender":   1.5,   # defensive intercept
    "goalkeeper": 0.5,   # only claims when physically very close (in-box)
}


@dataclass
class PlayerSnapshot:
    player_id: int = 0
    team: int = 1
    role: str = "striker"
    penalized: bool = False
    fallen: bool = False
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0
    loc_confidence: float = 0.0
    ball_seen: bool = False
    ball_dist: float = 999.0
    ball_bearing: float = 0.0
    ball_confidence: float = 0.0
    committed: bool = False
    playbook: str = "none"
    stamp: float = field(default_factory=time.monotonic)

    def is_stale(self, staleness_sec: float) -> bool:
        return (time.monotonic() - self.stamp) > staleness_sec

    def ownership_score(self) -> float:
        """Lower score = better claim on ball."""
        if not self.ball_seen or self.penalized or self.fallen:
            return 9999.0
        weight = ROLE_WEIGHT.get(self.role, 1.0)
        return self.ball_dist * (1.0 - self.ball_confidence * 0.3) / weight


class TeamCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("team_coordinator_node")

        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._self_id = int(self.declare_parameter("player_id", 2).value)
        self._team = int(self.declare_parameter("team_number", 1).value)
        self._staleness_sec = float(self.declare_parameter(
            "snapshot_staleness_sec", rules.get("snapshot_staleness_sec", 2.0)).value)
        self._hysteresis = float(self.declare_parameter(
            "ball_owner_hysteresis", rules.get("ball_owner_hysteresis", 0.20)).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        # Local robot state (populated from other topics)
        self._local: PlayerSnapshot = PlayerSnapshot(
            player_id=self._self_id, team=self._team
        )
        self._ball_dist: float = 999.0
        self._ball_bearing: float = 0.0
        self._ball_visible: bool = False
        self._ball_confidence: float = 0.0

        # Dict of all known players (keyed by player_id)
        self._players: Dict[int, PlayerSnapshot] = {}

        # Ball owner tracking
        self._current_ball_owner: int = 0  # 0 = unknown
        self._current_owner_score: float = 9999.0

        # Subscriptions
        self.create_subscription(String, "/team_comm/rx", self._cb_rx, 10)
        self.create_subscription(Vector3Stamped, "/ball/estimate", self._cb_ball, 10)
        self.create_subscription(String, "/tactical/role", self._cb_role, 10)
        self.create_subscription(String, "/tactical/intent", self._cb_intent, 10)

        # Publishers
        self._snapshot_pub = self.create_publisher(String, "/team_coordinator/snapshot", 10)
        self._owner_pub = self.create_publisher(Int32, "/team_coordinator/ball_owner", 10)

        self.create_timer(1.0 / self._publish_rate, self._tick)
        self.get_logger().info(
            f"TeamCoordinator ready (self={self._self_id}, team={self._team})"
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_ball(self, msg: Vector3Stamped) -> None:
        self._ball_dist = msg.vector.x
        self._ball_visible = msg.vector.z > 0.5
        self._ball_bearing = msg.vector.y
        self._ball_confidence = msg.vector.z
        self._local.ball_dist = self._ball_dist
        self._local.ball_bearing = self._ball_bearing
        self._local.ball_seen = self._ball_visible
        self._local.ball_confidence = self._ball_confidence
        self._local.stamp = time.monotonic()

    def _cb_role(self, msg: String) -> None:
        self._local.role = msg.data.strip()

    def _cb_intent(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            # Bug 3 fix: use canonical IntentType.value strings
            self._local.committed = data.get("intent") in (
                "kick", "approach_ball", "align_to_kick", "dribble_out",
                "intercept_box", "intercept_threat",
            )
        except (json.JSONDecodeError, AttributeError):
            pass

    def _cb_rx(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        pid = int(data.get("player_id", -1))
        if pid <= 0 or pid == self._self_id:
            return
        if int(data.get("team", -1)) != self._team:
            return

        snap = PlayerSnapshot(
            player_id=pid,
            team=int(data.get("team", self._team)),
            role=str(data.get("role", "striker")),
            penalized=bool(data.get("penalized", False)),
            fallen=bool(data.get("fallen", False)),
            pose_x=float(data.get("pose_x", 0.0)),
            pose_y=float(data.get("pose_y", 0.0)),
            pose_yaw=float(data.get("pose_yaw", 0.0)),
            loc_confidence=float(data.get("loc_confidence", 0.0)),
            ball_seen=bool(data.get("ball_seen", False)),
            ball_dist=float(data.get("ball_dist", 999.0)),
            ball_bearing=float(data.get("ball_bearing", 0.0)),
            ball_confidence=float(data.get("ball_confidence", 0.0)),
            committed=bool(data.get("committed", False)),
            playbook=str(data.get("playbook", "none")),
            stamp=time.monotonic(),
        )
        self._players[pid] = snap

    # ------------------------------------------------------------------ #
    #  Arbitration + publish                                               #
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        now = time.monotonic()

        # Remove stale entries
        stale = [pid for pid, snap in self._players.items()
                 if snap.is_stale(self._staleness_sec)]
        for pid in stale:
            del self._players[pid]

        # Update local snapshot timestamp
        self._local.stamp = now

        # Build combined player dict (local + remotes)
        all_players: Dict[int, PlayerSnapshot] = {self._self_id: self._local}
        all_players.update(self._players)

        # Ball ownership arbitration
        best_id = 0
        best_score = 9999.0
        for pid, snap in all_players.items():
            s = snap.ownership_score()
            if s < best_score:
                best_score = s
                best_id = pid

        # Hysteresis: only switch owner if improvement exceeds threshold
        if self._current_ball_owner == 0 or self._current_ball_owner not in all_players:
            self._current_ball_owner = best_id
            self._current_owner_score = best_score
        else:
            if best_score < self._current_owner_score * (1.0 - self._hysteresis):
                self._current_ball_owner = best_id
                self._current_owner_score = best_score
            else:
                # Re-score current owner in case their data updated
                curr = all_players.get(self._current_ball_owner)
                if curr is not None:
                    self._current_owner_score = curr.ownership_score()

        # Publish snapshot
        snapshot_payload = {
            "ball_owner_id": self._current_ball_owner,
            "players": {
                str(pid): {
                    "player_id": snap.player_id,
                    "role": snap.role,
                    "penalized": snap.penalized,
                    "fallen": snap.fallen,
                    "ball_seen": snap.ball_seen,
                    "ball_dist": round(snap.ball_dist, 3),
                    "ball_bearing": round(snap.ball_bearing, 3),
                    "committed": snap.committed,
                    "playbook": snap.playbook,
                    "pose_x": round(snap.pose_x, 3),
                    "pose_y": round(snap.pose_y, 3),
                    "fresh": not snap.is_stale(self._staleness_sec),
                }
                for pid, snap in all_players.items()
            },
        }
        self._snapshot_pub.publish(String(data=json.dumps(snapshot_payload)))
        self._owner_pub.publish(Int32(data=self._current_ball_owner))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeamCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
