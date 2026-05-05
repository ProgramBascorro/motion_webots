"""Layer 6 — Planner Node.

Translates /tactical/intent → walk velocity via A* 8-direction + APF.
Falls back to bearing-only navigation when localization confidence is low.

Publishes directly to /motion/command (action=walk) for navigation intents.
Tactical node handles non-walk intents (kick, stop, search) to /motion/command.
"""
import json
import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, String

from .config_loader import load_rules
from .planning.astar_8dir import plan as astar_plan
from .planning.apf_local import blend as apf_blend

# Only these intents require A*+APF navigation; everything else is handled
# directly by tactical_node (kick, stop, search) or is a hold/position intent
# that tactical resolves with a stop command.
_NAVIGATE_INTENTS = frozenset({
    "approach_ball",
    "align_to_kick",
    "dribble_out",
    "intercept_threat",
    "intercept_box",
})


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")

        # Bug 6 fix: load tunable params from rules_file if provided
        rules_file = self.declare_parameter("rules_file", "").value
        rules = load_rules(rules_file)

        self._loc_min_confidence = float(
            self.declare_parameter(
                "planner_fallback_min_confidence",
                rules.get("planner_fallback_min_confidence", 0.30),
            ).value
        )
        self._replan_threshold_m = float(
            self.declare_parameter("astar_replan_threshold_m", 0.30).value
        )
        self._goal_reached_dist_m = float(
            self.declare_parameter("goal_reached_dist_m", 0.05).value
        )
        self._replan_debounce_sec = float(
            self.declare_parameter("astar_replan_debounce_sec", 0.5).value
        )
        self._publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        # Walk velocity limits — defaults match rc_hl_kidsize.yaml
        self._max_vx = float(self.declare_parameter(
            "walk_max_vx", rules.get("walk_max_vx", 0.060)).value)
        self._max_vy = float(self.declare_parameter(
            "walk_max_vy", rules.get("walk_max_vy", 0.040)).value)
        self._max_vyaw = float(self.declare_parameter(
            "walk_max_vyaw", rules.get("walk_max_vyaw", 0.40)).value)

        # APF parameters (override from rules_file if present)
        self._k_att = float(self.declare_parameter(
            "apf_k_att", rules.get("apf_k_att", 1.5)).value)
        self._k_rep = float(self.declare_parameter(
            "apf_k_rep", rules.get("apf_k_rep", 0.8)).value)
        self._influence_radius = float(self.declare_parameter(
            "apf_influence_radius_m", rules.get("apf_influence_radius_m", 0.5)).value)
        # Yaw-only threshold: if bearing > this, turn first before walking
        self._yaw_only_threshold = float(self.declare_parameter(
            "yaw_only_threshold", rules.get("yaw_only_threshold", 0.6)).value)

        # State
        self._intent: dict = {}
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        # Confidence from /localization/confidence topic (consistent with tactical)
        self._loc_confidence = 0.0
        self._obstacles: List[Tuple[float, float]] = []

        self._last_goal: Optional[Tuple[float, float]] = None
        self._last_replan_time = 0.0
        self._cached_astar_dir: Optional[Tuple[float, float]] = None

        # Subscriptions
        self.create_subscription(String, "/tactical/intent", self._cb_intent, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self._cb_pose, 10
        )
        # Subscribe to the same /localization/confidence used by tactical (not covariance-derived)
        self.create_subscription(
            Float32, "/localization/confidence", self._cb_confidence, 10
        )
        self.create_subscription(String, "/perception/obstacles", self._cb_obstacles, 10)

        # Bug 2 fix: publish directly to /motion/command so motion_node executes it
        self._cmd_pub = self.create_publisher(String, "/motion/command", 10)

        self.create_timer(1.0 / self._publish_rate, self._tick)
        self.get_logger().info("PlannerNode ready (A* + APF)")

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_intent(self, msg: String) -> None:
        try:
            self._intent = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny, cosy)

    def _cb_confidence(self, msg: Float32) -> None:
        # Use the same confidence value as tactical (consistent threshold behaviour)
        self._loc_confidence = float(msg.data)

    def _cb_obstacles(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._obstacles = [
                (float(o["x"]), float(o["y"])) for o in data
                if "x" in o and "y" in o
            ]
        except (json.JSONDecodeError, KeyError, TypeError):
            self._obstacles = []

    # ------------------------------------------------------------------ #
    #  Planning tick                                                       #
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        intent_type = self._intent.get("intent", "")
        # Only process the 5 pure-navigation intents; all others are handled by
        # tactical_node directly (kick, stop, search) or produce a stop.
        if intent_type not in _NAVIGATE_INTENTS:
            return

        goal_xy = self._resolve_goal(intent_type)
        if goal_xy is None:
            self._publish_stop("no_goal")
            return

        robot_xy = (self._pose_x, self._pose_y)
        dist_to_goal = math.hypot(
            goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
        )

        if dist_to_goal < self._goal_reached_dist_m:
            self._publish_stop("goal_reached")
            return

        # Choose navigation mode
        if self._loc_confidence < self._loc_min_confidence:
            # Fallback: bearing-only using ball/goal bearing from intent
            vx, vy, vyaw = self._bearing_only(intent_type)
        else:
            vx, vy, vyaw = self._astar_apf(robot_xy, goal_xy)

        cmd = {
            "action": "walk",
            "vx": round(_clamp(vx, -self._max_vx, self._max_vx), 4),
            "vy": round(_clamp(vy, -self._max_vy, self._max_vy), 4),
            "vyaw": round(_clamp(vyaw, -self._max_vyaw, self._max_vyaw), 4),
        }
        self._cmd_pub.publish(String(data=json.dumps(cmd)))

    def _publish_stop(self, reason: str) -> None:
        self._cmd_pub.publish(
            String(data=json.dumps({"action": "stop", "source": "planner", "reason": reason}))
        )

    def _resolve_goal(
        self, intent_type: str
    ) -> Optional[Tuple[float, float]]:
        """Convert intent to a world-frame goal position."""
        td = float(self._intent.get("target_dist", 0.0))
        tb = float(self._intent.get("target_bearing", 0.0))

        if intent_type in _NAVIGATE_INTENTS:
            # Goal = current pose + (dist, bearing) projected to world frame
            gx = self._pose_x + td * math.cos(self._pose_yaw + tb)
            gy = self._pose_y + td * math.sin(self._pose_yaw + tb)
            return (gx, gy)

        return None

    def _astar_apf(
        self,
        robot_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        now = time.monotonic()
        goal_changed = (
            self._last_goal is None
            or math.hypot(
                goal_xy[0] - self._last_goal[0],
                goal_xy[1] - self._last_goal[1],
            ) > self._replan_threshold_m
        )
        debounce_passed = (now - self._last_replan_time) > self._replan_debounce_sec

        if goal_changed and debounce_passed:
            self._cached_astar_dir = astar_plan(
                robot_xy, goal_xy, self._obstacles
            )
            self._last_goal = goal_xy
            self._last_replan_time = now

        astar_dir = self._cached_astar_dir
        if astar_dir is None:
            astar_dir = self._direct_dir(robot_xy, goal_xy)

        # APF modulation
        fx, fy = apf_blend(
            astar_dir, goal_xy, robot_xy, self._obstacles,
            k_att=self._k_att, k_rep=self._k_rep,
            influence_radius=self._influence_radius,
        )

        # Convert world-frame velocity to robot frame
        cos_yaw = math.cos(-self._pose_yaw)
        sin_yaw = math.sin(-self._pose_yaw)
        vx_robot = fx * cos_yaw - fy * sin_yaw
        vy_robot = fx * sin_yaw + fy * cos_yaw

        # Proportional scaling: normalise to max_vx (no hard-clamp at 1.0)
        speed = math.hypot(vx_robot, vy_robot)
        if speed > 1e-3:
            scale = self._max_vx / speed   # true proportional, not capped at 1.0
            vx_robot *= scale
            vy_robot *= scale

        # Yaw correction toward goal
        goal_bearing = math.atan2(
            goal_xy[1] - self._pose_y, goal_xy[0] - self._pose_x
        )
        yaw_err = goal_bearing - self._pose_yaw
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi
        vyaw = _clamp(1.0 * yaw_err, -self._max_vyaw, self._max_vyaw)

        # If heading error is large, suppress forward motion (turn-in-place first)
        if abs(yaw_err) > self._yaw_only_threshold:
            vx_robot = 0.0
            vy_robot = 0.0

        return (vx_robot, vy_robot, vyaw)

    def _bearing_only(
        self, intent_type: str
    ) -> Tuple[float, float, float]:
        """Bearing-only fallback when localization confidence is low.

        If bearing > yaw_only_threshold, turn in place first so the robot
        faces the target before walking (prevents sideways spiralling).
        """
        tb = float(self._intent.get("target_bearing", 0.0))
        td = float(self._intent.get("target_dist", 0.5))
        vyaw = _clamp(1.2 * tb, -self._max_vyaw, self._max_vyaw)
        # Only walk forward when roughly facing the target
        if abs(tb) > self._yaw_only_threshold:
            vx = 0.0   # turn-only: face target first
        else:
            vx = _clamp(self._max_vx * min(1.0, td / 0.5), 0.0, self._max_vx)
        return (vx, 0.0, vyaw)

    @staticmethod
    def _direct_dir(
        robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> Tuple[float, float]:
        dx = goal_xy[0] - robot_xy[0]
        dy = goal_xy[1] - robot_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return (1.0, 0.0)
        return (dx / dist, dy / dist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
