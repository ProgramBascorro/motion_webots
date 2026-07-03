#!/usr/bin/env python3
"""Perfect-tracking servo emulator for standalone-sim mode.

Without a physics sim (Webots/Gazebo), nothing publishes
/robotis_op3/joint_states, so manager_sim's robotis_controller deadlocks
waiting for feedback and walking_module's commanded angles never reach
the goal_joint_states stream. We synthesize the feedback ourselves.

Wire-up (matches robotis_controller's gazebo mode):

    robotis_controller (gazebo_mode_==true) publishes one Float64 per
    joint on /robotis_op3/<joint>_position/command (see
    robotis_controller.cpp ~L660).  We subscribe to all 20 of those,
    aggregate them into a JointState, and republish back as
    /robotis_op3/joint_states at 100 Hz. The latch is seeded with all
    zeros so the very first tick has data — that bootstrap is what gets
    the controller out of its "waiting for feedback" loop.

Equivalent to assuming the servos track perfectly with zero latency.
Good enough for previewing walking parameter shapes; doesn't model
dynamics.

Spawned by apply_node.py inside the sim ROS_DOMAIN_ID, so this never
touches the real robot's domain.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# Canonical OP3 joint ordering. Mirrors the dynamixel device list in
# op3_manager/config/OP3.robot (IDs 1-20). robotis_controller in sim
# mode advertises one position/command topic per joint name in this
# list.
_OP3_JOINTS = (
    "r_sho_pitch",
    "l_sho_pitch",
    "r_sho_roll",
    "l_sho_roll",
    "r_el",
    "l_el",
    "r_hip_yaw",
    "l_hip_yaw",
    "r_hip_roll",
    "l_hip_roll",
    "r_hip_pitch",
    "l_hip_pitch",
    "r_knee",
    "l_knee",
    "r_ank_pitch",
    "l_ank_pitch",
    "r_ank_roll",
    "l_ank_roll",
    "head_pan",
    "head_tilt",
)

_GAZEBO_ROBOT_NAME = "robotis_op3"
_TICK_PERIOD_S = 0.01  # 100 Hz


class SimFeedbackRelay(Node):
    def __init__(self) -> None:
        super().__init__("sim_feedback_relay")

        self._latch_pos = {name: 0.0 for name in _OP3_JOINTS}

        self._pub = self.create_publisher(
            JointState, "/robotis_op3/joint_states", 10
        )

        # One subscription per joint. The controller publishes Float64
        # on each of these in gazebo mode; we just stash the latest
        # value in the latch and let the timer fan it back out as a
        # single JointState.
        self._subs = []
        for name in _OP3_JOINTS:
            topic = f"/{_GAZEBO_ROBOT_NAME}/{name}_position/command"
            self._subs.append(
                self.create_subscription(
                    Float64, topic, self._make_cb(name), 10
                )
            )

        self._timer = self.create_timer(_TICK_PERIOD_S, self._tick)

        self.get_logger().info(
            "sim_feedback_relay ready: aggregating 20× "
            f"/{_GAZEBO_ROBOT_NAME}/<joint>_position/command into "
            "/robotis_op3/joint_states @ 100 Hz"
        )

    def _make_cb(self, name: str):
        def _cb(msg: Float64) -> None:
            try:
                self._latch_pos[name] = float(msg.data)
            except (TypeError, ValueError):
                pass
        return _cb

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_OP3_JOINTS)
        msg.position = [self._latch_pos[n] for n in _OP3_JOINTS]
        # Velocity/effort are unobserved in this perfect-tracking model
        # — leave them empty rather than fabricate zeros (a downstream
        # consumer can tell "unknown" from "stationary" that way).
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimFeedbackRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
