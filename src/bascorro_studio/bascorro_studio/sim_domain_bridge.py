#!/usr/bin/env python3
"""Cross-domain relay between the standalone-sim ROS_DOMAIN_ID and the
real-robot domain.

Implemented as two child processes (one per ROS_DOMAIN_ID) talking through
``multiprocessing.Queue`` with the ``spawn`` start method, so each child
gets a fresh interpreter and rclpy.init() reads the ROS_DOMAIN_ID we set
on its env right before importing rclpy. An earlier in-process variant
used ``rclpy.init(domain_id=…)`` with two ``rclpy.Context`` instances;
on Humble that init param has been observed to silently fall back to the
inherited env, which leaks both contexts onto the same domain and makes
the relay invisible to the real-domain rosbridge. The two-process layout
removes that ambiguity — each side genuinely runs in its own DOMAIN.

Relays:
    sim DOMAIN  -> real DOMAIN :
        /robotis_op3/joint_states
            -> /bascorro_studio/sim_preview/joint_states
    real DOMAIN -> sim DOMAIN :
        /bascorro_studio/sim/enable_ctrl_module -> /robotis/enable_ctrl_module
        /bascorro_studio/sim/walking/command    -> /robotis/walking/command
        /bascorro_studio/sim/walking/set_params -> /robotis/walking/set_params
        /bascorro_studio/sim/action/page_num    -> /robotis/action/page_num
"""

import multiprocessing as mp
import os
import sys
import time


SIM_DOMAIN_ID = os.environ.get("BASCORRO_SIM_DOMAIN_ID", "42")
REAL_DOMAIN_ID = os.environ.get("BASCORRO_REAL_DOMAIN_ID", "0")


def _serialize_walking_param(msg) -> dict:
    """Flatten a WalkingParam message into a dict of primitive fields so
    it can cross a multiprocessing.Queue. rclpy.Message instances are
    picklable in principle, but we want the receiver to construct a
    *fresh* message bound to its own context, not unpack the sender's."""
    out = {}
    for slot in getattr(msg, "__slots__", ()):
        field = slot.lstrip("_")
        try:
            val = getattr(msg, slot)
        except AttributeError:
            continue
        if isinstance(val, (int, float, bool, str)):
            out[field] = val
    return out


def _apply_walking_param(msg, fields: dict) -> None:
    for name, val in fields.items():
        if hasattr(msg, name):
            try:
                setattr(msg, name, val)
            except (AssertionError, TypeError):
                # rclpy's property setters enforce typing — skip silently
                # rather than crash the relay if a field's type changed.
                pass


def _sim_side(real_to_sim, sim_to_real) -> None:
    """Runs with ROS_DOMAIN_ID=SIM. Subscribes to the sim's joint states
    (forwards to ``sim_to_real``) and publishes the relayed commands from
    ``real_to_sim`` onto the sim domain's /robotis/* topics."""
    os.environ["ROS_DOMAIN_ID"] = str(SIM_DOMAIN_ID)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32, String

    try:
        from op3_walking_module_msgs.msg import WalkingParam  # type: ignore
    except Exception:  # pylint: disable=broad-except
        WalkingParam = None

    rclpy.init()
    node = rclpy.create_node("sim_domain_bridge_sim")
    print(
        f"[sim_domain_bridge_sim] up on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}",
        flush=True,
    )

    enable_pub = node.create_publisher(String, "/robotis/enable_ctrl_module", 10)
    wcmd_pub = node.create_publisher(String, "/robotis/walking/command", 10)
    page_pub = node.create_publisher(Int32, "/robotis/action/page_num", 10)
    wparam_pub = (
        node.create_publisher(WalkingParam, "/robotis/walking/set_params", 10)
        if WalkingParam is not None
        else None
    )

    def on_joint_state(msg: JointState) -> None:
        try:
            sim_to_real.put_nowait({
                "type": "joint_states",
                "name": list(msg.name),
                "position": [float(p) for p in msg.position],
                "velocity": [float(v) for v in msg.velocity],
                "effort": [float(e) for e in msg.effort],
            })
        except Exception:  # pylint: disable=broad-except
            # Queue full / closed — drop the frame rather than block the
            # ROS callback thread.
            pass

    node.create_subscription(
        JointState, "/robotis_op3/joint_states", on_joint_state, 10
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            while True:
                try:
                    cmd = real_to_sim.get_nowait()
                except Exception:
                    break
                kind = cmd.get("type")
                if kind == "enable_ctrl_module":
                    m = String()
                    m.data = str(cmd.get("data", ""))
                    enable_pub.publish(m)
                elif kind == "walking_command":
                    m = String()
                    m.data = str(cmd.get("data", ""))
                    wcmd_pub.publish(m)
                elif kind == "action_page":
                    m = Int32()
                    m.data = int(cmd.get("data", 0))
                    page_pub.publish(m)
                elif kind == "walking_param" and wparam_pub is not None:
                    m = WalkingParam()
                    _apply_walking_param(m, cmd.get("fields", {}))
                    wparam_pub.publish(m)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # pylint: disable=broad-except
            pass
        try:
            rclpy.shutdown()
        except Exception:  # pylint: disable=broad-except
            pass


def _real_side(real_to_sim, sim_to_real) -> None:
    """Runs with ROS_DOMAIN_ID=REAL. Subscribes to the studio's
    /bascorro_studio/sim/* command topics (forwards to ``real_to_sim``)
    and publishes the sim's joint stream onto
    /bascorro_studio/sim_preview/joint_states so the existing rosbridge
    (also on the real domain) can stream it to the browser."""
    os.environ["ROS_DOMAIN_ID"] = str(REAL_DOMAIN_ID)
    # Don't inherit LOCALHOST_ONLY: if the user runs the real robot over
    # a LAN, locking the real domain to localhost would block discovery
    # of the actual OPEN-CR machine. The rosbridge consumer of our
    # preview topic is local either way.
    os.environ.pop("ROS_LOCALHOST_ONLY", None)

    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32, String

    try:
        from op3_walking_module_msgs.msg import WalkingParam  # type: ignore
    except Exception:  # pylint: disable=broad-except
        WalkingParam = None

    rclpy.init()
    node = rclpy.create_node("sim_domain_bridge_real")
    print(
        f"[sim_domain_bridge_real] up on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}",
        flush=True,
    )

    preview_pub = node.create_publisher(
        JointState, "/bascorro_studio/sim_preview/joint_states", 10
    )

    def push_cmd(payload):
        try:
            real_to_sim.put_nowait(payload)
        except Exception:  # pylint: disable=broad-except
            pass

    node.create_subscription(
        String,
        "/bascorro_studio/sim/enable_ctrl_module",
        lambda m: push_cmd({"type": "enable_ctrl_module", "data": str(m.data)}),
        10,
    )
    node.create_subscription(
        String,
        "/bascorro_studio/sim/walking/command",
        lambda m: push_cmd({"type": "walking_command", "data": str(m.data)}),
        10,
    )
    node.create_subscription(
        Int32,
        "/bascorro_studio/sim/action/page_num",
        lambda m: push_cmd({"type": "action_page", "data": int(m.data)}),
        10,
    )
    if WalkingParam is not None:
        node.create_subscription(
            WalkingParam,
            "/bascorro_studio/sim/walking/set_params",
            lambda m: push_cmd({
                "type": "walking_param",
                "fields": _serialize_walking_param(m),
            }),
            10,
        )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            while True:
                try:
                    data = sim_to_real.get_nowait()
                except Exception:
                    break
                if data.get("type") != "joint_states":
                    continue
                js = JointState()
                js.header.stamp = node.get_clock().now().to_msg()
                js.name = data.get("name", [])
                js.position = data.get("position", [])
                js.velocity = data.get("velocity", [])
                js.effort = data.get("effort", [])
                preview_pub.publish(js)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # pylint: disable=broad-except
            pass
        try:
            rclpy.shutdown()
        except Exception:  # pylint: disable=broad-except
            pass


def main() -> None:
    # spawn: each child gets a fresh interpreter and re-imports rclpy
    # after we've set ROS_DOMAIN_ID for that child. With fork (Linux
    # default), rclpy state from the parent would leak and our env tweaks
    # would be ignored.
    ctx = mp.get_context("spawn")
    real_to_sim = ctx.Queue(maxsize=2048)
    sim_to_real = ctx.Queue(maxsize=2048)

    print(
        f"[sim_domain_bridge] orchestrator starting "
        f"(SIM_DOMAIN={SIM_DOMAIN_ID}, REAL_DOMAIN={REAL_DOMAIN_ID})",
        flush=True,
    )

    sim_p = ctx.Process(
        target=_sim_side, args=(real_to_sim, sim_to_real), name="sim_side",
    )
    real_p = ctx.Process(
        target=_real_side, args=(real_to_sim, sim_to_real), name="real_side",
    )
    sim_p.start()
    real_p.start()

    try:
        while True:
            if not sim_p.is_alive() or not real_p.is_alive():
                # One side died — bring the whole bridge down so apply_node's
                # liveness reporter can flag the failure instead of running
                # half-broken.
                print(
                    f"[sim_domain_bridge] child exited: "
                    f"sim_alive={sim_p.is_alive()}, real_alive={real_p.is_alive()}",
                    flush=True,
                )
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for p in (sim_p, real_p):
            if p.is_alive():
                p.terminate()
                p.join(timeout=2)
                if p.is_alive():
                    p.kill()
        sys.exit(0 if not (sim_p.exitcode or real_p.exitcode) else 1)


if __name__ == "__main__":
    main()
