"""Webots Walking Bridge Node — Layer 8 simulation adapter.

Bridges motion_node (/robotis/* topics, expecting op3_manager on hardware)
to Webots extern_controller (/robotis_op3/*_position/command Float64).

Fixes over v1:
  - Subscribes to /robotis_op3/joint_states → action interpolation starts from
    actual robot pose (not always-zero standing pose).
  - Action playback is timer-driven at CONTROL_RATE instead of sleep() thread,
    so timing is in sync with ROS2 clock regardless of Webots real-time factor.
  - Hip-roll lateral sway added to walking gait for better balance.
  - Configurable sign-flip parameters so sign convention can be corrected in
    launch file without editing code (gait_flip_hip, gait_flip_knee, etc.).
  - walk_x/y/yaw amplitudes read from parameters (set by WalkingParam service).

DXL ↔ Webots radian convention
-------------------------------
  webots_rad = (dxl_pos − HOME[j]) × DXL_RAD × DIR[j]

  HOME: DXL positions that equal 0 rad in Webots, extracted from the
        return-to-stand step of action pages 120–123 in motion_4095.bin.
  DIR:  sign factor from op3_kinematics_dynamics axis vectors (±1).

Webots sign conventions (from URDF axis analysis, VERIFY in sim)
-----------------------------------------------------------------
  r/l_hip_pitch : + = leg forward
  r_knee        : + = knee bends      l_knee: − = knee bends
  r_ank_pitch   : + = plantar flex    l_ank_pitch: − = plantar flex
  r/l_hip_roll  : − during right-swing (body sways left, over left support leg)
  r/l_sho_pitch : + = arm forward

If robot walks backward: set gait_flip_hip:=true
If robot tips sideways:  set gait_flip_roll:=true
"""

from __future__ import annotations

import dataclasses
import math
import os
import struct
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32, String

try:
    from op3_walking_module_msgs.msg import WalkingParam
    from op3_walking_module_msgs.srv import GetWalkingParam
    _HAS_WALKING_MSGS = True
except ImportError:
    _HAS_WALKING_MSGS = False

try:
    from ament_index_python.packages import get_package_share_directory
    _ACTION_FILE_DEFAULT = os.path.join(
        get_package_share_directory('op3_action_module'), 'data', 'motion_4095.bin'
    )
except Exception:
    _ACTION_FILE_DEFAULT = ''


# ── Hardware / DXL constants ─────────────────────────────────────────────────

DXL_RAD = 2.0 * math.pi / 4096.0

# Direction factors from op3_kinematics_dynamics (sum of axis vector)
DIR: Dict[str, int] = {
    'r_sho_pitch': -1, 'l_sho_pitch': +1,
    'r_sho_roll':  -1, 'l_sho_roll':  -1,
    'r_el':        +1, 'l_el':        +1,
    'r_hip_yaw':   -1, 'l_hip_yaw':   -1,
    'r_hip_roll':  -1, 'l_hip_roll':  -1,
    'r_hip_pitch': -1, 'l_hip_pitch': +1,
    'r_knee':      -1, 'l_knee':      +1,
    'r_ank_pitch': +1, 'l_ank_pitch': -1,
    'r_ank_roll':  +1, 'l_ank_roll':  +1,
    'head_pan':    +1, 'head_tilt':   -1,
}

# DXL home positions = 0 rad in Webots (from action file return-to-stand step)
HOME: Dict[str, int] = {
    'r_sho_pitch': 2048,
    'l_sho_pitch': 2105, 'r_sho_roll': 1990,  'l_sho_roll': 1533,
    'r_el':        2560,  'l_el':       2558,
    'r_hip_yaw':   1535,  'l_hip_yaw':  2047,
    'r_hip_roll':  2048,  'l_hip_roll': 2067,
    'r_hip_pitch': 2038,  'l_hip_pitch': 2331,
    'r_knee':      1767,  'l_knee':      1505,
    'r_ank_pitch': 2593,  'l_ank_pitch': 1707,
    'r_ank_roll':  2384,  'l_ank_roll':  2064,
    'head_pan':    2033,  'head_tilt':   2063,
}

DXL_ID_TO_JOINT: Dict[int, str] = {
    1:'r_sho_pitch', 2:'l_sho_pitch', 3:'r_sho_roll', 4:'l_sho_roll',
    5:'r_el',        6:'l_el',        7:'r_hip_yaw',  8:'l_hip_yaw',
    9:'r_hip_roll',  10:'l_hip_roll', 11:'r_hip_pitch', 12:'l_hip_pitch',
    13:'r_knee',     14:'l_knee',     15:'r_ank_pitch', 16:'l_ank_pitch',
    17:'r_ank_roll', 18:'l_ank_roll', 19:'head_pan',    20:'head_tilt',
}


def dxl_to_rad(joint: str, dxl_pos: int) -> float:
    return (dxl_pos - HOME[joint]) * DXL_RAD * DIR[joint]


# ── Action file parser ────────────────────────────────────────────────────────

_PAGE_SIZE   = 512
_HEADER_SIZE = 64
_STEP_SIZE   = 64
_INVALID_BIT = 0x4000


def _load_action_file(path: str) -> Dict[int, List]:
    """Return {page_num: [step_dict, ...]} or {} on error."""
    if not os.path.isfile(path):
        return {}
    try:
        data = open(path, 'rb').read()
    except OSError:
        return {}
    pages: Dict[int, List] = {}
    for pnum in range(min(len(data) // _PAGE_SIZE, 256)):
        hdr    = data[pnum * _PAGE_SIZE: pnum * _PAGE_SIZE + _HEADER_SIZE]
        stepn  = hdr[20]
        speed  = max(1, hdr[22])
        if stepn == 0:
            continue
        steps = []
        for s in range(stepn):
            base = pnum * _PAGE_SIZE + _HEADER_SIZE + s * _STEP_SIZE
            sd   = data[base: base + _STEP_SIZE]
            angles: Dict[str, float] = {}
            for jid in range(1, 21):
                raw = struct.unpack_from('<H', sd, (jid - 1) * 2)[0]
                if raw & _INVALID_BIT:
                    continue
                angles[DXL_ID_TO_JOINT[jid]] = dxl_to_rad(DXL_ID_TO_JOINT[jid], raw & 0x0FFF)
            # duration in control ticks (50 Hz), scaled by page speed (32 = nominal)
            time_ticks  = max(1, int(sd[63] * 32 / speed))
            pause_ticks = int(sd[62])
            steps.append({'angles': angles,
                          'time_ticks': time_ticks,
                          'pause_ticks': pause_ticks})
        pages[pnum] = steps
    return pages


# ── Timer-driven action state machine ────────────────────────────────────────

@dataclasses.dataclass
class _ActionState:
    steps:         List
    step_idx:      int = 0
    frame_idx:     int = 0
    pause_frames:  int = 0
    start_angles:  Dict = dataclasses.field(default_factory=dict)
    target_angles: Dict = dataclasses.field(default_factory=dict)
    done:          bool = False


# ═════════════════════════════════════════════════════════════════════════════
class WebotsWalkingBridge(Node):

    CONTROL_RATE = 50.0   # Hz — controls both gait update and action steps

    def __init__(self) -> None:
        super().__init__('webots_walking_bridge')

        # ── Parameters ───────────────────────────────────────────────────────
        self._prefix = self.declare_parameter(
            'joint_cmd_prefix', '/robotis_op3').value.rstrip('/')
        self._action_file_path = self.declare_parameter(
            'action_file_path', _ACTION_FILE_DEFAULT).value
        self._ball_est_topic = self.declare_parameter(
            'ball_estimate_topic', '/ball/estimate').value
        # Head pan gain: head_pan = bearing * gain (capped at joint limits)
        self._head_pan_gain = float(self.declare_parameter('head_pan_gain', 0.6).value)
        # Sweep amplitude when ball is lost (head scans back-and-forth)
        self._head_sweep_amp = float(self.declare_parameter('head_sweep_amp', 0.4).value)

        # Gait sign-flip corrections (set via launch args if robot walks wrong)
        self._flip_hip   = bool(self.declare_parameter('gait_flip_hip',   False).value)
        self._flip_knee  = bool(self.declare_parameter('gait_flip_knee',  False).value)
        self._flip_ankle = bool(self.declare_parameter('gait_flip_ankle', False).value)
        self._flip_roll  = bool(self.declare_parameter('gait_flip_roll',  False).value)

        # ── Action file ───────────────────────────────────────────────────────
        self._pages = _load_action_file(self._action_file_path)
        if self._pages:
            self.get_logger().info(
                f'Action file loaded: {len(self._pages)} pages')
        else:
            self.get_logger().warn(
                f'Action file not found: {self._action_file_path} — '
                'action playback disabled')

        # ── State ─────────────────────────────────────────────────────────────
        self._lock           = threading.Lock()
        self._walking        = False
        self._gait_phase     = 0.0
        self._period         = 0.60
        self._x_amp          = 0.0
        self._y_amp          = 0.002
        self._z_lift         = 0.020
        self._angle_amp      = 0.0
        self._action: Optional[_ActionState] = None

        # Head tracking state
        self._ball_bearing:    float = 0.0
        self._ball_visible:    bool  = False
        self._head_sweep_phase: float = 0.0   # for sweeping when ball is lost

        # Joint positions from extern_controller (actual robot pose)
        self._joint_pos: Dict[str, float] = {j: 0.0 for j in DIR}

        # ── Joint publishers ──────────────────────────────────────────────────
        self._pubs: Dict[str, object] = {
            j: self.create_publisher(Float64, f'{self._prefix}/{j}_position/command', 1)
            for j in DIR
        }

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Actual joint positions from Webots (used as action-start pose)
        self.create_subscription(
            JointState,
            f'{self._prefix}/joint_states',
            self._cb_joint_states, 1)

        # Ball estimate for head pan tracking
        self.create_subscription(
            Vector3Stamped, self._ball_est_topic, self._cb_ball_estimate, 1)

        self.create_subscription(String, '/robotis/enable_ctrl_module',
                                 lambda _: None, 10)
        self.create_subscription(String, '/robotis/walking/command',
                                 self._cb_walking_cmd, 10)
        self.create_subscription(Int32,  '/robotis/action/page_num',
                                 self._cb_action_page, 10)
        if _HAS_WALKING_MSGS:
            self.create_subscription(WalkingParam, '/robotis/walking/set_params',
                                     self._cb_set_params, 10)

        # ── GetWalkingParam service → makes baseline_ready = True ─────────────
        if _HAS_WALKING_MSGS:
            self.create_service(
                GetWalkingParam,
                '/robotis/walking/get_params',
                self._srv_get_params)

        # ── Single 50 Hz control tick ─────────────────────────────────────────
        self.create_timer(1.0 / self.CONTROL_RATE, self._tick)

        self.get_logger().info(
            f'WebotsWalkingBridge ready (prefix={self._prefix}, '
            f'flip_hip={self._flip_hip}, flip_knee={self._flip_knee}, '
            f'flip_ankle={self._flip_ankle}, flip_roll={self._flip_roll})')

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState) -> None:
        """Store actual joint positions from Webots for action interpolation."""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_pos:
                    self._joint_pos[name] = float(pos)

    def _cb_ball_estimate(self, msg: Vector3Stamped) -> None:
        """Update ball bearing for head pan tracking."""
        self._ball_bearing = float(msg.vector.y)
        self._ball_visible = bool(msg.vector.z > 0.5)

    def _head_pan_cmd(self) -> float:
        """Return desired head_pan angle in Webots radians.

        When ball is visible: track ball bearing (gain < 1 to avoid overrun).
        When ball is lost: sweep head back-and-forth to aid search.
        """
        dt = 1.0 / self.CONTROL_RATE
        if self._ball_visible:
            self._head_sweep_phase = 0.0   # reset sweep when ball found
            # Clamp to head joint limits (~±1.6 rad for OP3)
            pan = max(-1.4, min(1.4, self._ball_bearing * self._head_pan_gain))
            return pan
        else:
            # Sinusoidal sweep
            self._head_sweep_phase = (
                self._head_sweep_phase + 2.0 * math.pi * dt / 3.0  # 3s sweep period
            ) % (2.0 * math.pi)
            return math.sin(self._head_sweep_phase) * self._head_sweep_amp

    def _cb_walking_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        with self._lock:
            if cmd == 'start':
                if not self._walking:
                    self._gait_phase = 0.0
                self._walking = True
                self._action  = None   # cancel any playing action
            elif cmd == 'stop':
                self._walking = False
                self._gait_phase = 0.0

    def _cb_set_params(self, msg: 'WalkingParam') -> None:
        with self._lock:
            if msg.period_time > 0.05:
                self._period = float(msg.period_time)
            self._x_amp      = float(msg.x_move_amplitude)
            self._y_amp      = float(msg.y_move_amplitude) or 0.002
            self._z_lift     = float(msg.z_move_amplitude) or 0.020
            self._angle_amp  = float(msg.angle_move_amplitude)

    def _srv_get_params(self, _req, resp):
        if _HAS_WALKING_MSGS:
            wp = WalkingParam()
            with self._lock:
                wp.period_time          = self._period
                wp.x_move_amplitude     = self._x_amp
                wp.y_move_amplitude     = self._y_amp
                wp.z_move_amplitude     = self._z_lift
                wp.angle_move_amplitude = self._angle_amp
                wp.dsp_ratio            = 0.2
                wp.step_fb_ratio        = 0.25
                wp.balance_enable       = False
            resp.parameters = wp
        return resp

    def _cb_action_page(self, msg: Int32) -> None:
        page = int(msg.data)
        if page not in self._pages:
            self.get_logger().warn(f'Action page {page} not in file')
            return
        with self._lock:
            self._walking = False
            start = dict(self._joint_pos)   # actual current pose
            first_step = self._pages[page][0]
            target = dict(start)
            target.update(first_step['angles'])
            self._action = _ActionState(
                steps         = self._pages[page],
                step_idx      = 0,
                frame_idx     = 0,
                pause_frames  = 0,
                start_angles  = start,
                target_angles = target,
                done          = False,
            )
        self.get_logger().info(f'Playing action page {page} ({len(self._pages[page])} steps)')

    # ──────────────────────────────────────────────────────────────────────────
    # Main 50 Hz tick
    # ──────────────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        with self._lock:
            action   = self._action
            walking  = self._walking

        if action is not None and not action.done:
            self._advance_action()
            return

        # Head tracking runs regardless of walking state
        head_cmd = {'head_pan': self._head_pan_cmd()}

        if walking:
            angles = self._gait_step()
            angles.update(head_cmd)
            self._publish_angles(angles)
        else:
            # Hold standing pose (Webots 0 = natural standing), head tracks ball
            self._publish_angles({**{j: 0.0 for j in DIR}, **head_cmd})

    # ──────────────────────────────────────────────────────────────────────────
    # Timer-driven action playback (no sleep, runs at CONTROL_RATE)
    # ──────────────────────────────────────────────────────────────────────────

    def _advance_action(self) -> None:
        with self._lock:
            a = self._action
            if a is None or a.done:
                return

            # ── In pause between steps ────────────────────────────────────────
            if a.pause_frames > 0:
                a.pause_frames -= 1
                self._publish_angles(a.target_angles)
                return

            # ── Advance interpolation frame ───────────────────────────────────
            step = a.steps[a.step_idx]
            n    = step['time_ticks']
            a.frame_idx += 1
            alpha = a.frame_idx / n
            interp = {j: a.start_angles.get(j, 0.0)
                         + alpha * (a.target_angles.get(j, 0.0)
                                    - a.start_angles.get(j, 0.0))
                      for j in set(a.start_angles) | set(a.target_angles)}
            self._publish_angles(interp)

            # ── Step complete? ────────────────────────────────────────────────
            if a.frame_idx >= n:
                a.pause_frames = step['pause_ticks']
                a.step_idx    += 1
                a.frame_idx    = 0

                if a.step_idx >= len(a.steps):
                    a.done = True
                    self.get_logger().info('Action playback finished')
                    return

                # Prepare next step
                a.start_angles  = dict(a.target_angles)
                next_step       = a.steps[a.step_idx]
                a.target_angles = dict(a.start_angles)
                a.target_angles.update(next_step['angles'])

    # ──────────────────────────────────────────────────────────────────────────
    # Walking gait generator (sinusoidal, 50 Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def _gait_step(self) -> Dict[str, float]:
        """
        Advance gait phase and return joint angles.

        Sign conventions (URDF-derived, verify in simulation):
          r/l_hip_pitch : + = leg forward
          r_knee        : + = knee bent (flexion)
          l_knee        : − = knee bent
          r_ank_pitch   : + = plantar flex
          l_ank_pitch   : − = plantar flex
          r/l_hip_roll  : − during right swing (body sways left over stance leg)

        If gait_flip_* params are set True, the sign of that group is reversed.
        """
        with self._lock:
            x_amp     = self._x_amp
            z_lift    = self._z_lift
            y_amp     = self._y_amp
            angle_amp = self._angle_amp
            period    = self._period
            phase     = self._gait_phase
        dt = 1.0 / self.CONTROL_RATE
        self._gait_phase = (phase + 2.0 * math.pi * dt / period) % (2.0 * math.pi)

        s  = math.sin(phase)    # >0 = right swing, <0 = left swing
        c  = math.cos(phase)

        # ── Hip pitch: forward/backward step ─────────────────────────────────
        hip_sign = -1.0 if self._flip_hip else 1.0
        r_hip = hip_sign * (+x_amp * s)    # right leg forward during right swing
        l_hip = hip_sign * (-x_amp * s)    # left leg backward (pushing body fwd)

        # ── Knee bend: only during swing phase ───────────────────────────────
        # r_knee (+) = bent when sin > 0 (right swing)
        # l_knee (−) = bent when sin < 0 (left swing)
        knee_sign = -1.0 if self._flip_knee else 1.0
        r_knee = knee_sign * (+z_lift * max(0.0, s))
        l_knee = knee_sign * (-z_lift * max(0.0, -s))

        # ── Ankle compensation (simplified flat-foot) ─────────────────────────
        ank_sign = -1.0 if self._flip_ankle else 1.0
        ankle_k  = 0.75
        r_ank = ank_sign * (-r_hip * ankle_k)
        l_ank = ank_sign * (+l_hip * ankle_k)   # l_ank sign is already flipped by convention

        # ── Hip roll: lateral sway toward support leg ─────────────────────────
        # During right swing: body sways left → both hip rolls negative
        # Amplitude ≈ 0.10 rad (≈5.7°) for CoM shift over support foot
        roll_sign = -1.0 if self._flip_roll else 1.0
        sway_amp  = min(y_amp * 3.0, 0.12)   # proportional to y_move, max 0.12 rad
        r_roll    = roll_sign * (-sway_amp * s)   # negative during right swing
        l_roll    = roll_sign * (-sway_amp * s)

        # ── Hip yaw for turning ───────────────────────────────────────────────
        r_yaw = -angle_amp * max(0.0,  s) * 0.5
        l_yaw = +angle_amp * max(0.0, -s) * 0.5

        # ── Arm swing (counter to legs) ───────────────────────────────────────
        arm_amp  = abs(x_amp) * 0.35
        r_sho = -arm_amp * s
        l_sho = +arm_amp * s

        return {
            'r_hip_pitch': r_hip,   'l_hip_pitch': l_hip,
            'r_knee':      r_knee,  'l_knee':      l_knee,
            'r_ank_pitch': r_ank,   'l_ank_pitch': l_ank,
            'r_hip_roll':  r_roll,  'l_hip_roll':  l_roll,
            'r_hip_yaw':   r_yaw,   'l_hip_yaw':   l_yaw,
            'r_sho_pitch': r_sho,   'l_sho_pitch': l_sho,
        }

    # ──────────────────────────────────────────────────────────────────────────
    # Publisher helper
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_angles(self, angles: Dict[str, float]) -> None:
        for joint, rad in angles.items():
            pub = self._pubs.get(joint)
            if pub is not None:
                pub.publish(Float64(data=float(rad)))


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebotsWalkingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
