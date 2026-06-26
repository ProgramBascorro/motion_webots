#!/usr/bin/env python3
"""
Forward kinematics: INIT_BARU (action page 2) joint values -> walking_module
init_x/y/z_offset.

USAGE:
    Update PAGE2 dict below with the joint values from the robot's
    motion bin (decoded via action editor / yaml export), then run:
        python3 scripts/init_baru_fk.py

OUTPUT:
    Numbers to paste into op3_walking_module/config/param.yaml and
    bascorro_studio/web/src/App.jsx WALKING_DEFAULT_PARAMS so the
    walking_module's IK-derived neutral pose matches the calibrated
    INIT_BARU standing geometry.

WHY:
    walking_module's "walking ready" pose is computed from init_x/y/z_offset
    via IK (op3_walking_module.cpp:1220 `ep[2] = z_offset_ - leg_length`).
    For the boot→walking transition to be smooth (no jerk when soccer demo
    switches body to walking_module), the IK pose MUST match the INIT_BARU
    page 2 pose. This script derives that match.
"""
import math

# ===== INPUT: joint values from INIT_BARU page 2 (raw dynamixel 0-4095) =====
# Read from action yaml export of motion_4095_ros1_lama.bin, page index 2
# step 0 positions. Example here is from Robot 0's calibration.
PAGE2 = {
    "r_sho_pitch": 2198, "l_sho_pitch": 1888,
    "r_sho_roll": 1737,  "l_sho_roll": 2365,
    "r_el": 2334,        "l_el": 1754,
    "r_hip_yaw": 2045,   "l_hip_yaw": 2047,
    "r_hip_roll": 2045,  "l_hip_roll": 2010,
    "r_hip_pitch": 2471, "l_hip_pitch": 1612,
    "r_knee": 1371,      "l_knee": 2724,
    "r_ank_pitch": 1727, "l_ank_pitch": 2377,
    "r_ank_roll": 2061,  "l_ank_roll": 2070,
    "head_pan": 2049,    "head_tilt": 1922,
}

# ===== OP3 spec link lengths (m) =====
THIGH = 0.093
CALF  = 0.093
ANKLE = 0.0335
HIP_Y = 0.037           # half hip width
LEG_LENGTH = THIGH + CALF + ANKLE   # 0.2195

def dxl_to_rad(raw):
    return (raw - 2048) * (2 * math.pi / 4096)

def Rx(a):
    c, s = math.cos(a), math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def Ry(a):
    c, s = math.cos(a), math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def Rz(a):
    c, s = math.cos(a), math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def matmul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def matvec(A, v):
    return [sum(A[i][j]*v[j] for j in range(3)) for i in range(3)]
def add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

def fk_leg(side, hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll):
    """side: +1 left, -1 right. OP3 right leg has mirrored pitch sign."""
    q1, q2, q3, q4, q5, q6 = (dxl_to_rad(x) for x in
                              (hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll))
    if side == -1:  # right leg pitch signs inverted vs left
        q3, q4, q5 = -q3, -q4, -q5
    hip_origin = [0, side * HIP_Y, 0]
    R = matmul(Rz(q1), Rx(q2)); R = matmul(R, Ry(q3))
    thigh_vec = matvec(R, [0, 0, -THIGH])
    knee_origin = add(hip_origin, thigh_vec)
    R = matmul(R, Ry(q4))
    calf_vec = matvec(R, [0, 0, -CALF])
    ank_origin = add(knee_origin, calf_vec)
    R = matmul(R, Ry(q5)); R = matmul(R, Rx(q6))
    ankle_vec = matvec(R, [0, 0, -ANKLE])
    return add(ank_origin, ankle_vec)

rf = fk_leg(-1, PAGE2["r_hip_yaw"], PAGE2["r_hip_roll"], PAGE2["r_hip_pitch"],
            PAGE2["r_knee"], PAGE2["r_ank_pitch"], PAGE2["r_ank_roll"])
lf = fk_leg(+1, PAGE2["l_hip_yaw"], PAGE2["l_hip_roll"], PAGE2["l_hip_pitch"],
            PAGE2["l_knee"], PAGE2["l_ank_pitch"], PAGE2["l_ank_roll"])

avg_x = (rf[0] + lf[0]) / 2
avg_z = (rf[2] + lf[2]) / 2

# Walking_module conventions:
#   ep[2] = z_offset - leg_length        (see op3_walking_module.cpp:1220)
#   So z_offset = leg_length + foot_z_from_hip
init_x_offset = -avg_x                     # body shifts opposite to foot offset
init_y_offset = abs(rf[1] - lf[1]) / 2     # half feet spread
init_z_offset = LEG_LENGTH + avg_z          # avg_z is negative (foot below hip)

print(f"Right foot (m):  x={rf[0]:+.4f}  y={rf[1]:+.4f}  z={rf[2]:+.4f}")
print(f"Left foot  (m):  x={lf[0]:+.4f}  y={lf[1]:+.4f}  z={lf[2]:+.4f}")
print()
print(f"=== Paste into op3_walking_module/config/param.yaml ===")
print(f"x_offset: {init_x_offset:+.4f}")
print(f"y_offset: {init_y_offset:+.4f}")
print(f"z_offset: {init_z_offset:+.4f}")
print()
print(f"=== Paste into Bascorro WALKING_DEFAULT_PARAMS (App.jsx) ===")
print(f"  init_x_offset: {init_x_offset:+.4f},")
print(f"  init_y_offset: {init_y_offset:+.4f},")
print(f"  init_z_offset: {init_z_offset:+.4f},")
print()
print("Note: pitch_offset is NOT derived here (joint sign convention unreliable).")
print("Keep proven 4° = 0.0698 rad. Adjust by ±1° if robot leans wrong.")
