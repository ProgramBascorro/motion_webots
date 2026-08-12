#!/usr/bin/env python3
"""
Forward kinematics: action page 2 joint values -> walking_module
init_x/y/z_offset.

Page 2 was named INIT_BARU when this script was written; on 2026-08-12 it was
re-captured and renamed WALKING_READY. The PAGE2 dict below is still the OLD
snapshot -- see the note above it before trusting any number here.

=============================================================================
 NOTE FOR THIS ROBOT (CHRONUS) — READ BEFORE USING THE OUTPUT
=============================================================================
This script assumes raw dynamixel 2048 == mechanically-straight joint for
every servo. That held (roughly) for the source robot ("Robot 0"), but it does
NOT hold for this robot: its servo zero-calibration differs, so its page 2
raw values look like nearly-straight legs to this script and the FK returns:

    init_x_offset = -0.0023
    init_y_offset = +0.0291
    init_z_offset = +0.0034   <-- WRONG (out of sane range 0.02..0.10)

The robot actually stands crouched (working z_offset = 0.075). So the FK-derived
offsets are NOT used here. Instead, op3_walking_module uses a capture/bias
mechanism (see op3_walking_module.cpp ~line 504-544, 1207): when walking_module
is enabled it captures the live page 2 pose from the controller and biases
the gait onto it. That reads the REAL pose at runtime, so it needs neither a
correct FK chain nor a correct servo zero-calibration. param.yaml keeps the
empirically-tuned offsets (x=-0.015, y=0.015, z=0.075).

This file is kept only as documentation / tooling. Decision made 2026-06-27.
=============================================================================

USAGE:
    Update PAGE2 dict below with the joint values from the robot's motion bin
    (op3_action_module/data/motion_4095_CHRONUS.bin, page index 2 step 0),
    then run:
        python3 scripts/init_baru_fk.py
"""
import math

# ===== INPUT: joint values from action page 2 (raw dynamixel 0-4095) =====
# Decoded from motion_4095_CHRONUS.bin, page index 2, step 0 positions.
#
# STALE: this is page 2 as it was when named "INIT_BARU". Page 2 was re-captured
# on 2026-08-12 and renamed "WALKING_READY"; five leg joints moved, l_ank_pitch
# by 65 degrees (1956 -> 1211). Re-decode the bin before re-running this.
PAGE2 = {
    "r_sho_pitch": 2457, "l_sho_pitch": 1674,
    "r_sho_roll": 2398,  "l_sho_roll": 1785,
    "r_el": 3530,        "l_el": 569,
    "r_hip_yaw": 2043,   "l_hip_yaw": 2069,
    "r_hip_roll": 2014,  "l_hip_roll": 1971,
    "r_hip_pitch": 1943, "l_hip_pitch": 2145,
    "r_knee": 2216,      "l_knee": 1886,
    "r_ank_pitch": 2108, "l_ank_pitch": 1956,
    "r_ank_roll": 2049,  "l_ank_roll": 2017,
    "head_pan": 2072,    "head_tilt": 2113,
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

init_x_offset = -avg_x
init_y_offset = abs(rf[1] - lf[1]) / 2
init_z_offset = LEG_LENGTH + avg_z

print(f"Right foot (m):  x={rf[0]:+.4f}  y={rf[1]:+.4f}  z={rf[2]:+.4f}")
print(f"Left foot  (m):  x={lf[0]:+.4f}  y={lf[1]:+.4f}  z={lf[2]:+.4f}")
print()
print(f"init_x_offset: {init_x_offset:+.4f}")
print(f"init_y_offset: {init_y_offset:+.4f}")
print(f"init_z_offset: {init_z_offset:+.4f}")
print()
sane = 0.02 < init_z_offset < 0.10
print(f"sanity (0.02 < z < 0.10): {sane}")
if not sane:
    print("!! z_offset out of range -> FK unreliable for this robot's calibration.")
    print("!! Do NOT paste into param.yaml. Use walking_module capture/bias instead.")
