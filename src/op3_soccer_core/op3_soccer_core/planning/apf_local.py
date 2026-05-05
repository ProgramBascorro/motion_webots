"""Artificial Potential Field — local obstacle avoidance.

Blends with A* direction to produce smooth velocity commands.
Returns (vx, vy) — NOT normalized; caller should clamp to max velocity.
"""
import math
from typing import List, Tuple


def blend(
    astar_dir: Tuple[float, float],
    goal_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    obstacles: List[Tuple[float, float]],
    k_att: float = 1.5,
    k_rep: float = 0.8,
    influence_radius: float = 0.5,
) -> Tuple[float, float]:
    """Return (vx, vy) blending A* direction with APF repulsion.

    astar_dir: (dx, dy) unit vector from A* — used as attractive direction.
    Repulsive forces push away from obstacles within influence_radius metres.
    """
    # Attractive component (from A* direction)
    dist_to_goal = math.hypot(
        goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
    )
    att_scale = min(k_att, k_att * dist_to_goal)
    fx = astar_dir[0] * att_scale
    fy = astar_dir[1] * att_scale

    # Repulsive component from each obstacle
    for ox, oy in obstacles:
        dx = robot_xy[0] - ox
        dy = robot_xy[1] - oy
        dist = math.hypot(dx, dy)
        if dist < 1e-3 or dist > influence_radius:
            continue
        rep_mag = k_rep * (1.0 / dist - 1.0 / influence_radius) / (dist * dist)
        fx += rep_mag * dx / dist
        fy += rep_mag * dy / dist

    return (fx, fy)
