"""A* 8-direction grid planner.

Grid: 10cm resolution over a 9×6m RoboCup KidSize field → 90×60 cells.
Returns (vx, vy) unit direction toward next waypoint from A* path.
"""
import heapq
import math
import time
from typing import List, Optional, Tuple

# Field dimensions
FIELD_W = 9.0   # metres
FIELD_H = 6.0
CELL = 0.10     # metres per cell
COLS = int(FIELD_W / CELL)   # 90
ROWS = int(FIELD_H / CELL)   # 60

_SQRT2 = math.sqrt(2)

# 8-connected neighbours: (dc, dr, cost)
_NEIGHBOURS = [
    (-1, -1, _SQRT2), (0, -1, 1.0), (1, -1, _SQRT2),
    (-1,  0, 1.0),                   (1,  0, 1.0),
    (-1,  1, _SQRT2), (0,  1, 1.0), (1,  1, _SQRT2),
]


def _world_to_cell(x: float, y: float) -> Tuple[int, int]:
    c = int((x + FIELD_W / 2) / CELL)
    r = int((y + FIELD_H / 2) / CELL)
    return (max(0, min(COLS - 1, c)), max(0, min(ROWS - 1, r)))


def _cell_to_world(c: int, r: int) -> Tuple[float, float]:
    x = (c + 0.5) * CELL - FIELD_W / 2
    y = (r + 0.5) * CELL - FIELD_H / 2
    return (x, y)


def plan(
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    obstacles: List[Tuple[float, float]],
    obstacle_radius_m: float = 0.4,
) -> Optional[Tuple[float, float]]:
    """Return (vx, vy) unit direction from A*.

    Returns None if no path found or goal == start.
    Obstacles are (x, y) world-frame positions.
    """
    t0 = time.monotonic()

    sc, sr = _world_to_cell(*start_xy)
    gc, gr = _world_to_cell(*goal_xy)

    if (sc, sr) == (gc, gr):
        return None

    # Build obstacle set
    inflate_cells = max(1, int(obstacle_radius_m / CELL))
    blocked = set()
    for ox, oy in obstacles:
        oc, or_ = _world_to_cell(ox, oy)
        for dc in range(-inflate_cells, inflate_cells + 1):
            for dr in range(-inflate_cells, inflate_cells + 1):
                cc, cr = oc + dc, or_ + dr
                if 0 <= cc < COLS and 0 <= cr < ROWS:
                    blocked.add((cc, cr))

    # A* search
    open_heap = []
    heapq.heappush(open_heap, (0.0, sc, sr))
    came_from = {(sc, sr): None}
    g_cost = {(sc, sr): 0.0}

    goal = (gc, gr)
    found = False

    while open_heap:
        _, cc, cr = heapq.heappop(open_heap)
        if (cc, cr) == goal:
            found = True
            break
        for dc, dr, step_cost in _NEIGHBOURS:
            nc, nr = cc + dc, cr + dr
            if not (0 <= nc < COLS and 0 <= nr < ROWS):
                continue
            if (nc, nr) in blocked:
                continue
            ng = g_cost[(cc, cr)] + step_cost
            if ng < g_cost.get((nc, nr), 1e18):
                g_cost[(nc, nr)] = ng
                h = math.hypot(nc - gc, nr - gr)
                heapq.heappush(open_heap, (ng + h, nc, nr))
                came_from[(nc, nr)] = (cc, cr)

    elapsed_ms = (time.monotonic() - t0) * 1000
    if elapsed_ms > 30:
        pass  # slow replan — caller can log if needed

    if not found:
        # Fallback: direct bearing
        dx = goal_xy[0] - start_xy[0]
        dy = goal_xy[1] - start_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return None
        return (dx / dist, dy / dist)

    # Reconstruct path and return direction to first waypoint after start
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()

    if len(path) < 2:
        return None

    # Direction toward next cell
    next_c, next_r = path[1]
    wx, wy = _cell_to_world(next_c, next_r)
    dx = wx - start_xy[0]
    dy = wy - start_xy[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return None
    return (dx / dist, dy / dist)
