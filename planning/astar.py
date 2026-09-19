"""
planning/astar.py
===================
Time-aware A* search for a single robot on the warehouse grid.

Standard A* would only avoid STATIC obstacles (walls/shelves) and could
send two robots into the same cell at the same time. This implementation
searches over (cell, timestep) states instead of just cells, so it can
also avoid cells already claimed in a ReservationTable - this is what
makes multi-robot planning (planning/multi_agent_planner.py) actually
collision-aware, per project spec section 13.

Heuristic: Manhattan distance (admissible and consistent for a
4-directional grid with unit edge costs), per project spec section 12.

The search also allows a "wait in place" action (staying at the same
cell for one timestep), which is essential for time-aware planning: a
robot sometimes needs to wait one step for a cell to free up rather than
being forced to take a detour or fail to find any path at all.
"""

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from simulation.physics import manhattan_distance
from simulation.warehouse import Warehouse
from planning.reservation_table import ReservationTable

Coordinate = Tuple[int, int]

# Search bounds: how many timesteps into the future we're willing to
# search before giving up. Prevents unbounded search when the grid is
# heavily reserved. Tuned generously relative to typical warehouse
# traversal lengths.
MAX_SEARCH_HORIZON = 300


def find_path(
    start: Coordinate,
    goal: Coordinate,
    warehouse: Warehouse,
    reservations: Optional[ReservationTable] = None,
    robot_id: str = "",
    start_timestep: int = 0,
    max_horizon: int = MAX_SEARCH_HORIZON,
) -> Optional[List[Coordinate]]:
    """Find a shortest, reservation-conflict-free path from start to goal.

    Args:
        start: starting grid cell.
        goal: destination grid cell.
        warehouse: grid used for static/dynamic obstacle checks.
        reservations: optional time-indexed reservation table. If given,
            the search avoids any (cell, timestep) already claimed by a
            DIFFERENT robot than `robot_id`.
        robot_id: the id of the robot planning this path (used to ignore
            the robot's own existing reservations, e.g. when re-planning).
        start_timestep: the reservation-table timestep corresponding to
            `start` (usually the robot's current step index).
        max_horizon: safety cap on search depth (in timesteps).

    Returns:
        A list of coordinates from start to goal inclusive (length >= 1),
        or None if no conflict-free path exists within max_horizon.
    """
    if start == goal:
        return [start]

    if warehouse.is_statically_blocked(goal):
        return None

    # Priority queue of (f_score, tie_breaker, cell, timestep)
    open_heap: List[Tuple[int, int, Coordinate, int]] = []
    counter = 0
    g_score: Dict[Tuple[Coordinate, int], int] = {(start, start_timestep): 0}
    came_from: Dict[Tuple[Coordinate, int], Tuple[Coordinate, int]] = {}

    h0 = manhattan_distance(start, goal)
    heapq.heappush(open_heap, (h0, counter, start, start_timestep))
    visited = set()

    while open_heap:
        _, _, current, t = heapq.heappop(open_heap)
        if (current, t) in visited:
            continue
        visited.add((current, t))

        if current == goal:
            return _reconstruct_path(came_from, (current, t))

        if t - start_timestep >= max_horizon:
            continue

        # Candidate moves: 4-directional neighbors, plus "wait in place".
        candidates = warehouse.neighbors(current) + [current]

        for next_cell in candidates:
            next_t = t + 1

            if next_cell != current:
                if warehouse.is_statically_blocked(next_cell):
                    continue
                if warehouse.is_dynamically_blocked(next_cell):
                    continue

            if reservations is not None:
                if reservations.is_reserved(next_cell, next_t, exclude_robot=robot_id):
                    continue
                # Prevent a "swap" (head-on) collision: don't let the robot
                # move into a cell that is being vacated by another robot
                # who is simultaneously moving into the current cell.
                owner_of_next_now = reservations.get_reservation_owner(next_cell, t)
                owner_of_current_next = reservations.get_reservation_owner(current, next_t)
                if (owner_of_next_now not in (None, robot_id)
                        and owner_of_current_next not in (None, robot_id)
                        and owner_of_next_now == owner_of_current_next):
                    continue

            tentative_g = g_score[(current, t)] + 1
            key = (next_cell, next_t)
            if tentative_g < g_score.get(key, float("inf")):
                g_score[key] = tentative_g
                came_from[key] = (current, t)
                f_score = tentative_g + manhattan_distance(next_cell, goal)
                counter += 1
                heapq.heappush(open_heap, (f_score, counter, next_cell, next_t))

    return None  # no conflict-free path found within the horizon


def _reconstruct_path(came_from: Dict[Tuple[Coordinate, int], Tuple[Coordinate, int]],
                       end_key: Tuple[Coordinate, int]) -> List[Coordinate]:
    path = [end_key[0]]
    key = end_key
    while key in came_from:
        key = came_from[key]
        path.append(key[0])
    path.reverse()
    return path


def find_path_ignoring_reservations(start: Coordinate, goal: Coordinate,
                                     warehouse: Warehouse) -> Optional[List[Coordinate]]:
    """Plain single-agent A* (no time dimension, no reservations). Useful
    for quick feasibility checks (e.g. "is this destination reachable at
    all given current static+dynamic blocks?") without the cost of a
    full time-expanded search.
    """
    return find_path(start, goal, warehouse, reservations=None)
