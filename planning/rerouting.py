"""
planning/rerouting.py
=======================
Implements the re-routing flow from project spec section 19:

    current position -> detect obstruction -> remove blocked cells ->
    run A* -> check reservations -> select valid path -> broadcast new
    intent -> continue

This module doesn't itself broadcast anything (that's the caller's job,
since it owns the NetworkManager) - it just answers "does this robot need
to replan right now, and if so, what's the new path?".
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from planning.multi_agent_planner import plan_path, path_is_still_valid
from planning.reservation_table import ReservationTable
from simulation.warehouse import Warehouse

Coordinate = Tuple[int, int]

logger = logging.getLogger("rerouting")


def needs_reroute(current_path: List[Coordinate], warehouse: Warehouse) -> bool:
    """True if the robot's remaining planned path now runs through a cell
    that has become blocked (static or dynamic) since it was planned -
    e.g. a newly-blocked aisle, per project spec section 19.
    """
    if not current_path:
        return False
    return not path_is_still_valid(current_path, warehouse)


def attempt_reroute(
    robot_id: str,
    current_position: Coordinate,
    destination: Coordinate,
    warehouse: Warehouse,
    reservations: ReservationTable,
    current_step_index: int,
) -> Optional[List[Coordinate]]:
    """Try to find a new conflict-free path from current_position to
    destination, given the warehouse's current (possibly newly blocked)
    layout and the reservation table's current (possibly stale, but
    best-available) view of peer claims.

    Returns the new path on success, or None if the robot is genuinely
    stuck (caller should set status BLOCKED and notify peers, per spec
    section 19's "if no path exists" branch).
    """
    new_path = plan_path(
        robot_id=robot_id,
        start=current_position,
        goal=destination,
        warehouse=warehouse,
        reservations=reservations,
        current_step_index=current_step_index,
    )
    if new_path is None:
        logger.warning("%s: no path found from %s to %s - reporting BLOCKED",
                        robot_id, current_position, destination)
        return None

    logger.info("%s: re-routed, new path length %d", robot_id, len(new_path))
    return new_path
