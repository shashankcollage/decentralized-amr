"""
planning/multi_agent_planner.py
=================================
A thin layer over planning/astar.py that a Robot calls to (re)plan its own
path, converting between the robot's own step index (an integer count of
"cells moved so far", incremented once per completed cell transition) and
the ReservationTable's abstract timestep index.

This module does not communicate with anyone or hold any state itself -
it's a pure function library used by robots/robot_controller.py, which is
where the actual decision of "when to call this" lives (e.g. on
destination assignment, on detecting a blocked cell, after backing off
from a conflict).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from planning.astar import find_path
from planning.reservation_table import ReservationTable
from simulation.warehouse import Warehouse

Coordinate = Tuple[int, int]


def plan_path(
    robot_id: str,
    start: Coordinate,
    goal: Coordinate,
    warehouse: Warehouse,
    reservations: ReservationTable,
    current_step_index: int = 0,
) -> Optional[List[Coordinate]]:
    """Plan a reservation-aware path for one robot and claim it in the
    given reservation table.

    Returns the planned path (including the start cell), or None if no
    conflict-free path could be found. On success, the path is already
    reserved in `reservations` under `robot_id` - callers do not need to
    call reserve_path() again.
    """
    # Release this robot's previous claims first - a stale reservation
    # from an old plan would otherwise make replanning think cells the
    # robot itself once wanted are still "taken" by someone else (they
    # aren't; this robot owns them and is about to overwrite its own
    # intent), and would leak reserved-but-unused cells.
    reservations.release_path(robot_id)

    path = find_path(
        start=start,
        goal=goal,
        warehouse=warehouse,
        reservations=reservations,
        robot_id=robot_id,
        start_timestep=current_step_index,
    )
    if path is None:
        return None

    ok = reservations.reserve_path(robot_id, path, start_timestep=current_step_index)
    if not ok:
        # Should not normally happen since A* already avoided reserved
        # cells, but race conditions between planning and a peer's
        # just-arrived reservation are possible in principle; treat as
        # planning failure so the caller can retry/back off.
        reservations.release_path(robot_id)
        return None
    return path


def path_is_still_valid(path: List[Coordinate], warehouse: Warehouse) -> bool:
    """Cheap feasibility re-check: does `path` still avoid all STATIC and
    DYNAMIC obstacles? Does not check reservations (those change too fast
    to be worth checking here - conflict detection handles that
    separately per-tick). Used to detect "my path now runs through an
    aisle that just got blocked" without a full replan.
    """
    return all(warehouse.is_passable(cell) or i == 0 for i, cell in enumerate(path))
