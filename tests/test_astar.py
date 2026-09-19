"""
tests/test_astar.py
======================
Tests for planning/astar.py and planning/reservation_table.py.
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.warehouse import Warehouse
from planning.astar import find_path
from planning.reservation_table import ReservationTable

WAREHOUSE_JSON = os.path.join(os.path.dirname(__file__), "..", "data", "warehouse.json")


def test_valid_path_exists():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    path = find_path((2, 2), (17, 17), wh)
    assert path is not None
    assert path[0] == (2, 2)
    assert path[-1] == (17, 17)
    for cell in path:
        assert wh.is_passable(cell)


def test_path_to_blocked_goal_returns_none():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    path = find_path((2, 2), (4, 3), wh)  # (4,3) is a SHELF
    assert path is None


def test_no_path_when_fully_enclosed():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    # Enclose (5,5) completely with dynamic blocks (it's normally open).
    wh.block_aisle([(4, 5), (6, 5), (5, 4), (5, 6)])
    path = find_path((5, 5), (17, 17), wh)
    assert path is None


def test_path_avoids_reservation_conflict():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    rt = ReservationTable()
    corridor = [(10, y) for y in range(1, 19)]
    rt.reserve_path("R1", corridor, start_timestep=0)

    path = find_path((11, 1), (10, 18), wh, reservations=rt, robot_id="R2", start_timestep=0)
    assert path is not None
    for i, cell in enumerate(path):
        owner = rt.get_reservation_owner(cell, i)
        assert owner in (None, "R2")


def test_reservation_table_reserve_and_release():
    rt = ReservationTable()
    path = [(1, 1), (2, 1), (3, 1)]
    ok = rt.reserve_path("R1", path, start_timestep=0)
    assert ok
    assert rt.is_reserved((2, 1), 1)
    assert rt.get_reservation_owner((2, 1), 1) == "R1"

    rt.release_path("R1")
    assert not rt.is_reserved((2, 1), 1)


def test_reservation_conflict_detection():
    rt = ReservationTable()
    rt.reserve_path("R1", [(1, 1), (2, 1)], start_timestep=0)
    # R2 reserving (2,1) at t=0 does not conflict (R1 only owns (2,1) at
    # t=1), so this succeeds - the conflict shows up when we check R2's
    # path against timestep 1, where R1 already holds (2,1).
    ok = rt.reserve_path("R2", [(2, 1), (3, 1)], start_timestep=0)
    assert ok

    conflicts = rt.find_conflicts("R2", [(2, 1), (3, 1)], start_timestep=1)
    assert any(c[1] == (2, 1) and c[2] == "R1" for c in conflicts)
