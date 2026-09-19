"""
tests/test_collision.py
==========================
Tests for planning/conflict_detector.py and coordination/conflict_resolution.py.
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from robots.robot_state import LocalWorldModel, RobotStatus
from planning.conflict_detector import (
    detect_spatial_collisions, detect_predicted_conflicts, ConflictType,
)
from coordination.conflict_resolution import resolve_conflict, Resolution
from planning.conflict_detector import Conflict


def _wm_with_peer(robot_id, position, path=None, status=RobotStatus.MOVING, now=0.0):
    wm = LocalWorldModel()
    wm.update_peer(robot_id, position=position, path=path or [], status=status,
                   last_message_time=now)
    return wm


def test_safe_robots_no_spatial_collision():
    wm = _wm_with_peer("R2", (10, 10))
    conflicts = detect_spatial_collisions((0, 0), wm, now=0.0, timeout=config.ROBOT_TIMEOUT)
    assert conflicts == []


def test_dangerous_distance_triggers_spatial_collision():
    wm = _wm_with_peer("R2", (5, 5))
    conflicts = detect_spatial_collisions((5, 5), wm, now=0.0, timeout=config.ROBOT_TIMEOUT,
                                            safe_distance=1.0)
    assert len(conflicts) == 1
    assert conflicts[0].conflict_type == ConflictType.SPATIAL


def test_same_cell_conflict_detected():
    wm = _wm_with_peer("R2", (5, 5), path=[(6, 6)])
    conflicts = detect_predicted_conflicts("R1", (5, 6), (6, 6), wm, now=0.0, timeout=config.ROBOT_TIMEOUT)
    assert any(c.conflict_type == ConflictType.SAME_CELL for c in conflicts)


def test_crossing_path_no_false_positive():
    # R2 is far away and not heading anywhere near R1's next cell.
    wm = _wm_with_peer("R2", (0, 0), path=[(0, 1)])
    conflicts = detect_predicted_conflicts("R1", (10, 10), (10, 11), wm, now=0.0, timeout=config.ROBOT_TIMEOUT)
    assert conflicts == []


def test_resolve_conflict_higher_priority_proceeds():
    conflict = Conflict(ConflictType.SAME_CELL, "R2", (5, 5))
    decision = resolve_conflict("R1", conflict, self_priority_score=0.9, other_priority_score=0.3)
    assert decision.resolution == Resolution.PROCEED


def test_resolve_conflict_lower_priority_yields():
    conflict = Conflict(ConflictType.SAME_CELL, "R2", (5, 5))
    decision = resolve_conflict("R1", conflict, self_priority_score=0.2, other_priority_score=0.8)
    assert decision.resolution == Resolution.YIELD


def test_resolve_conflict_tie_break_deterministic():
    conflict = Conflict(ConflictType.SAME_CELL, "R2", (5, 5))
    d1 = resolve_conflict("R1", conflict, self_priority_score=0.5, other_priority_score=0.5)
    d2 = resolve_conflict("R1", conflict, self_priority_score=0.5, other_priority_score=0.5)
    assert d1.resolution == d2.resolution  # deterministic, not random
    assert d1.resolution == Resolution.PROCEED  # "R1" < "R2" lexicographically
