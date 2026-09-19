"""
tests/test_deadlock.py
=========================
Tests for coordination/wait_for_graph.py and coordination/deadlock.py.
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from coordination.wait_for_graph import build_wait_for_graph, find_cycles, cycle_containing
from coordination.deadlock import detect_deadlock, is_suspiciously_long_wait
from coordination.priority import PriorityInputs
import config


def test_no_cycle_when_no_one_waiting():
    graph = build_wait_for_graph({"R1": None, "R2": None})
    assert find_cycles(graph) == []


def test_no_cycle_in_a_simple_chain():
    graph = build_wait_for_graph({"R1": "R2", "R2": None})
    assert find_cycles(graph) == []


def test_three_way_cycle_detected():
    # R1 -> R2 -> R3 -> R1
    graph = build_wait_for_graph({"R1": "R2", "R2": "R3", "R3": "R1"})
    cycles = find_cycles(graph)
    assert len(cycles) >= 1
    assert cycle_containing(cycles, "R1") is not None
    assert cycle_containing(cycles, "R2") is not None
    assert cycle_containing(cycles, "R3") is not None


def test_two_way_head_on_cycle_detected():
    graph = build_wait_for_graph({"R1": "R2", "R2": "R1"})
    cycles = find_cycles(graph)
    assert len(cycles) == 1


def test_is_suspiciously_long_wait():
    assert not is_suspiciously_long_wait(1.0, wait_timeout=config.WAIT_TIMEOUT)
    assert is_suspiciously_long_wait(config.WAIT_TIMEOUT + 0.1, wait_timeout=config.WAIT_TIMEOUT)


def test_detect_deadlock_picks_lowest_priority_to_back_off():
    waiting_for = {"R1": "R2", "R2": "R3", "R3": "R1"}
    inputs = {
        "R1": PriorityInputs(task_urgency=0.5, waiting_time_seconds=3.0, battery_percent=80.0, distance_to_conflict=2.0),
        "R2": PriorityInputs(task_urgency=0.1, waiting_time_seconds=1.0, battery_percent=50.0, distance_to_conflict=2.0),
        "R3": PriorityInputs(task_urgency=0.9, waiting_time_seconds=5.0, battery_percent=20.0, distance_to_conflict=1.0),
    }
    info = detect_deadlock("R1", waiting_for, inputs)
    assert info is not None
    assert info.robot_to_back_off in ("R1", "R2", "R3")
    # R2 has the lowest urgency/waiting/battery/proximity combination -
    # verify it is indeed the computed minimum, not asserted blindly.
    from coordination.priority import calculate_priority
    scores = {rid: calculate_priority(inp) for rid, inp in inputs.items()}
    expected = min(scores, key=lambda rid: (scores[rid], rid))
    assert info.robot_to_back_off == expected


def test_detect_deadlock_returns_none_for_uninvolved_robot():
    waiting_for = {"R1": "R2", "R2": "R3", "R3": "R1"}
    inputs = {rid: PriorityInputs(0.5, 1.0, 80.0, 2.0) for rid in ("R1", "R2", "R3")}
    info = detect_deadlock("R4", waiting_for, inputs)
    assert info is None
