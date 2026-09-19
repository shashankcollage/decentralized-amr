"""
tests/test_task_allocation.py
================================
Tests for tasks/auction.py, tasks/task_manager.py, tasks/task_reassignment.py.
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tasks.auction import BidInputs, calculate_bid, select_winner
from tasks.task import Task, TaskStatus
from tasks.task_manager import TaskManager
from tasks.task_reassignment import task_needs_reassignment, find_tasks_needing_reassignment
import config


def test_lowest_cost_robot_wins():
    bids = {"R1": 10.0, "R2": 5.0, "R3": 20.0}
    assert select_winner(bids) == "R2"


def test_select_winner_tie_break_deterministic():
    bids = {"R2": 5.0, "R1": 5.0}
    assert select_winner(bids) == "R1"  # lexicographically smaller wins ties


def test_select_winner_empty_bids():
    assert select_winner({}) is None


def test_closer_robot_bids_lower_cost():
    close = BidInputs(robot_position=(1, 1), pickup_location=(2, 2), battery_percent=100.0,
                       current_workload=0, nearby_robot_count=0)
    far = BidInputs(robot_position=(15, 15), pickup_location=(2, 2), battery_percent=100.0,
                     current_workload=0, nearby_robot_count=0)
    assert calculate_bid(close) < calculate_bid(far)


def test_critical_battery_robot_bids_much_higher():
    normal = BidInputs(robot_position=(1, 1), pickup_location=(2, 2), battery_percent=90.0,
                        current_workload=0, nearby_robot_count=0)
    critical = BidInputs(robot_position=(1, 1), pickup_location=(2, 2), battery_percent=5.0,
                          current_workload=0, nearby_robot_count=0)
    assert calculate_bid(critical) > calculate_bid(normal)


def test_task_manager_assign_and_complete():
    tm = TaskManager()
    tm.add_task(Task("T1", (0, 0), (5, 5)))
    assert tm.waiting_tasks()[0].task_id == "T1"

    tm.mark_assigned("T1", "R1", now=10.0)
    assert tm.get("T1").status == TaskStatus.ASSIGNED
    assert tm.get("T1").assigned_robot == "R1"
    assert tm.waiting_tasks() == []

    tm.mark_completed("T1")
    assert tm.get("T1").status == TaskStatus.COMPLETED
    assert tm.all_completed()


def test_failed_robot_task_returns_to_waiting_pool():
    tm = TaskManager()
    tm.add_task(Task("T1", (0, 0), (5, 5)))
    tm.mark_assigned("T1", "R1", now=0.0)
    assert tm.get("T1").status == TaskStatus.ASSIGNED

    tm.mark_waiting_again("T1")
    task = tm.get("T1")
    assert task.status == TaskStatus.WAITING
    assert task.assigned_robot is None
    assert task in tm.waiting_tasks()


def test_task_reassignment_when_robot_unavailable():
    task = Task("T1", (0, 0), (5, 5))
    task.status = TaskStatus.ASSIGNED
    task.assigned_robot = "R1"
    task.assigned_time = 0.0
    needs = task_needs_reassignment(
        task, assigned_robot_is_available=False, assigned_robot_battery=90.0,
        critical_battery_threshold=config.CRITICAL_BATTERY, now=5.0, reassign_timeout=90.0,
    )
    assert needs


def test_task_no_reassignment_when_robot_fine():
    task = Task("T1", (0, 0), (5, 5))
    task.status = TaskStatus.ASSIGNED
    task.assigned_robot = "R1"
    task.assigned_time = 0.0
    needs = task_needs_reassignment(
        task, assigned_robot_is_available=True, assigned_robot_battery=90.0,
        critical_battery_threshold=config.CRITICAL_BATTERY, now=5.0, reassign_timeout=90.0,
    )
    assert not needs


def test_find_tasks_needing_reassignment_batch():
    t1 = Task("T1", (0, 0), (5, 5))
    t1.status = TaskStatus.ASSIGNED
    t1.assigned_robot = "R1"
    t1.assigned_time = 0.0
    t2 = Task("T2", (1, 1), (6, 6))
    t2.status = TaskStatus.ASSIGNED
    t2.assigned_robot = "R2"
    t2.assigned_time = 0.0

    stuck = find_tasks_needing_reassignment(
        [t1, t2], unavailable_robot_ids=["R1"], robot_batteries={"R1": 90.0, "R2": 90.0},
        critical_battery_threshold=config.CRITICAL_BATTERY, now=5.0, reassign_timeout=90.0,
    )
    assert [t.task_id for t in stuck] == ["T1"]
