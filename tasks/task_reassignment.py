"""
tasks/task_reassignment.py
=============================
Detects when an in-progress task needs to be returned to the WAITING
pool for re-auction, per project spec section 22:

    if: robot fails / battery too low / aisle permanently blocked /
        robot disconnected / task timeout occurs
    then: task -> WAITING, another robot can bid for it.

This module only decides WHETHER a task needs reassignment - the actual
mark_waiting_again() call and re-triggering of bidding happens in
robots/robot_controller.py, which is what actually observes peer
liveness (via its own NetworkManager) and its own battery/task state.
"""

from __future__ import annotations

from typing import List, Optional

from tasks.task import Task, TaskStatus


def task_needs_reassignment(
    task: Task,
    assigned_robot_is_available: bool,
    assigned_robot_battery: Optional[float],
    critical_battery_threshold: float,
    now: float,
    reassign_timeout: float,
) -> bool:
    """True if `task` (currently ASSIGNED/IN_PROGRESS) should be returned
    to the WAITING pool.

    Conditions checked (any one triggers reassignment):
      * the assigned robot is no longer available (failed/disconnected,
        as determined by the caller's NetworkManager.unavailable_peers())
      * the assigned robot's battery has dropped to critical
      * the task has been assigned/in-progress for longer than
        `reassign_timeout` without completing (a generic timeout that
        also covers "permanently blocked aisle" - if a robot truly can't
        make progress, it will eventually blow this timeout regardless
        of the specific cause)
    """
    if task.status not in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
        return False

    if not assigned_robot_is_available:
        return True

    if assigned_robot_battery is not None and assigned_robot_battery <= critical_battery_threshold:
        return True

    reference_time = task.assigned_time if task.assigned_time is not None else task.created_time
    if (now - reference_time) > reassign_timeout and task.status != TaskStatus.COMPLETED:
        return True

    return False


def find_tasks_needing_reassignment(
    tasks: List[Task],
    unavailable_robot_ids: List[str],
    robot_batteries: dict,
    critical_battery_threshold: float,
    now: float,
    reassign_timeout: float,
) -> List[Task]:
    """Convenience batch version of task_needs_reassignment() over a list
    of tasks, used by robots/robot_controller.py once per tick."""
    result = []
    for task in tasks:
        if task.assigned_robot is None:
            continue
        is_available = task.assigned_robot not in unavailable_robot_ids
        battery = robot_batteries.get(task.assigned_robot)
        if task_needs_reassignment(task, is_available, battery, critical_battery_threshold,
                                     now, reassign_timeout):
            result.append(task)
    return result
