"""
tasks/task_manager.py
========================
Holds the pool of Task objects and their statuses.

DECENTRALIZATION NOTE: TaskManager is a class, not a service - every
robot instantiates its OWN TaskManager and updates it from messages
(TASK_ASSIGNMENT, TASK_COMPLETED - see communication/message.py) plus its
own bidding decisions. Because the same class/logic runs independently
inside every robot, there is no single "the" TaskManager coordinating the
fleet, even though every robot's copy converges to the same view given
the same messages - the same "eventually consistent by construction"
pattern used for the wait-for graph (coordination/wait_for_graph.py) and
the reservation table (planning/reservation_table.py).

For simulation convenience, task loading (from JSON) and the initial
pool are shared setup, exactly like the Warehouse object in robots/robot.py's
docstring - a simulation-convenience shortcut, not a decentralization
violation, since each robot still decides which tasks to bid on and who
wins independently.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from tasks.task import Task, TaskStatus

logger = logging.getLogger("task_manager")


@dataclass
class TaskManager:
    tasks: Dict[str, Task] = field(default_factory=dict)

    @classmethod
    def from_json(cls, path: str) -> "TaskManager":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        tm = cls()
        for entry in data.get("tasks", []):
            task = Task(
                task_id=entry["task_id"],
                pickup_location=tuple(entry["pickup_location"]),
                dropoff_location=tuple(entry["dropoff_location"]),
                priority=entry.get("priority", 0.5),
            )
            tm.tasks[task.task_id] = task
        logger.info("Loaded %d tasks from %s", len(tm.tasks), path)
        return tm

    def add_task(self, task: Task) -> None:
        self.tasks[task.task_id] = task

    def get(self, task_id: str) -> Optional[Task]:
        return self.tasks.get(task_id)

    def waiting_tasks(self) -> List[Task]:
        return [t for t in self.tasks.values() if t.status == TaskStatus.WAITING]

    def tasks_assigned_to(self, robot_id: str) -> List[Task]:
        return [t for t in self.tasks.values()
                if t.assigned_robot == robot_id and t.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS)]

    def mark_assigned(self, task_id: str, robot_id: str, now: float = 0.0) -> None:
        task = self.tasks.get(task_id)
        if task is None:
            return
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot = robot_id
        task.assigned_time = now
        logger.info("Task %s assigned to %s", task_id, robot_id)

    def mark_in_progress(self, task_id: str) -> None:
        task = self.tasks.get(task_id)
        if task is not None:
            task.status = TaskStatus.IN_PROGRESS

    def mark_completed(self, task_id: str) -> None:
        task = self.tasks.get(task_id)
        if task is not None:
            task.status = TaskStatus.COMPLETED

    def mark_waiting_again(self, task_id: str) -> None:
        """Return a task to the WAITING pool for re-auction, e.g. after
        its assigned robot fails or times out (see task_reassignment.py).
        """
        task = self.tasks.get(task_id)
        if task is not None:
            task.status = TaskStatus.WAITING
            task.assigned_robot = None
            task.assigned_time = None
            logger.info("Task %s returned to WAITING pool for re-auction", task_id)

    def all_completed(self) -> bool:
        return bool(self.tasks) and all(t.status == TaskStatus.COMPLETED for t in self.tasks.values())

    def to_dict(self) -> dict:
        return {tid: t.to_dict() for tid, t in self.tasks.items()}
