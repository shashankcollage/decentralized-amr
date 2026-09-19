"""
tasks/task.py
===============
The Task data model, per project spec section 20.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

Coordinate = Tuple[int, int]


class TaskStatus(str, Enum):
    WAITING = "WAITING"
    BIDDING = "BIDDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass
class Task:
    task_id: str
    pickup_location: Coordinate
    dropoff_location: Coordinate
    priority: float = 0.5           # 0..1, higher = more urgent
    status: TaskStatus = TaskStatus.WAITING
    assigned_robot: Optional[str] = None
    created_time: float = 0.0       # simulated-clock time the task was created (0.0 = "at sim start")
    assigned_time: Optional[float] = None  # simulated-clock time it moved out of WAITING
    deadline: Optional[float] = None  # simulated-clock deadline, optional

    def age(self, now: float) -> float:
        return now - self.created_time

    def is_overdue(self, now: float) -> bool:
        if self.deadline is None:
            return False
        return now > self.deadline

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "pickup_location": list(self.pickup_location),
            "dropoff_location": list(self.dropoff_location),
            "priority": self.priority,
            "status": self.status.value,
            "assigned_robot": self.assigned_robot,
            "created_time": self.created_time,
            "assigned_time": self.assigned_time,
            "deadline": self.deadline,
        }

    @staticmethod
    def from_dict(d: dict) -> "Task":
        return Task(
            task_id=d["task_id"],
            pickup_location=tuple(d["pickup_location"]),
            dropoff_location=tuple(d["dropoff_location"]),
            priority=d.get("priority", 0.5),
            status=TaskStatus(d.get("status", "WAITING")),
            assigned_robot=d.get("assigned_robot"),
            created_time=d.get("created_time", 0.0),
            deadline=d.get("deadline"),
        )
