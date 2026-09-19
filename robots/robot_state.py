"""
robots/robot_state.py
======================
Pure data structures describing a robot's own state and its local model of
the world (what it currently believes about its peers, based on messages
received so far). No decision logic lives here - only data + small helpers.

Keeping this file dependency-free (no sockets, no pygame, no Flask) is what
allows it to run unmodified on an edge device such as a Raspberry Pi.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

Coordinate = Tuple[int, int]


class RobotStatus(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    WAITING = "WAITING"
    NEGOTIATING = "NEGOTIATING"
    REROUTING = "REROUTING"
    BLOCKED = "BLOCKED"
    CHARGING = "CHARGING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class BatteryState(str, Enum):
    NORMAL = "NORMAL"
    LOW = "LOW"
    CRITICAL = "CRITICAL"


@dataclass
class RobotState:
    """The full local state of a single robot (its own edge-node state).

    This is the data a robot broadcasts (a subset, see communication/message.py)
    and the data it uses internally to make decisions.
    """

    robot_id: str
    position: Coordinate
    previous_position: Coordinate
    velocity: Tuple[float, float] = (0.0, 0.0)
    destination: Optional[Coordinate] = None
    current_task: Optional[str] = None

    battery: float = 100.0
    status: RobotStatus = RobotStatus.IDLE

    current_path: List[Coordinate] = field(default_factory=list)   # remaining path to walk
    planned_path: List[Coordinate] = field(default_factory=list)   # full path as last planned

    priority: float = 0.0
    waiting_time: float = 0.0          # seconds continuously spent waiting
    wait_start_time: Optional[float] = None

    completed_tasks: int = 0
    collision_count: int = 0
    distance_travelled: float = 0.0

    task_start_time: Optional[float] = None
    task_completion_time: Optional[float] = None

    # Bookkeeping, not part of the "official" required-field list but needed
    # for realistic operation:
    total_waiting_time: float = 0.0
    reroute_count: int = 0
    last_heartbeat_sent: float = 0.0
    creation_time: float = field(default_factory=time.time)

    def battery_state(self, low_threshold: float, critical_threshold: float) -> BatteryState:
        if self.battery <= critical_threshold:
            return BatteryState.CRITICAL
        if self.battery <= low_threshold:
            return BatteryState.LOW
        return BatteryState.NORMAL

    def start_waiting(self, now: float) -> None:
        if self.wait_start_time is None:
            self.wait_start_time = now
        self.status = RobotStatus.WAITING

    def stop_waiting(self, now: float) -> None:
        if self.wait_start_time is not None:
            elapsed = now - self.wait_start_time
            self.waiting_time = 0.0
            self.total_waiting_time += elapsed
            self.wait_start_time = None

    def current_waiting_duration(self, now: float) -> float:
        if self.wait_start_time is None:
            return 0.0
        return now - self.wait_start_time

    def to_public_dict(self) -> dict:
        """A JSON-serializable snapshot suitable for broadcasting or dashboard display."""
        return {
            "robot_id": self.robot_id,
            "position": list(self.position),
            "previous_position": list(self.previous_position),
            "velocity": list(self.velocity),
            "destination": list(self.destination) if self.destination else None,
            "current_task": self.current_task,
            "battery": round(self.battery, 2),
            "status": self.status.value,
            "current_path": [list(p) for p in self.current_path],
            "priority": round(self.priority, 3),
            "waiting_time": round(self.current_waiting_duration(time.time()), 2),
            "completed_tasks": self.completed_tasks,
            "collision_count": self.collision_count,
            "distance_travelled": round(self.distance_travelled, 2),
        }


@dataclass
class PeerInfo:
    """What a robot remembers about ONE other robot, derived purely from
    messages it has received (or their absence)."""

    robot_id: str
    position: Coordinate
    velocity: Tuple[float, float] = (0.0, 0.0)
    destination: Optional[Coordinate] = None
    status: RobotStatus = RobotStatus.IDLE
    battery: float = 100.0
    path: List[Coordinate] = field(default_factory=list)
    priority: float = 0.0
    current_task: Optional[str] = None
    waiting_for: Optional[str] = None
    last_message_time: float = 0.0

    def is_stale(self, now: float, timeout: float) -> bool:
        return (now - self.last_message_time) > timeout


@dataclass
class LocalWorldModel:
    """A robot's local, eventually-consistent view of its peers.

    This is intentionally NOT a global truth - it is built up only from
    messages actually received by this robot, which is what makes the
    architecture decentralized: no robot ever has guaranteed complete or
    perfectly current information about the fleet.
    """

    peers: Dict[str, PeerInfo] = field(default_factory=dict)

    def update_peer(self, robot_id: str, **kwargs) -> None:
        if robot_id not in self.peers:
            self.peers[robot_id] = PeerInfo(
                robot_id=robot_id,
                position=kwargs.get("position", (0, 0)),
            )
        peer = self.peers[robot_id]
        for key, value in kwargs.items():
            if hasattr(peer, key):
                setattr(peer, key, value)

    def get_active_peers(self, now: float, timeout: float) -> List[PeerInfo]:
        """Peers we've heard from recently enough to trust."""
        return [p for p in self.peers.values() if not p.is_stale(now, timeout)]

    def get_stale_peers(self, now: float, timeout: float) -> List[PeerInfo]:
        return [p for p in self.peers.values() if p.is_stale(now, timeout)]

    def mark_failed(self, robot_id: str) -> None:
        if robot_id in self.peers:
            self.peers[robot_id].status = RobotStatus.FAILED

    def occupied_cells(self, now: float, timeout: float, exclude: Optional[str] = None) -> Dict[Coordinate, str]:
        """Map of cell -> robot_id for all currently-trusted peer positions."""
        result = {}
        for p in self.get_active_peers(now, timeout):
            if p.robot_id == exclude:
                continue
            if p.status != RobotStatus.FAILED:
                result[p.position] = p.robot_id
        return result
