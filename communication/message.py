"""
communication/message.py
==========================
Defines every message type exchanged between robots (project spec
section 10) and the Message dataclass used to build/parse them.

Messages are plain JSON over UDP (see communication/peer.py). No message
type implies or requires a central relay - every message is either
broadcast to all known peers or (for negotiation/response types) sent
directly to one peer's known address.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

Coordinate = Tuple[int, int]


class MessageType(str, Enum):
    ROBOT_STATE = "ROBOT_STATE"
    HEARTBEAT = "HEARTBEAT"
    PATH_INTENT = "PATH_INTENT"
    TASK_BID = "TASK_BID"
    TASK_ASSIGNMENT = "TASK_ASSIGNMENT"
    CONFLICT_REQUEST = "CONFLICT_REQUEST"
    CONFLICT_RESPONSE = "CONFLICT_RESPONSE"
    RESERVATION_REQUEST = "RESERVATION_REQUEST"
    RESERVATION_RESPONSE = "RESERVATION_RESPONSE"
    DEADLOCK_ALERT = "DEADLOCK_ALERT"
    REROUTE_NOTICE = "REROUTE_NOTICE"
    TASK_COMPLETED = "TASK_COMPLETED"
    ROBOT_FAILURE = "ROBOT_FAILURE"


@dataclass
class Message:
    """A single robot-to-robot message.

    `payload` holds type-specific fields (e.g. a ROBOT_STATE message's
    payload has position/velocity/battery/etc; a TASK_BID's payload has
    task_id/bid_cost). Keeping a generic payload dict (rather than one
    dataclass per message type) keeps the wire format simple and matches
    the flat JSON example given in the project spec.
    """

    type: MessageType
    robot_id: str
    timestamp: float
    payload: Dict[str, Any] = field(default_factory=dict)
    signature: Optional[str] = None  # HMAC hex digest, filled in by protocol.py

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = self.type.value
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Message":
        return Message(
            type=MessageType(d["type"]),
            robot_id=d["robot_id"],
            timestamp=d["timestamp"],
            payload=d.get("payload", {}),
            signature=d.get("signature"),
        )

    @staticmethod
    def from_json(raw: str) -> "Message":
        return Message.from_dict(json.loads(raw))


def make_robot_state_message(
    robot_id: str,
    position: Coordinate,
    velocity: Tuple[float, float],
    destination: Optional[Coordinate],
    battery: float,
    status: str,
    priority: float,
    path: List[Coordinate],
    current_task: Optional[str],
    waiting_for: Optional[str] = None,
    timestamp: Optional[float] = None,
) -> Message:
    """Build the ROBOT_STATE message every robot broadcasts periodically,
    matching the exact JSON schema given in project spec section 10.
    """
    return Message(
        type=MessageType.ROBOT_STATE,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={
            "position": list(position),
            "velocity": list(velocity),
            "destination": list(destination) if destination else None,
            "battery": battery,
            "status": status,
            "priority": priority,
            "path": [list(c) for c in path],
            "current_task": current_task,
            "waiting_for": waiting_for,
        },
    )


def make_task_bid_message(robot_id: str, task_id: str, bid_cost: float,
                           timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.TASK_BID,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"task_id": task_id, "bid_cost": bid_cost},
    )


def make_task_assignment_message(robot_id: str, task_id: str, winner_robot_id: str,
                                  timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.TASK_ASSIGNMENT,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"task_id": task_id, "winner": winner_robot_id},
    )


def make_task_completed_message(robot_id: str, task_id: str,
                                 timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.TASK_COMPLETED,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"task_id": task_id},
    )


def make_deadlock_alert_message(robot_id: str, cycle: List[str], backing_off: str,
                                 timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.DEADLOCK_ALERT,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"cycle": cycle, "backing_off": backing_off},
    )


def make_reroute_notice_message(robot_id: str, new_path: List[Coordinate],
                                 timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.REROUTE_NOTICE,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"new_path": [list(c) for c in new_path]},
    )


def make_robot_failure_message(robot_id: str, last_position: Coordinate,
                                timestamp: Optional[float] = None) -> Message:
    return Message(
        type=MessageType.ROBOT_FAILURE,
        robot_id=robot_id,
        timestamp=(timestamp if timestamp is not None else time.time()),
        payload={"last_position": list(last_position)},
    )
