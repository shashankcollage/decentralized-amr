"""
coordination/conflict_resolution.py
======================================
Turns a detected Conflict (planning/conflict_detector.py) into a decision:
should THIS robot wait, or is it clear to proceed? Implements the flow
from project spec section 15:

    1. determine conflicting robots        <- caller (conflict_detector)
    2. compare priorities                  <- this module
    3. check reservation ownership         <- this module
    4. determine whether one robot waits   <- this module
    5. check alternate path                <- caller (rerouting)
    6. negotiate if necessary              <- coordination/negotiation.py
    7. re-route one robot if needed        <- caller (rerouting)
    8. continue movement                   <- caller (robot_controller)

DESIGN NOTE: this module takes already-computed priority SCORES (floats),
not raw PriorityInputs, because in practice a robot only ever has its own
full PriorityInputs available - a peer's priority score arrives as an
already-computed number in its ROBOT_STATE broadcast (see
robots/robot_state.py's `priority` field and coordination/priority.py's
calculate_priority(), which each robot runs once per tick on itself and
broadcasts the result). This keeps the interface honest about what a
decentralized robot actually knows about its peers.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from coordination.priority import higher_priority_robot
from planning.conflict_detector import Conflict
from planning.reservation_table import ReservationTable

Coordinate = Tuple[int, int]


class Resolution(str, Enum):
    PROCEED = "PROCEED"
    YIELD = "YIELD"


@dataclass
class ResolutionDecision:
    resolution: Resolution
    reason: str
    other_robot_id: str


def resolve_conflict(
    self_robot_id: str,
    conflict: Conflict,
    self_priority_score: float,
    other_priority_score: float,
    reservations: Optional[ReservationTable] = None,
    contested_cell: Optional[Coordinate] = None,
    contested_timestep: Optional[int] = None,
) -> ResolutionDecision:
    """Decide whether `self_robot_id` should proceed or yield given a
    single detected conflict with `conflict.other_robot_id`.

    Resolution order (per spec section 15):
      1. If the contested cell is already reservation-owned by one of the
         two robots, that ownership wins outright (first-come-first-served
         - avoids priority flip-flopping mid-negotiation).
      2. Otherwise, compare priority scores; higher priority proceeds.
      3. Exact ties are broken deterministically by robot ID (see
         coordination/priority.py) - never randomly.
    """
    other_id = conflict.other_robot_id

    if reservations is not None and contested_cell is not None and contested_timestep is not None:
        owner = reservations.get_reservation_owner(contested_cell, contested_timestep)
        if owner == self_robot_id:
            return ResolutionDecision(Resolution.PROCEED, "reservation_owned_by_self", other_id)
        if owner == other_id:
            return ResolutionDecision(Resolution.YIELD, "reservation_owned_by_other", other_id)

    winner = higher_priority_robot(self_robot_id, self_priority_score, other_id, other_priority_score)

    if winner == self_robot_id:
        return ResolutionDecision(Resolution.PROCEED, "higher_priority", other_id)
    return ResolutionDecision(Resolution.YIELD, "lower_priority", other_id)
