"""
coordination/negotiation.py
==============================
Intersection/choke-point negotiation, per project spec section 17:

    1. predict arrival time
    2. broadcast intent
    3. check other robot intents
    4. request reservation
    5. receive responses
    6. resolve conflict
    7. enter intersection only after permission/reservation
    8. release reservation after leaving

IMPLEMENTATION NOTE / DOCUMENTED SIMPLIFICATION:
Steps 2-6 are collapsed into a single deterministic computation instead of
a literal multi-round CONFLICT_REQUEST/CONFLICT_RESPONSE message exchange
with retries and timeouts. This is possible because every robot broadcasts
its full intended path (PATH_INTENT/ROBOT_STATE) every heartbeat, and
conflict resolution (coordination/conflict_resolution.py) is a *pure,
deterministic* function of two robots' priority scores - so as soon as
both robots have received each other's latest broadcast, they
independently compute the SAME outcome without needing to literally wait
for each other's answer. This preserves the "only proceeds after
permission" safety property (a robot won't move into a contested
intersection until it has a definite PROCEED result) while avoiding the
substantial extra complexity of a stateful multi-round handshake with
timeouts and retransmission, which would not materially change which
robot goes first in this simulation.

A production system with real (non-negligible) network latency SHOULD
use the literal request/response round trip that
communication/message.py's CONFLICT_REQUEST/CONFLICT_RESPONSE and
RESERVATION_REQUEST/RESERVATION_RESPONSE message types exist to support;
this module's `predict_arrival_timestep` and `should_wait_for_intersection`
are written so that swapping in a real handshake later only changes how
`other_priority_score`/reservation ownership are obtained, not this
module's decision logic itself.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from coordination.conflict_resolution import resolve_conflict, ResolutionDecision
from planning.conflict_detector import Conflict, ConflictType
from planning.reservation_table import ReservationTable

Coordinate = Tuple[int, int]


def predict_arrival_timestep(current_step_index: int, path: list, intersection_cell: Coordinate) -> Optional[int]:
    """Given a robot's remaining path, predict which timestep it expects
    to arrive at `intersection_cell` (or None if that cell isn't on the
    path at all)."""
    if intersection_cell not in path:
        return None
    return current_step_index + path.index(intersection_cell)


def should_wait_for_intersection(
    self_robot_id: str,
    conflict: Conflict,
    self_priority_score: float,
    other_priority_score: float,
    reservations: Optional[ReservationTable] = None,
    contested_timestep: Optional[int] = None,
) -> ResolutionDecision:
    """Resolve an INTERSECTION-type conflict specifically. Delegates to
    the same conflict_resolution machinery used for ordinary same-cell
    conflicts, since intersections are just cells with an extra flag
    (Warehouse.is_intersection) - the resolution rules don't need to
    differ, only the earlier trigger point (conflict_detector checks
    intersections one step further ahead) does.
    """
    assert conflict.conflict_type == ConflictType.INTERSECTION
    return resolve_conflict(
        self_robot_id=self_robot_id,
        conflict=conflict,
        self_priority_score=self_priority_score,
        other_priority_score=other_priority_score,
        reservations=reservations,
        contested_cell=conflict.cell,
        contested_timestep=contested_timestep,
    )
