"""
planning/conflict_detector.py
================================
Given what a robot knows about itself (its own planned path) and its
peers (via LocalWorldModel, built from received messages), detect
potential conflicts before they happen.

Two families of checks, matching project spec section 14:

A. SPATIAL collision - are two robots currently too close right now
   (euclidean distance < SAFE_DISTANCE)? Used as an immediate safety
   check independent of any plan.

B. PREDICTED collision - do a robot's own next planned steps collide
   with a peer's claimed/known path at the same timestep? Subdivided
   into head-on, same-cell, following, and intersection conflicts (see
   simulation/physics.py for the underlying geometric predicates).

This module only detects and classifies - it never decides who yields;
that's coordination/conflict_resolution.py's job, kept deliberately
separate so priority logic can be unit-tested and reasoned about on its
own.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import config
from simulation.physics import (
    euclidean_distance,
    is_head_on,
    is_same_cell_conflict,
    is_following_conflict,
)
from robots.robot_state import LocalWorldModel, PeerInfo

Coordinate = Tuple[int, int]


class ConflictType(str, Enum):
    SPATIAL = "SPATIAL"                # too close right now
    SAME_CELL = "SAME_CELL"            # both want the identical next cell
    HEAD_ON = "HEAD_ON"                # swapping cells with each other
    FOLLOWING = "FOLLOWING"            # moving into a cell the peer currently occupies
    INTERSECTION = "INTERSECTION"      # both approaching a flagged intersection cell


@dataclass
class Conflict:
    conflict_type: ConflictType
    other_robot_id: str
    cell: Optional[Coordinate] = None

    def __repr__(self) -> str:
        return f"Conflict({self.conflict_type.value} with {self.other_robot_id} at {self.cell})"


def detect_spatial_collisions(self_position: Coordinate, world_model: LocalWorldModel,
                                now: float, timeout: float,
                                safe_distance: float = config.SAFE_DISTANCE) -> List[Conflict]:
    """Check current euclidean proximity against every trusted peer."""
    conflicts = []
    for peer in world_model.get_active_peers(now, timeout):
        d = euclidean_distance(self_position, peer.position)
        if d < safe_distance:
            conflicts.append(Conflict(ConflictType.SPATIAL, peer.robot_id, peer.position))
    return conflicts


def detect_predicted_conflicts(
    self_robot_id: str,
    self_position: Coordinate,
    self_next_cell: Optional[Coordinate],
    world_model: LocalWorldModel,
    now: float,
    timeout: float,
) -> List[Conflict]:
    """Check whether moving to `self_next_cell` this tick would conflict
    with any trusted peer's reported position or announced next step.

    Each peer's "next cell" is approximated as the first entry of its
    known `path` (see PeerInfo.path, populated from PATH_INTENT/ROBOT_STATE
    messages) if available, else its current position (i.e. assume it
    might stay put - the conservative assumption).
    """
    if self_next_cell is None:
        return []

    conflicts: List[Conflict] = []
    for peer in world_model.get_active_peers(now, timeout):
        if peer.robot_id == self_robot_id:
            continue

        peer_next = peer.path[0] if peer.path else peer.position

        if is_same_cell_conflict(self_next_cell, peer_next):
            conflicts.append(Conflict(ConflictType.SAME_CELL, peer.robot_id, self_next_cell))
        elif is_head_on(self_position, self_next_cell, peer.position, peer_next):
            conflicts.append(Conflict(ConflictType.HEAD_ON, peer.robot_id, self_next_cell))
        elif is_following_conflict(self_position, self_next_cell, peer.position):
            conflicts.append(Conflict(ConflictType.FOLLOWING, peer.robot_id, self_next_cell))

    return conflicts


def detect_intersection_conflicts(
    self_robot_id: str,
    self_next_cell: Optional[Coordinate],
    intersections: List[Coordinate],
    world_model: LocalWorldModel,
    now: float,
    timeout: float,
) -> List[Conflict]:
    """Flag when both this robot and a peer are about to enter the same
    flagged intersection cell (a stricter, earlier-warning version of
    SAME_CELL specifically for choke points, per project spec section 17).
    """
    if self_next_cell is None or self_next_cell not in intersections:
        return []

    conflicts = []
    for peer in world_model.get_active_peers(now, timeout):
        if peer.robot_id == self_robot_id:
            continue
        peer_next = peer.path[0] if peer.path else peer.position
        if peer_next == self_next_cell:
            conflicts.append(Conflict(ConflictType.INTERSECTION, peer.robot_id, self_next_cell))
    return conflicts
