"""
planning/reservation_table.py
===============================
A time-indexed reservation table: (cell, timestep) -> owning robot_id.

IMPORTANT ARCHITECTURAL NOTE (decentralization):
Each Robot owns its OWN instance of this table. It is never a single
shared/centralized structure that "decides" anything. A robot populates
its table from two sources only:
  1. its own planned path (reserve_path, called on itself)
  2. PATH_INTENT / ROBOT_STATE messages received from peers (see
     communication/message.py and robots/robot_controller.py), which
     update the table with what OTHER robots claim to intend

Because every robot builds the same kind of table from broadcast intents,
the fleet converges on a consistent (if sometimes briefly stale) shared
understanding without any robot being a designated authority - the same
principle as the LocalWorldModel in robots/robot_state.py.

Discretization: "timestep" here is an integer step index (not wall-clock
seconds). Robot speed is assumed uniform (config.ROBOT_SPEED_CELLS_PER_SEC)
so step index i corresponds to approximately i / speed seconds - see
planning/multi_agent_planner.py for the conversion. This is a deliberate
simplification: real variable-speed timing would need continuous-time
reservations, which is unnecessary complexity for this grid simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Coordinate = Tuple[int, int]
ReservationKey = Tuple[Coordinate, int]  # (cell, timestep)


@dataclass
class ReservationTable:
    """Maps (cell, timestep) -> robot_id claiming that cell at that step."""

    _table: Dict[ReservationKey, str] = field(default_factory=dict)
    # Tracks which reservation keys belong to which robot, so a whole path
    # can be released in one call without scanning the entire table.
    _by_robot: Dict[str, set] = field(default_factory=dict)

    def is_reserved(self, cell: Coordinate, timestep: int, exclude_robot: Optional[str] = None) -> bool:
        owner = self._table.get((cell, timestep))
        if owner is None:
            return False
        if exclude_robot is not None and owner == exclude_robot:
            return False
        return True

    def get_reservation_owner(self, cell: Coordinate, timestep: int) -> Optional[str]:
        return self._table.get((cell, timestep))

    def reserve_path(self, robot_id: str, path: List[Coordinate], start_timestep: int = 0) -> bool:
        """Reserve every (cell, timestep) pair along `path`, where the robot
        occupies path[i] at timestep (start_timestep + i).

        Returns False (and reserves nothing) if any cell/timestep in the
        path is already reserved by a DIFFERENT robot - callers should plan
        around existing reservations (see multi_agent_planner) so this
        should rarely fail, but the check is kept here as a safety net
        matching the project's "never allow two robots to intentionally
        occupy the same cell simultaneously" invariant.
        """
        new_keys: List[ReservationKey] = []
        for i, cell in enumerate(path):
            t = start_timestep + i
            owner = self._table.get((cell, t))
            if owner is not None and owner != robot_id:
                return False
            new_keys.append((cell, t))

        for key in new_keys:
            self._table[key] = robot_id
        self._by_robot.setdefault(robot_id, set()).update(new_keys)
        return True

    def release_path(self, robot_id: str) -> None:
        """Release every reservation currently held by `robot_id`."""
        keys = self._by_robot.pop(robot_id, set())
        for key in keys:
            if self._table.get(key) == robot_id:
                del self._table[key]

    def release_robot_claims_from_peer(self, robot_id: str) -> None:
        """Alias of release_path, used when we learn (via message or
        timeout) that a peer's claims should no longer be trusted, e.g.
        after it fails or its reservations expire."""
        self.release_path(robot_id)

    def update_peer_path(self, robot_id: str, path: List[Coordinate], start_timestep: int = 0) -> None:
        """Overwrite what we believe robot_id has reserved, based on a
        PATH_INTENT/ROBOT_STATE message just received from it. This does
        NOT perform conflict checking (we trust the peer's own claim about
        its own path) - conflict checking happens when the LOCAL robot
        tries to reserve ITS OWN path against this table.
        """
        self.release_path(robot_id)
        keys = [((cell), start_timestep + i) for i, cell in enumerate(path)]
        for key in keys:
            self._table[key] = robot_id
        self._by_robot.setdefault(robot_id, set()).update(keys)

    def find_conflicts(self, robot_id: str, path: List[Coordinate],
                        start_timestep: int = 0) -> List[Tuple[int, Coordinate, str]]:
        """Return a list of (timestep, cell, conflicting_robot_id) for every
        step of `path` that is already claimed by a different robot."""
        conflicts = []
        for i, cell in enumerate(path):
            t = start_timestep + i
            owner = self._table.get((cell, t))
            if owner is not None and owner != robot_id:
                conflicts.append((t, cell, owner))
        return conflicts

    def clear(self) -> None:
        self._table.clear()
        self._by_robot.clear()
