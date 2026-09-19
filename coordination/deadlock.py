"""
coordination/deadlock.py
==========================
Deadlock detection per project spec section 18:

    conditions:
      * robot waiting beyond WAIT_TIMEOUT
      * circular dependency (cycle in the wait-for graph)
      * no available path
      * repeated conflict

    when detected:
      1. identify involved robots
      2. calculate priorities
      3. select a robot to back off
      4. release its reservation if appropriate
      5. re-plan its path
      6. broadcast DEADLOCK_ALERT
      7. continue operation

This module implements steps 1-3 (detection + who-backs-off selection).
Steps 4-6 are carried out by the caller (robots/robot_controller.py),
which owns the reservation table and the network manager needed to
actually release a reservation and broadcast the alert.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import config
from coordination.priority import PriorityInputs, calculate_priority
from coordination.wait_for_graph import build_wait_for_graph, find_cycles, cycle_containing


@dataclass
class DeadlockInfo:
    cycle: List[str]
    robot_to_back_off: str


def is_suspiciously_long_wait(waiting_duration: float, wait_timeout: float = config.WAIT_TIMEOUT) -> bool:
    """A robot waiting longer than WAIT_TIMEOUT is a necessary (but not
    sufficient on its own) condition for suspecting deadlock - it's what
    triggers a robot to even bother checking the wait-for graph, rather
    than checking on every single tick regardless of state.
    """
    return waiting_duration >= wait_timeout


def detect_deadlock(
    self_robot_id: str,
    waiting_for: Dict[str, Optional[str]],
    priority_inputs_by_robot: Dict[str, PriorityInputs],
) -> Optional[DeadlockInfo]:
    """Build the wait-for graph from currently-known waiting relationships
    and check whether `self_robot_id` is part of a cycle.

    If a cycle is found, also determines which robot in that cycle should
    back off: the one with the LOWEST priority score (ties broken by
    robot ID), so exactly one deterministic robot backs off fleet-wide -
    critical so that two robots in the same cycle don't both decide "it's
    the other one" and neither backs off, or both back off unnecessarily.

    Returns None if this robot is not currently part of any detected
    cycle.
    """
    graph = build_wait_for_graph(waiting_for)
    cycles = find_cycles(graph)
    cycle = cycle_containing(cycles, self_robot_id)
    if cycle is None:
        return None

    # Compute a priority score for every robot in the cycle using
    # whatever inputs we have locally available (from LocalWorldModel);
    # a robot missing from priority_inputs_by_robot is treated as lowest
    # possible priority (0.0) so it's picked to back off rather than
    # crashing on missing data.
    scored = [(rid, calculate_priority(priority_inputs_by_robot[rid])
               if rid in priority_inputs_by_robot else 0.0)
              for rid in cycle]

    # Lowest priority backs off; tie-break deterministically by robot id.
    scored.sort(key=lambda pair: (pair[1], pair[0]))
    robot_to_back_off = scored[0][0]

    return DeadlockInfo(cycle=cycle, robot_to_back_off=robot_to_back_off)
