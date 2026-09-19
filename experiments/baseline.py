"""
experiments/baseline.py
==========================
The traditional "stop-and-wait" baseline, per project spec section 26.

Behaviour: if another robot is within a conflict range while this robot
is moving, STOP completely and wait until the area is fully clear, then
continue. No priority comparison, no negotiation, no re-routing around
the other robot - just a hard stop. This is the naive strategy the
decentralized system (robots/robot_controller.py) is compared against in
experiments/benchmark.py to measure the actual improvement percentage.

Implemented as a thin alternative movement policy that reuses the same
Robot/RobotState/Warehouse plumbing as the decentralized system (A* for
the initial path, LocalWorldModel for peer positions) but replaces
conflict resolution with an unconditional stop.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import config
from planning.astar import find_path
from robots.robot_state import RobotState, RobotStatus, LocalWorldModel
from simulation.physics import euclidean_distance, velocity_towards
from simulation.warehouse import Warehouse

Coordinate = Tuple[int, int]

logger = logging.getLogger("baseline")

# How far ahead a stop-and-wait robot looks for a conflicting peer before
# deciding to freeze. Deliberately generous (compared to SAFE_DISTANCE)
# since this controller has no negotiation to fall back on - it must be
# conservative to guarantee zero collisions with zero coordination.
STOP_AND_WAIT_CONFLICT_RANGE = config.SAFE_DISTANCE + 1.5


class StopAndWaitController:
    """Drives ONE robot using the naive stop-and-wait policy.

    Owns the same RobotState/LocalWorldModel shapes as
    robots/robot_controller.py's RobotController so its metrics
    (distance_travelled, waiting_time, etc.) are directly comparable.
    """

    def __init__(self, robot_id: str, warehouse: Warehouse,
                 speed: float = config.ROBOT_SPEED_CELLS_PER_SEC) -> None:
        self.robot_id = robot_id
        self.warehouse = warehouse
        self.speed = speed
        self._step_progress = 0.0

    def close(self) -> None:
        """No-op: StopAndWaitController has no network socket to close.
        Present so callers (main.py, benchmark.py) can treat it uniformly
        with the decentralized Robot's close()."""
        pass

    def tick(self, state: RobotState, world_model: LocalWorldModel, dt: float, now: float) -> None:
        if state.status == RobotStatus.FAILED:
            return
        if state.destination is None or state.position == state.destination:
            if state.status == RobotStatus.MOVING or state.status == RobotStatus.WAITING:
                state.status = RobotStatus.COMPLETED
                state.velocity = (0.0, 0.0)
                state.task_completion_time = now
            return

        if not state.current_path or state.current_path[0] not in self.warehouse.neighbors(state.position) + [state.position]:
            path = find_path(state.position, state.destination, self.warehouse)
            if path is None:
                state.status = RobotStatus.BLOCKED
                state.start_waiting(now)
                return
            state.planned_path = path
            state.current_path = path[1:]

        if not state.current_path:
            return

        next_cell = state.current_path[0]

        # Stop-and-wait's entire "coordination" strategy: if ANY trusted
        # peer is within STOP_AND_WAIT_CONFLICT_RANGE of either our
        # current position or our next intended cell, freeze completely.
        conflict = any(
            euclidean_distance(next_cell, peer.position) < STOP_AND_WAIT_CONFLICT_RANGE
            or euclidean_distance(state.position, peer.position) < STOP_AND_WAIT_CONFLICT_RANGE
            for peer in world_model.get_active_peers(now, config.ROBOT_TIMEOUT)
            if peer.robot_id != self.robot_id and peer.status != RobotStatus.FAILED
        )

        if conflict:
            state.start_waiting(now)
            state.status = RobotStatus.WAITING
            state.velocity = (0.0, 0.0)
            return

        state.status = RobotStatus.MOVING
        state.stop_waiting(now)

        self._step_progress += self.speed * dt
        state.velocity = velocity_towards(state.position, next_cell, self.speed)

        if self._step_progress >= 1.0:
            self._step_progress = 0.0
            state.previous_position = state.position
            state.position = next_cell
            state.current_path = state.current_path[1:]
            state.distance_travelled += 1.0
            state.battery = max(0.0, state.battery - config.ENERGY_PER_CELL)

            if state.position == state.destination:
                state.status = RobotStatus.COMPLETED
                state.velocity = (0.0, 0.0)
                state.task_completion_time = now
