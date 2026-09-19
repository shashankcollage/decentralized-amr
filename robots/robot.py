"""
robots/robot.py
================
The Robot class: a single autonomous mobile robot's outer shell.

As of Phase 2+, all real decision-making (planning, collision avoidance,
negotiation, deadlock detection, task bidding, battery/charging) lives in
robots/robot_controller.py's RobotController - this class is a thin,
stable holder of RobotState + LocalWorldModel that delegates every tick()
to its controller. This split matches the architecture diagram in the
project spec (Robot containing Localization/Sensor/Planner/etc. as
sub-modules) while keeping Robot's own public interface unchanged from
Phase 1, so existing callers (Simulator, dashboard, tests) don't need to
change.

BACKWARD-COMPATIBLE / STANDALONE MODE: if `all_robot_ids` is not given
(or has only one entry) and/or `enable_networking=False`, the robot runs
with no UDP socket at all and behaves exactly like Phase 1: it plans
paths with plain A* against the static warehouse only (no peer
awareness), and arriving at a destination sets status COMPLETED. This is
what the Phase 1 tests exercise, and it's also useful for quick
single-robot smoke tests. Pass `all_robot_ids` (the full fleet's IDs) and
`enable_networking=True` to get the full decentralized multi-robot stack.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from robots.robot_state import RobotState, RobotStatus, LocalWorldModel
from robots.robot_controller import RobotController
from simulation.warehouse import Warehouse
from tasks.task_manager import TaskManager

import config

Coordinate = Tuple[int, int]

logger = logging.getLogger("robot")


class Robot:
    def __init__(
        self,
        robot_id: str,
        start_position: Coordinate,
        warehouse: Warehouse,
        speed_cells_per_sec: float = config.ROBOT_SPEED_CELLS_PER_SEC,
        all_robot_ids: Optional[List[str]] = None,
        task_manager: Optional[TaskManager] = None,
        enable_networking: bool = False,
        port_offset: int = 0,
    ) -> None:
        self.warehouse = warehouse
        self.speed = speed_cells_per_sec

        self.state = RobotState(
            robot_id=robot_id,
            position=start_position,
            previous_position=start_position,
            battery=config.BATTERY_INITIAL,
            status=RobotStatus.IDLE,
        )
        self.world_model = LocalWorldModel()

        self.controller = RobotController(
            robot_id=robot_id,
            warehouse=warehouse,
            all_robot_ids=all_robot_ids,
            task_manager=task_manager,
            enable_networking=enable_networking,
            speed=speed_cells_per_sec,
            port_offset=port_offset,
        )

        logger.info("%s created at %s (networking=%s)", robot_id, start_position, self.controller.enable_networking)

    # -- convenience properties -----------------------------------------
    @property
    def robot_id(self) -> str:
        return self.state.robot_id

    @property
    def position(self) -> Coordinate:
        return self.state.position

    @property
    def status(self) -> RobotStatus:
        return self.state.status

    def is_active(self) -> bool:
        return self.state.status not in (RobotStatus.FAILED, RobotStatus.COMPLETED)

    # -- localization-style accessors --------------------------------------
    def get_position(self) -> Coordinate:
        return self.state.position

    def get_velocity(self) -> Tuple[float, float]:
        return self.state.velocity

    def distance_to(self, other: "Robot") -> float:
        from simulation.physics import euclidean_distance
        return euclidean_distance(self.position, other.position)

    # -- destination / task assignment -------------------------------------
    def set_destination(self, destination: Coordinate) -> None:
        """Directly assign a destination (standalone/no-task-manager mode,
        or used by scenario scripts to override task-driven movement)."""
        self.state.destination = destination
        if self.state.status == RobotStatus.IDLE:
            self.state.status = RobotStatus.MOVING

    def simulate_failure(self, now: Optional[float] = None) -> None:
        """Force this robot into FAILED status, for robot-failure scenario
        testing (project spec section 25)."""
        self.controller.simulate_failure(self.state, now=now)

    # -- per-tick update ---------------------------------------------------
    def tick(self, dt: float, now: float) -> None:
        if not self.is_active():
            return
        self.controller.tick(self.state, self.world_model, dt, now)

    def close(self) -> None:
        self.controller.close()

    def __repr__(self) -> str:
        return (f"Robot({self.robot_id}, pos={self.position}, "
                f"status={self.state.status.value}, battery={self.state.battery:.1f})")
