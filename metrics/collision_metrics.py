"""
metrics/collision_metrics.py
===============================
The safety-invariant checker required by project spec section 36:
"never allow two robots to occupy the same physical cell at the same
timestep. If a collision occurs unexpectedly: record it, stop affected
robots, display an alert... Do not hide collisions from metrics."

This is deliberately independent of, and in addition to, each robot's
own local collision avoidance (planning/conflict_detector.py) - it's the
audit layer that would catch a bug in that logic, not a replacement for
it. Attach via Simulator.register_tick_callback like the other observers.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

Coordinate = Tuple[int, int]

logger = logging.getLogger("collision_metrics")


@dataclass
class CollisionEvent:
    tick: int
    sim_time: float
    cell: Coordinate
    robot_ids: List[str]


@dataclass
class CollisionMonitor:
    on_collision: Optional[Callable[[CollisionEvent], None]] = None
    events: List[CollisionEvent] = field(default_factory=list)
    stop_on_collision: bool = False

    def on_tick(self, sim) -> None:
        positions: Dict[Coordinate, List[str]] = {}
        for rid, robot in sim.robots.items():
            if robot.status.value in ("FAILED", "COMPLETED"):
                continue
            positions.setdefault(robot.position, []).append(rid)

        for cell, robot_ids in positions.items():
            if len(robot_ids) > 1:
                event = CollisionEvent(tick=sim.tick_count, sim_time=sim.sim_time,
                                        cell=cell, robot_ids=robot_ids)
                self.events.append(event)
                logger.error("UNEXPECTED COLLISION at tick %d (t=%.1fs): robots %s at cell %s",
                             sim.tick_count, sim.sim_time, robot_ids, cell)
                if self.on_collision:
                    self.on_collision(event)
                if self.stop_on_collision:
                    for rid in robot_ids:
                        sim.robots[rid].state.velocity = (0.0, 0.0)
                        sim.robots[rid].state.current_path = []

    def collision_count(self) -> int:
        return len(self.events)
