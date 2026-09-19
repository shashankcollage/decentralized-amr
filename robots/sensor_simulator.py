"""
robots/sensor_simulator.py
=============================
Virtual sensor simulation, per project spec section 9.

Detects (within config.SENSOR_RANGE cells of the robot's own position):
  * static obstacles (walls/shelves)
  * dynamic obstacles (blocked aisles)
  * nearby robots (from the LocalWorldModel, i.e. only peers we've
    actually heard from - a robot cannot "sense" a peer whose messages
    it never received, which is a deliberately realistic constraint)

This class is written as an abstraction specifically so a real
implementation (LIDAR, depth camera, etc.) could later replace it without
changing its callers (robots/robot_controller.py) - only this file would
need to change for a hardware deployment, per project spec section 9's
"do not use real hardware-specific APIs" + "create an abstraction" ask.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import config
from simulation.warehouse import Warehouse
from simulation.physics import euclidean_distance
from robots.robot_state import LocalWorldModel, PeerInfo

Coordinate = Tuple[int, int]


@dataclass
class SensorReading:
    nearby_static_obstacles: List[Coordinate]
    nearby_dynamic_obstacles: List[Coordinate]
    nearby_robots: List[PeerInfo]


class SensorSimulator:
    def __init__(self, sensor_range: int = config.SENSOR_RANGE) -> None:
        self.sensor_range = sensor_range

    def scan(self, self_position: Coordinate, warehouse: Warehouse,
             world_model: LocalWorldModel, now: float, timeout: float,
             self_robot_id: str) -> SensorReading:
        static_obstacles = []
        dynamic_obstacles = []

        x0, y0 = self_position
        r = self.sensor_range
        for x in range(x0 - r, x0 + r + 1):
            for y in range(y0 - r, y0 + r + 1):
                cell = (x, y)
                if not warehouse.in_bounds(cell):
                    continue
                if euclidean_distance(self_position, cell) > r:
                    continue
                if warehouse.is_statically_blocked(cell):
                    static_obstacles.append(cell)
                elif warehouse.is_dynamically_blocked(cell):
                    dynamic_obstacles.append(cell)

        nearby_robots = [
            peer for peer in world_model.get_active_peers(now, timeout)
            if peer.robot_id != self_robot_id and euclidean_distance(self_position, peer.position) <= r
        ]

        return SensorReading(
            nearby_static_obstacles=static_obstacles,
            nearby_dynamic_obstacles=dynamic_obstacles,
            nearby_robots=nearby_robots,
        )

    def detects_blocked_aisle_ahead(self, reading: SensorReading, path_ahead: List[Coordinate]) -> bool:
        """Convenience check: does any cell on the robot's immediate
        upcoming path (already-planned) show up as a dynamic obstacle in
        this sensor reading? This is what triggers re-routing (see
        planning/rerouting.py's needs_reroute, which checks the whole
        warehouse state directly - this sensor-based version instead
        models "the robot only reacts to obstacles within sensor range",
        a more realistic trigger condition for when re-routing kicks in).
        """
        blocked_set = set(reading.nearby_dynamic_obstacles)
        return any(cell in blocked_set for cell in path_ahead)
