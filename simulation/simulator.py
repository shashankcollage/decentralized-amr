"""
simulation/simulator.py
========================
The Simulator drives the fixed-timestep loop described in the project spec
(section 29). In Phase 1 the loop is deliberately minimal:

    for each tick:
        for each robot: robot.tick(dt, now)
        sim_time += dt

Later phases insert the additional steps (message exchange, world-model
updates, conflict/deadlock detection, re-planning, metrics recording)
around this core loop WITHOUT changing its shape: the simulator remains a
pure orchestrator. It never decides where a robot goes - that stays inside
Robot / planning / coordination.
"""

from __future__ import annotations

import logging
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import config
from robots.robot import Robot
from robots.robot_state import RobotStatus
from simulation.warehouse import Warehouse

Coordinate = Tuple[int, int]

logger = logging.getLogger("simulator")


@dataclass
class SimulatorConfig:
    dt: float = config.DT
    max_sim_time: float = config.MAX_SIMULATION_TIME
    seed: int = config.RANDOM_SEED


class Simulator:
    """Owns the warehouse and the robot fleet, and advances simulated time.

    The simulator is deliberately dumb: it does not contain any collision
    avoidance, task-allocation, or negotiation logic. That logic lives
    inside each Robot so that the fleet's behaviour is genuinely
    decentralized even though, for convenience, it all runs in one
    Python process during simulation.
    """

    def __init__(self, warehouse: Warehouse, sim_config: Optional[SimulatorConfig] = None) -> None:
        self.warehouse = warehouse
        self.cfg = sim_config or SimulatorConfig()
        self.rng = random.Random(self.cfg.seed)

        self.robots: Dict[str, Robot] = {}
        self.sim_time: float = 0.0
        self.tick_count: int = 0
        self.running: bool = False
        self.paused: bool = False

        # Guards concurrent access when the simulator is driven by a
        # background thread (e.g. dashboard/app.py) while a Flask request
        # thread reads state via summary()/robot_positions(). The
        # simulator's own decision-making is unaffected either way - this
        # lock exists purely so a *reader* never sees a half-updated tick,
        # not to coordinate any robot behaviour.
        self.lock = threading.Lock()

        # Hooks other subsystems (dashboard, metrics) can attach to without
        # the simulator depending on them - keeps this module import-light.
        self.on_tick_callbacks: List[Callable[["Simulator"], None]] = []

    # -- fleet management --------------------------------------------------
    def add_robot(self, robot: Robot) -> None:
        self.robots[robot.robot_id] = robot

    def spawn_robots(self, num_robots: int, start_positions: Optional[List[Coordinate]] = None) -> None:
        """Create `num_robots` robots at either given or automatically chosen
        free cells."""
        if start_positions is None:
            start_positions = self._pick_free_cells(num_robots)
        if len(start_positions) < num_robots:
            raise ValueError(
                f"Not enough free cells to spawn {num_robots} robots "
                f"(found {len(start_positions)})."
            )
        for i in range(num_robots):
            robot_id = f"R{i + 1}"
            robot = Robot(robot_id=robot_id, start_position=start_positions[i], warehouse=self.warehouse)
            self.add_robot(robot)
        logger.info("Spawned %d robots: %s", num_robots, list(self.robots.keys()))

    def _pick_free_cells(self, n: int) -> List[Coordinate]:
        free = [pos for pos, cell in self.warehouse.grid.items() if self.warehouse.is_passable(pos)]
        self.rng.shuffle(free)
        return free[:n]

    # -- simulation control --------------------------------------------------
    def register_tick_callback(self, callback: Callable[["Simulator"], None]) -> None:
        self.on_tick_callbacks.append(callback)

    def step(self) -> None:
        """Advance the simulation by exactly one fixed timestep.

        Uses the accumulated SIMULATED clock (self.sim_time), not
        wall-clock time.time(), as the `now` passed to every robot. This
        is required for two things the project spec asks for: (1) true
        determinism given a fixed seed (real wall-clock timestamps are
        never reproducible between runs), and (2) headless/benchmark runs
        being able to execute far faster than real time while heartbeat
        intervals, bid windows, and wait-timeouts still behave exactly as
        configured in simulated seconds. In --dashboard's real_time=True
        mode, sim_time still tracks wall-clock closely because run() paces
        ticks with time.sleep(dt/speed) between them.

        Acquires ``self.lock`` for the duration of the tick so that a
        concurrent reader (e.g. a Flask dashboard route running on another
        thread) never observes a half-updated fleet state.
        """
        now = self.sim_time
        dt = self.cfg.dt

        with self.lock:
            for robot in self.robots.values():
                robot.tick(dt, now)

            self.sim_time += dt
            self.tick_count += 1

        for callback in self.on_tick_callbacks:
            callback(self)

    def run(self, max_ticks: Optional[int] = None, real_time: bool = False,
             speed: float = config.SIMULATION_SPEED) -> None:
        """Run the simulation loop until completion, a tick limit, or the
        configured max_sim_time safety cutoff is reached.

        Args:
            max_ticks: optional hard cap on the number of ticks (useful for
                tests/headless runs).
            real_time: if True, sleep between ticks so the simulation runs
                at (approximately) wall-clock speed * `speed`.
            speed: simulation speed multiplier used when real_time=True.
        """
        self.running = True
        ticks = 0
        while self.running:
            if self.paused:
                if real_time:
                    time.sleep(self.cfg.dt)
                continue

            self.step()
            ticks += 1

            if self.sim_time >= self.cfg.max_sim_time:
                logger.warning("Reached MAX_SIMULATION_TIME safety cutoff (%.1fs)", self.sim_time)
                break
            if max_ticks is not None and ticks >= max_ticks:
                break
            if self.all_robots_done():
                logger.info("All robots finished/failed at sim_time=%.1fs (%d ticks)",
                            self.sim_time, self.tick_count)
                break

            if real_time:
                time.sleep(self.cfg.dt / max(speed, 1e-6))

        self.running = False

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def reset(self) -> None:
        self.sim_time = 0.0
        self.tick_count = 0
        self.robots.clear()
        self.warehouse.blocked_aisles.clear()

    def all_robots_done(self) -> bool:
        if not self.robots:
            return False
        return all(not r.is_active() for r in self.robots.values())

    # -- introspection -------------------------------------------------------
    def robot_positions(self) -> Dict[str, Coordinate]:
        with self.lock:
            return {rid: r.position for rid, r in self.robots.items()}

    def render_ascii(self) -> str:
        return self.warehouse.render_ascii(self.robot_positions())

    def summary(self) -> Dict:
        with self.lock:
            return {
                "sim_time": round(self.sim_time, 2),
                "tick_count": self.tick_count,
                "running": self.running,
                "paused": self.paused,
                "robots": {rid: r.state.to_public_dict() for rid, r in self.robots.items()},
            }
