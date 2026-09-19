"""
metrics/collector.py
=======================
Aggregates the per-run statistics required by project spec section 27:
total completion time, average task completion time, distance travelled,
number of waits, total waiting time, number of re-routes, number of
deadlocks, number of collisions, communication messages, battery
consumption.

Attaches as a Simulator tick callback (see
simulation/simulator.py's register_tick_callback) so it observes every
tick without the simulator itself needing to know metrics exist - same
"pure observer" pattern as TraceRecorder.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class RunMetrics:
    scenario_name: str = ""
    controller_type: str = ""  # "decentralized" | "stop_and_wait"

    total_completion_time: float = 0.0
    task_completion_times: List[float] = field(default_factory=list)
    total_distance: float = 0.0
    total_waiting_time: float = 0.0
    num_waits: int = 0
    num_reroutes: int = 0
    num_deadlocks: int = 0
    num_collisions: int = 0
    total_messages: int = 0
    total_battery_consumed: float = 0.0
    completed_tasks: int = 0
    total_tasks: int = 0

    def average_task_completion_time(self) -> float:
        if not self.task_completion_times:
            return 0.0
        return sum(self.task_completion_times) / len(self.task_completion_times)

    def to_dict(self) -> dict:
        return {
            "scenario_name": self.scenario_name,
            "controller_type": self.controller_type,
            "total_completion_time": round(self.total_completion_time, 2),
            "average_task_completion_time": round(self.average_task_completion_time(), 2),
            "total_distance": round(self.total_distance, 2),
            "total_waiting_time": round(self.total_waiting_time, 2),
            "num_waits": self.num_waits,
            "num_reroutes": self.num_reroutes,
            "num_deadlocks": self.num_deadlocks,
            "num_collisions": self.num_collisions,
            "total_messages": self.total_messages,
            "total_battery_consumed": round(self.total_battery_consumed, 2),
            "completed_tasks": self.completed_tasks,
            "total_tasks": self.total_tasks,
        }


class MetricsCollector:
    def __init__(self, scenario_name: str, controller_type: str, total_tasks: int) -> None:
        self.metrics = RunMetrics(scenario_name=scenario_name, controller_type=controller_type,
                                    total_tasks=total_tasks)
        self._was_waiting: Dict[str, bool] = {}
        self._last_completed_tasks: Dict[str, int] = {}
        self._last_reroute_count: Dict[str, int] = {}
        self._initial_battery: Dict[str, float] = {}
        self._deadlock_robots_seen: set = set()

    def on_tick(self, sim) -> None:
        m = self.metrics
        for rid, robot in sim.robots.items():
            state = robot.state
            if rid not in self._initial_battery:
                self._initial_battery[rid] = state.battery

            currently_waiting = state.status.value == "WAITING"
            was_waiting = self._was_waiting.get(rid, False)
            if currently_waiting and not was_waiting:
                m.num_waits += 1
            self._was_waiting[rid] = currently_waiting

            last_reroutes = self._last_reroute_count.get(rid, 0)
            if state.reroute_count > last_reroutes:
                m.num_reroutes += (state.reroute_count - last_reroutes)
                self._last_reroute_count[rid] = state.reroute_count

            last_completed = self._last_completed_tasks.get(rid, 0)
            if state.completed_tasks > last_completed:
                m.completed_tasks += (state.completed_tasks - last_completed)
                m.task_completion_times.append(sim.sim_time)
                self._last_completed_tasks[rid] = state.completed_tasks
                m.total_completion_time = sim.sim_time

        m.total_waiting_time = sum(r.state.total_waiting_time for r in sim.robots.values())
        m.total_distance = sum(r.state.distance_travelled for r in sim.robots.values())
        m.total_battery_consumed = sum(
            max(0.0, self._initial_battery.get(rid, 100.0) - r.state.battery)
            for rid, r in sim.robots.items()
        )

    def record_deadlock(self, cycle: List[str]) -> None:
        key = tuple(sorted(cycle))
        if key not in self._deadlock_robots_seen:
            self._deadlock_robots_seen.add(key)
            self.metrics.num_deadlocks += 1

    def record_collision(self) -> None:
        self.metrics.num_collisions += 1

    def finalize_messages(self, sim) -> None:
        total = 0
        for robot in sim.robots.values():
            controller = getattr(robot, "controller", None)
            network = getattr(controller, "network", None) if controller else None
            if network is not None:
                total += network.messages_sent
        self.metrics.total_messages = total
