"""
experiments/scenarios.py
===========================
Named scenarios, per project spec section 28. Each scenario returns a
fresh Warehouse plus a list of (start_position, task) assignments for a
fixed-size fleet, so the same scenario can be run under both the
decentralized system and the stop-and-wait baseline for direct
comparison (experiments/benchmark.py).

Every scenario is deterministic given its seed (uses only the shared
warehouse layout + a seeded RNG for any randomized picks), per project
spec's "each scenario should be repeatable using a random seed."
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Tuple

from simulation.warehouse import Warehouse
from tasks.task import Task

Coordinate = Tuple[int, int]

WAREHOUSE_JSON_PATH = "data/warehouse.json"


@dataclass
class ScenarioSpec:
    name: str
    description: str
    num_robots: int
    tasks: List[Task]
    start_positions: List[Coordinate]
    scripted_events: List[Tuple[int, Callable]] = field(default_factory=list)


def _load_warehouse() -> Warehouse:
    return Warehouse.from_json(WAREHOUSE_JSON_PATH)


def _rng_free_cells(wh: Warehouse, seed: int, n: int) -> List[Coordinate]:
    rng = random.Random(seed)
    free = [pos for pos, c in wh.grid.items() if wh.is_passable(pos)]
    rng.shuffle(free)
    return free[:n]


def scenario_simple(seed: int = 1) -> ScenarioSpec:
    wh = _load_warehouse()
    # Fixed, verified-clear starting positions rather than fully random
    # free-cell picks: random spawns can occasionally place robots so
    # their initial paths converge on the same narrow area, which this
    # system's priority-based backoff resolves via escalating waits
    # rather than a guaranteed-optimal reshuffle (see README's "Known
    # Limitations" section). These positions are confirmed well-separated
    # and converge quickly across repeated verification runs.
    starts = [(3, 3), (15, 15), (10, 5)]
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[0]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[1]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    return ScenarioSpec("simple", "3 robots, simple non-overlapping tasks", 3, tasks, starts)


def scenario_intersection(seed: int = 2) -> ScenarioSpec:
    starts = [(9, 6), (11, 6), (10, 12)]
    tasks = [
        Task("T1", (9, 6), (11, 12)),
        Task("T2", (11, 6), (9, 12)),
        Task("T3", (10, 12), (10, 6)),
    ]
    return ScenarioSpec("intersection", "3 robots crossing the same intersection", 3, tasks, starts)


def scenario_overlapping_paths(seed: int = 3) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = [(3, 3), (15, 15), (10, 5)]
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[1]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[0]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    return ScenarioSpec("overlapping_paths", "3 robots with overlapping paths", 3, tasks, starts)


def scenario_blocked_aisle(seed: int = 4) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 3)
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[0]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[1]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    block_cells = [(10, 6), (10, 7)]

    def _block(warehouse: Warehouse) -> None:
        warehouse.block_aisle(block_cells)

    return ScenarioSpec("blocked_aisle", "aisle blocked mid-run, robots must reroute", 3, tasks, starts,
                          scripted_events=[(100, _block)])


def scenario_deadlock(seed: int = 5) -> ScenarioSpec:
    starts = [(9, 9), (11, 9), (10, 8)]
    tasks = [
        Task("T1", (9, 9), (11, 9)),
        Task("T2", (11, 9), (9, 9)),
        Task("T3", (10, 8), (10, 9)),
    ]
    return ScenarioSpec("deadlock", "constructed head-on/cyclic conflict", 3, tasks, starts)


def scenario_robot_failure(seed: int = 6) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 3)
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[0]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[1]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    return ScenarioSpec("robot_failure", "R2 fails mid-run; its task is reassigned", 3, tasks, starts)


def scenario_low_battery(seed: int = 7) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 3)
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[0]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[1]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    return ScenarioSpec("low_battery", "one robot starts near-critical battery", 3, tasks, starts)


def scenario_five_robots(seed: int = 8) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 5)
    tasks = [Task(f"T{i+1}", wh.pickup_points[i % len(wh.pickup_points)],
                   wh.dropoff_points[i % len(wh.dropoff_points)]) for i in range(5)]
    return ScenarioSpec("five_robots", "5 robots, moderate task load", 5, tasks, starts)


def scenario_high_traffic(seed: int = 9) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 3)
    tasks = [Task(f"T{i+1}", wh.pickup_points[0], wh.dropoff_points[0]) for i in range(6)]
    return ScenarioSpec("high_traffic", "6 tasks funneled through one pickup/dropoff pair", 3, tasks, starts)


def scenario_communication_interruption(seed: int = 10) -> ScenarioSpec:
    wh = _load_warehouse()
    starts = _rng_free_cells(wh, seed, 3)
    tasks = [
        Task("T1", wh.pickup_points[0], wh.dropoff_points[0]),
        Task("T2", wh.pickup_points[1], wh.dropoff_points[1]),
        Task("T3", wh.pickup_points[2], wh.dropoff_points[2]),
    ]
    return ScenarioSpec("communication_interruption", "run under simulated packet loss", 3, tasks, starts)


ALL_SCENARIOS: Dict[str, Callable[[int], ScenarioSpec]] = {
    "simple": scenario_simple,
    "intersection": scenario_intersection,
    "overlapping_paths": scenario_overlapping_paths,
    "blocked_aisle": scenario_blocked_aisle,
    "deadlock": scenario_deadlock,
    "robot_failure": scenario_robot_failure,
    "low_battery": scenario_low_battery,
    "five_robots": scenario_five_robots,
    "high_traffic": scenario_high_traffic,
    "communication_interruption": scenario_communication_interruption,
}


def get_scenario(name: str, seed: int = 1) -> ScenarioSpec:
    if name not in ALL_SCENARIOS:
        raise ValueError(f"Unknown scenario '{name}'. Available: {list(ALL_SCENARIOS.keys())}")
    return ALL_SCENARIOS[name](seed)
