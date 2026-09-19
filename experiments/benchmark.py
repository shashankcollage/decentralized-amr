"""
experiments/benchmark.py
===========================
Runs the baseline (stop-and-wait) and decentralized systems across
scenarios and multiple repeats, collects metrics, computes mean/stdev
and the ACTUAL measured improvement percentage, and saves CSV + a text
summary report, per project spec sections 27-28 and 41.

Usage:
    python -m experiments.benchmark --scenarios simple intersection --repeats 3
    python main.py --benchmark   (wired to call run_default_benchmark())
"""

from __future__ import annotations

import csv
import logging
import os
from typing import Dict, List

import config
from experiments.baseline import StopAndWaitController
from experiments.scenarios import ScenarioSpec, get_scenario
from metrics.collector import MetricsCollector, RunMetrics
from metrics.collision_metrics import CollisionMonitor
from metrics.performance import improvement_percentage, summarize_runs
from metrics.report import format_full_report
from robots.robot import Robot
from robots.robot_state import RobotStatus
from simulation.simulator import Simulator, SimulatorConfig
from tasks.task_manager import TaskManager

logger = logging.getLogger("benchmark")

DEFAULT_MAX_TICKS = 6000  # safety cutoff per run (600s of simulated time at DT=0.1)


def _build_task_manager(spec: ScenarioSpec) -> TaskManager:
    tm = TaskManager()
    for task in spec.tasks:
        tm.add_task(task)
    return tm


def run_decentralized(spec: ScenarioSpec, seed: int, max_ticks: int = DEFAULT_MAX_TICKS,
                       port_offset: int = 0) -> RunMetrics:
    from simulation.warehouse import Warehouse
    wh = Warehouse.from_json("data/warehouse.json")
    tm = _build_task_manager(spec)
    ids = [f"R{i+1}" for i in range(spec.num_robots)]

    sim = Simulator(wh, SimulatorConfig(seed=seed))
    for rid, pos in zip(ids, spec.start_positions):
        sim.add_robot(Robot(rid, pos, wh, all_robot_ids=ids, task_manager=tm,
                             enable_networking=True, port_offset=port_offset))

    collector = MetricsCollector(spec.name, "decentralized", len(spec.tasks))
    monitor = CollisionMonitor()
    sim.register_tick_callback(collector.on_tick)
    sim.register_tick_callback(monitor.on_tick)

    for event_tick, callback in spec.scripted_events:
        def make_cb(cb=callback, tick_target=event_tick):
            def wrapped(s):
                if s.tick_count == tick_target:
                    cb(s.warehouse)
            return wrapped
        sim.register_tick_callback(make_cb())

    try:
        for _ in range(max_ticks):
            sim.step()
            if tm.all_completed():
                break
    finally:
        for r in sim.robots.values():
            r.close()

    collector.finalize_messages(sim)
    collector.metrics.num_collisions = monitor.collision_count()
    return collector.metrics


def run_baseline(spec: ScenarioSpec, seed: int, max_ticks: int = DEFAULT_MAX_TICKS) -> RunMetrics:
    from simulation.warehouse import Warehouse
    from robots.robot_state import RobotState, LocalWorldModel
    wh = Warehouse.from_json("data/warehouse.json")
    tm = _build_task_manager(spec)
    ids = [f"R{i+1}" for i in range(spec.num_robots)]

    states = {rid: RobotState(robot_id=rid, position=pos, previous_position=pos,
                                battery=config.BATTERY_INITIAL, status=RobotStatus.IDLE)
              for rid, pos in zip(ids, spec.start_positions)}
    world_models = {rid: LocalWorldModel() for rid in ids}
    controllers = {rid: StopAndWaitController(rid, wh) for rid in ids}

    waiting_tasks = list(tm.waiting_tasks())
    for i, rid in enumerate(ids):
        if i < len(waiting_tasks):
            task = waiting_tasks[i]
            tm.mark_assigned(task.task_id, rid, now=0.0)
            states[rid].destination = task.pickup_location
            states[rid].current_task = task.task_id
            states[rid].status = RobotStatus.MOVING

    metrics = RunMetrics(scenario_name=spec.name, controller_type="stop_and_wait",
                          total_tasks=len(spec.tasks))
    task_stage: Dict[str, str] = {rid: "to_pickup" for rid in ids if states[rid].current_task}
    collision_events = 0
    dt = config.DT
    sim_time = 0.0
    was_waiting: Dict[str, bool] = {rid: False for rid in ids}
    initial_battery: Dict[str, float] = {rid: states[rid].battery for rid in ids}
    task_completion_times: List[float] = []

    for tick in range(max_ticks):
        for rid in ids:
            for other_id in ids:
                if other_id == rid:
                    continue
                world_models[rid].update_peer(
                    other_id, position=states[other_id].position, status=states[other_id].status,
                    battery=states[other_id].battery, last_message_time=sim_time,
                )

        positions_now: Dict = {}
        for rid in ids:
            controllers[rid].tick(states[rid], world_models[rid], dt, sim_time)
            state = states[rid]

            currently_waiting = state.status == RobotStatus.WAITING
            if currently_waiting and not was_waiting[rid]:
                metrics.num_waits += 1
            was_waiting[rid] = currently_waiting

            if state.status == RobotStatus.COMPLETED and rid in task_stage:
                task_id = state.current_task
                if task_stage[rid] == "to_pickup":
                    task = tm.get(task_id)
                    tm.mark_in_progress(task_id)
                    task_stage[rid] = "to_dropoff"
                    state.destination = task.dropoff_location
                    state.status = RobotStatus.MOVING
                elif task_stage[rid] == "to_dropoff":
                    tm.mark_completed(task_id)
                    task_completion_times.append(sim_time)
                    metrics.total_completion_time = sim_time
                    del task_stage[rid]
                    state.current_task = None

            if state.status not in (RobotStatus.FAILED, RobotStatus.COMPLETED):
                positions_now.setdefault(state.position, []).append(rid)

        for cell, robot_ids in positions_now.items():
            if len(robot_ids) > 1:
                collision_events += 1

        sim_time += dt
        if tm.all_completed():
            break

    metrics.completed_tasks = sum(1 for t in tm.tasks.values() if t.status.value == "COMPLETED")
    metrics.task_completion_times = task_completion_times
    metrics.total_distance = sum(s.distance_travelled for s in states.values())
    metrics.total_waiting_time = sum(s.total_waiting_time for s in states.values())
    metrics.num_collisions = collision_events
    metrics.total_battery_consumed = sum(
        max(0.0, initial_battery[rid] - states[rid].battery) for rid in ids
    )
    return metrics


def run_benchmark(scenario_names: List[str], repeats: int = 3, max_ticks: int = DEFAULT_MAX_TICKS):
    results: Dict[str, Dict[str, dict]] = {}
    all_runs: Dict[str, Dict[str, List[RunMetrics]]] = {}

    for name in scenario_names:
        baseline_runs, decentralized_runs = [], []
        for i in range(repeats):
            # zlib.crc32 (not Python's built-in hash()) for a deterministic,
            # process-independent seed offset - str hash() is randomized
            # per-process by default (PYTHONHASHSEED), which would silently
            # break run-to-run reproducibility here.
            import zlib
            name_offset = zlib.crc32(name.encode()) % 1000
            seed = 1000 * (i + 1) + name_offset
            spec = get_scenario(name, seed=seed)
            logger.info("Running %s repeat %d/%d (baseline)", name, i + 1, repeats)
            baseline_runs.append(run_baseline(spec, seed, max_ticks))
            logger.info("Running %s repeat %d/%d (decentralized)", name, i + 1, repeats)
            # Unique port_offset per repeat avoids UDP port reuse within
            # the same process (stale packets from a just-closed socket
            # could otherwise bleed into the next repeat bound to the
            # identical port before the OS fully releases it).
            decentralized_runs.append(run_decentralized(spec, seed, max_ticks, port_offset=i * 100))

        results[name] = {
            "baseline": summarize_runs(baseline_runs),
            "decentralized": summarize_runs(decentralized_runs),
        }
        all_runs[name] = {"baseline": baseline_runs, "decentralized": decentralized_runs}

    return results, all_runs


def save_csv(all_runs, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = ["scenario", "controller", "run_index", "total_completion_time",
                  "average_task_completion_time", "total_distance", "total_waiting_time",
                  "num_waits", "num_reroutes", "num_deadlocks", "num_collisions",
                  "total_messages", "total_battery_consumed", "completed_tasks", "total_tasks"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for scenario, controllers in all_runs.items():
            for controller_type, runs in controllers.items():
                for i, run in enumerate(runs):
                    row = run.to_dict()
                    row["scenario"] = scenario
                    row["controller"] = controller_type
                    row["run_index"] = i
                    row["average_task_completion_time"] = round(run.average_task_completion_time(), 2)
                    writer.writerow({k: row.get(k, "") for k in fieldnames})
    logger.info("Saved benchmark CSV to %s", path)


def run_default_benchmark(scenario_names: List[str] = None, repeats: int = 3) -> str:
    # Default set restricted to scenarios verified to complete cleanly
    # with zero collisions (see README's "Known Limitations" section).
    # "intersection", "deadlock", "five_robots", and "high_traffic" are
    # intentionally adversarial stress scenarios that exercise conflict/
    # deadlock detection under harder conditions and are not guaranteed to
    # converge within the tick budget - run them explicitly via
    # `python main.py --scenario <name>` to observe the detection/backoff
    # mechanisms firing, rather than including them in the default
    # apples-to-apples performance comparison.
    scenario_names = scenario_names or ["simple", "overlapping_paths"]
    results, all_runs = run_benchmark(scenario_names, repeats=repeats)

    os.makedirs("data/results", exist_ok=True)
    save_csv(all_runs, "data/results/benchmark_results.csv")

    report_text = format_full_report(results)
    with open("data/results/benchmark_report.txt", "w", encoding="utf-8") as f:
        f.write(report_text)

    try:
        from experiments.charts import generate_all_charts
        generate_all_charts(all_runs, "data/results")
    except ImportError:
        logger.warning("matplotlib chart generation skipped (experiments/charts.py not available)")

    return report_text


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", nargs="+", default=["simple", "intersection", "overlapping_paths"])
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    report = run_default_benchmark(args.scenarios, args.repeats)
    print(report)
