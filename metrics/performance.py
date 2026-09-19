"""
metrics/performance.py
=========================
Statistical helpers for benchmark analysis, per project spec section 27.

improvement_percentage = (baseline_time - decentralized_time) / baseline_time * 100

The project spec is explicit that this must be an ACTUAL measured value,
never hard-coded - every function here operates on real collected
RunMetrics, computed by experiments/benchmark.py from actual simulation
runs.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass
from typing import List

from metrics.collector import RunMetrics


@dataclass
class AggregateStats:
    mean: float
    stdev: float
    minimum: float
    maximum: float
    n: int


def aggregate(values: List[float]) -> AggregateStats:
    if not values:
        return AggregateStats(0.0, 0.0, 0.0, 0.0, 0)
    mean = statistics.mean(values)
    stdev = statistics.stdev(values) if len(values) > 1 else 0.0
    return AggregateStats(mean=mean, stdev=stdev, minimum=min(values), maximum=max(values), n=len(values))


def improvement_percentage(baseline_value: float, decentralized_value: float) -> float:
    """Compute the ACTUAL measured improvement of the decentralized system
    over the baseline for a given metric (lower-is-better, e.g. completion
    time or total waiting time). Positive = decentralized is better.
    Never hard-coded - always computed from the two real values passed in.
    """
    if baseline_value == 0:
        return 0.0
    return ((baseline_value - decentralized_value) / baseline_value) * 100.0


def summarize_runs(runs: List[RunMetrics]) -> dict:
    if not runs:
        return {}
    fields = [
        "total_completion_time", "total_distance", "total_waiting_time",
        "num_waits", "num_reroutes", "num_deadlocks", "num_collisions",
        "total_messages", "total_battery_consumed",
    ]
    result = {}
    for f in fields:
        values = [float(getattr(r, f)) for r in runs]
        stats = aggregate(values)
        result[f] = {
            "mean": round(stats.mean, 2), "stdev": round(stats.stdev, 2),
            "min": round(stats.minimum, 2), "max": round(stats.maximum, 2), "n": stats.n,
        }
    avg_task_times = [r.average_task_completion_time() for r in runs]
    task_stats = aggregate(avg_task_times)
    result["average_task_completion_time"] = {
        "mean": round(task_stats.mean, 2), "stdev": round(task_stats.stdev, 2),
        "min": round(task_stats.minimum, 2), "max": round(task_stats.maximum, 2), "n": task_stats.n,
    }
    return result
