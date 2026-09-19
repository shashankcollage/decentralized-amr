"""
experiments/charts.py
========================
Generates the comparison charts required by project spec section 42,
using matplotlib. Saves to data/results/. All values plotted come
directly from collected RunMetrics - nothing here is fabricated.
"""

from __future__ import annotations

import os
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")  # headless-safe backend, no display needed
import matplotlib.pyplot as plt

from metrics.collector import RunMetrics

CHART_FIELDS = [
    ("total_completion_time", "Total Completion Time (s)"),
    ("total_distance", "Distance Travelled (cells)"),
    ("total_waiting_time", "Total Waiting Time (s)"),
    ("num_reroutes", "Number of Re-routes"),
    ("num_deadlocks", "Number of Deadlocks"),
    ("total_battery_consumed", "Battery Consumed (%)"),
    ("total_messages", "Communication Messages"),
    ("num_collisions", "Collisions"),
]


def _mean(runs: List[RunMetrics], field: str) -> float:
    if not runs:
        return 0.0
    return sum(getattr(r, field) for r in runs) / len(runs)


def generate_all_charts(all_runs: Dict[str, Dict[str, List[RunMetrics]]], output_dir: str) -> List[str]:
    os.makedirs(output_dir, exist_ok=True)
    saved_paths = []

    scenarios = list(all_runs.keys())

    for field, label in CHART_FIELDS:
        baseline_values = [_mean(all_runs[s]["baseline"], field) for s in scenarios]
        decentralized_values = [_mean(all_runs[s]["decentralized"], field) for s in scenarios]

        fig, ax = plt.subplots(figsize=(max(6, len(scenarios) * 1.5), 4.5))
        x = range(len(scenarios))
        width = 0.35
        ax.bar([i - width / 2 for i in x], baseline_values, width, label="Stop-and-Wait Baseline")
        ax.bar([i + width / 2 for i in x], decentralized_values, width, label="Decentralized")
        ax.set_xticks(list(x))
        ax.set_xticklabels(scenarios, rotation=20, ha="right")
        ax.set_ylabel(label)
        ax.set_title(f"{label}: Baseline vs Decentralized")
        ax.legend()
        fig.tight_layout()

        path = os.path.join(output_dir, f"chart_{field}.png")
        fig.savefig(path, dpi=120)
        plt.close(fig)
        saved_paths.append(path)

    return saved_paths
