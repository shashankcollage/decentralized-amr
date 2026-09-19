"""
metrics/report.py
====================
Generates the human-readable benchmark summary report, per project spec
section 41's example output format. All numbers come from actual
collected RunMetrics/summarize_runs() output - nothing here is
hard-coded.
"""

from __future__ import annotations

from typing import Dict

from metrics.performance import improvement_percentage


def format_scenario_report(scenario_name: str, baseline_summary: dict, decentralized_summary: dict) -> str:
    lines = [f"SCENARIO: {scenario_name}", ""]

    lines.append("Baseline (stop-and-wait):")
    lines.append(f"  Completion Time: {baseline_summary['total_completion_time']['mean']:.1f}s "
                 f"(stdev {baseline_summary['total_completion_time']['stdev']:.1f})")
    lines.append(f"  Collisions: {baseline_summary['num_collisions']['mean']:.1f}")
    lines.append(f"  Average Waiting: {baseline_summary['total_waiting_time']['mean']:.1f}s")
    lines.append("")

    lines.append("Decentralized:")
    lines.append(f"  Completion Time: {decentralized_summary['total_completion_time']['mean']:.1f}s "
                 f"(stdev {decentralized_summary['total_completion_time']['stdev']:.1f})")
    lines.append(f"  Collisions: {decentralized_summary['num_collisions']['mean']:.1f}")
    lines.append(f"  Average Waiting: {decentralized_summary['total_waiting_time']['mean']:.1f}s")
    lines.append("")

    time_improvement = improvement_percentage(
        baseline_summary['total_completion_time']['mean'],
        decentralized_summary['total_completion_time']['mean'],
    )
    waiting_improvement = improvement_percentage(
        baseline_summary['total_waiting_time']['mean'],
        decentralized_summary['total_waiting_time']['mean'],
    )
    lines.append("Improvement:")
    lines.append(f"  Completion time: {time_improvement:.1f}%")
    lines.append(f"  Total waiting time: {waiting_improvement:.1f}%")
    lines.append("")

    return "\n".join(lines)


def format_full_report(results: Dict[str, Dict[str, dict]]) -> str:
    """results: {scenario_name: {"baseline": summary, "decentralized": summary}}"""
    sections = ["=" * 60, "DECENTRALIZED AMR BENCHMARK REPORT", "=" * 60, ""]
    for scenario_name, pair in results.items():
        sections.append(format_scenario_report(scenario_name, pair["baseline"], pair["decentralized"]))
        sections.append("-" * 60)
        sections.append("")
    return "\n".join(sections)
