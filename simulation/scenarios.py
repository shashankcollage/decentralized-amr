"""
simulation/scenarios.py
==========================
Per the project's required file structure, scenario definitions are
expected at both simulation/scenarios.py and experiments/scenarios.py.
Rather than duplicate the logic in two places (a maintenance hazard),
this module re-exports everything from experiments/scenarios.py, which
is where the actual scenario definitions live alongside the benchmark
code that consumes them.
"""

from experiments.scenarios import (  # noqa: F401
    ScenarioSpec,
    get_scenario,
    ALL_SCENARIOS,
    scenario_simple,
    scenario_intersection,
    scenario_overlapping_paths,
    scenario_blocked_aisle,
    scenario_deadlock,
    scenario_robot_failure,
    scenario_low_battery,
    scenario_five_robots,
    scenario_high_traffic,
    scenario_communication_interruption,
)
