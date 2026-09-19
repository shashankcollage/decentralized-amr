"""
coordination/priority.py
==========================
Deterministic priority scoring, per project spec section 16.

    priority = urgency_weight   * urgency
             + waiting_weight   * waiting_time_factor
             + battery_weight   * battery_factor
             + proximity_weight * proximity_factor

All inputs are normalized to [0, 1] so the weights (config.py) are
directly comparable. The final tie-breaker is always the robot ID
(lexicographic), so priority(A, B) is a total order with no randomness
anywhere - required for reproducible, arguable-about experiments.
"""

from __future__ import annotations

from dataclasses import dataclass

import config


@dataclass
class PriorityInputs:
    """Raw, un-normalized inputs to the priority score."""
    task_urgency: float          # 0 (no task / low priority task) .. 1 (urgent deadline)
    waiting_time_seconds: float  # continuous time this robot has been waiting
    battery_percent: float       # 0..100
    distance_to_conflict: float  # cells; smaller = closer = more urgent to resolve


def _normalize_waiting(waiting_time_seconds: float) -> float:
    """Longer waits push priority towards 1.0, saturating at WAIT_TIMEOUT
    so a robot that has waited the full timeout is treated as maximally
    urgent (about to be flagged for deadlock otherwise)."""
    return min(1.0, waiting_time_seconds / max(config.WAIT_TIMEOUT, 1e-6))


def _normalize_battery(battery_percent: float) -> float:
    """Lower battery -> higher priority (get out of the way faster / let
    a low-battery robot go first so it can reach a charger sooner)."""
    return 1.0 - min(1.0, max(0.0, battery_percent) / 100.0)


def _normalize_proximity(distance_to_conflict: float, max_relevant_distance: float = 10.0) -> float:
    """Closer to the conflict point -> higher priority (it has more to
    lose / less room to maneuver an alternative)."""
    if distance_to_conflict <= 0:
        return 1.0
    return max(0.0, 1.0 - min(distance_to_conflict, max_relevant_distance) / max_relevant_distance)


def calculate_priority(inputs: PriorityInputs) -> float:
    """Compute the composite, normalized priority score (roughly 0..1,
    can slightly exceed 1 with saturated inputs across all terms)."""
    urgency_term = config.PRIORITY_WEIGHT_URGENCY * max(0.0, min(1.0, inputs.task_urgency))
    waiting_term = config.PRIORITY_WEIGHT_WAITING * _normalize_waiting(inputs.waiting_time_seconds)
    battery_term = config.PRIORITY_WEIGHT_BATTERY * _normalize_battery(inputs.battery_percent)
    proximity_term = config.PRIORITY_WEIGHT_PROXIMITY * _normalize_proximity(inputs.distance_to_conflict)
    return urgency_term + waiting_term + battery_term + proximity_term


def higher_priority_robot(robot_id_a: str, priority_a: float,
                          robot_id_b: str, priority_b: float) -> str:
    """Return whichever robot_id should win a conflict. Ties are broken
    deterministically by robot ID (lexicographically smaller wins), never
    randomly - required by project spec section 16.
    """
    if priority_a > priority_b:
        return robot_id_a
    if priority_b > priority_a:
        return robot_id_b
    # Exact tie: deterministic, reproducible tie-break.
    return robot_id_a if robot_id_a < robot_id_b else robot_id_b
