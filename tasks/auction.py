"""
tasks/auction.py
==================
Decentralized auction-based task allocation, per project spec section 21.

    cost = distance_to_pickup * BID_COST_WEIGHT_DISTANCE
         + estimated_travel_time * BID_COST_WEIGHT_TIME
         + congestion_cost * BID_COST_WEIGHT_CONGESTION
         + battery_penalty * BID_COST_WEIGHT_BATTERY
         + current_workload * BID_COST_WEIGHT_WORKLOAD

Lowest valid bid wins. Every robot computes its OWN bid for a task using
only information it already has locally (its own position/battery, plus
whatever it can infer about congestion from its LocalWorldModel) - no
robot computes anyone else's bid. Winner selection (select_winner) is a
pure function of the collected bids, so every robot that has received
the same set of TASK_BID broadcasts computes the identical winner without
any of them acting as an auctioneer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import config
from simulation.physics import manhattan_distance
from robots.robot_state import BatteryState

Coordinate = Tuple[int, int]


@dataclass
class BidInputs:
    robot_position: Coordinate
    pickup_location: Coordinate
    battery_percent: float
    current_workload: int          # e.g. number of tasks already queued (0 in this sim)
    nearby_robot_count: int        # congestion proxy: how many peers are near the pickup
    speed_cells_per_sec: float = config.ROBOT_SPEED_CELLS_PER_SEC


def calculate_bid(inputs: BidInputs) -> float:
    """Lower is better (this robot is a stronger candidate for the task)."""
    distance = manhattan_distance(inputs.robot_position, inputs.pickup_location)
    estimated_travel_time = distance / max(inputs.speed_cells_per_sec, 1e-6)

    congestion_cost = inputs.nearby_robot_count * 2.0

    if inputs.battery_percent <= config.CRITICAL_BATTERY:
        battery_penalty = 1000.0  # effectively disqualifies a critical-battery robot
    elif inputs.battery_percent <= config.LOW_BATTERY:
        battery_penalty = 20.0
    else:
        battery_penalty = 0.0

    workload_cost = inputs.current_workload * 5.0

    return (
        config.BID_COST_WEIGHT_DISTANCE * distance
        + config.BID_COST_WEIGHT_TIME * estimated_travel_time
        + config.BID_COST_WEIGHT_CONGESTION * congestion_cost
        + config.BID_COST_WEIGHT_BATTERY * battery_penalty
        + config.BID_COST_WEIGHT_WORKLOAD * workload_cost
    )


def select_winner(bids: Dict[str, float]) -> Optional[str]:
    """Given {robot_id: bid_cost}, return the winning robot_id (lowest
    cost; ties broken deterministically by robot_id), or None if there
    were no bids at all.
    """
    if not bids:
        return None
    # Sort by (cost, robot_id) so ties are always broken the same way on
    # every robot that runs this same function over the same bid set.
    ranked = sorted(bids.items(), key=lambda pair: (pair[1], pair[0]))
    return ranked[0][0]
