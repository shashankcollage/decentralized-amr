"""
simulation/physics.py
======================
Small, stateless physics helpers shared by the simulator and by robots'
sensor/localization modules. Kept separate from Warehouse (topology) and
from Robot (decision logic) so it can be unit-tested in isolation.
"""

from __future__ import annotations

import math
import random
from typing import Optional, Tuple

Coordinate = Tuple[int, int]


def euclidean_distance(a: Coordinate, b: Coordinate) -> float:
    """Straight-line distance between two grid cells."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def manhattan_distance(a: Coordinate, b: Coordinate) -> int:
    """4-directional grid distance between two cells."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def velocity_towards(current: Coordinate, target: Coordinate, speed: float) -> Tuple[float, float]:
    """Unit-direction velocity vector (scaled by speed) from current to target cell.

    Since movement in this simulation is grid-based (one cell per step),
    the vector is axis-aligned: robots move in an L-shaped, 4-directional
    manner rather than diagonally, matching the A* planner's action space.
    """
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    if dx == 0 and dy == 0:
        return (0.0, 0.0)
    # Move along whichever axis has the larger remaining distance first;
    # ties broken by x. This only affects the *reported* velocity vector for
    # display/telemetry purposes - actual stepping follows the planned path.
    if abs(dx) >= abs(dy):
        return (speed * (1 if dx > 0 else -1), 0.0)
    return (0.0, speed * (1 if dy > 0 else -1))


def apply_localization_noise(position: Tuple[float, float], std_dev: float,
                              rng: Optional[random.Random] = None) -> Tuple[float, float]:
    """Return a noisy copy of a position, simulating imperfect localization.

    With std_dev == 0.0 (the default in config.py) this is a no-op, which
    keeps the simulation perfectly deterministic for a given random seed.
    """
    if std_dev <= 0.0:
        return position
    r = rng or random
    return (
        position[0] + r.gauss(0.0, std_dev),
        position[1] + r.gauss(0.0, std_dev),
    )


def is_head_on(pos_a: Coordinate, next_a: Coordinate, pos_b: Coordinate, next_b: Coordinate) -> bool:
    """Two robots are swapping cells with each other (classic head-on conflict)."""
    return next_a == pos_b and next_b == pos_a and pos_a != pos_b


def is_same_cell_conflict(next_a: Coordinate, next_b: Coordinate) -> bool:
    """Two robots planning to occupy the identical cell at the same timestep."""
    return next_a == next_b


def is_following_conflict(pos_a: Coordinate, next_a: Coordinate, pos_b: Coordinate) -> bool:
    """Robot A is moving into the cell robot B currently occupies (rear-end risk)."""
    return next_a == pos_b and pos_a != pos_b
