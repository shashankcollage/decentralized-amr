"""
robots/localization.py
=========================
Simulated localization, per project spec section 8.

Provides a robot's belief about its own position/velocity. In this
simulation the "true" position is exact grid-cell tracking (see
robots/robot.py), and this module's job is to expose that through the
same interface a real localization stack (e.g. wheel odometry + AMCL on
a real AMR) would use, optionally injecting Gaussian noise so downstream
logic is never written assuming perfect ground truth.

With config.LOCALIZATION_NOISE_STD == 0.0 (the default), get_position()
returns the exact position and the simulation stays perfectly
deterministic given a seed.
"""

from __future__ import annotations

import random
from typing import Optional, Tuple

import config
from simulation.physics import apply_localization_noise, euclidean_distance

Coordinate = Tuple[int, int]


class Localization:
    """Wraps a robot's ground-truth state with a noise model, exposing
    the get_position()/get_velocity()/distance_to() interface required
    by project spec section 8.
    """

    def __init__(self, noise_std: float = config.LOCALIZATION_NOISE_STD,
                 rng: Optional[random.Random] = None) -> None:
        self.noise_std = noise_std
        self.rng = rng or random.Random()

    def get_position(self, true_position: Coordinate) -> Tuple[float, float]:
        """Return a (possibly noisy) estimate of the robot's own position."""
        return apply_localization_noise(
            (float(true_position[0]), float(true_position[1])), self.noise_std, self.rng
        )

    def get_velocity(self, true_velocity: Tuple[float, float]) -> Tuple[float, float]:
        """Localization noise is not applied to velocity in this model
        (velocity is derived from planned motion, not sensed) - this
        method exists purely to satisfy the required interface shape and
        to give a single place to add velocity noise later if needed."""
        return true_velocity

    def distance_to(self, self_position: Coordinate, other_position: Coordinate) -> float:
        return euclidean_distance(self_position, other_position)
