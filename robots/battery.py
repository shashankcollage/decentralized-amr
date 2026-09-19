"""
robots/battery.py
====================
Battery modeling, per project spec sections 23-24.

    battery -= distance * ENERGY_PER_CELL          (while moving)
    battery -= ENERGY_IDLE_PER_SEC * dt             (while idle/waiting)
    battery += CHARGE_RATE_PER_SEC * dt             (while charging)

States: NORMAL / LOW / CRITICAL (thresholds in config.py). Behavioural
rules (also per spec):
    LOW      -> avoid accepting unnecessary new tasks
    CRITICAL -> prioritize charging over everything else
"""

from __future__ import annotations

from dataclasses import dataclass

import config
from robots.robot_state import BatteryState


@dataclass
class BatteryManager:
    level: float = config.BATTERY_INITIAL

    def state(self) -> BatteryState:
        if self.level <= config.CRITICAL_BATTERY:
            return BatteryState.CRITICAL
        if self.level <= config.LOW_BATTERY:
            return BatteryState.LOW
        return BatteryState.NORMAL

    def drain_for_movement(self, distance_cells: float) -> None:
        self.level = max(0.0, self.level - distance_cells * config.ENERGY_PER_CELL)

    def drain_idle(self, dt: float) -> None:
        self.level = max(0.0, self.level - config.ENERGY_IDLE_PER_SEC * dt)

    def charge(self, dt: float) -> None:
        self.level = min(100.0, self.level + config.CHARGE_RATE_PER_SEC * dt)

    def is_fully_charged(self) -> bool:
        return self.level >= 100.0

    def should_seek_charging(self) -> bool:
        """CRITICAL battery always seeks charging. LOW battery seeks
        charging too if it isn't already carrying a task it could still
        safely complete - the caller (robot_controller.py) decides that
        "safely complete" judgment; this method covers the unconditional
        CRITICAL case and the general LOW recommendation."""
        return self.state() in (BatteryState.LOW, BatteryState.CRITICAL)

    def can_accept_new_task(self) -> bool:
        """Per spec: a LOW-battery robot should avoid accepting
        unnecessary new tasks; a CRITICAL robot must not accept any."""
        return self.state() == BatteryState.NORMAL
