"""
simulation/trace_recorder.py
=============================
Records per-tick snapshots of the simulation (warehouse layout once, then
robot states every N ticks) to a JSON file, so a run can be replayed and
inspected visually later - in a browser, without pygame/Flask, and without
needing to watch the simulation live.

This is deliberately a pure *observer*: it hooks into
Simulator.register_tick_callback() and only reads state. It never
influences robot decisions, keeping the decentralization guarantee intact
(the same guarantee that applies to the pygame renderer and the Flask
dashboard added in later phases).

Usage:
    from simulation.trace_recorder import TraceRecorder
    recorder = TraceRecorder(sim, every_n_ticks=2)
    sim.register_tick_callback(recorder.on_tick)
    sim.run(...)
    recorder.save("data/results/trace.json")
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Coordinate = Tuple[int, int]


@dataclass
class TraceRecorder:
    simulator: "Simulator"  # noqa: F821 - avoid circular import at type-check time
    every_n_ticks: int = 2
    max_frames: int = 3000

    frames: List[dict] = field(default_factory=list)
    _warehouse_snapshot: Optional[dict] = None

    def __post_init__(self) -> None:
        self._warehouse_snapshot = self._build_warehouse_snapshot()

    def _build_warehouse_snapshot(self) -> dict:
        return self.simulator.warehouse.to_render_snapshot()

    def on_tick(self, sim: "Simulator") -> None:
        if sim.tick_count % self.every_n_ticks != 0:
            return
        if len(self.frames) >= self.max_frames:
            return

        frame = {
            "tick": sim.tick_count,
            "sim_time": round(sim.sim_time, 2),
            "robots": {rid: r.state.to_public_dict() for rid, r in sim.robots.items()},
            "blocked_aisles": [list(c) for c in sim.warehouse.blocked_aisles],
        }
        self.frames.append(frame)

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        data = {
            "warehouse": self._warehouse_snapshot,
            "frames": self.frames,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f)
