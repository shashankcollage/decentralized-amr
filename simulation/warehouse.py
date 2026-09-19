"""
simulation/warehouse.py
=======================
Grid-based warehouse world model.

This module owns the *physical* layout of the warehouse: which cells are
walls, shelves, pickup/dropoff points, charging stations, and which cells
are temporarily blocked (e.g. a simulated spill or maintenance closure).

It intentionally contains NO robot decision logic. Robots query this
object (or a copy of the static layout) to plan paths, but the warehouse
itself never decides where a robot should go.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

logger = logging.getLogger("warehouse")

Coordinate = Tuple[int, int]


class CellType(str, Enum):
    """All cell types supported by the grid."""
    EMPTY = "EMPTY"
    SHELF = "SHELF"
    WALL = "WALL"
    PICKUP = "PICKUP"
    DROPOFF = "DROPOFF"
    CHARGING_STATION = "CHARGING_STATION"
    INTERSECTION = "INTERSECTION"
    ROBOT = "ROBOT"  # transient marker used only for text rendering


# Cell types a robot may never physically enter.
STATIC_BLOCKING_TYPES: Set[CellType] = {CellType.SHELF, CellType.WALL}


@dataclass
class Warehouse:
    """A 2D grid-based warehouse.

    Attributes:
        width: number of columns.
        height: number of rows.
        grid: dict mapping (x, y) -> CellType for every cell in the grid.
        pickup_points: list of pickup coordinates.
        dropoff_points: list of dropoff coordinates.
        charging_stations: list of charging-station coordinates.
        intersections: list of coordinates flagged as choke points that
            require negotiation before entry.
        blocked_aisles: set of coordinates that are *dynamically* blocked
            (e.g. spill, maintenance). These are layered on top of the
            static grid and can change at runtime.
    """

    width: int
    height: int
    grid: Dict[Coordinate, CellType] = field(default_factory=dict)
    pickup_points: List[Coordinate] = field(default_factory=list)
    dropoff_points: List[Coordinate] = field(default_factory=list)
    charging_stations: List[Coordinate] = field(default_factory=list)
    intersections: List[Coordinate] = field(default_factory=list)
    blocked_aisles: Set[Coordinate] = field(default_factory=set)

    # -- construction ---------------------------------------------------
    @classmethod
    def empty(cls, width: int, height: int) -> "Warehouse":
        """Create an empty warehouse (all cells EMPTY) with a perimeter wall."""
        grid: Dict[Coordinate, CellType] = {}
        for x in range(width):
            for y in range(height):
                if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                    grid[(x, y)] = CellType.WALL
                else:
                    grid[(x, y)] = CellType.EMPTY
        return cls(width=width, height=height, grid=grid)

    @classmethod
    def from_json(cls, path: str) -> "Warehouse":
        """Load a warehouse layout from a JSON configuration file.

        Expected JSON schema (see data/warehouse.json for a full example)::

            {
              "width": 20,
              "height": 20,
              "cells": [{"x": 3, "y": 3, "type": "SHELF"}, ...],
              "pickup_points": [[1, 1], [1, 2]],
              "dropoff_points": [[18, 18]],
              "charging_stations": [[10, 10]],
              "intersections": [[9, 9], [10, 9]]
            }

        Any cell not explicitly listed defaults to EMPTY (perimeter cells
        default to WALL, matching :meth:`empty`).
        """
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        wh = cls.empty(data["width"], data["height"])

        for cell in data.get("cells", []):
            x, y = cell["x"], cell["y"]
            wh.grid[(x, y)] = CellType(cell["type"])

        wh.pickup_points = [tuple(p) for p in data.get("pickup_points", [])]
        wh.dropoff_points = [tuple(p) for p in data.get("dropoff_points", [])]
        wh.charging_stations = [tuple(p) for p in data.get("charging_stations", [])]
        wh.intersections = [tuple(p) for p in data.get("intersections", [])]

        for p in wh.pickup_points:
            wh.grid[p] = CellType.PICKUP
        for p in wh.dropoff_points:
            wh.grid[p] = CellType.DROPOFF
        for p in wh.charging_stations:
            wh.grid[p] = CellType.CHARGING_STATION
        for p in wh.intersections:
            # Intersections keep their underlying passable type but are
            # also tracked separately for negotiation logic.
            if wh.grid.get(p, CellType.EMPTY) not in STATIC_BLOCKING_TYPES:
                wh.grid.setdefault(p, CellType.INTERSECTION)

        logger.info("Loaded warehouse '%s': %dx%d, %d pickups, %d dropoffs, %d chargers",
                     path, wh.width, wh.height, len(wh.pickup_points),
                     len(wh.dropoff_points), len(wh.charging_stations))
        return wh

    def to_json(self, path: str) -> None:
        """Serialize the current layout (including dynamic blocks) to JSON."""
        cells = [{"x": x, "y": y, "type": t.value} for (x, y), t in self.grid.items()
                 if t not in (CellType.EMPTY,)]
        data = {
            "width": self.width,
            "height": self.height,
            "cells": cells,
            "pickup_points": [list(p) for p in self.pickup_points],
            "dropoff_points": [list(p) for p in self.dropoff_points],
            "charging_stations": [list(p) for p in self.charging_stations],
            "intersections": [list(p) for p in self.intersections],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    # -- queries ----------------------------------------------------------
    def in_bounds(self, pos: Coordinate) -> bool:
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def cell_type(self, pos: Coordinate) -> CellType:
        return self.grid.get(pos, CellType.EMPTY)

    def is_statically_blocked(self, pos: Coordinate) -> bool:
        """True if the cell is permanently impassable (wall/shelf) or out of bounds."""
        if not self.in_bounds(pos):
            return True
        return self.cell_type(pos) in STATIC_BLOCKING_TYPES

    def is_dynamically_blocked(self, pos: Coordinate) -> bool:
        """True if the cell is temporarily blocked (e.g. spill, maintenance)."""
        return pos in self.blocked_aisles

    def is_passable(self, pos: Coordinate) -> bool:
        """True if a robot may currently occupy this cell."""
        return not self.is_statically_blocked(pos) and not self.is_dynamically_blocked(pos)

    def is_intersection(self, pos: Coordinate) -> bool:
        return pos in self.intersections

    def neighbors(self, pos: Coordinate, allow_diagonal: bool = False) -> List[Coordinate]:
        """4-directional (optionally 8-directional) neighbor cells that exist on the grid.

        This does NOT filter by passability - callers (e.g. the planner)
        decide what "blocked" means for their purposes (static vs. also
        reserved cells, etc.).
        """
        x, y = pos
        deltas = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if allow_diagonal:
            deltas += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        result = []
        for dx, dy in deltas:
            np_ = (x + dx, y + dy)
            if self.in_bounds(np_):
                result.append(np_)
        return result

    # -- dynamic world mutation --------------------------------------------
    def block_aisle(self, cells: List[Coordinate]) -> None:
        """Mark a set of cells as temporarily blocked (dynamic obstacle)."""
        for c in cells:
            self.blocked_aisles.add(c)
        logger.warning("Aisle blocked: %s", cells)

    def unblock_aisle(self, cells: List[Coordinate]) -> None:
        for c in cells:
            self.blocked_aisles.discard(c)
        logger.info("Aisle unblocked: %s", cells)

    def nearest_of_type(self, from_pos: Coordinate, points: List[Coordinate]) -> Optional[Coordinate]:
        """Return the Manhattan-nearest point from a list (e.g. nearest charger)."""
        if not points:
            return None
        return min(points, key=lambda p: abs(p[0] - from_pos[0]) + abs(p[1] - from_pos[1]))

    # -- text rendering (debugging aid; pygame renderer is separate) -------
    def render_ascii(self, robot_positions: Optional[Dict[str, Coordinate]] = None) -> str:
        """Render the grid as ASCII text, useful for logs/tests/headless debugging."""
        robot_positions = robot_positions or {}
        pos_to_label: Dict[Coordinate, str] = {}
        for robot_id, pos in robot_positions.items():
            pos_to_label[pos] = robot_id[:2]

        symbol_map = {
            CellType.EMPTY: ".",
            CellType.SHELF: "#",
            CellType.WALL: "%",
            CellType.PICKUP: "P",
            CellType.DROPOFF: "D",
            CellType.CHARGING_STATION: "C",
            CellType.INTERSECTION: "+",
            CellType.ROBOT: "R",
        }
        lines = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                pos = (x, y)
                if pos in pos_to_label:
                    row.append(pos_to_label[pos].ljust(2))
                elif pos in self.blocked_aisles:
                    row.append("X ")
                else:
                    row.append(symbol_map[self.cell_type(pos)].ljust(2))
            lines.append("".join(row))
        return "\n".join(lines)

    def to_render_snapshot(self) -> dict:
        """A JSON-serializable static-layout snapshot shared by every visual
        consumer of the warehouse (offline trace viewer, live dashboard,
        and later the pygame renderer), so the rendering data model only
        has one implementation to keep correct.
        """
        cells = {f"{x},{y}": t.value for (x, y), t in self.grid.items()}
        return {
            "width": self.width,
            "height": self.height,
            "cells": cells,
            "pickup_points": [list(p) for p in self.pickup_points],
            "dropoff_points": [list(p) for p in self.dropoff_points],
            "charging_stations": [list(p) for p in self.charging_stations],
            "intersections": [list(p) for p in self.intersections],
        }
