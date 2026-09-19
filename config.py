"""
config.py
=========
Single source of truth for every tunable parameter in the simulation.

Design note: this file holds *parameters only*. It contains no decision
logic. Robot decision-making modules (planning/, coordination/, tasks/)
import values from here but never write back to it, so the same config
can later be flashed onto real edge devices (Raspberry Pi / Jetson Nano)
without change.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Tuple


# ---------------------------------------------------------------------------
# Warehouse / grid
# ---------------------------------------------------------------------------
GRID_WIDTH: int = 20
GRID_HEIGHT: int = 20
CELL_SIZE_PX: int = 32          # pixel size of a grid cell for rendering

# ---------------------------------------------------------------------------
# Fleet
# ---------------------------------------------------------------------------
NUM_ROBOTS: int = 3
ROBOT_START_PORT: int = 5001    # R1 = 127.0.0.1:5001, R2 = 5002, ...

# ---------------------------------------------------------------------------
# Motion / physics
# ---------------------------------------------------------------------------
DT: float = 0.1                 # seconds per simulation tick
ROBOT_SPEED_CELLS_PER_SEC: float = 2.0   # nominal speed
LOCALIZATION_NOISE_STD: float = 0.0      # std-dev of simulated GPS-like noise (cells)

# ---------------------------------------------------------------------------
# Safety / collision
# ---------------------------------------------------------------------------
SAFE_DISTANCE: float = 1.0       # minimum allowed euclidean distance (cells)
SENSOR_RANGE: int = 4            # cells

# ---------------------------------------------------------------------------
# Communication (decentralized P2P, UDP)
# ---------------------------------------------------------------------------
HEARTBEAT_INTERVAL: float = 0.2  # seconds between ROBOT_STATE broadcasts
ROBOT_TIMEOUT: float = 2.0       # seconds of silence before peer is considered lost
PACKET_LOSS_RATE: float = 0.0    # 0.0 - 1.0 probability a message is dropped
NETWORK_DELAY: float = 0.0       # seconds of artificial delay applied to messages
UDP_BUFFER_SIZE: int = 65536
HMAC_SHARED_SECRET: bytes = b"decentralized-amr-shared-secret-CHANGE-ME"
MESSAGE_MAX_AGE: float = 5.0     # seconds; older messages are rejected as stale

# ---------------------------------------------------------------------------
# Battery
# ---------------------------------------------------------------------------
BATTERY_INITIAL: float = 100.0
LOW_BATTERY: float = 25.0
CRITICAL_BATTERY: float = 10.0
ENERGY_PER_CELL: float = 0.5     # % battery consumed per cell travelled
ENERGY_IDLE_PER_SEC: float = 0.01
CHARGE_RATE_PER_SEC: float = 5.0  # % battery gained per second while charging

# ---------------------------------------------------------------------------
# Coordination / negotiation
# ---------------------------------------------------------------------------
WAIT_TIMEOUT: float = 5.0        # seconds a robot may wait before deadlock suspicion
CONFLICT_RESOLUTION_TIMEOUT: float = 1.0
DEADLOCK_CHECK_INTERVAL: float = 1.0
MAX_REPLAN_ATTEMPTS: int = 5

# Priority score weights (must sum to something sensible; not required = 1.0)
PRIORITY_WEIGHT_URGENCY: float = 0.35
PRIORITY_WEIGHT_WAITING: float = 0.30
PRIORITY_WEIGHT_BATTERY: float = 0.15
PRIORITY_WEIGHT_PROXIMITY: float = 0.20

# ---------------------------------------------------------------------------
# Task allocation (auction)
# ---------------------------------------------------------------------------
BID_COST_WEIGHT_DISTANCE: float = 1.0
BID_COST_WEIGHT_TIME: float = 0.5
BID_COST_WEIGHT_CONGESTION: float = 0.3
BID_COST_WEIGHT_BATTERY: float = 0.4
BID_COST_WEIGHT_WORKLOAD: float = 0.2
TASK_BID_WINDOW: float = 0.5     # seconds robots have to submit bids
TASK_REASSIGN_TIMEOUT: float = 90.0  # seconds with no progress before reassignment

# ---------------------------------------------------------------------------
# Simulation control
# ---------------------------------------------------------------------------
SIMULATION_SPEED: float = 1.0    # multiplier; > 1 = faster than real time
RANDOM_SEED: int = 42
MAX_SIMULATION_TIME: float = 600.0  # safety cutoff (seconds of sim time)

# ---------------------------------------------------------------------------
# Dashboard
# ---------------------------------------------------------------------------
DASHBOARD_HOST: str = "127.0.0.1"
DASHBOARD_PORT: int = 8080
DASHBOARD_POLL_INTERVAL_MS: int = 500

# ---------------------------------------------------------------------------
# Logging / data
# ---------------------------------------------------------------------------
LOG_DIR: str = "data/simulation_logs"
LOG_LEVEL: str = "INFO"
SQLITE_DB_PATH: str = "data/simulation_logs/experiments.db"


@dataclass(frozen=True)
class GridPosition:
    """A simple, hashable (x, y) grid coordinate used throughout the project."""
    x: int
    y: int

    def as_tuple(self) -> Tuple[int, int]:
        return (self.x, self.y)


def robot_address(robot_index: int, host: str = "127.0.0.1") -> Tuple[str, int]:
    """Deterministically map a 0-based robot index to a UDP (host, port) address.

    This is the "peer list" used instead of a central registry (see
    communication/discovery.py). Robot 0 -> port ROBOT_START_PORT, etc.
    """
    return (host, ROBOT_START_PORT + robot_index)
