"""
metrics/database.py
======================
SQLite persistence for experiment metrics, per project spec section 35.

Tables: robots, tasks, events, metrics, experiments. Each row stores
timestamp, experiment_id, robot_id, event_type, event_data (JSON text)
as specified. This is a thin, dependency-free wrapper around the
stdlib sqlite3 module - no ORM needed for this scale of data.
"""

from __future__ import annotations

import json
import sqlite3
import time
from typing import Optional

import config

SCHEMA = """
CREATE TABLE IF NOT EXISTS experiments (
    experiment_id TEXT PRIMARY KEY,
    scenario_name TEXT,
    controller_type TEXT,
    started_at REAL,
    finished_at REAL
);
CREATE TABLE IF NOT EXISTS robots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id TEXT,
    robot_id TEXT,
    final_status TEXT,
    completed_tasks INTEGER,
    distance_travelled REAL,
    battery REAL
);
CREATE TABLE IF NOT EXISTS tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id TEXT,
    task_id TEXT,
    status TEXT,
    assigned_robot TEXT,
    completion_time REAL
);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id TEXT,
    timestamp REAL,
    robot_id TEXT,
    event_type TEXT,
    event_data TEXT
);
CREATE TABLE IF NOT EXISTS metrics (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id TEXT,
    metric_name TEXT,
    metric_value REAL
);
"""


class MetricsDatabase:
    def __init__(self, db_path: str = config.SQLITE_DB_PATH) -> None:
        import os
        os.makedirs(os.path.dirname(db_path) or ".", exist_ok=True)
        self.conn = sqlite3.connect(db_path)
        self.conn.executescript(SCHEMA)
        self.conn.commit()

    def start_experiment(self, experiment_id: str, scenario_name: str, controller_type: str) -> None:
        self.conn.execute(
            "INSERT OR REPLACE INTO experiments (experiment_id, scenario_name, controller_type, started_at) "
            "VALUES (?, ?, ?, ?)",
            (experiment_id, scenario_name, controller_type, time.time()),
        )
        self.conn.commit()

    def finish_experiment(self, experiment_id: str) -> None:
        self.conn.execute("UPDATE experiments SET finished_at = ? WHERE experiment_id = ?",
                           (time.time(), experiment_id))
        self.conn.commit()

    def log_event(self, experiment_id: str, robot_id: Optional[str], event_type: str,
                  event_data: dict, timestamp: Optional[float] = None) -> None:
        self.conn.execute(
            "INSERT INTO events (experiment_id, timestamp, robot_id, event_type, event_data) "
            "VALUES (?, ?, ?, ?, ?)",
            (experiment_id, timestamp if timestamp is not None else time.time(),
             robot_id, event_type, json.dumps(event_data)),
        )
        self.conn.commit()

    def log_metric(self, experiment_id: str, metric_name: str, metric_value: float) -> None:
        self.conn.execute(
            "INSERT INTO metrics (experiment_id, metric_name, metric_value) VALUES (?, ?, ?)",
            (experiment_id, metric_name, metric_value),
        )
        self.conn.commit()

    def log_robot_final_state(self, experiment_id: str, robot_id: str, final_status: str,
                                completed_tasks: int, distance_travelled: float, battery: float) -> None:
        self.conn.execute(
            "INSERT INTO robots (experiment_id, robot_id, final_status, completed_tasks, "
            "distance_travelled, battery) VALUES (?, ?, ?, ?, ?, ?)",
            (experiment_id, robot_id, final_status, completed_tasks, distance_travelled, battery),
        )
        self.conn.commit()

    def log_task_final_state(self, experiment_id: str, task_id: str, status: str,
                               assigned_robot: Optional[str], completion_time: Optional[float]) -> None:
        self.conn.execute(
            "INSERT INTO tasks (experiment_id, task_id, status, assigned_robot, completion_time) "
            "VALUES (?, ?, ?, ?, ?)",
            (experiment_id, task_id, status, assigned_robot, completion_time),
        )
        self.conn.commit()

    def close(self) -> None:
        self.conn.close()
