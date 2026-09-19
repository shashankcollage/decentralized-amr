"""
tests/test_trace_recorder.py
==============================
Verifies TraceRecorder captures frames correctly and produces valid,
loadable JSON - the data source for tools/build_trace_viewer.py.
"""

import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.warehouse import Warehouse
from simulation.simulator import Simulator, SimulatorConfig
from simulation.trace_recorder import TraceRecorder

WAREHOUSE_JSON = os.path.join(os.path.dirname(__file__), "..", "data", "warehouse.json")


def test_recorder_captures_frames_at_configured_interval():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=1))
    sim.spawn_robots(2)
    for i, robot in enumerate(sim.robots.values()):
        robot.set_destination(wh.dropoff_points[i % len(wh.dropoff_points)])

    recorder = TraceRecorder(sim, every_n_ticks=5)
    sim.register_tick_callback(recorder.on_tick)
    sim.run(max_ticks=50)

    assert len(recorder.frames) > 0
    for frame in recorder.frames:
        assert frame["tick"] % 5 == 0


def test_recorder_saved_json_is_valid_and_loadable():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=2))
    sim.spawn_robots(2)
    for i, robot in enumerate(sim.robots.values()):
        robot.set_destination(wh.dropoff_points[i % len(wh.dropoff_points)])

    recorder = TraceRecorder(sim, every_n_ticks=2)
    sim.register_tick_callback(recorder.on_tick)
    sim.run(max_ticks=100)

    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, "trace.json")
        recorder.save(path)
        assert os.path.exists(path)

        with open(path) as f:
            data = json.load(f)

        assert "warehouse" in data
        assert "frames" in data
        assert data["warehouse"]["width"] == wh.width
        assert data["warehouse"]["height"] == wh.height
        assert len(data["frames"]) == len(recorder.frames)


def test_recorder_frame_contains_robot_positions_and_status():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=3))
    sim.spawn_robots(2)
    for i, robot in enumerate(sim.robots.values()):
        robot.set_destination(wh.dropoff_points[i % len(wh.dropoff_points)])

    recorder = TraceRecorder(sim, every_n_ticks=1)
    sim.register_tick_callback(recorder.on_tick)
    sim.run(max_ticks=10)

    frame = recorder.frames[0]
    assert set(frame["robots"].keys()) == {"R1", "R2"}
    for robot_data in frame["robots"].values():
        assert "position" in robot_data
        assert "status" in robot_data
        assert "battery" in robot_data
