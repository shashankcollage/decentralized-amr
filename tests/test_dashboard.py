"""
tests/test_dashboard.py
=========================
Tests dashboard/app.py using Flask's test client (no real socket/server
needed). Verifies the dashboard is genuinely read-only monitoring: the
API surfaces state and can pause/resume the simulation clock, but never
sets a robot's position, destination, or path.
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.warehouse import Warehouse
from simulation.simulator import Simulator, SimulatorConfig
from dashboard.app import create_app

WAREHOUSE_JSON = os.path.join(os.path.dirname(__file__), "..", "data", "warehouse.json")


def _make_sim(seed=1, n=3):
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=seed))
    sim.spawn_robots(n)
    for i, r in enumerate(sim.robots.values()):
        r.set_destination(wh.dropoff_points[i % len(wh.dropoff_points)])
    return sim


def test_index_page_serves_html():
    sim = _make_sim()
    client = create_app(sim).test_client()
    resp = client.get("/")
    assert resp.status_code == 200
    assert b"Live Dashboard" in resp.data


def test_api_status_returns_expected_fields():
    sim = _make_sim()
    client = create_app(sim).test_client()
    resp = client.get("/api/status")
    assert resp.status_code == 200
    data = resp.get_json()
    for field in ("sim_time", "tick_count", "running", "paused", "num_robots",
                  "active_robots", "completed_robots", "average_battery",
                  "total_distance", "total_collisions"):
        assert field in data
    assert data["num_robots"] == 3


def test_api_robots_returns_all_robot_states():
    sim = _make_sim()
    client = create_app(sim).test_client()
    resp = client.get("/api/robots")
    data = resp.get_json()
    assert set(data.keys()) == {"R1", "R2", "R3"}
    for robot_data in data.values():
        assert "position" in robot_data
        assert "status" in robot_data


def test_api_warehouse_returns_static_layout():
    sim = _make_sim()
    client = create_app(sim).test_client()
    resp = client.get("/api/warehouse")
    data = resp.get_json()
    assert data["width"] == 20
    assert data["height"] == 20
    assert "blocked_aisles" in data


def test_pause_and_resume_control_the_clock_only():
    sim = _make_sim()
    client = create_app(sim).test_client()

    before = sim.robot_positions()
    resp = client.post("/api/simulation/pause")
    assert resp.get_json() == {"paused": True}
    assert sim.paused is True

    # Pausing must not move any robot or change its destination - it only
    # affects the simulator's own clock/run loop.
    after = sim.robot_positions()
    assert before == after
    for robot in sim.robots.values():
        assert robot.state.destination is not None

    resp = client.post("/api/simulation/resume")
    assert resp.get_json() == {"paused": False}
    assert sim.paused is False


def test_dashboard_never_exposes_a_set_position_or_set_destination_route():
    """The dashboard's Flask routes must only ever read/pause/resume -
    there should be no endpoint that lets a client dictate where a robot
    goes, which would violate the decentralization requirement."""
    sim = _make_sim()
    app = create_app(sim)
    rule_paths = {rule.rule for rule in app.url_map.iter_rules()}
    for forbidden_fragment in ("set_position", "set_destination", "move", "goto"):
        assert not any(forbidden_fragment in path for path in rule_paths)
