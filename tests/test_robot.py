"""
tests/test_robot.py
====================
Phase 1 tests: warehouse loading/queries and basic single-robot movement.
Later phases add test_astar.py, test_collision.py, test_deadlock.py,
test_task_allocation.py, and test_communication.py.
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.warehouse import Warehouse, CellType
from simulation.simulator import Simulator, SimulatorConfig
from robots.robot import Robot
from robots.robot_state import RobotStatus
import config


WAREHOUSE_JSON = os.path.join(os.path.dirname(__file__), "..", "data", "warehouse.json")


def test_warehouse_loads_from_json():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    assert wh.width == 20
    assert wh.height == 20
    assert len(wh.pickup_points) == 3
    assert len(wh.dropoff_points) == 3
    assert len(wh.charging_stations) == 2


def test_warehouse_perimeter_is_wall():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    for x in range(wh.width):
        assert wh.cell_type((x, 0)) == CellType.WALL
        assert wh.cell_type((x, wh.height - 1)) == CellType.WALL


def test_shelf_cells_are_statically_blocked():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    assert wh.cell_type((4, 3)) == CellType.SHELF
    assert wh.is_statically_blocked((4, 3)) is True
    assert wh.is_passable((4, 3)) is False


def test_empty_cell_is_passable():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    assert wh.is_passable((10, 10)) is True


def test_dynamic_block_and_unblock():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    cell = (10, 10)
    assert wh.is_passable(cell) is True
    wh.block_aisle([cell])
    assert wh.is_passable(cell) is False
    wh.unblock_aisle([cell])
    assert wh.is_passable(cell) is True


def test_robot_moves_towards_destination():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    robot = Robot("R1", start_position=(2, 2), warehouse=wh)
    robot.set_destination((2, 10))

    now = 0.0
    for _ in range(200):
        robot.tick(dt=0.1, now=now)
        now += 0.1
        if robot.status == RobotStatus.COMPLETED:
            break

    assert robot.status == RobotStatus.COMPLETED
    assert robot.position == (2, 10)
    assert robot.state.distance_travelled > 0


def test_robot_battery_decreases_while_moving():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    robot = Robot("R1", start_position=(2, 2), warehouse=wh)
    robot.set_destination((2, 10))
    initial_battery = robot.state.battery

    for i in range(50):
        robot.tick(dt=0.1, now=i * 0.1)

    assert robot.state.battery < initial_battery


def test_robot_with_no_destination_stays_idle():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    robot = Robot("R1", start_position=(2, 2), warehouse=wh)
    robot.tick(dt=0.1, now=0.0)
    assert robot.position == (2, 2)
    assert robot.status == RobotStatus.IDLE


def test_simulator_spawns_requested_number_of_robots():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=1))
    sim.spawn_robots(3)
    assert len(sim.robots) == 3
    assert set(sim.robots.keys()) == {"R1", "R2", "R3"}


def test_simulator_spawned_robots_do_not_overlap():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=7))
    sim.spawn_robots(5)
    positions = [r.position for r in sim.robots.values()]
    assert len(positions) == len(set(positions))


def test_simulator_runs_and_completes_all_robots():
    wh = Warehouse.from_json(WAREHOUSE_JSON)
    sim = Simulator(wh, SimulatorConfig(seed=42))
    sim.spawn_robots(3)
    for i, robot in enumerate(sim.robots.values()):
        robot.set_destination(wh.dropoff_points[i % len(wh.dropoff_points)])

    sim.run(max_ticks=2000)
    assert sim.all_robots_done()
    for robot in sim.robots.values():
        assert robot.status == RobotStatus.COMPLETED


def test_simulator_deterministic_with_same_seed():
    wh1 = Warehouse.from_json(WAREHOUSE_JSON)
    sim1 = Simulator(wh1, SimulatorConfig(seed=99))
    sim1.spawn_robots(3)

    wh2 = Warehouse.from_json(WAREHOUSE_JSON)
    sim2 = Simulator(wh2, SimulatorConfig(seed=99))
    sim2.spawn_robots(3)

    assert sim1.robot_positions() == sim2.robot_positions()


# -- Battery tests (project spec: normal / low / critical) -----------------
from robots.battery import BatteryManager
from robots.robot_state import BatteryState


def test_battery_normal_state():
    b = BatteryManager(level=80.0)
    assert b.state() == BatteryState.NORMAL
    assert b.can_accept_new_task()
    assert not b.should_seek_charging()


def test_battery_low_state():
    b = BatteryManager(level=config.LOW_BATTERY - 1)
    assert b.state() == BatteryState.LOW
    assert not b.can_accept_new_task()
    assert b.should_seek_charging()


def test_battery_critical_state():
    b = BatteryManager(level=config.CRITICAL_BATTERY - 1)
    assert b.state() == BatteryState.CRITICAL
    assert not b.can_accept_new_task()
    assert b.should_seek_charging()


def test_battery_charge_and_drain():
    b = BatteryManager(level=50.0)
    b.drain_for_movement(2.0)
    assert b.level < 50.0
    b.charge(dt=1.0)
    assert b.level > 0.0
    b2 = BatteryManager(level=99.9)
    b2.charge(dt=10.0)
    assert b2.is_fully_charged()
    assert b2.level == 100.0
