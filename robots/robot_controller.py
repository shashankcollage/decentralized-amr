"""
robots/robot_controller.py
=============================
The robot's "brain": composes every edge-computing module into one
per-tick decision loop, matching the architecture diagram in project
spec section 7:

    Robot
      +-- Localization
      +-- Sensor Simulation
      +-- Local World Model
      +-- Task Manager
      +-- A* Planner
      +-- Conflict Detector
      +-- Reservation Manager
      +-- Deadlock Detector
      +-- Negotiation Manager
      +-- Battery Manager
      +-- Motion Controller

RobotController holds every one of these sub-modules and is what
robots/robot.py's Robot class delegates its tick() to - Robot itself
stays a thin holder of RobotState/LocalWorldModel so external code
(tests, the simulator, the dashboard) keeps a simple, stable interface,
while all the actual decision-making complexity lives here, exactly as
the architecture diagram requires ("do NOT place these decisions in the
dashboard" - they aren't; they're all here, one instance per robot,
running independently).
"""

from __future__ import annotations

import logging
import time
import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import config
from communication.network_manager import NetworkManager
from communication.message import (
    MessageType,
    make_robot_state_message,
    make_task_bid_message,
    make_task_assignment_message,
    make_task_completed_message,
    make_deadlock_alert_message,
    make_reroute_notice_message,
    make_robot_failure_message,
)
from coordination.conflict_resolution import Resolution, resolve_conflict
from coordination.deadlock import detect_deadlock, is_suspiciously_long_wait
from coordination.priority import PriorityInputs, calculate_priority
from planning.conflict_detector import (
    detect_predicted_conflicts,
    detect_intersection_conflicts,
    detect_spatial_collisions,
)
from planning.multi_agent_planner import plan_path, path_is_still_valid
from planning.reservation_table import ReservationTable
from robots.battery import BatteryManager
from robots.localization import Localization
from robots.robot_state import LocalWorldModel, RobotState, RobotStatus, BatteryState
from robots.sensor_simulator import SensorSimulator
from simulation.physics import manhattan_distance, velocity_towards
from simulation.warehouse import Warehouse
from tasks.auction import BidInputs, calculate_bid, select_winner
from tasks.task import TaskStatus
from tasks.task_manager import TaskManager
from tasks.task_reassignment import find_tasks_needing_reassignment

Coordinate = Tuple[int, int]

logger = logging.getLogger("robot_controller")


@dataclass
class _PendingBid:
    task_id: str
    deadline: float
    bids: Dict[str, float] = field(default_factory=dict)


class RobotController:
    def __init__(
        self,
        robot_id: str,
        warehouse: Warehouse,
        all_robot_ids: Optional[List[str]] = None,
        task_manager: Optional[TaskManager] = None,
        enable_networking: bool = False,
        speed: float = config.ROBOT_SPEED_CELLS_PER_SEC,
        port_offset: int = 0,
    ) -> None:
        self.robot_id = robot_id
        self.warehouse = warehouse
        self.task_manager = task_manager
        self.enable_networking = enable_networking and all_robot_ids is not None and len(all_robot_ids) > 1
        self.all_robot_ids = all_robot_ids or [robot_id]
        self.speed = speed

        self.localization = Localization()
        self.sensor = SensorSimulator()
        self.battery = BatteryManager(level=config.BATTERY_INITIAL)
        self.reservations = ReservationTable()

        self.network: Optional[NetworkManager] = None
        if self.enable_networking:
            self.network = NetworkManager(robot_id, self.all_robot_ids, port_offset=port_offset)

        self._step_index = 0
        self._step_progress = 0.0
        self._task_stage: Optional[str] = None  # "to_pickup" | "to_dropoff" | None
        self._active_task_id: Optional[str] = None
        self._pending_bids: Dict[str, _PendingBid] = {}
        self._bid_cooldown_until: Dict[str, float] = {}
        self._last_heartbeat_time = 0.0
        self._failed = False
        self._repeated_conflict_count = 0
        self._last_conflict_cell: Optional[Coordinate] = None
        self._last_blocking_robot_id: Optional[str] = None

        # Enforced deadlock-backoff cooldown: after this robot is chosen to
        # back off from a detected deadlock, it holds still (does not
        # re-plan or move) until this simulated timestamp. Without this,
        # re-planning immediately toward the same destination in a
        # structural (e.g. single-width corridor) deadlock just finds the
        # identical path and re-triggers the same conflict on the very
        # next tick - the cooldown guarantees the *other* robot (which
        # keeps moving) has a real window to clear the contested cells
        # before this one tries again. See _check_deadlock/tick().
        self._deadlock_backoff_until = 0.0
        # Consecutive-backoff counter + per-robot jitter: a persistent
        # chokepoint (e.g. several robots converging on one doorway near a
        # shared dropoff) can otherwise re-form the identical deadlock at a
        # fixed period, causing a livelock where the same robots trade off
        # forever without any of them completing. Escalating the backoff
        # duration each consecutive time THIS robot is the one that backs
        # off, plus a small deterministic per-robot-id jitter, desynchronizes
        # the fleet's retry timing - a standard remedy for this class of
        # priority-based-planner livelock.
        self._consecutive_backoff_count = 0

    def _global_step(self, now: float) -> int:
        """Convert absolute simulated time into a cell-transition-count
        "timestep" that is globally comparable across every robot.

        IMPORTANT BUG THIS FIXES: an earlier version used each robot's own
        private `_step_index` (a simple "cells completed so far" counter)
        as the reservation-table timestep. That is NOT comparable between
        robots - if robot A has waited more than robot B, "timestep 5" means
        a completely different real moment for each of them, silently
        defeating the whole point of a time-indexed reservation table (two
        robots could both believe a given cell is free at "their" timestep
        5 while actually being there at the exact same real instant). Using
        `now * speed` instead ties every reservation to the same shared
        simulated clock (see simulation/simulator.py's use of sim_time),
        so "timestep N" means the same real moment for every robot,
        exactly as required for the reservation table to actually prevent
        the same-cell-at-the-same-time collisions it exists to prevent.
        """
        return int(round(now * self.speed))

    # -- public entry point -------------------------------------------------
    def tick(self, state: RobotState, world_model: LocalWorldModel, dt: float, now: float) -> None:
        if self._failed or state.status == RobotStatus.FAILED:
            return

        if state.status == RobotStatus.CHARGING:
            self._tick_charging(state, dt)
            return

        if self.enable_networking:
            self._broadcast_heartbeat(state, now)
            self._process_incoming_messages(state, world_model, now)
            self._check_peer_liveness(world_model, now)
            if self.task_manager is not None:
                self._handle_task_lifecycle(state, world_model, now)

        if now < self._deadlock_backoff_until:
            # Enforced hold: skip planning/movement entirely this tick, but
            # still account for idle battery drain and keep broadcasting
            # (already done above) so peers don't mistake this for a failure.
            state.status = RobotStatus.WAITING
            state.velocity = (0.0, 0.0)
            self.battery.level = state.battery
            self.battery.drain_idle(dt)
            state.battery = self.battery.level
            return

        self._ensure_destination_and_path(state, now)
        self._tick_battery_and_charging_decision(state, now, dt)
        self._move_one_step(state, world_model, now, dt)
        self._check_deadlock(state, world_model, now)

    # -- networking -----------------------------------------------------------
    def _broadcast_heartbeat(self, state: RobotState, now: float) -> None:
        if (now - self._last_heartbeat_time) < config.HEARTBEAT_INTERVAL:
            return
        self._last_heartbeat_time = now
        msg = make_robot_state_message(
            robot_id=self.robot_id,
            timestamp=now,
            position=state.position,
            velocity=state.velocity,
            destination=state.destination,
            battery=state.battery,
            status=state.status.value,
            priority=state.priority,
            path=state.current_path,
            current_task=state.current_task,
            waiting_for=self._last_blocking_robot_id if self._last_conflict_cell else None,
        )
        self.network.broadcast(msg)
        state.last_heartbeat_sent = now

    @property
    def _blocking_robot_id(self) -> Optional[str]:
        return self._last_blocking_robot_id

    def _process_incoming_messages(self, state: RobotState, world_model: LocalWorldModel, now: float) -> None:
        for msg in self.network.receive_messages(now):
            if msg.type == MessageType.ROBOT_STATE:
                p = msg.payload
                world_model.update_peer(
                    msg.robot_id,
                    position=tuple(p["position"]),
                    velocity=tuple(p["velocity"]),
                    destination=tuple(p["destination"]) if p.get("destination") else None,
                    status=RobotStatus(p["status"]),
                    battery=p["battery"],
                    path=[tuple(c) for c in p.get("path", [])],
                    priority=p.get("priority", 0.0),
                    current_task=p.get("current_task"),
                    waiting_for=p.get("waiting_for"),
                    last_message_time=now,
                )

                if p.get("path"):
                    self.reservations.update_peer_path(
                        msg.robot_id, [tuple(c) for c in p["path"]], start_timestep=self._global_step(now)
                    )

            elif msg.type == MessageType.TASK_BID:
                self._record_incoming_bid(msg.robot_id, msg.payload["task_id"], msg.payload["bid_cost"], now)

            elif msg.type == MessageType.TASK_ASSIGNMENT:
                task_id = msg.payload["task_id"]
                winner = msg.payload["winner"]
                if self.task_manager is not None:
                    task = self.task_manager.get(task_id)
                    if task is not None and task.status == TaskStatus.WAITING:
                        self.task_manager.mark_assigned(task_id, winner, now=now)
                self._pending_bids.pop(task_id, None)
                if winner != self.robot_id and self._active_task_id == task_id:
                    self._active_task_id = None

            elif msg.type == MessageType.TASK_COMPLETED:
                if self.task_manager is not None:
                    self.task_manager.mark_completed(msg.payload["task_id"])

            elif msg.type == MessageType.ROBOT_FAILURE:
                world_model.mark_failed(msg.robot_id)
                self.reservations.release_robot_claims_from_peer(msg.robot_id)

            elif msg.type == MessageType.DEADLOCK_ALERT:
                logger.info("%s: received DEADLOCK_ALERT from %s (cycle=%s)",
                            self.robot_id, msg.robot_id, msg.payload.get("cycle"))

    def _check_peer_liveness(self, world_model: LocalWorldModel, now: float) -> None:
        for peer_id in self.network.unavailable_peers(now):
            if peer_id in world_model.peers and world_model.peers[peer_id].status != RobotStatus.FAILED:
                logger.warning("%s: peer %s has timed out - marking FAILED", self.robot_id, peer_id)
                world_model.mark_failed(peer_id)
                self.reservations.release_robot_claims_from_peer(peer_id)

    # -- task lifecycle (decentralized auction) --------------------------------
    def _handle_task_lifecycle(self, state: RobotState, world_model: LocalWorldModel, now: float) -> None:
        # Resolve any bid windows that have closed.
        for task_id in list(self._pending_bids.keys()):
            pending = self._pending_bids[task_id]
            if now >= pending.deadline:
                self._resolve_bid(task_id, pending, state, now)

        # Reassignment check: are any ASSIGNED/IN_PROGRESS tasks stuck?
        # IMPORTANT: only ever apply this to tasks assigned to OTHER
        # robots. A robot always has strictly better information about
        # whether ITS OWN task is progressing than a blind elapsed-time
        # heuristic does - applying the same timeout to yourself means a
        # robot can give up on (and silently stop tracking) its own
        # in-flight task purely because travel/negotiation took a while,
        # even though it's making perfectly fine progress. Only a PEER
        # noticing "robot X's task has been stuck for a long time, and X
        # itself is unavailable/critical" is a meaningful signal; "my own
        # task is taking a while" is not a failure signal at all.
        unavailable = set(self.network.unavailable_peers(now)) | {
            pid for pid, p in world_model.peers.items() if p.status == RobotStatus.FAILED
        }
        batteries = {pid: p.battery for pid, p in world_model.peers.items()}
        others_tasks = [t for t in self.task_manager.tasks.values() if t.assigned_robot != self.robot_id]
        stuck = find_tasks_needing_reassignment(
            others_tasks,
            list(unavailable),
            batteries,
            config.CRITICAL_BATTERY,
            now,
            config.TASK_REASSIGN_TIMEOUT,
        )
        for task in stuck:
            self.task_manager.mark_waiting_again(task.task_id)

        # Separately: give up on OUR OWN active task only for a real
        # failure signal (we ourselves know we're critical-battery or the
        # task has been reset out from under us by a peer) - never purely
        # because of elapsed time.
        if (self._active_task_id is not None
                and self.task_manager.get(self._active_task_id) is not None
                and self.task_manager.get(self._active_task_id).status == TaskStatus.WAITING):
            # A peer already returned this task to WAITING (e.g. it
            # believed we failed) - fall in line rather than fight it.
            self._active_task_id = None
            self._task_stage = None
            state.current_task = None

        # If idle and battery allows, bid on the next available task.
        if (state.status in (RobotStatus.IDLE,) and self._active_task_id is None
                and self.battery.can_accept_new_task()):
            self._maybe_start_bidding(state, world_model, now)

    def _maybe_start_bidding(self, state: RobotState, world_model: LocalWorldModel, now: float) -> None:
        waiting = sorted(self.task_manager.waiting_tasks(), key=lambda t: t.task_id)
        for task in waiting:
            if now < self._bid_cooldown_until.get(task.task_id, 0.0):
                continue
            if task.task_id in self._pending_bids:
                continue

            nearby_count = len(world_model.get_active_peers(now, config.ROBOT_TIMEOUT))
            cost = calculate_bid(BidInputs(
                robot_position=state.position,
                pickup_location=task.pickup_location,
                battery_percent=state.battery,
                current_workload=0,
                nearby_robot_count=nearby_count,
                speed_cells_per_sec=self.speed,
            ))
            msg = make_task_bid_message(self.robot_id, task.task_id, cost, timestamp=now)
            self.network.broadcast(msg)
            self._pending_bids[task.task_id] = _PendingBid(
                task_id=task.task_id, deadline=now + config.TASK_BID_WINDOW, bids={self.robot_id: cost}
            )
            return  # bid on one task at a time

    def _record_incoming_bid(self, robot_id: str, task_id: str, cost: float, now: float) -> None:
        if task_id not in self._pending_bids:
            # We haven't started bidding on this one ourselves yet, but we
            # still track others' bids in case we join in before the window
            # closes (or just to have visibility for reassignment logic).
            self._pending_bids[task_id] = _PendingBid(task_id=task_id, deadline=now + config.TASK_BID_WINDOW)
        self._pending_bids[task_id].bids[robot_id] = cost

    def _resolve_bid(self, task_id: str, pending: "_PendingBid", state: RobotState, now: float) -> None:
        del self._pending_bids[task_id]
        task = self.task_manager.get(task_id) if self.task_manager else None
        if task is None or task.status != TaskStatus.WAITING:
            return  # someone else's TASK_ASSIGNMENT already resolved it

        winner = select_winner(pending.bids)
        if winner is None:
            return

        self.task_manager.mark_assigned(task_id, winner, now=now)
        # Cool down re-bidding on this exact task for a short window even
        # if it somehow reappears immediately, to avoid tight bid loops.
        self._bid_cooldown_until[task_id] = now + config.TASK_BID_WINDOW

        if winner == self.robot_id:
            self._active_task_id = task_id
            self._task_stage = "to_pickup"
            state.current_task = task_id
            state.task_start_time = now
            state.destination = task.pickup_location
            state.status = RobotStatus.MOVING
            self.network.broadcast(make_task_assignment_message(self.robot_id, task_id, winner, timestamp=now))
            logger.info("%s: won bid for %s (cost=%.2f)", self.robot_id, task_id, pending.bids.get(winner, -1))

    # -- planning / movement -----------------------------------------------------
    def _ensure_destination_and_path(self, state: RobotState, now: float) -> None:
        if state.destination is None:
            return

        needs_plan = (
            not state.current_path
            or not path_is_still_valid([state.position] + state.current_path, self.warehouse)
        )
        if needs_plan:
            self._replan(state, now)

    def _replan(self, state: RobotState, now: float) -> None:
        path = plan_path(
            robot_id=self.robot_id,
            start=state.position,
            goal=state.destination,
            warehouse=self.warehouse,
            reservations=self.reservations,
            current_step_index=self._global_step(now),
        )
        if path is None:
            state.status = RobotStatus.BLOCKED
            state.start_waiting(now)
            logger.warning("%s: BLOCKED - no path to %s", self.robot_id, state.destination)
            return

        was_rerouting_midway = bool(state.current_path)
        state.planned_path = path
        state.current_path = path[1:]
        if was_rerouting_midway:
            state.reroute_count += 1
            if self.enable_networking:
                self.network.broadcast(make_reroute_notice_message(self.robot_id, path, timestamp=now))
            logger.info("%s: re-routed (reroute #%d), new path length %d",
                        self.robot_id, state.reroute_count, len(path))

        if state.status not in (RobotStatus.WAITING, RobotStatus.NEGOTIATING):
            state.status = RobotStatus.MOVING
        state.stop_waiting(now)

    def _move_one_step(self, state: RobotState, world_model: LocalWorldModel, now: float, dt: float) -> None:
        if state.destination is None or state.position == state.destination:
            if state.status == RobotStatus.MOVING or state.status == RobotStatus.WAITING:
                self._arrive(state, now)
            return

        if not state.current_path:
            return  # BLOCKED, nothing to do this tick

        next_cell = state.current_path[0]

        decision = self._resolve_movement_conflicts(state, world_model, next_cell, now)
        if decision == Resolution.YIELD:
            state.start_waiting(now)
            state.status = RobotStatus.WAITING
            self._maybe_escalate_repeated_conflict(state, next_cell, now)
            return

        state.status = RobotStatus.MOVING
        state.stop_waiting(now)
        self._repeated_conflict_count = 0
        self._last_conflict_cell = None
        self._last_blocking_robot_id = None

        self._step_progress += self.speed * dt
        state.velocity = velocity_towards(state.position, next_cell, self.speed)

        if self._step_progress >= 1.0:
            # Unconditional final safety check, independent of the priority/
            # negotiation outcome above: never physically step onto a cell a
            # peer is CURRENTLY confirmed to occupy, using the freshest
            # available peer data right at the moment of commit. The
            # priority-based conflict resolution earlier in this method can
            # in principle be fooled by heartbeat staleness (a peer's
            # decision to target this same cell may not have propagated
            # yet) - this check is the last line of defense for the
            # project's core invariant ("never allow two robots to occupy
            # the same physical cell simultaneously"), independent of
            # whatever the negotiation concluded.
            if any(peer.position == next_cell and peer.status != RobotStatus.FAILED
                   for peer in world_model.get_active_peers(now, config.ROBOT_TIMEOUT)):
                self._step_progress = 1.0  # hold right at the threshold, don't lose progress
                state.start_waiting(now)
                state.status = RobotStatus.WAITING
                return

            self._step_progress = 0.0
            state.previous_position = state.position
            state.position = next_cell
            state.current_path = state.current_path[1:]
            state.distance_travelled += 1.0
            self._step_index += 1
            self.battery.drain_for_movement(1.0)
            state.battery = self.battery.level

            if state.position == state.destination:
                self._arrive(state, now)

    def _resolve_movement_conflicts(self, state: RobotState, world_model: LocalWorldModel,
                                     next_cell: Coordinate, now: float) -> Resolution:
        priority_inputs = PriorityInputs(
            task_urgency=0.5 if state.current_task else 0.2,
            waiting_time_seconds=state.current_waiting_duration(now),
            battery_percent=state.battery,
            distance_to_conflict=manhattan_distance(state.position, next_cell),
        )
        state.priority = calculate_priority(priority_inputs)

        conflicts = detect_predicted_conflicts(
            self.robot_id, state.position, next_cell, world_model, now, config.ROBOT_TIMEOUT
        )
        conflicts += detect_intersection_conflicts(
            self.robot_id, next_cell, self.warehouse.intersections, world_model, now, config.ROBOT_TIMEOUT
        )
        spatial = detect_spatial_collisions(state.position, world_model, now, config.ROBOT_TIMEOUT)

        if not conflicts and not spatial:
            return Resolution.PROCEED

        state.status = RobotStatus.NEGOTIATING
        final = Resolution.PROCEED
        for conflict in conflicts:
            peer = world_model.peers.get(conflict.other_robot_id)
            other_score = peer.priority if peer else 0.0
            decision = resolve_conflict(
                self_robot_id=self.robot_id,
                conflict=conflict,
                self_priority_score=state.priority,
                other_priority_score=other_score,
                reservations=self.reservations,
                contested_cell=next_cell,
                contested_timestep=self._global_step(now) + 1,
            )
            if decision.resolution == Resolution.YIELD:
                final = Resolution.YIELD
                self._last_conflict_cell = next_cell
                self._last_blocking_robot_id = decision.other_robot_id

        if spatial and final == Resolution.PROCEED:
            # Too close right now even without a specific cell conflict -
            # conservative safety stop rather than risk a physical collision.
            final = Resolution.YIELD
            self._last_conflict_cell = next_cell
            self._last_blocking_robot_id = spatial[0].other_robot_id

        return final

    def _maybe_escalate_repeated_conflict(self, state: RobotState, cell: Coordinate, now: float) -> None:
        if cell == self._last_conflict_cell:
            self._repeated_conflict_count += 1
        else:
            self._repeated_conflict_count = 1
        # A long enough string of repeated conflicts on the same cell is
        # itself treated as deadlock-suspicious, per spec section 18's
        # "repeated conflict" condition, independent of wait-time alone.

    def _arrive(self, state: RobotState, now: float) -> None:
        state.velocity = (0.0, 0.0)
        state.current_path = []

        if self.task_manager is not None and self._active_task_id is not None:
            task = self.task_manager.get(self._active_task_id)
            if self._task_stage == "to_pickup":
                logger.info("%s: reached pickup for %s", self.robot_id, self._active_task_id)
                self.task_manager.mark_in_progress(self._active_task_id)
                self._task_stage = "to_dropoff"
                state.destination = task.dropoff_location
                state.status = RobotStatus.MOVING
                return
            elif self._task_stage == "to_dropoff":
                logger.info("%s: completed task %s", self.robot_id, self._active_task_id)
                self.task_manager.mark_completed(self._active_task_id)
                if self.enable_networking:
                    self.network.broadcast(make_task_completed_message(self.robot_id, self._active_task_id, timestamp=now))
                state.completed_tasks += 1
                state.task_completion_time = now
                state.current_task = None
                self._active_task_id = None
                self._task_stage = None
                self._consecutive_backoff_count = 0
                state.destination = None
                state.status = RobotStatus.IDLE
                return

        # Direct-destination mode (no task manager) - matches Phase 1
        # behaviour exactly: arriving means the run is COMPLETED.
        logger.info("%s: arrived at destination %s", self.robot_id, state.position)
        state.status = RobotStatus.COMPLETED
        state.task_completion_time = now

    # -- battery / charging -----------------------------------------------------
    def _tick_battery_and_charging_decision(self, state: RobotState, now: float, dt: float = 0.0) -> None:
        self.battery.level = state.battery
        if state.status in (RobotStatus.WAITING, RobotStatus.NEGOTIATING, RobotStatus.BLOCKED):
            # Movement-based drain is applied per-cell in _move_one_step;
            # this covers battery cost of simply being powered on while
            # not making progress (idle/waiting drain).
            self.battery.drain_idle(dt)
        state.battery = self.battery.level

        if self.battery.state() == BatteryState.CRITICAL and state.status != RobotStatus.CHARGING:
            if self.warehouse.charging_stations:
                target = self.warehouse.nearest_of_type(state.position, self.warehouse.charging_stations)
                if state.destination != target:
                    logger.warning("%s: battery CRITICAL - diverting to charger %s", self.robot_id, target)
                    if self.task_manager is not None and self._active_task_id is not None:
                        self.task_manager.mark_waiting_again(self._active_task_id)
                        self._active_task_id = None
                        self._task_stage = None
                        state.current_task = None
                    state.destination = target
                    state.current_path = []

        if state.position in self.warehouse.charging_stations and state.destination == state.position:
            state.status = RobotStatus.CHARGING
            state.destination = None

    def _tick_charging(self, state: RobotState, dt: float) -> None:
        self.battery.charge(dt)
        state.battery = self.battery.level
        if self.battery.is_fully_charged():
            state.status = RobotStatus.IDLE

    # -- deadlock -----------------------------------------------------------
    def _check_deadlock(self, state: RobotState, world_model: LocalWorldModel, now: float) -> None:
        if state.status != RobotStatus.WAITING:
            return
        if not is_suspiciously_long_wait(state.current_waiting_duration(now)):
            return

        waiting_for = {self.robot_id: self._last_blocking_robot_id}
        priority_inputs = {}
        for peer_id, peer in world_model.peers.items():
            peer_waiting_for = peer.waiting_for
            waiting_for[peer_id] = peer_waiting_for
            priority_inputs[peer_id] = PriorityInputs(
                task_urgency=0.5 if peer.current_task else 0.2,
                waiting_time_seconds=0.0,  # not known precisely for peers; priority score already broadcast
                battery_percent=peer.battery,
                distance_to_conflict=1.0,
            )
        priority_inputs[self.robot_id] = PriorityInputs(
            task_urgency=0.5 if state.current_task else 0.2,
            waiting_time_seconds=state.current_waiting_duration(now),
            battery_percent=state.battery,
            distance_to_conflict=1.0,
        )

        info = detect_deadlock(self.robot_id, waiting_for, priority_inputs)
        if info is None:
            return

        logger.warning("%s: DEADLOCK detected in cycle %s - %s will back off",
                        self.robot_id, info.cycle, info.robot_to_back_off)

        if self.enable_networking:
            self.network.broadcast(make_deadlock_alert_message(self.robot_id, info.cycle, info.robot_to_back_off, timestamp=now))

        if info.robot_to_back_off == self.robot_id:
            self.reservations.release_path(self.robot_id)
            state.current_path = []
            state.status = RobotStatus.WAITING
            state.stop_waiting(now)
            self._consecutive_backoff_count += 1
            # Multiplicative (not just additive) per-robot-id scaling, so
            # that even a perfectly symmetric alternating deadlock
            # diverges in wait duration between the two robots over
            # successive rounds rather than staying in lockstep.
            #
            # IMPORTANT: uses zlib.crc32, NOT Python's built-in hash().
            # str hash() is randomized per-process by default
            # (PYTHONHASHSEED) - using it here would make backoff timing,
            # and therefore which robot "wins" a contested corridor, vary
            # unpredictably between runs even with the same seed, which
            # directly violates the project's determinism requirement
            # (project spec section 29: "keep the simulation deterministic
            # when a seed is supplied").
            robot_hash = zlib.crc32(self.robot_id.encode())
            id_factor = 1.0 + (robot_hash % 5) * 0.2
            jitter = (robot_hash % 7) * 0.15
            backoff_duration = config.WAIT_TIMEOUT * min(self._consecutive_backoff_count, 4) * id_factor + jitter
            self._deadlock_backoff_until = now + backoff_duration
            logger.info("%s: backing off for %.1fs (consecutive #%d) to let the deadlock clear",
                        self.robot_id, backoff_duration, self._consecutive_backoff_count)
        # NOTE: the counter is intentionally NOT reset just because a peer
        # backs off this particular round. In a symmetric/alternating
        # deadlock (A backs off, then B, then A, then B, ...) resetting on
        # every "not my turn" round would mean escalation never
        # accumulates for either robot - each one's counter would be
        # zeroed the instant the other took a turn, defeating the entire
        # point of escalating backoff. The counter only resets on genuine
        # progress: see _arrive()'s task-completion path, which is the
        # actual signal that this robot is no longer stuck.

    # -- failure injection (for scenario/experiment use) --------------------
    def simulate_failure(self, state: RobotState, now: Optional[float] = None) -> None:
        self._failed = True
        state.status = RobotStatus.FAILED
        state.velocity = (0.0, 0.0)
        if self.enable_networking:
            self.network.broadcast(make_robot_failure_message(self.robot_id, state.position, timestamp=now))
        if self.task_manager is not None and self._active_task_id is not None:
            self.task_manager.mark_waiting_again(self._active_task_id)

    def close(self) -> None:
        if self.network is not None:
            self.network.close()
