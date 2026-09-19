"""
dashboard/app.py
=================
Flask monitoring dashboard.

CRITICAL DESIGN RULE (see project spec section 32/33): this app is
monitoring/control-of-the-experiment ONLY. It never decides where a robot
goes - it only reads Simulator.summary()/robot_positions() (both
thread-safe, see simulation/simulator.py) and may pause/resume the overall
clock, which is an experiment-control action, not a per-robot movement
decision. If this process is killed, the simulator thread that's actually
driving the robots (see main.py's --dashboard code path) keeps running
completely unaffected - there is no dependency in the other direction.

Endpoints implemented (see project spec section 33):
    GET  /                          -> live HTML dashboard page
    GET  /api/status                -> sim_time, tick_count, running/paused
    GET  /api/robots                 -> per-robot state dict
    GET  /api/warehouse               -> static warehouse layout snapshot
    POST /api/simulation/pause        -> pause the simulation clock
    POST /api/simulation/resume       -> resume the simulation clock
"""

from __future__ import annotations

from flask import Flask, jsonify, render_template

import config
from simulation.simulator import Simulator


def create_app(simulator: Simulator) -> Flask:
    app = Flask(__name__)

    # Warehouse layout never changes shape at runtime (only blocked_aisles
    # does, which IS included in /api/robots-adjacent state below), so it's
    # computed once rather than on every poll.
    warehouse_snapshot = simulator.warehouse.to_render_snapshot()

    @app.route("/")
    def index():
        return render_template(
            "index.html",
            poll_interval_ms=config.DASHBOARD_POLL_INTERVAL_MS,
        )

    @app.route("/api/status")
    def api_status():
        summary = simulator.summary()
        return jsonify({
            "sim_time": summary["sim_time"],
            "tick_count": summary["tick_count"],
            "running": summary["running"],
            "paused": summary["paused"],
            "num_robots": len(summary["robots"]),
            "active_robots": sum(1 for r in summary["robots"].values()
                                  if r["status"] not in ("FAILED", "COMPLETED")),
            "completed_robots": sum(1 for r in summary["robots"].values()
                                     if r["status"] == "COMPLETED"),
            "average_battery": round(
                sum(r["battery"] for r in summary["robots"].values()) / max(len(summary["robots"]), 1), 1
            ),
            "total_distance": round(sum(r["distance_travelled"] for r in summary["robots"].values()), 2),
            "total_collisions": sum(r["collision_count"] for r in summary["robots"].values()),
        })

    @app.route("/api/robots")
    def api_robots():
        summary = simulator.summary()
        return jsonify(summary["robots"])

    @app.route("/api/warehouse")
    def api_warehouse():
        snapshot = dict(warehouse_snapshot)
        with simulator.lock:
            snapshot["blocked_aisles"] = [list(c) for c in simulator.warehouse.blocked_aisles]
        return jsonify(snapshot)

    @app.route("/api/simulation/pause", methods=["POST"])
    def api_pause():
        simulator.pause()
        return jsonify({"paused": True})

    @app.route("/api/simulation/resume", methods=["POST"])
    def api_resume():
        simulator.resume()
        return jsonify({"paused": False})

    @app.route("/api/tasks")
    def api_tasks():
        """Task pool state, if a TaskManager was attached to this run's
        robots (task-driven mode). Read-only - the dashboard never
        creates, assigns, or cancels tasks itself."""
        for robot in simulator.robots.values():
            tm = getattr(robot.controller, "task_manager", None)
            if tm is not None:
                return jsonify(tm.to_dict())
        return jsonify({})

    @app.route("/api/conflicts")
    def api_conflicts():
        """Best-effort snapshot of robots currently in NEGOTIATING status,
        as a proxy for "conflicts happening right now" - the dashboard
        does not run its own conflict detection (that stays inside each
        robot); it only reports what robots already computed."""
        summary = simulator.summary()
        negotiating = {rid: r for rid, r in summary["robots"].items() if r["status"] == "NEGOTIATING"}
        return jsonify(negotiating)

    @app.route("/api/deadlocks")
    def api_deadlocks():
        """Robots currently in an enforced deadlock-backoff cooldown,
        read directly off each robot's own controller state - again,
        purely observational."""
        result = {}
        for rid, robot in simulator.robots.items():
            controller = robot.controller
            backoff_until = getattr(controller, "_deadlock_backoff_until", 0.0)
            if backoff_until > simulator.sim_time:
                result[rid] = {
                    "backoff_until": round(backoff_until, 2),
                    "consecutive_backoff_count": getattr(controller, "_consecutive_backoff_count", 0),
                }
        return jsonify(result)

    @app.route("/api/metrics")
    def api_metrics():
        """Fleet-wide derived metrics, computed fresh from current robot
        states every call - not a separate tracked/mutable store."""
        summary = simulator.summary()
        robots = summary["robots"]
        total_messages = 0
        for robot in simulator.robots.values():
            network = getattr(robot.controller, "network", None)
            if network is not None:
                total_messages += network.messages_sent
        return jsonify({
            "sim_time": summary["sim_time"],
            "tick_count": summary["tick_count"],
            "total_distance": round(sum(r["distance_travelled"] for r in robots.values()), 2),
            "total_completed_tasks": sum(r["completed_tasks"] for r in robots.values()),
            "total_collisions": sum(r["collision_count"] for r in robots.values()),
            "total_messages": total_messages,
            "average_battery": round(sum(r["battery"] for r in robots.values()) / max(len(robots), 1), 1),
        })

    return app
