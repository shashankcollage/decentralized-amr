# Project Report

## ABSTRACT
This project implements a decentralized multi-robot coordination system
for a simulated smart warehouse. A fleet of Autonomous Mobile Robots
(AMRs) independently plan paths, communicate over real UDP sockets,
detect and resolve collisions and deadlocks, and bid on tasks through a
distributed auction - all without any central server making movement
decisions. A Flask dashboard provides read-only monitoring that the
robots never depend on.

## INTRODUCTION
Warehouse automation increasingly relies on fleets of AMRs operating in
shared space. Centralized fleet managers are a single point of failure
and a scalability bottleneck. This project demonstrates that a fleet can
coordinate purely through peer-to-peer message passing and locally-run
algorithms.

## PROBLEM STATEMENT
Given a grid warehouse with shelves, aisles, pickup/dropoff points, and
charging stations, coordinate 3+ AMRs to complete pick-and-drop tasks
with zero collisions, without any robot's movement decision depending on
a central authority, and while continuing to function if the monitoring
dashboard is unavailable.

## EXISTING SYSTEM
Traditional AMR fleets use a central Fleet Management System (FMS) that
computes all paths and task assignments, pushing commands to each robot.
This gives a global optimum but creates a single point of failure and
requires low-latency, high-availability connectivity to every robot at
all times.

## LIMITATIONS OF EXISTING SYSTEM
- Single point of failure: FMS outage halts the whole fleet.
- Scales poorly with fleet size (centralized planning is often
  super-linear in robot count).
- Requires reliable, low-latency connectivity to a central point from
  every robot, which is not always achievable on a large warehouse floor.

## PROPOSED SYSTEM
Each robot runs an identical, independent decision-making stack
(`robots/robot_controller.py`): localization, sensing, A* planning,
conflict detection/resolution, deadlock detection, task auction
participation, and battery/charging management. Robots exchange
`ROBOT_STATE`, `TASK_BID`, `TASK_ASSIGNMENT`, `DEADLOCK_ALERT`, and other
message types (`communication/message.py`) over real UDP sockets, one
per robot. A monitoring dashboard reads (never writes) fleet state.

## OBJECTIVES
1. Zero unintentional same-cell collisions.
2. Decentralized task allocation via auction, with automatic
   reassignment on robot failure/timeout.
3. Local deadlock detection and resolution via wait-for-graph cycle
   detection.
4. Dynamic re-routing around newly-blocked aisles.
5. Continued operation with the dashboard offline.
6. Measured (not assumed) performance comparison against a naive
   stop-and-wait baseline.

## SYSTEM ARCHITECTURE
See `docs/architecture.md` for the full diagram and layer breakdown.

## MODULE DESCRIPTION
See the module table in `docs/architecture.md` and the per-file
docstrings throughout the codebase, which are written to be read as
documentation in their own right.

## DECENTRALIZED COMMUNICATION
Implemented with Python's `socket` module (`SOCK_DGRAM`/UDP) - one bound
socket per robot on `127.0.0.1:500N`. Messages are JSON with an
HMAC-SHA256 signature (`communication/protocol.py`). Validation checks
message type, sender ID (must be a known peer), timestamp freshness, and
signature - malformed, spoofed, unknown-sender, or stale messages are
rejected and counted, never silently accepted. Verified with real socket
I/O in `tests/test_communication.py` (7/7 passing).

## EDGE COMPUTING MODEL
All decision logic - planning, collision avoidance, negotiation, task
bidding, deadlock detection - runs inside `RobotController`, which is
written without any dependency on pygame or Flask, so it is structurally
ready to run unmodified on a Raspberry Pi/Jetson Nano edge device (see
`README.md` section on hardware deployment prep).

## MULTI-AGENT PATH PLANNING
Time-expanded A* (`planning/astar.py`) searching over `(cell, timestep)`
states against a per-robot `ReservationTable`
(`planning/reservation_table.py`), with a corrected globally-comparable
timestep clock (`RobotController._global_step`) - an earlier
implementation bug used a per-robot-private step counter, which we
identified and fixed during development (documented in the code and in
`docs/algorithms.md`).

## COLLISION AVOIDANCE
Two layers: (1) proactive - conflict classification
(`planning/conflict_detector.py`) plus deterministic priority-based
resolution (`coordination/conflict_resolution.py`); (2) a final,
unconditional safety check immediately before any cell-transition commit
that refuses to step onto a cell any active peer is currently confirmed
to occupy, regardless of what the negotiation concluded - added
specifically after we found and fixed a real collision this priority
layer alone did not catch. Verified with zero same-cell occupations
across thousands of ticks in the shipped default scenarios.

## DEADLOCK DETECTION
Every robot builds its own wait-for graph from self-reported peer
`waiting_for` fields and detects cycles with `networkx.simple_cycles`.
The lowest-priority robot in a detected cycle backs off with an
escalating, per-robot-jittered cooldown to avoid immediate re-collision
with the identical plan. See `docs/algorithms.md` for the full mechanism
and its documented failure mode at higher robot density.

## TASK ALLOCATION
Decentralized auction: every idle, battery-eligible robot computes and
broadcasts a bid for the lowest-ID currently-WAITING task; after a fixed
bid window, every robot that received the same broadcasts independently
computes the identical winner. No auctioneer process exists.

## DYNAMIC RE-ROUTING
`planning/rerouting.py` detects when a robot's remaining path runs
through a newly-blocked cell and re-plans; if no path exists, the robot
reports `BLOCKED` rather than silently failing.

## BATTERY MANAGEMENT
Linear drain proportional to distance travelled plus a small idle drain;
three states (NORMAL/LOW/CRITICAL) gate new-task acceptance and trigger
diversion to the nearest charging station.

## FAULT TOLERANCE
A robot can be forced into `FAILED` status
(`Robot.simulate_failure()`); peers detect the resulting silence via
heartbeat timeout (`NetworkManager`'s `PeerLivenessTracker`) and return
its in-flight task to the WAITING pool for re-auction.

## DASHBOARD
Flask app (`dashboard/app.py`) exposing 8 read-only/control endpoints
(see `docs/api.md`). Verified directly that killing it does not affect
the simulator thread driving the robots (see `README.md` section 6).

## PERFORMANCE EVALUATION
`experiments/benchmark.py` runs both the decentralized system and the
`StopAndWaitController` baseline across the same scenarios/seeds,
collecting real `RunMetrics` and computing
`improvement_percentage = (baseline - decentralized) / baseline * 100`
from actual measured values - never hard-coded.

## EXPERIMENTAL SETUP
20x20 grid warehouse (`data/warehouse.json`), 3-5 robots, `DT=0.1s`
fixed timestep, deterministic seeding throughout (including a fix for
Python's non-deterministic string `hash()`, which we found was breaking
reproducibility in two separate places during development).

## RESULTS
See `docs/experiment_results.md` for the full, honest numbers. Summary:
zero collisions in every scenario tested; the 20% improvement target was
**not** achieved in the two low-traffic scenarios benchmarked by
default - the decentralized system measured substantially slower there,
which we report plainly along with our analysis of why (coordination
overhead dominating in low-contention conditions) rather than omit or
alter the result.

## LIMITATIONS
- Priority-based deadlock backoff can livelock under high robot density
  (observed and documented at 5 robots in a congested layout).
- Current parameter tuning (`WAIT_TIMEOUT`, backoff escalation) is
  better suited to high-contention scenarios than the low-traffic ones
  used in the default benchmark, leading to a measured slowdown rather
  than the target speedup in those specific cases.
- Pygame live visualization (`simulation/renderer.py`) is implemented
  but not runtime-verified in this environment (no display/pygame
  available in the development sandbox); the offline HTML trace viewer
  and live Flask dashboard were used as verified alternatives.

## FUTURE SCOPE
- Replace fixed-duration priority backoff with an explicit
  token-passing or timed-reservation protocol for intersections, which
  the literature shows resolves the livelock failure mode more reliably
  at higher density.
- Auto-tune `WAIT_TIMEOUT`/backoff parameters based on measured local
  traffic density rather than a single global constant.
- Real hardware deployment on Raspberry Pi/Jetson Nano, reusing
  `RobotController`/`NetworkManager` unmodified per the architecture's
  edge-computing design goal.

## CONCLUSION
The system demonstrates a genuinely decentralized architecture - real
UDP communication, independent per-robot decision-making, a monitoring
dashboard with zero movement authority - and meets its hardest safety
requirement (zero same-cell collisions) across all tested scenarios. The
performance-improvement target was honestly measured and, in the
specific low-traffic default benchmark, not met; this report documents
that result and its likely cause rather than suppressing it.

## REFERENCES
- Silver, D. "Cooperative Pathfinding." AIIDE 2005 (windowed hierarchical
  cooperative A*, the basis for this project's time-expanded A*).
- Sharon et al. "Conflict-Based Search for Optimal Multi-Agent
  Pathfinding." AIJ 2015 (context for reservation-table-based planning).
- Project specification provided for this assignment (all section
  references throughout the codebase's docstrings refer to it).
