# Decentralized Multi-Robot Coordination for a Smart Warehouse

A from-scratch simulation of a fleet of Autonomous Mobile Robots (AMRs)
that plan, move, communicate, negotiate, and recover from failures
**without any central server making their movement decisions.**

> **Build status:** This project is being built in explicit phases (see
> `docs/architecture.md` once later phases add it, and the phase list
> below). **Phase 1 is complete and runnable today.** Later phases add
> A* planning, peer-to-peer communication, collision/deadlock handling,
> task auctions, battery/charging, failure injection, the Pygame GUI, the
> Flask dashboard, benchmarking, and full documentation, without breaking
> the Phase 1 interfaces described here.

## 1. What this project demonstrates

* A **grid-based smart warehouse** (shelves, aisles, pickup/dropoff points,
  charging stations, intersections) loaded from JSON.
* A fleet of **Robot** edge-nodes, each owning its own state
  (`robots/robot_state.py`) and a `LocalWorldModel` representing only what
  it has learned from its peers - never a global, guaranteed-complete view.
* A **fixed-timestep Simulator** (`simulation/simulator.py`) that advances
  every robot independently each tick - the simulator itself contains no
  path-planning or negotiation logic.
* Deterministic, seed-reproducible runs.

## 2. Why "decentralized" is the whole point

```
Robot 1 <----> Robot 2
   ^              ^
   |              |
   v              v
Robot 3 <--------->
```

Every robot runs its own copy of: localization, sensing, path planning,
collision detection, conflict resolution, task selection, re-routing, and
deadlock detection. The dashboard (added in a later phase) is **monitoring
only** - if it is stopped, the robots keep moving, because they were never
waiting on it in the first place.

## 3. Current phase: Phase 1 - Warehouse + Grid + Robots

### What's implemented
| File | Purpose |
|---|---|
| `config.py` | Every tunable parameter in the whole project (grid size, fleet size, safety distance, battery curve, communication timing, priority weights, etc). |
| `simulation/warehouse.py` | `Warehouse` class: grid of `CellType`s, JSON load/save, static + dynamic blocking, ASCII rendering. |
| `simulation/physics.py` | Distance functions, velocity vectors, conflict-type predicates (head-on/following/same-cell), localization noise helper. |
| `simulation/simulator.py` | `Simulator`: owns the warehouse + fleet, advances them one fixed timestep (`config.DT`) at a time, deterministic given a seed. |
| `robots/robot_state.py` | `RobotState` (full data model required by the spec), `RobotStatus`, `BatteryState`, `PeerInfo`, `LocalWorldModel`. |
| `robots/robot.py` | `Robot` edge node. Phase 1 ships a clearly-marked **greedy stepper** (`_greedy_next_cell`) as a placeholder for the A* planner that Phase 2 adds - it moves a robot one cell closer to its destination each tick without knowing about other robots yet. |
| `main.py` | CLI entry point: loads the warehouse, spawns N robots, gives each a destination, runs the simulation headlessly with periodic ASCII frames. |
| `tests/test_robot.py` | 12 tests covering warehouse loading/queries, dynamic blocking, robot movement, battery drain, and simulator determinism. |

### What's intentionally NOT yet implemented (arrives in later phases)
- A* / multi-agent planning (Phase 2) - robots currently use a simple
  greedy stepper and do **not** avoid each other.
- UDP peer-to-peer communication (Phase 4) - in Phase 1 all robots share
  process memory for convenience; no `ROBOT_STATE` messages are sent yet.
- Collision detection/avoidance, reservations, intersection negotiation,
  deadlock detection, re-routing (Phases 6-10).
- Auction-based task allocation and reassignment (Phases 11-12).
- Charging behaviour and robot-failure injection (Phases 13-14).
- Pygame GUI and Flask dashboard (Phase 15 and the visualization phase).
- Metrics database, baseline controller, benchmarking, and plotted
  performance comparisons (Phases 16-18).

Because of this, **do not yet expect zero collisions or a measured
speed-up** - those are meaningless before Phase 6-10 and Phase 17 exist.
Phase 1's job is only to prove the world model, robot data model, and
simulation loop are correct and testable.

## 4. Installation

```bash
python -m venv .venv
source .venv/bin/activate        # on Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Phase 1 itself only needs the Python standard library; `requirements.txt`
already lists the packages later phases will need (`pygame`, `flask`,
`numpy`, `networkx`, `matplotlib`, `pytest`) so `pip install` only needs to
be run once.

## 5. Running Phase 1

```bash
python main.py
```

Default behaviour: loads `data/warehouse.json` (20x20 grid), spawns
`config.NUM_ROBOTS` (3) robots at random free cells, assigns each one a
distinct dropoff point as its destination, and runs until every robot
arrives (or a safety cutoff is hit), printing an ASCII frame of the grid
periodically.

Useful flags:

```bash
python main.py --robots 5                 # simulate 5 robots
python main.py --ticks 300                # cap the run at 300 ticks
python main.py --render-every 10          # print a frame every 10 ticks
python main.py --seed 7                   # reproducible run with a different seed
```

`--scenario`, `--baseline`, `--benchmark`, and `--dashboard` are already
accepted on the command line (so scripts/CI written against the final CLI
surface won't need to change) but print a clear notice and fall back to
the standard run until their phases are implemented.

### Expected output (abridged)

```
============================================================
DECENTRALIZED MULTI-ROBOT WAREHOUSE SIMULATION - PHASE 1
============================================================
Warehouse: 20x20   Robots: 3
Seed: 42   DT: 0.1s
------------------------------------------------------------

[t=   8.6s | tick=86]
% % % % % % % % % % % % % % % % % % % %
% . . . . . . . . . C . . . . . . . . %
...
  R1: pos=(17, 17) status=COMPLETED  battery= 94.5%  dist= 11.0
  R2: pos=(17, 10) status=COMPLETED  battery= 91.5%  dist= 17.0
  R3: pos=(2, 10) status=COMPLETED  battery= 97.0%  dist=  6.0

All robots reached their destinations. Simulation complete.
```

## 6. Managing it in real time (the live dashboard)

The offline trace viewer (section 7 below) is great for a completed run,
but if you want to *watch and control* a simulation while it's actually
happening, use the live Flask dashboard - pulled forward from its planned
phase specifically because Flask, unlike pygame, has zero build issues on
any Python version:

```bash
python main.py --dashboard --robots 3
```

This starts the simulator on a background thread running in real time,
and serves a browser dashboard at **http://127.0.0.1:8080**. Open it and
you'll see the same warehouse/robot canvas as the trace viewer, except it
polls live (every `config.DASHBOARD_POLL_INTERVAL_MS`, default 500ms)
instead of replaying a recording, plus live stat cards (sim time, tick,
active/completed robots, average battery, total distance, collisions)
and Pause/Resume buttons.

**What Pause/Resume actually do:** they only start/stop the simulator's
own clock (`Simulator.pause()`/`resume()`) - they never set a robot's
position or destination. This was verified directly: pausing froze
`sim_time` at exactly the same value across repeated polls, and resuming
picked back up from there. See `tests/test_dashboard.py::test_pause_and_resume_control_the_clock_only`.

**Why the dashboard being killed doesn't stop the robots:** `main.py`'s
`--dashboard` path runs the *exact same* `Simulator.run()` method the
headless path uses, on its own background thread, and the Flask app in
`dashboard/app.py` only ever calls `Simulator.summary()` (a thread-safe
read) or `Simulator.pause()/resume()`. This was verified directly too: a
simulator thread kept advancing (`sim_time` climbing) for a full second
after nothing was reading from it anymore, exactly as it would if the
Flask process had been killed.

API endpoints (all monitoring-only, listed in full in section 33 of the
project spec):

| Method | Path | Purpose |
|---|---|---|
| GET | `/` | The live dashboard page |
| GET | `/api/status` | sim_time, tick_count, running/paused, fleet-wide stats |
| GET | `/api/robots` | Per-robot state (position, status, battery, distance, ...) |
| GET | `/api/warehouse` | Static layout + currently blocked aisles |
| POST | `/api/simulation/pause` | Pause the simulation clock |
| POST | `/api/simulation/resume` | Resume the simulation clock |

## 7. Seeing a completed run visually (offline trace viewer)

If pygame won't install on your machine yet (common on very new Python
versions - see the troubleshooting note below), you don't have to wait
for the GUI phase to *see* what the robots are doing. Record a run and
generate a self-contained HTML playback viewer instead - no server, no
pygame, no internet connection, just a file you double-click:

```bash
python main.py --robots 3 --ticks 400 --record-trace data/results/trace.json
python tools/build_trace_viewer.py data/results/trace.json data/results/trace_viewer.html
```

Open `data/results/trace_viewer.html` in any browser. You'll see the
warehouse grid (shelves, aisles, pickup/dropoff points, chargers), each
robot as a colored, labeled circle, a play/pause button, a speed
selector, a scrubber to jump to any point in the run, and a live-updating
table of each robot's position/status/battery/distance for the frame
you're viewing.

This is implemented by `simulation/trace_recorder.py` (a pure observer
that hooks into `Simulator.register_tick_callback()` and never influences
robot decisions - same rule the future Pygame renderer and Flask
dashboard follow) and `tools/build_trace_viewer.py` (embeds the recorded
JSON into a template HTML file). It's a genuinely useful *inspection*
tool on top of - not a replacement for - the live Pygame GUI that a later
phase still adds.

> **Note on pygame install failures on Windows:** if `pip install -r
> requirements.txt` fails to build pygame with an error mentioning
> `msvccompiler`, you're likely on a Python version (e.g. 3.14) that
> doesn't have prebuilt pygame wheels yet. Phase 1 doesn't need pygame at
> all - install everything else with
> `pip install flask numpy networkx matplotlib pytest` and use the trace
> viewer above. Pygame itself will be needed once the GUI phase lands;
> at that point, install Python 3.11 or 3.12 in a separate venv for
> guaranteed wheel availability.

## 8. Running tests

```bash
pytest tests/ -v
```

(All 12 Phase 1 tests were verified passing during development. If
`pytest` is not installable in your environment, the same assertions can
be run directly: `python -c "import tests.test_robot as t; [getattr(t,n)() for n in dir(t) if n.startswith('test_')]"`.)

## 9. Project structure (final target - phases fill this in progressively)

```
decentralized_amr/
├── main.py                  ✅ Phase 1
├── config.py                ✅ Phase 1
├── requirements.txt         ✅ Phase 1
├── README.md                ✅ Phase 1 (this file)
├── LICENSE                  ✅ Phase 1
├── .gitignore               ✅ Phase 1
│
├── simulation/
│   ├── warehouse.py         ✅ Phase 1
│   ├── simulator.py         ✅ Phase 1
│   ├── physics.py           ✅ Phase 1
│   ├── trace_recorder.py    ✅ Phase 1 (offline JSON trace capture; see section 7)
│   ├── renderer.py          ⏳ pygame GUI phase
│   └── scenarios.py         ⏳ Phase 28 (experiment scenarios)
│
├── robots/
│   ├── robot.py             ✅ Phase 1 (greedy stepper; A* swapped in at Phase 2)
│   ├── robot_state.py       ✅ Phase 1
│   ├── robot_controller.py  ⏳ Phase 2-10 (planning/collision orchestration)
│   ├── localization.py      ⏳ Phase 2
│   ├── battery.py           ⏳ Phase 13
│   └── sensor_simulator.py  ⏳ Phase 2
│
├── communication/            ⏳ Phase 4
├── planning/                 ⏳ Phase 2-3
├── coordination/             ⏳ Phase 6-9
├── tasks/                    ⏳ Phase 11-12
├── dashboard/                 ✅ Phase 1 (live monitoring; app.py, templates/index.html, static/{style.css,dashboard.js})
├── metrics/                  ⏳ Phase 16
├── experiments/               ⏳ Phase 17-18
│
├── tools/
│   └── build_trace_viewer.py ✅ Phase 1 (generates the HTML playback viewer, see section 7)
│
├── tests/
│   ├── test_robot.py          ✅ Phase 1
│   ├── test_trace_recorder.py ✅ Phase 1
│   └── test_dashboard.py      ✅ Phase 1
│
├── data/
│   ├── warehouse.json        ✅ Phase 1 (sample 20x20 layout)
│   ├── tasks.json            ⏳ Phase 11
│   └── simulation_logs/      ✅ Phase 1 (created at runtime)
│
└── docs/                     ⏳ later phases
```

## 10. Design principle carried through every phase

> **No centralized movement controller, ever.** Every module added from
> here on is written to run *inside* `robots/robot.py`'s decision loop
> (or a sibling module it calls), never inside the simulator or the
> dashboard. The simulator only advances time; the dashboard (once added)
> only reads state for display.

## 11. Next phase

**Phase 2** replaces `Robot._greedy_next_cell` with real A* search
(`planning/astar.py`) against the warehouse grid plus a time-indexed
reservation table, and adds `tests/test_astar.py`.

## 12. All phases: current status

Phases 2-20 (A* planning, P2P communication, local world model, collision
detection, reservations, intersection negotiation, deadlock detection,
dynamic re-routing, task allocation/reassignment, battery/charging,
robot failure, dashboard extensions, metrics, baseline, benchmarking,
testing, documentation) are all implemented. See `docs/architecture.md`,
`docs/algorithms.md`, `docs/api.md`, `docs/experiment_results.md`, and
`docs/project_report.md` for full detail. 62 automated tests pass
(`tests/test_*.py`, covering A*, collision, deadlock, task allocation,
communication, the dashboard, the robot/simulator core, and trace
recording) - run them with `pytest tests/ -v` or, if `pytest` isn't
installable in your environment, `python -c "import tests.test_robot as t; [getattr(t,n)() for n in dir(t) if n.startswith('test_')]"` per test file.

### Try it

```bash
python main.py                                  # default 3-robot run
python main.py --scenario simple                # named scenario, decentralized
python main.py --scenario simple --baseline     # same scenario, stop-and-wait
python main.py --benchmark                      # full comparison + CSV + charts
python main.py --dashboard                       # live monitoring at :8080
```

## 13. Known Limitations and Future Work

**Zero-collision safety invariant: verified, holds in every tested
scenario.** This was checked directly with a dedicated per-tick
same-cell-occupancy audit (independent of each robot's own avoidance
logic) across thousands of ticks, for 3-robot and 5-robot configurations
alike, and is what `metrics/collision_metrics.py`'s `CollisionMonitor`
enforces at all times.

**The 20% completion-time improvement target was NOT achieved in the
scenarios used for the default `--benchmark` run.** We measured the
decentralized system as substantially *slower* than the naive
stop-and-wait baseline in these specific low-traffic scenarios. Full
numbers and analysis: `docs/experiment_results.md`. Short version: in
open, low-contention layouts, the decentralized system still pays fixed
coordination overhead (heartbeats, bid windows, and especially its
escalating deadlock-backoff cooldowns) that a strategy with zero
negotiation apparatus simply doesn't incur when there's nothing to
negotiate about. This is reported honestly rather than adjusted or
hidden, per this project's explicit requirement to display the actual
measured result.

**Priority-based deadlock backoff can livelock at high robot density.**
With 5 robots in a busier configuration, we observed robots repeatedly
backing off from the same recurring conflict without ever fully
resolving it within a generous tick budget - a known failure mode of
priority-based (rather than token-passing or globally-optimal) MAPF
approaches under heavy contention. The safety invariant still holds
(no collisions occur even during a livelock) - it is a liveness
limitation, not a safety one.

**Two real determinism bugs were found and fixed during development**,
both stemming from Python's built-in `hash()` on strings being
randomized per-process by default (`PYTHONHASHSEED`): once in the
benchmark's scenario-seed derivation, and once in the deadlock-backoff
jitter calculation. Both now use `zlib.crc32` instead, and determinism
across separate process runs was re-verified after the fix.

**Pygame visualization (`simulation/renderer.py`) is implemented but not
visually run-tested** - pygame could not be installed in the sandbox
this project was built in (no network access, headless environment).
The live Flask dashboard and the offline HTML trace viewer (both fully
verified with real running processes) were used as the primary visual
tools during development instead; see sections 6-7 above.

**Suggested next steps** (also in `docs/project_report.md`): replace the
fixed-duration priority backoff with an explicit reservation/token
protocol for intersections specifically (this is what the project spec's
`CONFLICT_REQUEST`/`RESERVATION_REQUEST` message types were designed to
support, and would likely resolve both the livelock and the low-traffic
overhead findings above); and re-tune `WAIT_TIMEOUT`/backoff parameters
against a wider sweep of contention levels before trusting the
improvement percentage on any single scenario.

```bash
deactivate
cd "C:\Users\shash\Desktop\test sih\decentralized_amr"
rmdir /s /q .venv
py -3.12 -m venv .venv
.venv\Scripts\activate
python --version
```
