# Algorithms

## A* path planning (`planning/astar.py`)

4-directional grid search over `(cell, timestep)` states (not just
`cell`), so it can avoid both static obstacles and cells already claimed
in a `ReservationTable`. Heuristic: Manhattan distance (admissible and
consistent for unit-cost 4-directional movement). A "wait in place"
action is included so the search can find a valid plan that waits one
step for a cell to clear, rather than being forced into a longer detour
or failing outright.

## Multi-agent planning (`planning/multi_agent_planner.py`)

Each robot re-runs the above A* against its OWN `ReservationTable`,
which it populates from: (a) its own current path, and (b)
`PATH_INTENT`-style data carried in peers' `ROBOT_STATE` broadcasts. This
is what makes planning "multi-agent aware" without a shared/centralized
reservation authority - see `docs/architecture.md`.

**Important correctness detail**: reservation timesteps are computed as
`round(now * speed)` (an absolute, globally-comparable clock), not each
robot's own private step counter. An earlier implementation used a
per-robot counter, which allowed two robots to each believe a cell was
free at "their" timestep 5 while actually being there at the same real
instant - see `robot_controller.py`'s `_global_step()` docstring for the
full explanation of this fixed bug.

## Collision detection (`planning/conflict_detector.py`)

Four conflict types, each a small geometric predicate over a robot's own
next planned cell vs. a peer's known position/path:
- `SAME_CELL`: both want the identical next cell.
- `HEAD_ON`: two robots are about to swap cells with each other.
- `FOLLOWING`: moving into a cell a peer currently occupies.
- `INTERSECTION`: both approaching the same flagged choke-point cell.

Plus a `SPATIAL` check (euclidean distance < `SAFE_DISTANCE` right now),
independent of any plan, as an unconditional safety net.

## Priority scoring (`coordination/priority.py`)

```
priority = W_urgency   * task_urgency
         + W_waiting   * min(1, waiting_time / WAIT_TIMEOUT)
         + W_battery   * (1 - battery/100)
         + W_proximity * (1 - min(distance, 10)/10)
```

All weights are in `config.py`. Ties are broken by robot ID
(lexicographically smaller wins) - **never** randomly, per the project
requirement for reproducible conflict resolution.

## Deadlock detection (`coordination/wait_for_graph.py`, `deadlock.py`)

Every robot builds a directed "wait-for" graph from its own knowledge:
`self -> whoever is blocking me` plus each peer's *self-reported*
`waiting_for` field (broadcast in `ROBOT_STATE`). `networkx.simple_cycles`
finds cycles. A cycle = a deadlock, unambiguously. The lowest-priority
robot in the cycle (deterministic tie-break by ID) backs off.

**Backoff mechanism** (`robots/robot_controller.py`): the chosen robot
releases its reservation and enters an enforced cooldown - it does NOT
immediately re-plan, because in a genuinely narrow/structural conflict,
an immediate re-plan toward the same destination just re-finds the
identical path and re-triggers the same conflict on the very next tick.
The cooldown duration escalates with consecutive backoffs
(`WAIT_TIMEOUT * min(count, 4)`) with a small per-robot multiplicative
and additive jitter (derived from a deterministic `zlib.crc32` hash of
the robot ID, specifically NOT Python's built-in `hash()`, which is
randomized per-process and would break run-to-run determinism) to
desynchronize a perfectly symmetric alternating deadlock.

## Task allocation (`tasks/auction.py`)

```
bid_cost = W_distance   * manhattan(robot, pickup)
         + W_time       * manhattan(robot, pickup) / speed
         + W_congestion * nearby_robot_count * 2
         + W_battery    * battery_penalty(0, 20, or 1000)
         + W_workload   * current_workload * 5
```

Every idle robot with acceptable battery bids on the lowest-ID currently
WAITING task by broadcasting a `TASK_BID` message. After
`TASK_BID_WINDOW` seconds, every robot that has received the same set of
bids independently computes the same winner (lowest cost, ties by robot
ID) via `select_winner()` - no auctioneer process exists.

## Re-routing (`planning/rerouting.py`)

Triggered when a robot's remaining path runs through a cell that has
since become statically or dynamically blocked
(`path_is_still_valid()` returns False). The robot strips the invalid
path, re-runs A* against the current warehouse + reservation state, and
either gets a new path (broadcasts `REROUTE_NOTICE`) or, if none exists,
reports `BLOCKED` status.

## Simplifications explicitly documented in code

- Intersection negotiation (`coordination/negotiation.py`) collapses the
  spec's multi-round `CONFLICT_REQUEST`/`CONFLICT_RESPONSE` handshake
  into a single deterministic computation, since every robot already
  broadcasts its full path every heartbeat and conflict resolution is a
  pure function of two priority scores - both sides compute the identical
  outcome without a literal request/response round trip. See that
  module's docstring for the full justification.
- Charging-station contention is handled by ordinary collision avoidance
  (only one robot can physically occupy the charger cell at a time)
  rather than an explicit reservation protocol for the station itself.
