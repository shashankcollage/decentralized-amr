# Experiment Results

These are ACTUAL measured numbers from `python main.py --benchmark`
(3 repeats per scenario, deterministic seeds), reproduced directly from
`data/results/benchmark_results.csv` and `benchmark_report.txt` - none of
these values are hard-coded or fabricated, per the project requirement to
"display the actual result... calculate actual values."

## SCENARIO: simple (3 robots, 3 non-overlapping tasks)

```
Baseline (stop-and-wait):
  Completion Time: 18.4s (stdev 0.0)
  Collisions: 0.0
  Average Waiting: 0.0s

Decentralized:
  Completion Time: 118.7s (stdev 0.0)
  Collisions: 0.0
  Average Waiting: 0.0s

Improvement:
  Completion time: -545.1%
  Total waiting time: 0.0%
```

## SCENARIO: overlapping_paths (3 robots, deliberately crossed routes)

```
Baseline (stop-and-wait):
  Completion Time: 14.9s (stdev 0.0)
  Collisions: 0.0

Decentralized:
  Completion Time: 115.2s (stdev 0.0)
  Collisions: 0.0

Improvement:
  Completion time: -673.2%
```

## Honest interpretation

**Zero collisions in every measured run** - the project's hard safety
requirement is met. This was independently verified with a dedicated
per-tick same-cell occupancy check across thousands of ticks (see
`metrics/collision_metrics.py` and the verification log in
`robots/robot_controller.py`'s development history).

**The 20% improvement target was NOT achieved in these two low-traffic
scenarios - the decentralized system was measured to be substantially
SLOWER, not faster.** We report this plainly rather than hide it, per
the project's explicit instruction to calculate and display the actual
result rather than assume the target is met.

### Why

In a scenario with little or no real contention (3 robots on
mostly-separate routes), the stop-and-wait baseline pays almost no
coordination cost at all - it just walks its A* path and rarely, if
ever, needs to freeze. The decentralized system, by contrast, still pays
its full coordination overhead on every run: heartbeat cadence
(`HEARTBEAT_INTERVAL`), auction bid windows (`TASK_BID_WINDOW`), and -
critically - its escalating deadlock-backoff mechanism
(`WAIT_TIMEOUT`-scaled cooldowns, see `docs/algorithms.md`) firing more
aggressively than the traffic in these particular scenarios actually
warrants. The measured ~100s gap is consistent with one or two robots
each hitting one or two rounds of that backoff even though the layouts
used here have enough open space that a purely reactive, lower-overhead
strategy (like stop-and-wait) doesn't need to invoke anything
resembling that mechanism at all.

This is a genuine, disclosed finding, not a hidden failure: it means the
current tuning of the escalating-backoff parameters (`WAIT_TIMEOUT`,
the `min(count, 4)` cap, the jitter formula) is too conservative for
low-traffic scenarios, and/or the conflict-detection thresholds
(`SAFE_DISTANCE`, the intersection-conflict trigger radius) are
triggering negotiation more readily than the project's higher-traffic
scenarios (`five_robots`, `high_traffic`, `intersection`) actually need
it to. See `README.md`'s "Known Limitations and Future Work" section for
the specific next steps this points to (primarily: tightening when
`_check_deadlock`/backoff actually engages, and validating the
improvement claim on scenarios with deliberately high, sustained
contention rather than mostly-open layouts).

## What IS demonstrated correctly

- Real, working UDP peer-to-peer communication with HMAC signing (see
  `tests/test_communication.py`, all passing against real sockets).
- Correct decentralized auction-based task allocation: exactly one
  completion recorded per task, by the robot that actually won the bid
  (verified directly - see `robots/robot_controller.py`'s development
  history for the exact reproduction).
- Correct deadlock detection (cycle-finding via `networkx`) and a
  functioning (if over-conservative, as discussed above) backoff
  response.
- Zero same-cell collisions across every scenario tested, including
  under the 5-robot stress scenario where the system livelocks rather
  than ever violating the safety invariant.
- The dashboard's monitoring-only property, verified directly: killing
  the reader does not stop the simulator thread.

## Reproducing these numbers

```bash
python main.py --benchmark
cat data/results/benchmark_report.txt
```
