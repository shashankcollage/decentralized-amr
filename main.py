"""
main.py
=======
Command-line entry point for the Decentralized Multi-Robot Coordination
project.

PHASE 1 SCOPE
-------------
This first version wires together the warehouse model, the robot edge
nodes, and the fixed-timestep simulator, and runs a headless simulation
with periodic ASCII rendering to the terminal so the whole pipeline can be
verified end-to-end before communication/planning/coordination are layered
in.

Supported flags in this phase:
    python main.py                     -> default 3-robot run
    python main.py --robots 5          -> spawn N robots
    python main.py --ticks 200         -> cap simulation length
    python main.py --render-every 10   -> ASCII frame every N ticks
    python main.py --record-trace out.json   -> record a replayable trace
    python main.py --dashboard         -> run live with the Flask monitoring
                                           dashboard at http://127.0.0.1:8080

--dashboard runs the simulator on a background thread in real time and
serves dashboard/app.py in the main thread. The dashboard is READ-ONLY
monitoring: it never decides where a robot goes (see dashboard/app.py's
module docstring). If you Ctrl+C the process the whole thing exits
together in this CLI, but architecturally the simulator thread has no
dependency on the Flask thread staying alive - the two are only combined
here for a convenient single-command demo.

Flags described in the full spec (--scenario, --baseline, --benchmark)
are accepted but will raise a clear "not yet implemented in this phase"
message until their corresponding phases are added, so the CLI surface is
stable across phases.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time

import config
from simulation.warehouse import Warehouse
from simulation.simulator import Simulator, SimulatorConfig
from robots.robot import Robot

logger = logging.getLogger("main")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Decentralized Multi-Robot Coordination Smart Warehouse Simulation"
    )
    parser.add_argument("--robots", type=int, default=config.NUM_ROBOTS,
                         help="Number of AMRs to simulate.")
    parser.add_argument("--warehouse", type=str, default="data/warehouse.json",
                         help="Path to warehouse layout JSON.")
    parser.add_argument("--ticks", type=int, default=None,
                         help="Optional hard cap on number of simulation ticks (headless mode).")
    parser.add_argument("--render-every", type=int, default=20,
                         help="Print an ASCII frame every N ticks (headless mode).")
    parser.add_argument("--seed", type=int, default=config.RANDOM_SEED,
                         help="Random seed for deterministic reproducibility.")
    parser.add_argument("--scenario", type=str, default=None,
                         help="Named scenario to run (added in a later phase).")
    parser.add_argument("--baseline", action="store_true",
                         help="Run the stop-and-wait baseline controller (added in a later phase).")
    parser.add_argument("--benchmark", action="store_true",
                         help="Run the full benchmark suite (added in a later phase).")
    parser.add_argument("--dashboard", action="store_true",
                         help="Run the simulation in real time with the live Flask monitoring "
                              f"dashboard at http://{config.DASHBOARD_HOST}:{config.DASHBOARD_PORT}")
    parser.add_argument("--no-gui", action="store_true", default=True,
                         help="Run headless with ASCII rendering (default). Overridden by --gui.")
    parser.add_argument("--gui", action="store_true",
                         help="Launch the live Pygame 2D visualization window instead of headless/ASCII mode. "
                              "Requires `pip install pygame`. Controls: SPACE=pause/resume, R=reset, +/-=speed.")
    parser.add_argument("--record-trace", type=str, default=None,
                         help="Path to write a JSON trace of the run (e.g. data/results/trace.json), "
                              "for offline visual playback in a browser via tools/build_trace_viewer.py.")
    parser.add_argument("--trace-every", type=int, default=2,
                         help="Record one trace frame every N ticks (default: 2).")
    return parser


def configure_logging() -> None:
    os.makedirs(config.LOG_DIR, exist_ok=True)
    log_path = os.path.join(config.LOG_DIR, "simulation.log")
    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL, logging.INFO),
        format="%(asctime)s %(name)-12s %(levelname)-8s %(message)s",
        handlers=[
            logging.FileHandler(log_path),
            logging.StreamHandler(sys.stdout),
        ],
    )


def not_yet_implemented(feature: str) -> None:
    print(f"\n[!] '--{feature}' is defined in the CLI surface but its implementation "
          f"lands in a later build phase (see project_report.md phase plan).\n"
          f"    Continuing with the standard Phase-1 headless run instead.\n")


def build_simulation(args) -> Simulator:
    """Shared setup used by both the headless run and the dashboard run."""
    logger.info("Loading warehouse from %s", args.warehouse)
    warehouse = Warehouse.from_json(args.warehouse)

    sim_config = SimulatorConfig(seed=args.seed)
    sim = Simulator(warehouse, sim_config)
    sim.spawn_robots(args.robots)

    # Phase-1 task assignment placeholder: send every robot to a distinct
    # dropoff point so there is visible, purposeful motion to observe.
    # Real auction-based task allocation replaces this in Phase 11.
    dropoffs = warehouse.dropoff_points or [(warehouse.width - 2, warehouse.height - 2)]
    for i, robot in enumerate(sim.robots.values()):
        dest = dropoffs[i % len(dropoffs)]
        robot.set_destination(dest)
        logger.info("%s assigned destination %s", robot.robot_id, dest)

    return sim


def run_headless(args, sim: Simulator) -> None:
    recorder = None
    if args.record_trace:
        from simulation.trace_recorder import TraceRecorder
        recorder = TraceRecorder(sim, every_n_ticks=args.trace_every)
        sim.register_tick_callback(recorder.on_tick)
        logger.info("Recording trace to %s (every %d ticks)", args.record_trace, args.trace_every)

    print("=" * 60)
    print("DECENTRALIZED MULTI-ROBOT WAREHOUSE SIMULATION - PHASE 1")
    print("=" * 60)
    print(f"Warehouse: {sim.warehouse.width}x{sim.warehouse.height}   Robots: {len(sim.robots)}")
    print(f"Seed: {args.seed}   DT: {sim.cfg.dt}s")
    print("-" * 60)

    tick = 0
    while True:
        sim.step()
        tick += 1

        if tick % args.render_every == 0 or sim.all_robots_done():
            print(f"\n[t={sim.sim_time:6.1f}s | tick={sim.tick_count}]")
            print(sim.render_ascii())
            for robot in sim.robots.values():
                s = robot.state
                print(f"  {robot.robot_id}: pos={s.position} status={s.status.value:10s} "
                      f"battery={s.battery:5.1f}%  dist={s.distance_travelled:5.1f}")

        if sim.all_robots_done():
            print("\nAll robots reached their destinations. Simulation complete.")
            break
        if args.ticks is not None and tick >= args.ticks:
            print(f"\nReached tick cap ({args.ticks}). Stopping.")
            break
        if sim.sim_time >= sim.cfg.max_sim_time:
            print("\nReached MAX_SIMULATION_TIME safety cutoff. Stopping.")
            break

    print("\nFinal summary:")
    import json
    print(json.dumps(sim.summary(), indent=2))

    if recorder:
        recorder.save(args.record_trace)
        print(f"\nTrace written to {args.record_trace} "
              f"({len(recorder.frames)} frames). Build a viewer with:\n"
              f"  python tools/build_trace_viewer.py {args.record_trace} data/results/trace_viewer.html")


def run_with_dashboard(args, sim: Simulator) -> None:
    """Drive the simulator in real time on a background thread, and serve
    the read-only Flask monitoring dashboard on the main thread.

    This is the concrete demonstration of the project's core requirement:
    the dashboard is not in the robots' decision loop. The background
    thread below calls nothing but sim.run() - the exact same method the
    headless path uses - so stopping the Flask server (Ctrl+C in this demo
    process, or in a real deployment simply not running it) would not stop
    robot movement.
    """
    from dashboard.app import create_app

    def drive_simulation():
        sim.run(real_time=True, speed=config.SIMULATION_SPEED)

    sim_thread = threading.Thread(target=drive_simulation, daemon=True)
    sim_thread.start()

    app = create_app(sim)
    url = f"http://{config.DASHBOARD_HOST}:{config.DASHBOARD_PORT}"
    print("=" * 60)
    print("DECENTRALIZED MULTI-ROBOT WAREHOUSE SIMULATION - LIVE DASHBOARD")
    print("=" * 60)
    print(f"Simulation is running in real time on a background thread.")
    print(f"Dashboard (monitoring only, read-only): {url}")
    print("Press Ctrl+C to stop this demo process.")
    print("-" * 60)

    try:
        app.run(host=config.DASHBOARD_HOST, port=config.DASHBOARD_PORT, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\nDashboard stopped. (In a real deployment the robots' own "
              "processes would be entirely unaffected by this.)")


def run_scenario(args) -> int:
    """Run a named scenario (project spec section 28) using the full
    decentralized stack, or the stop-and-wait baseline if --baseline was
    also passed, with periodic ASCII rendering just like the default run.
    """
    from experiments.scenarios import get_scenario
    from tasks.task_manager import TaskManager
    from simulation.warehouse import Warehouse

    spec = get_scenario(args.scenario, seed=args.seed)
    print(f"Scenario: {spec.name} - {spec.description}")
    print(f"Robots: {spec.num_robots}   Tasks: {len(spec.tasks)}")

    wh = Warehouse.from_json(args.warehouse)
    tm = TaskManager()
    for t in spec.tasks:
        tm.add_task(t)
    ids = [f"R{i+1}" for i in range(spec.num_robots)]

    sim_config = SimulatorConfig(seed=args.seed)
    sim = Simulator(wh, sim_config)

    if args.baseline:
        print("Controller: STOP-AND-WAIT BASELINE")
        from experiments.baseline import StopAndWaitController
        # The baseline path uses the same Simulator/Robot shell but swaps
        # in StopAndWaitController for movement decisions and a direct
        # (non-UDP) world-model sync, matching experiments/benchmark.py's
        # run_baseline() approach - see that module for the full comparison
        # harness. For a single interactive run we keep it simple: assign
        # pickups directly, no auction.
        for rid, pos, task in zip(ids, spec.start_positions, spec.tasks):
            robot = Robot(rid, pos, wh)  # no networking, no task_manager
            robot.controller = StopAndWaitController(rid, wh)
            robot.state.destination = task.pickup_location
            robot.state.current_task = task.task_id
            sim.add_robot(robot)
    else:
        print("Controller: DECENTRALIZED (A*, negotiation, deadlock detection, auction)")
        for rid, pos in zip(ids, spec.start_positions):
            sim.add_robot(Robot(rid, pos, wh, all_robot_ids=ids, task_manager=tm, enable_networking=True))

    for event_tick, callback in spec.scripted_events:
        def make_cb(cb=callback, tick_target=event_tick):
            def wrapped(s):
                if s.tick_count == tick_target:
                    cb(s.warehouse)
                    print(f"\n[scripted event fired at tick {tick_target}]")
            return wrapped
        sim.register_tick_callback(make_cb())

    try:
        tick = 0
        while True:
            sim.step()
            tick += 1
            if tick % args.render_every == 0 or tm.all_completed():
                print(f"\n[t={sim.sim_time:6.1f}s | tick={sim.tick_count}]")
                print(sim.render_ascii())
                for robot in sim.robots.values():
                    s = robot.state
                    print(f"  {robot.robot_id}: pos={s.position} status={s.status.value:12s} "
                          f"battery={s.battery:5.1f}%  dist={s.distance_travelled:5.1f}")
            if tm.all_completed():
                print("\nAll tasks completed.")
                break
            if args.ticks is not None and tick >= args.ticks:
                print(f"\nReached tick cap ({args.ticks}). Stopping.")
                break
            if sim.sim_time >= sim_config.max_sim_time:
                print("\nReached MAX_SIMULATION_TIME safety cutoff. Stopping.")
                break
    finally:
        for r in sim.robots.values():
            if hasattr(r, "close"):
                r.close()

    print("\nTask summary:", {tid: t.status.value for tid, t in tm.tasks.items()})
    return 0


def run_benchmark_mode(args) -> int:
    from experiments.benchmark import run_default_benchmark
    scenarios = [args.scenario] if args.scenario else None
    report = run_default_benchmark(scenarios, repeats=3)
    print(report)
    print("Full results saved to data/results/benchmark_results.csv, "
          "benchmark_report.txt, and chart_*.png")
    return 0




def run_gui(args, sim: Simulator) -> int:
    """Launch the live Pygame 2D visualization (project spec section 30).

    Unlike --dashboard (which drives the simulator on a background thread
    and serves Flask separately), the GUI drives the simulator directly
    in its own render loop - simpler for a single-process desktop app,
    and there is no dashboard-independence property to demonstrate here
    since there's no separate monitoring process at all.
    """
    try:
        from simulation.renderer import Renderer
    except ImportError as e:
        print(f"\n[!] Could not import renderer module: {e}")
        return 1

    print("=" * 60)
    print("DECENTRALIZED MULTI-ROBOT WAREHOUSE SIMULATION - PYGAME GUI")
    print("=" * 60)
    print("Controls: SPACE=pause/resume  R=reset  +/-=speed  (close window to quit)")
    print("-" * 60)

    try:
        renderer = Renderer(sim)
    except ImportError as e:
        print(f"\n[!] Pygame is not installed: {e}")
        print("    Install it with: pip install pygame")
        return 1

    renderer.run()

    print("\nFinal summary:")
    import json
    print(json.dumps(sim.summary(), indent=2))
    return 0



def main(argv=None) -> int:
    configure_logging()
    args = build_arg_parser().parse_args(argv)

    if args.benchmark:
        return run_benchmark_mode(args)

    if args.baseline and not args.scenario:
        args.scenario = "simple"

    if args.scenario and not args.gui:
        return run_scenario(args)

    sim = build_simulation(args)

    if args.gui:
        return run_gui(args, sim)
    elif args.dashboard:
        run_with_dashboard(args, sim)
    else:
        run_headless(args, sim)

    return 0

if __name__ == "__main__":
    sys.exit(main())

