#!/usr/bin/env python3
"""
bench_astar.py — Hybrid A* planning-time benchmark (AFTER the spatial-index fix)
===============================================================================

PURPOSE
-------
Measure how long `tactical_brain/A_planner.calc_hybrid_a_star()` actually
takes now that `build_spatial_index()` / bucketed `check_collision()` are in
place, and how many path steps it returns.

This exists because of a specific, documented regression. Before bucketing,
`check_collision()` scanned the ENTIRE `obstacle_set` for every candidate
node, so search cost grew linearly with however much slam_toolbox had
mapped. That was fine early in a run (~200 occupied cells) and blew up to
**~300 SECONDS per search once the map passed ~1600 occupied cells** (see the
comment on `build_spatial_index` in A_planner.py). ~300 s is the "BEFORE"
number. This script produces the "AFTER" number, at the same obstacle counts,
so the two can be quoted side by side.

It answers the three questions the dev log asks:
  1. What is the average path-calculation time?
  2. How many steps does the returned path contain?
  3. Is the 100 ms requirement met?

On question 3, read the verdict block carefully. **100 ms is not the real
deadline.** The behaviour tree is driven by `self.tree_timer =
self.create_timer(0.5, self.sense_and_think)` in main_brain.py, and
`sense_and_think` is what runs the search — so the true budget is 500 ms
(one tick). The script prints BOTH verdicts and labels which is which; it
does not decide for you which one to quote, but only one of them is a
requirement.

EXACT RUN COMMAND
-----------------
Default sweep (4 obstacle densities x 3 start/goal pairs x 20 repeats):

    python3 measurement_scripts/bench_astar.py

Fast smoke test (this is what was used to verify the script runs):

    python3 measurement_scripts/bench_astar.py --repeats 2

Full run against the real arena size, CSV for the report:

    python3 measurement_scripts/bench_astar.py \
        --arena-size 5.0 \
        --densities 200 800 1600 3200 \
        --repeats 20 \
        --csv results/astar_bench.csv

Harsher map (filler scattered inside the arena too). Expect failures and be
patient — a search that finds no path runs to A_planner's max_iter of 100000
before returning None, which takes SECONDS, so `--repeats` costs far more
here than in the default mode:

    python3 measurement_scripts/bench_astar.py --fill-mode uniform --repeats 3

Reproduce ONLY the historical regression point (~1600 occupied cells):

    python3 measurement_scripts/bench_astar.py --densities 1600 --repeats 20

PREREQUISITES
-------------
  * Python 3.8+ and `numpy` (A_planner.py imports numpy at module scope;
    nothing else here needs it).
  * NO built ROS 2 workspace, NO ROS 2 install, NO robot, NO Gazebo. The
    script puts the `tactical_brain` package directory on `sys.path` and
    imports `A_planner` as a plain module — it only needs math/heapq/numpy/
    time. Both known locations of the package are tried:
        AutonomousWarfare/AutonomousWarfare/ros2_ws/src/tactical_brain/tactical_brain
        AutonomousWarfare/ros2_ws/src/tactical_brain/tactical_brain
    whichever resolves wins; `--planner-path` overrides both. If neither
    exists you get one clear error, not an ImportError traceback.
  * Run it ON THE PI if the number is going to be quoted as the robot's
    planning time. A laptop measures a laptop.
  * Nothing else should be competing for CPU (stop the ROS stack first) —
    this is a pure single-threaded CPU benchmark and it will happily report
    a co-scheduled ai_inference node's cost as planner latency.

WHAT IS ACTUALLY MEASURED (read before quoting the output)
----------------------------------------------------------
  * The timed region is exactly one `calc_hybrid_a_star()` call, with
    `time.perf_counter()`. That call internally includes the Dijkstra
    heuristic map (`calc_holonomic_heuristic_with_obstacle`) AND
    `build_spatial_index()` — both are rebuilt from scratch on every call in
    the real node too, so they belong inside the measurement.
  * Units: `start`/`goal` are (x, y, yaw) in METERS/RADIANS.
    `obstacle_set` / `teammates_aura_set` are sets of (x_ind, y_ind) INTEGER
    grid cells where index = round(meters / XY_RESOLUTION), XY_RESOLUTION =
    0.1. `danger_dict` maps such a cell to a unix timestamp.
  * Obstacle maps are synthetic and seeded (`--seed`), so a re-run reproduces
    the same maps cell-for-cell. They are NOT a recorded slam_toolbox map.
  * No obstacle is ever placed on a start or goal cell, nor within
    PLANNING_CLEARANCE (0.40 m) + `--clearance-margin` of one. Without that
    guard `check_collision` rejects the first expansion of every trial and
    the benchmark measures nothing but instant failure.
  * `--fill-mode surround` (DEFAULT) grows the obstacle count OUTSIDE the
    drivable arena, in the surrounding mapped region. The drivable layout
    (one wall with a gap, two pillars) is IDENTICAL at every density. That is
    deliberate: the regression was driven by `len(obstacle_set)`, not by
    path difficulty, so holding difficulty constant isolates the variable
    under test. slam_toolbox really does map well beyond the arena, so these
    cells are not fictional padding.
  * `--fill-mode uniform` scatters the filler over the arena too. This is
    the harsher, more realistic-looking case, but at high densities it makes
    the arena genuinely unplannable and trials start returning None. Failures
    are counted and reported separately; a failure is NOT a plan, fast or
    slow. Note a failing search is the SLOWEST outcome, not the fastest: it
    expands until `max_iter = 100000` before giving up, which is seconds.
  * The search is deterministic for a fixed map, so repeats re-measure the
    same work. Spread across repeats is scheduler/cache noise, which is
    exactly what p95 is there to show.
  * EXPECT A FLAT CURVE ACROSS DENSITIES. That flatness IS the result: it is
    what "no longer O(len(obstacle_set)) per node" looks like. Post-fix, the
    dominant term is the Dijkstra heuristic pre-pass, which sweeps a fixed
    0..60 index grid (`calc_holonomic_heuristic_with_obstacle` hard-codes
    `int(6.0 / xy_resolution)`) and therefore costs about the same no matter
    how much of the arena is mapped. If you see time climbing with cell
    count, the spatial index has regressed.

EXPECTED OUTPUT FORMAT
----------------------
  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  !! SAMPLE BELOW IS AN ILLUSTRATIVE LAYOUT MOCK-UP.                      !!
  !! EVERY DIGIT IN IT IS INVENTED TO SHOW COLUMN ALIGNMENT.              !!
  !! IT IS NOT A MEASUREMENT AND MUST NEVER BE QUOTED AS ONE.             !!
  !! Run the script to obtain real values.                               !!
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    ==============================================================
     Hybrid A* planning benchmark  (post spatial-index)
    ==============================================================
    host           : <hostname>  (Linux <kernel> <arch>)
    planner        : <path>/tactical_brain/A_planner.py
    arena          : <..> m  -> ARENA_MIN <..>  ARENA_MAX <..>
    resolutions    : XY <..> m   YAW <..> deg   MOVE_STEP <..> m
    clearance      : PLANNING_CLEARANCE <..> m
    repeats        : <n> per (density x scenario)

    --------------------------------------------------------------
     MAP: target <n> cells -> actual <n> occupied  (fill=surround)
    --------------------------------------------------------------
      [short ~1.0 m]  (<..>,<..>) -> (<..>,<..>)
          time ms  min <..>  mean <..>  median <..>  p95 <..>  max <..>
          path     steps <..>   cost <..>   failures <n>/<n>
          verdict  vs 500.0 ms budget (REAL): <PASS|FAIL>
                   vs 100.0 ms (dev-log question, NOT the deadline): <..>

    ==============================================================
     SUMMARY
    ==============================================================
    cells  scenario        n  fail  steps   cost    min    mean  median     p95     max
    ---------------------------------------------------------------------------------
     <..>  short          <n>  <n>   <..>   <..>   <..>   <..>    <..>    <..>    <..>

    HISTORICAL COMPARISON at ~1600 occupied cells
      before (full-scan check_collision) : ~300000 ms  (~300 s, documented)
      after  (bucketed spatial index)    : <..> ms  (mean, worst scenario)
      speedup                            : ~<..>x

    OVERALL VERDICT  (worst p95 over every row)
      real deadline  500.0 ms (behaviour-tree timer 0.5 s) : <PASS|FAIL>
      dev-log figure 100.0 ms (not a requirement)          : <MET|NOT MET>

`--csv PATH` appends one row per (density x scenario), header written only
when the file is created. Columns: timestamp_utc, host, arena_size,
fill_mode, seed, target_cells, actual_cells, scenario, start_x, start_y,
start_yaw, goal_x, goal_y, goal_yaw, straight_line_m, repeats, failures,
steps_median, steps_min, steps_max, cost_median, t_min_ms, t_mean_ms,
t_median_ms, t_p95_ms, t_max_ms, budget_ms, verdict_budget, verdict_100ms.
"""

from __future__ import annotations

import argparse
import csv
import math
import platform
import socket
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

# ─────────────────────────── project constants ────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent

PKG_SUBPATH = Path("ros2_ws/src/tactical_brain/tactical_brain")
# The package exists at BOTH of these in different checkouts of this repo.
PLANNER_CANDIDATES = (
    REPO_ROOT / "AutonomousWarfare" / "AutonomousWarfare" / PKG_SUBPATH,
    REPO_ROOT / "AutonomousWarfare" / PKG_SUBPATH,
    Path.home() / "ros2_ws" / "src" / "tactical_brain" / "tactical_brain",
)

# main_brain.py: self.tree_timer = self.create_timer(0.5, self.sense_and_think)
TREE_TIMER_SEC = 0.5
DEFAULT_BUDGET_MS = TREE_TIMER_SEC * 1000.0      # 500 ms — the REAL deadline
DEV_LOG_BUDGET_MS = 100.0                        # the number the dev log asks about

# A_planner.build_spatial_index docstring: full-scan check_collision was
# confirmed to reach ~300 s per search past ~1600 occupied cells.
HISTORICAL_REGRESSION_CELLS = 1600
HISTORICAL_REGRESSION_MS = 300_000.0

DEFAULT_DENSITIES = (200, 800, 1600, 3200)

# Everything below is expressed as a FRACTION of the arena side length so
# `--arena-size` rescales the layout instead of invalidating it. Baseline is
# the 5.0 m arena the numbers were tuned on.
BASE_ARENA = 5.0

# Interior structure: one wall with a gap at the top, plus two pillars.
# Deliberately light — the regression scaled with len(obstacle_set), not with
# maze complexity, and a pathological maze would just add search time noise.
WALL_X_FRAC = 0.40            # vertical wall at 2.0 m on a 5 m arena
WALL_Y0_FRAC, WALL_Y1_FRAC = 0.00, 0.44   # spans 0.0 -> 2.2 m, gap above
PILLAR_FRACS = ((0.68, 0.28), (0.20, 0.84))   # (3.4,1.4) and (1.0,4.2) m
PILLAR_HALF_M = 0.15          # 0.3 m square pillars

# Start/goal pairs, as fractions of the arena side.
SCENARIO_FRACS = (
    ("short",  "~1.0 m",     (0.18, 0.18, 0.0),  (0.18, 0.38, math.pi / 2)),
    ("medium", "~3.0 m",     (0.18, 0.18, 0.0),  (0.52, 0.68, 0.0)),
    ("long",   "~5.1 m diag", (0.14, 0.14, 0.0), (0.86, 0.86, math.pi / 4)),
)

# Cost-only actors (they change step_cost, never feasibility). Positions as
# arena fractions; radii are world_model.R_ENEMY / R_TEAMMATE.
ENEMY_FRACS = ((0.60, 0.50), (0.30, 0.80), (0.80, 0.25))
TEAMMATE_FRACS = ((0.40, 0.70), (0.70, 0.60), (0.25, 0.45))
R_ENEMY_M = 0.5               # world_model.R_ENEMY
R_TEAMMATE_M = 0.3            # world_model.R_TEAMMATE

# Mapped region grown beyond the arena, in metres per side. slam_toolbox maps
# the room, not just the drivable box.
SURROUND_MARGIN_M = 2.0

CSV_COLUMNS = [
    "timestamp_utc", "host", "arena_size", "fill_mode", "seed",
    "target_cells", "actual_cells", "scenario",
    "start_x", "start_y", "start_yaw", "goal_x", "goal_y", "goal_yaw",
    "straight_line_m", "repeats", "failures",
    "steps_median", "steps_min", "steps_max", "cost_median",
    "t_min_ms", "t_mean_ms", "t_median_ms", "t_p95_ms", "t_max_ms",
    "budget_ms", "verdict_budget", "verdict_100ms",
]


# ───────────────────────────── small helpers ──────────────────────────────
def die(msg: str, code: int = 2) -> "None":
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(code)


def percentile(sorted_vals, q: float) -> float:
    """Linear-interpolation percentile. `sorted_vals` must already be sorted."""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def cell(value_m: float, resolution: float) -> int:
    """Metres -> integer grid index, exactly as calc_hybrid_a_star does it."""
    return int(round(value_m / resolution))


# ──────────────────────── import the planner module ───────────────────────
def import_planner(explicit):
    """Put the tactical_brain package dir on sys.path and import A_planner.

    A_planner.py has no intra-package imports (math/heapq/numpy/time only),
    so it loads as a plain top-level module with no ROS 2 workspace built.
    """
    candidates = []
    if explicit:
        # An explicit path that does not hold A_planner.py is a typo, not an
        # invitation to go looking elsewhere — say so instead of silently
        # benchmarking a different copy of the planner than the one asked for.
        forced = Path(explicit).expanduser().resolve()
        forced_dir = forced.parent if forced.is_file() else forced
        if not (forced_dir / "A_planner.py").is_file():
            die(f"--planner-path {forced_dir} does not contain A_planner.py")
        candidates.append(forced_dir)
    candidates.extend(PLANNER_CANDIDATES)
    # The package has been relocated inside this repo before; glob as a last
    # resort rather than fail on a stale hard-coded path.
    try:
        candidates.extend(sorted(REPO_ROOT.glob("**/tactical_brain/tactical_brain")))
    except OSError:
        pass

    tried = []
    for candidate in candidates:
        if candidate is None:
            continue
        pkg_dir = candidate
        if pkg_dir.is_file():                     # user passed A_planner.py itself
            pkg_dir = pkg_dir.parent
        tried.append(str(pkg_dir))
        if not (pkg_dir / "A_planner.py").is_file():
            continue
        sys.path.insert(0, str(pkg_dir))
        try:
            import A_planner                       # noqa: PLC0415
        except ImportError as exc:
            die(f"found {pkg_dir / 'A_planner.py'} but could not import it "
                f"({exc}).\n       A_planner needs numpy:  pip install numpy")
        return A_planner, pkg_dir / "A_planner.py"

    die("could not locate the tactical_brain package (A_planner.py).\n"
        "       Tried:\n" + "\n".join(f"         {t}" for t in dict.fromkeys(tried))
        + "\n       Pass the directory explicitly with --planner-path.")


# ────────────────────────────── map building ──────────────────────────────
def build_scenarios(arena_size: float):
    scale = arena_size
    scenarios = []
    for name, label, start_f, goal_f in SCENARIO_FRACS:
        start = (start_f[0] * scale, start_f[1] * scale, start_f[2])
        goal = (goal_f[0] * scale, goal_f[1] * scale, goal_f[2])
        scenarios.append({
            "name": name,
            "label": label,
            "start": start,
            "goal": goal,
            "straight_m": math.hypot(goal[0] - start[0], goal[1] - start[1]),
        })
    return scenarios


def structure_cells(arena_size: float, resolution: float):
    """The fixed drivable-arena layout: one gapped wall plus two pillars.

    Note there is NO perimeter wall: check_collision already geofences on
    ARENA_MIN/ARENA_MAX, so a ring of perimeter cells would only inflate
    len(obstacle_set) without changing what the search can reach.
    """
    cells = set()
    wall_x = cell(WALL_X_FRAC * arena_size, resolution)
    y0 = cell(WALL_Y0_FRAC * arena_size, resolution)
    y1 = cell(WALL_Y1_FRAC * arena_size, resolution)
    for y in range(y0, y1 + 1):
        cells.add((wall_x, y))

    half = max(1, cell(PILLAR_HALF_M, resolution))
    for fx, fy in PILLAR_FRACS:
        cx = cell(fx * arena_size, resolution)
        cy = cell(fy * arena_size, resolution)
        for dx in range(-half, half + 1):
            for dy in range(-half, half + 1):
                cells.add((cx + dx, cy + dy))
    return cells


def forbidden_predicate(scenarios, resolution: float, min_sep_m: float):
    """A cell is forbidden if it sits within min_sep_m of any start/goal.

    PLANNING_CLEARANCE is enforced by check_collision against the *node*
    position, so an obstacle any closer than that to the start or the goal
    makes every trial fail before it has planned anything.
    """
    points = []
    for sc in scenarios:
        points.append((sc["start"][0], sc["start"][1]))
        points.append((sc["goal"][0], sc["goal"][1]))

    def forbidden(ix, iy):
        x, y = ix * resolution, iy * resolution
        for px, py in points:
            if math.hypot(x - px, y - py) <= min_sep_m:
                return True
        return False

    return forbidden


def build_map(planner, target_cells, scenarios, args, rng):
    """Return (obstacle_set, meta) for one obstacle density."""
    resolution = planner.XY_RESOLUTION
    min_sep = planner.PLANNING_CLEARANCE + args.clearance_margin

    forbidden = forbidden_predicate(scenarios, resolution, min_sep)

    struct = structure_cells(args.arena_size, resolution)
    dropped_struct = {c for c in struct if forbidden(*c)}
    obstacles = struct - dropped_struct

    lo = cell(-SURROUND_MARGIN_M, resolution)
    hi = cell(args.arena_size + SURROUND_MARGIN_M, resolution)
    arena_lo, arena_hi = 0, cell(args.arena_size, resolution)

    pool = []
    for ix in range(lo, hi + 1):
        inside_x = arena_lo <= ix <= arena_hi
        for iy in range(lo, hi + 1):
            if args.fill_mode == "surround" and inside_x and arena_lo <= iy <= arena_hi:
                continue                     # keep the drivable arena untouched
            if (ix, iy) in obstacles:
                continue
            if forbidden(ix, iy):
                continue
            pool.append((ix, iy))

    want = max(0, target_cells - len(obstacles))
    truncated = False
    if want > len(pool):
        want = len(pool)
        truncated = True
    obstacles.update(rng.sample(pool, want))

    meta = {
        "target": target_cells,
        "actual": len(obstacles),
        "structure": len(struct) - len(dropped_struct),
        "dropped_structure": len(dropped_struct),
        "pool": len(pool),
        "truncated": truncated,
    }
    return obstacles, meta


def build_dynamic(planner, args):
    """danger_dict {(x_ind,y_ind): unix_ts} and teammates_aura_set.

    Cost modifiers only — neither can make a node infeasible, so they never
    change the failure count, only the returned cost and the shape of the
    search frontier.
    """
    resolution = planner.XY_RESOLUTION
    now = time.time()

    danger = {}
    r_enemy = cell(R_ENEMY_M, resolution)
    for fx, fy in ENEMY_FRACS[:max(0, args.enemies)]:
        ex = cell(fx * args.arena_size, resolution)
        ey = cell(fy * args.arena_size, resolution)
        for dx in range(-r_enemy, r_enemy + 1):
            for dy in range(-r_enemy, r_enemy + 1):
                danger[(ex + dx, ey + dy)] = now

    aura = set()
    r_mate = cell(R_TEAMMATE_M, resolution)
    for fx, fy in TEAMMATE_FRACS[:max(0, args.teammates)]:
        tx = cell(fx * args.arena_size, resolution)
        ty = cell(fy * args.arena_size, resolution)
        for dx in range(-r_mate, r_mate + 1):
            for dy in range(-r_mate, r_mate + 1):
                aura.add((tx + dx, ty + dy))
    return danger, aura


# ──────────────────────────── run one scenario ────────────────────────────
def run_scenario(planner, scenario, obstacle_set, danger, aura, args):
    times, steps, costs = [], [], []
    failures = 0

    for _ in range(args.repeats):
        start_t = time.perf_counter()
        result = planner.calc_hybrid_a_star(
            scenario["start"], scenario["goal"], obstacle_set,
            planner.XY_RESOLUTION, planner.YAW_RESOLUTION, danger, aura)
        elapsed_ms = (time.perf_counter() - start_t) * 1000.0
        times.append(elapsed_ms)

        if result is None:
            failures += 1
            continue
        (rx, _ry, _rt, _rd), cost = result
        steps.append(len(rx))
        costs.append(cost)

    ordered = sorted(times)
    return {
        "scenario": scenario["name"],
        "label": scenario["label"],
        "start": scenario["start"],
        "goal": scenario["goal"],
        "straight_m": scenario["straight_m"],
        "n": len(times),
        "failures": failures,
        "min": ordered[0],
        "mean": statistics.fmean(times),
        "median": statistics.median(times),
        "p95": percentile(ordered, 0.95),
        "max": ordered[-1],
        "steps_median": int(statistics.median(steps)) if steps else None,
        "steps_min": min(steps) if steps else None,
        "steps_max": max(steps) if steps else None,
        "cost_median": statistics.median(costs) if costs else None,
    }


# ───────────────────────────────── report ─────────────────────────────────
def verdict(value_ms, budget_ms):
    return "PASS" if value_ms <= budget_ms else "FAIL"


def print_map_block(meta, args, results, budget_ms):
    print("\n" + "-" * 78)
    head = (f" MAP: target {meta['target']} cells -> actual {meta['actual']} "
            f"occupied  (fill={args.fill_mode})")
    print(head)
    print("-" * 78)
    print(f"   structure {meta['structure']} cells "
          f"(wall+pillars, identical at every density)"
          + (f", {meta['dropped_structure']} dropped for start/goal clearance"
             if meta["dropped_structure"] else ""))
    if meta["truncated"]:
        print(f"   WARNING: only {meta['pool']} legal filler cells available — "
              f"could not reach the {meta['target']}-cell target.")

    for res in results:
        print(f"\n  [{res['scenario']} {res['label']}]  "
              f"({res['start'][0]:.2f},{res['start'][1]:.2f},"
              f"{res['start'][2]:.2f}) -> "
              f"({res['goal'][0]:.2f},{res['goal'][1]:.2f},"
              f"{res['goal'][2]:.2f})   straight-line {res['straight_m']:.2f} m")
        print(f"      time ms  min {res['min']:9.2f}  mean {res['mean']:9.2f}  "
              f"median {res['median']:9.2f}  p95 {res['p95']:9.2f}  "
              f"max {res['max']:9.2f}")
        if res["steps_median"] is None:
            print(f"      path     NO PATH RETURNED in any of {res['n']} trials "
                  f"(failures {res['failures']}/{res['n']})")
            print("               a fast None is not a fast plan — do not quote "
                  "this row as planning time")
        else:
            span = ("" if res["steps_min"] == res["steps_max"]
                    else f" (range {res['steps_min']}..{res['steps_max']})")
            print(f"      path     steps {res['steps_median']}{span}   "
                  f"cost {res['cost_median']:.3f}   "
                  f"failures {res['failures']}/{res['n']}")
        print(f"      verdict  vs {budget_ms:.1f} ms budget (REAL deadline, "
              f"tree timer {TREE_TIMER_SEC} s) : "
              f"{verdict(res['p95'], budget_ms)}   [p95]")
        print(f"               vs {DEV_LOG_BUDGET_MS:.1f} ms (dev-log question, "
              f"NOT the deadline)  : {verdict(res['p95'], DEV_LOG_BUDGET_MS)}"
              f"   [p95]")


def print_summary(rows, budget_ms):
    print("\n" + "=" * 90)
    print(" SUMMARY   (time in ms; one row per obstacle density x start/goal pair)")
    print("=" * 90)
    print(f"{'cells':>6}{'scenario':>10}{'n':>4}{'fail':>6}{'steps':>7}"
          f"{'cost':>9}{'min':>9}{'mean':>9}{'median':>9}{'p95':>9}{'max':>9}")
    print("-" * 90)
    for cells_n, res in rows:
        steps = "-" if res["steps_median"] is None else str(res["steps_median"])
        cost = "-" if res["cost_median"] is None else f"{res['cost_median']:.2f}"
        print(f"{cells_n:>6}{res['scenario']:>10}{res['n']:>4}"
              f"{res['failures']:>6}{steps:>7}{cost:>9}"
              f"{res['min']:>9.2f}{res['mean']:>9.2f}{res['median']:>9.2f}"
              f"{res['p95']:>9.2f}{res['max']:>9.2f}")

    # ── historical comparison at the documented regression point
    near = [(c, r) for c, r in rows
            if abs(c - HISTORICAL_REGRESSION_CELLS)
            <= 0.25 * HISTORICAL_REGRESSION_CELLS and r["steps_median"] is not None]
    print("\n HISTORICAL COMPARISON at ~%d occupied cells"
          % HISTORICAL_REGRESSION_CELLS)
    print("   before (full-scan check_collision) : ~%.0f ms  (~%.0f s, the "
          "documented regression)" % (HISTORICAL_REGRESSION_MS,
                                      HISTORICAL_REGRESSION_MS / 1000.0))
    if near:
        worst = max(near, key=lambda item: item[1]["mean"])
        after_ms = worst[1]["mean"]
        print(f"   after  (bucketed spatial index)    : {after_ms:.2f} ms  "
              f"(mean, worst scenario '{worst[1]['scenario']}' @ "
              f"{worst[0]} cells)")
        if after_ms > 0:
            print(f"   speedup                            : "
                  f"~{HISTORICAL_REGRESSION_MS / after_ms:,.0f}x")
    else:
        print("   after  (bucketed spatial index)    : not measured — no "
              "successful row near that density in this run")

    # ── overall verdict
    worst_p95 = max(r["p95"] for _c, r in rows)
    total_fail = sum(r["failures"] for _c, r in rows)
    total_runs = sum(r["n"] for _c, r in rows)
    print("\n OVERALL VERDICT  (worst p95 over every row: %.2f ms)" % worst_p95)
    print(f"   real deadline  {budget_ms:.1f} ms (behaviour-tree timer "
          f"{TREE_TIMER_SEC} s, main_brain.sense_and_think) : "
          f"{verdict(worst_p95, budget_ms)}")
    met = "MET" if worst_p95 <= DEV_LOG_BUDGET_MS else "NOT MET"
    print(f"   dev-log figure {DEV_LOG_BUDGET_MS:.1f} ms (asked about in the "
          f"log; NOT a requirement)          : {met}")
    print(f"   planning failures: {total_fail}/{total_runs} trials returned None")
    if total_fail:
        print("   (rows with failures measure search-exhaustion time, not "
              "path-calculation time)")


def write_csv(path, rows, meta):
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    new_file = not target.exists()
    with target.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        for cells_n, res in rows:
            writer.writerow({
                "timestamp_utc": meta["timestamp"],
                "host": meta["host"],
                "arena_size": meta["arena_size"],
                "fill_mode": meta["fill_mode"],
                "seed": meta["seed"],
                "target_cells": meta["targets"][cells_n],
                "actual_cells": cells_n,
                "scenario": res["scenario"],
                "start_x": f"{res['start'][0]:.3f}",
                "start_y": f"{res['start'][1]:.3f}",
                "start_yaw": f"{res['start'][2]:.4f}",
                "goal_x": f"{res['goal'][0]:.3f}",
                "goal_y": f"{res['goal'][1]:.3f}",
                "goal_yaw": f"{res['goal'][2]:.4f}",
                "straight_line_m": f"{res['straight_m']:.3f}",
                "repeats": res["n"],
                "failures": res["failures"],
                "steps_median": "" if res["steps_median"] is None else res["steps_median"],
                "steps_min": "" if res["steps_min"] is None else res["steps_min"],
                "steps_max": "" if res["steps_max"] is None else res["steps_max"],
                "cost_median": "" if res["cost_median"] is None else f"{res['cost_median']:.4f}",
                "t_min_ms": f"{res['min']:.4f}",
                "t_mean_ms": f"{res['mean']:.4f}",
                "t_median_ms": f"{res['median']:.4f}",
                "t_p95_ms": f"{res['p95']:.4f}",
                "t_max_ms": f"{res['max']:.4f}",
                "budget_ms": meta["budget_ms"],
                "verdict_budget": verdict(res["p95"], meta["budget_ms"]),
                "verdict_100ms": verdict(res["p95"], DEV_LOG_BUDGET_MS),
            })
    print(f"\nCSV {'created' if new_file else 'appended'}: {target}")


# ─────────────────────────────────  main  ─────────────────────────────────
def build_parser():
    parser = argparse.ArgumentParser(
        description="Measure Hybrid A* planning time and path step count "
                    "after the spatial-index optimisation.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="No ROS 2 workspace, no robot and no Gazebo required. "
               "Run it on the Pi if the number will be quoted.")
    parser.add_argument("--repeats", type=int, default=20,
                        help="Timed calc_hybrid_a_star() calls per "
                             "(density x scenario).")
    parser.add_argument("--densities", type=int, nargs="+",
                        default=list(DEFAULT_DENSITIES), metavar="N",
                        help="Target occupied-cell counts to sweep. 1600 is "
                             "the documented regression point.")
    parser.add_argument("--arena-size", type=float, default=5.0,
                        help="Drivable arena side length [m]; passed to "
                             "A_planner.set_arena_bounds().")
    parser.add_argument("--arena-margin", type=float, default=0.1,
                        help="Geofence margin passed to set_arena_bounds().")
    parser.add_argument("--fill-mode", choices=("surround", "uniform"),
                        default="surround",
                        help="'surround' grows the map OUTSIDE the arena so "
                             "path difficulty is constant and only "
                             "len(obstacle_set) varies. 'uniform' scatters "
                             "filler over the arena too (harsher; expect "
                             "failures at high density).")
    parser.add_argument("--clearance-margin", type=float, default=0.05,
                        help="Extra metres on top of PLANNING_CLEARANCE kept "
                             "free around every start and goal.")
    parser.add_argument("--enemies", type=int, default=1,
                        help="Enemy danger discs (R=0.5 m) written into "
                             "danger_dict. 0 disables.")
    parser.add_argument("--teammates", type=int, default=1,
                        help="Teammate aura discs (R=0.3 m) written into "
                             "teammates_aura_set. 0 disables.")
    parser.add_argument("--budget-ms", type=float, default=DEFAULT_BUDGET_MS,
                        help="The real deadline: one behaviour-tree tick "
                             "(main_brain create_timer(0.5, ...)).")
    parser.add_argument("--seed", type=int, default=0,
                        help="RNG seed for the synthetic maps "
                             "(reproducibility).")
    parser.add_argument("--planner-path", default=None, metavar="DIR",
                        help="Explicit tactical_brain package directory "
                             "containing A_planner.py.")
    parser.add_argument("--csv", default=None, metavar="PATH",
                        help="Append machine-readable results to this CSV.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.repeats < 1:
        die("--repeats must be >= 1")
    if args.arena_size <= 2 * args.arena_margin:
        die("--arena-size must be larger than 2 * --arena-margin")
    if any(d < 0 for d in args.densities):
        die("--densities must be >= 0")

    import random
    rng = random.Random(args.seed)

    planner, planner_file = import_planner(args.planner_path)
    planner.set_arena_bounds(args.arena_size, args.arena_margin)

    scenarios = build_scenarios(args.arena_size)
    danger, aura = build_dynamic(planner, args)

    print("=" * 78)
    print(" Hybrid A* planning benchmark  (post spatial-index)")
    print("=" * 78)
    print(f"host           : {socket.gethostname()}  "
          f"({platform.system()} {platform.release()} {platform.machine()})")
    print(f"python         : {platform.python_version()}")
    print(f"planner        : {planner_file}")
    print(f"arena          : {args.arena_size:.2f} m  -> ARENA_MIN "
          f"{planner.ARENA_MIN:.2f}  ARENA_MAX {planner.ARENA_MAX:.2f}")
    print(f"resolutions    : XY {planner.XY_RESOLUTION} m   "
          f"YAW {math.degrees(planner.YAW_RESOLUTION):.1f} deg   "
          f"MOVE_STEP {planner.MOVE_STEP} m")
    print(f"clearance      : PLANNING_CLEARANCE {planner.PLANNING_CLEARANCE} m "
          f"(+{args.clearance_margin} m kept free around start/goal)")
    print(f"danger/aura    : {len(danger)} danger cells "
          f"({args.enemies} enemies), {len(aura)} aura cells "
          f"({args.teammates} teammates)")
    print(f"repeats        : {args.repeats} per (density x scenario)")
    print(f"seed           : {args.seed}   fill-mode: {args.fill_mode}")
    print(f"budget         : {args.budget_ms:.1f} ms REAL (tree timer "
          f"{TREE_TIMER_SEC} s) | {DEV_LOG_BUDGET_MS:.1f} ms dev-log figure")

    rows = []
    targets_by_actual = {}
    for target in args.densities:
        obstacle_set, meta = build_map(planner, target, scenarios, args, rng)
        targets_by_actual[meta["actual"]] = meta["target"]
        results = []
        for scenario in scenarios:
            print(f"\n>>> {meta['actual']} cells / {scenario['name']} "
                  f"({args.repeats} repeats) ...", flush=True)
            results.append(run_scenario(planner, scenario, obstacle_set,
                                        danger, aura, args))
        print_map_block(meta, args, results, args.budget_ms)
        rows.extend((meta["actual"], res) for res in results)

    print_summary(rows, args.budget_ms)

    print("\nreminder: this is calc_hybrid_a_star() in isolation. A real "
          "sense_and_think tick also\n          rebuilds the danger grid, "
          "runs the behaviour tree and publishes the path,\n          so the "
          "node's per-tick cost is HIGHER than the figures above.")

    if args.csv:
        write_csv(args.csv, rows, {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "host": socket.gethostname(),
            "arena_size": args.arena_size,
            "fill_mode": args.fill_mode,
            "seed": args.seed,
            "budget_ms": args.budget_ms,
            "targets": targets_by_actual,
        })

    worst_p95 = max(r["p95"] for _c, r in rows)
    return 0 if worst_p95 <= args.budget_ms else 1


if __name__ == "__main__":
    sys.exit(main())
