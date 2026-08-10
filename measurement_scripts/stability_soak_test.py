#!/usr/bin/env python3
"""
stability_soak_test.py — continuous-uptime soak test for the enemies dict race
==============================================================================

PURPOSE
-------
Measure "how long does it run before it crashes", for the specific crash that
commit 667a459 (2026-07-08) fixed: a data race on the shared
`enemies_by_detector` dict, now guarded by `self._state_lock =
threading.Lock()`.

The hazard, exactly as it exists in the node (the lock lives in
tactical_brain/team_comms.py; main_brain.py drives both sides of it):

  WRITERS, on the pose callback group's executor thread
      main_brain.local_enemy_position_callback
          -> TeamComms.record_local_detection
                 self.enemies_by_detector[self.robot_id] = {...}
      TeamComms._team_enemy_callback   (teammate broadcast over Zenoh)
                 self.enemies_by_detector[detector_id] = {...}

  READER, on the behaviour-tree timer thread (create_timer(0.5, ...))
      main_brain.sense_and_think
          -> TeamComms.get_enemies_snapshot
                 world_model.prune_stale_enemies(...)
                     {k: v for k, v in enemies_by_detector.items() if ...}

A MultiThreadedExecutor runs those two callback groups on two real threads.
An insert landing inside the reader's `.items()` iteration raises
`RuntimeError: dictionary changed size during iteration` inside the timer
callback, which kills the node. That is the crash being measured.

This script reproduces that pattern in isolation and soaks it:
  * `--no-lock`   : the BEFORE case. Should crash. Its time-to-first-exception
                    is the uptime figure for the unfixed code.
  * `--with-lock` : the AFTER case (default). Should survive the full
                    `--duration` with zero exceptions.
  * `--attach-live --pid N` : the real thing. Watches an actually-running node
                    process and reports wall-clock uptime until it dies.

Deliberately NO ROS 2 and NO ROBOT for the synthetic modes. A race that only
shows up under a real MultiThreadedExecutor is a race you cannot soak-test for
an hour on a bench; this stresses the same two-thread dict pattern directly,
at a far higher event rate than sensor_fusion_node could ever produce.

EXACT RUN COMMAND
-----------------
Confirm the fix holds for an hour (the headline "continuous uptime" number):

    python3 measurement_scripts/stability_soak_test.py

Reproduce the ORIGINAL crash and time it, 10 runs, for "crashes per N runs":

    python3 measurement_scripts/stability_soak_test.py \
        --no-lock --duration 60 --runs 10 --json results/soak_before.json

Fast smoke test (this is what was used to verify the script runs):

    python3 measurement_scripts/stability_soak_test.py \
        --no-lock --duration 5 --runs 2

Before/after pair for the report:

    python3 measurement_scripts/stability_soak_test.py --no-lock   --duration 300 --runs 5 --json results/before.json
    python3 measurement_scripts/stability_soak_test.py --with-lock --duration 3600 --runs 1 --json results/after.json

Watch a REAL running node instead (no synthetic threads at all):

    python3 measurement_scripts/stability_soak_test.py \
        --attach-live --pid $(pgrep -f main_brain) \
        --duration 3600 --sample-interval 5 --json results/live_soak.json

PREREQUISITES
-------------
  * Python 3.8+. Standard library ONLY — no numpy, no ROS 2, no rclpy, no
    robot, no Gazebo, no network.
  * `--attach-live` additionally needs a Linux /proc (it reads
    /proc/<pid>/stat and /proc/<pid>/status). It is read-only: it never
    signals, traces or attaches to the target, so it cannot itself perturb
    the node it is measuring.
  * For a meaningful `--duration 3600` result, run it on the Pi, and do not
    run it under a debugger or with tracing enabled — `sys.settrace` changes
    thread-switch behaviour enough to hide the race.

WHY --switch-interval EXISTS (do not remove it)
-----------------------------------------------
The commit notes the bug only reproduced with a very small interpreter
switch interval. `sys.setswitchinterval()` sets how long a thread may hold
the GIL before the interpreter offers it to another thread; the default 5 ms
is long enough that a short dict iteration usually completes uninterrupted,
so the race is rare and the "before" case can soak for a long time looking
healthy. Dropping it to 1e-6 s forces switches mid-iteration and turns a
rare production crash into a reliable one. It makes the crash FASTER, not
different — the exception raised is the same one seen on the robot. It
follows that `--no-lock` time-to-crash here is a LOWER BOUND on the field
MTBF, not a prediction of it; the `--with-lock` survival is the real result.

WHAT THE NUMBERS MEAN (read before quoting the output)
------------------------------------------------------
  * "uptime" for a synthetic run = seconds from the moment the threads start
    to the first exception in ANY of them, measured with
    `time.perf_counter()`. A run that reaches `--duration` with no exception
    reports "survived" and contributes its full duration to the mean.
  * "iterations" = reader prune passes + writer inserts, summed. It is a
    stress-volume figure, so a survival can be quoted as "N million
    dict operations with zero exceptions" rather than just "1 hour".
  * MTBF = total soak seconds / number of crashes. With zero crashes it is
    reported as "> total soak seconds" — an hour of silence is a lower bound,
    never a proof.
  * `--attach-live` measures the PROCESS, so it reports any death (this race,
    an unrelated exception, the OOM killer, a manual kill) without being able
    to tell them apart. Check the node's own log for the cause. RSS growth is
    sampled alongside it purely as a leak canary.

EXPECTED OUTPUT FORMAT
----------------------
  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  !! SAMPLE BELOW IS AN ILLUSTRATIVE LAYOUT MOCK-UP.                      !!
  !! EVERY DIGIT IN IT IS INVENTED TO SHOW COLUMN ALIGNMENT.              !!
  !! IT IS NOT A MEASUREMENT AND MUST NEVER BE QUOTED AS ONE.             !!
  !! Run the script to obtain real values.                               !!
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    ==============================================================
     enemies_by_detector race — stability soak
    ==============================================================
    host            : <hostname>  (Linux <kernel> <arch>)
    mode            : synthetic   locking: NO LOCK (pre-667a459)
    writers         : <n> threads   reader: 1 thread
    switch interval : <..> s  (was <..> s)
    duration        : <n> s per run    runs: <n>

    --- run 1/2 ----------------------------------------------------
      CRASHED after <..> s
        RuntimeError: dictionary changed size during iteration
        thread: reader        iterations: <n>

    ==============================================================
     AGGREGATE
    ==============================================================
    runs                : <n>
    crashes             : <n> / <n>   (<..>%)
    uptime (s)          : min <..>  mean <..>  median <..>  max <..>
    total soak          : <..> s
    total iterations    : <n>
    MTBF                : <..> s
    exceptions seen     : RuntimeError: dictionary changed size ... x<n>

    VERDICT: <..>

`--json PATH` writes one object: {config, runs: [...], aggregate: {...}}.
Every run entry carries seed-independent facts only (uptime_s, iterations,
crashed, exc_type, exc_msg, thread), so runs are directly comparable.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import socket
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

# ─────────────────────────── project constants ────────────────────────────
# world_model.ENEMY_MEMORY_TIMEOUT — how long a sighting stays believed.
ENEMY_MEMORY_TIMEOUT = 2.0
# main_brain.py: self.tree_timer = self.create_timer(0.5, self.sense_and_think)
TREE_TIMER_SEC = 0.5
# The commit this test covers.
FIX_COMMIT = "667a459"
FIX_DATE = "2026-07-08"
# The exception the unfixed code dies with.
TARGET_EXC = "dictionary changed size during iteration"


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


def fmt_duration(seconds) -> str:
    if seconds is None:
        return "n/a"
    if seconds < 90:
        return f"{seconds:.3f} s"
    if seconds < 5400:
        return f"{seconds:.1f} s ({seconds / 60:.1f} min)"
    return f"{seconds:.0f} s ({seconds / 3600:.2f} h)"


# ───────────────── the shared state under test (mirrors TeamComms) ────────
class SharedEnemyState:
    """A stand-in for TeamComms, carrying only what the race touches.

    `enemies_by_detector` is REASSIGNED by the reader (prune_stale_enemies
    returns a new dict) and MUTATED IN PLACE by the writers — exactly as in
    team_comms.py. Writers therefore re-read the attribute on every insert
    rather than caching a reference, or they would end up writing into an
    already-discarded dict and the race would quietly disappear.
    """

    __slots__ = ("enemies_by_detector", "lock", "writes", "prunes")

    def __init__(self, use_lock: bool):
        self.enemies_by_detector = {}
        self.lock = threading.Lock() if use_lock else None
        self.writes = 0
        self.prunes = 0


class _NullLock:
    """`with` target for the --no-lock case. Costs nothing, guards nothing."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


NULL_LOCK = _NullLock()


def prune_stale_enemies(enemies_by_detector, memory_timeout):
    """Byte-for-byte the same shape as world_model.prune_stale_enemies.

    The dict comprehension below is the exact line that raises when a writer
    inserts a new key mid-iteration.
    """
    current_time = time.time()
    return {
        detector_id: enemy
        for detector_id, enemy in enemies_by_detector.items()
        if current_time - enemy.get("timestamp", current_time) < memory_timeout
    }


# ────────────────────────────── worker threads ────────────────────────────
class Crash(Exception):
    """Never raised — the container type for a recorded worker failure."""


def writer_loop(state, stop_event, failure, detector_ids, index, args):
    """Mirrors record_local_detection / _team_enemy_callback.

    Each writer owns a slice of the detector-id space so the dict genuinely
    grows and shrinks (a fixed key set would only ever overwrite, never
    resize, and resizing is what makes the iteration explode).
    """
    guard = state.lock if state.lock is not None else NULL_LOCK
    ids = detector_ids[index::args.writers]
    if not ids:
        ids = detector_ids
    position = 0
    local_writes = 0
    try:
        while not stop_event.is_set():
            detector_id = ids[position % len(ids)]
            position += 1
            now = time.time()
            with guard:
                # The attribute is re-read here on purpose; see SharedEnemyState.
                state.enemies_by_detector[detector_id] = {
                    "x": 1.0, "y": 2.0, "timestamp": now,
                }
            local_writes += 1
            if args.write_delay:
                time.sleep(args.write_delay)
    except BaseException as exc:                                  # noqa: BLE001
        failure.record(exc, f"writer-{index}")
        stop_event.set()
    finally:
        with threading.Lock():
            state.writes += local_writes


def reader_loop(state, stop_event, failure, args):
    """Mirrors sense_and_think -> get_enemies_snapshot -> prune_stale_enemies."""
    guard = state.lock if state.lock is not None else NULL_LOCK
    local_prunes = 0
    try:
        while not stop_event.is_set():
            with guard:
                state.enemies_by_detector = prune_stale_enemies(
                    state.enemies_by_detector, args.stale_timeout)
                snapshot = list(state.enemies_by_detector.values())
            local_prunes += 1
            if snapshot and args.read_delay:
                time.sleep(args.read_delay)
    except BaseException as exc:                                  # noqa: BLE001
        failure.record(exc, "reader")
        stop_event.set()
    finally:
        with threading.Lock():
            state.prunes += local_prunes


class FirstFailure:
    """Records only the first exception, whichever thread raises it first."""

    def __init__(self):
        self._lock = threading.Lock()
        self.exc = None
        self.exc_type = None
        self.thread = None
        self.at = None

    def record(self, exc, thread_name):
        with self._lock:
            if self.exc is None:
                self.at = time.perf_counter()
                self.exc = exc
                self.exc_type = type(exc).__name__
                self.thread = thread_name


# ────────────────────────────── one synthetic run ─────────────────────────
def run_soak(args, run_index):
    state = SharedEnemyState(use_lock=args.with_lock)
    stop_event = threading.Event()
    failure = FirstFailure()
    detector_ids = [f"robot_{i}" for i in range(args.keys)]

    threads = [threading.Thread(
        target=writer_loop,
        args=(state, stop_event, failure, detector_ids, i, args),
        name=f"writer-{i}", daemon=True) for i in range(args.writers)]
    threads.append(threading.Thread(
        target=reader_loop, args=(state, stop_event, failure, args),
        name="reader", daemon=True))

    started = time.perf_counter()
    for thread in threads:
        thread.start()

    deadline = started + args.duration
    try:
        while time.perf_counter() < deadline and not stop_event.is_set():
            # Poll rather than join(): a join would have to be interrupted to
            # honour --duration, and the poll costs nothing next to the
            # millions of dict operations the workers are doing.
            time.sleep(0.01)
    except KeyboardInterrupt:
        stop_event.set()
        raise
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=5.0)

    ended = time.perf_counter()
    crashed = failure.exc is not None
    uptime = (failure.at - started) if crashed else (ended - started)

    return {
        "run": run_index,
        "crashed": crashed,
        "uptime_s": uptime,
        "wall_s": ended - started,
        "iterations": state.writes + state.prunes,
        "writes": state.writes,
        "prunes": state.prunes,
        "exc_type": failure.exc_type,
        "exc_msg": str(failure.exc) if crashed else None,
        "thread": failure.thread,
        "is_target_race": bool(crashed and TARGET_EXC in str(failure.exc)),
    }


def print_run(result, total_runs, args):
    print(f"\n--- run {result['run']}/{total_runs} " + "-" * 46)
    if result["crashed"]:
        print(f"  CRASHED after {fmt_duration(result['uptime_s'])}")
        print(f"    {result['exc_type']}: {result['exc_msg']}")
        print(f"    thread: {result['thread']:<12} "
              f"iterations: {result['iterations']:,}")
        if not result["is_target_race"]:
            print(f"    NOTE: this is NOT the '{TARGET_EXC}' race this test "
                  f"targets — investigate separately.")
    else:
        print(f"  SURVIVED {fmt_duration(result['uptime_s'])} with no exception")
        print(f"    iterations: {result['iterations']:,} "
              f"({result['writes']:,} writes / {result['prunes']:,} prunes)")
        if not args.with_lock:
            print("    NOTE: --no-lock was expected to crash. Raise --duration, "
                  "lower --switch-interval,\n          or raise --writers; the "
                  "race is real but probabilistic.")


# ──────────────────────────── --attach-live mode ──────────────────────────
def read_rss_kb(pid):
    """VmRSS in KiB from /proc/<pid>/status, or None if the process is gone."""
    try:
        with open(f"/proc/{pid}/status", "r") as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        return None
    return None


def read_proc_name(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            raw = handle.read().replace(b"\x00", b" ").strip()
        if raw:
            return raw.decode("utf-8", "replace")[:120]
    except OSError:
        pass
    try:
        with open(f"/proc/{pid}/comm", "r") as handle:
            return handle.read().strip()
    except OSError:
        return "<unknown>"


def proc_alive(pid):
    """Alive == /proc/<pid> exists and the task is not a zombie."""
    try:
        with open(f"/proc/{pid}/stat", "r") as handle:
            fields = handle.read().rsplit(")", 1)[-1].split()
    except OSError:
        return False
    return bool(fields) and fields[0] != "Z"


def attach_live(args):
    if not Path("/proc").is_dir():
        die("--attach-live needs a Linux /proc filesystem")
    if args.pid is None:
        die("--attach-live requires --pid (e.g. --pid $(pgrep -f main_brain))")
    if not proc_alive(args.pid):
        die(f"pid {args.pid} is not running (or is a zombie) — nothing to watch")

    name = read_proc_name(args.pid)
    print("=" * 78)
    print(" LIVE PROCESS SOAK  (read-only /proc sampling)")
    print("=" * 78)
    print(f"host            : {socket.gethostname()}  "
          f"({platform.system()} {platform.release()} {platform.machine()})")
    print(f"pid             : {args.pid}")
    print(f"cmdline         : {name}")
    print(f"sample interval : {args.sample_interval} s")
    print(f"max duration    : {fmt_duration(args.duration)}")
    print("\nsampling (Ctrl-C to stop early and still get a report) ...")

    started = time.perf_counter()
    samples = []
    died_at = None
    interrupted = False
    try:
        while True:
            now = time.perf_counter()
            elapsed = now - started
            if elapsed >= args.duration:
                break
            alive = proc_alive(args.pid)
            rss = read_rss_kb(args.pid) if alive else None
            if rss is not None:
                samples.append((elapsed, rss))
            if not alive:
                died_at = elapsed
                break
            if args.verbose:
                print(f"  t={elapsed:8.1f} s  alive  "
                      f"RSS {rss / 1024.0:8.1f} MiB" if rss is not None
                      else f"  t={elapsed:8.1f} s  alive  RSS n/a")
            time.sleep(args.sample_interval)
    except KeyboardInterrupt:
        interrupted = True
        print("\n  interrupted by user")

    uptime = died_at if died_at is not None else (time.perf_counter() - started)
    rss_values = [kb for _t, kb in samples]

    print("\n" + "=" * 78)
    print(" LIVE RESULT")
    print("=" * 78)
    if died_at is not None:
        print(f"  PROCESS DIED after {fmt_duration(died_at)} of observation")
        print("  cause is NOT determinable from /proc — check the node's own "
              "log/journal.")
    elif interrupted:
        print(f"  still alive after {fmt_duration(uptime)} (observation "
              f"stopped early by the user)")
    else:
        print(f"  SURVIVED the full {fmt_duration(args.duration)} observation "
              f"window, still alive")
    print(f"  samples         : {len(samples)}")
    if rss_values:
        first, last = rss_values[0], rss_values[-1]
        print(f"  RSS MiB         : first {first / 1024.0:.1f}  "
              f"last {last / 1024.0:.1f}  min {min(rss_values) / 1024.0:.1f}  "
              f"mean {statistics.fmean(rss_values) / 1024.0:.1f}  "
              f"max {max(rss_values) / 1024.0:.1f}")
        growth = (last - first) / 1024.0
        rate = growth / (uptime / 3600.0) if uptime > 0 else 0.0
        print(f"  RSS growth      : {growth:+.1f} MiB over "
              f"{fmt_duration(uptime)}  ({rate:+.1f} MiB/h)")
        print("  (leak canary only — RSS also moves with normal allocator "
              "behaviour)")
    else:
        print("  RSS MiB         : no samples read")

    return {
        "mode": "attach-live",
        "pid": args.pid,
        "cmdline": name,
        "observed_s": uptime,
        "died": died_at is not None,
        "died_after_s": died_at,
        "interrupted": interrupted,
        "sample_interval_s": args.sample_interval,
        "samples": len(samples),
        "rss_first_kb": rss_values[0] if rss_values else None,
        "rss_last_kb": rss_values[-1] if rss_values else None,
        "rss_min_kb": min(rss_values) if rss_values else None,
        "rss_max_kb": max(rss_values) if rss_values else None,
        "rss_mean_kb": statistics.fmean(rss_values) if rss_values else None,
    }


# ─────────────────────────────── aggregation ──────────────────────────────
def print_aggregate(results, args):
    uptimes = sorted(r["uptime_s"] for r in results)
    crashes = [r for r in results if r["crashed"]]
    total_soak = sum(r["uptime_s"] for r in results)
    total_iters = sum(r["iterations"] for r in results)

    print("\n" + "=" * 78)
    print(" AGGREGATE")
    print("=" * 78)
    print(f"  runs                : {len(results)}")
    print(f"  crashes             : {len(crashes)} / {len(results)}   "
          f"({100.0 * len(crashes) / len(results):.1f}%)")
    print(f"  uptime (s)          : min {uptimes[0]:.3f}  "
          f"mean {statistics.fmean(uptimes):.3f}  "
          f"median {statistics.median(uptimes):.3f}  "
          f"p95 {percentile(uptimes, 0.95):.3f}  max {uptimes[-1]:.3f}")
    print(f"  total soak          : {fmt_duration(total_soak)}")
    print(f"  total iterations    : {total_iters:,} dict operations")
    if crashes:
        mtbf = total_soak / len(crashes)
        print(f"  MTBF                : {fmt_duration(mtbf)}  "
              f"(total soak / crashes)")
    else:
        print(f"  MTBF                : > {fmt_duration(total_soak)}  "
              f"(no crash observed — a lower bound, not a proof)")

    tally = {}
    for res in crashes:
        key = f"{res['exc_type']}: {res['exc_msg']}"
        tally[key] = tally.get(key, 0) + 1
    if tally:
        print("  exceptions seen     :")
        for key, count in sorted(tally.items(), key=lambda kv: -kv[1]):
            print(f"      x{count}  {key}")
    else:
        print("  exceptions seen     : none")

    print("\n VERDICT")
    target_hits = sum(1 for r in crashes if r["is_target_race"])
    if args.with_lock:
        if crashes:
            print(f"  FAIL — the lock is supposed to make this impossible, but "
                  f"{len(crashes)}/{len(results)} runs crashed.")
            if target_hits:
                print(f"         {target_hits} of them are the exact "
                      f"{FIX_COMMIT} race. The fix is not holding.")
        else:
            print(f"  PASS — {len(results)} run(s), "
                  f"{fmt_duration(total_soak)} of continuous operation and "
                  f"{total_iters:,} dict")
            print(f"         operations with zero exceptions, at a "
                  f"switch interval of {args.switch_interval} "
                  f"(worst case for this race).")
            print(f"         This is the AFTER figure for {FIX_COMMIT} "
                  f"({FIX_DATE}).")
    else:
        if target_hits:
            print(f"  REPRODUCED — {target_hits}/{len(results)} runs died with "
                  f"'{TARGET_EXC}',")
            print(f"         mean uptime {statistics.fmean(uptimes):.3f} s. "
                  f"This is the BEFORE figure: it is what the")
            print(f"         node did prior to {FIX_COMMIT}. Re-run with "
                  f"--with-lock for the after.")
        elif crashes:
            print(f"  CRASHED, but not with the targeted race — see the "
                  f"exception tally above.")
        else:
            print(f"  NOT REPRODUCED — {len(results)} unlocked run(s) survived. "
                  f"The race is probabilistic;")
            print(f"         raise --duration/--writers or lower "
                  f"--switch-interval and try again.")

    return {
        "runs": len(results),
        "crashes": len(crashes),
        "target_race_crashes": target_hits,
        "crash_rate": len(crashes) / len(results),
        "uptime_min_s": uptimes[0],
        "uptime_mean_s": statistics.fmean(uptimes),
        "uptime_median_s": statistics.median(uptimes),
        "uptime_p95_s": percentile(uptimes, 0.95),
        "uptime_max_s": uptimes[-1],
        "total_soak_s": total_soak,
        "total_iterations": total_iters,
        "mtbf_s": (total_soak / len(crashes)) if crashes else None,
        "mtbf_is_lower_bound": not crashes,
        "exceptions": tally,
    }


def write_json(path, payload):
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
    print(f"\nJSON written: {target}")


# ─────────────────────────────────  main  ─────────────────────────────────
def build_parser():
    parser = argparse.ArgumentParser(
        description="Soak test for the enemies_by_detector race fixed by "
                    f"{FIX_COMMIT} ({FIX_DATE}): continuous uptime until "
                    "crash, with and without the threading.Lock.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="Standard library only. No ROS 2 and no robot needed for the "
               "synthetic modes.")
    lock_group = parser.add_mutually_exclusive_group()
    lock_group.add_argument(
        "--with-lock", dest="with_lock", action="store_true", default=True,
        help="AFTER case: guard every access with threading.Lock, as "
             "TeamComms._state_lock does. Expected to survive.")
    lock_group.add_argument(
        "--no-lock", dest="with_lock", action="store_false",
        help="BEFORE case: no lock. Expected to die with "
             f"'RuntimeError: {TARGET_EXC}'.")
    parser.add_argument("--duration", type=float, default=3600.0, metavar="SEC",
                        help="Max seconds per run. A run ends early on the "
                             "first exception.")
    parser.add_argument("--runs", type=int, default=1, metavar="N",
                        help="Repeat the soak N times -> 'crashes in a series "
                             "of N runs'.")
    parser.add_argument("--writers", type=int, default=2, metavar="N",
                        help="Writer threads (2 mirrors the node: local "
                             "detection + teammate broadcast).")
    parser.add_argument("--keys", type=int, default=256, metavar="N",
                        help="Distinct detector ids in play. More keys = more "
                             "dict resizing = the race fires sooner.")
    parser.add_argument("--stale-timeout", type=float, default=0.05,
                        metavar="SEC",
                        help="Reader's prune horizon. Deliberately far below "
                             f"the node's {ENEMY_MEMORY_TIMEOUT} s so entries "
                             "churn in and out fast.")
    parser.add_argument("--switch-interval", type=float, default=1e-6,
                        metavar="SEC",
                        help="sys.setswitchinterval(). Small values force "
                             "thread switches mid-iteration; the commit notes "
                             "the bug only reproduced this way.")
    parser.add_argument("--write-delay", type=float, default=0.0, metavar="SEC",
                        help="Sleep after each insert. 0 = maximum stress; "
                             "0.02 roughly matches a real detection rate.")
    parser.add_argument("--read-delay", type=float, default=0.0, metavar="SEC",
                        help=f"Sleep after each prune. 0 = maximum stress; "
                             f"{TREE_TIMER_SEC} matches the real tree timer "
                             f"(and will essentially never crash).")
    parser.add_argument("--attach-live", action="store_true",
                        help="Do not run synthetic threads: watch a real "
                             "running node via /proc and time its uptime.")
    parser.add_argument("--pid", type=int, default=None,
                        help="Target pid for --attach-live.")
    parser.add_argument("--sample-interval", type=float, default=5.0,
                        metavar="SEC",
                        help="/proc sampling period for --attach-live.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print every --attach-live sample as it is taken.")
    parser.add_argument("--json", default=None, metavar="PATH",
                        help="Write the full result object to this JSON file.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.duration <= 0:
        die("--duration must be > 0")
    if args.runs < 1:
        die("--runs must be >= 1")
    if args.writers < 1:
        die("--writers must be >= 1")
    if args.keys < 1:
        die("--keys must be >= 1")
    if args.switch_interval <= 0:
        die("--switch-interval must be > 0")

    started_utc = datetime.now(timezone.utc).isoformat(timespec="seconds")

    if args.attach_live:
        live = attach_live(args)
        live["timestamp_utc"] = started_utc
        live["host"] = socket.gethostname()
        if args.json:
            write_json(args.json, {"config": vars(args), "live": live})
        return 0 if not live["died"] else 1

    previous_interval = sys.getswitchinterval()
    sys.setswitchinterval(args.switch_interval)

    print("=" * 78)
    print(" enemies_by_detector race — stability soak")
    print("=" * 78)
    print(f"host            : {socket.gethostname()}  "
          f"({platform.system()} {platform.release()} {platform.machine()})")
    print(f"python          : {platform.python_version()}  "
          f"(pid {os.getpid()})")
    print(f"mode            : synthetic   locking: "
          + ("threading.Lock (post-%s)" % FIX_COMMIT if args.with_lock
             else "NO LOCK (pre-%s)" % FIX_COMMIT))
    print(f"writers         : {args.writers} threads   reader: 1 thread   "
          f"keys: {args.keys}")
    print(f"switch interval : {args.switch_interval} s  "
          f"(was {previous_interval} s)")
    print(f"stale timeout   : {args.stale_timeout} s   "
          f"delays: write {args.write_delay} s / read {args.read_delay} s")
    print(f"duration        : {fmt_duration(args.duration)} per run    "
          f"runs: {args.runs}")

    results = []
    try:
        for index in range(1, args.runs + 1):
            print(f"\n>>> soaking run {index}/{args.runs} "
                  f"(up to {fmt_duration(args.duration)}) ...", flush=True)
            result = run_soak(args, index)
            results.append(result)
            print_run(result, args.runs, args)
    except KeyboardInterrupt:
        print("\n  interrupted by user — reporting the runs completed so far")
        if not results:
            sys.setswitchinterval(previous_interval)
            return 130
    finally:
        sys.setswitchinterval(previous_interval)

    aggregate = print_aggregate(results, args)

    if args.json:
        write_json(args.json, {
            "config": {**vars(args), "timestamp_utc": started_utc,
                       "host": socket.gethostname(),
                       "fix_commit": FIX_COMMIT, "fix_date": FIX_DATE},
            "runs": results,
            "aggregate": aggregate,
        })

    # Exit code: 0 when the outcome is the expected one for the chosen mode.
    if args.with_lock:
        return 0 if aggregate["crashes"] == 0 else 1
    return 0 if aggregate["target_race_crashes"] else 1


if __name__ == "__main__":
    sys.exit(main())
