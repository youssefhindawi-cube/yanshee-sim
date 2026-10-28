#!/usr/bin/env python3
"""Tune SimGait: sweep its tuning constants in headless simulations and report the fastest walk that stays up.

    python3 tools/tune_gait.py              # stage 1 coarse grid at full speed, stage 2 finalists across scenarios,
                                            # stage 3 a finer grid around the winner
    python3 tools/tune_gait.py --quick      # a smaller grid (a minute instead of several)
    python3 tools/tune_gait.py --check      # evaluate the current SimGait defaults across the scenario set
    python3 tools/tune_gait.py --jobs 8     # parallel simulations (default: CPUs - 2)

A run walks a scenario (speed_v, speed_h, period, steps, wave) and measures the speed along the intended
direction, the sideways drift, the heading change and the largest torso tilt. A run passes when the walk
finishes (no tilt over MAX_TILT) and keeps its heading. Results go to --out (default: tools/gait_tuning.json);
copy the winner's constants into SimGait by hand (its class attributes) and re-run --check.
"""
import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import sys
import threading
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("YANSHEE_SIM_HEADLESS", "1")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

MAX_TILT = 15.0  # deg: a walk that tilts more than this fails
TILT_MARGIN = 1.5  # deg: stage 1 and 3 candidates must stay this far under MAX_TILT (robustness)
MAX_YAW = 25.0  # deg of heading change over a run that still passes
STEPS = 16
# scenarios every finalist must pass: (speed_v, speed_h, period, wave)
SCENARIOS = [(5, 0, 1, False), (3, 0, 1, False), (1, 0, 1, False), (-3, 0, 1, False), (0, 3, 1, False),
             (0, -3, 1, False), (5, 0, 3, False), (5, 0, 5, False), (3, 3, 1, False), (5, 0, 1, True),
             (0, 0, 1, False), (0, 0, 1, False, 3), (0, 0, 1, False, -3)]  # a 5th value is `turn`
HARD = [(5, 0, 1, False), (-3, 0, 1, False), (3, 3, 1, False), (5, 0, 5, False)]  # stage 1 filter
FULL = {"cycle": [0.8, 1.0], "sway": [6, 8, 10], "lift": [14, 18], "stride": [4, 5, 6], "crouch": [10, 16],
        "d_roll": [0.02, 0.04, 0.06], "k_pitch": [0, 0.5], "side": [1, 1.5], "back": [0.5, 0.8], "diag": [0.5, 1],
        "cycle_per_period": [0.05], "max_deg_s": [300]}
QUICK = {"cycle": [0.8, 1.0], "sway": [8], "lift": [18], "stride": [4, 5], "crouch": [16], "d_roll": [0.03, 0.05],
         "k_pitch": [0, 0.5], "side": [1], "back": [0.5, 0.8], "diag": [0.5, 1], "cycle_per_period": [0.05]}

_bot = None


def _init():
    global _bot
    from yanshee_sim.robot import SimRobot
    _bot = SimRobot(viewer=False, realtime=False)


def evaluate(job):
    """job = (tuning dict, scenario tuple) -> result dict."""
    from yanshee_sim.sim_extras import SimGait
    tuning, scenario = job
    speed_v, speed_h, period, wave = scenario[:4]
    turn = scenario[4] if len(scenario) > 4 else 0
    bot = _bot
    bot.respawn()
    bot.wait(0.5)
    tilt = [0.0]
    wait = bot.wait

    def watched(seconds):
        wait(seconds)
        tilt[0] = max(tilt[0], bot.torso_tilt_deg())
    bot.wait = watched
    try:
        gait = SimGait(bot, max_tilt_deg=MAX_TILT, **tuning)
        p0, t0 = bot.data.qpos[:2].copy(), bot.data.time
        status = gait.run({"speed_v": speed_v, "speed_h": speed_h, "period": period, "steps": STEPS, "wave": wave,
                           "turn": turn}, threading.Event())
        wait(0.5)
        moved, dt = bot.data.qpos[:2] - p0, bot.data.time - t0
        w, x, y, z = bot.data.qpos[3:7]
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    finally:
        bot.wait = wait
    # intended direction: +y forward, -x to the robot's left
    want = [-speed_h, speed_v]
    norm = math.hypot(*want)
    if norm:
        along = (moved[0] * want[0] + moved[1] * want[1]) / norm
        across = abs(-moved[0] * want[1] + moved[1] * want[0]) / norm
    else:
        along, across = 0.0, math.hypot(*moved)
    if turn:  # turning in place: it must turn the right way, not keep its heading
        ok = status == "done" and tilt[0] < MAX_TILT and yaw * turn > 5
    else:
        ok = status == "done" and tilt[0] < MAX_TILT and abs(yaw) < MAX_YAW
    return {"tuning": tuning, "scenario": list(scenario), "ok": ok, "status": status,
            "speed_cm_s": round(100 * along / dt, 2), "drift_cm_s": round(100 * across / dt, 2),
            "yaw_deg": round(yaw, 1), "max_tilt_deg": round(tilt[0], 1), "sim_s": round(dt, 1)}


def grid(space):
    keys = list(space)
    return [dict(zip(keys, vals)) for vals in itertools.product(*(space[k] for k in keys))]


def run_jobs(pool, jobs, label):
    t0, out = time.time(), []
    for i, r in enumerate(pool.imap_unordered(evaluate, jobs, chunksize=2), 1):
        out.append(r)
        if i % 100 == 0 or i == len(jobs):
            print(f"  {label}: {i}/{len(jobs)} runs, {time.time() - t0:.0f} s", flush=True)
    return out


def score(results, margin=0.0):
    """Per tuning: passes every scenario (tilting at most MAX_TILT - margin)? Rank by the full-speed forward
    speed minus drift."""
    by = {}
    for r in results:
        by.setdefault(json.dumps(r["tuning"], sort_keys=True), []).append(r)
    rows = []
    for key, rs in by.items():
        main = [r for r in rs if r["scenario"][:3] == [5, 0, 1] and not r["scenario"][3]]
        fwd = main[0] if main else max(rs, key=lambda r: r["speed_cm_s"])
        tilt = max(r["max_tilt_deg"] for r in rs)
        passes = all(r["ok"] for r in rs) and tilt <= MAX_TILT - margin
        rows.append({"tuning": json.loads(key), "passes": passes, "n": len(rs),
                     "failed": [r["scenario"] for r in rs if not r["ok"]],
                     "speed_cm_s": fwd["speed_cm_s"], "drift_cm_s": fwd["drift_cm_s"], "max_tilt_deg": tilt,
                     "score": fwd["speed_cm_s"] - 0.5 * fwd["drift_cm_s"] if passes else -1})
    return sorted(rows, key=lambda r: -r["score"])


def show(rows, n=10):
    for r in rows[:n]:
        t = " ".join(f"{k}={v:g}" for k, v in r["tuning"].items())
        print(f"  {'PASS' if r['passes'] else 'fail'} {r['speed_cm_s']:5.1f} cm/s drift {r['drift_cm_s']:4.1f} "
              f"tilt {r['max_tilt_deg']:4.1f}  {t}" + (f"  failed {r['failed']}" if r["failed"] else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--check", action="store_true", help="only evaluate the current defaults")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--finalists", type=int, default=12)
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "gait_tuning.json"))
    args = ap.parse_args(argv)
    from yanshee_sim.sim_extras import SimGait
    defaults = {k: getattr(SimGait, k.upper()) for k in SimGait.TUNABLE}

    with mp.Pool(args.jobs, initializer=_init) as pool:
        if args.check:
            res = run_jobs(pool, [(defaults, s) for s in SCENARIOS], "check")
            for r in sorted(res, key=lambda r: r["scenario"]):
                print(f"  {'ok  ' if r['ok'] else 'FAIL'} {r['scenario']}: {r['speed_cm_s']:5.1f} cm/s, drift "
                      f"{r['drift_cm_s']:.1f}, yaw {r['yaw_deg']:+.0f}, tilt {r['max_tilt_deg']:.1f} ({r['status']})")
            return 0 if all(r["ok"] for r in res) else 1

        space = QUICK if args.quick else FULL
        cands = [dict(defaults, **g) for g in grid(space)]
        print(f"stage 1: {len(cands)} settings x {len(HARD)} hard scenarios ({args.jobs} jobs)")
        s1 = score(run_jobs(pool, [(c, s) for c in cands for s in HARD], "stage 1"), TILT_MARGIN)
        show(s1)
        finalists = [r["tuning"] for r in s1 if r["passes"]][:args.finalists]
        print(f"stage 2: {len(finalists)} finalists x {len(SCENARIOS)} scenarios")
        s2 = score(run_jobs(pool, [(c, s) for c in finalists for s in SCENARIOS], "stage 2"))
        show(s2)
        best = next((r["tuning"] for r in s2 if r["passes"]), None)
        if best is None:
            print("no finalist passed every scenario; loosen the grid or the pass criteria")
            json.dump({"stage1": s1, "stage2": s2}, open(args.out, "w"), indent=1)
            return 1
        fine = {"cycle": sorted({best["cycle"] - 0.1, best["cycle"], best["cycle"] + 0.1}),
                "stride": sorted({best["stride"] - 0.5, best["stride"], best["stride"] + 0.5}),
                "sway": sorted({best["sway"] - 1, best["sway"], best["sway"] + 1}),
                "d_roll": sorted({best["d_roll"] - 0.01, best["d_roll"], best["d_roll"] + 0.01}),
                "k_pitch": [0, 0.3, 0.6], "d_pitch": [0, 0.03]}
        cands = [dict(best, **g) for g in grid(fine)]
        print(f"stage 3: {len(cands)} settings around the winner x {len(SCENARIOS)} scenarios")
        s3 = score(run_jobs(pool, [(c, s) for c in cands for s in SCENARIOS], "stage 3"), TILT_MARGIN)
        show(s3)
    winner = next((r for r in s3 if r["passes"]), next(r for r in s2 if r["passes"]))
    json.dump({"winner": winner, "stage3": s3[:40], "stage2": s2, "stage1": s1[:60], "scenarios": SCENARIOS,
               "max_tilt": MAX_TILT, "max_yaw": MAX_YAW, "steps": STEPS}, open(args.out, "w"), indent=1)
    print(f"winner ({winner['speed_cm_s']} cm/s): " + " ".join(f"{k}={v:g}" for k, v in winner["tuning"].items()))
    print(f"results written to {args.out}; copy the winner into SimGait's class attributes and run --check")
    return 0


if __name__ == "__main__":
    sys.exit(main())
