#!/usr/bin/env python3
"""Build the simulator's AppWalk (model/appwalk.json) from the robot's own walks.

UBTECH's firmware walk (gait_node) is open-loop: the same command sends the same servo frames every time (29 Oct
2026: forward walks within 0.3 deg in the legs). So the simulator can walk the robot's way by playing them back. Each
robot log (robot_dump/robot_logs/robot_<session>.jsonl, robot clock) has every frame gait_node sent; the laptop log
(robot_dump/sim2real_<session>.jsonl) says which walk command started it (walk_cmd rows, else the walking statistics
in robot_dump/walk_stats.jsonl). Per direction (one speed) this keeps: the start of the walk most others start like
(20 ms frames; the start follows the pose the robot walked off from), the cycle that repeats while walking, the
wind-downs recorded after stops, keyed by the point of the cycle where they began, the delay to them (the firmware
stops at the first step boundary at least min_lag frames after the command: the median delay less half a step, as
a stop lands anywhere in a step) and the five 100 ms frames that stand the robot up; plus the TurnLeft / TurnRight /
ResetSwitch motions the mobile app's turns play. A sideways walk logged without its sign (before walk_cmd rows) is
told left from right by replaying it in the simulator.

    python3 tools/extract_appwalk.py            # every session that has a robot log
    python3 tools/extract_appwalk.py SESSION... # given laptop logs
"""
import argparse
import bisect
import collections
import glob
import json
import os
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
os.environ.setdefault("MUJOCO_GL", "egl")
from sim2real_report import load  # noqa: E402
from yanshee_sim import hts  # noqa: E402
from yanshee_sim.robot_stream import hts_copy, to_degrees  # noqa: E402

OUT = os.path.join(ROOT, "model", "appwalk.json")
LEGS = range(6, 16)  # hips, knees, ankles in servo order
TOL = 6  # units (4096 per turn, about 0.5 deg): frames this close in every leg joint are the same frame
GAP_S = 0.4  # gait_node frames further apart than this belong to different walks
SWITCH_DIR = {"walk+": "forward", "walk-": "backward", "side": "side", "diag": "diag"}
TURNS = ("TurnLeft", "TurnRight", "ResetSwitch")


def legdiff(a, b):
    return max(abs(a[j] - b[j]) for j in LEGS)


def direction(v, h):
    if v and h:
        return ("forward" if v > 0 else "backward") + "_" + ("left" if h > 0 else "right")
    return "forward" if v > 0 else "backward" if v < 0 else "left" if h > 0 else "right" if h < 0 else None


def commands(rows, t0, t1):
    """(time, direction or None for a stop, speed, ramp step?) of each walk command, laptop clock. A ramp step
    (the controller's speed ramp, 2 Nov 2026) is neither a walk's start nor its stop."""
    cmd = [(r["t"], direction(r["v"], r["h"]) if not r.get("turn") or r["v"] or r["h"] else "turn",
            max(abs(r["v"]), abs(r["h"])) or None, bool(r.get("ramp") or r.get("down"))) for r in rows if r["kind"] == "walk_cmd"]
    if cmd:
        return cmd
    stats = os.path.join(ROOT, "robot_dump", "walk_stats.jsonl")  # older sessions: the walking statistics
    out = []
    for line in open(stats) if os.path.exists(stats) else []:
        r = json.loads(line)
        if r.get("kind") == "switch" and r.get("method") == "AppWalk" and t0 <= r["t"] <= t1:
            to = r["switch"].split("->")[1]
            out.append((r["t"], SWITCH_DIR.get(to, "turn" if to.startswith("turn") else None), None, False))
    return sorted(set(out))


def walks_of(path):
    rows, robot_rows, robot_path, off, _ = load(path)
    fr = [r for r in robot_rows if r["kind"] == "frame" and r.get("src") == "/gait_node"]
    if not fr:
        return []
    groups, cur = [], [fr[0]]
    for a, b in zip(fr, fr[1:]):
        if b["t"] - a["t"] > GAP_S:
            groups.append(cur)
            cur = []
        cur.append(b)
    groups.append(cur)
    cmd = commands(rows, fr[0]["t"] - 60, fr[-1]["t"] + 60)
    ct = [c[0] for c in cmd]
    out = []
    for g in groups:
        walk = [r["c"] for r in g if r["ms"] == 20]
        stand = [r["c"] for r in g if r["ms"] == 100]
        i = bisect.bisect_right(ct, g[0]["t"] + 0.05) - 1
        if i < 0 or not cmd[i][1] or cmd[i][1] == "turn" or not stand or len(walk) < 30:
            continue  # not a walk started by a command we know, or no stand-up at its end
        if cmd[i][3]:
            continue  # started by a ramp step (speed changing within the walk): not a table's worth
        stop = next((c[0] for c in cmd[i + 1:] if not c[3]), None)  # the next command that is not a ramp step
        out.append({"session": os.path.basename(path)[9:-6], "dir": cmd[i][1], "speed": cmd[i][2], "walk": walk,
                    "stand": stand, "delay_s": g[0]["t"] - cmd[i][0],
                    "stop_k": (stop - g[0]["t"]) / 0.02 if stop else None})
    return out


def steady(frames):
    """(start, end, period): frames[start:end] repeat with the period; end = where the walk's own wind-down begins."""
    best = None
    for P in range(16, 25):
        e = [legdiff(frames[j], frames[j - P]) <= TOL for j in range(P, len(frames))]
        run = (0, 0)
        j = 0
        while j < len(e):
            if e[j]:
                k = j
                while k < len(e) and e[k]:
                    k += 1
                if k - j > run[1] - run[0]:
                    run = (j, k)
                j = k
            else:
                j += 1
        if run[1] - run[0] >= P and (best is None or run[1] - run[0] > best[1] - best[0]):
            best = (run[0], run[1] + P, P)  # e[j] compares frame j + P with frame j
    return best


def sim_moved(frames, stand):
    """cm the simulator moves (right, forward) playing these frames from the reset pose."""
    from yanshee_sim.robot import SimRobot
    sim = SimRobot(viewer=False, realtime=False)
    sim.imu_noise = False
    sim.respawn()
    sim.wait(0.5)
    p0 = sim.data.xpos[sim.model.body("base_link").id].copy()
    for c, ms in [(c, 20) for c in frames] + [(c, 100) for c in stand]:
        sim._send({n: d for n, d in to_degrees(c).items() if n != "NeckLR"}, ms)
        sim.wait(ms / 1000)
    sim.wait(1.0)
    p1 = sim.data.xpos[sim.model.body("base_link").id]
    fell = sim.torso_tilt_deg() > 45
    sim.close()
    return [round(float(p1[0] - p0[0]) * 100, 1), round(float(p1[1] - p0[1]) * 100, 1)], fell


def align(w, cycle, upto):
    """(shift, error): the walk's frames just before frame upto match cycle[(k + shift) % P]; error in units."""
    P = len(cycle)
    ks = range(max(0, upto - P), upto)
    return min(((sh, max(legdiff(w[k], cycle[(k + sh) % P]) for k in ks)) for sh in range(P)), key=lambda x: x[1])


UNLOGGED_SPEED = 2  # walks logged without a speed (before 1 Nov 2026) were driven at the mobile app's speed 2


def build(walks):
    by = collections.defaultdict(list)
    for w in walks:
        by[w["dir"]].append(w)
    table, notes = {}, []
    for d, ws in by.items():
        # one table per speed: the speed walked most under the plain direction name (what the simulator plays by
        # default), the others as direction@speed (2 Nov 2026: speed 5 starts differ from speed 2, and lurch)
        groups = collections.defaultdict(list)
        for w in ws:
            groups[w["speed"] or UNLOGGED_SPEED].append(w)  # the 29 Oct sessions logged no speed: the app's 2
        order = sorted(groups, key=lambda s: (-len(groups[s]), s))
        if len(groups) > 1:
            notes.append(f"{d}: walks at speeds {sorted(groups)}; the plain table is speed {order[0]}")
        for speed in order:
            key = d if speed == order[0] else f"{d}@{speed}"
            entry, more = _direction_table(key, groups[speed])
            notes.extend(more)
            if entry is not None:
                table[key] = entry
    return _signed_side_tables(table, notes)


def _direction_table(d, ws):
    """The table for one direction at one speed from its walks: (entry or None, notes)."""
    notes, table = [], {}
    if True:
        # the start differs with the pose the robot walked off from; the walk the most others start like is kept
        starts = []
        for w in sorted(ws, key=lambda w: -len(w["walk"])):
            v = next((v for v in starts if all(legdiff(a, b) <= TOL for a, b in zip(v[0]["walk"][:12], w["walk"][:12]))), None)
            (v.append(w) if v is not None else starts.append([w]))
        starts.sort(key=len, reverse=True)
        found = next(((v, w, steady(w["walk"])) for v in starts for w in v if steady(w["walk"])), None)
        if found is None:
            notes.append(f"{d}: no walk long enough to find its repeating cycle; skipped")
            return None, notes
        v, base, (s, e, P) = found
        cycle = base["walk"][s:s + P]
        winds, lags = {}, []
        for w in ws:  # each stop: where its walk leaves the cycle, at which point of the cycle, how long after the stop
            if w["stop_k"] is None or w["stop_k"] < s + P:
                continue  # stopped before its walk reached the cycle
            upto = min(int(w["stop_k"]), len(w["walk"]))
            sh, err = align(w["walk"], cycle, upto)
            if err > TOL:
                continue
            at = next((k for k in range(upto, len(w["walk"])) if legdiff(w["walk"][k], cycle[(k + sh) % P]) > TOL), None)
            if at is None:
                continue
            lags.append(at - w["stop_k"])
            ph = (at + sh) % P
            if ph not in winds or len(w["walk"]) - at > len(winds[ph]):
                winds[ph] = w["walk"][at:]
        speeds = sorted({w["speed"] for w in ws if w["speed"]})
        table[d] = {"speed": speeds[0] if len(speeds) == 1 else None, "recorded": len(ws),
                    "start": base["walk"][:s], "cycle": cycle, "min_lag": max(0, round(statistics.median(lags) - P / 4)) if lags else 13,
                    "winddowns": {str(ph): f for ph, f in sorted(winds.items())}, "stand": base["stand"],
                    "start_delay_s": round(statistics.median(w["delay_s"] for w in ws), 2)}
        if len(starts) > 1:
            notes.append(f"{d}: {len(starts)} different starts ({[len(x) for x in starts]} walks; the pose it walked "
                         f"off from); kept the start of {len(v)}")
        notes.append(f"{d}: stops after {sorted(round(x) for x in lags)} frames, leaving the cycle at {sorted(winds)}")
    return table[d], notes


def _signed_side_tables(table, notes):
    for d in [k for k in table if k in ("side", "diag")]:  # logged without a sign: the simulator tells which way
        t = table.pop(d)
        (dx, dy), _ = sim_moved(play(t, 3 * len(t["cycle"])), t["stand"])
        name = ("left" if dx < 0 else "right") if d == "side" else \
            ("forward" if dy > 0 else "backward") + "_" + ("left" if dx < 0 else "right")
        notes.append(f"{d}: the simulator moved {dx:+.1f} cm right, {dy:+.1f} cm forward playing it: {name}")
        table.setdefault(name, t)
    return table, notes


def play(t, n):
    """The frames of a walk n frames long (start, the cycle looped), then the stop the firmware would make."""
    from yanshee_sim.appwalk import AppWalkTable
    return AppWalkTable(t).frames_for(n)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="*")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    paths = args.sessions or sorted(p for p in glob.glob(os.path.join(ROOT, "robot_dump", "sim2real_*.jsonl"))
                                    if os.path.exists(os.path.join(ROOT, "robot_dump", "robot_logs",
                                                                   "robot_" + os.path.basename(p)[9:])))
    walks = [w for p in paths for w in walks_of(p)]
    table, notes = build(walks)
    turns = {}
    for name in TURNS:
        try:
            turns[name] = hts.load(hts_copy(f"/opt/yanshee/lib/move/hts/{name}.hts", None))["frames"]
        except Exception as e:  # noqa: BLE001 - no local copy: the simulator cannot turn the robot's way
            notes.append(f"{name}: no local copy of the robot's motion ({e})")
    out = {"about": "UBTECH's firmware walk as the robot sent it (servo units, 4096 per turn, 65535 = leave alone); "
                    "built by tools/extract_appwalk.py, played by yanshee_sim/appwalk.py",
           "sessions": sorted({w["session"] for w in walks}), "frame_ms": 20, "stand_ms": 100,
           "walks": table, "turns": turns}
    for d, t in table.items():
        n = 5000 // out["frame_ms"]  # a 5 s walk
        moved, fell = sim_moved(play(t, n), t["stand"])
        t["sim_5s_cm"] = moved
        print(f"{d:15s} {t['recorded']} walks, speed {t['speed'] or '?'}, start {len(t['start'])} frames, cycle "
              f"{len(t['cycle'])} frames, stops recorded at cycle points {sorted(int(k) for k in t['winddowns'])}, "
              f"stop {t['min_lag']} frames after the command at least; the simulator walking 5 s: "
              f"{moved[1]:+.1f} cm forward {moved[0]:+.1f} cm right" + (", FELL" if fell else ""))
    for n in notes:
        print("  " + n)
    with open(args.out, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"{args.out}: {len(table)} directions, turns {sorted(turns)}, from {out['sessions']}")


if __name__ == "__main__":
    main()
