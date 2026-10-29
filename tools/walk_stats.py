#!/usr/bin/env python3
"""Compare walking methods from robot_dump/walk_stats.jsonl (written by tools/joystick.py and our gait driver).

    python3 tools/walk_stats.py                 # one row per method
    python3 tools/walk_stats.py --falls         # also every fall with what the robot was doing
"""
import argparse
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim.walkstats import PATH  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=PATH)
    ap.add_argument("--falls", action="store_true")
    args = ap.parse_args()
    if not os.path.exists(args.file):
        raise SystemExit(f"no statistics yet ({args.file})")
    rows = [json.loads(line) for line in open(args.file) if line.strip()]
    by = collections.defaultdict(lambda: {"sessions": set(), "moving": 0.0, "switches": 0, "falls": collections.Counter(),
                                          "after_switch": collections.Counter(), "getups": 0, "getups_ok": 0, "lying": 0})
    for r in rows:
        m = by[r["method"]]
        m["sessions"].add(r["session"])
        if r["kind"] == "moving":
            m["moving"] += r["seconds"]
        elif r["kind"] == "switch":
            m["switches"] += 1
        elif r["kind"] == "fall":
            m["falls"][r["posture"]] += 1
            if r.get("since_switch_s") is not None and r["since_switch_s"] <= 3.0:
                m["after_switch"][r.get("last_switch")] += 1
        elif r["kind"] == "lying":
            m["lying"] += 1
        elif r["kind"] == "getup":
            m["getups"] += 1
            m["getups_ok"] += r["ok"]
    print(f"{'method':14s} {'sessions':>8s} {'walking':>8s} {'switches':>8s} {'falls':>5s} {'/10 min':>7s} "
          f"{'face down':>9s} {'on back':>7s} {'side':>4s} {'get-ups ok':>10s}  falls within 3 s of a switch")
    for name, m in sorted(by.items()):
        n = sum(m["falls"].values())
        rate = n / (m["moving"] / 600) if m["moving"] > 0 else float("nan")
        side = m["falls"]["side_left"] + m["falls"]["side_right"]
        print(f"{name:14s} {len(m['sessions']):8d} {m['moving'] / 60:7.1f}m {m['switches']:8d} {n:5d} {rate:7.2f} "
              f"{m['falls']['face_down']:9d} {m['falls']['on_back']:7d} {side:4d} {m['getups_ok']:4d}/{m['getups']:<5d}  "
              + (", ".join(f"{k} x{v}" for k, v in m["after_switch"].most_common()) or "-"))
    if args.falls:
        for r in rows:
            if r["kind"] == "fall":
                print(f"  {r['session']} {r['method']:12s} {r['posture']:10s} while {r['action']:10s} "
                      f"{r.get('since_switch_s')} s after {r.get('last_switch')}")


if __name__ == "__main__":
    main()
