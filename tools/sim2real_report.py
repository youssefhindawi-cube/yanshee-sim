#!/usr/bin/env python3
"""Sim-to-real report for one GUI session with the robot connected. Two logs, one session name:

- robot_dump/sim2real_<session>.jsonl  laptop clock: the shadow simulator, the mirror input, activities, clock pings
- robot_dump/robot_logs/robot_<session>.jsonl  robot clock, written on the robot: IMU, encoders, every servo frame
  (ours with the laptop's send time), hal's monitor, firmware log lines. Copied on disconnect, or with
  tools/fetch_robot_logs.py.

The robot's times are moved to the laptop clock with the offset from the quickest clock pings, then per walk (AppWalk
or SwayStep): duration, frames (the firmware's or ours), the robot's tilt (IMU) against the shadow simulator's, who
fell, the delay from a frame to the encoders reaching it and the error left, the shadow's joints against the
encoders; then how late our frames reached the robot over the link, battery, temperature, firmware warnings.

    python3 tools/sim2real_report.py                    # the latest session
    python3 tools/sim2real_report.py FILE [--joints]    # a given laptop log; --joints: tracking error per joint
"""
import argparse
import bisect
import collections
import glob
import json
import math
import os
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.servos import NAMES  # noqa: E402

UPRIGHT_EULER_X = 92.0
LAGS_MS = range(0, 420, 20)


def tilt(ex, ey):
    c = math.cos(math.radians(ex - UPRIGHT_EULER_X)) * math.cos(math.radians(ey))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def latest(times, values, t):
    i = bisect.bisect_right(times, t) - 1
    return values[i] if i >= 0 else None


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def tracking(frames, encs):
    """Delay (ms) from frame to encoders that fits best, the median error at that delay, and per joint."""
    if len(frames) < 10 or len(encs) < 3:
        return None
    ft = [f["t"] for f in frames]
    best = None
    for lag in LAGS_MS:
        errs = collections.defaultdict(list)
        for e in encs:
            f = latest(ft, frames, e["t"] - lag / 1000)
            if f is None:
                continue
            enc = dict(zip(NAMES, e["s"]))
            for n, d in f["angles"].items():
                if n != "NeckLR" and n in enc:  # hal ignores the head on /hal_angles_set
                    errs[n].append(abs(enc[n] - d))
        allerr = [x for v in errs.values() for x in v]
        if allerr and (best is None or statistics.median(allerr) < best[1]):
            best = (lag, statistics.median(allerr), {n: statistics.median(v) for n, v in errs.items()})
    return best


def clock_offset(rows):
    """Robot clock minus laptop clock (s), from the quickest third of the pings."""
    pings = sorted((r["rtt"], r["offset"]) for r in rows if r["kind"] == "clock")
    if not pings:
        return None, []
    best = pings[:max(1, len(pings) // 3)]
    return statistics.median(o for _, o in best), pings


def load(path):
    """Laptop rows plus the robot's rows moved to the laptop clock (kind from the robot's 'k')."""
    rows = [json.loads(line) for line in open(path) if line.strip()]
    link = next((r for r in rows if r["kind"] == "link"), None)
    off, pings = clock_offset(rows)
    robot_path, robot_rows = None, []
    if link and link.get("robot_log"):
        robot_path = os.path.join(os.path.dirname(os.path.abspath(path)), "robot_logs", os.path.basename(link["robot_log"]))
        if os.path.exists(robot_path):
            for line in open(robot_path):
                try:
                    r = json.loads(line)
                except ValueError:
                    continue  # a line cut short when the robot lost power
                r["kind"], r["rt"] = r.pop("k"), r["t"]
                r["t"] = r["t"] - (off or 0.0)
                if r["kind"] == "frame":
                    r["angles"] = {n: u * 360.0 / 4096 for n, u in zip(NAMES, r["c"]) if u != 65535}
                robot_rows.append(r)
    return rows, robot_rows, robot_path, off, pings


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--joints", action="store_true", help="tracking error per joint")
    args = ap.parse_args()
    path = args.file or max(glob.glob(os.path.join(ROOT, "robot_dump", "sim2real_*.jsonl")), key=os.path.getmtime, default=None)
    if not path:
        raise SystemExit("no robot_dump/sim2real_*.jsonl yet: connect the robot in the GUI first")
    rows, robot_rows, robot_path, off, pings = load(path)
    by = collections.defaultdict(list)
    for r in sorted(rows + robot_rows, key=lambda r: r["t"]):
        by[r["kind"]].append(r)
    span = (max(r["t"] for r in rows) - min(r["t"] for r in rows)) / 60
    print(f"{path}\n{len(rows)} laptop records over {span:.1f} min; robot log: "
          + (f"{robot_path} ({len(robot_rows)} records)" if robot_rows else
             f"MISSING ({robot_path or 'no link in this session'}): run tools/fetch_robot_logs.py --ip IP"))
    if pings:
        rtts = [p[0] * 1000 for p in pings]
        print(f"clock: robot - laptop = {off * 1000:+.0f} ms from {len(pings)} pings (round trip median {statistics.median(rtts):.0f} ms, "
              f"95% {pct(rtts, 0.95):.0f} ms, max {max(rtts):.0f} ms)")
    ours = [r for r in by["frame"] if r.get("src") == "ours" and "lt" in r]
    if ours:
        delay = [(r["t"] - r["lt"]) * 1000 for r in ours]
        gaps = [(b["rt"] - a["rt"]) * 1000 for a, b in zip(ours, ours[1:]) if b.get("seq", 0) == a.get("seq", 0) + 1]
        print(f"our frames over the link: {len(ours)}, late by median {statistics.median(delay):.0f} ms, 95% {pct(delay, 0.95):.0f} ms, "
              f"max {max(delay):.0f} ms; gaps between frames on the robot 95% {pct(gaps, 0.95) if gaps else 0:.0f} ms, "
              f"{sum(1 for g in gaps if g > 60)} over 60 ms (20 ms sent)")
    resets = by["shadow_reset"]
    print(f"\n{'walk':>4} {'method':9} {'s':>5} {'frames':>14} {'robot tilt':>10} {'shadow tilt':>11} {'robot':>7} "
          f"{'shadow':>7} {'delay':>6} {'err':>5} {'shadow-enc':>10}")
    end = max(r["t"] for r in rows + robot_rows)
    for i, rs in enumerate(resets):
        t0, t1 = rs["t"], resets[i + 1]["t"] if i + 1 < len(resets) else end
        win = lambda k, a=None, b=None: [r for r in by[k] if (a or t0) <= r["t"] < (b or t1)]  # noqa: E731
        frames = win("frame")
        if frames:
            t1 = min(t1, frames[-1]["t"] + 5.0)  # the walk and 5 s after its last frame
        frames, encs, imus, shadows = win("frame"), win("enc"), win("imu"), win("shadow")
        src = collections.Counter(f.get("src") or "?" for f in frames)
        rt = max((tilt(r["e"][0], r["e"][1]) for r in imus), default=None)
        st = max((r["tilt"] for r in shadows), default=None)
        tr = tracking(frames, encs)
        se, et = [], [e["t"] for e in encs]
        for r in shadows:
            e = latest(et, encs, r["t"])
            if e is not None and r["t"] - e["t"] < 0.3:
                enc = dict(zip(NAMES, e["s"]))
                se += [abs(enc[n] - d) for n, d in r["joints"].items() if n in enc and n != "NeckLR"]
        fsrc = "/".join(f"{'ours' if k == 'ours' else k.strip('/').split('_')[0]} {v}" for k, v in src.items()) or "-"
        print(f"{i + 1:>4} {rs.get('method', '?'):9} {t1 - t0:5.0f} {fsrc:>14} "
              f"{'-' if rt is None else f'{rt:.1f}':>10} {'-' if st is None else f'{st:.1f}':>11} "
              f"{'-' if rt is None else 'FELL' if rt > 50 else 'up':>7} {'FELL' if any(r['fallen'] for r in shadows) else 'up':>7} "
              f"{'-' if not tr else str(tr[0]) + 'ms':>6} {'-' if not tr else f'{tr[1]:.1f}':>5} "
              f"{f'{statistics.median(se):.1f}' if se else '-':>10}")
        if args.joints and tr:
            print("      tracking error per joint (deg): " + ", ".join(f"{n} {v:.1f}" for n, v in sorted(tr[2].items())))
    if not resets:
        print("   (no walks on the robot in this session)")
    hal = by["hal"]
    volts = [r["battery"]["voltage"] for r in hal if isinstance(r.get("battery"), dict) and "voltage" in r["battery"]]
    temps = [r["board"]["iBoardTemperature"] for r in hal if isinstance(r.get("board"), dict) and "iBoardTemperature" in r["board"]]
    if volts and temps:
        charging = sorted({r["battery"].get("charging") for r in hal if isinstance(r.get("battery"), dict)})
        falls = sum(1 for r in hal if (r.get("fall") or {}).get("data"))
        print(f"\nbattery {min(volts)}-{max(volts)} mV (charging flags seen: {charging}); board {min(temps)}-{max(temps)} C; "
              f"hal fall flag set in {falls} of {len(hal)} reads")
    imu = by["imu"]
    if len(imu) > 10:
        gaps = [(b["rt"] - a["rt"]) * 1000 for a, b in zip(imu, imu[1:]) if "rt" in a and "rt" in b]
        if gaps:
            print(f"IMU on the robot: {1000 / statistics.median(gaps):.1f} per second, 95% gap {pct(gaps, 0.95):.0f} ms")
    for r in by["info"][-1:]:
        print("servo calibration offsets: " + ", ".join(f"{n} {o}" for n, o in zip(NAMES, r.get("servo_offsets") or [])))
    warn = collections.Counter((r.get("node"), r.get("msg", "")[:70]) for r in by["rosout"] if r.get("lvl") in ("WARN", "ERROR", "FATAL"))
    if warn:
        print("firmware warnings/errors:")
        for (node, msg), n in warn.most_common(12):
            print(f"  {n:4d} x {node}: {msg}")


if __name__ == "__main__":
    main()
