#!/usr/bin/env python3
"""Automated walking trial through the web GUI in Both mode: every command walks the real robot with AppWalk and
the simulator with SwayStep, side by side in the GUI and the MuJoCo window. Records the robot's IMU and its firmware
servo commands (/hal_angles_set) over ssh, the sim's tilt and falls, and per-scenario numbers; falls also go to the
walking statistics (tools/walk_stats.py). Auto get-up is on.

    python3 scripts/web_gui.py --mode both --ip ROBOT_IP --viewer     # first, in another terminal
    python3 tools/walk_trial.py --ip ROBOT_IP                         # then this (Esc / STOP in the GUI aborts)

Saves robot_dump/walk_trial_<time>.json and prints a summary per scenario.
"""
import argparse
import datetime
import json
import os
import sys
import threading
import time

import requests

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from yanshee_sim import recovery  # noqa: E402
import record_gait  # noqa: E402

# (name, [(seconds, speed_v, speed_h, turn), ...]) with S = the speed; the mobile app walks at 2
SCENARIOS = [("forward", [(5, "S", 0, 0)]), ("backward", [(4, "-S", 0, 0)]),
             ("turn left", [(4, 0, 0, "S")]), ("turn right", [(4, 0, 0, "-S")]),
             ("forward -> turn left", [(3, "S", 0, 0), (3, 0, 0, "S")]),
             ("forward -> backward", [(3, "S", 0, 0), (3, "-S", 0, 0)]),
             ("side left", [(3, 0, "S", 0)]), ("forward fast", [(4, "F", 0, 0)])]


def val(x, s, fast):
    return {"S": s, "-S": -s, "F": fast}.get(x, x) if isinstance(x, str) else x


class Gui:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def post(self, path, body=None):
        return requests.post(self.url + path, json=body or {}, timeout=10).json()

    def state(self):
        return requests.get(self.url + "/api/state", timeout=5).json()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="robot IP (for recording the IMU and servo commands over ssh)")
    ap.add_argument("--gui", default="http://127.0.0.1:8080")
    ap.add_argument("--speed", type=int, default=2)
    ap.add_argument("--fast", type=int, default=3)
    ap.add_argument("--only", help="comma-separated scenario numbers (1-based)")
    ap.add_argument("--rest", type=float, default=4.0, help="seconds between scenarios")
    ap.add_argument("--no-ssh", action="store_true", help="do not record over ssh (fake robot)")
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    args = ap.parse_args()
    gui = Gui(args.gui)
    st = gui.state()
    r = st.get("robot") or {}
    if st.get("mode") != "both" or not r.get("connected"):
        raise SystemExit(f"start the GUI in Both mode with the robot connected first (mode {st.get('mode')}, robot {r.get('connected')})")
    if (r.get("battery") or 0) < 40:
        raise SystemExit(f"robot battery {r.get('battery')} %: charge it to 40 % or more first")
    gui.post("/api/autogetup", {"enable": True})
    picks = [SCENARIOS[int(i) - 1] for i in args.only.split(",")] if args.only else SCENARIOS
    total = sum(sec for _, segs in picks for sec, *_ in segs) + args.rest * len(picks)
    angles, imu = [], []
    if not args.no_ssh:
        host = args.ip.split(":")[0]
        for topic, out in (("/hal_angles_set", angles), ("/hal_gyro_report", imu)):
            threading.Thread(target=record_gait.stream, args=(host, args.user, args.password, topic, total + 60, out),
                             daemon=True).start()
        time.sleep(3.0)
    print(f"trial: {len(picks)} scenarios, about {total:.0f} s of walking and rests; robot battery {r.get('battery')} %")
    results, samples, stop = [], [], threading.Event()

    def watch():  # the GUI's view: robot posture, sim tilt and falls, activity, 5 per second
        while not stop.is_set():
            try:
                s = gui.state()
                samples.append({"t": time.time(), "robot_posture": (s.get("robot") or {}).get("posture"),
                                "sim_tilt": (s.get("sim") or {}).get("tilt_deg"), "sim_fallen": (s.get("sim") or {}).get("fallen"),
                                "activity": (s.get("activity") or {}).get("name")})
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.2)
    threading.Thread(target=watch, daemon=True).start()
    try:
        for n, (name, segs) in enumerate(picks, 1):
            t0 = time.time()
            if (gui.state().get("robot") or {}).get("posture") not in ("upright", None):
                print(f"robot is not upright before '{name}': stopping the trial (stand it up, then rerun with --only)")
                break
            print(f"[{n}/{len(picks)}] {name}", flush=True)
            for sec, v, h, t in segs:
                body = {"speed_v": val(v, args.speed, args.fast), "speed_h": val(h, args.speed, args.fast),
                        "turn": val(t, args.speed, args.fast), "steps": 0, "period": 1, "wave": True, "lease": 1.0}
                end = time.time() + sec
                while time.time() < end:
                    res = gui.post("/api/gait", body)
                    if not res.get("ok"):
                        print(f"   refused: {res.get('error')}", flush=True)
                        break
                    time.sleep(0.3)
            gui.post("/api/gait/stop")
            t1 = time.time()
            time.sleep(args.rest)
            # still down after the rest: wait for auto get-up (up to 45 s)
            down_until = time.time() + 45
            while time.time() < down_until and (gui.state().get("robot") or {}).get("posture") not in ("upright", None):
                time.sleep(1.0)
            win = [s for s in samples if t0 <= s["t"] <= time.time()]
            results.append({"name": name, "t0": t0, "walk_end": t1, "t_end": time.time(),
                            "robot_down": sorted({s["robot_posture"] for s in win if s["robot_posture"] not in ("upright", "tilted", None)}),
                            "sim_fell": any(s["sim_fallen"] for s in win),
                            "sim_max_tilt": max((s["sim_tilt"] or 0) for s in win) if win else None})
            print(f"   robot {'FELL ' + ', '.join(results[-1]['robot_down']) if results[-1]['robot_down'] else 'stayed up'}; "
                  f"sim {'FELL' if results[-1]['sim_fell'] else 'stayed up'} (max tilt {results[-1]['sim_max_tilt']})", flush=True)
    except KeyboardInterrupt:
        print("interrupted: stopping")
    finally:
        gui.post("/api/gait/stop")
        stop.set()
    # the robot's IMU per scenario (from /hal_gyro_report: euler x, y, z are fields 16-18 of the CSV line)
    for res in results:
        rows = [(t, record_gait.numbers(line)) for t, line in imu if not line.startswith("%") and res["t0"] <= t <= res["t_end"]]
        eul = [(n[-3], n[-2], n[-1]) for _, n in rows if len(n) >= 4]
        if eul:
            res["robot_max_tilt"] = round(max(max(abs(abs(x) - 92), abs(y)) for x, y, _ in eul), 1)
            z0, z1 = eul[0][2], eul[-1][2]
            res["robot_heading_change"] = round((z1 - z0 + 180) % 360 - 180, 1)
            res["robot_down_imu"] = sorted({recovery.posture(x, y) for x, y, _ in eul} - {"upright", "tilted"})
        res["servo_commands"] = sum(1 for t, line in angles if not line.startswith("%") and res["t0"] <= t <= res["walk_end"])
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out = os.path.join(ROOT, "robot_dump", f"walk_trial_{stamp}.json")
    json.dump({"speed": args.speed, "results": results, "gui_samples": samples,
               "imu": [(t, line) for t, line in imu], "angles": [(t, line) for t, line in angles]}, open(out, "w"))
    print(f"\n{'scenario':24s} {'robot':>14s} {'robot tilt':>10s} {'heading':>8s} {'sim':>9s} {'sim tilt':>8s} {'servo cmds':>10s}")
    for res in results:
        print(f"{res['name']:24s} {('FELL ' + '/'.join(res['robot_down'])) if res['robot_down'] else 'up':>14s} "
              f"{res.get('robot_max_tilt', '-'):>10} {res.get('robot_heading_change', '-'):>8} "
              f"{'FELL' if res['sim_fell'] else 'up':>9s} {res['sim_max_tilt'] or 0:8.1f} {res['servo_commands']:>10}")
    print(f"saved {out}\nfalls so far per method: python3 tools/walk_stats.py")


if __name__ == "__main__":
    main()
