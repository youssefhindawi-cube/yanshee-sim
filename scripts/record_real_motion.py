#!/usr/bin/env python3
"""Record a built-in motion from the real robot into a motion file the simulator can replay.

The robot plays the motion while this script polls all 17 servo angles; samples become keyframes.

    python3 scripts/record_real_motion.py --ip 192.168.3.13 --name wave --direction both
    python3 scripts/record_real_motion.py --ip 192.168.3.13 --name WakaWaka --hz 4 --max-seconds 90

Output: motions/recorded/<name>[_<direction>].json  (yanapi_sim plays these for start_play_motion).
Fidelity is limited by the polling rate (the REST read takes tens of ms); fast dances will be smoothed.
"""
import argparse
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import NAMES, RealRobot, motion  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--name", required=True, help="built-in motion name, e.g. wave, bow, raise, WakaWaka")
    ap.add_argument("--direction", default="")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--max-seconds", type=float, default=30.0)
    args = ap.parse_args()

    bot = RealRobot(args.ip)
    api = bot.api
    api.sync_play_motion(name="reset")
    samples, stop = [], threading.Event()

    def poll():
        t0 = time.time()
        while not stop.is_set() and time.time() - t0 < args.max_seconds:
            res = api.get_servos_angles(NAMES)
            if res.get("code") == 0:
                samples.append((time.time() - t0, res["data"]))
            time.sleep(max(0.0, 1 / args.hz - 0.01))

    th = threading.Thread(target=poll, daemon=True)
    th.start()
    api.sync_play_motion(name=args.name, direction=args.direction)  # blocks until the robot reports idle
    time.sleep(0.5)
    stop.set()
    th.join()

    frames, prev_t = [], 0.0
    for t, angles in samples[1:]:
        frames.append({"angles": {n: angles[n] for n in NAMES},
                       "runtime": int(min(max((t - prev_t) * 1000, 200), 4000))})
        prev_t = t
    name = args.name + (f"_{args.direction}" if args.direction else "")
    out = os.path.join(ROOT, "motions", "recorded", name + ".json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    motion.save({"name": name, "description": f"Recorded from the real robot at {args.hz} Hz",
                 "frames": frames, "end_in_reset": True}, out)
    print(f"{len(frames)} keyframes over {prev_t:.1f} s -> {out}")


if __name__ == "__main__":
    main()
