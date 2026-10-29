#!/usr/bin/env python3
"""Record the firmware walk exactly: every servo command its gait_node publishes on /hal_angles_set (all 17 angles,
at its own rate) and the IMU, read over ssh on the robot. Optionally starts the walk itself for a repeatable record.

    python3 tools/record_gait.py --ip IP --seconds 20                 # passive: drive with the joystick or the app
    python3 tools/record_gait.py --ip IP --walk 2,0 --seconds 6       # forward at speed 2 for 6 s, then the app's stop

Saves robot_dump/gait_record_<time>.json (messages and IMU) and motions/recorded/firmware_walk_<time>.json (the same
angles as a motion, one frame per message) that the simulator can replay for comparison with our gait.
"""
import argparse
import datetime
import json
import os
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.rest import YansheeRest  # noqa: E402
from yanshee_sim.servos import NAMES  # noqa: E402

ROS = "source /opt/ros/kinetic/setup.bash; source /opt/yanshee/setup.bash 2>/dev/null; "


def stream(host, user, password, topic, seconds, out):
    """Collect `rostopic echo -p TOPIC` lines (CSV) for `seconds` into out (list of (laptop time, line))."""
    cmd = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{host}",
           ROS + f"timeout {seconds + 2} rostopic echo -p {topic}"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
                         env=dict(os.environ, SSHPASS=password))
    for line in p.stdout:
        out.append((time.time(), line.rstrip("\n")))
    p.wait()


def numbers(line):
    return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?(?:e-?\d+)?", line)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--walk", help="speed_v,speed_h: start the firmware walk for the recording (period 1, arm swing)")
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    args = ap.parse_args()
    host = args.ip.split(":")[0]
    angles, imu = [], []
    threads = [threading.Thread(target=stream, args=(host, args.user, args.password, "/hal_angles_set", args.seconds, angles)),
               threading.Thread(target=stream, args=(host, args.user, args.password, "/hal_gyro_report", args.seconds, imu))]
    for t in threads:
        t.start()
    time.sleep(2.5)  # ssh and rostopic start-up
    rest = YansheeRest(args.ip, timeout=4.0)
    if args.walk:
        v, h = (int(x) for x in args.walk.split(","))
        print(f"walking v={v} h={h} for {args.seconds - 3:.0f} s: {rest.gait(v, h, 0, 1, True).get('msg')}", flush=True)
        time.sleep(max(1.0, args.seconds - 3))
        print(f"stop (all zeros): {rest.gait(0, 0, 0, 0, True).get('msg')}", flush=True)
    else:
        print(f"recording {args.seconds:.0f} s: drive the robot now", flush=True)
    for t in threads:
        t.join(args.seconds + 15)

    msgs = []
    for t, line in angles:
        if line.startswith("%") or not line.strip():
            continue
        n = numbers(line)
        if len(n) >= 19:  # %time, 17 angles, time
            msgs.append({"t": round(t, 3), "stamp": n[0], "angles": [int(x) for x in n[1:18]], "time": int(n[18])})
    gyro = []
    for t, line in imu:
        if not line.startswith("%") and line.strip():
            gyro.append({"t": round(t, 3), "values": numbers(line)[1:]})
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    os.makedirs(os.path.join(ROOT, "robot_dump"), exist_ok=True)
    raw = os.path.join(ROOT, "robot_dump", f"gait_record_{stamp}.json")
    json.dump({"walk": args.walk, "seconds": args.seconds, "messages": msgs, "imu": gyro,
               "imu_header": next((line for _, line in imu if line.startswith("%")), None),
               "angles_header": next((line for _, line in angles if line.startswith("%")), None)}, open(raw, "w"))
    print(f"{len(msgs)} servo commands, {len(gyro)} IMU samples -> {raw}")
    if len(msgs) >= 2:
        dts = [b["stamp"] - a["stamp"] for a, b in zip(msgs, msgs[1:])]
        dts = sorted(d / 1e9 if d > 1e6 else d for d in dts)
        print(f"  command rate: median {1 / dts[len(dts) // 2]:.1f} per second; 'time' field values {sorted({m['time'] for m in msgs})[:6]}")
        print(f"  first angles {dict(zip(NAMES, msgs[0]['angles']))}")
        frames = [{"angles": dict(zip(NAMES, m["angles"])), "runtime": max(20, m["time"]), "hold": 0} for m in msgs]
        os.makedirs(os.path.join(ROOT, "motions", "recorded"), exist_ok=True)
        mo = os.path.join(ROOT, "motions", "recorded", f"firmware_walk_{stamp}.json")
        json.dump({"name": f"firmware_walk_{stamp}", "description": f"The firmware walk as published on /hal_angles_set "
                   f"(walk {args.walk or 'driven by hand'}), one frame per message.", "frames": frames}, open(mo, "w"), indent=1)
        print(f"  as a motion: {mo}")


if __name__ == "__main__":
    main()
