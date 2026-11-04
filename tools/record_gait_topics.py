#!/usr/bin/env python3
"""Parked task 5: what does the robot's REST server send its walking program? Record, on the robot, the gait
node's input topics while it walks, so the robot link can publish the same thing itself and skip the REST server
(56-77 ms per command, measured 2 Nov 2026).

    python3 tools/record_gait_topics.py --ip IP                       # record 60 s while YOU walk the robot (GUI or app)
    python3 tools/record_gait_topics.py --ip IP --walk 2 --seconds 20 # the tool itself sends one 4 s walk at speed 2
                                                                      # (and its stop) over REST, timed: robot standing,
                                                                      # clear floor, someone ready to catch it
    python3 tools/record_gait_topics.py --ip IP --seconds 15 --quiet  # no walking: just check the recorder runs

On the robot a small ROS node (Python 2) logs every message on /motion_planning_speed (ubt_msgs: speed_v, speed_h),
/teleopkey (geometry_msgs/Twist), the REST server's /MoveAction/goal (what /apollo actually publishes: an actionlib
goal that the /move node turns into the speed topic) and the gait node's own frames on /hal_angles_set, each with the robot's clock and
the publishing node's name, to /home/pi/yanshee_logs/gait_topics_<stamp>.jsonl; the file is copied to
robot_dump/robot_logs/ and summed up: who publishes what, the values, and the delay from a REST command to the
topic message and to the first gait frame.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.rest import YansheeRest  # noqa: E402
from yanshee_sim.robot_stream import ROBOT_LOG_DIR, ROS_ENV, fetch  # noqa: E402

REMOTE_PATH = "/tmp/yanshee_gait_topics.py"
RECORDER = r'''
import sys, json, time, rospy
from ubt_msgs.msg import angles_set, motion_planning_speed, MoveActionGoal, MoveActionResult
from geometry_msgs.msg import Twist
SECONDS, PATH = float(sys.argv[1]), sys.argv[2]
f = open(PATH, "a")
def rec(d):
    d["t"] = time.time()
    f.write(json.dumps(d) + "\n")
    f.flush()
def src(m):
    return (getattr(m, "_connection_header", None) or {}).get("callerid", "")
rospy.init_node("yanshee_gait_topics", anonymous=True, disable_signals=True)
rospy.Subscriber("/motion_planning_speed", motion_planning_speed,
                 lambda m: rec({"k": "speed", "v": m.speed_v, "h": m.speed_h, "src": src(m)}), queue_size=100)
rospy.Subscriber("/teleopkey", Twist, lambda m: rec({"k": "teleop", "lin": [m.linear.x, m.linear.y, m.linear.z],
                 "ang": [m.angular.x, m.angular.y, m.angular.z], "src": src(m)}), queue_size=100)
rospy.Subscriber("/hal_angles_set", angles_set, lambda m: rec({"k": "frame", "ms": m.time, "src": src(m),
                 "c": list(m.angles)}), queue_size=200)
def fields(o, depth=0):
    if depth > 4 or not hasattr(o, "__slots__"):
        return o if isinstance(o, (int, float, str, bool, list)) else str(o)
    return dict((k, fields(getattr(o, k), depth + 1)) for k in o.__slots__)
rospy.Subscriber("/MoveAction/goal", MoveActionGoal, lambda m: rec({"k": "goal", "src": src(m), "goal": fields(m.goal)}), queue_size=50)
rospy.Subscriber("/MoveAction/result", MoveActionResult, lambda m: rec({"k": "result", "src": src(m), "result": fields(m.result),
                 "status": m.status.status}), queue_size=50)
rec({"k": "start", "seconds": SECONDS})
sys.stdout.write("ready\n")
sys.stdout.flush()
time.sleep(SECONDS)
rec({"k": "stop"})
f.close()
rospy.signal_shutdown("done")
'''


def ssh_cmd(args, host):
    return ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10", f"{args.user}@{host}"]


def rosinfo(args, host, env):
    """What ROS says about the gait node's inputs and the REST server node, before anything moves (read-only)."""
    cmd = (ROS_ENV + "for t in /motion_planning_speed /teleopkey; do echo \"== $t\"; rostopic info $t 2>&1 | grep -A3 -E 'Type|Publishers'; done; "
           "echo '== /apollo (the REST server) publications:'; rosnode info /apollo 2>/dev/null | sed -n '/Publications/,/Subscriptions/p' | head -30")
    r = subprocess.run(ssh_cmd(args, host) + [cmd], env=env, capture_output=True, text=True, timeout=60)
    return r.stdout.strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--seconds", type=float, default=60.0, help="recording time (default 60)")
    ap.add_argument("--walk", type=int, metavar="SPEED", help="the tool sends one 4 s walk at this speed over REST, timed")
    ap.add_argument("--quiet", action="store_true", help="no reminder to walk the robot (a passive check)")
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    args = ap.parse_args()
    host = args.ip.split(":")[0]
    env = dict(os.environ, SSHPASS=args.password)
    print(rosinfo(args, host, env), flush=True)
    up = subprocess.run(ssh_cmd(args, host) + [f"cat > {REMOTE_PATH}"], input=RECORDER, env=env, text=True, timeout=30,
                        capture_output=True)
    if up.returncode:
        sys.exit(f"cannot upload the recorder: {up.stderr.strip()[:200]}")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    remote_log = f"{ROBOT_LOG_DIR}/gait_topics_{stamp}.jsonl"
    proc = subprocess.Popen(ssh_cmd(args, host) + [ROS_ENV + f"mkdir -p {ROBOT_LOG_DIR}; exec python2 -u {REMOTE_PATH} {args.seconds} {remote_log}"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    line = proc.stdout.readline()
    if "ready" not in line:
        print(line + proc.stdout.read())
        sys.exit("the recorder did not start on the robot (ROS?)")
    t0 = time.time()
    print(f"recording for {args.seconds:g} s on the robot ({remote_log}) ...", flush=True)
    cmds = []
    if args.walk:
        rest = YansheeRest(args.ip, timeout=6)
        time.sleep(2.0)
        ts = time.time()
        res = rest.gait(args.walk, 0, 0, 1, False)
        cmds.append({"lt": ts, "lt_done": time.time(), "cmd": f"gait v={args.walk}", "code": res.get("code")})
        print(f"  REST walk at speed {args.walk} sent ({(time.time() - ts) * 1000:.0f} ms)", flush=True)
        time.sleep(4.0)
        ts = time.time()
        res = rest.gait(0, 0, 0, 0, False)
        cmds.append({"lt": ts, "lt_done": time.time(), "cmd": "gait stop", "code": res.get("code")})
        print(f"  REST stop sent ({(time.time() - ts) * 1000:.0f} ms)", flush=True)
    elif not args.quiet:
        print("  WALK THE ROBOT NOW from the GUI or the app (a few short walks and turns) ...", flush=True)
    rest_out = proc.stdout.read()
    proc.wait()
    if rest_out.strip():
        print(rest_out.strip()[-400:])
    local = fetch(host, remote_log, os.path.join(ROOT, "robot_dump", "robot_logs"), user=args.user, password=args.password)
    rows = [json.loads(l) for l in open(local) if l.strip()]
    print(f"\ncopied {os.path.relpath(local, ROOT)}: {len(rows)} rows")
    summary(rows, cmds, t0)
    with open(local, "a") as f:
        for c in cmds:
            f.write(json.dumps(dict(c, k="rest_cmd")) + "\n")


def summary(rows, cmds, t0):
    speeds = [r for r in rows if r["k"] == "speed"]
    teleop = [r for r in rows if r["k"] == "teleop"]
    frames = [r for r in rows if r["k"] == "frame"]
    print(f"/motion_planning_speed: {len(speeds)} messages" + (f" from {sorted({r['src'] for r in speeds})}" if speeds else ""))
    for r in speeds[:12]:
        print(f"   +{r['t'] - rows[0]['t']:6.2f} s  speed_v {r['v']:+d} speed_h {r['h']:+d}")
    print(f"/teleopkey: {len(teleop)} messages" + (f" from {sorted({r['src'] for r in teleop})}" if teleop else ""))
    for r in teleop[:6]:
        print(f"   +{r['t'] - rows[0]['t']:6.2f} s  linear {r['lin']} angular {r['ang']}")
    by_src = {}
    for r in frames:
        by_src[r["src"]] = by_src.get(r["src"], 0) + 1
    goals = [r for r in rows if r["k"] == "goal"]
    print(f"/MoveAction/goal (the REST server's own output): {len(goals)} goals" + (f" from {sorted({r['src'] for r in goals})}" if goals else ""))
    for r in goals[:8]:
        print(f"   +{r['t'] - rows[0]['t']:6.2f} s  {json.dumps(r['goal'])[:150]}")
    print(f"/hal_angles_set frames: {by_src or 0}")
    if speeds and frames:  # from a speed message to the gait node's first frame after it
        gaps = []
        for s in speeds:
            if s["v"] or s["h"]:
                nxt = next((f for f in frames if f["t"] > s["t"] and f["src"].endswith("gait_node")), None)
                if nxt:
                    gaps.append((nxt["t"] - s["t"]) * 1000)
        if gaps:
            print(f"speed message -> first gait frame: median {statistics.median(gaps):.0f} ms ({len(gaps)} walks)")
    if cmds and speeds:  # the laptop's REST command to the topic message (clocks differ by the ping offset: rough)
        for c in cmds:
            after = [s for s in speeds if s["t"] >= c["lt"] - 0.5]
            if after:
                print(f"REST '{c['cmd']}' at +{c['lt'] - t0:.2f} s -> first speed message +{after[0]['t'] - rows[0]['t']:.2f} s "
                      f"(robot clock; the REST call itself took {(c['lt_done'] - c['lt']) * 1000:.0f} ms)")


if __name__ == "__main__":
    main()
