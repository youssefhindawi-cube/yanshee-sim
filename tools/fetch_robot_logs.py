#!/usr/bin/env python3
"""Copy the robot's own logs (written on the robot by the robot link, robot clock: /home/pi/yanshee_logs/robot_*.jsonl)
to robot_dump/robot_logs/. The GUI copies a session's log when it disconnects; use this when it could not (GUI closed,
robot switched off first).

    python3 tools/fetch_robot_logs.py --ip IP                  # copy the logs this laptop does not have yet
    python3 tools/fetch_robot_logs.py --ip IP --delete-copied  # then delete, on the robot, the ones copied intact
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.robot_stream import ROBOT_LOG_DIR, fetch  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    ap.add_argument("--delete-copied", action="store_true", help="delete on the robot the logs now copied intact")
    args = ap.parse_args()
    host, env = args.ip.split(":")[0], dict(os.environ, SSHPASS=args.password)
    ssh = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", f"{args.user}@{host}"]
    ls = subprocess.run(ssh + [f"ls -l --time-style=+%s {ROBOT_LOG_DIR}/*.jsonl 2>/dev/null"], env=env, capture_output=True,
                        text=True, timeout=30)
    files = {}
    for line in ls.stdout.splitlines():
        p = line.split()
        if len(p) >= 7:
            files[p[-1]] = int(p[4])
    dest = os.path.join(ROOT, "robot_dump", "robot_logs")
    print(f"{len(files)} logs on the robot ({sum(files.values()) / 1e6:.1f} MB)")
    copied = []
    for remote, size in sorted(files.items()):
        local = os.path.join(dest, os.path.basename(remote))
        if os.path.exists(local) and os.path.getsize(local) == size:
            copied.append(remote)
            continue
        path = fetch(host, remote, dest, user=args.user, env=env)
        ok = os.path.getsize(path) == size
        print(f"  {os.path.basename(remote)}: {size / 1e6:.2f} MB {'copied' if ok else 'copied, size differs (still being written?)'}")
        if ok:
            copied.append(remote)
    running = subprocess.run(ssh + ["ps -eo args | grep '[y]anshee_sway_stream.py'"], env=env, capture_output=True,
                             text=True, timeout=30).stdout.split()
    active = {f"{ROBOT_LOG_DIR}/robot_{a}.jsonl" for a in running}  # a link still writing: never delete its log
    copied = [c for c in copied if c not in active]
    if args.delete_copied and copied:
        subprocess.run(ssh + ["rm -f " + " ".join(copied)], env=env, timeout=30, check=True)
        print(f"deleted {len(copied)} copied logs on the robot")
    print(f"local copies: {os.path.relpath(dest, ROOT)}/")


if __name__ == "__main__":
    main()
