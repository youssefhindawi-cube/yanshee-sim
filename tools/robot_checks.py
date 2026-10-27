#!/usr/bin/env python3
"""Measurements to take on the real Yanshee that the simulator still lacks (docs/ROBOT_SESSION_PLAN.md).

    python3 tools/robot_checks.py latency --ip IP            # REST round trips (battery, angles, gyro)
    python3 tools/robot_checks.py imu --ip IP                # IMU conventions: upright, leaning, lying (interactive)
    python3 tools/robot_checks.py servo --ip IP              # step response of one arm servo (speed, lag, readback rate)
    python3 tools/robot_checks.py gait --ip IP               # seconds per step for the built-in gait; distance by tape
    python3 tools/robot_checks.py inspect --ip IP            # read-only ssh dump of the Raspberry Pi (+ its .hts files)

Every mode writes its numbers to robot_dump/<mode>_<time>.json (or robot_dump/inspect_<time>/) so they can be
turned into simulator constants later. Nothing is written to the robot: inspect only reads and copies files.
"""
import argparse
import datetime
import json
import os
import statistics
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.rest import YansheeRest  # noqa: E402
from yanshee_sim.servos import NAMES, RESET_POSE  # noqa: E402

ARM_SERVOS = NAMES[:6] + ["NeckLR"]
OUT_DIR = os.path.join(ROOT, "robot_dump")


def _stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _save(mode, payload):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"{mode}_{_stamp()}.json")
    with open(path, "w") as f:
        json.dump(payload, f, indent=1)
    print(f"saved {path}")
    return path


def _ok(res, what):
    if not isinstance(res, dict) or res.get("code") != 0:
        raise SystemExit(f"{what}: robot answered {res}")
    return res.get("data")


def _confirm(text, yes):
    if not yes:
        input(text + " Press Enter to go on (Ctrl+C aborts). ")


def _timed(fn, n):
    """n calls of fn -> list of round-trip ms."""
    out = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        out.append((time.perf_counter() - t0) * 1000)
    return out


# -- latency ---------------------------------------------------------------------------------------
def latency(args):
    rest = YansheeRest(args.ip, timeout=args.timeout)
    rows = {"battery": _timed(lambda: _ok(rest.battery(), "battery"), args.n),
            "angles(17)": _timed(lambda: _ok(rest.get_angles(NAMES), "angles"), args.n),
            "angles(1)": _timed(lambda: _ok(rest.get_angles(["NeckLR"]), "angle"), args.n),
            "gyro": _timed(lambda: _ok(rest.gyro(), "gyro"), args.n)}
    for k, v in rows.items():
        print(f"  {k:11s} min {min(v):6.1f} ms  median {statistics.median(v):6.1f} ms  max {max(v):6.1f} ms")
    _save("latency", {"ip": args.ip, "n": args.n, "ms": rows})


# -- imu ---------------------------------------------------------------------------------------------
POSTURES = [("upright", "Stand the robot upright on its feet, still."),
            ("back20", "Hold it leaning about 20 degrees BACKWARD (feet on the floor, head back)."),
            ("right20", "Hold it leaning about 20 degrees to ITS RIGHT."),
            ("face_down", "Lay it FACE DOWN on the floor."),
            ("on_back", "Lay it on its BACK.")]
KEYS = ["euler-x", "euler-y", "euler-z", "accel-x", "accel-y", "accel-z", "gyro-x", "gyro-y", "gyro-z"]


def _sample_gyro(rest, seconds):
    rows, t0, calls = [], time.perf_counter(), 0
    while time.perf_counter() - t0 < seconds:
        g = (_ok(rest.gyro(), "gyro") or {}).get("gyro") or [{}]
        rows.append(g[0])
        calls += 1
    distinct = len({tuple(r.get(k) for k in KEYS) for r in rows})
    mean = {k: statistics.fmean(float(r[k]) for r in rows if k in r) for k in KEYS if any(k in r for r in rows)}
    return mean, calls / seconds, distinct / seconds


def imu(args):
    rest = YansheeRest(args.ip, timeout=args.timeout)
    results = {}
    postures = POSTURES[:1] if args.auto else POSTURES
    for key, text in postures:
        _confirm(text, args.auto)
        mean, rate, fresh = _sample_gyro(rest, args.seconds)
        results[key] = {"mean": mean, "reads_per_s": round(rate, 1), "distinct_samples_per_s": round(fresh, 1)}
        print(f"  {key:9s} " + "  ".join(f"{k} {mean[k]:+8.2f}" for k in KEYS if k in mean))
        print(f"            {rate:.1f} reads/s, {fresh:.1f} distinct samples/s")
    up = results["upright"]["mean"]
    notes = [f"upright euler-x {up.get('euler-x', float('nan')):+.1f}: the sim assumes +90 (IMU_UPRIGHT_EULER_X in robot.py)",
             f"upright accel: x {up.get('accel-x', 0):+.2f} y {up.get('accel-y', 0):+.2f} z {up.get('accel-z', 0):+.2f} "
             "(the sim assumes about -1 on z, in g)"]
    if "back20" in results:
        d = results["back20"]["mean"].get("euler-x", 0) - up.get("euler-x", 0)
        notes.append(f"leaning back changes euler-x by {d:+.1f}: the sim raises it (UBTECH's balance sample too)")
    if "right20" in results:
        d = results["right20"]["mean"].get("euler-y", 0) - up.get("euler-y", 0)
        notes.append(f"leaning right changes euler-y by {d:+.1f}: the sim makes it positive")
    for key, expect in (("face_down", "about 0"), ("on_back", "about +-180")):
        if key in results:
            notes.append(f"{key} euler-x {results[key]['mean'].get('euler-x', 0):+.1f} (course: {expect})")
    print("\n".join("  " + n for n in notes))
    _save("imu", {"ip": args.ip, "seconds": args.seconds, "postures": results, "notes": notes})


# -- servo -------------------------------------------------------------------------------------------
def servo(args):
    if args.servo not in ARM_SERVOS and not args.legs:
        raise SystemExit(f"{args.servo} is a leg servo; the test moves one arm or neck servo (use --legs to override)")
    rest = YansheeRest(args.ip, timeout=args.timeout)
    if _ok(rest.battery(), "battery").get("percent", 0) < 30:
        raise SystemExit("battery under 30%: the robot refuses motions")
    _confirm(f"The {args.servo} servo will move from {args.start} to {args.end} deg and back. Keep the arm clear.",
             args.yes)
    _ok(rest.set_angles({args.servo: args.start}, 1000), "set start")
    time.sleep(1.3)
    log, t0 = [], time.perf_counter()
    t_cmd = time.perf_counter()
    _ok(rest.set_angles({args.servo: args.end}, args.runtime), "step")
    t_sent = time.perf_counter()
    while time.perf_counter() - t0 < args.seconds:
        a = _ok(rest.get_angles([args.servo]), "read").get(args.servo)
        log.append((round(time.perf_counter() - t_cmd, 4), a))
    _ok(rest.set_angles({args.servo: args.start}, 1000), "back")
    moved = [t for t, a in log if a is not None and abs(a - args.start) >= 2]
    near = [t for t, a in log if a is not None and abs(a - args.end) <= 3]
    reads = len(log) / args.seconds
    distinct = len({a for _, a in log})
    report = {"servo": args.servo, "start": args.start, "end": args.end, "runtime_ms": args.runtime,
              "set_call_ms": round((t_sent - t_cmd) * 1000, 1), "reads_per_s": round(reads, 1),
              "first_change_s": moved[0] if moved else None, "within_3deg_s": near[0] if near else None,
              "distinct_readings": distinct, "log": log}
    if moved and near:
        span = abs(args.end - args.start)
        report["mean_deg_s"] = round(span / max(near[0] - moved[0], 1e-3), 1)
        print(f"  set_servos_angles call took {report['set_call_ms']} ms; first change after {moved[0]:.3f} s; "
              f"within 3 deg after {near[0]:.3f} s -> about {report['mean_deg_s']} deg/s over {span} deg "
              f"(commanded {span / (args.runtime / 1000):.0f} deg/s)")
    else:
        print(f"  no movement seen in the readback: {log[:5]} ...")
    print(f"  readback: {reads:.1f} reads/s, {distinct} distinct values; the sim uses kp 8, slew 400 deg/s")
    _save("servo", report)


# -- gait --------------------------------------------------------------------------------------------
def gait(args):
    rest = YansheeRest(args.ip, timeout=args.timeout)
    if _ok(rest.battery(), "battery").get("percent", 0) < 30:
        raise SystemExit("battery under 30%: the robot refuses motions")
    settings = [tuple(int(x) for x in s.split(",")) for s in args.settings.split()]
    _confirm("The robot will walk. Put it on the floor with a clear 1 m ahead, mark its start position, "
             "and keep a hand near the red button on its head.", args.yes)
    rows = []
    if args.stop_test:
        # does a gentle stop (speeds stepped down, then exit) keep the robot up where a sudden exit would not?
        for label, gentle in (("gentle stop (speeds stepped down, then exit)", True), ("sudden exit_motion_gait", False)):
            _confirm(f"Stop test at speed_v {args.stop_speed}: 3 s of walking, then a {label}. Be ready to catch it.",
                     args.yes)
            _ok(rest.gait(speed_v=args.stop_speed, speed_h=0, steps=0, period=1), "gait")
            time.sleep(3.0)
            t0 = time.perf_counter()
            if gentle:
                rest.gait_decelerate(args.stop_speed, 0, 1)
            else:
                rest.gait_exit()
            seconds = time.perf_counter() - t0
            stayed = None if args.yes else input(f"  {label} took {seconds:.1f} s. Did it stay up? [y/n] ").lower().startswith("y")
            rows.append({"stop_test": label, "speed_v": args.stop_speed, "stop_seconds": round(seconds, 2), "stayed_up": stayed})
            time.sleep(2.0)
    try:
        for speed_v, period in settings:
            _confirm(f"Next: speed_v {speed_v}, period {period}, {args.steps} steps. Mark the start.", args.yes)
            t0 = time.perf_counter()
            _ok(rest.gait(speed_v=speed_v, speed_h=0, steps=args.steps, period=period), "gait")
            status, deadline = None, time.perf_counter() + args.steps * 4 + 10
            while time.perf_counter() < deadline:
                res = rest.gait_state()
                status = (res.get("data") or {}).get("status") if res.get("code") == 0 else None
                if status in (8, "8", 7, "7", 5, "5"):
                    break
                time.sleep(0.1)
            seconds = time.perf_counter() - t0
            _ok(rest.gait_exit(), "gait exit")
            cm = None
            if not args.yes:
                text = input(f"  took {seconds:.1f} s (final state {status}). Distance walked in cm (Enter to skip): ")
                cm = float(text) if text.strip() else None
            row = {"speed_v": speed_v, "period": period, "steps": args.steps, "seconds": round(seconds, 2),
                   "s_per_step": round(seconds / args.steps, 3), "final_state": status, "cm": cm,
                   "cm_per_s": round(cm / seconds, 2) if cm else None,
                   "cm_per_step": round(cm / args.steps, 2) if cm else None}
            rows.append(row)
            print("  " + "  ".join(f"{k} {v}" for k, v in row.items()))
    finally:
        rest.gait_exit()
    print("  the sim's SimGait: cycle 0.7 s + 0.05 s per period unit (two steps), about 19 cm/s at speed 5")
    _save("gait", {"ip": args.ip, "rows": rows})


# -- inspect -----------------------------------------------------------------------------------------
INSPECT_CMDS = [
    "cat /etc/ubt_version.txt; cat /etc/os-release | head -3; uname -a; python3 --version; python --version",
    "cat /proc/cpuinfo | grep -i -E 'model|hardware|revision' | sort -u",
    "dpkg -l | grep -i -E 'ubt|yanshee|ros-kinetic-(ros-base|ros-core)' | awk '{print $2, $3}'",
    "systemctl list-units --type=service --all --no-pager | grep -i -E 'ubt|dm_|yanshee|ros'",
    "ps aux | grep -i -E 'ros|yanshee|ubt|gait|motion|servo' | grep -v grep | cut -c1-200",
    "ls -la /opt/yanshee /opt/yanshee/lib 2>/dev/null; find /opt/yanshee -maxdepth 3 2>/dev/null | head -400",
    "cat /opt/yanshee/lib/sysconfig/sysconfig.conf 2>/dev/null; cat /opt/yanshee/setup 2>/dev/null | head -200",
    "ls -la /mnt/1xrobot/res/hts /home/pi/Documents/motions /usr/local/recv 2>/dev/null",
    "find / -xdev -name '*.hts' 2>/dev/null | head -500",
    "find / -xdev -name '*.layers' 2>/dev/null | head -200",
    "find / -xdev \\( -iname '*gait*' -o -iname '*motion_planning*' \\) 2>/dev/null | grep -v -E 'proc|sys/' | head -100",
    "find / -xdev \\( -iname '*mpu*' -o -iname '*imu*' -o -iname '*gyro*' \\) -not -path '*/proc/*' 2>/dev/null | head -100",
    "find / -xdev -iname '*servo*' -not -path '*/proc/*' 2>/dev/null | head -100",
    "bash -lc 'source /opt/ros/kinetic/setup.bash 2>/dev/null; source /opt/yanshee/setup.bash 2>/dev/null; "
    "rospack list 2>/dev/null | grep -i -E \"ubt|yanshee|gait|apollo\"; rosservice list 2>/dev/null | head -80; "
    "rostopic list 2>/dev/null | head -80'",
    "ls -la /home/pi; ls -la /home/pi/Documents 2>/dev/null; df -h / ; free -m",
    "i2cdetect -y 1 2>/dev/null | head -12",
    "cat /boot/config.txt 2>/dev/null | grep -v '^#' | grep -v '^$'",
]


def inspect(args):
    try:
        import paramiko
    except ImportError:
        raise SystemExit("pip install paramiko (or run the commands in INSPECT_CMDS over ssh by hand)")
    out = os.path.join(OUT_DIR, f"inspect_{_stamp()}")
    os.makedirs(os.path.join(out, "hts"), exist_ok=True)
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # a robot on the LAN: no known_hosts entry yet
    client.connect(args.ip, username=args.user, password=args.password, timeout=10, look_for_keys=False,
                   allow_agent=False)
    found_hts = []
    with open(os.path.join(out, "inspect.txt"), "w") as log:
        for cmd in INSPECT_CMDS:
            log.write(f"\n$ {cmd}\n")
            try:
                _, so, se = client.exec_command(cmd, timeout=120)
                text = so.read().decode(errors="replace") + se.read().decode(errors="replace")
            except Exception as e:  # noqa: BLE001 - one failed command must not stop the dump
                text = f"[failed: {e}]\n"
            log.write(text)
            if "name '*.hts'" in cmd:
                found_hts = [ln.strip() for ln in text.splitlines() if ln.strip().endswith(".hts")]
            print(f"  ran: {cmd[:70]}... ({len(text)} chars)")
    sftp = client.open_sftp()
    copied = 0
    for path in found_hts[:500]:
        dest = os.path.join(out, "hts", path.strip("/").replace("/", "__"))
        try:
            sftp.get(path, dest)
            copied += 1
        except Exception as e:  # noqa: BLE001
            print(f"  could not copy {path}: {e}")
    for extra in ("/etc/ubt_version.txt", "/opt/yanshee/lib/sysconfig/sysconfig.conf"):
        try:
            sftp.get(extra, os.path.join(out, extra.strip("/").replace("/", "__")))
        except Exception:  # noqa: BLE001 - optional files
            pass
    sftp.close()
    client.close()
    print(f"  {len(found_hts)} .hts files found, {copied} copied to {out}/hts/ "
          f"(play one: python3 scripts/play_motion.py {out}/hts/FILE.hts)")
    print(f"  full dump: {out}/inspect.txt")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def common(p):
        p.add_argument("--ip", required=True, help="robot IP (or IP:PORT)")
        p.add_argument("--timeout", type=float, default=4.0)
        p.add_argument("--yes", action="store_true", help="skip the confirmations")
    p = sub.add_parser("latency", help="REST round trips")
    common(p)
    p.add_argument("--n", type=int, default=20)
    p = sub.add_parser("imu", help="IMU conventions in five postures")
    common(p)
    p.add_argument("--seconds", type=float, default=4.0, help="sampling time per posture")
    p.add_argument("--auto", action="store_true", help="upright only, no prompts (for tests)")
    p = sub.add_parser("servo", help="step response of one arm servo")
    common(p)
    p.add_argument("--servo", default="RightShoulderFlex")
    p.add_argument("--start", type=int, default=RESET_POSE["RightShoulderFlex"])
    p.add_argument("--end", type=int, default=90)
    p.add_argument("--runtime", type=int, default=200, help="commanded move time, ms (200 = as fast as allowed)")
    p.add_argument("--seconds", type=float, default=1.5, help="readback time after the command")
    p.add_argument("--legs", action="store_true", help="allow a leg servo (the robot must be held)")
    p = sub.add_parser("gait", help="time the built-in gait per setting")
    common(p)
    p.add_argument("--settings", default="3,1 5,1 5,3 5,5 -3,1", help="speed_v,period pairs")
    p.add_argument("--steps", type=int, default=10)
    p.add_argument("--stop-test", action="store_true", help="first compare a gentle stop with a sudden exit")
    p.add_argument("--stop-speed", type=int, default=3, help="speed_v for --stop-test (start low)")
    p = sub.add_parser("inspect", help="read-only ssh dump of the Raspberry Pi")
    p.add_argument("--ip", required=True)
    p.add_argument("--user", default="pi")
    p.add_argument("--password", default="raspberry")
    args = ap.parse_args(argv)
    try:
        {"latency": latency, "imu": imu, "servo": servo, "gait": gait, "inspect": inspect}[args.mode](args)
    except KeyboardInterrupt:
        print("\naborted")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
