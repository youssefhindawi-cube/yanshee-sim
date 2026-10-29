#!/usr/bin/env python3
"""Measurements to take on the real Yanshee that the simulator still lacks (docs/ROBOT_SESSION_PLAN.md).

    python3 tools/robot_checks.py latency --ip IP            # REST round trips (battery, angles, gyro)
    python3 tools/robot_checks.py imu --ip IP                # IMU conventions: upright, leaning, lying (interactive)
    python3 tools/robot_checks.py gyro --ip IP               # gyro axes and signs: three hand rotations (interactive)
    python3 tools/robot_checks.py fall --ip IP               # firmware fall management: lay it down, does it get up?
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
POSTURES = [("upright", "1/6 Stand the robot upright on its feet and take your hands off."),
            ("back20", "2/6 Hold it leaning about 20 degrees BACKWARD from the ankles (feet flat, head back)."),
            ("right20", "3/6 Hold it leaning about 20 degrees to ITS RIGHT (your left when you face it), feet flat."),
            ("face_down", "4/6 Lay it FACE DOWN on the mat and take your hands off."),
            ("on_back", "5/6 Lay it on its BACK, face up, and take your hands off."),
            ("turned_left", "6/6 Stand it upright where it stood for 1/6, then lift it, turn it a QUARTER TURN TO ITS LEFT "
                            "(anticlockwise seen from above: it ends up facing your right), set it down, hands off.")]
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
             "(the sim emulates +1 on y: chip axes x right, y up, z back, measured 2026-10-29)"]
    if "back20" in results:
        d = results["back20"]["mean"].get("euler-x", 0) - up.get("euler-x", 0)
        notes.append(f"leaning back changes euler-x by {d:+.1f}: the sim raises it (UBTECH's balance sample too)")
    if "right20" in results:
        d = results["right20"]["mean"].get("euler-y", 0) - up.get("euler-y", 0)
        notes.append(f"leaning right changes euler-y by {d:+.1f}: the sim makes it positive")
    for key, expect in (("face_down", "about 0"), ("on_back", "about +-180")):
        if key in results:
            notes.append(f"{key} euler-x {results[key]['mean'].get('euler-x', 0):+.1f} (course: {expect})")
    if "turned_left" in results:
        d = (results["turned_left"]["mean"].get("euler-z", 0) - up.get("euler-z", 0) + 180) % 360 - 180
        notes.append(f"a quarter turn to its left changes euler-z by {d:+.1f}: the sim makes it +90")
    for key in ("back20", "right20", "face_down", "on_back"):
        if key in results:
            a = results[key]["mean"]
            notes.append(f"{key} accel: x {a.get('accel-x', 0):+.2f} y {a.get('accel-y', 0):+.2f} z {a.get('accel-z', 0):+.2f}")
    print("\n".join("  " + n for n in notes))
    _save("imu", {"ip": args.ip, "seconds": args.seconds, "postures": results, "notes": notes})


# The three hand rotations and the axis the sim's gyro (chip axes right/up/back, measured) puts each one on: (key, prompt,
# gyro channel, expected sign of the integral, nominal degrees, the euler angle that shows the move was made and held).
ROTATIONS = [("turn_left", "Stand it upright. At GO, turn it a QUARTER TURN TO ITS LEFT on the spot (anticlockwise seen "
                           "from above) in about a second, then hold still.", "gyro-y", +1, 90, "euler-z"),
             ("tilt_back", "Hold it upright by the torso. At GO, tilt it about 30 degrees BACKWARD (head back) and "
                           "hold it there until the readings print.", "gyro-x", +1, 30, "euler-x"),
             ("tilt_right", "Hold it upright by the torso. At GO, tilt it about 30 degrees to ITS RIGHT (your left "
                            "when you face it) and hold it there until the readings print.", "gyro-z", -1, 30, "euler-y")]


def _gyro_axis(rows, euler):
    """Which gyro channel turns with `euler` during the move, and with which sign: the sum over samples of
    rate * change of the euler angle. Rotation before or after the move (the robot still being turned when GO
    printed) hardly changes the target angle, so it barely counts; a bad still-window bias does not matter."""
    score = {k: 0.0 for k in ("gyro-x", "gyro-y", "gyro-z")}
    for a, b in zip(rows, rows[1:]):
        if euler in a and euler in b:
            de = (float(b[euler]) - float(a[euler]) + 180) % 360 - 180
            for k in score:
                if k in b:
                    score[k] += float(b[k]) * de
    top = max(score, key=lambda k: abs(score[k]))
    return top, (1 if score[top] > 0 else -1), score


def _integrate_gyro(rest, seconds, bias):
    """Integral in degrees of each gyro channel minus bias over `seconds` (rectangle rule between REST reads),
    the bias-free peak rate of each channel, and the raw samples."""
    total, last, rows = {k: 0.0 for k in ("gyro-x", "gyro-y", "gyro-z")}, time.perf_counter(), []
    t0 = last
    while last - t0 < seconds:
        g = ((_ok(rest.gyro(), "gyro") or {}).get("gyro") or [{}])[0]
        now = time.perf_counter()
        for k in total:
            if k in g:
                total[k] += (float(g[k]) - bias.get(k, 0.0)) * (now - last)
        rows.append({"t": round(now - t0, 3), **{k: g[k] for k in KEYS if k in g}})
        last = now
    peak = {k: max((float(r[k]) - bias.get(k, 0.0) for r in rows if k in r), key=abs, default=0.0) for k in total}
    return total, len(rows) / seconds, peak, rows


def gyro(args):
    rest = YansheeRest(args.ip, timeout=args.timeout)
    results, notes = {}, []
    only = set(args.only.split(",")) if args.only else None
    for key, text, channel, sign, deg, euler in ROTATIONS:
        if only and key not in only:
            continue
        _confirm(text + " (Enter, keep it still for 2 s, then move at GO)", args.yes)
        bias, _, _ = _sample_gyro(rest, 2.0)
        print("  GO", flush=True)
        total, rate, peak, rows = _integrate_gyro(rest, args.seconds, bias)
        end = [r for r in rows if r["t"] > args.seconds - 0.8 and euler in r] or rows[-1:]
        moved = (statistics.fmean(float(r[euler]) for r in end) - bias.get(euler, 0.0) + 180) % 360 - 180
        results[key] = {"integral_deg": {k: round(v, 1) for k, v in total.items()}, "peak_deg_s": {k: round(v, 1) for k, v in peak.items()},
                        f"{euler}_change": round(moved, 1), "reads_per_s": round(rate, 1), "samples": rows}
        print("  integral " + "  ".join(f"{k} {v:+7.1f} deg" for k, v in total.items()) + f"   ({rate:.1f} reads/s)")
        print("  peak     " + "  ".join(f"{k} {v:+7.1f} d/s" for k, v in peak.items()) + f"   {euler} moved {moved:+.1f} deg")
        if abs(moved) < deg / 3:
            notes.append(f"{key}: NOT MEASURED: {euler} moved only {moved:+.1f} deg by the end (expected about {deg}); "
                         "the move was missed or undone before the readings printed. Peak rates: "
                         + ", ".join(f"{k} {v:+.0f}" for k, v in peak.items()))
            continue
        axis, axis_sign, _ = _gyro_axis(rows, euler)
        results[key]["axis"] = [axis, axis_sign]
        agree = axis == channel and axis_sign == sign * (1 if moved > 0 else -1)
        notes.append(f"{key}: {euler} moved {moved:+.0f} deg; the gyro channel that turns with it is {axis} "
                     f"({'+' if axis_sign > 0 else '-'}); the sim puts it on {channel} ({'+' if sign > 0 else '-'}): "
                     f"{'agrees' if agree else 'DIFFERS'}")
    print("\n".join("  " + n for n in notes))
    _save("gyro", {"ip": args.ip, "seconds": args.seconds, "rotations": results, "notes": notes})


# -- fall: the firmware's own get-up (fall management, test plan C4) ----------------------------------
# What the robot's hal node does (read from its binary and its log on 2026-10-29): with fall management ON it
# watches the IMU for a FALL, a fast change of the euler angles with a gyro spike ("Gesture: forward fall",
# "backward fall", "left and right fall"). A slow change, a robot being laid down by hand, is logged as "fall
# detection, but cause by human beings !" and ignored. A real fall is "ready to raise up !": after a timer it
# plays GetupFront or GetupRear, and while robot_fall is set the gait, motion and servo state machines refuse
# commands. Laying the robot down therefore never triggers it; the trials below let it fall.
# The on/off switch: PUT /devices/fall_management only rewrites sysconfig's config file; on our robot sysconfig's
# boot-time hand-over to hal failed ("Send FallManager srv(value: 0) failed!") and hal never logged receiving a
# setting, so the trials also set hal's own service /hal_fallmanager_config over ssh (it answers rc 0).
# The robot's get-up motions, decoded from its own .hts files (frames' run + hold times):
GETUP_S = {"GetupFront": 13.8, "GetupRear": 8.1, "Base_GetupF": 6.8, "Base_GetupB": 7.0}
FALL_TRIALS = [("face_down", "Stand it upright in the middle of the mat. Press Enter, then make it FALL FORWARD: hold it by "
                             "the torso, lean it about 30 degrees forward over the mat and let go (or push its upper back "
                             "lightly). Hands off once it falls.", "front_motion"),
               ("on_back", "Stand it upright in the middle of the mat. Press Enter, then make it FALL BACKWARD: lean it "
                           "about 30 degrees back over the mat and let go (or push its chest lightly). Hands off once it "
                           "falls.", "rear_motion")]
FALL_LOG_KEYS = ("fall", "getup", "gesture", "raise up", "human")


class _RobotLog:
    """The robot's own log lines about falls, read live from /rosout over ssh (the hal node logs every decision)."""

    def __init__(self, host, user, password):
        import subprocess
        import threading
        env = dict(os.environ, SSHPASS=password)
        cmd = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8", f"{user}@{host}",
               "source /opt/ros/kinetic/setup.bash; source /opt/yanshee/setup.bash 2>/dev/null; exec rostopic echo /rosout/msg"]
        self.lines, self.t0 = [], time.perf_counter()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            text = line.strip()
            text = text[6:] if text.startswith("data: ") else text
            text = text.strip('"').strip()
            if text and text != "---" and any(k in text.lower() for k in FALL_LOG_KEYS):
                self.lines.append((round(time.perf_counter() - self.t0, 2), text))
                print(f"  [robot log] {text}", flush=True)

    def mark(self):
        self.t0 = time.perf_counter()
        return len(self.lines)

    def close(self):
        self.proc.terminate()


def _hal_fallmanager(host, user, password, enable):
    """Set the hal node's fall manager directly (ROS service over ssh). Returns the service's rc, or None."""
    import subprocess
    env = dict(os.environ, SSHPASS=password)
    cmd = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8", f"{user}@{host}",
           "source /opt/ros/kinetic/setup.bash; source /opt/yanshee/setup.bash 2>/dev/null; "
           f"rosservice call /hal_fallmanager_config 'enable: {'true' if enable else 'false'}'"]
    try:
        out = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in out.splitlines():
        if line.strip().startswith("rc:"):
            return int(line.split(":")[1])
    return None


def _wait_upright(rest, seconds):
    """Wait until the robot has been upright (euler-x 60-120) for 1 s; False after `seconds`."""
    t0, since = time.perf_counter(), None
    while time.perf_counter() - t0 < seconds:
        g = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
        ex = g.get("euler-x")
        if ex is not None and 60 < abs(ex) < 120:
            since = since or time.perf_counter()
            if time.perf_counter() - since > 1.0:
                return True
        else:
            since = None
        time.sleep(0.1)
    return False


def _watch_getup(rest, timeout, react_s):
    """Poll the IMU every cycle and, alternately, the motion state and the 17 servo angles. Phases: wait for the
    robot to be down (euler-x within 30 of 0 or 180), then for the first servo move of more than 8 degrees once it
    has settled for 0.6 s (the get-up starting), then until it has been upright (euler-x 60-120) for 2 s. Gives up
    after `timeout` s, or `react_s` s without going down or, once down, without a get-up starting."""
    names = list(RESET_POSE)
    t0, rows, events, ref, upright_since, i = time.perf_counter(), [], {}, None, None, 0
    while True:
        t = time.perf_counter() - t0
        if t > timeout or ("lying" not in events and t > react_s) or \
                ("lying" in events and "getup_start" not in events and t - events["lying"] > react_s):
            break
        r = {"t": round(t, 2)}
        try:
            g = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
            r["euler-x"], r["gyro"] = g.get("euler-x"), [g.get(k) for k in ("gyro-x", "gyro-y", "gyro-z")]
            if i % 2 == 0:
                m = rest.motion_state().get("data") or {}
                r["motion"], r["status"] = m.get("name"), m.get("status")
            else:
                r["angles"] = rest.get_angles(names).get("data") or {}
        except Exception as e:  # noqa: BLE001 - a busy robot may drop a call; keep watching
            r["error"] = str(e)
        i += 1
        rows.append(r)
        ex = r.get("euler-x")
        if ex is not None and "lying" not in events and (abs(ex) < 30 or abs(ex) > 150):
            events["lying"], events["lying_euler_x"] = r["t"], round(ex, 1)
        a = r.get("angles")
        if a and "lying" in events and r["t"] - events["lying"] > 0.6:
            if ref is None:
                ref = dict(a)
            elif "getup_start" not in events and max(abs(a[n] - ref[n]) for n in ref if n in a) > 8:
                events["getup_start"] = r["t"]
        if r.get("motion") and r.get("status") not in (None, "", "idle") and "motion" not in events:
            events["motion"], events["motion_seen"] = r["motion"], r["t"]
        if ex is not None and 60 < abs(ex) < 120 and "getup_start" in events:
            upright_since = r["t"] if upright_since is None else upright_since
            if r["t"] - upright_since > 2.0:
                events["upright"] = upright_since
                break
        else:
            upright_since = None
    return events, rows


def fall(args):
    rest = YansheeRest(args.ip, timeout=args.timeout)
    b = _ok(rest.battery(), "battery") or {}
    print(f"  battery {b.get('percent')} %, {'CHARGING' if b.get('charging') else 'on battery'}")
    if b.get("charging"):
        _confirm("Unplug the charger first: the get-up would drag the cable around.", args.yes)
        b = _ok(rest.battery(), "battery") or {}
        if b.get("charging") and not args.force:
            raise SystemExit("the robot still reports CHARGING: unplug the cable (or pass --force)")
    if (b.get("percent") or 0) < 50 and not args.force:
        raise SystemExit("battery under 50 %: charge it first (or pass --force)")
    log = None
    if not args.no_log:
        try:
            log = _RobotLog(args.ip.split(":")[0], args.user, args.password)
        except OSError as e:
            print(f"  (no live robot log: {e})")
    host = args.ip.split(":")[0]
    was = (_ok(rest.get_fall_management(), "fall management") or {}).get("enable")
    _ok(rest.set_fall_management(True), "fall management on")
    hal_rc = _hal_fallmanager(host, args.user, args.password, True)
    print(f"  fall management was {'ON' if was else 'OFF'}; it is ON for the test "
          f"(REST setting, and hal's own service: {'rc ' + str(hal_rc) if hal_rc is not None else 'NOT REACHED over ssh'})")
    results = {}
    only = set(args.only.split(",")) if args.only else None
    try:
        for key, text, motion_arg in FALL_TRIALS:
            if only and key not in only:
                continue
            fallback = getattr(args, motion_arg)
            _confirm(f"{text} Soft mat on the floor, 1 m clear.", args.yes)
            if not _wait_upright(rest, 1.5):
                print("  it is not standing yet: stand it upright in the middle of the mat (waiting up to 60 s)")
                if not _wait_upright(rest, 60):
                    print("  still not upright; skipping this trial")
                    continue
            print("  upright: now let it fall", flush=True)
            n = log.mark() if log else 0
            events, rows = _watch_getup(rest, args.seconds, args.react)
            how = "by itself"
            if "lying" not in events:
                print(f"  it did not go down within {args.react:.0f} s (euler-x stayed upright)")
                results[key] = {"events": events, "how": "did not fall", "samples": rows,
                                "robot_log": log.lines[n:] if log else None}
                continue
            if "getup_start" not in events:
                print(f"  down (euler-x {events['lying_euler_x']}), but nothing moved for {args.react:.0f} s: "
                      "fall management did not start a get-up")
                try:
                    _confirm(f"  Play {fallback} now (what the GUI's Get up button does)?", args.yes)
                except KeyboardInterrupt:
                    print("  skipped")
                    results[key] = {"events": events, "how": "did not get up", "samples": rows,
                                    "robot_log": log.lines[n:] if log else None}
                    continue
                _ok(rest.play_motion(fallback), fallback)
                events2, rows2 = _watch_getup(rest, args.seconds, 10)
                last = rows[-1]["t"] if rows else 0
                events.update({"played": fallback, **{f"played_{k}": v for k, v in events2.items()}})
                rows += [{**r, "t": round(r["t"] + last, 2)} for r in rows2]
                how = f"only after {fallback} was played"
            start = events.get("getup_start", events.get("played_getup_start"))
            end = events.get("upright", events.get("played_upright"))
            took = round(end - start, 1) if start is not None and end is not None else None
            waited = round(events["getup_start"] - events["lying"], 1) if "getup_start" in events else None
            final = next((r["euler-x"] for r in reversed(rows) if r.get("euler-x") is not None), None)
            stood = final is not None and 60 < abs(final) < 120
            motion = events.get("motion") or events.get("played_motion") or events.get("played")
            results[key] = {"events": events, "how": how, "stood_up": stood, "final_euler_x": final, "motion": motion,
                            "waited_s": waited, "getup_s": took, "samples": rows,
                            "robot_log": log.lines[n:] if log else None}
            print(f"  {key}: down at euler-x {events.get('lying_euler_x')}; "
                  + (f"fall management started a get-up {waited} s later; " if waited is not None else "")
                  + f"motion {motion or 'not reported'}; upright {took} s after it started; "
                  f"{'STOOD UP' if stood else 'DID NOT STAND'} {how}; final euler-x {final}")
    finally:
        _ok(rest.set_fall_management(bool(args.leave_on)), "fall management")
        _hal_fallmanager(host, args.user, args.password, bool(args.leave_on))
        print(f"  fall management switched {'ON' if args.leave_on else 'OFF'} (it was {'ON' if was else 'OFF'} before the test)")
        if log:
            log.close()
    _save("fall", {"ip": args.ip, "battery": b, "fall_management_before": was, "hal_service_rc": hal_rc,
                   "trials": results, "robot_log_all": log.lines if log else None})


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
    p = sub.add_parser("gyro", help="gyro axes and signs from three hand rotations")
    common(p)
    p.add_argument("--seconds", type=float, default=5.0, help="integration time after GO")
    p.add_argument("--only", help="comma-separated moves to run: turn_left,tilt_back,tilt_right")
    p = sub.add_parser("fall", help="firmware fall management: does it get up by itself, face down and on its back")
    common(p)
    p.add_argument("--seconds", type=float, default=40.0, help="longest wait per trial")
    p.add_argument("--react", type=float, default=20.0, help="give up after this long without a fall, or without a get-up once down")
    p.add_argument("--only", help="face_down or on_back")
    p.add_argument("--leave-on", action="store_true", help="leave fall management ON afterwards (default OFF)")
    p.add_argument("--front-motion", default="GetupFront", help="get-up to play if it does not get up by itself face down")
    p.add_argument("--rear-motion", default="GetupRear", help="get-up to play if it does not get up by itself on its back")
    p.add_argument("--no-log", action="store_true", help="do not read the robot's log over ssh")
    p.add_argument("--user", default="pi")
    p.add_argument("--password", default="raspberry")
    p.add_argument("--force", action="store_true", help="run under 50 %% battery")
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
        {"latency": latency, "imu": imu, "gyro": gyro, "fall": fall, "servo": servo, "gait": gait, "inspect": inspect}[args.mode](args)
    except KeyboardInterrupt:
        print("\naborted")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
