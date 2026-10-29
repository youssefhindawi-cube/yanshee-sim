#!/usr/bin/env python3
"""Play a get-up on the real Yanshee frame by frame and watch it: the IMU's euler-x through every frame and each
servo's readback against its target at the end of the frame (a joint blocked by the floor or the body shows up as
a large error, a stall-locked servo as a refused command). Same per-frame layout as the simulator's runs.

    python3 tools/getup_lab.py --ip IP GetupFront                 # the robot's own get-up, as its .hts file has it
    python3 tools/getup_lab.py --ip IP motions/getup_front_fixed.json
    python3 tools/getup_lab.py --ip IP GetupFront --step          # wait for Enter after every frame (slow motion)
    python3 tools/getup_lab.py --ip IP GetupFront --to 8          # stop after frame 8

The robot must already lie in the right posture (face down for a front get-up). Fall management is switched off
for the run (REST and the hal node's own service), so the firmware neither interferes nor refuses commands. A
.hts name is looked up in robot_dump/inspect_*/hts (copied by `robot_checks.py inspect`). The log is saved to
robot_dump/getup_<name>_<time>.json.
"""
import argparse
import datetime
import glob
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from yanshee_sim import hts, recovery  # noqa: E402
from yanshee_sim.rest import YansheeRest  # noqa: E402
from yanshee_sim.servos import NAMES, RESET_POSE, SERVOS  # noqa: E402
import robot_checks  # noqa: E402

LIMITS = {s[1]: (s[2], s[3]) for s in SERVOS}
POSTURE_EULER_X = {"face_down": 0.0, "on_back": 180.0}


def load_frames(name):
    """[{angles, runtime, hold}] from a JSON motion ({"frames": [...]}) or a robot .hts by name or path."""
    if name.endswith(".json"):
        return json.load(open(name))["frames"]
    path = name if name.endswith(".hts") else (glob.glob(os.path.join(ROOT, "robot_dump", "inspect_*", "hts", f"*__{name}.hts")) or [None])[0]
    if not path:
        raise SystemExit(f"no .hts for {name!r}: run tools/robot_checks.py inspect first, or give a path")
    out = []
    for fr in hts.load(path)["frames"]:
        a = fr["angles"] if isinstance(fr["angles"], dict) else dict(zip(NAMES, fr["angles"]))
        out.append({"angles": {k: int(v) for k, v in a.items()}, "runtime": int(fr["runtime"]), "hold": int(fr["hold"])})
    return out


def check_frames(frames):
    for i, fr in enumerate(frames, 1):
        bad = [f"{n}={v}" for n, v in fr["angles"].items() if n not in LIMITS or not LIMITS[n][0] <= v <= LIMITS[n][1]]
        if bad:
            raise SystemExit(f"frame {i} is outside the servo limits: {', '.join(bad)}")


def engine_run(rest, name, frames):
    """Let the robot's own motion engine play `name`, exactly as the app or the firmware would, and only watch the
    IMU as fast as REST allows, summarised per frame from the file's timing. A frame-by-frame replay over REST
    starts every frame about 0.2 s late (one servo call's round trip): harmless for slow 1 s frames, but it
    removes the momentum fast get-ups rely on (GetupRear rolled over backwards when replayed, stood when played)."""
    if name.endswith((".json", ".hts")):
        raise SystemExit("--engine plays a motion stored on the robot: give its name (upload it as a .hts first)")
    ends, t = [], 0
    for fr in frames:
        t += fr["runtime"] + fr["hold"]
        ends.append(t / 1000)
    res = rest.play_motion(name)
    if res.get("code") != 0:
        raise SystemExit(f"the robot refused {name}: {res}")
    t0, samples, n = time.perf_counter(), [], 0
    while time.perf_counter() - t0 < ends[-1] + 1.5:
        gy = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
        s = {"t": round(time.perf_counter() - t0, 2), **{k: gy.get(k) for k in ("euler-x", "euler-y", "gyro-x", "gyro-y", "gyro-z")}}
        if n % 2 == 1:  # joint readback every other cycle (about 3 per second)
            s["angles"] = rest.get_angles(NAMES).get("data") or {}
        samples.append(s)
        n += 1
    print("  frame  ends at  euler-x end (min .. max)   servos behind the frame's target by > 20 deg near its end")
    log, start = [], 0.0
    for i, e in enumerate(ends, 1):
        part = [s for s in samples if start <= s["t"] < e]
        xs = [s["euler-x"] for s in part if s.get("euler-x") is not None] or [float("nan")]
        reads = [s for s in part if s.get("angles")]
        behind = {}
        if reads:
            a, target = reads[-1]["angles"], frames[i - 1]["angles"]
            behind = {k: a[k] - v for k, v in target.items() if k in a and abs(a[k] - v) > 20}
        print(f"  {i:5d}  {e:6.1f} s  {xs[-1]:+7.1f} ({min(xs):+7.1f} .. {max(xs):+7.1f})   "
              + (", ".join(f"{k} {d:+d}" for k, d in behind.items()) or ("-" if reads else "(no joint reading in this frame)")), flush=True)
        log.append({"frame": i, "ends_s": e, "behind": behind, "samples": part})
        start = e
    return log


def locked_servos(rest):
    """(stall-locked servos, the angles they report): recovery.locked_servos."""
    return recovery.locked_servos(rest), rest.get_angles(NAMES).get("data") or {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("motion", help="robot .hts name (GetupFront), .hts path, or JSON motion path")
    ap.add_argument("--ip", required=True)
    ap.add_argument("--posture", choices=sorted(POSTURE_EULER_X), default=None,
                    help="posture it must start in (default: face_down unless the name says Rear/back)")
    ap.add_argument("--step", action="store_true", help="wait for Enter after every frame")
    ap.add_argument("--from", dest="first", type=int, default=1)
    ap.add_argument("--to", dest="last", type=int, default=None)
    ap.add_argument("--slow", type=float, default=1.0, help="multiply every frame's run and hold time")
    ap.add_argument("--engine", action="store_true",
                    help="let the robot's own motion engine play it (a motion stored on the robot) and only watch")
    ap.add_argument("--finish-reset", action="store_true",
                    help="play reset afterwards to stand up out of the motion's last pose (Base_GetupF ends crouched)")
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    ap.add_argument("--force", action="store_true", help="skip the battery and posture checks")
    args = ap.parse_args()

    frames = load_frames(args.motion)
    check_frames(frames)
    last = args.last or len(frames)
    posture = args.posture or ("on_back" if any(k in args.motion.lower() for k in ("rear", "back", "getupb")) else "face_down")
    rest = YansheeRest(args.ip, timeout=4.0)
    b = robot_checks._ok(rest.battery(), "battery") or {}
    g = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
    ex0 = g.get("euler-x")
    print(f"  {args.motion}: {len(frames)} frames, playing {args.first}-{last}; battery {b.get('percent')} %"
          f"{' CHARGING' if b.get('charging') else ''}; euler-x now {ex0:+.1f} (expected about {POSTURE_EULER_X[posture]:+.0f} {posture})")
    if not args.force:
        if (b.get("percent") or 0) < 50 or b.get("charging"):
            raise SystemExit("needs 50 % or more and the charger unplugged (or --force)")
        if ex0 is None or abs(((abs(ex0) - POSTURE_EULER_X[posture]) + 180) % 360 - 180) > 30:
            raise SystemExit(f"the robot is not lying {posture} (euler-x {ex0}); lay it down first (or --force)")
    host = args.ip.split(":")[0]
    robot_checks._ok(rest.set_fall_management(False), "fall management off")
    robot_checks._hal_fallmanager(host, args.user, args.password, False)

    log, t0 = [], time.perf_counter()
    if args.engine:
        log = engine_run(rest, args.motion, frames)
        last = args.first - 1  # the robot's engine played it: no frame-by-frame replay below
    else:
        print("  frame  run ms  euler-x end (min .. max)   servos off target by > 15 deg at the end of the frame")
    try:
        for i in range(args.first, last + 1):
            fr = frames[i - 1]
            run, hold = max(200, round(fr["runtime"] * args.slow)), round(fr["hold"] * args.slow)
            res = rest.set_angles(fr["angles"], run)
            refused = [n for n, ok in (res.get("data") or {}).items() if ok is False]
            samples, end = [], time.perf_counter() + (run + hold) / 1000
            while time.perf_counter() < end:
                gy = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
                samples.append({"t": round(time.perf_counter() - t0, 2), **{k: gy.get(k) for k in ("euler-x", "euler-y", "gyro-x", "gyro-y", "gyro-z")}})
            back = rest.get_angles(list(fr["angles"])).get("data") or {}
            off = {n: back[n] - v for n, v in fr["angles"].items() if n in back and abs(back[n] - v) > 15}
            xs = [s["euler-x"] for s in samples if s.get("euler-x") is not None] or [float("nan")]
            print(f"  {i:5d}  {run:6d}  {xs[-1]:+7.1f} ({min(xs):+7.1f} .. {max(xs):+7.1f})   "
                  + (", ".join(f"{n} {d:+d}" for n, d in off.items()) or "-")
                  + (f"   REFUSED: {', '.join(refused)}" if refused else ""), flush=True)
            log.append({"frame": i, "target": fr["angles"], "runtime": run, "hold": hold, "readback": back,
                        "off": off, "refused": refused, "imu": samples})
            if refused:
                print("  a servo refused its command (stall lock?): stopping. Recover it with program mode, then reset.")
                break
            if args.step and i < last:
                input("  Enter for the next frame (Ctrl+C stops) ")
    except KeyboardInterrupt:
        print("  stopped")
    time.sleep(1.5)
    locked, ended_at = locked_servos(rest)
    if locked:
        print(f"  LOCKED servos (refuse commands, readback stale): {', '.join(f'{k} reads {ended_at.get(k)}' for k in locked)}")
    if args.finish_reset:
        if locked:
            print("  not playing reset: clear the locked servos first (program mode for them, then reset)")
        else:
            res = rest.play_motion("reset")
            print(f"  reset (stand up out of the motion's last pose): {res.get('msg')}")
            time.sleep(2.5)
    g = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
    final = g.get("euler-x")
    now = rest.get_angles(NAMES).get("data") or {}
    bent = {k: now[k] - v for k, v in RESET_POSE.items() if k in now and abs(now[k] - v) > 10}
    stood = final is not None and 60 < abs(final) < 120
    print(f"  final euler-x {final:+.1f}: {'STANDING' if stood else 'NOT standing'}"
          + (f"; joints off the standing pose by > 10 deg: {', '.join(f'{k} {d:+d}' for k, d in bent.items())}" if bent else "; legs and arms in the standing pose" if stood else ""))
    name = os.path.splitext(os.path.basename(args.motion))[0]
    out = os.path.join(ROOT, "robot_dump", f"getup_{name}_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"motion": args.motion, "battery": b, "posture": posture, "slow": args.slow, "start_euler_x": ex0,
               "final_euler_x": final, "stood_up": stood, "locked": locked, "final_angles": now, "frames": log},
              open(out, "w"), indent=1)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
