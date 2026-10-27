#!/usr/bin/env python3
"""Record a built-in motion from the real robot into a motion file the simulator can replay.

The robot plays the motion while this script polls all 17 servo angles; samples become keyframes.

    python3 scripts/record_real_motion.py --ip 192.168.3.13 --name wave --direction both
    python3 scripts/record_real_motion.py --ip 192.168.3.13 --name WakaWaka --hz 4 --max-seconds 90

Output: motions/recorded/<name>[_<direction>].json, or --out PATH (yanapi_sim plays the former
for start_play_motion).
--hz is at most 5: a keyframe cannot be shorter than YanAPI's 200 ms minimum runtime, so faster
sampling would replay slower than the original. Fast moves come out smoothed. Readings are clamped
to the servo limits. Nothing is written (exit code 1) if the robot refuses the motion, the angle
polling fails, or fewer than 2 keyframes were captured; an existing recording is then kept.
"""
import argparse
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import NAMES, RealRobot, motion  # noqa: E402
from yanshee_sim.servos import MAX_RUNTIME_MS, MIN_RUNTIME_MS, clamp_angles  # noqa: E402

MAX_HZ = 1000 / MIN_RUNTIME_MS  # 5 Hz


def _hz(text):
    hz = float(text)
    if not 0 < hz <= MAX_HZ:
        raise argparse.ArgumentTypeError(f"must be > 0 and <= {MAX_HZ:g}")
    return hz


def _play(bot, name, direction=""):
    """Start a built-in motion; returns (accepted, reply)."""
    try:
        res = bot.play_builtin(name, direction)
    except Exception as e:  # noqa: BLE001 - refusals and transport errors end the recording
        return False, str(e)
    return (res.get("code") == 0 if isinstance(res, dict) else bool(res)), res


def _status(rest):
    try:
        res = rest.motion_state()
        return (res.get("data") or {}).get("status", "") if res.get("code") == 0 else ""
    except Exception:  # noqa: BLE001 - a missed state poll only delays the end check
        return ""


def _wait_idle(rest, deadline, start_grace=1.5):
    """Wait until the motion has run and the robot reports idle again (or the deadline passes)."""
    t0, seen_run = time.time(), False
    while time.time() < deadline:
        status = _status(rest)
        seen_run |= status not in ("", "idle")
        if status == "idle" and (seen_run or time.time() - t0 > start_grace):
            return True
        time.sleep(0.1)
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="robot IP, or IP:PORT")
    ap.add_argument("--name", required=True, help="built-in motion name, e.g. wave, bow, raise, WakaWaka")
    ap.add_argument("--direction", default="")
    ap.add_argument("--hz", type=_hz, default=5.0, help=f"samples per second, 0 < hz <= {MAX_HZ:g} (default 5)")
    ap.add_argument("--max-seconds", type=float, default=30.0)
    ap.add_argument("--out", help="output file (default motions/recorded/NAME[_DIRECTION].json)")
    args = ap.parse_args(argv)

    try:
        bot = RealRobot(args.ip)
    except Exception as e:  # noqa: BLE001 - unreachable, low battery, ...
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    rest = bot.rest
    started, res = _play(bot, "reset")
    if not started:
        print(f"ERROR: the robot did not play 'reset': {res}", file=sys.stderr)
        return 1
    _wait_idle(rest, time.time() + 10)

    samples, failure, stop = [], [], threading.Event()

    def poll():
        t0 = time.time()
        try:
            while not stop.is_set() and time.time() - t0 < args.max_seconds:
                res = rest.get_angles(NAMES)
                if res.get("code") == 0 and all(n in res.get("data", {}) for n in NAMES):
                    samples.append((time.time() - t0, clamp_angles({n: res["data"][n] for n in NAMES})))
                stop.wait(max(0.0, 1 / args.hz - 0.01))
        except Exception as e:  # noqa: BLE001 - reported by the main thread
            failure.append(e)

    th = threading.Thread(target=poll, daemon=True)
    th.start()
    started, res = _play(bot, args.name, args.direction)
    if started:
        _wait_idle(rest, time.time() + args.max_seconds)
        time.sleep(0.5)
    stop.set()
    th.join()

    if not started:
        print(f"ERROR: the robot did not play '{args.name}': {res}; nothing written", file=sys.stderr)
        return 1
    if failure:
        print(f"ERROR: reading the servo angles failed: {failure[0]}; nothing written", file=sys.stderr)
        return 1
    if len(samples) < 3:
        print(f"ERROR: only {len(samples)} samples captured; nothing written", file=sys.stderr)
        return 1

    frames, prev_t = [], samples[0][0]
    for t, angles in samples[1:]:
        frames.append({"angles": angles,
                       "runtime": int(min(max((t - prev_t) * 1000, MIN_RUNTIME_MS), MAX_RUNTIME_MS))})
        prev_t = t
    name = args.name + (f"_{args.direction}" if args.direction else "")
    out = os.path.abspath(args.out or os.path.join(ROOT, "motions", "recorded", name + ".json"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    tmp = out + ".tmp"
    motion.save({"name": name, "description": f"Recorded from the real robot at {args.hz:g} Hz",
                 "frames": frames, "end_in_reset": True}, tmp)
    os.replace(tmp, out)  # atomic: a failed run never leaves a half-written file
    print(f"{len(frames)} keyframes over {prev_t - samples[0][0]:.1f} s -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
