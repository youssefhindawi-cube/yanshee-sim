#!/usr/bin/env python3
"""Check a motion file, preview it in the simulator, and (optionally) run it on the real robot.

    python3 scripts/play_motion.py motions/wave_right.json                 # simulator with viewer
    python3 scripts/play_motion.py motions/wave_right.json --fixed         # torso held in the air
    python3 scripts/play_motion.py motions/wave_right.json --headless      # fast check, exit 1 on FALL
    python3 scripts/play_motion.py motions/wave_right.json --real --ip 192.168.3.13 --speed 0.5

Checks run at the requested --speed. On the real robot the motion starts from the reset pose: the
robot is first moved there slowly (if that move is collision-free), exactly as the checks assume.
"""
import argparse
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import NAMES, RESET_POSE, RealRobot, SimRobot, motion  # noqa: E402
from yanshee_sim.servos import MAX_RUNTIME_MS, clamp_angles  # noqa: E402

FALL_DEG = 45  # torso tilt above this counts as a fall
RESET_SPEED_DEG_S = 60  # speed of the move to reset on the real robot (before playing and on Ctrl+C)


def _positive(text):
    v = float(text)
    if not v > 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {text}")
    return v


class TiltWatch:
    """Wraps a SimRobot: waits in small steps and records the largest torso tilt seen during playback."""

    def __init__(self, bot, stop_event=None):
        self.bot, self.stop_event, self.max_tilt = bot, stop_event, 0.0

    def __getattr__(self, name):
        return getattr(self.bot, name)

    def wait(self, seconds):
        while seconds > 1e-9:
            dt = min(0.02, seconds)
            self.bot.wait(dt)
            seconds -= dt
            self.max_tilt = max(self.max_tilt, self.bot.torso_tilt_deg())
            if self.stop_event is not None and not self.bot.is_running():
                self.stop_event.set()  # viewer closed
                return


def move_to_reset(bot):
    """Move the real robot from its current pose to RESET_POSE at RESET_SPEED_DEG_S after a collision sweep."""
    cur = clamp_angles(bot.get_servos_angles(NAMES)["data"])
    biggest = max([abs(RESET_POSE[n] - d) for n, d in cur.items()] + [0])
    runtime = int(min(max(1000, biggest / RESET_SPEED_DEG_S * 1000), MAX_RUNTIME_MS))
    hits = motion.self_collisions({"frames": [{"angles": dict(RESET_POSE), "runtime": runtime}],
                                   "end_in_reset": False}, start_pose=cur)
    if hits:
        raise RuntimeError(f"the move from the current pose to reset would self-collide ({hits[0][2]}); "
                           "put the robot in its reset pose with the app first")
    bot.set_servos_angles(dict(RESET_POSE), runtime)
    bot.wait(runtime / 1000)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("motion")
    ap.add_argument("--real", action="store_true", help="run on the real robot (after the checks pass)")
    ap.add_argument("--ip", help="robot IP for --real")
    ap.add_argument("--speed", type=_positive, default=1.0,
                    help="playback speed factor > 0; use 0.5 for first hardware runs")
    ap.add_argument("--fixed", action="store_true", help="simulator: hold the torso in the air (no falling)")
    ap.add_argument("--headless", action="store_true",
                    help="simulator: no window, runs as fast as possible; exit 1 on a fall or self-collision")
    ap.add_argument("--loop", action="store_true", help="simulator: repeat until the window is closed")
    ap.add_argument("--yes", action="store_true", help="real: skip the confirmation prompt")
    args = ap.parse_args()
    if args.loop and (args.headless or args.real):
        ap.error("--loop needs the simulator window (not --headless or --real)")
    if args.real and not args.ip:
        ap.error("--real needs --ip")

    try:
        mo = motion.load(args.motion)
    except (OSError, ValueError) as e:
        sys.exit(f"cannot read {args.motion}: {e}")
    errors, warnings = motion.validate(mo, speed=args.speed)
    for w in warnings:
        print("warning:", w)
    for e in errors:
        print("ERROR:", e)
    if errors:
        sys.exit("fix the errors above first")
    hits = motion.self_collisions(mo)
    for frame, frac, pair in hits[:10]:
        where = "the return to reset" if frame == len(mo["frames"]) else f"frame {frame + 1}"
        print(f"SELF-COLLISION: {where} at {int(frac * 100)}%: {pair[0]} touches {pair[1]}")
    print(f"{mo['name']}: {len(mo['frames'])} frames, {motion.duration_s(mo, args.speed):.1f} s at speed {args.speed:g}")

    if args.real:
        if hits:
            sys.exit("refusing to send this motion to the robot: fix the self-collisions above first")
        if not args.yes:
            input("Robot on a flat surface, clear of edges, emergency stop within reach? Press Enter to start (Ctrl+C aborts) ")
        bot = RealRobot(args.ip)
        print(f"connected, battery {bot.battery}%")
        try:
            print("moving to the reset pose (the motion starts from there)")
            move_to_reset(bot)
            done = motion.play(bot, mo, speed=args.speed,
                               on_frame=lambda i, f: print(f"frame {i + 1}: {f.get('angles', {})}"))
            print("done" if done else "stopped")
        except KeyboardInterrupt:
            print("interrupted: returning to reset")
            try:
                move_to_reset(bot)
            except Exception as e:  # a dead link or a blocked path: leave the robot where it is
                sys.exit(f"could not return to reset: {e}")
        except Exception as e:
            sys.exit(f"robot error: {e}")
        return

    bot = SimRobot(viewer=not args.headless, realtime=not args.headless, fixed_base=args.fixed)
    stop = threading.Event()
    watch = TiltWatch(bot, stop)
    try:
        while True:
            motion.play(watch, mo, speed=args.speed, stop_event=stop)
            if stop.is_set():
                break
            watch.wait(1.0)  # let it settle: a late fall still counts
            print(f"max torso tilt during playback {watch.max_tilt:.1f} deg, at the end {bot.torso_tilt_deg():.1f} deg")
            if not args.loop or not bot.is_running():
                break
        if not args.headless:
            print("close the viewer window to exit")
            while bot.is_running():
                bot.wait(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        bot.close()
    fell = watch.max_tilt > FALL_DEG
    if fell:
        print(f"FALL: torso tilted {watch.max_tilt:.0f} deg (> {FALL_DEG})")
    if fell or hits:
        sys.exit(1)


if __name__ == "__main__":
    main()
