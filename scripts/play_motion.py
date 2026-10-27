#!/usr/bin/env python3
"""Check a motion file, preview it in the simulator, and (optionally) run it on the real robot.

    python3 scripts/play_motion.py motions/wave_right.json                 # simulator with viewer
    python3 scripts/play_motion.py motions/wave_right.json --fixed         # torso held in the air
    python3 scripts/play_motion.py motions/wave_right.json --real --ip 192.168.3.13 --speed 0.5
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import RealRobot, SimRobot, motion  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("motion")
    ap.add_argument("--real", action="store_true", help="run on the real robot (after the checks pass)")
    ap.add_argument("--ip", help="robot IP for --real")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor; use 0.5 for first hardware runs")
    ap.add_argument("--fixed", action="store_true", help="simulator: hold the torso in the air (no falling)")
    ap.add_argument("--headless", action="store_true", help="simulator: no window")
    ap.add_argument("--loop", action="store_true", help="simulator: repeat until the window is closed")
    ap.add_argument("--yes", action="store_true", help="real: skip the confirmation prompt")
    args = ap.parse_args()

    mo = motion.load(args.motion)
    errors, warnings = motion.validate(mo)
    for w in warnings:
        print("warning:", w)
    for e in errors:
        print("ERROR:", e)
    hits = motion.self_collisions(mo)
    for frame, frac, pair in hits[:10]:
        print(f"SELF-COLLISION: frame {frame} at {int(frac * 100)}%: {pair[0]} touches {pair[1]}")
    print(f"{mo['name']}: {len(mo['frames'])} frames, {motion.duration_s(mo) / args.speed:.1f} s")

    if args.real:
        if errors or hits:
            sys.exit("refusing to send this motion to the robot: fix the errors / self-collisions above first")
        if not args.ip:
            sys.exit("--real needs --ip")
        if not args.yes:
            input("Robot on a flat surface, clear of edges, emergency stop within reach? Press Enter to start (Ctrl+C aborts) ")
        bot = RealRobot(args.ip)
        print(f"connected, battery {bot.battery}%")
        try:
            motion.play(bot, mo, speed=args.speed, on_frame=lambda i, f: print(f"frame {i}: {f['angles']}"))
        except KeyboardInterrupt:
            print("interrupted: returning to reset")
            bot.reset()
        return

    bot = SimRobot(viewer=not args.headless, fixed_base=args.fixed)
    try:
        while True:
            motion.play(bot, mo, speed=args.speed)
            print(f"max torso tilt now {bot.torso_tilt_deg():.1f} deg")
            if not args.loop or not bot.is_running():
                break
        if not args.headless:
            print("close the viewer window to exit")
            while bot.is_running():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        bot.close()


if __name__ == "__main__":
    main()
