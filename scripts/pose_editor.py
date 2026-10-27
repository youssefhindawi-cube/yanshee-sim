#!/usr/bin/env python3
"""Build a motion keyframe by keyframe while watching the simulator, then save it as JSON.

    python3 scripts/pose_editor.py --out motions/my_move.json [--fixed]

Commands (type in this terminal):
    set NeckLR 60 [RightElbowFlex 120 ...]   move servos (degrees), 600 ms
    time 800                                  runtime for the next moves/keyframes (ms)
    add [hold_ms]                             append the current targets as a keyframe
    undo                                      drop the last keyframe
    play                                      replay all keyframes from the reset pose
    reset                                     go back to the reset pose
    show                                      print current targets and keyframes
    save                                      write the motion file
    quit
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import LIMITS, RESET_POSE, ServoCommandError, SimRobot, motion  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fixed", action="store_true")
    args = ap.parse_args()
    mo = motion.load(args.out) if os.path.exists(args.out) else {
        "name": os.path.splitext(os.path.basename(args.out))[0], "description": "", "frames": [], "end_in_reset": True}
    bot = SimRobot(viewer=True, fixed_base=args.fixed)
    targets, runtime = dict(RESET_POSE), 600
    print(__doc__)
    try:
        while bot.is_running():
            try:
                parts = input("pose> ").split()
            except EOFError:
                break
            if not parts:
                continue
            cmd, rest = parts[0], parts[1:]
            try:
                if cmd == "set":
                    pairs = {rest[i]: float(rest[i + 1]) for i in range(0, len(rest) - 1, 2)}
                    bot.set_servos_angles(pairs, runtime)
                    targets.update(pairs)
                elif cmd == "time":
                    runtime = int(rest[0])
                elif cmd == "add":
                    prev = dict(RESET_POSE)
                    for f in mo["frames"]:
                        prev.update(f["angles"])
                    changed = {n: round(v, 1) for n, v in targets.items() if abs(v - prev.get(n, v)) > 0.05}
                    mo["frames"].append({"angles": changed, "runtime": runtime, "hold": int(rest[0]) if rest else 0})
                    print(f"keyframe {len(mo['frames']) - 1}: {changed}")
                elif cmd == "undo" and mo["frames"]:
                    mo["frames"].pop()
                elif cmd == "play":
                    bot.reset()
                    motion.play(bot, mo)
                    targets = dict(RESET_POSE)
                elif cmd == "reset":
                    bot.reset()
                    targets = dict(RESET_POSE)
                elif cmd == "show":
                    print("targets:", {n: targets[n] for n in LIMITS})
                    for i, f in enumerate(mo["frames"]):
                        print(f"  {i}: {f}")
                elif cmd == "save":
                    errors, warnings = motion.validate(mo)
                    print("\n".join(["ERROR: " + e for e in errors] + ["warning: " + w for w in warnings]) or "checks passed")
                    for fr, frac, pair in motion.self_collisions(mo)[:5]:
                        print(f"SELF-COLLISION: frame {fr} at {int(frac * 100)}%: {pair}")
                    motion.save(mo, args.out)
                    print("saved", args.out)
                elif cmd in ("quit", "exit"):
                    break
                else:
                    print("unknown command; see the list above")
            except (ServoCommandError, ValueError, IndexError) as e:
                print("error:", e)
    finally:
        bot.close()


if __name__ == "__main__":
    main()
