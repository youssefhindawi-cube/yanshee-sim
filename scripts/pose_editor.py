#!/usr/bin/env python3
"""Build a motion keyframe by keyframe while watching the simulator, then save it as JSON.

    python3 scripts/pose_editor.py --out motions/my_move.json [--fixed]
    YANSHEE_SIM_HEADLESS=1 ... runs without a window (commands still work)

Commands (type in this terminal):
    set NeckLR 60 [RightElbowFlex 120 ...]   move servos (degrees), 600 ms
    time 800                                  runtime for the next moves/keyframes (200-4000 ms)
    add [hold_ms]                             append the current targets as a keyframe
    undo                                      drop the last keyframe
    play                                      replay all keyframes from the reset pose
    reset                                     go back to the reset pose
    show                                      print current targets and keyframes
    save                                      write the motion file
    quit

An existing --out file is loaded and the simulator moves to the pose after its last keyframe,
so new keyframes continue from there.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import LIMITS, RESET_POSE, ServoCommandError, SimRobot, motion  # noqa: E402
from yanshee_sim.servos import MAX_RUNTIME_MS, MIN_RUNTIME_MS, check_angles, clamp_angles  # noqa: E402


def accumulated_pose(mo):
    """The servo targets after the last keyframe (frames only name the servos they change)."""
    pose = dict(RESET_POSE)
    for f in mo["frames"]:
        pose.update(clamp_angles({n: d for n, d in f.get("angles", {}).items() if isinstance(d, (int, float))}))
    return pose


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fixed", action="store_true")
    args = ap.parse_args()
    mo = {"name": os.path.splitext(os.path.basename(args.out))[0], "description": "", "frames": [], "end_in_reset": True}
    if os.path.exists(args.out):
        try:
            mo = motion.load(args.out)
            if not isinstance(mo, dict) or not isinstance(mo.get("frames"), list):
                raise ValueError("not a motion file (needs a 'frames' list)")
        except (OSError, ValueError) as e:
            sys.exit(f"cannot load {args.out}: {e}")
    bot = SimRobot(viewer=os.environ.get("YANSHEE_SIM_HEADLESS") != "1", fixed_base=args.fixed)
    targets, runtime = accumulated_pose(mo), 600
    if mo["frames"]:  # continue from where the loaded motion ends
        if args.fixed:
            bot.jump_to(targets)
        else:
            bot.sync_set_servo_rotate(targets, 1000)
        print(f"loaded {len(mo['frames'])} keyframes from {args.out}")
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
                    if not rest or len(rest) % 2:
                        raise ValueError("usage: set NAME DEG [NAME DEG ...]")
                    pairs = {rest[i]: float(rest[i + 1]) for i in range(0, len(rest), 2)}
                    problems = check_angles(pairs)
                    if problems:
                        raise ValueError("; ".join(problems))
                    bot.set_servos_angles(pairs, runtime)
                    targets.update(pairs)
                elif cmd == "time":
                    ms = int(rest[0])
                    if not MIN_RUNTIME_MS <= ms <= MAX_RUNTIME_MS:
                        raise ValueError(f"time must be {MIN_RUNTIME_MS}-{MAX_RUNTIME_MS} ms")
                    runtime = ms
                elif cmd == "add":
                    hold = int(rest[0]) if rest else 0
                    if hold < 0:
                        raise ValueError("hold must be >= 0 ms")
                    prev = accumulated_pose(mo)
                    changed = {n: round(v, 1) for n, v in targets.items() if abs(v - prev.get(n, v)) > 0.05}
                    mo["frames"].append({"angles": changed, "runtime": runtime, "hold": hold})
                    print(f"keyframe {len(mo['frames'])}: {changed}")
                elif cmd == "undo":
                    if mo["frames"]:
                        mo["frames"].pop()
                        print(f"removed keyframe {len(mo['frames']) + 1}")
                    else:
                        print("nothing to undo")
                elif cmd == "play":
                    bot.reset()
                    motion.play(bot, mo)
                    targets = dict(RESET_POSE) if mo.get("end_in_reset") else accumulated_pose(mo)
                elif cmd == "reset":
                    bot.reset()
                    targets = dict(RESET_POSE)
                elif cmd == "show":
                    print("targets:", {n: targets[n] for n in LIMITS})
                    for i, f in enumerate(mo["frames"]):
                        print(f"  {i + 1}: {f}")
                elif cmd == "save":
                    errors, warnings = motion.validate(mo)
                    print("\n".join(["ERROR: " + e for e in errors] + ["warning: " + w for w in warnings]) or "checks passed")
                    for fr, frac, pair in motion.self_collisions(mo)[:5]:
                        where = "return to reset" if fr == len(mo["frames"]) else f"frame {fr + 1}"
                        print(f"SELF-COLLISION: {where} at {int(frac * 100)}%: {pair}")
                    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
                    motion.save(mo, args.out)
                    print("saved", args.out)
                elif cmd in ("quit", "exit"):
                    break
                else:
                    print("unknown command; see the list above")
            except (ServoCommandError, ValueError, IndexError, OSError) as e:
                print("error:", e)
    finally:
        bot.close()


if __name__ == "__main__":
    main()
