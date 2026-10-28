#!/usr/bin/env python3
"""Find a get-up motion for the simulator by search: knock the robot flat, play candidate keyframes, keep what stands.

    python3 tools/tune_getup.py front            # face down (default): random sampling, then hill-climbing
    python3 tools/tune_getup.py rear             # lying on its back
    python3 tools/tune_getup.py front --samples 600 --climb 400 --jobs 20

A candidate is FRAMES symmetric keyframes (right-side servo values, mirrored to the left with the official rule)
played for FRAME_MS each, then the reset pose. Score = torso tilt after the motion plus a penalty when the robot
is not resting on its feet. The best candidate that stands (tilt < 15 deg, and still standing 2 s later) is
written to motions/builtin/getup_<posture>.json, which Controller.get_up() plays; results go to --out.
"""
import argparse
import json
import multiprocessing as mp
import os
import random
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("YANSHEE_SIM_HEADLESS", "1")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.servos import RESET_POSE, clamp_angles, mirror_pose  # noqa: E402

FRAMES = 5
FRAME_MS = 900
HOLD_MS = 200
RISE_MS = 2500  # the closing move to the reset pose, slow so a crouch becomes a stand
# right-side servos a symmetric get-up moves, with their search ranges (servo degrees)
PARAMS = [("RightShoulderRoll", 0, 180), ("RightShoulderFlex", 0, 180), ("RightElbowFlex", 0, 180),
          ("RightHipFB", 10, 110), ("RightKneeFlex", 0, 132), ("RightAnkleFB", 40, 180)]
KNOCK = {"front": +1, "rear": -1}

_bot = None


def _init():
    global _bot
    from yanshee_sim.robot import SimRobot
    _bot = SimRobot(viewer=False, realtime=False)
    _bot.strict = False


def knock(bot, direction):
    """Push at the head for 0.4 s: +1 tips the robot onto its face, -1 onto its back. Returns (tilt, euler-x)."""
    head = bot.model.body("NeckLR").id
    with bot._lock:
        bot.data.xfrc_applied[head] = [0, 12.0 * direction, 0, 0, 0, 0]
    bot.wait(0.4)
    with bot._lock:
        bot.data.xfrc_applied[head] = 0
    bot.wait(2.5)
    return bot.torso_tilt_deg(), bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]


def frames_of(vec):
    """vec: FRAMES * len(PARAMS) servo values -> motion frames (right side mirrored to the left) + reset."""
    out = []
    for k in range(FRAMES):
        right = {name: vec[k * len(PARAMS) + i] for i, (name, lo, hi) in enumerate(PARAMS)}
        pose = {**right, **mirror_pose(right)}
        out.append({"angles": clamp_angles(pose), "runtime": FRAME_MS, "hold": HOLD_MS})
    out.append({"angles": dict(RESET_POSE), "runtime": RISE_MS, "hold": 600})
    return out


def evaluate(job):
    vec, posture = job
    bot = _bot
    bot.respawn()
    bot.wait(0.5)
    tilt0, _ = knock(bot, KNOCK[posture])
    if tilt0 < 60:  # the knock did not work this time: no information
        return {"vec": vec, "score": 999.0, "tilt": tilt0, "standing": False}
    frames = frames_of(vec)
    base = bot.model.body("base_link").id
    for fr in frames[:-1]:
        bot.set_servos_angles(fr["angles"], fr["runtime"])
        bot.wait((fr["runtime"] + fr["hold"]) / 1000)
    tilt1, z1 = bot.torso_tilt_deg(), float(bot.data.xpos[base][2])  # the posture reached before rising
    fr = frames[-1]
    bot.set_servos_angles(fr["angles"], fr["runtime"])
    bot.wait((fr["runtime"] + fr["hold"]) / 1000)
    bot.wait(1.0)
    tilt = bot.torso_tilt_deg()
    z = float(bot.data.xpos[base][2])
    standing = tilt < 15 and z > 0.07
    if standing:
        bot.wait(2.0)
        standing = bot.torso_tilt_deg() < 15
    # reward getting the hips up and the torso upright before the rise, then the final stand
    score = tilt + 0.5 * tilt1 + 200 * max(0.0, 0.10 - z1) + (0 if z > 0.07 else 30) + (0 if standing else 10)
    return {"vec": vec, "score": round(float(score), 2), "tilt": round(float(tilt), 1), "z": round(z, 3),
            "tilt_before_rise": round(float(tilt1), 1), "z_before_rise": round(z1, 3), "standing": bool(standing)}


def random_vec(rng):
    return [rng.uniform(lo, hi) for _ in range(FRAMES) for (_, lo, hi) in PARAMS]


def perturb(vec, rng, sigma):
    out = []
    for k in range(FRAMES):
        for i, (_, lo, hi) in enumerate(PARAMS):
            v = vec[k * len(PARAMS) + i] + rng.gauss(0, sigma)
            out.append(min(max(v, lo), hi))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("posture", nargs="?", default="front", choices=("front", "rear"))
    ap.add_argument("--samples", type=int, default=500)
    ap.add_argument("--climb", type=int, default=400)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    rng = random.Random(args.seed)
    t0 = time.time()
    with mp.Pool(args.jobs, initializer=_init) as pool:
        print(f"stage 1: {args.samples} random candidates ({args.jobs} jobs)", flush=True)
        results = pool.map(evaluate, [(random_vec(rng), args.posture) for _ in range(args.samples)], chunksize=2)
        results.sort(key=lambda r: r["score"])
        for r in results[:5]:
            print(f"  score {r['score']:6.1f} tilt {r['tilt']:5.1f} z {r.get('z', 0):.3f} before rise: tilt "
                  f"{r.get('tilt_before_rise', 0):5.1f} z {r.get('z_before_rise', 0):.3f} standing {r['standing']}")
        best = results[0]
        print(f"stage 2: hill-climbing from the best for {args.climb} candidates", flush=True)
        done = 0
        while done < args.climb:
            batch = [(perturb(best["vec"], rng, 20.0 if best["score"] > 40 else 8.0), args.posture) for _ in range(args.jobs)]
            for r in pool.map(evaluate, batch):
                done += 1
                if r["score"] < best["score"]:
                    best = r
                    print(f"  {done:4d}: score {r['score']:6.1f} tilt {r['tilt']:5.1f} z {r.get('z', 0):.3f} before rise: tilt "
                          f"{r.get('tilt_before_rise', 0):5.1f} z {r.get('z_before_rise', 0):.3f} standing {r['standing']}", flush=True)
    print(f"best after {time.time() - t0:.0f} s: score {best['score']} tilt {best['tilt']} standing {best['standing']}")
    out = args.out or os.path.join(ROOT, "tools", f"getup_{args.posture}_search.json")
    json.dump({"posture": args.posture, "best": best, "top": results[:20], "params": PARAMS, "frames": FRAMES,
               "frame_ms": FRAME_MS}, open(out, "w"), indent=1)
    print(f"search results: {out}")
    if best["standing"]:
        path = os.path.join(ROOT, "motions", "builtin", f"getup_{args.posture}.json")
        motion = {"name": f"getup_{args.posture}", "description": f"Stand up from lying {'face down' if args.posture == 'front' else 'on the back'} "
                  "(found by tools/tune_getup.py in the simulator; not a copy of the robot's own get-up)",
                  "frames": frames_of(best["vec"])[:-1], "end_in_reset": True}
        json.dump(motion, open(path, "w"), indent=2)
        print(f"written {path}")
        return 0
    print("no candidate stood up; Controller.get_up() will fall back to respawn() for this posture")
    return 1


if __name__ == "__main__":
    sys.exit(main())
