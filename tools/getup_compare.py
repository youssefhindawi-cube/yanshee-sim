#!/usr/bin/env python3
"""Replay a real get-up run in the simulator and compare the two IMU traces (euler-x over time).

    python3 tools/getup_compare.py robot_dump/getup_GetupFront_20261029_134112.json          # smooth floor
    python3 tools/getup_compare.py robot_dump/getup_GetupFront_20261029_134112.json --mat    # the play mat
    python3 tools/getup_compare.py LOG --torque 1.3 --friction 1.2 --armature 0.0005          # what-if servo / floor

LOG is a file written by tools/getup_lab.py. The simulator lies down the way the log says (face down / on its back),
plays the same .hts (from robot_dump/inspect_*/hts, copied by `robot_checks.py inspect`) on time, as the robot's
own motion engine does, then resets. It prints whether each one stood up and the RMS difference of euler-x;
--trace prints both every 0.5 s. The options override the model's servo and floor values for this run only, which
is how SERVO_TORQUE and MAT_FRICTION in tools/build_models.py were fitted.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from yanshee_sim import hts  # noqa: E402
from yanshee_sim.robot import SCENE, SCENE_MAT, SimRobot  # noqa: E402
from yanshee_sim.servos import NAMES  # noqa: E402
import tune_getup  # noqa: E402

DT = 0.05  # s between simulator IMU samples


def robot_trace(log):
    """(log dict, t, euler-x) of a getup_lab log (old logs keep the samples under "imu")."""
    d = json.load(open(log))
    t, x = [], []
    for f in d["frames"]:
        for s in f.get("samples") or f.get("imu") or []:
            t.append(s["t"])
            x.append(s["euler-x"])
    return d, np.array(t), np.array(x)


def hts_frames(name):
    paths = glob.glob(os.path.join(ROOT, "robot_dump", "inspect_*", "hts", f"*move__hts__{name}.hts"))
    if not paths:
        raise SystemExit(f"no .hts for {name!r} in robot_dump/inspect_*/hts: run tools/robot_checks.py inspect")
    return hts.load(paths[0])["frames"]


def spec_hook(args):
    def hook(spec):
        import mujoco
        for j in spec.joints:
            if j.type == mujoco.mjtJoint.mjJNT_HINGE:
                if args.armature is not None:
                    j.armature = args.armature
                if args.damping is not None:
                    j.damping = args.damping
        for a in spec.actuators:
            if args.torque is not None:
                a.forcerange = [-args.torque, args.torque]
            if args.kp is not None:
                a.gainprm[0], a.biasprm[1] = args.kp, -args.kp
        if args.friction is not None:
            spec.geom("floor").friction = [args.friction, 0.02, 0.001]
    return hook


def simulate(args, frames, posture):
    """Sim euler-x every DT s while playing frames from lying in posture, then 1.5 s after a reset."""
    bot = SimRobot(viewer=False, realtime=False, scene=SCENE_MAT if args.mat else SCENE, spec_hook=spec_hook(args))
    bot.strict = False
    tune_getup.knock(bot, +1 if posture == "face_down" else -1)
    t, x = [], []

    def run(seconds):
        for _ in range(int(round(seconds / DT))):
            bot.wait(DT)
            t.append(len(t) * DT + DT)
            x.append(bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"])

    for fr in frames:
        angles = fr["angles"] if isinstance(fr["angles"], dict) else dict(zip(NAMES, fr["angles"]))
        bot.set_servos_angles({k: int(v) for k, v in angles.items()}, int(fr["runtime"]))
        run((int(fr["runtime"]) + int(fr["hold"])) / 1000)
    bot.reset(1000)
    run(1.5)
    return np.array(t), np.array(x)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="a robot_dump/getup_*.json from tools/getup_lab.py")
    ap.add_argument("--mat", action="store_true", help="simulate on the play mat (model/scene_mat.xml)")
    ap.add_argument("--friction", type=float, help="floor friction instead of the scene's")
    ap.add_argument("--torque", type=float, help="servo torque limit, N*m")
    ap.add_argument("--kp", type=float, help="servo position gain, N*m/rad")
    ap.add_argument("--armature", type=float, help="servo reflected rotor inertia, kg*m^2")
    ap.add_argument("--damping", type=float, help="joint damping, N*m*s/rad")
    ap.add_argument("--trace", action="store_true", help="print both euler-x traces every 0.5 s")
    args = ap.parse_args(argv)
    d, rt, rx = robot_trace(args.log)
    motion = d["motion"]
    if motion.endswith(".json"):
        frames = json.load(open(os.path.join(ROOT, motion)))["frames"]
    else:
        frames = hts_frames(motion)
    st, sx = simulate(args, frames, d["posture"])
    unwrap = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731  (on the back euler-x wraps at +-180)
    rms = np.sqrt(np.mean((np.interp(rt, st, unwrap(sx)) - unwrap(rx)) ** 2)) if len(rt) else float("nan")
    stood = 60 < abs(sx[-1]) < 120
    print(f"{motion}: robot {'stood up' if d['stood_up'] else 'did not stand up'} (battery {d['battery'].get('voltage')} mV),"
          f" sim {'stood up' if stood else 'did not stand up'};  euler-x RMS difference {rms:.1f} deg,"
          f" lowest euler-x robot {rx.min():.1f} / sim {sx.min():.1f}")
    if args.trace:
        for tt in np.arange(0, st[-1], 0.5):
            print(f"  t={tt:5.1f}  sim {sx[np.argmin(abs(st - tt))]:7.1f}  robot {rx[np.argmin(abs(rt - tt))]:7.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
