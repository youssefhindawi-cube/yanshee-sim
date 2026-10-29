"""Play a get-up (robot .hts or JSON frames) in the sim from face down; report euler-x per frame (robot convention)."""
import sys, os, math, glob, json
sys.path.insert(0, ".")
import numpy as np, mujoco
from yanshee_sim.robot import SimRobot
from yanshee_sim import hts
from yanshee_sim.servos import RESET_POSE, NAMES

def load_frames(name):
    if name.endswith(".json"):
        return json.load(open(name))["frames"]
    f = glob.glob(f"robot_dump/inspect_*/hts/*__{name}.hts")[0]
    out = []
    for fr in hts.load(f)["frames"]:
        a = fr["angles"] if isinstance(fr["angles"], dict) else dict(zip(NAMES, fr["angles"]))
        out.append({"angles": a, "runtime": fr["runtime"], "hold": fr["hold"]})
    return out

def run(frames, posture="face_down", friction=None, quiet=False, kp_scale=1.0):
    bot = SimRobot(viewer=False, realtime=False)
    bot.imu_noise = False
    m, d = bot.model, bot.data
    if friction is not None:
        m.geom_friction[:, 0] = friction
    if kp_scale != 1.0:
        m.actuator_gainprm[:, 0] *= kp_scale; m.actuator_biasprm[:, 1] *= kp_scale
    bot.wait(0.5)
    a = math.radians(-90 if posture == "face_down" else 90) / 2
    with bot._lock:
        d.qpos[3:7] = [math.cos(a), math.sin(a), 0, 0]; d.qpos[2] = 0.5; d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        world = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0]
        bottom = min(d.geom_xpos[i][2] - m.geom_rbound[i] for i in range(m.ngeom) if i not in world and m.geom_contype[i])
        d.qpos[2] += 0.004 - bottom; mujoco.mj_forward(m, d)
    bot.wait(1.5)
    ex = lambda: bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    trace, per = [], []
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        lo, hi = 999, -999
        steps = max(1, round((fr["runtime"] + fr["hold"]) / 50))
        for _ in range(steps):
            bot.wait(0.05); e = ex(); lo, hi = min(lo, e), max(hi, e); trace.append(round(e, 1))
        per.append((i, round(e, 1), round(lo, 1), round(hi, 1)))
    bot.wait(1.5); final = ex()
    bot.close()
    if not quiet:
        for i, e, lo, hi in per:
            print(f"   f{i:2d} end euler-x {e:+7.1f}  (range {lo:+7.1f} .. {hi:+7.1f})")
        print(f"   final euler-x {final:+.1f} -> {'STANDING' if 60 < abs(final) < 120 else 'NOT standing'}")
    return final, per, trace

if __name__ == "__main__":
    for name in sys.argv[1:]:
        print(f"== {name}")
        run(load_frames(name))
