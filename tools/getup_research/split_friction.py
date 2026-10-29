"""Sim with separate friction for the soles and for the rest of the robot (knees, shins, hands, torso), to test:
smooth soles slide on the mat (legs splay, a hip roll is driven to its OUTWARD stop) while the knees grip (GetupFront pikes)."""
import sys, os, math, copy
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
import sim_getup as sg
from yanshee_sim.servos import RESET_POSE

def run(frames, mu_sole, mu_body, kp=0.85):
    import mujoco
    from yanshee_sim.robot import SimRobot
    bot = SimRobot(viewer=False, realtime=False); bot.imu_noise = False
    m, d = bot.model, bot.data
    soles = [m.geom("RightAnkleUD_sole").id, m.geom("LeftAnkleUD_sole").id]
    floor = m.geom("floor").id
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0:
            m.geom_friction[g, 0] = mu_sole if g in soles else mu_body
    m.geom_friction[floor, 0] = min(mu_sole, mu_body)  # MuJoCo takes the larger of the pair: the robot geom decides
    if kp != 1.0:
        m.actuator_gainprm[:, 0] *= kp; m.actuator_biasprm[:, 1] *= kp
    hq = {n: m.joint(n).qposadr[0] for n in ("RightHipLR", "LeftHipLR")}
    bot.wait(0.5)
    a = math.radians(-90) / 2
    with bot._lock:
        d.qpos[3:7] = [math.cos(a), math.sin(a), 0, 0]; d.qpos[2] = 0.5; d.qvel[:] = 0; mujoco.mj_forward(m, d)
        world = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0]
        bottom = min(d.geom_xpos[i][2] - m.geom_rbound[i] for i in range(m.ngeom) if i not in world and m.geom_contype[i])
        d.qpos[2] += 0.004 - bottom; mujoco.mj_forward(m, d)
    bot.wait(1.5)
    per, worst = [], {"RightHipLR": 999.0, "LeftHipLR": -999.0}
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        for _ in range(max(1, round((fr["runtime"] + fr["hold"]) / 20))):
            bot.wait(0.02)
            if i >= 12:  # how far the load pushes each hip roll OUTWARD past its command while the stance closes
                tr, tl = fr["angles"]["RightHipLR"], fr["angles"]["LeftHipLR"]
                worst["RightHipLR"] = max(worst["RightHipLR"] if worst["RightHipLR"] < 900 else 0.0, tr - math.degrees(d.qpos[hq["RightHipLR"]]))
                worst["LeftHipLR"] = max(worst["LeftHipLR"] if worst["LeftHipLR"] > -900 else 0.0, math.degrees(d.qpos[hq["LeftHipLR"]]) - tl)
        per.append(round(bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]))
    bot.wait(1.5)
    final = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    bot.close()
    return final, per, {k: round(v) for k, v in worst.items()}

def job(a):
    name, frames, ms, mb = a
    return (name, ms, mb) + run(frames, ms, mb)

if __name__ == "__main__":
    import multiprocessing as mp
    gf = sg.load_frames("GetupFront")
    bf = sg.load_frames("Base_GetupF") + [{"angles": dict(RESET_POSE), "runtime": 1200, "hold": 0}]
    jobs = [(n, f, ms, mb) for n, f in (("Base_GetupF+reset", bf),) for ms in (0.1, 0.2, 0.35, 0.6, 1.0) for mb in (1.0, 2.0)]
    with mp.Pool(16) as pool:
        for name, ms, mb, final, per, worst in pool.map(job, jobs):
            print(f"{name:18s} soles {ms:4.2f} body {mb:3.1f}: {'STAND' if 60 < abs(final) < 120 else 'fail '} {final:+5.0f}  "
                  f"hip rolls pushed out past command R {worst['RightHipLR']:4d} / L {worst['LeftHipLR']:4d} deg  per frame {per}")
