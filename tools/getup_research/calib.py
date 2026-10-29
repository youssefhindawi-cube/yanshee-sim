"""Moved from a session scratchpad on 2026-10-29; run from the repo root, e.g. MUJOCO_GL=egl python3 tools/getup_research/calib.py.
Calibrate the sim to the robot's records: at the END of each closing frame (12 on), how far is each hip roll from
its target, outward? Robot, mat: Base_GetupF 53-92 deg, fix 3 62-86 deg (then locked); smooth floor: under 20."""
import sys, os, math, json
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import sim_getup as sg
from yanshee_sim.servos import RESET_POSE

def run(frames, mu_sole, mu_body, hip_nm, all_nm=None):
    import mujoco
    from yanshee_sim.robot import SimRobot
    bot = SimRobot(viewer=False, realtime=False); bot.imu_noise = False
    m, d = bot.model, bot.data
    soles = [m.geom("RightAnkleUD_sole").id, m.geom("LeftAnkleUD_sole").id]
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0:
            m.geom_friction[g, 0] = mu_sole if g in soles else mu_body
    m.geom_friction[m.geom("floor").id, 0] = min(mu_sole, mu_body)
    if all_nm:
        m.actuator_forcerange[:] = [[-all_nm, all_nm]] * m.nu
    for n in ("RightHipLR", "LeftHipLR"):
        m.actuator_forcerange[m.actuator(n).id] = [-hip_nm, hip_nm]
    hq = {n: m.joint(n).qposadr[0] for n in ("RightHipLR", "LeftHipLR")}
    bot.wait(0.5)
    q = math.radians(-90) / 2
    with bot._lock:
        d.qpos[3:7] = [math.cos(q), math.sin(q), 0, 0]; d.qpos[2] = 0.5; d.qvel[:] = 0; mujoco.mj_forward(m, d)
        world = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0]
        bottom = min(d.geom_xpos[i][2] - m.geom_rbound[i] for i in range(m.ngeom) if i not in world and m.geom_contype[i])
        d.qpos[2] += 0.004 - bottom; mujoco.mj_forward(m, d)
    bot.wait(1.5)
    short = 0.0
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        bot.wait((fr["runtime"] + fr["hold"]) / 1000)
        if i >= 12:
            r, l = math.degrees(d.qpos[hq["RightHipLR"]]), math.degrees(d.qpos[hq["LeftHipLR"]])
            short = max(short, fr["angles"]["RightHipLR"] - r, l - fr["angles"]["LeftHipLR"])
    bot.wait(1.5)
    final = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    bot.close()
    return final, short

def job(a):
    name, fr, ms, mb, nm, alln = a
    return (name, ms, mb, nm, alln) + run(fr, ms, mb, nm, alln)

if __name__ == "__main__":
    import multiprocessing as mp
    S = os.path.dirname(__file__)
    base = sg.load_frames("Base_GetupF") + [{"angles": dict(RESET_POSE), "runtime": 1200, "hold": 0}]
    f3 = json.load(open(os.path.join(S, "best_fix3.json")))["frames"]
    jobs = [(n, f, ms, mb, nm, alln) for n, f in (("Base_GetupF", base), ("fix 3", f3))
            for ms, mb in ((0.2, 1.0), (0.6, 1.5), (1.0, 1.5)) for nm in (1.2, 0.9, 0.7, 0.5) for alln in (None, 1.0)]
    with mp.Pool(22) as pool:
        res = pool.map(job, jobs)
    for alln in (None, 1.0):
        print(f"\nother servos at {'2.0 N m (model)' if alln is None else '1.0 N m'}: hip-roll shortfall at frame ends (deg) / stands?  rows: soles/body friction; columns hip-roll N m 1.2, 0.9, 0.7, 0.5")
        for name in ("Base_GetupF", "fix 3"):
            for ms, mb in ((0.2, 1.0), (0.6, 1.5), (1.0, 1.5)):
                row = [r for r in res if r[0] == name and r[1] == ms and r[2] == mb and r[4] == alln]
                print(f"  {name:11s} {ms:3.1f}/{mb:3.1f}  " + "   ".join(f"{r[6]:4.0f} {'S' if 60 < abs(r[5]) < 120 else 'x'}" for r in row))
    print("\nrobot: mat Base_GetupF 53-92 / fix 3 62-86 (both lock); smooth floor Base_GetupF < 20, stands")
