"""Fix 3: close Base_GetupF's stance WHILE it rises, sliding the feet in a little every frame (optionally a raised,
lightly loaded foot), and keep every joint clear of its end stops. Measured per run on the corrected-inertia model:
standing, hip-roll peak torque, how near any joint gets to its limit, the load on a foot while its hip roll moves."""
import sys, os, json, math, random, copy
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import multiprocessing as mp
import numpy as np
from search_front import LIM
import sim_getup as sg
from yanshee_sim.servos import RESET_POSE

BASE = sg.load_frames("Base_GetupF")
K0 = 1.09  # ankle roll per hip roll in Base_GetupF (frame 12)
CONDS = [(1.0, 1.0), (1.5, 0.7), (1.5, 1.0), (2.0, 0.7), (2.0, 1.0), (2.0, 0.85)]
MARGIN = 10  # deg: generated targets stay this far inside the servo limits
PARAMS = [("spread9", 0.6, 1.0), ("c10", 0.0, 0.6), ("c11", 0.0, 0.8), ("c12", 0.2, 1.0), ("c13", 0.5, 1.0),
          ("tR", 0.8, 2.5), ("k", 0.9, 1.25), ("raise", 0.0, 12.0), ("alt", 0.0, 1.0)]

def clampm(n, v):
    lo, hi = LIM[n]
    return int(round(min(max(v, lo + MARGIN), hi - MARGIN)))

def build(p):
    fr = copy.deepcopy(BASE)
    for i in range(9, 14):
        a, o = fr[i - 1]["angles"], BASE[i - 1]["angles"]
        c = {9: None, 10: p["c10"], 11: p["c11"], 12: p["c12"], 13: p["c13"]}[i]
        for hip, ank in (("RightHipLR", "RightAnkleUD"), ("LeftHipLR", "LeftAnkleUD")):
            if i == 9:
                h = 90 + p["spread9"] * (o[hip] - 90)
            else:
                start = 90 + p["spread9"] * (o[hip] - 90)  # the narrower stance carried on, then slid in
                h = start + c * (90 - start)
            h = clampm(hip, h)
            a[hip] = h
            a[ank] = clampm(ank, o[ank] - p["k"] * (h - o[hip]))  # relative to Base's own ankle: no jolt
        if i >= 11 and p["raise"] > 0:  # lighten the foot that slides: alternate legs frame by frame (alt) or both
            right = (i % 2 == 0) if p["alt"] > 0.5 else None
            for side, sign in (("Right", -1), ("Left", +1)):
                if right is None or (right and side == "Right") or (right is False and side == "Left"):
                    a[f"{side}KneeFlex"] = clampm(f"{side}KneeFlex", a[f"{side}KneeFlex"] + sign * p["raise"])
                    a[f"{side}HipFB"] = clampm(f"{side}HipFB", a[f"{side}HipFB"] + sign * p["raise"] * 0.5)
        if i >= 10:
            fr[i - 1]["runtime"] = int(round(fr[i - 1]["runtime"] * p["tR"]))
    fr.append({"angles": dict(RESET_POSE), "runtime": 1200, "hold": 0})
    return fr

def run(frames, friction, kp, detail=False):
    import mujoco
    from yanshee_sim.robot import SimRobot
    bot = SimRobot(viewer=False, realtime=False); bot.imu_noise = False
    m, d = bot.model, bot.data
    m.geom_friction[:, 0] = friction
    if kp != 1.0:
        m.actuator_gainprm[:, 0] *= kp; m.actuator_biasprm[:, 1] *= kp
    lim = float(m.actuator_forcerange[m.actuator("RightHipLR").id][1])
    hips = {"R": m.actuator("RightHipLR").id, "L": m.actuator("LeftHipLR").id}
    hipq = {"R": m.joint("RightHipLR").qposadr[0], "L": m.joint("LeftHipLR").qposadr[0]}
    soles = {"R": m.geom("RightAnkleUD_sole").id, "L": m.geom("LeftAnkleUD_sole").id}
    jn = [m.joint(i).name for i in range(m.njnt) if m.joint(i).name in LIM]
    c6 = np.zeros(6)
    def loads():
        f = {"R": 0.0, "L": 0.0}
        for i in range(d.ncon):
            con = d.contact[i]
            for k, g in soles.items():
                if g in (con.geom1, con.geom2):
                    mujoco.mj_contactForce(m, d, i, c6); f[k] += abs(c6[0])
        return f
    bot.wait(0.5)
    a = math.radians(-90) / 2
    with bot._lock:
        d.qpos[3:7] = [math.cos(a), math.sin(a), 0, 0]; d.qpos[2] = 0.5; d.qvel[:] = 0; mujoco.mj_forward(m, d)
        world = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0]
        bottom = min(d.geom_xpos[i][2] - m.geom_rbound[i] for i in range(m.ngeom) if i not in world and m.geom_contype[i])
        d.qpos[2] += 0.004 - bottom; mujoco.mj_forward(m, d)
    bot.wait(1.5)
    peak, near, drag, per = 0.0, 999.0, 0.0, []
    prev = {k: d.qpos[hipq[k]] for k in hipq}
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        for _ in range(max(1, round((fr["runtime"] + fr["hold"]) / 20))):
            bot.wait(0.02)
            if i >= 9:
                tq = {k: abs(float(d.actuator_force[hips[k]])) / lim for k in hips}
                peak = max(peak, max(tq.values()))
                for n in jn:
                    lo, hi = LIM[n]; q = math.degrees(d.qpos[m.joint(n).qposadr[0]])  # servo deg = joint angle in deg
                    near = min(near, q - lo, hi - q) if n.endswith("HipLR") else near
                f = loads()
                for k in hips:  # load on a foot whose hip roll is moving: dragging a loaded foot
                    v = abs(d.qpos[hipq[k]] - prev[k]) / 0.02
                    if v > 0.3:
                        drag += f[k] * v * 0.02
                    prev[k] = d.qpos[hipq[k]]
        if detail:
            per.append(round(bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"], 1))
    bot.wait(1.5)
    final = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    bot.close()
    return final, peak, near, drag, per

def job(arg):
    cid, frames, cond = arg
    return (cid, cond) + run(frames, *cond)[:4]

def evaluate(pool, cands):
    res = {}
    for cid, cond, *r in pool.imap_unordered(job, [(i, build(p), c) for i, p in enumerate(cands) for c in CONDS], chunksize=2):
        res.setdefault(cid, {})[cond] = r
    out = []
    for i, p in enumerate(cands):
        r = [res[i][c] for c in CONDS]
        stand = sum(60 < abs(f) < 120 for f, *_ in r)
        peak = max(x[1] for x in r); drag = max(x[3] for x in r)
        dur = sum(f["runtime"] + f["hold"] for f in build(p)) / 1000
        s = stand * 100 - max(0.0, peak - 0.55) * 400 - peak * 30 - drag * 0.6 - dur * 0.5
        out.append((s, p, r, dur))
    return sorted(out, key=lambda x: -x[0])

if __name__ == "__main__":
    S = os.path.dirname(__file__)
    rnd = random.Random(17)
    with mp.Pool(22) as pool:
        orig = {"spread9": 1.0, "c10": 0.0, "c11": 0.0, "c12": 0.0, "c13": 0.0, "tR": 1.0, "k": K0, "raise": 0.0, "alt": 0.0}
        base_r = evaluate(pool, [orig])[0]
        print("Base_GetupF (generated, margins applied) + reset:", [round(x[0]) for x in base_r[2]], "hip peak",
              f"{max(x[1] for x in base_r[2]):.0%}", "drag", round(max(x[3] for x in base_r[2]), 1), flush=True)
        ranked = evaluate(pool, [{n: rnd.uniform(lo, hi) for n, lo, hi in PARAMS} for _ in range(220)])
        print("stage 1:", [(round(s), sum(60 < abs(x[0]) < 120 for x in r), f"{max(x[1] for x in r):.0%}", round(max(x[3] for x in r), 1)) for s, _, r, _ in ranked[:6]], flush=True)
        seeds = [p for _, p, _, _ in ranked[:6]]
        ranked = evaluate(pool, seeds + [{n: min(max(p[n] + rnd.gauss(0, 0.07 * (hi - lo)), lo), hi) for n, lo, hi in PARAMS} for p in seeds for _ in range(20)])
    for s, p, r, dur in ranked[:4]:
        print(f"score {s:6.1f} {dur:.1f} s finals {[round(x[0]) for x in r]} hip peak {[f'{x[1]:.0%}' for x in r]} "
              f"drag {[round(x[3], 1) for x in r]}\n    {({k: round(v, 2) for k, v in p.items()})}")
    json.dump({"params": ranked[0][1], "result": ranked[0][2], "frames": build(ranked[0][1])}, open(os.path.join(S, "best_fix3.json"), "w"), indent=1)
    print("saved best_fix3.json")
