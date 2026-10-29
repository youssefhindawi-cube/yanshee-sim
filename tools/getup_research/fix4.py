"""Fix 4: fix 3's close-while-rising design, searched in a simulator calibrated to the robot's mat failure: hip-roll
servos limited to 0.9 N m (UBTECH's stall figure; at 2 N m the sim never splays), the knees and body gripping the
mat (1.5) and the soles at mat friction. Scored on standing, on the legs NOT splaying (a hip roll pushed outward
past where its close began) and on still working on a smooth floor."""
import sys, os, json, math, random, copy
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import multiprocessing as mp
import fix3
from search_front import LIM
from yanshee_sim.servos import RESET_POSE

# (sole friction, body friction, hip-roll N m, servo gain scale): three mat-like, one smooth floor
CONDS = [(0.6, 1.5, 0.9, 1.0), (1.0, 1.5, 0.9, 1.0), (0.6, 1.5, 0.8, 0.85), (0.2, 1.0, 0.9, 1.0)]
fix3.PARAMS = [("spread9", 0.6, 1.0), ("c10", 0.0, 0.8), ("c11", 0.0, 0.9), ("c12", 0.2, 1.0), ("c13", 0.5, 1.0),
               ("tR", 0.8, 3.0), ("k", 0.9, 1.25), ("raise", 0.0, 12.0), ("alt", 0.0, 1.0), ("deep", -15, 15)]

def build(p):
    fr = fix3.build(p)
    for i in range(10, 15):  # a deeper (+) or higher (-) crouch while the stance closes: hip and knee flexion
        a = fr[i - 1]["angles"]
        for side, sign in (("Right", -1), ("Left", +1)):
            for j, w in (("KneeFlex", 1.0), ("HipFB", 0.5)):
                n = f"{side}{j}"
                a[n] = fix3.clampm(n, a[n] + sign * p["deep"] * w)
    return fr

def run(frames, mu_sole, mu_body, hip_nm, kp):
    import mujoco
    from yanshee_sim.robot import SimRobot
    bot = SimRobot(viewer=False, realtime=False); bot.imu_noise = False
    m, d = bot.model, bot.data
    soles = [m.geom("RightAnkleUD_sole").id, m.geom("LeftAnkleUD_sole").id]
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0:
            m.geom_friction[g, 0] = mu_sole if g in soles else mu_body
    m.geom_friction[m.geom("floor").id, 0] = min(mu_sole, mu_body)
    if kp != 1.0:
        m.actuator_gainprm[:, 0] *= kp; m.actuator_biasprm[:, 1] *= kp
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
    r0, l0 = frames[8]["angles"]["RightHipLR"], frames[8]["angles"]["LeftHipLR"]  # the stance the close starts from
    splay, lo = 0.0, 999.0
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        for _ in range(max(1, round((fr["runtime"] + fr["hold"]) / 20))):
            bot.wait(0.02)
            if i >= 10:
                r, l = math.degrees(d.qpos[hq["RightHipLR"]]), math.degrees(d.qpos[hq["LeftHipLR"]])
                splay = max(splay, r0 - r, l - l0)
            if i >= 13:
                lo = min(lo, bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"])
    bot.wait(1.5)
    final = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    bot.close()
    return final, splay, lo

def job(arg):
    cid, frames, cond = arg
    return (cid, cond) + run(frames, *cond)

def evaluate(pool, cands):
    res = {}
    for cid, cond, *r in pool.imap_unordered(job, [(i, build(p), c) for i, p in enumerate(cands) for c in CONDS], chunksize=2):
        res.setdefault(cid, {})[cond] = r
    out = []
    for i, p in enumerate(cands):
        r = [res[i][c] for c in CONDS]
        stand = sum(60 < abs(f) < 120 for f, _, _ in r)
        splay = max(sp for _, sp, _ in r[:3])  # on the mat-like conditions
        dur = sum(f["runtime"] + f["hold"] for f in build(p)) / 1000
        s = stand * 100 - max(0.0, splay - 8) * 12 - dur * 0.5
        out.append((s, p, r, dur))
    return sorted(out, key=lambda x: -x[0])

if __name__ == "__main__":
    S = os.path.dirname(__file__)
    rnd = random.Random(29)
    orig = {"spread9": 1.0, "c10": 0.0, "c11": 0.0, "c12": 0.0, "c13": 0.0, "tR": 1.0, "k": fix3.K0, "raise": 0.0, "alt": 0.0, "deep": 0.0}
    prev = json.load(open(os.path.join(S, "best_fix3.json")))["params"]; prev = dict(prev, deep=0.0)
    with mp.Pool(22) as pool:
        r0 = evaluate(pool, [orig, prev])
        for s, p, r, dur in r0:
            print(("Base_GetupF+reset" if p is orig else "fix 3 (failed on the robot)"), "finals", [round(x[0]) for x in r], "splay", [round(x[1]) for x in r], flush=True)
        ranked = evaluate(pool, [{n: rnd.uniform(lo, hi) for n, lo, hi in fix3.PARAMS} for _ in range(240)])
        print("stage 1:", [(round(s), sum(60 < abs(x[0]) < 120 for x in r), round(max(x[1] for x in r[:3]))) for s, _, r, _ in ranked[:6]], flush=True)
        seeds = [p for _, p, _, _ in ranked[:6]]
        ranked = evaluate(pool, seeds + [{n: min(max(p[n] + rnd.gauss(0, 0.07 * (hi - lo)), lo), hi) for n, lo, hi in fix3.PARAMS} for p in seeds for _ in range(20)])
    for s, p, r, dur in ranked[:4]:
        print(f"score {s:6.1f} {dur:.1f} s finals {[round(x[0]) for x in r]} splay {[round(x[1]) for x in r]} lowest euler-x after f13 {[round(x[2]) for x in r]}\n    {({k: round(v, 2) for k, v in p.items()})}")
    json.dump({"params": ranked[0][1], "result": ranked[0][2], "frames": build(ranked[0][1])}, open(os.path.join(S, "best_fix4.json"), "w"), indent=1)
    print("saved best_fix4.json")
