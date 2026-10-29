"""Fix 2: Base_GetupF frames 1-12 unchanged (they get the robot from face down into a wide upright crouch), then
close the stance one foot at a time: slide the pelvis over one foot (both hip rolls turn together, feet planted),
bring the unloaded foot in (slightly lifted), re-centre, repeat on the other side; then Base's end pose and reset.
Ankle rolls follow the hip rolls so the soles stay flat (the ratio Base_GetupF itself uses at frame 12)."""
import sys, os, json, math, random, copy
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import multiprocessing as mp
from search_front import LIM
import sim_getup as sg
from yanshee_sim.servos import RESET_POSE

BASE = sg.load_frames("Base_GetupF")
F12, F13, F14 = BASE[11]["angles"], BASE[12]["angles"], BASE[13]["angles"]
FB = ("RightHipFB", "RightKneeFlex", "RightAnkleFB", "LeftHipFB", "LeftKneeFlex", "LeftAnkleFB")
K_ANKLE = (F12["RightAnkleUD"] - 90) / -(F12["RightHipLR"] - 90)  # about 1.09: ankle roll per hip roll
CONDS = [(1.0, 1.0), (1.5, 0.7), (2.0, 0.7), (2.0, 1.0), (2.5, 0.85), (2.5, 1.0)]
PARAMS = [("shift", 8, 35), ("passes", 1, 2.99), ("lift", 0, 25), ("t_shift", 400, 1500), ("t_move", 400, 1500),
          ("hold", 0, 500), ("k", 0.8, 1.3), ("upright", 0, 1), ("pitch", -15, 15)]

def clamp(a):
    return {n: int(round(min(max(v, LIM[n][0]), LIM[n][1]))) for n, v in a.items()}

UPRIGHT, PITCH = 0.0, 0.0  # set per candidate by closing()


def pose(hR, hL, k, liftR=0.0, liftL=0.0):
    a = dict(F12)
    for n in FB:  # the crouch held while closing: frame 12 (leaning forward) .. frame 13 (upright)
        a[n] = F12[n] + UPRIGHT * (F13[n] - F12[n])
    a["RightAnkleFB"] += PITCH; a["LeftAnkleFB"] -= PITCH  # lean the shins forward or back (mirrored)
    a["RightHipLR"], a["LeftHipLR"] = hR, hL
    a["RightAnkleUD"], a["LeftAnkleUD"] = 90 - k * (hR - 90), 90 - k * (hL - 90)
    # lift a foot: more knee and hip flexion on that leg (right decreases, left increases, as from reset to F12)
    a["RightKneeFlex"] -= liftR; a["RightHipFB"] -= liftR * 0.5
    a["LeftKneeFlex"] += liftL; a["LeftHipFB"] += liftL * 0.5
    return clamp(a)

def closing(p):
    global UPRIGHT, PITCH
    UPRIGHT, PITCH = p.get("upright", 0.0), p.get("pitch", 0.0)
    hR, hL = float(F12["RightHipLR"]), float(F12["LeftHipLR"])  # 35, 145: 55 deg out each
    passes = int(p["passes"])
    step = (90 - hR) / passes  # how far each foot comes in per pass
    s, k, fr = p["shift"], p["k"], []
    add = lambda a, t: fr.append({"angles": a, "runtime": int(t), "hold": int(p["hold"])})  # noqa: E731
    for _ in range(passes):
        # weight over the right foot: the pelvis slides right (both hip rolls +s), the left foot comes in, lifted
        add(pose(hR + s, hL + s, k), p["t_shift"])
        add(pose(hR + s, hL + s - step, k, liftL=p["lift"]), p["t_move"])
        add(pose(hR + s, hL + s - step, k), p["t_move"] * 0.5)
        hL -= step
        add(pose(hR, hL, k), p["t_shift"])
        # weight over the left foot, the right foot comes in
        add(pose(hR - s, hL - s, k), p["t_shift"])
        add(pose(hR - s + step, hL - s, k, liftR=p["lift"]), p["t_move"])
        add(pose(hR - s + step, hL - s, k), p["t_move"] * 0.5)
        hR += step
        add(pose(hR, hL, k), p["t_shift"])
    add(clamp(dict(F14)), 1000)
    add(dict(RESET_POSE), 1200)
    return fr

def build(p):
    return copy.deepcopy(BASE[:12]) + closing(p)

def run(frames, friction, kp):
    import mujoco
    from yanshee_sim.robot import SimRobot
    bot = SimRobot(viewer=False, realtime=False); bot.imu_noise = False
    m, d = bot.model, bot.data
    m.geom_friction[:, 0] = friction
    if kp != 1.0:
        m.actuator_gainprm[:, 0] *= kp; m.actuator_biasprm[:, 1] *= kp
    lim = float(m.actuator_forcerange[m.actuator("RightHipLR").id][1])
    ids = [m.actuator(n).id for n in ("RightHipLR", "LeftHipLR")]
    bot.wait(0.5)
    a = math.radians(-90) / 2
    with bot._lock:
        d.qpos[3:7] = [math.cos(a), math.sin(a), 0, 0]; d.qpos[2] = 0.5; d.qvel[:] = 0; mujoco.mj_forward(m, d)
        world = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0]
        bottom = min(d.geom_xpos[i][2] - m.geom_rbound[i] for i in range(m.ngeom) if i not in world and m.geom_contype[i])
        d.qpos[2] += 0.004 - bottom; mujoco.mj_forward(m, d)
    bot.wait(1.5)
    peak, lo, hi, per = 0.0, 999.0, -999.0, []
    for i, fr in enumerate(frames, 1):
        bot.set_servos_angles({k: v for k, v in fr["angles"].items() if k in RESET_POSE}, max(200, fr["runtime"]))
        for _ in range(max(1, round((fr["runtime"] + fr["hold"]) / 20))):
            bot.wait(0.02)
            if i > 12:
                peak = max(peak, max(abs(float(d.actuator_force[j])) for j in ids) / lim)
                e = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]; lo, hi = min(lo, e), max(hi, e)
        per.append(round(bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"], 1))
    bot.wait(1.5)
    final = bot.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
    bot.close()
    return final, peak, lo, hi, per

def job(arg):
    cid, frames, cond = arg
    return (cid, cond) + run(frames, *cond)[:4]

def evaluate(pool, cands):
    res = {}
    for cid, cond, final, peak, lo, hi in pool.imap_unordered(job, [(i, build(p), c) for i, p in enumerate(cands) for c in CONDS], chunksize=2):
        res.setdefault(cid, {})[cond] = (final, peak, lo, hi)
    out = []
    for i, p in enumerate(cands):
        r = [res[i][c] for c in CONDS]
        stand = sum(60 < abs(f) < 120 for f, _, _, _ in r)
        worst = max(pk for _, pk, _, _ in r)
        sway = max(max(abs(lo - 92), abs(hi - 92)) for _, _, lo, hi in r)  # tipping during the close
        dur = sum(f["runtime"] + f["hold"] for f in closing(p)) / 1000
        s = stand * 100 - max(0.0, worst - 0.6) * 300 - worst * 30 - max(0.0, sway - 25) * 2 - dur * 0.5
        out.append((s, p, r, dur))
    return sorted(out, key=lambda x: -x[0])

if __name__ == "__main__":
    S = os.path.dirname(__file__)
    rnd = random.Random(3)
    with mp.Pool(22) as pool:
        ranked = evaluate(pool, [{n: rnd.uniform(lo, hi) for n, lo, hi in PARAMS} for _ in range(150)])
        print("stage 1 best:", [(round(s), sum(60 < abs(x[0]) < 120 for x in r), f"{max(x[1] for x in r):.0%}") for s, _, r, _ in ranked[:6]], flush=True)
        seeds = [p for _, p, _, _ in ranked[:5]]
        ranked = evaluate(pool, seeds + [{n: min(max(p[n] + rnd.gauss(0, 0.08 * (hi - lo)), lo), hi) for n, lo, hi in PARAMS} for p in seeds for _ in range(18)])
    for s, p, r, dur in ranked[:4]:
        print(f"score {s:6.1f} close {dur:.1f} s  finals {[round(x[0]) for x in r]}  hip-roll peak {[f'{x[1]:.0%}' for x in r]}  "
              f"euler-x during close {min(x[2] for x in r):.0f}..{max(x[3] for x in r):.0f}\n    {({k: round(v, 2) for k, v in p.items()})}")
    best = ranked[0][1]
    json.dump({"params": best, "conds": CONDS, "result": ranked[0][2], "frames": build(best)}, open(os.path.join(S, "best_fix2.json"), "w"), indent=1)
    print("saved best_fix2.json")
