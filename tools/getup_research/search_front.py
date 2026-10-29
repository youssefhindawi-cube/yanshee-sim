"""Search small changes to the robot's GetupFront that stand up across floor-grip / servo-strength conditions,
including the two that reproduce the real robot's failure (friction 1.5-2.0, servos at 70 %)."""
import sys, os, json, random, math, copy
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import multiprocessing as mp
from yanshee_sim.servos import SERVOS
LIM = {s[1]: (s[2], s[3]) for s in SERVOS}
CONDS = [(1.0, 1.0), (1.5, 0.7), (2.0, 0.7), (2.0, 1.0), (2.5, 0.85), (1.0, 0.6)]
# name, lo, hi
PARAMS = [("tA", 0.8, 2.0), ("tB", 0.8, 2.0), ("hipA", -25, 25), ("kneeA", -25, 25), ("ankA", -25, 25),
          ("hipB", -25, 25), ("kneeB", -25, 25), ("ankB", -25, 25), ("shF", -30, 30), ("hold8", 0, 1500)]
PAIRS = {"hip": ("RightHipFB", "LeftHipFB"), "knee": ("RightKneeFlex", "LeftKneeFlex"),
         "ank": ("RightAnkleFB", "LeftAnkleFB"), "sh": ("RightShoulderFlex", "LeftShoulderFlex")}

def build(base, p):
    fr = copy.deepcopy(base)
    def add(frames, pair, d):
        r, l = PAIRS[pair]
        for i in frames:
            a = fr[i - 1]["angles"]
            a[r] = min(max(a[r] + d, LIM[r][0]), LIM[r][1]); a[l] = min(max(a[l] - d, LIM[l][0]), LIM[l][1])
    add(range(5, 9), "hip", p["hipA"]); add(range(5, 9), "knee", p["kneeA"]); add(range(5, 9), "ank", p["ankA"])
    add(range(9, 12), "hip", p["hipB"]); add(range(9, 12), "knee", p["kneeB"]); add(range(9, 12), "ank", p["ankB"])
    add(range(7, 10), "sh", p["shF"])
    for i in range(5, 9):
        fr[i - 1]["runtime"] = round(fr[i - 1]["runtime"] * p["tA"])
    for i in range(9, 12):
        fr[i - 1]["runtime"] = round(fr[i - 1]["runtime"] * p["tB"])
    fr[7]["hold"] += round(p["hold8"])
    for f in fr:
        f["angles"] = {k: int(round(v)) for k, v in f["angles"].items()}
    return fr

def job(arg):
    import sim_getup as sg
    cid, frames, cond = arg
    final, per, trace = sg.run(frames, friction=cond[0], quiet=True, kp_scale=cond[1])
    return cid, cond, final

def score(finals, p):
    stand = sum(60 < abs(f) < 120 for f in finals)
    close = sum(-min(abs(f - 90), 90) for f in finals) / len(finals)
    size = sum(abs(p[k]) for k in ("hipA", "kneeA", "ankA", "hipB", "kneeB", "ankB", "shF"))
    slow = (p["tA"] - 1) * 4 + (p["tB"] - 1) * 3 + p["hold8"] / 1000
    return stand * 100 + close * 0.3 - size * 0.15 - slow * 2

def evaluate(pool, base, cands):
    jobs = [(i, build(base, p), c) for i, p in enumerate(cands) for c in CONDS]
    res = {}
    for cid, cond, final in pool.imap_unordered(job, jobs, chunksize=4):
        res.setdefault(cid, {})[cond] = final
    out = []
    for i, p in enumerate(cands):
        finals = [res[i][c] for c in CONDS]
        out.append((score(finals, p), p, finals))
    return sorted(out, key=lambda x: -x[0])

if __name__ == "__main__":
    import sim_getup as sg
    base = sg.load_frames("GetupFront")
    rnd = random.Random(7)
    zero = {"tA": 1.0, "tB": 1.0, "hipA": 0, "kneeA": 0, "ankA": 0, "hipB": 0, "kneeB": 0, "ankB": 0, "shF": 0, "hold8": 0}
    def rand():
        return {n: rnd.uniform(lo, hi) for n, lo, hi in PARAMS}
    def perturb(p, s):
        q = dict(p)
        for n, lo, hi in PARAMS:
            q[n] = min(max(q[n] + rnd.gauss(0, s * (hi - lo)), lo), hi)
        return q
    with mp.Pool(22) as pool:
        ranked = evaluate(pool, base, [zero] + [rand() for _ in range(500)])
        print("stage 1 best:", [(round(s), sum(60 < abs(f) < 120 for f in fin)) for s, _, fin in ranked[:8]], flush=True)
        seeds = [p for _, p, _ in ranked[:10]]
        ranked = evaluate(pool, base, seeds + [perturb(p, 0.12) for p in seeds for _ in range(35)])
        print("stage 2 best:", [(round(s), sum(60 < abs(f) < 120 for f in fin)) for s, _, fin in ranked[:8]], flush=True)
        seeds = [p for _, p, _ in ranked[:4]]
        ranked = evaluate(pool, base, seeds + [perturb(p, 0.05) for p in seeds for _ in range(50)])
    for s, p, fin in ranked[:5]:
        print(f"score {s:7.1f} stands {sum(60 < abs(f) < 120 for f in fin)}/{len(CONDS)} finals {[round(f) for f in fin]}")
        print("   ", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in p.items()})
    best = ranked[0][1]
    json.dump({"params": best, "conds": CONDS, "finals": ranked[0][2], "frames": build(base, best)},
              open(os.path.join(os.path.dirname(__file__), "best_front.json"), "w"), indent=1)
    print("saved best_front.json")
