"""Fix 5: search fix 3/4's close-while-rising design in the calibrated sim (hip rolls 1.2 N m, mat soles 1.0, knees
1.5), scored on standing AND on every hip roll being within ~15 deg of its target at the end of each frame from 12
(the robot locked whenever this shortfall stayed at 50-90 deg; the smooth-floor success had under 20)."""
import sys, os, json, random
sys.path.insert(0, "."); sys.path.insert(0, os.path.dirname(__file__))
import multiprocessing as mp
import fix3, fix4, calib

CONDS = [(1.0, 1.5, 1.2), (0.6, 1.5, 1.2), (1.0, 1.5, 1.0), (0.2, 1.0, 1.2)]  # mat x2, weaker mat, smooth floor

def job(arg):
    cid, frames, cond = arg
    return (cid, cond) + calib.run(frames, *cond)

def evaluate(pool, cands):
    res = {}
    for cid, cond, final, short in pool.imap_unordered(job, [(i, fix4.build(p), c) for i, p in enumerate(cands) for c in CONDS], chunksize=2):
        res.setdefault(cid, {})[cond] = (final, short)
    out = []
    for i, p in enumerate(cands):
        r = [res[i][c] for c in CONDS]
        stand = sum(60 < abs(f) < 120 for f, _ in r)
        worst = max(sh for _, sh in r)
        dur = sum(f["runtime"] + f["hold"] for f in fix4.build(p)) / 1000
        s = stand * 100 - max(0.0, worst - 15) * 8 - worst * 0.5 - dur * 0.4
        out.append((s, p, r, dur))
    return sorted(out, key=lambda x: -x[0])

if __name__ == "__main__":
    S = os.path.dirname(__file__)
    rnd = random.Random(41)
    P = fix3.PARAMS
    with mp.Pool(22) as pool:
        ranked = evaluate(pool, [{n: rnd.uniform(lo, hi) for n, lo, hi in P} for _ in range(300)])
        print("stage 1:", [(round(s), sum(60 < abs(x[0]) < 120 for x in r), round(max(x[1] for x in r))) for s, _, r, _ in ranked[:8]], flush=True)
        seeds = [p for _, p, _, _ in ranked[:6]]
        ranked = evaluate(pool, seeds + [{n: min(max(p[n] + rnd.gauss(0, 0.07 * (hi - lo)), lo), hi) for n, lo, hi in P} for p in seeds for _ in range(22)])
        print("stage 2:", [(round(s), sum(60 < abs(x[0]) < 120 for x in r), round(max(x[1] for x in r))) for s, _, r, _ in ranked[:8]], flush=True)
        seeds = [p for _, p, _, _ in ranked[:4]]
        ranked = evaluate(pool, seeds + [{n: min(max(p[n] + rnd.gauss(0, 0.03 * (hi - lo)), lo), hi) for n, lo, hi in P} for p in seeds for _ in range(20)])
    for s, p, r, dur in ranked[:4]:
        print(f"score {s:6.1f} {dur:.1f} s finals {[round(x[0]) for x in r]} hip shortfall {[round(x[1]) for x in r]}\n    {({k: round(v, 2) for k, v in p.items()})}")
    json.dump({"params": ranked[0][1], "result": ranked[0][2], "frames": fix4.build(ranked[0][1])}, open(os.path.join(S, "best_fix5.json"), "w"), indent=1)
    print("saved best_fix5.json")
