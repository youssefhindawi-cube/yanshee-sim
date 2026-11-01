#!/usr/bin/env python3
"""Replay the robot's own walks in the simulator, offline, at the robot's timestamps, and compare body motion.

The live shadow simulator got the firmware's servo frames over Wi-Fi (late and bunched); this feeds the same frames
from the robot's own log (robot clock) into a headless simulator stepped as fast as it goes, starting from the robot's
encoder pose, and compares the simulator's IMU with the robot's: pitch and roll error, who falls. Model knobs can be
swept to see which ones close the gap.

    python3 tools/sim2real_replay.py robot_dump/sim2real_20261029_165523.jsonl
    python3 tools/sim2real_replay.py SESSION --sweep          # friction x servo gain x damping x torque grid
    python3 tools/sim2real_replay.py SESSION --friction 1.5 --kp 2 --damping 1 --torque 1.15

Knobs (applied to the loaded model, not to model/yanshee.xml): friction = scale on every geom's sliding friction,
kp = scale on the servos' position gain (8 N m/rad), damping = scale on joint damping (0.08), torque = servo limit N m.
"""
import argparse
import bisect
import collections
import itertools
import json
import math
import os
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
os.environ.setdefault("MUJOCO_GL", "egl")
from sim2real_report import load, tilt  # noqa: E402
from yanshee_sim.servos import NAMES  # noqa: E402

SETTLE_S = 0.5  # the simulator stands this long in the robot's pose before the first frame
AFTER_S = 3.0   # and keeps going this long after the last frame
SAMPLE_S = 0.05


LOOPING = ("TurnLeft", "TurnRight")  # started with repeat 0 by AppWalk: hal loops them until the next motion


def motion_frames(events, t0, t1):
    """hal's motion starts/ends in a walk ([(t, robot path, status)]) as servo frames [(t - t0, angles, ms)], played
    from the local copies of the robot's .hts files (yanshee_sim.robot_stream.hts_copy)."""
    from yanshee_sim import hts
    from yanshee_sim.robot_stream import hts_copy
    out = []
    for k, (t, path, status) in enumerate(events):
        if status != "start":
            continue
        end = next((e[0] for e in events[k + 1:]), t1)
        try:
            frames = hts.load(hts_copy(path, None))["frames"]
        except Exception as e:  # noqa: BLE001 - not copied yet (needs the robot): skip that motion
            print(f"   (no local copy of {path}: {e})")
            continue
        name, tt = os.path.splitext(os.path.basename(path))[0], t
        while tt < end:
            for f in frames:
                if tt >= end:
                    break
                out.append((tt - t0, f["angles"], f["runtime"]))
                tt += (f["runtime"] + f.get("hold", 0)) / 1000.0
            if name not in LOOPING:
                break
    return out


def walks(path):
    """The session's walks driven by servo frames or hal's .hts motions (turns): [{"i", "method", "t0", "frames",
    "imu", "pose"}] (laptop clock)."""
    rows, rrows, _, _, _ = load(path)
    by = collections.defaultdict(list)
    for r in sorted(rows + rrows, key=lambda r: r["t"]):
        by[r["kind"]].append(r)
    resets, end = by["shadow_reset"], max(r["t"] for r in rows + rrows)
    enc_t = [e["t"] for e in by["enc"]]
    out = []
    for i, rs in enumerate(resets):
        t0, t1 = rs["t"], resets[i + 1]["t"] if i + 1 < len(resets) else end
        frames = [(f["t"] - t0, f["angles"], f.get("ms") or 20) for f in by["frame"] if t0 <= f["t"] < t1]
        events = [(r["t"], r["msg"].split("[", 1)[1].split("]", 1)[0], r["msg"].split("status[", 1)[1].split("]", 1)[0])
                  for r in by["rosout"] if t0 <= r["t"] < t1 and r.get("msg", "").startswith("Publish action[")]
        events = [e for e in events if "Getup" not in e[1]]  # not the firmware's get-up after a fall
        frames = sorted(frames + motion_frames(events, t0, t1), key=lambda f: f[0])
        if len(frames) < 5:
            continue  # nothing moved
        t1 = min(t1, t0 + frames[-1][0] + AFTER_S)
        before = [r for r in by["imu"] if t0 - 1.0 <= r["t"] < t0]
        j = bisect.bisect_right(enc_t, t0) - 1
        pose = dict(zip(NAMES, by["enc"][j]["s"])) if j >= 0 else rs.get("pose")
        out.append({"i": i + 1, "method": rs.get("method"), "t0": t0, "t1": t1, "frames": frames,
                    "turned": any("Turn" in e[1] for e in events),
                    "imu": [(r["t"] - t0, r["e"][0], r["e"][1]) for r in by["imu"] if t0 <= r["t"] < t1],
                    "heading": [(r["t"] - t0, r["e"][2]) for r in by["imu"] if t0 <= r["t"] < t1],
                    "ref": (statistics.median(r["e"][0] for r in before), statistics.median(r["e"][1] for r in before))
                    if before else (92.0, 0.0), "pose": pose})
    return out


def make_sim(friction=1.0, kp=1.0, damping=1.0, torque=None, frictionloss=None, torsion=None):
    from yanshee_sim.robot import SimRobot
    sim = SimRobot(viewer=False, realtime=False)
    sim.imu_noise = False
    m = sim.model
    m.geom_friction[:, 0] *= friction
    if torsion is not None:
        m.geom_friction[:, 1] = torsion
    m.actuator_gainprm[:, 0] *= kp
    m.actuator_biasprm[:, 1] *= kp
    for n in sim._kp:  # the simulator restores these on every respawn
        sim._kp[n] *= kp
        sim._bias[n] = sim._bias[n].copy()
        sim._bias[n][0] *= kp
    m.dof_damping[6:] *= damping
    if frictionloss is not None:
        m.dof_frictionloss[6:] = frictionloss
    if torque:
        m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1] = -torque, torque
    return sim


def replay(walk, knobs, sim=None):
    """Sim IMU (t, pitch, roll) relative to its standing lean, and when it fell (None)."""
    own = sim is None
    sim = sim or make_sim(**knobs)
    try:
        sim.respawn()
        sim.jump_to({n: d for n, d in (walk["pose"] or {}).items() if n in NAMES})
        sim.wait(SETTLE_S)
        g = sim.get_sensors_gyro()["data"]["gyro"][0]
        ref = (g["euler-x"], g["euler-y"])
        out, t, k, fell = [], 0.0, 0, None
        end = walk["frames"][-1][0] + AFTER_S
        while t < end:
            while k < len(walk["frames"]) and walk["frames"][k][0] <= t:
                _, angles, ms = walk["frames"][k]
                sim._send(angles, max(20, ms))
                k += 1
            sim.wait(SAMPLE_S)
            t += SAMPLE_S
            sim._imu_hold = None
            g = sim.get_sensors_gyro()["data"]["gyro"][0]
            p, r = g["euler-x"] - ref[0], g["euler-y"] - ref[1]
            out.append((t, p, r, g["euler-z"]))
            if fell is None and sim.torso_tilt_deg() > 50:
                fell = t
        return out, fell
    finally:
        if own:
            sim.close()


def compare(walk, sim_series, sim_fell):
    rx, ry = walk["ref"]
    robot = [(t, ex - rx, ey - ry) for t, ex, ey in walk["imu"]]
    r_fell = next((t for t, p, r in robot if tilt(p + 92, r) > 50), None)
    cut = min(x for x in (r_fell, sim_fell, robot[-1][0] if robot else 0) if x is not None)
    st = [s[0] for s in sim_series]
    ep, er = [], []
    for t, p, r in robot:
        if t > cut:
            break
        j = min(bisect.bisect_left(st, t), len(st) - 1)
        ep.append(p - sim_series[j][1])
        er.append(r - sim_series[j][2])
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")  # noqa: E731
    turn = lambda a, b: ((b - a + 180) % 360) - 180  # noqa: E731
    hd = [h for t, h in walk.get("heading", []) if t <= cut]
    sh = [s[3] for s in sim_series if s[0] <= cut]
    return {"pitch_rms": rms(ep), "roll_rms": rms(er), "robot_fell": r_fell, "sim_fell": sim_fell,
            "agree": (r_fell is None) == (sim_fell is None),
            "robot_turn": turn(hd[0], hd[-1]) if len(hd) > 1 else None, "sim_turn": turn(sh[0], sh[-1]) if len(sh) > 1 else None}


def run_one(args):
    walk, knobs = args
    series, fell = replay(walk, knobs)
    return compare(walk, series, fell)


def evaluate(ws, knobs, jobs):
    with ProcessPoolExecutor(max_workers=jobs) as ex:
        return list(ex.map(run_one, [(w, knobs) for w in ws]))


def summary(res):
    agree = sum(r["agree"] for r in res)
    return (agree, statistics.mean(r["pitch_rms"] for r in res), statistics.mean(r["roll_rms"] for r in res))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--friction", type=float, default=1.0)
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--damping", type=float, default=1.0)
    ap.add_argument("--torque", type=float)
    ap.add_argument("--frictionloss", type=float, help="joint Coulomb friction, N m")
    ap.add_argument("--torsion", type=float, help="torsional (twisting) contact friction (model: 0.02)")
    ap.add_argument("--grid", help="JSON list of knob dicts to evaluate instead of --sweep's grid")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--method", default="AppWalk")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", help="write the results as JSON")
    args = ap.parse_args()
    ws = [w for w in walks(args.session) if not args.method or w["method"] == args.method]
    print(f"{len(ws)} {args.method or ''} walks with servo frames: {[w['i'] for w in ws]}")
    grid = [dict(friction=f, kp=k, damping=d, torque=q) for f, k, d, q in
            itertools.product((0.6, 1.0, 1.5), (0.5, 1.0, 2.0, 4.0), (1.0, 2.0, 4.0), (1.15, 1.6, 2.0))] if args.sweep else \
        [dict(friction=args.friction, kp=args.kp, damping=args.damping, torque=args.torque, frictionloss=args.frictionloss,
              torsion=args.torsion)]
    if args.grid:
        grid = json.loads(args.grid)
    results = []
    for knobs in grid:
        res = evaluate(ws, knobs, args.jobs)
        agree, prms, rrms = summary(res)
        results.append({"knobs": knobs, "agree": agree, "pitch_rms": prms, "roll_rms": rrms, "walks": res})
        print(f"{json.dumps(knobs):70s} falls agree {agree}/{len(ws)}  pitch rms {prms:5.2f}  roll rms {rrms:5.2f}", flush=True)
        if len(grid) == 1:
            for w, r in zip(ws, res):
                print(f"   walk {w['i']:2d}{' turn' if w.get('turned') else '     '}: robot {'FELL %.1fs' % r['robot_fell'] if r['robot_fell'] else 'up':10s} "
                      f"sim {'FELL %.1fs' % r['sim_fell'] if r['sim_fell'] else 'up':10s} pitch rms {r['pitch_rms']:5.2f} roll rms {r['roll_rms']:5.2f}"
                      f"  heading robot {r['robot_turn'] or 0:+5.0f} sim {r['sim_turn'] or 0:+5.0f}")
    if args.sweep or args.grid:
        best = sorted(results, key=lambda r: (-r["agree"], r["pitch_rms"] + r["roll_rms"]))[:8]
        print("\nbest:")
        for r in best:
            print(f"  {json.dumps(r['knobs']):70s} agree {r['agree']}/{len(ws)} pitch {r['pitch_rms']:5.2f} roll {r['roll_rms']:5.2f}")
    if args.out:
        json.dump(results, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
