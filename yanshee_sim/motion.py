"""Keyframe motions shared by the simulator and the real robot.

Motion file (JSON):
    {
      "name": "wave_right",
      "description": "Raise the right arm and wave twice",
      "frames": [
        {"angles": {"RightShoulderFlex": 175, "RightElbowFlex": 120}, "runtime": 700, "hold": 100},
        ...
      ],
      "end_in_reset": true
    }

Each frame is one set_servos_angles(angles, runtime) call followed by a pause of runtime + hold ms.
Servos not named in a frame keep their previous target. Angles are servo degrees.
"""
import json
import math

from .servos import LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, RESET_POSE, check_angles

# A conservative speed warning threshold for a 2 kg robot's servos (deg/s). Tune with experience.
MAX_SPEED_DEG_S = 300


def load(path):
    with open(path) as f:
        motion = json.load(f)
    motion.setdefault("name", path)
    motion.setdefault("end_in_reset", True)
    return motion


def save(motion, path):
    with open(path, "w") as f:
        json.dump(motion, f, indent=2)
        f.write("\n")


def validate(motion, start_pose=None):
    """Return (errors, warnings). Errors must be fixed before the motion is sent to a real robot."""
    errors, warnings = [], []
    pose = dict(start_pose or RESET_POSE)
    for i, fr in enumerate(motion.get("frames", [])):
        angles, runtime = fr.get("angles", {}), fr.get("runtime", 500)
        errors += [f"frame {i}: {p}" for p in check_angles(angles)]
        if not MIN_RUNTIME_MS <= runtime <= MAX_RUNTIME_MS:
            errors.append(f"frame {i}: runtime {runtime} ms outside {MIN_RUNTIME_MS}-{MAX_RUNTIME_MS}")
        for n, deg in angles.items():
            if n in LIMITS and runtime > 0:
                speed = abs(deg - pose.get(n, deg)) / (runtime / 1000)
                if speed > MAX_SPEED_DEG_S:
                    warnings.append(f"frame {i}: {n} moves at {speed:.0f} deg/s (> {MAX_SPEED_DEG_S})")
        pose.update({n: d for n, d in angles.items() if n in LIMITS})
    if not motion.get("frames"):
        errors.append("motion has no frames")
    return errors, warnings


def duration_s(motion):
    t = sum((f.get("runtime", 500) + f.get("hold", 0)) / 1000 for f in motion["frames"])
    return t + (1.0 if motion.get("end_in_reset") else 0.0)


def play(robot, motion, speed=1.0, on_frame=None):
    """Run a motion on any backend. speed < 1 slows it down (useful for first tries on hardware)."""
    for i, fr in enumerate(motion["frames"]):
        runtime = fr.get("runtime", 500) / speed
        if on_frame:
            on_frame(i, fr)
        robot.set_servos_angles(fr["angles"], runtime)
        robot.wait((min(max(runtime, MIN_RUNTIME_MS), MAX_RUNTIME_MS) + fr.get("hold", 0) / speed) / 1000)
    if motion.get("end_in_reset"):
        robot.reset(runtime=1000)


def self_collisions(motion, samples_per_frame=8, scene=None):
    """Sweep the motion kinematically in MuJoCo and report body pairs that come into contact.

    Pairs already touching in the reset pose (neighbouring parts of the convex hulls) are ignored,
    so a report means two parts of the robot move into each other during this motion.
    """
    import mujoco
    from .robot import SCENE

    spec = mujoco.MjSpec.from_file(scene or SCENE)
    for g in spec.geoms:  # let robot collision hulls collide with each other (must be set before compiling)
        if g.group == 3:
            g.conaffinity = 1
    m = spec.compile()
    d = mujoco.MjData(m)
    key = m.key("reset").id
    qadr = {n: m.jnt_qposadr[m.joint(n).id] for n in LIMITS}

    def pairs_at(pose):
        mujoco.mj_resetDataKeyframe(m, d, key)
        for n, deg in pose.items():
            d.qpos[qadr[n]] = math.radians(deg)
        d.qpos[2] += 0.2  # lift off the floor: only robot-robot contacts matter here
        mujoco.mj_forward(m, d)
        out = set()
        for c in d.contact[:d.ncon]:
            b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
            if c.dist < -0.002:  # ignore grazing contacts under 2 mm
                out.add(tuple(sorted((b1, b2))))
        return out

    baseline = pairs_at(RESET_POSE)
    report, pose = [], dict(RESET_POSE)
    for i, fr in enumerate(motion["frames"]):
        target = {**pose, **{n: d for n, d in fr["angles"].items() if n in LIMITS}}
        for k in range(1, samples_per_frame + 1):
            s = k / samples_per_frame
            p = {n: pose[n] + (target[n] - pose[n]) * s for n in pose}
            for pair in pairs_at(p) - baseline:
                report.append((i, round(s, 2), pair))
        pose = target
    return report
