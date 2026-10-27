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
Servos not named in a frame keep their previous target. Angles are servo degrees. A frame without
"angles" is a pause. With end_in_reset, play() finishes with a RESET_POSE move over RESET_RUNTIME_MS.
"""
import json
import math
import threading

from .servos import LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, RESET_POSE, check_angles

# A conservative speed warning threshold for a 2 kg robot's servos (deg/s). Tune with experience.
MAX_SPEED_DEG_S = 300
RESET_RUNTIME_MS = 1000  # the end_in_reset move, at speed 1
WAIT_SLICE_S = 0.05  # play() checks stop_event this often while waiting


def load(path):
    """A motion dict from a JSON file, or from one of the robot's own .hts files (see hts.py)."""
    if str(path).lower().endswith(".hts"):
        from . import hts
        return hts.load(path)
    with open(path) as f:
        motion = json.load(f)
    if isinstance(motion, dict):
        motion.setdefault("name", path)
        motion.setdefault("end_in_reset", True)
    return motion


def save(motion, path):
    """Write a motion dict as JSON, or as an .hts file when the path ends in .hts."""
    if str(path).lower().endswith(".hts"):
        from . import hts
        return hts.save(motion, path)
    with open(path, "w") as f:
        json.dump(motion, f, indent=2)
        f.write("\n")


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _frames(motion):
    """The frame list, or [] when the motion is malformed (validate() reports why)."""
    frames = motion.get("frames") if isinstance(motion, dict) else None
    return frames if isinstance(frames, list) else []


def _angles(fr):
    """Numeric angles of known servos in a frame ({} for pauses and malformed frames)."""
    a = fr.get("angles") if isinstance(fr, dict) else None
    return {n: d for n, d in a.items() if n in LIMITS and _num(d)} if isinstance(a, dict) else {}


def _times(fr):
    """(runtime ms, hold ms) of a frame, with defaults for missing or malformed values."""
    if not isinstance(fr, dict):
        return 500, 0
    runtime, hold = fr.get("runtime", 500), fr.get("hold", 0)
    return (runtime if _num(runtime) else 500), (max(hold, 0) if _num(hold) else 0)


def effective_runtime_ms(runtime, speed=1.0):
    """The runtime the servos really get: runtime / speed clamped to YanAPI's 200-4000 ms."""
    return min(max(runtime / speed, MIN_RUNTIME_MS), MAX_RUNTIME_MS)


def _ok_speed(speed):
    return _num(speed) and speed > 0


def validate(motion, start_pose=None, speed=1.0):
    """Return (errors, warnings) for playing `motion` at `speed` from `start_pose` (default RESET_POSE).

    Never raises: malformed input is reported as errors. Errors must be fixed before the motion is
    sent to a real robot. Speeds are checked with the runtimes the robot really gets at `speed`.
    Messages count frames from 1 ("frame 1" is frames[0]); the end_in_reset move is "return to reset".
    """
    errors, warnings = [], []
    if not _ok_speed(speed):
        errors.append(f"speed {speed!r} must be a number > 0")
        speed = 1.0
    if not isinstance(motion, dict):
        return errors + ["motion must be a JSON object"], warnings
    frames = motion.get("frames")
    if not isinstance(frames, list):
        return errors + ["motion has no frames" if frames is None else "'frames' must be a list"], warnings
    if not frames:
        errors.append("motion has no frames")
    pose = dict(RESET_POSE)
    pose.update({n: d for n, d in (start_pose or {}).items() if n in LIMITS and _num(d)})

    def check_move(label, target, runtime):
        eff = effective_runtime_ms(runtime, speed)
        if target and speed != 1.0 and eff != runtime / speed:
            warnings.append(f"{label}: runtime {runtime:g} ms at speed {speed:g} is {runtime / speed:.0f} ms, "
                            f"clamped to {eff:.0f} ms")
        for n, deg in target.items():
            v = abs(deg - pose.get(n, deg)) / (eff / 1000)
            if v > MAX_SPEED_DEG_S:
                warnings.append(f"{label}: {n} moves at {v:.0f} deg/s (> {MAX_SPEED_DEG_S})")
        pose.update(target)

    for i, fr in enumerate(frames, 1):
        if not isinstance(fr, dict):
            errors.append(f"frame {i}: must be an object with 'angles', 'runtime' and 'hold'")
            continue
        angles, runtime, hold = fr.get("angles", {}), fr.get("runtime", 500), fr.get("hold", 0)
        if not isinstance(angles, dict):
            errors.append(f"frame {i}: 'angles' must be an object {{servo: degrees}}")
            angles = {}
        bad = [n for n, d in angles.items() if not _num(d)]
        errors += [f"frame {i}: {n}={angles[n]!r} is not a number" for n in bad]
        errors += [f"frame {i}: {p}" for p in check_angles({n: d for n, d in angles.items() if n not in bad})]
        if not _num(runtime):
            errors.append(f"frame {i}: runtime {runtime!r} is not a number")
            runtime = 500
        elif not MIN_RUNTIME_MS <= runtime <= MAX_RUNTIME_MS:
            errors.append(f"frame {i}: runtime {runtime} ms outside {MIN_RUNTIME_MS}-{MAX_RUNTIME_MS}")
        if not _num(hold) or hold < 0:
            errors.append(f"frame {i}: hold {hold!r} must be a number >= 0")
        check_move(f"frame {i}", _angles(fr), runtime)
    if frames and motion.get("end_in_reset"):
        check_move("return to reset", dict(RESET_POSE), RESET_RUNTIME_MS)
    return errors, warnings


def duration_s(motion, speed=1.0):
    """Playing time in seconds at `speed`, as play() waits it (clamped runtimes, scaled holds and reset)."""
    if not _ok_speed(speed):
        speed = 1.0
    t = 0.0
    for fr in _frames(motion):
        runtime, hold = _times(fr)
        t += (effective_runtime_ms(runtime, speed) + hold / speed) / 1000
    if isinstance(motion, dict) and motion.get("end_in_reset"):
        t += effective_runtime_ms(RESET_RUNTIME_MS, speed) / 1000
    return t


def _wait(robot, seconds, stop_event):
    """robot.wait(seconds) in small slices; False if stop_event was set."""
    if stop_event is None:
        robot.wait(seconds)
        return True
    while seconds > 1e-9:
        if stop_event.is_set():
            return False
        dt = min(WAIT_SLICE_S, seconds)
        robot.wait(dt)
        seconds -= dt
    return not stop_event.is_set()


def play(robot, motion, speed=1.0, on_frame=None, stop_event=None):
    """Run a motion on any backend. speed < 1 slows it down (useful for first tries on hardware).

    stop_event (threading.Event) cancels between frames and during waits: the robot keeps the pose
    it has and the final reset is skipped. Returns True when the motion completed, False if cancelled.
    Runtimes are clamped to 200-4000 ms after scaling, exactly as validate(..., speed=) assumes.
    """
    if not _ok_speed(speed):
        raise ValueError(f"speed must be a number > 0, got {speed!r}")
    for i, fr in enumerate(_frames(motion)):
        if stop_event is not None and stop_event.is_set():
            return False
        runtime, hold = _times(fr)
        runtime = effective_runtime_ms(runtime, speed)
        if on_frame:
            on_frame(i, fr)
        angles = _angles(fr)
        if angles:
            robot.set_servos_angles(angles, int(round(runtime)))
        if not _wait(robot, (runtime + hold / speed) / 1000, stop_event):
            return False
    if isinstance(motion, dict) and motion.get("end_in_reset"):
        if stop_event is not None and stop_event.is_set():
            return False
        runtime = effective_runtime_ms(RESET_RUNTIME_MS, speed)
        robot.set_servos_angles(dict(RESET_POSE), int(round(runtime)))
        if not _wait(robot, runtime / 1000, stop_event):
            return False
    return True


class CollisionChecker:
    """Kinematic self-collision test for single poses. Compile once, query many times (about 1 ms per pose).

    Pairs already touching in the reset pose are ignored (`baseline`), so a hit means two parts of the
    robot move into each other. Thread-safe.
    """

    def __init__(self, scene=None):
        import mujoco
        from .robot import SCENE
        self._mj = mujoco
        self._lock = threading.Lock()
        spec = mujoco.MjSpec.from_file(scene or SCENE)
        for g in spec.geoms:  # let robot collision hulls collide with each other (must be set before compiling)
            if g.group == 3:
                g.conaffinity = 1
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._key = self.model.key("reset").id
        self._qadr = {n: self.model.jnt_qposadr[self.model.joint(n).id] for n in LIMITS}
        self.baseline = self.contacts(RESET_POSE, baseline=set())

    def contacts(self, pose, baseline=None):
        """Body pairs in contact for a {servo: deg} pose (unnamed servos stay in the reset pose),
        minus `baseline` (default: the pairs touching in the reset pose; pass set() for all pairs)."""
        m, d = self.model, self.data
        out = set()
        with self._lock:
            self._mj.mj_resetDataKeyframe(m, d, self._key)
            for n, deg in pose.items():
                if n in self._qadr and _num(deg):
                    d.qpos[self._qadr[n]] = math.radians(deg)
            d.qpos[2] += 0.2  # lift off the floor: only robot-robot contacts matter here
            self._mj.mj_forward(m, d)
            for c in d.contact[:d.ncon]:
                if c.dist < -0.002:  # ignore grazing contacts under 2 mm
                    b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
                    out.add(tuple(sorted((b1, b2))))
        return out - (self.baseline if baseline is None else baseline)


def self_collisions(motion, samples_per_frame=8, scene=None, start_pose=None, checker=None):
    """Sweep a motion kinematically and report (frame, fraction, (body, body)) for every self-collision.

    The sweep starts at start_pose (default RESET_POSE); pairs already touching there are ignored.
    With end_in_reset, the return move to RESET_POSE is swept too and reported as frame len(frames).
    Malformed frames are skipped (validate() reports them).
    """
    checker = checker or CollisionChecker(scene)
    pose = dict(RESET_POSE)
    pose.update({n: d for n, d in (start_pose or {}).items() if n in LIMITS and _num(d)})
    baseline = checker.baseline | checker.contacts(pose, baseline=set())
    targets = [_angles(fr) for fr in _frames(motion)]
    if targets and motion.get("end_in_reset"):
        targets.append(dict(RESET_POSE))
    report = []
    for i, angles in enumerate(targets):
        target = {**pose, **angles}
        if target == pose:
            continue
        for k in range(1, samples_per_frame + 1):
            s = k / samples_per_frame
            p = {n: pose[n] + (target[n] - pose[n]) * s for n in pose}
            for pair in sorted(checker.contacts(p, baseline=baseline)):
                report.append((i, round(s, 2), pair))
        pose = target
    return report
