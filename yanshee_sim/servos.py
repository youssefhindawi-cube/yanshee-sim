"""Yanshee servo table: the single source of truth for names, IDs and limits.

Angles are servo degrees exactly as YanAPI uses them (0-180). The simulator and the
URDF use the same numbers converted to radians, so a simulated joint angle maps 1:1
onto the real servo command.
"""
import math
import numbers

# (servo id, YanAPI name, min deg, max deg)
# Limits come from the YanAPI.set_servos_angles docstring ("runnable angle range").
SERVOS = [
    (1, "RightShoulderRoll", 0, 180),
    (2, "RightShoulderFlex", 0, 180),
    (3, "RightElbowFlex", 0, 180),
    (4, "LeftShoulderRoll", 0, 180),
    (5, "LeftShoulderFlex", 0, 180),
    (6, "LeftElbowFlex", 0, 180),
    (7, "RightHipLR", 0, 120),
    (8, "RightHipFB", 10, 180),
    (9, "RightKneeFlex", 0, 180),
    (10, "RightAnkleFB", 0, 180),
    (11, "RightAnkleUD", 65, 180),
    (12, "LeftHipLR", 60, 180),
    (13, "LeftHipFB", 0, 170),
    (14, "LeftKneeFlex", 0, 180),
    (15, "LeftAnkleFB", 0, 180),
    (16, "LeftAnkleUD", 0, 115),
    (17, "NeckLR", 15, 165),
]

NAMES = [s[1] for s in SERVOS]
IDS = {s[1]: s[0] for s in SERVOS}
LIMITS = {s[1]: (s[2], s[3]) for s in SERVOS}

# Standing reset pose, in servo degrees. It matches the Webots model's saved joint positions:
# legs, neck and left arm match the Yanshee Course default pose exactly; the right arm
# mirrors the left (180 - x). Verify against get_servos_angles() on your robot after running
# the built-in "reset" motion, and update if it differs. This is the single source: run
# tools/build_models.py afterwards to regenerate the MuJoCo "reset" keyframe and the RViz
# slider start values (ros2_ws/.../launch/display.launch.py).
RESET_POSE = {
    "RightShoulderRoll": 90, "RightShoulderFlex": 140, "RightElbowFlex": 164,
    "LeftShoulderRoll": 90, "LeftShoulderFlex": 40, "LeftElbowFlex": 15,
    "RightHipLR": 90, "RightHipFB": 60, "RightKneeFlex": 76, "RightAnkleFB": 110, "RightAnkleUD": 90,
    "LeftHipLR": 90, "LeftHipFB": 120, "LeftKneeFlex": 104, "LeftAnkleFB": 70, "LeftAnkleUD": 90,
    "NeckLR": 90,
}

# YanAPI.set_servos_angles accepts runtime between these values (milliseconds).
MIN_RUNTIME_MS = 200
MAX_RUNTIME_MS = 4000


def _finite(deg):
    return isinstance(deg, numbers.Real) and not isinstance(deg, bool) and math.isfinite(deg)


def check_angles(angles):
    """Return a list of problems with a {name: deg} dict (unknown names, non-numbers/NaN, out-of-range values)."""
    problems = []
    for name, deg in angles.items():
        if name not in LIMITS:
            problems.append(f"unknown servo '{name}'")
            continue
        if not _finite(deg):
            problems.append(f"{name}={deg!r} is not a finite number")
            continue
        lo, hi = LIMITS[name]
        if not lo <= deg <= hi:
            problems.append(f"{name}={deg} outside {lo}-{hi}")
    return problems


def clamp_angles(angles):
    """Clamp a {name: deg} dict into each servo's limits (unknown names and NaN/non-numbers are dropped)."""
    return {n: min(max(d, LIMITS[n][0]), LIMITS[n][1]) for n, d in angles.items() if n in LIMITS and _finite(d)}
