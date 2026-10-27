"""Consistency checks for the generated models. Run: python3 -m pytest tests -q  (or python3 tests/test_models.py)"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.servos import LIMITS, NAMES, RESET_POSE  # noqa: E402

SCENE = os.path.join(ROOT, "model", "scene.xml")
URDF = os.path.join(ROOT, "ros2_ws", "src", "yanshee_description", "urdf", "yanshee.urdf")


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def urdf_fk(q):
    """World position of every <servo>_link origin for joint angles q (rad), base_link at the origin."""
    root = ET.parse(URDF).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint") if j.get("type") == "revolute"}
    cache = {"base_link": np.eye(4), "torso_link": np.eye(4)}  # base_to_torso is a fixed identity joint

    def tf(link):
        if link in cache:
            return cache[link]
        j = joints[link]
        o = j.find("origin")
        T = np.eye(4)
        T[:3, :3] = rpy_to_mat(*map(float, o.get("rpy").split()))
        T[:3, 3] = list(map(float, o.get("xyz").split()))
        R = np.eye(4)
        R[:3, :3] = axis_angle(np.array(list(map(float, j.find("axis").get("xyz").split()))), q[j.get("name")])
        cache[link] = tf(j.find("parent").get("link")) @ T @ R
        return cache[link]

    return {n: tf(n + "_link")[:3, 3] for n in NAMES}


def mujoco_fk(m, d, q):
    d.qpos[:] = 0
    d.qpos[3] = 1  # free joint at origin, identity orientation
    for n, v in q.items():
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
    mujoco.mj_kinematics(m, d)
    return {n: d.xpos[m.body(n).id].copy() for n in NAMES}


def test_reset_pose_within_limits():
    for n, deg in RESET_POSE.items():
        lo, hi = LIMITS[n]
        assert lo <= deg <= hi, n


def test_urdf_matches_mujoco():
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        q = {n: math.radians(rng.uniform(*LIMITS[n])) for n in NAMES}
        a, b = urdf_fk(q), mujoco_fk(m, d, q)
        worst = max(worst, max(np.linalg.norm(a[n] - b[n]) for n in NAMES))
    assert worst < 1e-6, f"URDF and MJCF disagree by {worst * 1000:.3f} mm"


def test_keyframe_is_reset_pose():
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("reset").id)
    for n, deg in RESET_POSE.items():
        assert abs(math.degrees(d.qpos[m.jnt_qposadr[m.joint(n).id]]) - deg) < 0.6, n
        assert abs(math.degrees(d.ctrl[m.actuator(n).id]) - deg) < 0.6, n


def test_stands_in_reset_pose():
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("reset").id)
    z_spawn = d.qpos[2]
    for _ in range(int(3.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    w, x, y, z = d.qpos[3:7]
    up_z = 1 - 2 * (x * x + y * y)  # world z of the torso's z axis
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    assert tilt < 5, f"torso tilted {tilt:.1f} deg after 3 s"
    assert z_spawn - d.qpos[2] < 0.006, f"torso sagged {(z_spawn - d.qpos[2]) * 1000:.1f} mm"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
