"""Consistency checks for the generated models and the SimRobot backend.
Run: python3 -m pytest tests -q  (or python3 tests/test_models.py)"""
import ast
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.robot import ServoCommandError, SimRobot  # noqa: E402
from yanshee_sim.servos import LIMITS, NAMES, RESET_POSE, check_angles, clamp_angles  # noqa: E402

SCENE = os.path.join(ROOT, "model", "scene.xml")
URDF = os.path.join(ROOT, "ros2_ws", "src", "yanshee_description", "urdf", "yanshee.urdf")
LAUNCH = os.path.join(ROOT, "ros2_ws", "src", "yanshee_description", "launch", "display.launch.py")


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
        assert abs(math.degrees(d.qpos[m.jnt_qposadr[m.joint(n).id]]) - deg) < 0.01, n
        assert abs(math.degrees(d.ctrl[m.actuator(n).id]) - deg) < 0.01, n


def test_launch_zeros_are_reset_pose():
    tree = ast.parse(open(LAUNCH).read())
    node = next(n for n in tree.body if isinstance(n, ast.Assign) and n.targets[0].id == "RESET_POSE_RAD")
    zeros = ast.literal_eval(node.value)
    assert set(zeros) == set(NAMES)
    for n, deg in RESET_POSE.items():
        assert abs(math.degrees(zeros[n]) - deg) < 0.01, n


def test_reset_pose_feet_on_floor():
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("reset").id)
    mujoco.mj_kinematics(m, d)
    low = math.inf
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            a, k = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
            low = min(low, (m.mesh_vert[a:a + k] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min())
    assert 0 < low < 0.002, f"lowest vertex at {low * 1000:.2f} mm"


def test_build_models_help_does_not_build():
    out = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "build_models.py"), "--help"],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0 and "usage" in out.stdout and "bodies:" not in out.stdout


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


# -- SimRobot ---------------------------------------------------------------------------
def _raises(fn, exc=ServoCommandError):
    try:
        fn()
    except exc:
        return True
    return False


def test_sim_rejects_nan_and_unknown_names():
    nan = float("nan")
    assert clamp_angles({"NeckLR": nan, "RightHipLR": 500, "Foo": 1}) == {"RightHipLR": 120}
    assert check_angles({"NeckLR": nan}) and check_angles({"NeckLR": "90"})
    bot = SimRobot(viewer=False, realtime=False)
    bot.strict = False
    assert _raises(lambda: bot.set_servos_angles({"NeckLR": nan}))
    assert _raises(lambda: bot.set_servos_angles({"NeckLR": 90}, runtime=float("inf")))
    assert _raises(lambda: bot.get_servo_angle_value("Foo"))
    assert _raises(lambda: bot.get_servos_angles(["NeckLR", "Foo"]))
    assert _raises(lambda: bot.set_servos_mode("limp", ["NeckLR"]))
    assert bot.get_servos_angles("NeckLR")["data"] == {"NeckLR": 90}
    bot.wait(-1)  # same as wait(0), in every mode
    bot.wait(0.5)
    assert bot.torso_tilt_deg() < 5
    bot.close()


def test_sim_viewer_reset_goes_to_reset_pose():
    bot = SimRobot(viewer=False, realtime=False)
    bot.set_servos_angles({"NeckLR": 30}, 200)
    bot.wait(0.6)
    bot.set_servos_angles({"NeckLR": 150}, 200)
    bot.wait(0.6)
    with bot._lock:
        mujoco.mj_resetData(bot.model, bot.data)  # what the viewer's Backspace / Reset does
    bot.wait(1.0)
    assert bot.get_servo_angle_value("NeckLR") == 90
    assert bot.torso_tilt_deg() < 5
    bot.close()


def test_sim_stop_all_and_respawn():
    bot = SimRobot(viewer=False, realtime=False)
    bot.set_servos_angles({"NeckLR": 30}, 2000)
    bot.wait(0.5)
    bot.stop_all()
    bot.wait(1.5)
    assert 65 <= bot.get_servo_angle_value("NeckLR") <= 85  # stopped part-way, not at 30
    bot.jump_to({"NeckLR": 20})
    bot.respawn()
    assert bot.get_servo_angle_value("NeckLR") == 90 and bot.data.time == 0
    bot.close()


def test_sim_program_mode_is_limp():
    bot = SimRobot(viewer=False, realtime=False, fixed_base=True)
    arm = ["RightShoulderFlex", "RightElbowFlex"]
    assert bot.set_servos_mode("program", arm)["data"] == {n: True for n in arm}
    assert bot.get_servos_mode(arm + ["NeckLR"])["data"] == {"RightShoulderFlex": "program",
                                                             "RightElbowFlex": "program", "NeckLR": "work"}
    bot.wait(1.5)
    sagged = bot.get_servo_angle_value("RightShoulderFlex")
    assert abs(sagged - RESET_POSE["RightShoulderFlex"]) > 5, "limp arm did not move under gravity"
    bot.set_servos_mode("work", arm)
    bot.wait(1.0)
    assert abs(bot.get_servo_angle_value("RightShoulderFlex") - sagged) <= 2, "work mode did not hold the pose"
    bot.close()


def test_sim_imu_fields():
    keys = {"id", "gyro-x", "gyro-y", "gyro-z", "accel-x", "accel-y", "accel-z",
            "compass-x", "compass-y", "compass-z", "euler-x", "euler-y", "euler-z"}
    for fixed in (False, True):
        bot = SimRobot(viewer=False, realtime=False, fixed_base=fixed)
        bot.wait(0.5)
        g = bot.get_sensors_gyro()["data"]["gyro"][0]
        assert set(g) == keys and all(type(v) in (int, float) for v in g.values())
        assert 9.0 < math.hypot(g["accel-x"], g["accel-y"], g["accel-z"]) < 10.6, (fixed, g)
        assert abs(g["euler-x"]) < 5 and abs(g["euler-y"]) < 5
        bot.close()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
