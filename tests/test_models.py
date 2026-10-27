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


def test_feet_stand_still():
    """The feet meet the floor through flat box soles: a standing robot must not rock (the mesh soles made the
    hips and ankles chatter by 1-1.5 deg and the IMU accel jump by 8 m/s^2)."""
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    floor = {g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE}
    mujoco.mj_resetDataKeyframe(m, d, m.key("reset").id)
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    legs = [m.jnt_qposadr[m.joint(n).id] for n in NAMES if "Hip" in n or "Knee" in n or "Ankle" in n]
    lo, hi, counts, touching = d.qpos[legs].copy(), d.qpos[legs].copy(), set(), set()
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        lo, hi = np.minimum(lo, d.qpos[legs]), np.maximum(hi, d.qpos[legs])
        floor_contacts = [c for c in d.contact[:d.ncon] if c.geom1 in floor or c.geom2 in floor]
        counts.add(len(floor_contacts))
        touching |= {m.geom(c.geom2 if c.geom1 in floor else c.geom1).name for c in floor_contacts}
    assert math.degrees((hi - lo).max()) < 0.05, f"leg joints moved {math.degrees((hi - lo).max()):.2f} deg while standing"
    assert touching == {"RightAnkleUD_sole", "LeftAnkleUD_sole"}, touching
    assert counts == {8}, f"floor contact count varied: {sorted(counts)}"


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


def test_sim_gait_stops_without_falling():
    """A stop at full speed, at any point of the step cycle, must fade the walk out rather than freeze it
    (a 0.4 s fade-out tipped the robot over in half the cases)."""
    import threading
    import time
    from yanshee_sim.sim_extras import SimGait
    for stop_at in (2.5, 2.62, 2.74, 2.86):
        bot = SimRobot(viewer=False, realtime=False)
        bot.wait(0.5)
        gait, stop, tilt = SimGait(bot), threading.Event(), [0.0]
        stopped = [False]
        wait = bot.wait

        def watched(seconds):
            wait(seconds)
            if stopped[0]:
                tilt[0] = max(tilt[0], bot.torso_tilt_deg())
        bot.wait = watched
        th = threading.Thread(target=lambda: gait.run({"speed_v": 5, "period": 1, "steps": 0}, stop))
        th.start()
        while bot.data.time < stop_at:
            time.sleep(0.001)
        stopped[0] = True
        stop.set()
        th.join(20)
        wait(1.0)
        tilt[0] = max(tilt[0], bot.torso_tilt_deg())
        bot.close()
        assert gait.status == "stopped" and tilt[0] < 15, (stop_at, gait.status, tilt[0])


def test_sim_imu_fields():
    keys = {"id", "gyro-x", "gyro-y", "gyro-z", "accel-x", "accel-y", "accel-z",
            "compass-x", "compass-y", "compass-z", "euler-x", "euler-y", "euler-z"}
    for fixed in (False, True):
        bot = SimRobot(viewer=False, realtime=False, fixed_base=fixed)
        bot.wait(0.5)
        g = bot.get_sensors_gyro()["data"]["gyro"][0]
        assert set(g) == keys and all(type(v) in (int, float) for v in g.values())
        assert 0.95 < math.hypot(g["accel-x"], g["accel-y"], g["accel-z"]) < 1.05, (fixed, g)  # g units
        assert g["accel-z"] < -0.9, g  # the down axis reads about -1 g upright
        assert abs(g["euler-x"] - 90) < 5 and abs(g["euler-y"]) < 5 and abs(g["euler-z"]) < 5, g
        assert abs(g["gyro-x"] + 1.14) < 2 and abs(g["gyro-y"]) < 2 and abs(g["gyro-z"]) < 2, g  # bias + noise
        assert bot.get_sensors_gyro()["data"]["gyro"][0] == g  # held for 1/19 s of sim time
        bot.wait(0.1)
        assert bot.get_sensors_gyro()["data"]["gyro"][0] != g
        bot.imu_noise = False
        bot.wait(0.1)
        h = bot.get_sensors_gyro()["data"]["gyro"][0]
        tol = 1e-6 if fixed else 1.0  # a free-standing robot still sways a little while settling
        assert abs(h["gyro-x"]) < tol and abs(h["gyro-y"]) < tol and abs(h["gyro-z"]) < tol, h
        bot.close()
    bot = SimRobot(viewer=False, realtime=False)
    lean = bot.torso_lean()
    bot.close()
    assert len(lean) == 4 and abs(lean[0]) < 5 and abs(lean[1]) < 5


def test_sim_imu_fall_convention():
    """euler-x is about 0 face down and about +-180 on the back (the course's fall thresholds), 90 upright;
    euler-y is the sideways lean and euler-z the heading (+ = turned left)."""
    bot = SimRobot(viewer=False, realtime=False)
    bot.imu_noise = False

    def read(quat_wxyz):
        with bot._lock:
            bot.data.qpos[2] = 0.4  # in the air, so the orientation holds while falling
            bot.data.qpos[3:7] = quat_wxyz
            bot.data.qvel[:] = 0
        bot.wait(0.06)
        g = bot.get_sensors_gyro()["data"]["gyro"][0]
        return g["euler-x"], g["euler-y"], g["euler-z"]
    s2 = math.sqrt(0.5)
    ex, ey, ez = read((s2, -s2, 0, 0))  # pitched 90 deg forward about x: face down
    assert abs(ex) < 3 and abs(ey) < 3, (ex, ey)
    ex, ey, ez = read((s2, s2, 0, 0))  # on the back
    assert abs(ex) > 177 and abs(ey) < 3, (ex, ey)
    ex, ey, ez = read((math.cos(math.radians(10)), math.sin(math.radians(10)), 0, 0))  # leaning 20 deg back
    assert 107 < ex < 113, ex  # UBTECH's self-balancing sample: upright 91.95, leaning back raises euler-x
    ex, ey, ez = read((math.cos(math.radians(10)), 0, math.sin(math.radians(10)), 0))  # 20 deg lean about y
    assert abs(ex - 90) < 3 and 15 < ey < 25, (ex, ey)
    ex, ey, ez = read((math.cos(math.radians(15)), 0, 0, math.sin(math.radians(15))))  # turned 30 deg left
    assert abs(ex - 90) < 3 and 27 < ez < 33, (ex, ez)
    bot.close()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
