#!/usr/bin/env python3
"""Convert the Webots Yanshee model (exported to JSON) into MuJoCo MJCF, a ROS 2 URDF and STL meshes.

Usage:
    python3 tools/build_models.py            # reads source_model/robot.json, writes model/ and the ROS 2 package
    python3 tools/build_models.py --help

The MuJoCo "reset" keyframe and the RViz slider start values come from yanshee_sim/servos.py
RESET_POSE, so edit the pose there and rebuild.

Conventions of the generated models
-----------------------------------
* Joint names are the real YanAPI servo names (RightShoulderRoll ... NeckLR).
* Joint angle in radians == servo angle in degrees converted to radians. Webots stores the
  saved joint positions this way (90 deg -> 1.5708 rad), so q_sim = radians(servo_deg).
* Every body frame is world-aligned in the saved (standing) pose and sits on its joint anchor.
  MuJoCo uses `ref` so qpos == servo radians; the URDF bakes the same offset into the joint origin.
* Model frame: z up, robot faces +y, feet at z ~ 0.
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim.servos import IDS, LIMITS, NAMES, RESET_POSE  # noqa: E402

SRC = os.path.join(ROOT, "source_model", "robot.json")
MODEL_DIR = os.path.join(ROOT, "model")
ROS_PKG = os.path.join(ROOT, "ros2_ws", "src", "yanshee_description")
FOOT_CLEARANCE = 0.0005  # m between the lowest vertex of the reset pose and the floor at spawn
# The feet touch the floor through a flat box sole, not their mesh: the mesh sole has pads and recesses and
# sits about 1.2 deg toe-down in the reset pose, so the standing robot rocked between 2 and 6 contact points
# (1-1.5 deg hip/ankle chatter, IMU accel noise of 8 m/s^2). The box is level in the reset pose, its bottom
# at the lowest pad, and gives four steady corner contacts. The mesh keeps colliding with the robot itself.
SOLE_BODIES = ("RightAnkleUD", "LeftAnkleUD")
SOLE_INSET = 0.002      # m inside the sole outline (rounded corners)
SOLE_THICKNESS = 0.004  # m

# Webots solid names -> YanAPI servo names (the Webots file has one typo).
SOLID_FIX = {"LeftAndleFB": "LeftAnkleFB"}
DEVICE_TYPES = {"Camera", "LED", "DistanceSensor", "Accelerometer", "Gyro", "Compass", "InertialUnit"}

# The Webots model stores each link's inertia with a lumped motor-rotor term of 1.4e-4 kg*m^2 added on that link's
# own joint axis (every non-root link, same value). Without it the tensors match a uniform-density integration of the
# link's mesh within 1-5 %; with it 15 of 17 break the triangle inequality, and MuJoCo's balanceinertia used to turn
# them into spheres (an 8 g shoulder bracket with a 78 mm radius of gyration). The term is taken off here, because a
# rotor only adds inertia through its own gear: that is the joint's armature (SERVO_ARMATURE), not link inertia.
SOURCE_ROTOR_INERTIA = 1.4e-4  # kg*m^2

# Servo model. See README section 14. No Yanshee servo datasheet is published; UBTECH's Alpha 1 Pro manual gives
# its servos 8 kg*cm at 6.5 V and 12 kg*cm at 8.5 V (0.238 / 0.198 s per 60 deg), i.e. about 1.0 N*m at the 7.3-7.7 V
# our robot's battery read during the get-up runs. SERVO_TORQUE is fitted to those runs (29 Oct 2026, the robot's
# GetupFront played in the sim from lying face down): it stands up on a smooth floor (friction <= 1.0) only from
# 1.05 N*m and falls forward on a grippy one (friction 1.4, the mat) only below 1.3 N*m, as the robot did.
SERVO_KP = 8.0          # N*m/rad position gain (assumed)
SERVO_TORQUE = 1.15     # N*m stall torque limit
SERVO_DAMPING = 0.08    # N*m*s/rad
SERVO_ARMATURE = 0.002  # kg*m^2 reflected rotor inertia


def rot(axis_angle):
    x, y, z, a = axis_angle
    v = np.array([x, y, z], float)
    n = np.linalg.norm(v)
    if n < 1e-12 or abs(a) < 1e-15:
        return np.eye(3)
    x, y, z = v / n
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def local_tf(node):
    T = np.eye(4)
    T[:3, :3] = rot(node.get("rotation", [0, 0, 1, 0])) @ np.diag(node.get("scale", [1, 1, 1]))
    T[:3, 3] = node.get("translation", [0, 0, 0])
    return T


def mat_to_rpy(R):
    # URDF fixed-axis roll-pitch-yaw (R = Rz(yaw) Ry(pitch) Rx(roll))
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return roll, pitch, yaw


def mat_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


class Body:
    def __init__(self, name, parent, origin):
        self.name, self.parent, self.origin = name, parent, origin  # origin: world position of frame
        self.joint = None          # dict(axis, ref) for non-root bodies
        self.meshes = {}           # color tuple -> list of (V (n,3) world, F (m,3))
        self.mass, self.com, self.inertia = 0.0, np.zeros(3), np.zeros((3, 3))
        self.devices = []          # (type, name, world 4x4, node)


def load_source(path):
    """Read the Webots JSON export, with clear messages for a missing file or a Git LFS pointer."""
    if not os.path.exists(path):
        sys.exit(f"error: {path} not found. It is stored with Git LFS: run 'git lfs install && git lfs pull'.")
    with open(path, "rb") as f:
        head = f.read(64)
    if head.startswith(b"version https://git-lfs"):
        rel = os.path.relpath(path, ROOT) if os.path.abspath(path).startswith(ROOT + os.sep) else path
        sys.exit(f"error: {path} is a Git LFS pointer, not the model. Run 'git lfs checkout {rel}'"
                 " (or 'git lfs install && git lfs pull' if the object was never downloaded).")
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except ValueError as e:
        sys.exit(f"error: {path} is not valid JSON ({e}).")


def parse(src=SRC):
    data = load_source(src)
    root = Body("base_link", None, np.zeros(3))
    bodies = [root]

    def physics(body, node, T):
        ph = node.get("physics") or {}
        if "mass" not in ph:
            return
        R = T[:3, :3]
        com = T @ np.r_[ph.get("centerOfMass", [0, 0, 0]), 1]
        i = ph.get("inertiaMatrix", [0, 0, 0, 0, 0, 0])
        I = np.array([[i[0], i[3], i[4]], [i[3], i[1], i[5]], [i[4], i[5], i[2]]])
        I = R @ I @ R.T
        if body.joint is not None:
            a = body.joint["axis"]
            I = I - SOURCE_ROTOR_INERTIA * np.outer(a, a)
        ev = np.linalg.eigvalsh(I)
        if ev[0] <= 0 or ev[0] + ev[1] < ev[2] * (1 - 1e-6):
            sys.exit(f"error: {body.name}: inertia {ev} is not physical (positive, triangle inequality)")
        body.mass, body.com, body.inertia = ph["mass"], com[:3], I

    def walk(node, T, body):
        for child in node.get("children") or []:
            if not isinstance(child, dict):
                continue
            t = child.get("nodeType")
            if t == "HingeJoint":
                jp = child.get("jointParameters") or {}
                sol = child["endPoint"]
                name = SOLID_FIX.get(sol["name"], sol["name"])
                assert name in IDS, name
                anchor = (T @ np.r_[jp.get("anchor", [0, 0, 0]), 1])[:3]
                axis = T[:3, :3] @ np.array(jp.get("axis", [1, 0, 0]), float)
                b = Body(name, body, anchor)
                b.joint = dict(axis=axis / np.linalg.norm(axis), ref=jp.get("position", 0.0))
                bodies.append(b)
                Ts = T @ local_tf(sol)
                physics(b, sol, Ts)
                walk(sol, Ts, b)
            elif t in DEVICE_TYPES:
                body.devices.append((t, child.get("name", t), T @ local_tf(child), child))
            elif t in ("Transform", "Group", "Pose"):
                walk(child, T @ local_tf(child), body)
            elif t == "Shape":
                g = child.get("geometry") or {}
                if g.get("nodeType") != "IndexedFaceSet":
                    continue  # small sensor-marker cylinders
                P = np.array(g["coord"]["point"], float).reshape(-1, 3)
                V = (T[:3, :3] @ P.T).T + T[:3, 3]
                faces, poly = [], []
                for idx in g["coordIndex"]:
                    if idx == -1:
                        faces += [(poly[0], poly[k], poly[k + 1]) for k in range(1, len(poly) - 1)]
                        poly = []
                    else:
                        poly.append(idx)
                color = tuple(round(c, 3) for c in child.get("color", [0.6, 0.6, 0.6]))
                body.meshes.setdefault(color, []).append((V, np.array(faces, int)))

    physics(root, data, np.eye(4))  # robot frame == model frame (the Webots world rotation is dropped)
    walk(data, np.eye(4), root)
    names = sorted(b.name for b in bodies[1:])
    assert names == sorted(NAMES), f"joint set mismatch: {names}"
    return bodies


def write_stl(path, V, F):
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as f:
        f.write(b"yanshee_sim".ljust(80, b" "))
        f.write(struct.pack("<I", len(F)))
        rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"], rec["v"] = n, tri
        f.write(rec.tobytes())


def export_meshes(bodies, out_dirs):
    """Write one STL per (body, colour) plus a combined collision STL, in each body's frame."""
    files = {}
    for b in bodies:
        entries, allV, allF, off = [], [], [], 0
        for k, (color, parts) in enumerate(sorted(b.meshes.items())):
            V = np.vstack([p[0] for p in parts]) - b.origin
            F, o = [], 0
            for p in parts:
                F.append(p[1] + o)
                o += len(p[0])
            F = np.vstack(F)
            fname = f"{b.name}_{k}.stl"
            for d in out_dirs:
                write_stl(os.path.join(d, fname), V, F)
            entries.append((fname, color))
            allV.append(V)
            allF.append(F + off)
            off += len(V)
        col = f"{b.name}_collision.stl"
        for d in out_dirs:
            write_stl(os.path.join(d, col), np.vstack(allV), np.vstack(allF))
        files[b.name] = (entries, col)
    return files


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def sole_box(b):
    """(pos, half-size) in the body frame of a flat sole under body b: the outline of its downward-facing
    mesh faces within 8 mm of the lowest vertex, SOLE_INSET smaller, SOLE_THICKNESS thick, bottom at that vertex."""
    P, zmin = [], min(V[:, 2].min() for parts in b.meshes.values() for V, _ in parts)
    for parts in b.meshes.values():
        for V, F in parts:
            n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
            down = F[n[:, 2] < -0.9 * np.linalg.norm(n, axis=1)]
            P.append(V[np.unique(down.ravel())])
    P = np.vstack(P)
    P = P[P[:, 2] < zmin + 0.008]
    lo, hi = P[:, :2].min(axis=0), P[:, :2].max(axis=0)
    pos = np.array([*((lo + hi) / 2), zmin + SOLE_THICKNESS / 2]) - b.origin
    half = np.array([*((hi - lo) / 2 - SOLE_INSET), SOLE_THICKNESS / 2])
    return pos, half


def build_mjcf(bodies, files, z0=None):
    """MJCF text. z0 = spawn height of base_link; default: the saved pose's lowest vertex FOOT_CLEARANCE above
    the floor (main() then corrects it for RESET_POSE with spawn_height())."""
    if z0 is None:
        z0 = FOOT_CLEARANCE - min(v[0][:, 2].min() for b in bodies for parts in b.meshes.values() for v in parts)
    children = {}
    for b in bodies[1:]:
        children.setdefault(b.parent.name, []).append(b)
    colors = sorted({c for es, _ in files.values() for _, c in es})
    cname = {c: f"c{i}" for i, c in enumerate(colors)}

    L = ['<mujoco model="yanshee">',
         '  <!-- Generated by tools/build_models.py from source_model/robot.json. Do not edit by hand. -->',
         '  <compiler angle="radian" meshdir="meshes" autolimits="true"/>',
         '  <option timestep="0.002" integrator="implicitfast"/>',
         '  <default>',
         f'    <joint damping="{SERVO_DAMPING}" armature="{SERVO_ARMATURE}" frictionloss="0.01"/>',
         f'    <position kp="{SERVO_KP}" forcerange="-{SERVO_TORQUE} {SERVO_TORQUE}"/>',
         '    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" mass="0"/></default>',
         '    <default class="collision"><geom type="mesh" group="3" rgba="0.8 0.3 0.3 0.4" mass="0"'
         ' contype="1" conaffinity="0" condim="3" friction="1.0 0.02 0.001"/></default>',
         '  </default>',
         '  <asset>']
    for c in colors:
        L.append(f'    <material name="{cname[c]}" rgba="{fmt(c)} 1" specular="0.4" shininess="0.6"/>')
    for b in bodies:
        es, col = files[b.name]
        for f, _ in es:
            L.append(f'    <mesh name="{f[:-4]}" file="{f}"/>')
        L.append(f'    <mesh name="{col[:-4]}" file="{col}"/>')
    L.append('  </asset>')
    L.append('  <worldbody>')

    def emit(b, indent):
        sp = " " * indent
        if b.parent is None:
            L.append(f'{sp}<body name="{b.name}" pos="0 0 {z0:.5f}">')
            L.append(f'{sp}  <freejoint name="root"/>')
        else:
            L.append(f'{sp}<body name="{b.name}" pos="{fmt(b.origin - b.parent.origin)}">')
            lo, hi = LIMITS[b.name]
            L.append(f'{sp}  <joint name="{b.name}" axis="{fmt(b.joint["axis"])}" ref="{b.joint["ref"]:.6g}"'
                     f' range="{math.radians(lo):.6g} {math.radians(hi):.6g}"/>')
        I = b.inertia
        L.append(f'{sp}  <inertial pos="{fmt(b.com - b.origin)}" mass="{b.mass:.6g}"'
                 f' fullinertia="{fmt([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>')
        es, col = files[b.name]
        for f, c in es:
            L.append(f'{sp}  <geom class="visual" mesh="{f[:-4]}" material="{cname[c]}"/>')
        if b.name in SOLE_BODIES:  # contype 2: the mesh never meets the floor (conaffinity 1), the sole does
            pos, half = sole_box(b)
            L.append(f'{sp}  <geom class="collision" name="{b.name}_col" mesh="{col[:-4]}" contype="2"/>')
            L.append(f'{sp}  <geom class="collision" name="{b.name}_sole" type="box" pos="{fmt(pos)}" size="{fmt(half)}"/>')
        else:
            L.append(f'{sp}  <geom class="collision" name="{b.name}_col" mesh="{col[:-4]}"/>')
        for t, name, Tw, node in b.devices:
            p = Tw[:3, 3] - b.origin
            if t == "Camera":
                fovy = 51.1  # from real intrinsics fy=502.2 at 480 px (Serenade calibration)
                L.append(f'{sp}  <camera name="head_camera" pos="{fmt(p)}" quat="{fmt(mat_to_quat(Tw[:3,:3]))}"'
                         f' fovy="{fovy}" resolution="640 480"/>')
            elif t == "InertialUnit":
                L.append(f'{sp}  <site name="imu" pos="{fmt(p)}" size="0.004" rgba="0 1 0 0.5" group="4"/>')
            elif t == "DistanceSensor":
                L.append(f'{sp}  <site name="{name}" pos="{fmt(p)}" quat="{fmt(mat_to_quat(Tw[:3,:3] @ rot([0,1,0,math.pi/2])))}"'
                         f' size="0.003" rgba="1 0 0 0.5" group="4"/>')
        for c in children.get(b.name, []):
            emit(c, indent + 2)
        L.append(f'{sp}</body>')

    emit(bodies[0], 4)
    L.append('  </worldbody>')
    L.append('  <actuator>')
    order = sorted(bodies[1:], key=lambda b: IDS[b.name])
    for b in order:
        lo, hi = LIMITS[b.name]
        L.append(f'    <position name="{b.name}" joint="{b.name}" ctrlrange="{math.radians(lo):.6g} {math.radians(hi):.6g}"/>')
    L.append('  </actuator>')
    L.append('  <sensor>')
    L.append('    <gyro name="imu_gyro" site="imu"/>')
    L.append('    <accelerometer name="imu_accel" site="imu"/>')
    L.append('    <framequat name="imu_quat" objtype="site" objname="imu"/>')
    for b in bodies:
        for t, name, _, _ in b.devices:
            if t == "DistanceSensor":
                L.append(f'    <rangefinder name="{name}" site="{name}" cutoff="1.5"/>')
    L.append('  </sensor>')
    reset = {n: math.radians(d) for n, d in RESET_POSE.items()}  # keyframe = servos.RESET_POSE
    qpos = f"0 0 {z0:.5f} 1 0 0 0 " + fmt([reset[b.name] for b in bodies[1:]])
    ctrl = fmt([reset[b.name] for b in order])
    L.append('  <keyframe>')
    L.append(f'    <key name="reset" qpos="{qpos}" ctrl="{ctrl}"/>')
    L.append('  </keyframe>')
    L.append('</mujoco>')
    return "\n".join(L) + "\n"


SCENE = """<mujoco model="yanshee scene">
  <!-- Floor, light and camera around model/yanshee.xml -->
  <include file="yanshee.xml"/>
  <statistic center="0 0 0.18" extent="0.45"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="-115" elevation="-12" offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.45 0.55" rgb2="0.05 0.07 0.1" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.22 0.25" rgb2="0.25 0.28 0.31" width="300" height="300" mark="edge" markrgb="0.35 0.38 0.42"/>
    <material name="grid" texture="grid" texrepeat="10 10" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.5 1.5" dir="-0.3 0.3 -1" directional="true"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="grid" contype="0" conaffinity="1" friction="1.0 0.02 0.001"/>
    <camera name="front" pos="0 0.85 0.3" xyaxes="-1 0 0 0 -0.25 1"/>
  </worldbody>
</mujoco>
"""


# Floor friction of the soft play mat our robot is tested on: the value at which the sim's GetupFront falls forward
# with the fitted SERVO_TORQUE, as the robot's did on the mat (it stands up at the default 1.0, as on a smooth floor).
MAT_FRICTION = 1.4


def scene_mat():
    """SCENE with the mat's floor friction (model/scene_mat.xml)."""
    floor = 'conaffinity="1" friction="1.0 0.02 0.001"'
    assert SCENE.count(floor) == 1
    return (SCENE.replace("<!-- Floor, light and camera", f"<!-- Play mat (floor friction {MAT_FRICTION}): floor, light and camera")
            .replace(floor, f'conaffinity="1" friction="{MAT_FRICTION} 0.02 0.001"'))


def spawn_height(scene_path):
    """base_link height that puts the lowest mesh vertex of the reset keyframe FOOT_CLEARANCE above the floor."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(scene_path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("reset").id)
    mujoco.mj_kinematics(m, d)
    low = math.inf
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        low = min(low, (V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min())
    return d.qpos[2] + FOOT_CLEARANCE - low


LAUNCH = '''"""RViz view of the Yanshee URDF with a slider per servo.

Generated by tools/build_models.py (RESET_POSE_RAD comes from yanshee_sim/servos.py). Do not edit by hand.

    ros2 launch yanshee_description display.launch.py            # sliders start in the reset pose
    ros2 launch yanshee_description display.launch.py gui:=false # e.g. when another node publishes /joint_states

Slider values are radians; servo degrees = rad * 57.2958 (the same numbers YanAPI uses).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RESET_POSE_RAD = {
%s
}


def generate_launch_description():
    share = get_package_share_directory("yanshee_description")
    with open(os.path.join(share, "urdf", "yanshee.urdf")) as f:
        urdf = f.read()
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"zeros": RESET_POSE_RAD}], condition=IfCondition(gui)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "yanshee.rviz")],
             condition=IfCondition(rviz)),
    ])
'''


def build_launch():
    return LAUNCH % "\n".join(f'    "{n}": {math.radians(RESET_POSE[n]):.4f},' for n in NAMES)


def build_urdf(bodies, files):
    L = ['<?xml version="1.0"?>',
         '<!-- Generated by yanshee_sim/tools/build_models.py from the Webots Yanshee model. Do not edit by hand.',
         '     Joint names are YanAPI servo names; joint position (rad) = servo angle (deg) in radians. -->',
         '<robot name="yanshee">']
    colors = sorted({c for es, _ in files.values() for _, c in es})
    for i, c in enumerate(colors):
        L.append(f'  <material name="c{i}"><color rgba="{fmt(c)} 1"/></material>')
    cidx = {c: i for i, c in enumerate(colors)}
    mesh_uri = "package://yanshee_description/meshes/"
    # massless root (KDL cannot use a root link with inertia); the torso hangs off it with a fixed joint
    L.append('  <link name="base_link"/>')
    L.append('  <joint name="base_to_torso" type="fixed"><parent link="base_link"/><child link="torso_link"/>'
             '<origin xyz="0 0 0" rpy="0 0 0"/></joint>')
    for b in bodies:
        I = b.inertia
        L.append(f'  <link name="{b.name + "_link" if b.parent else "torso_link"}">')
        L.append(f'    <inertial><origin xyz="{fmt(b.com - b.origin)}" rpy="0 0 0"/><mass value="{b.mass:.6g}"/>'
                 f'<inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" iyy="{I[1,1]:.6g}"'
                 f' iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/></inertial>')
        es, col = files[b.name]
        for f, c in es:
            L.append(f'    <visual><geometry><mesh filename="{mesh_uri}{f}"/></geometry><material name="c{cidx[c]}"/></visual>')
        L.append(f'    <collision><geometry><mesh filename="{mesh_uri}{col}"/></geometry></collision>')
        L.append('  </link>')
    for b in bodies[1:]:
        parent = "torso_link" if b.parent.parent is None else b.parent.name + "_link"
        r, p, y = mat_to_rpy(rot([*b.joint["axis"], -b.joint["ref"]]))
        lo, hi = LIMITS[b.name]
        L.append(f'  <joint name="{b.name}" type="revolute">')
        L.append(f'    <parent link="{parent}"/><child link="{b.name}_link"/>')
        L.append(f'    <origin xyz="{fmt(b.origin - b.parent.origin)}" rpy="{r:.6g} {p:.6g} {y:.6g}"/>')
        L.append(f'    <axis xyz="{fmt(b.joint["axis"])}"/>')
        L.append(f'    <limit lower="{math.radians(lo):.6g}" upper="{math.radians(hi):.6g}" effort="{SERVO_TORQUE}" velocity="6.0"/>')
        L.append(f'    <dynamics damping="{SERVO_DAMPING}" friction="0.01"/>')
        L.append('  </joint>')
    # head camera as a fixed frame (REP-103 optical convention: z forward, x right, y down)
    for b in bodies:
        for t, _, Tw, _ in b.devices:
            if t == "Camera":
                Rcam = Tw[:3, :3] @ np.diag([1, -1, -1])  # Webots/MuJoCo camera (-z fwd, y up) -> optical frame
                r, p, y = mat_to_rpy(Rcam)
                L.append('  <link name="head_camera_optical_frame"/>')
                L.append('  <joint name="head_camera_joint" type="fixed">')
                L.append(f'    <parent link="{b.name}_link"/><child link="head_camera_optical_frame"/>')
                L.append(f'    <origin xyz="{fmt(Tw[:3,3] - b.origin)}" rpy="{r:.6g} {p:.6g} {y:.6g}"/>')
                L.append('  </joint>')
    L.append('</robot>')
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(
        description="Regenerate model/ (MuJoCo MJCF + meshes) and the ROS 2 package (URDF, meshes, display launch "
                    "file) from the Webots export. The reset keyframe comes from yanshee_sim/servos.py RESET_POSE.")
    ap.add_argument("--src", default=SRC, help="Webots JSON export (default: source_model/robot.json, Git LFS)")
    args = ap.parse_args()
    bodies = parse(args.src)
    ros_meshes = os.path.join(ROS_PKG, "meshes")
    mj_meshes = os.path.join(MODEL_DIR, "meshes")
    for d in (ros_meshes, mj_meshes):
        os.makedirs(d, exist_ok=True)
    files = export_meshes(bodies, [mj_meshes, ros_meshes])
    with open(os.path.join(MODEL_DIR, "scene.xml"), "w") as f:
        f.write(SCENE)
    with open(os.path.join(MODEL_DIR, "scene_mat.xml"), "w") as f:
        f.write(scene_mat())
    mjcf = os.path.join(MODEL_DIR, "yanshee.xml")
    with open(mjcf, "w") as f:
        f.write(build_mjcf(bodies, files))
    z0 = spawn_height(os.path.join(MODEL_DIR, "scene.xml"))  # rest the RESET_POSE feet on the floor
    with open(mjcf, "w") as f:
        f.write(build_mjcf(bodies, files, z0))
    os.makedirs(os.path.join(ROS_PKG, "launch"), exist_ok=True)
    with open(os.path.join(ROS_PKG, "launch", "display.launch.py"), "w") as f:
        f.write(build_launch())
    os.makedirs(os.path.join(ROS_PKG, "urdf"), exist_ok=True)
    with open(os.path.join(ROS_PKG, "urdf", "yanshee.urdf"), "w") as f:
        f.write(build_urdf(bodies, files))
    total = sum(b.mass for b in bodies)
    print(f"bodies: {len(bodies)}  joints: {len(bodies) - 1}  total mass: {total:.3f} kg  spawn z {z0:.5f} m")
    for b in bodies[1:]:
        print(f"  {IDS[b.name]:2d} {b.name:18s} saved pose {math.degrees(b.joint['ref']):7.2f} deg"
              f"  reset {RESET_POSE[b.name]:3d} deg  axis {fmt(b.joint['axis'])}")


if __name__ == "__main__":
    main()
