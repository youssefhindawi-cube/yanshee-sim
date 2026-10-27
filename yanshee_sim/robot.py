"""One robot interface, two backends: the MuJoCo simulator and the real Yanshee (via YanAPI).

    from yanshee_sim import connect
    bot = connect("sim")                       # or connect("real", ip="192.168.3.13")
    bot.set_servos_angles({"NeckLR": 60}, runtime=800)
    bot.wait(0.8)
    print(bot.get_servos_angles(["NeckLR"]))

Angles are servo degrees (0-180) on both backends; method names and return shapes follow YanAPI.
"""
import math
import os
import threading
import time

from .servos import IDS, LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, NAMES, RESET_POSE, check_angles

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "model", "scene.xml")


class ServoCommandError(ValueError):
    pass


class Robot:
    """Shared validation; backends implement _send/_read."""

    strict = True  # raise on out-of-range angles instead of clamping

    def _prepare(self, angles, runtime):
        problems = check_angles(angles)
        if problems:
            if self.strict:
                raise ServoCommandError("; ".join(problems))
            angles = {n: min(max(d, LIMITS[n][0]), LIMITS[n][1]) for n, d in angles.items() if n in LIMITS}
        runtime = int(min(max(runtime, MIN_RUNTIME_MS), MAX_RUNTIME_MS))
        return angles, runtime

    def set_servos_angles(self, angles, runtime=200):
        """Move servos to {name: degrees} over `runtime` ms. Returns immediately, like YanAPI."""
        angles, runtime = self._prepare(angles, runtime)
        return self._send(angles, runtime)

    def sync_set_servo_rotate(self, angles, runtime=200):
        res = self.set_servos_angles(angles, runtime)
        self.wait(min(max(runtime, MIN_RUNTIME_MS), MAX_RUNTIME_MS) / 1000)
        return res

    def get_servos_angles(self, names=None):
        return {"code": 0, "data": self._read(list(names or NAMES)), "msg": "success"}

    def get_servo_angle_value(self, name):
        return self._read([name])[name]

    def reset(self, runtime=1000):
        """Return to the standing reset pose through servo commands."""
        return self.sync_set_servo_rotate(dict(RESET_POSE), runtime)

    def wait(self, seconds):
        time.sleep(seconds)

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SimRobot(Robot):
    """MuJoCo backend. Physics runs in a background thread at real time (or on demand with realtime=False)."""

    def __init__(self, viewer=True, realtime=True, fixed_base=False, manual=False, scene=SCENE):
        """manual=True: servo targets come from the viewer's Control sliders instead of set_servos_angles."""
        import mujoco
        self._mj = mujoco
        if fixed_base:
            spec = mujoco.MjSpec.from_file(scene)
            base = spec.body("base_link")
            for j in list(base.joints):
                spec.delete(j)
            base.pos = [0, 0, 0.1]  # hang 10 cm above the floor
            for k in spec.keys:  # the keyframe loses the 7 free-joint values
                k.qpos = list(k.qpos)[7:]
            self.model = spec.compile()
        else:
            self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        m = self.model
        mujoco.mj_resetDataKeyframe(m, self.data, m.key("reset").id)
        mujoco.mj_forward(m, self.data)
        self._act = {n: m.actuator(n).id for n in NAMES}
        self._qadr = {n: m.jnt_qposadr[m.joint(n).id] for n in NAMES}
        # per actuator: (start rad, end rad, t_start, t_end)
        self._moves = {n: (self.data.ctrl[i], self.data.ctrl[i], 0.0, 0.0) for n, i in self._act.items()}
        self._lock = threading.RLock()
        self.manual = manual
        self._viewer = None
        self.realtime = realtime
        if viewer:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(m, self.data, show_left_ui=False)
        self._running = realtime
        if realtime:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    # -- physics --------------------------------------------------------------
    def _step(self):
        t = self.data.time
        for n, (a, b, t0, t1) in ({} if self.manual else self._moves).items():
            s = 1.0 if t >= t1 else max(0.0, (t - t0) / (t1 - t0))
            self.data.ctrl[self._act[n]] = a + (b - a) * s  # linear profile, like the servo's timed move
        self._mj.mj_step(self.model, self.data)

    def _loop(self):
        dt = self.model.opt.timestep
        start_wall, start_sim = time.perf_counter(), self.data.time
        last_sync = 0.0
        while self._running:
            with self._lock:
                self._step()
                sim_t = self.data.time
            now = time.perf_counter()
            if self._viewer is not None and now - last_sync > 1 / 60:
                if not self._viewer.is_running():
                    self._running = False
                    break
                with self._lock:
                    self._viewer.sync()
                last_sync = now
            ahead = (sim_t - start_sim) - (now - start_wall)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.25:  # fell behind (slow machine): re-anchor instead of spiralling
                start_wall, start_sim = now, sim_t

    # -- backend interface ----------------------------------------------------
    def _send(self, angles, runtime):
        with self._lock:
            t = self.data.time
            for n, deg in angles.items():
                cur = self.data.ctrl[self._act[n]]
                self._moves[n] = (cur, math.radians(deg), t, t + runtime / 1000)
        return {"code": 0, "data": {n: True for n in angles}, "msg": "success"}

    def _read(self, names):
        with self._lock:
            return {n: int(round(math.degrees(self.data.qpos[self._qadr[n]]))) for n in names}

    def wait(self, seconds):
        if self.realtime:
            time.sleep(seconds)
        else:
            with self._lock:
                for _ in range(int(round(seconds / self.model.opt.timestep))):
                    self._step()
                if self._viewer is not None:
                    self._viewer.sync()

    # -- simulator-only helpers -------------------------------------------------
    def get_target_angles(self):
        """Current servo targets in degrees (what the Control sliders show in manual mode)."""
        with self._lock:
            return {n: math.degrees(self.data.ctrl[i]) for n, i in self._act.items()}

    def jump_to(self, angles):
        """Teleport joints and targets to a {servo: deg} pose (no motion). Use while the robot is held (fixed_base)."""
        with self._lock:
            for n, deg in angles.items():
                if n in self._act:
                    rad = math.radians(deg)
                    self.data.qpos[self._qadr[n]] = rad
                    self.data.ctrl[self._act[n]] = rad
                    self._moves[n] = (rad, rad, 0.0, 0.0)
            self.data.qvel[:] = 0
            self._mj.mj_forward(self.model, self.data)

    def torso_tilt_deg(self):
        """Angle between the torso's up axis and world up (0 = upright). Useful to detect falls."""
        with self._lock:
            R = self.data.xmat[self.model.body("base_link").id].reshape(3, 3)
            return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))

    def get_sensors_gyro(self):
        """Simulated IMU: gyro (rad/s) and accelerometer (m/s^2) in the IMU site frame.
        Axis conventions differ from the real robot (raw IMU frame there is left-back-up); see README."""
        with self._lock:
            g = self.data.sensor("imu_gyro").data.copy()
            a = self.data.sensor("imu_accel").data.copy()
        return {"code": 0, "data": {"gyro": [{"gyro-x": g[0], "gyro-y": g[1], "gyro-z": g[2],
                                              "accel-x": a[0], "accel-y": a[1], "accel-z": a[2]}]}, "msg": "success"}

    def is_running(self):
        return self._viewer.is_running() if self._viewer is not None else True

    def close(self):
        self._running = False
        if self.realtime:
            self._thread.join(timeout=1)
        if self._viewer is not None:
            self._viewer.close()


class RealRobot(Robot):
    """Real Yanshee through the official YanAPI.py (REST on port 9090)."""

    def __init__(self, ip, yanapi_dir=None, min_battery=30):
        import importlib
        import socket
        import sys
        try:  # YanAPI's requests have no timeout, so check reachability first
            socket.create_connection((ip, 9090), timeout=3).close()
        except OSError as e:
            raise ConnectionError(f"cannot reach {ip}:9090 ({e}); check the robot is on and on the same Wi-Fi") from None
        yanapi_dir = yanapi_dir or os.environ.get("YANAPI_DIR") or os.path.join(
            os.path.dirname(ROOT), "yanshee robot", "yanshee", "yanshee_examples", "yanshee_examples")
        if yanapi_dir not in sys.path:
            sys.path.insert(0, yanapi_dir)
        self.api = importlib.import_module("YanAPI")
        self.api.yan_api_init(ip)
        self.ip = ip
        battery = self.api.get_robot_battery_value()
        if battery < 0:
            raise ConnectionError(f"robot at {ip} did not answer; check the IP and Wi-Fi (curl http://{ip}:9090/v1/devices/battery)")
        if battery < min_battery:
            raise RuntimeError(f"battery {battery}% is below {min_battery}%: charge before moving the robot")
        self.battery = battery

    def _send(self, angles, runtime):
        res = self.api.set_servos_angles({n: int(round(d)) for n, d in angles.items()}, runtime)
        if res.get("code") != 0:
            raise RuntimeError(f"robot refused servo command: {res}")
        return res

    def _read(self, names):
        res = self.api.get_servos_angles(names)
        if res.get("code") != 0:
            raise RuntimeError(f"servo read failed: {res}")
        return {n: res["data"][n] for n in names}

    def stop(self):
        """Stop any built-in motion (the chest button and head emergency stop still work too)."""
        return self.api.stop_play_motion()


def connect(target="sim", ip=None, **kw):
    """connect("sim", viewer=True) or connect("real", ip="192.168.x.y")."""
    if target == "sim":
        return SimRobot(**kw)
    if target == "real":
        if not ip:
            raise ValueError("connect('real') needs ip=")
        return RealRobot(ip, **kw)
    raise ValueError("target must be 'sim' or 'real'")


__all__ = ["connect", "SimRobot", "RealRobot", "ServoCommandError", "IDS"]
