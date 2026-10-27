"""One robot interface, two backends: the MuJoCo simulator and the real Yanshee (REST, see rest.py).

    from yanshee_sim import connect
    bot = connect("sim")                       # or connect("real", ip="192.168.3.13")
    bot.set_servos_angles({"NeckLR": 60}, runtime=800)
    bot.wait(0.8)
    print(bot.get_servos_angles(["NeckLR"]))

Angles are servo degrees (0-180) on both backends; method names and return shapes follow YanAPI.
"""
import math
import numbers
import os
import threading
import time

from .rest import RobotError, RobotProtocolError, RobotUnreachable, SPEEDS, YansheeRest
from .servos import IDS, LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, NAMES, RESET_POSE, check_angles

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "model", "scene.xml")
SERVO_MODES = ("work", "program")
LED_COLORS = {"button": ("white", "red", "green", "blue", "yellow", "purple", "cyan"), "camera": ("red", "green", "blue")}
LED_MODES = {"button": ("on", "off", "blink", "breath"), "camera": ("on", "off", "blink")}
# robot system motions (power off, servo calibration, burn-in...): never started by play_builtin unless asked
SYSTEM_MOTION_PREFIXES = ("shutdown", "actionaging", "calibration", "resetswitch", "enterenergysaving",
                          "exitenergysaving")


def is_system_motion(name):
    """True for the robot's system motions (Shutdown, calibration, ActionAging, ...), any case or spacing."""
    return isinstance(name, str) and "".join(name.split()).lower().startswith(SYSTEM_MOTION_PREFIXES)
_YANAPI_LOCK = threading.Lock()


class ServoCommandError(ValueError):
    pass


class RobotCommandError(RuntimeError):
    """The robot answered with a non-zero code. .code and .msg hold the robot's reply."""

    def __init__(self, what, res):
        self.code, self.msg, self.reply = res.get("code"), res.get("msg", ""), res
        hint = " (low battery: charge the robot)" if self.code == 6 else ""
        super().__init__(f"robot refused {what}: code {self.code} {self.msg}{hint}".rstrip())


def _is_number(v):
    return isinstance(v, numbers.Real) and not isinstance(v, bool) and math.isfinite(v)


def _check_names(names):
    names = [names] if isinstance(names, str) else list(names)
    unknown = [n for n in names if n not in LIMITS]
    if unknown:
        raise ServoCommandError("; ".join(f"unknown servo '{n}'" for n in unknown))
    return names


def _check_mode(mode, names):
    if mode not in SERVO_MODES:
        raise ServoCommandError(f"servo mode must be one of {SERVO_MODES}, not {mode!r}")
    return _check_names(names)


class Robot:
    """Shared validation; backends implement _send/_read."""

    strict = True  # raise on out-of-range angles instead of clamping

    def _prepare(self, angles, runtime):
        bad = [n for n, d in angles.items() if not _is_number(d)]
        if bad:  # NaN/inf/non-numbers are refused even when not strict: clamping cannot fix them
            raise ServoCommandError("; ".join(f"{n}={angles[n]!r} is not a finite number" for n in bad))
        if not _is_number(runtime):
            raise ServoCommandError(f"runtime={runtime!r} is not a finite number")
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
        return {"code": 0, "data": self._read(_check_names(NAMES if names is None else names)), "msg": "success"}

    def get_servo_angle_value(self, name):
        return self._read(_check_names([name]))[name]

    def reset(self, runtime=1000):
        """Return to the standing reset pose through servo commands."""
        return self.sync_set_servo_rotate(dict(RESET_POSE), runtime)

    def wait(self, seconds):
        time.sleep(max(0.0, seconds))

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SimRobot(Robot):
    """MuJoCo backend. Physics runs in a background thread at real time (or on demand with realtime=False)."""

    def __init__(self, viewer=True, realtime=True, fixed_base=False, manual=False, scene=SCENE, spec_hook=None):
        """manual=True: servo targets come from the viewer's Control sliders instead of set_servos_angles.
        spec_hook(spec): edits the MjSpec before compiling (e.g. sim_extras.add_leds adds LED markers)."""
        import mujoco
        self._mj = mujoco
        self.fixed_base = fixed_base
        if fixed_base or spec_hook is not None:
            spec = mujoco.MjSpec.from_file(scene)
            if fixed_base:
                base = spec.body("base_link")
                for j in list(base.joints):
                    spec.delete(j)
                base.pos = [0, 0, 0.1]  # hang 10 cm above the floor
                for k in spec.keys:  # the keyframe loses the 7 free-joint values
                    k.qpos = list(k.qpos)[7:]
            if spec_hook is not None:
                spec_hook(spec)
            self.model = spec.compile()
        else:
            self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._key = m.key("reset").id
        mujoco.mj_resetDataKeyframe(m, self.data, self._key)
        mujoco.mj_forward(m, self.data)
        self._act = {n: m.actuator(n).id for n in NAMES}
        self._qadr = {n: m.jnt_qposadr[m.joint(n).id] for n in NAMES}
        self._kp = {n: m.actuator_gainprm[i, 0] for n, i in self._act.items()}
        self._bias = {n: m.actuator_biasprm[i, 1:3].copy() for n, i in self._act.items()}
        self._modes = {n: "work" for n in NAMES}
        # per actuator: (start rad, end rad, t_start, t_end)
        self._moves = {n: (self.data.ctrl[i], self.data.ctrl[i], 0.0, 0.0) for n, i in self._act.items()}
        self._last_time = self.data.time
        self._lock = threading.RLock()
        self.led = {"button": {"color": "blue", "mode": "breath"}, "camera": {"color": "green", "mode": "on"}}
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
    def _hold(self):
        """Cancel all moves: every servo holds its current target."""
        self._moves = {n: (self.data.ctrl[i], self.data.ctrl[i], 0.0, 0.0) for n, i in self._act.items()}

    def _restart(self):
        """Back to the reset keyframe (standing, all servos working)."""
        self._mj.mj_resetDataKeyframe(self.model, self.data, self._key)
        for n in NAMES:
            self._set_gain(n, "work")
        self._mj.mj_forward(self.model, self.data)
        self._hold()
        self._last_time = self.data.time

    def _step(self):
        t = self.data.time
        if t < self._last_time:  # the viewer reset the data (Backspace): stale move times would replay old moves
            self._restart()
            t = self.data.time
        for n, (a, b, t0, t1) in ({} if self.manual else self._moves).items():
            s = 1.0 if t >= t1 else max(0.0, (t - t0) / (t1 - t0))
            self.data.ctrl[self._act[n]] = a + (b - a) * s  # linear profile, like the servo's timed move
        self._mj.mj_step(self.model, self.data)
        self._last_time = self.data.time

    def _loop(self):
        start_wall, start_sim = time.perf_counter(), self.data.time
        last_sync, prev_t = 0.0, self.data.time
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
            if sim_t < prev_t:  # data was reset: re-anchor the real-time clock
                start_wall, start_sim = now, sim_t
            prev_t = sim_t
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
        seconds = max(0.0, seconds)
        if self.realtime:
            time.sleep(seconds)
        else:
            with self._lock:
                for _ in range(int(round(seconds / self.model.opt.timestep))):
                    self._step()
                if self._viewer is not None:
                    self._viewer.sync()

    # -- servo modes, stop, LEDs ----------------------------------------------
    def _set_gain(self, n, mode):
        i = self._act[n]
        if mode == "program":  # limp: no position gain, joint moves freely (only its damping/friction)
            self.model.actuator_gainprm[i, 0] = 0.0
            self.model.actuator_biasprm[i, 1:3] = 0.0
        else:
            self.model.actuator_gainprm[i, 0] = self._kp[n]
            self.model.actuator_biasprm[i, 1:3] = self._bias[n]
        self._modes[n] = mode

    def set_servos_mode(self, mode, names):
        """'program' makes the servos limp (PRP: pose them by hand / let gravity act); 'work' holds where they are."""
        names = _check_mode(mode, names)
        with self._lock:
            for n in names:
                if mode == "work" and self._modes[n] == "program":
                    lo, hi = self.model.actuator_ctrlrange[self._act[n]]
                    q = min(max(self.data.qpos[self._qadr[n]], lo), hi)
                    self.data.ctrl[self._act[n]] = q
                    self._moves[n] = (q, q, 0.0, 0.0)
                self._set_gain(n, mode)
        return {"code": 0, "data": {n: True for n in names}, "msg": "success"}

    def get_servos_mode(self, names=None):
        names = _check_names(NAMES if names is None else names)
        with self._lock:
            return {"code": 0, "data": {n: self._modes[n] for n in names}, "msg": "success"}

    def stop_all(self):
        """Cancel every servo move in progress and hold the current pose."""
        with self._lock:
            self._hold()
        return {"code": 0, "data": {}, "msg": "success"}

    def respawn(self):
        """Teleport back to the standing reset pose (e.g. after a fall), all servos working."""
        with self._lock:
            self._restart()
        return {"code": 0, "data": {}, "msg": "success"}

    def set_led(self, type, color, mode):
        """Remember an LED setting (YanAPI set_robot_led values); the renderer shows it."""
        if type not in LED_COLORS or color not in LED_COLORS[type] or mode not in LED_MODES[type]:
            return {"code": 1, "data": {}, "msg": f"bad led setting {type}/{color}/{mode}"}
        with self._lock:
            self.led[type] = {"color": color, "mode": mode}
        return {"code": 0, "data": {}, "msg": "success"}

    def get_led(self):
        with self._lock:
            return {"code": 0, "data": [{"type": t, **v} for t, v in self.led.items()], "msg": "success"}

    # -- simulator-only helpers -------------------------------------------------
    def get_target_angles(self):
        """Current servo targets in degrees (what the Control sliders show in manual mode)."""
        with self._lock:
            return {n: math.degrees(self.data.ctrl[i]) for n, i in self._act.items()}

    def jump_to(self, angles):
        """Teleport joints and targets to a {servo: deg} pose (no motion). Use while the robot is held (fixed_base)."""
        with self._lock:
            for n, deg in angles.items():
                if n in self._act and _is_number(deg):
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
        """Simulated IMU in YanAPI's shape: gyro (rad/s), accel (m/s^2), compass (model field units) and
        euler-x/y/z (deg, roll/pitch/yaw of the IMU in the model frame; 0 when upright facing +y).
        Axis conventions differ from the real robot (raw IMU frame there is left-back-up); see README.
        With fixed_base the torso is welded to the world, so accel is gravity seen from the IMU and gyro is 0."""
        import numpy as np
        with self._lock:
            R = self.data.site("imu").xmat.reshape(3, 3).copy()
            if self.fixed_base:
                g, a = np.zeros(3), R.T @ -self.model.opt.gravity
            else:
                g = self.data.sensor("imu_gyro").data.copy()
                a = self.data.sensor("imu_accel").data.copy()
            c = R.T @ self.model.opt.magnetic
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        v = {"id": 1, "gyro-x": g[0], "gyro-y": g[1], "gyro-z": g[2], "accel-x": a[0], "accel-y": a[1], "accel-z": a[2],
             "compass-x": c[0], "compass-y": c[1], "compass-z": c[2], "euler-x": roll, "euler-y": pitch, "euler-z": yaw}
        return {"code": 0, "data": {"gyro": [{k: (x if k == "id" else float(x)) for k, x in v.items()}]}, "msg": "success"}

    def is_running(self):
        return self._viewer.is_running() if self._viewer is not None else True

    def close(self):
        self._running = False
        if self.realtime:
            self._thread.join(timeout=1)
        if self._viewer is not None:
            self._viewer.close()


class RealRobot(Robot):
    """Real Yanshee over its REST API (port 9090), with a timeout on every request. ip may be 'IP' or 'IP:PORT'."""

    def __init__(self, ip, yanapi_dir=None, min_battery=30, timeout=4.0):
        import socket
        self.rest = YansheeRest(ip, timeout=timeout)
        self.ip, self.host, self.port = ip, self.rest.host, self.rest.port
        self._yanapi_dir = yanapi_dir
        self._api = None
        addr = f"{self.host}:{self.port}"
        try:
            socket.create_connection((self.host, self.port), timeout=min(3.0, timeout)).close()
        except OSError as e:
            raise RobotUnreachable(f"cannot reach {addr} ({e}); check the robot is on and on the same Wi-Fi") from None
        info = self.battery_info()
        battery = info["percent"]
        if battery < min_battery:
            raise RuntimeError(f"battery {battery}% is below {min_battery}%: charge before moving the robot")
        self.battery = battery

    def battery_info(self):
        """{"percent", "charging", "voltage"} read now. Raises with the robot's own code/msg on failure."""
        addr = f"{self.host}:{self.port}"
        try:
            res = self.rest.battery()
        except RobotUnreachable as e:
            raise RobotUnreachable(f"robot at {addr} did not answer ({e}); check the IP and Wi-Fi "
                                   f"(curl http://{addr}/v1/devices/battery)") from None
        except RobotProtocolError as e:
            raise RobotProtocolError(f"unexpected reply from {addr}; is this a Yanshee? ({e})") from None
        if res.get("code") != 0:
            raise ConnectionError(f"robot at {addr} answered the battery query with error code "
                                  f"{res.get('code')}: {res.get('msg', '')}")
        data = res.get("data") if isinstance(res.get("data"), dict) else {}
        if not _is_number(data.get("percent")):
            raise RobotProtocolError(f"battery reply from {addr} has no percent: {res}")
        return {"percent": int(data["percent"]), "charging": data.get("charging"), "voltage": data.get("voltage")}

    @property
    def api(self):
        """The official YanAPI module aimed at this robot (advanced use only).

        YanAPI keeps one global address and has no timeouts: every access re-aims it at this robot, so do
        not interleave .api calls for two robots from different threads."""
        with _YANAPI_LOCK:
            if self._api is None:
                import importlib
                import sys
                d = self._yanapi_dir or os.environ.get("YANAPI_DIR") or os.path.join(
                    os.path.dirname(ROOT), "yanshee robot", "yanshee", "yanshee_examples", "yanshee_examples")
                if d not in sys.path:
                    sys.path.insert(0, d)
                try:
                    self._api = importlib.import_module("YanAPI")
                except ImportError as e:
                    raise ImportError(f"cannot import YanAPI from {d} ({e}); pass yanapi_dir= or set YANAPI_DIR, "
                                      "and install requests, nest_asyncio and opencv-python") from None
            self._api.yan_api_init(self.host)
            self._api.basic_url = f"http://{self.host}:{self.port}/v1/"
            return self._api

    def _send(self, angles, runtime):
        res = self.rest.set_angles({n: int(round(d)) for n, d in angles.items()}, runtime)
        if res.get("code") != 0:
            raise RobotCommandError("servo command", res)
        return res

    def _read(self, names):
        res = self.rest.get_angles(names)
        if res.get("code") != 0:
            raise RobotCommandError("servo read", res)
        data = res.get("data") if isinstance(res.get("data"), dict) else {}
        missing = [n for n in names if not _is_number(data.get(n))]
        if missing:
            raise RobotProtocolError(f"servo read is missing {missing}: {res}")
        return {n: data[n] for n in names}

    def set_servos_mode(self, mode, names):
        """'program' = limp, readable by hand (PRP); 'work' = holding position."""
        res = self.rest.set_servo_mode(mode, _check_mode(mode, names))
        if res.get("code") != 0:
            raise RobotCommandError(f"servo mode {mode}", res)
        return res

    def get_servos_mode(self, names=None):
        return self.rest.get_servo_mode(_check_names(NAMES if names is None else names))

    def play_builtin(self, name, direction="", speed="normal", repeat=1, version="v1", allow_system=False):
        """Start a built-in motion ('wave', 'WakaWaka', ...). Returns the reply with data.total_time (ms).
        System motions (Shutdown, calibration, ActionAging...) raise ValueError unless allow_system=True."""
        if is_system_motion(name) and not allow_system:
            raise ValueError(f"'{name}' is a robot system motion (power, calibration, burn-in); not started "
                             "(pass allow_system=True if you really mean it)")
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")
        if not isinstance(repeat, int) or repeat < 1:
            raise ValueError("repeat must be a positive int")
        res = self.rest.play_motion(name, direction, speed, repeat, version)
        if res.get("code") != 0:
            raise RobotCommandError(f"motion '{name}'", res)
        return res

    def stop(self):
        """Stop any built-in motion (the chest button and head emergency stop still work too)."""
        return self.rest.motion_control("stop")

    def stop_all(self):
        """Send every stop at once (motion v1 and v2, gait, music, TTS). Never raises; returns per-call results."""
        calls = {"motion_v1": lambda: self.rest.motion_control("stop", version="v1"),
                 "motion_v2": lambda: self.rest.motion_control("stop", version="v2"),
                 "gait": self.rest.gait_exit, "music": self.rest.music_stop, "tts": self.rest.tts_stop}
        results = {}

        def run(key, fn):
            try:
                results[key] = fn()
            except RobotError as e:
                results[key] = {"code": -1, "msg": str(e)}

        threads = [threading.Thread(target=run, args=kv, daemon=True) for kv in calls.items()]
        for t in threads:
            t.start()
        deadline = time.monotonic() + self.rest.timeout + 1
        for t in threads:
            t.join(max(0.0, deadline - time.monotonic()))
        for key in calls:
            results.setdefault(key, {"code": -1, "msg": "no reply in time"})
        failed = [k for k, r in results.items() if r.get("code") != 0]
        return {"code": 0 if not failed else -1, "data": results,
                "msg": "success" if not failed else "failed: " + ", ".join(failed)}


def connect(target="sim", ip=None, **kw):
    """connect("sim", viewer=True) or connect("real", ip="192.168.x.y")."""
    if target == "sim":
        return SimRobot(**kw)
    if target == "real":
        if not ip:
            raise ValueError("connect('real') needs ip=")
        return RealRobot(ip, **kw)
    raise ValueError("target must be 'sim' or 'real'")


__all__ = ["connect", "SimRobot", "RealRobot", "is_system_motion", "SYSTEM_MOTION_PREFIXES", "ServoCommandError",
           "RobotCommandError", "RobotUnreachable", "RobotProtocolError", "IDS"]
