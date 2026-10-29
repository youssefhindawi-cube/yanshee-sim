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
SCENE_MAT = os.path.join(ROOT, "model", "scene_mat.xml")  # the soft play mat: grippier floor (build_models.MAT_FRICTION)
SERVO_MODES = ("work", "program")
LED_COLORS = {"button": ("white", "red", "green", "blue", "yellow", "purple", "cyan"), "camera": ("red", "green", "blue")}
LED_MODES = {"button": ("on", "off", "blink", "breath"), "camera": ("on", "off", "blink")}
# LED geoms added to the model by sim_extras.add_leds (chest button ring, two eye discs), their colours, and
# the one function that turns an LED setting into a colour: the MuJoCo window and the web GUI's camera streams
# both render from the model's geom_rgba / mat_emission, which SimRobot repaints as the simulation runs.
LED_GEOMS = {"button": ("led_chest",), "camera": ("led_eye_left", "led_eye_right")}
LED_RGB = {"white": (1.0, 1.0, 1.0), "red": (1.0, 0.05, 0.05), "green": (0.1, 1.0, 0.2), "blue": (0.1, 0.35, 1.0),
           "yellow": (1.0, 0.85, 0.05), "purple": (0.65, 0.1, 1.0), "cyan": (0.05, 0.95, 1.0)}
LED_REPAINT_S = 0.02  # sim seconds between repaints (blink and breath animate at 50 Hz)


def led_rgba(state, t):
    """(r, g, b, a) and emission for one LED {"color", "mode"} at time t (on / off / blink 1 Hz / breath 3 s)."""
    r, g, b = LED_RGB.get(state.get("color"), LED_RGB["white"])
    mode = state.get("mode", "on")
    level = {"off": 0.0, "blink": 1.0 if (t % 1.0) < 0.5 else 0.0,
             "breath": 0.5 - 0.5 * math.cos(2 * math.pi * t / 3.0)}.get(mode, 1.0)
    dark = 0.12
    return (dark + (r - dark) * level, dark + (g - dark) * level, dark + (b - dark) * level, 1.0), 0.8 * level


def led_ids(model):
    """{kind: [(geom id, material id), ...]} for the LED geoms present in model ({} when add_leds was not used)."""
    out = {}
    for kind, names in LED_GEOMS.items():
        for n in names:
            try:
                gid = model.geom(n).id
            except KeyError:
                continue
            out.setdefault(kind, []).append((gid, int(model.geom_matid[gid])))
    return out


def paint_led_model(model, ids, led_state, t):
    """Write the LED colours for led_state ({kind: {"color", "mode"}}) at time t into model.geom_rgba and
    mat_emission, so every renderer of this model (the viewer window, offscreen cameras) shows the same LEDs."""
    for kind, entries in ids.items():
        rgba, emission = led_rgba(led_state.get(kind, {}), t)
        for gid, mid in entries:
            model.geom_rgba[gid] = rgba
            if mid >= 0:
                model.mat_emission[mid] = emission
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


# -- emulated IMU (what the robot's get_sensors_gyro() reports) -----------------------------------------
# Measured on our robot on 2026-10-29 (tools/robot_checks.py imu and gyro; robot_dump/imu_20261029_120041.json,
# gyro_20261029_121453.json, gyro_20261029_121727.json). The MPU9250 in the chest reports on its chip axes
# x = robot's right, y = up, z = back:
# - euler: aerospace roll/pitch/yaw of that frame. Upright euler-x 91.9 (88.6 before an app recalibration; the
#   sim uses 90), leaning back raises it (+20.6 for 20 deg), face down 0.3, on the back 178.9; euler-y +25.0
#   leaning 25 deg to the right; euler-z is an absolute heading, +89 after a quarter turn left, no drift in 6 min.
# - accel in g: +1 on y upright, +1 on z face down, -1 on z on the back, -0.44 on x leaning right.
# - gyro in deg/s, right-hand rule on the same axes: a turn to the left is + on y, leaning back + on x, leaning
#   right - on z. Bias at rest about (-0.85, -0.8, +0.5) deg/s over six still postures.
# Yanshee-Serenade's driver notes (gyro on left/back/up, accel on right/front/down) were wrong on every axis
# here; the compass still follows them (left/back/up, not measured). Rate 19.05 Hz (rostopic hz
# /hal_gyro_report). The noise densities 0.0164 m/s^2/sqrt(Hz) and 0.0014 rad/s/sqrt(Hz) come from their
# Allan-variance fit. The course lesson on falls (euler-x about 0 face down, +-180 on the back) and UBTECH's
# self-balancing sample (91.95 upright, a backward lean raises it) agree with the measurements.
IMU_RATE_HZ = 19.0
IMU_UPRIGHT_EULER_X = 90.0
IMU_ACCEL_SIGMA_G = 0.0164 * math.sqrt(IMU_RATE_HZ) / 9.80665
IMU_GYRO_SIGMA_DEG_S = math.degrees(0.0014 * math.sqrt(IMU_RATE_HZ))
IMU_GYRO_BIAS_DEG_S = (-0.85, -0.8, 0.5)  # measured at rest on our robot, chip axes


class SimRobot(Robot):
    """MuJoCo backend. Physics runs in a background thread at real time (or on demand with realtime=False)."""

    def __init__(self, viewer=True, realtime=True, fixed_base=False, manual=False, scene=SCENE, spec_hook=None,
                 key_callback=None):
        """manual=True: servo targets come from the viewer's Control sliders instead of set_servos_angles.
        spec_hook(spec): edits the MjSpec before compiling (e.g. sim_extras.add_leds adds LED markers).
        key_callback(keycode): called for key presses (and repeats while held) in the viewer window; also
        settable later as .on_key. GLFW key codes: W 87, A 65, S 83, D 68, Q 81, E 69, X 88, Esc 256, arrows 262-265."""
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
        self._led_ids, self._led_t = led_ids(m), -1.0
        self._support_ids = []  # the box soles (build_models adds them); else the ankle-roll bodies
        for n in ("RightAnkleUD_sole", "LeftAnkleUD_sole"):
            try:
                self._support_ids.append(("geom", m.geom(n).id))
            except KeyError:
                self._support_ids.append(("body", m.body(n[:-5]).id))
        self._qadr = {n: m.jnt_qposadr[m.joint(n).id] for n in NAMES}
        self._kp = {n: m.actuator_gainprm[i, 0] for n, i in self._act.items()}
        self._bias = {n: m.actuator_biasprm[i, 1:3].copy() for n, i in self._act.items()}
        self._modes = {n: "work" for n in NAMES}
        # per actuator: (start rad, end rad, t_start, t_end)
        self._moves = {n: (self.data.ctrl[i], self.data.ctrl[i], 0.0, 0.0) for n, i in self._act.items()}
        self._last_time = self.data.time
        self._lock = threading.RLock()
        self._imu_hold, self._imu_rng, self.imu_noise = None, None, True
        self.mirroring, self._mirror = False, None  # mirror mode: the pose comes from the real robot (set_mirror)
        self._cmd_t = -1e9  # sim time of the last servo command of its own (mirroring waits MIRROR_HOLDOFF_S after it)
        self.led = {"button": {"color": "blue", "mode": "breath"}, "camera": {"color": "green", "mode": "on"}}
        self.paint_leds(force=True)
        self.manual = manual
        self.on_key = key_callback
        self._viewer = None
        self.realtime = realtime
        if viewer:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(m, self.data, key_callback=self._viewer_key, show_left_ui=False)
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
        self._imu_hold = None  # the clock restarts: a held sample would otherwise be served again
        for n in NAMES:
            self._set_gain(n, "work")
        self._mj.mj_forward(self.model, self.data)
        self._hold()
        self._last_time = self.data.time

    def _step(self):
        if self.mirroring and self._mirror is not None:
            self._mirror_step()
            return
        t = self.data.time
        if t < self._last_time:  # the viewer reset the data (Backspace): stale move times would replay old moves
            self._restart()
            t = self.data.time
        for n, (a, b, t0, t1) in ({} if self.manual else self._moves).items():
            s = 1.0 if t >= t1 else max(0.0, (t - t0) / (t1 - t0))
            self.data.ctrl[self._act[n]] = a + (b - a) * s  # linear profile, like the servo's timed move
        self._mj.mj_step(self.model, self.data)
        self._last_time = self.data.time
        self.paint_leds()

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
            if self.mirroring:  # a command of its own: the sim stops showing the robot and moves itself
                self._end_mirror()
            self._cmd_t = t
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

    def paint_leds(self, force=False):
        """Repaint the LED geoms from .led at the current sim time (at most every LED_REPAINT_S of sim time unless
        force). Called from the physics step; call it yourself when stepping the model from outside (view.py)."""
        if not self._led_ids:
            return
        t = self.data.time
        if not force and 0 <= t - self._led_t < LED_REPAINT_S:
            return
        paint_led_model(self.model, self._led_ids, self.led, t)
        self._led_t = t

    def _viewer_key(self, keycode):
        """Key presses in the MuJoCo window (and key repeats while a key is held) go to on_key(keycode)."""
        cb = self.on_key
        if cb is None:
            return
        try:
            cb(int(keycode))
        except Exception as e:  # noqa: BLE001 - a handler error must not take the viewer down
            print(f"viewer key handler failed: {e}")

    def set_led(self, type, color, mode):
        """Set an LED (YanAPI set_robot_led values): the viewer window and the GUI's cameras show it at once."""
        if type not in LED_COLORS or color not in LED_COLORS[type] or mode not in LED_MODES[type]:
            return {"code": 1, "data": {}, "msg": f"bad led setting {type}/{color}/{mode}"}
        with self._lock:
            self.led[type] = {"color": color, "mode": mode}
            self.paint_leds(force=True)
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
            self._imu_hold = None
            for n, deg in angles.items():
                if n in self._act and _is_number(deg):
                    rad = math.radians(deg)
                    self.data.qpos[self._qadr[n]] = rad
                    self.data.ctrl[self._act[n]] = rad
                    self._moves[n] = (rad, rad, 0.0, 0.0)
            self.data.qvel[:] = 0
            self._mj.mj_forward(self.model, self.data)

    # -- mirror mode: show the real robot, physics paused -------------------------------------------------
    MIRROR_HOLDOFF_S = 0.5  # set_mirror is ignored this long after the sim's own last servo command
    MIRROR_UPRIGHT_EULER_X = 92.0  # the real robot's euler-x standing; the sim's emulation uses IMU_UPRIGHT_EULER_X

    def set_mirror(self, angles, euler_x, euler_y, heading=0.0, glide_s=0.25):
        """Show the real robot: every joint at its encoder reading ({servo: deg}), the body tilted as the robot's IMU
        says (euler-x about 92 standing, euler-y + leaning right, heading in deg + turned left) and set down so its
        lowest point touches the floor. Physics is paused while mirroring; the pose glides to the new reading over
        glide_s. stop_mirror(), or any servo command to the sim, resumes the physics from the mirrored pose.
        Returns False (ignored) within MIRROR_HOLDOFF_S of the sim's own last servo command."""
        if not self.mirroring and 0 <= self.data.time - self._cmd_t < self.MIRROR_HOLDOFF_S:
            return False
        mj, m = self._mj, self.model
        import numpy as np
        r = math.radians(euler_x - self.MIRROR_UPRIGHT_EULER_X + 90.0)
        p = math.radians(euler_y)
        up = np.array([-math.sin(p), -math.cos(p) * math.cos(r), math.cos(p) * math.sin(r)])  # world up, site axes
        R_site = self._tilt_to(up)
        y0 = math.atan2(-R_site[0, 1], R_site[1, 1])  # the emulation's heading of this tilt alone
        a = math.radians(heading) - y0
        Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
        R_site = Rz @ R_site
        S = np.zeros(9)
        mj.mju_quat2Mat(S, m.site_quat[m.site("imu").id])
        R_base = R_site @ S.reshape(3, 3).T
        quat = np.zeros(4)
        mj.mju_mat2Quat(quat, R_base.flatten())
        with self._lock:
            q = self.data.qpos.copy()
            for n, deg in angles.items():
                if n in self._qadr and _is_number(deg):
                    q[self._qadr[n]] = math.radians(deg)
            q[3:7] = quat
            q[2] = self._floor_height(q)
            start = self._mirror["now"] if self.mirroring and self._mirror else self.data.qpos.copy()
            t = self.data.time
            if not self.mirroring and 0 <= t - self._cmd_t < self.MIRROR_HOLDOFF_S:  # (a respawn restarts the clock)
                return False
            self._mirror = {"from": start, "to": q, "t0": t, "t1": t + max(glide_s, 1e-3), "now": start.copy()}
            self.mirroring = True
        return True

    @staticmethod
    def _tilt_to(up):
        """The world-from-site rotation that takes the site-frame vector `up` to world up, with the least rotation."""
        import numpy as np
        u = up / (np.linalg.norm(up) or 1.0)
        k = np.cross(u, [0.0, 0.0, 1.0])
        s, c = np.linalg.norm(k), float(u[2])
        if s < 1e-9:
            return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
        k = k / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return np.eye(3) + s * K + (1 - c) * (K @ K)

    def _floor_height(self, q):
        """Base height that puts the lowest point of the robot's collision geoms on the floor (z = 0)."""
        import numpy as np
        mj, m = self._mj, self.model
        d = mj.MjData(m)
        d.qpos[:] = q
        d.qpos[2] = 1.0
        mj.mj_kinematics(m, d)
        low = 1e9
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0 or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            R = d.geom_xmat[g].reshape(3, 3)
            c, half = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            low = min(low, (d.geom_xpos[g] + R @ c)[2] - float(np.abs(R[2]) @ half))
        return 1.0 - low + 0.001

    def _mirror_step(self):
        import numpy as np
        mj, d, mi = self._mj, self.data, self._mirror
        s = min(1.0, max(0.0, (d.time - mi["t0"]) / (mi["t1"] - mi["t0"])))
        q = mi["from"] + (mi["to"] - mi["from"]) * s
        qa, qb = mi["from"][3:7], mi["to"][3:7]
        if float(qa @ qb) < 0:
            qb = -qb
        quat = qa + (qb - qa) * s
        q[3:7] = quat / (np.linalg.norm(quat) or 1.0)
        mi["now"] = q
        d.qpos[:] = q
        d.qvel[:] = 0
        mj.mj_forward(self.model, d)
        d.time += self.model.opt.timestep
        self._last_time = d.time
        self.paint_leds()

    def stop_mirror(self):
        """Back to physics from the mirrored pose, every servo holding the angle it shows."""
        with self._lock:
            self._end_mirror()

    def _end_mirror(self):  # caller holds self._lock
        if not self.mirroring:
            return
        self.mirroring, self._mirror = False, None
        self.data.qvel[:] = 0
        for n in self._act:
            rad = float(self.data.qpos[self._qadr[n]])
            self.data.ctrl[self._act[n]] = rad
            self._moves[n] = (rad, rad, 0.0, 0.0)
        self._mj.mj_forward(self.model, self.data)

    def torso_tilt_deg(self):
        """Angle between the torso's up axis and world up (0 = upright). Useful to detect falls."""
        with self._lock:
            R = self.data.xmat[self.model.body("base_link").id].reshape(3, 3)
            return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))

    SUPPORT_R_M = 0.05  # m: the capture point may wander this far from the support centre before it counts as outside

    def balance(self):
        """Momentum check, model frame: centre of mass, its horizontal velocity, the capture point (where the robot
        would have to step to come to rest: com + v * sqrt(h / g)) and the support centre (midpoint of the soles).
        margin_m > 0 means the capture point lies that far outside a circle of SUPPORT_R_M around the support
        centre: momentum the feet cannot absorb, the robot is about to tip. SimGait brakes on it."""
        import numpy as np
        with self._lock:
            m, d = self.model, self.data
            self._mj.mj_subtreeVel(m, d)
            com, vel = d.subtree_com[0].copy(), d.subtree_linvel[0].copy()
            pts = [d.geom_xpos[i] if kind == "geom" else d.xpos[i] for kind, i in self._support_ids]
            support = np.mean(pts, axis=0)[:2] if pts else com[:2]
        h = max(com[2] - min((p[2] for p in pts), default=0.0), 0.05)
        cp = com[:2] + vel[:2] * math.sqrt(h / 9.81)
        off = cp - support
        return {"com_m": [float(com[0]), float(com[1]), float(com[2])], "vel_m_s": [float(vel[0]), float(vel[1])],
                "capture_point_m": [float(cp[0]), float(cp[1])], "support_m": [float(support[0]), float(support[1])],
                "offset_m": [float(off[0]), float(off[1])], "margin_m": float(math.hypot(off[0], off[1]) - self.SUPPORT_R_M)}

    def torso_lean(self):
        """(lean about x, lean about y, rate about x, rate about y) of the torso in the model frame, in deg and
        deg/s: rotation about the x (right) axis, + = leaning back; about the y (forward) axis, + = leaning
        right. Exact values from the physics, not the emulated IMU; SimGait balances on these."""
        with self._lock:
            R = self.data.site("imu").xmat.reshape(3, 3).copy()
            w = (0.0, 0.0, 0.0) if self.fixed_base else self.data.sensor("imu_gyro").data.copy()
        return (math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0])))),
                math.degrees(w[0]), math.degrees(w[1]))

    def get_sensors_gyro(self):
        """Emulated IMU in YanAPI's shape and, as far as the documents say, the real robot's frame and units
        (see IMU_* above): accel in g and gyro in deg/s on the chip axes right/up/back (measured), compass on
        left/back/up axes (documents),
        euler-x/y/z in deg with euler-x about 0 face down, +-180 on the back and IMU_UPRIGHT_EULER_X upright,
        euler-y the sideways lean (+ = right, assumed) and euler-z the heading (0 = facing model +y, + = turned
        left). Samples are held for 1/IMU_RATE_HZ of sim time and carry noise and gyro bias unless imu_noise is
        False. With fixed_base the torso is welded to the world: accel is gravity seen from the IMU, gyro 0."""
        import numpy as np
        with self._lock:
            t = self.data.time
            if self._imu_hold is not None and 0 <= t - self._imu_hold[0] < 1 / IMU_RATE_HZ:
                return {"code": 0, "data": {"gyro": [dict(self._imu_hold[1])]}, "msg": "success"}
            R = self.data.site("imu").xmat.reshape(3, 3).copy()
            if self.fixed_base:
                w, f = np.zeros(3), R.T @ -self.model.opt.gravity
            else:
                w = self.data.sensor("imu_gyro").data.copy()
                f = self.data.sensor("imu_accel").data.copy()
            mag = R.T @ self.model.opt.magnetic
        up = R.T @ np.array([0.0, 0.0, 1.0])  # world up on the site axes (x right, y forward, z up)
        up_c = np.array([up[0], up[2], -up[1]])  # on the chip axes (x right, y up, z back)
        sign = 1.0 if IMU_UPRIGHT_EULER_X >= 0 else -1.0
        roll = math.degrees(math.atan2(sign * up_c[1], up_c[2]))
        pitch = math.degrees(math.atan2(-up_c[0], math.hypot(up_c[1], up_c[2])))
        yaw = math.degrees(math.atan2(-R[0, 1], R[1, 1]))  # heading of the forward axis, 0 = model +y
        gyro = np.array([w[0], w[2], -w[1]]) * (180 / math.pi)  # chip axes right, up, back (measured)
        accel = np.array([f[0], f[2], -f[1]]) / 9.80665  # chip axes right, up, back, in g (measured)
        comp = np.array([-mag[0], -mag[1], mag[2]])
        if self.imu_noise:
            if self._imu_rng is None:
                self._imu_rng = np.random.default_rng(0)
            gyro += np.array(IMU_GYRO_BIAS_DEG_S) + self._imu_rng.normal(0, IMU_GYRO_SIGMA_DEG_S, 3)
            accel += self._imu_rng.normal(0, IMU_ACCEL_SIGMA_G, 3)
        v = {"id": 1, "gyro-x": gyro[0], "gyro-y": gyro[1], "gyro-z": gyro[2],
             "accel-x": accel[0], "accel-y": accel[1], "accel-z": accel[2],
             "compass-x": comp[0], "compass-y": comp[1], "compass-z": comp[2],
             "euler-x": roll, "euler-y": pitch, "euler-z": yaw}
        entry = {k: (x if k == "id" else float(x)) for k, x in v.items()}
        with self._lock:
            self._imu_hold = (t, entry)
        return {"code": 0, "data": {"gyro": [dict(entry)]}, "msg": "success"}

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
