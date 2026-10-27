"""Simulator extras for the relay controller: camera renderer, LED markers, sim built-ins, gait, battery, vision.

    from yanshee_sim.robot import SimRobot
    from yanshee_sim import sim_extras
    sim = SimRobot(viewer=False, spec_hook=sim_extras.add_leds)
    cams = sim_extras.SimRenderer(sim)
    jpeg = cams.latest("front")                 # JPEG bytes (waits up to 2 s for the first frame)
    sim_extras.load_builtin("wave", "right")     # motions/builtin/wave_right.json

Nothing here talks to a real robot. Rendering uses EGL when there is no display.
"""
import atexit
import json
import math
import os
import threading
import time
import weakref

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")  # must happen before mujoco is first imported

from .robot import ROOT
from .servos import LIMITS, RESET_POSE

BUILTIN_DIR = os.path.join(ROOT, "motions", "builtin")

# -- LEDs ----------------------------------------------------------------------
LED_GEOMS = {"button": ("led_chest",), "camera": ("led_eye_left", "led_eye_right")}
LED_RGB = {"white": (1.0, 1.0, 1.0), "red": (1.0, 0.05, 0.05), "green": (0.1, 1.0, 0.2), "blue": (0.1, 0.35, 1.0),
           "yellow": (1.0, 0.85, 0.05), "purple": (0.65, 0.1, 1.0), "cyan": (0.05, 0.95, 1.0)}
_EYE_GROUP = 1  # the eye LEDs sit in front of the head camera: hidden in its view


def add_leds(spec):
    """MjSpec hook: chest (button) and eye (camera) LED discs, visual only (no mass, no contacts)."""
    import mujoco
    disc = [0.7071068, 0.7071068, 0.0, 0.0]  # cylinder axis along y: the disc faces forward

    def led(body, name, pos, radius, group):
        g = body.add_geom()
        g.name, g.type, g.pos, g.quat = name, mujoco.mjtGeom.mjGEOM_CYLINDER, pos, disc
        g.size = [radius, 0.0012, 0.0]
        g.rgba = [0.15, 0.15, 0.15, 1.0]
        g.contype = g.conaffinity = 0
        g.group, g.mass = group, 0.0

    base, head = spec.body("base_link"), spec.body("NeckLR")
    led(base, "led_chest", [0.0, 0.0586, 0.2913], 0.009, 2)  # the chest button ring
    for name, x in (("led_eye_left", -0.0092), ("led_eye_right", 0.0098)):
        led(head, name, [x, 0.0252, 0.0403], 0.0045, _EYE_GROUP)


def led_rgba(state, t):
    """(r, g, b, a) and emission for one LED {"color", "mode"} at time t (on / off / blink 1 Hz / breath 3 s)."""
    r, g, b = LED_RGB.get(state.get("color"), LED_RGB["white"])
    mode = state.get("mode", "on")
    level = {"off": 0.0, "blink": 1.0 if (t % 1.0) < 0.5 else 0.0,
             "breath": 0.5 - 0.5 * math.cos(2 * math.pi * t / 3.0)}.get(mode, 1.0)
    dark = 0.12
    return (dark + (r - dark) * level, dark + (g - dark) * level, dark + (b - dark) * level, 1.0), 0.8 * level


# -- camera renderer -------------------------------------------------------------
CAMERAS = ("head_camera", "front", "side", "free")
CAMERA_ALIASES = {"head": "head_camera", "fpv": "head_camera"}


def camera_name(cam):
    """'head' -> 'head_camera'; raises ValueError for unknown cameras."""
    cam = CAMERA_ALIASES.get(cam, cam)
    if cam not in CAMERAS:
        raise ValueError(f"unknown camera {cam!r}; use one of {CAMERAS} or 'head'")
    return cam


def encode_jpeg(rgb, quality=80):
    """JPEG bytes from an HxWx3 uint8 RGB array (OpenCV, or Pillow as a fallback)."""
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    import io
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(rgb).save(out, "JPEG", quality=quality)
    return out.getvalue()


class SimRenderer:
    """One MuJoCo Renderer in its own thread, keeping the latest JPEG of each camera.

    Cameras: head_camera (the robot's FPV), front, side and free (a slow orbit); the last three track the
    robot. Only cameras asked for in the last `idle_s` seconds are rendered, at about `fps`. The sim's
    MjData is copied under its lock (mj_copyData) and rendered outside it, so physics never waits for the GPU.
    """

    FIRST_FRAME_S = 10.0

    def __init__(self, sim, width=640, height=480, fps=12, idle_s=5.0, quality=80):
        import mujoco
        self._mj, self.sim = mujoco, sim
        self.width, self.height, self.fps, self.idle_s, self.quality = width, height, fps, idle_s, quality
        self.error = None
        self._wanted = {}  # cam -> last time it was asked for
        self._frames = {}  # cam -> (jpeg bytes, rgb array, monotonic time, frame number)
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="SimRenderer", daemon=True)
        self._thread.start()

    # -- public ---------------------------------------------------------------
    def latest(self, cam="front", timeout=2.0, max_age=None):
        """JPEG bytes of the newest frame of cam (waits up to timeout s for a first or fresh-enough frame).
        Returns None if nothing could be rendered (see .error)."""
        f = self._get(cam, timeout, max_age)
        return f[0] if f else None

    def latest_rgb(self, cam="head_camera", timeout=2.0, max_age=None):
        """The newest frame as an HxWx3 uint8 RGB array (or None)."""
        f = self._get(cam, timeout, max_age)
        return f[1] if f else None

    def frame_number(self, cam):
        f = self._frames.get(camera_name(cam))
        return f[3] if f else 0

    def wait_frame(self, cam, after, timeout=2.0):
        """Block until cam has a frame newer than frame number `after`; returns (jpeg, number) or (None, after)."""
        cam = camera_name(cam)
        deadline = time.monotonic() + timeout
        with self._cond:
            self._wanted[cam] = time.monotonic()
            self._cond.notify_all()
            while self._running and self.error is None:
                f = self._frames.get(cam)
                if f and f[3] > after:
                    return f[0], f[3]
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._cond.wait(left)
        return None, after

    def close(self):
        if not self._running:
            return
        self._running = False
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=3)

    # -- internals --------------------------------------------------------------
    def _get(self, cam, timeout, max_age):
        cam = camera_name(cam)
        if not self._frames:  # the very first frame can take seconds (GPU / EGL warm-up)
            timeout = max(timeout, self.FIRST_FRAME_S)
        deadline = time.monotonic() + timeout
        with self._cond:
            self._wanted[cam] = time.monotonic()
            self._cond.notify_all()
            while True:
                f = self._frames.get(cam)
                if f and (max_age is None or time.monotonic() - f[2] <= max_age):
                    return f
                left = deadline - time.monotonic()
                if left <= 0 or not self._running or self.error is not None:
                    return f
                self._cond.wait(left)

    def _cameras(self):
        mj, m = self._mj, self.sim.model
        cams, base = {}, m.body("base_link").id
        for name, az, el, dist in (("front", -90.0, -10.0, 0.7), ("side", 180.0, -10.0, 0.7),
                                   ("free", -60.0, -20.0, 0.85)):
            c = mj.MjvCamera()
            c.type, c.trackbodyid = mj.mjtCamera.mjCAMERA_TRACKING, base
            c.azimuth, c.elevation, c.distance = az, el, dist
            cams[name] = c
        c = mj.MjvCamera()
        c.type, c.fixedcamid = mj.mjtCamera.mjCAMERA_FIXED, m.camera("head_camera").id
        cams["head_camera"] = c
        return cams

    def _led_geoms(self):
        m, out = self.sim.model, {}
        for kind, names in LED_GEOMS.items():
            for n in names:
                try:
                    out[m.geom(n).id] = kind
                except KeyError:
                    pass
        return out

    def _run(self):
        mj = self._mj
        try:
            m = self.sim.model
            renderer = mj.Renderer(m, self.height, self.width)
            # Registered after the GL backend's own atexit (eglTerminate), so this one runs first.
            ref = weakref.ref(self)
            atexit.register(lambda: ref() is not None and ref().close())
        except Exception as e:  # no GPU/EGL: report, the rest of the sim keeps working
            self.error = f"renderer unavailable: {e}"
            with self._cond:
                self._cond.notify_all()
            return
        try:
            data, cams, leds = mj.MjData(m), self._cameras(), self._led_geoms()
            opt_head, opt = mj.MjvOption(), mj.MjvOption()
            opt_head.geomgroup[_EYE_GROUP] = 0
            t0, n = time.monotonic(), 0
            while self._running:
                tick = time.monotonic()
                with self._cond:
                    wanted = [c for c, t in self._wanted.items() if tick - t < self.idle_s]
                    if not wanted:
                        self._cond.wait(0.2)
                        continue
                with self.sim._lock:
                    mj.mj_copyData(data, m, self.sim.data)
                led_state = {k: dict(v) for k, v in self.sim.led.items()}
                cams["free"].azimuth = -60.0 + 18.0 * (tick - t0)  # one turn every 20 s
                n += 1
                for cam in wanted:
                    renderer.update_scene(data, cams[cam], opt_head if cam == "head_camera" else opt)
                    self._paint_leds(renderer.scene, leds, led_state, tick)
                    rgb = renderer.render().copy()
                    jpeg = encode_jpeg(rgb, self.quality)
                    with self._cond:
                        self._frames[cam] = (jpeg, rgb, time.monotonic(), n)
                        self._cond.notify_all()
                time.sleep(max(0.0, 1.0 / self.fps - (time.monotonic() - tick)))
        except Exception as e:
            self.error = f"renderer stopped: {e}"
        finally:
            with self._cond:
                self._cond.notify_all()
            try:
                renderer.close()
            except Exception:
                pass

    def _paint_leds(self, scene, leds, led_state, t):
        if not leds:
            return
        geom_type = self._mj.mjtObj.mjOBJ_GEOM
        for i in range(scene.ngeom):
            g = scene.geoms[i]
            if g.objtype == geom_type and g.objid in leds:
                rgba, emission = led_rgba(led_state.get(leds[g.objid], {}), t)
                g.rgba[:] = rgba
                g.emission = emission


# -- sim built-ins ---------------------------------------------------------------
SIM_BUILTINS = {"raise": ("left", "right", "both"), "wave": ("left", "right", "both"),
                "stretch": ("left", "right", "both"), "come on": ("left", "right", "both"),
                "bend": ("left", "right"), "head": ("forward", "left", "right"),
                "bow": ("",), "crouch": ("",), "reset": ("",)}


def builtin_path(name, direction=""):
    """motions/builtin/NAME[_DIRECTION].json (spaces as '_'), whether it exists or not."""
    stem = name.strip().lower().replace(" ", "_") + (f"_{direction.strip().lower()}" if direction else "")
    return os.path.join(BUILTIN_DIR, stem + ".json")


def has_builtin(name, direction=""):
    return os.path.isfile(builtin_path(name, direction))


def load_builtin(name, direction=""):
    """The sim version of a parametric built-in as a motion dict, or None if there is none."""
    path = builtin_path(name, direction)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        mo = json.load(f)
    mo.setdefault("name", os.path.basename(path)[:-5])
    mo.setdefault("end_in_reset", True)
    return mo


# -- sim gait ---------------------------------------------------------------------
class SimGait:
    """Stepping gait for the free-standing sim, driven like control_motion_gait.

    One cycle is two steps: the torso sways over one foot (hip roll + ankle roll), the other leg lifts
    (hip/knee/ankle pitch, foot kept flat), swings and is put down. speed_v (-5..5, + forward) sets the
    stride, speed_h (-5..5, + to the robot's left) a side step, period (1..5) the step time, steps the
    number of steps (0 = until stopped), wave the arm swing. Balance is an ankle strategy on the torso lean
    (SimRobot.torso_lean, the exact model-frame angles): the ankles lean the torso back towards the lean it
    had in the starting crouch, in proportion to the angles (K_PITCH, K_ROLL) and rates (D_PITCH, D_ROLL).
    A tilt watchdog stops the walk and stands up if the torso tilts more than max_tilt_deg. Targets are slew-limited to MAX_DEG_S. This is
    not the robot's own gait (UBTECH's is closed source); it only makes the sim walk plausibly.

    The class attributes are the tuning picked by tools/tune_gait.py (fastest walk that stays up forward,
    backward, sideways and at every period); pass any of them in lower case to try others, e.g.
    SimGait(sim, sway=10, cycle=1.0).
    """

    # tools/tune_gait.py winner (tools/gait_tuning.json): 19.4 cm/s at speed_v 5 in the sim with the servo slew
    # held to 300 deg/s, every scenario under 12.5 deg of tilt. Rate damping (D_ROLL) is what keeps it up;
    # a roll gain (K_ROLL) fights the sway. UBTECH's own gait course uses a 0.8 s sway period.
    CROUCH = 10.0  # deg: knees bent while walking
    SWAY = 7.0  # deg of hip/ankle roll that moves the weight over the stance foot
    LIFT = 18.0  # deg of hip pitch used to lift the swing foot (knee twice that)
    STRIDE = 6.5  # deg of hip pitch per unit of speed_v
    SIDE = 1.0  # deg of hip/ankle roll per unit of speed_h
    BACK = 0.5  # backward strides are this fraction of forward ones (less stable)
    DIAG = 0.5  # side amplitude is scaled by this while also walking forward or backward
    CYCLE = 0.7  # s per two steps at period 0; a cycle lasts CYCLE + CYCLE_PER_PERIOD * period
    CYCLE_PER_PERIOD = 0.05
    K_PITCH = 0.3  # ankle pitch deg per deg of torso lean about x (+ = leaning back)
    K_ROLL = 0.0  # ankle roll deg per deg of torso lean about y (+ = leaning right)
    D_PITCH = 0.03  # ankle pitch deg per deg/s of lean rate about x
    D_ROLL = 0.06  # ankle roll deg per deg/s of lean rate about y
    MAX_DEG_S = 300.0  # servo slew limit on the targets (the servo family is rated about 0.2 s per 60 deg)
    STOP_S = 1.0  # s over which the stride and sway fade out when the walk stops (0.4 s tips the robot at full speed)
    DT = 0.02
    TUNABLE = ("crouch", "sway", "lift", "stride", "side", "back", "diag", "cycle", "cycle_per_period",
               "k_pitch", "k_roll", "d_pitch", "d_roll", "max_deg_s", "stop_s")

    def __init__(self, sim, max_tilt_deg=25.0, **tuning):
        self.sim, self.max_tilt = sim, max_tilt_deg
        for k, v in tuning.items():
            if k not in self.TUNABLE:
                raise TypeError(f"SimGait: unknown tuning parameter '{k}' (one of {self.TUNABLE})")
            setattr(self, k.upper(), float(v))
        self.steps_done = 0
        self.status = "idle"

    @classmethod
    def cycle_time(cls, period):
        """Seconds per cycle (two steps) at gait period 1..5, with the class defaults."""
        return cls.CYCLE + cls.CYCLE_PER_PERIOD * min(max(period, 1), 5)

    def _cycle(self, period):
        return self.CYCLE + self.CYCLE_PER_PERIOD * min(max(period, 1), 5)

    def pose(self, phi, stride=0.0, side=0.0, amp=1.0, wave=False, crouch=None):
        """Leg (and arm) servo angles at cycle phase phi (0..1): left leg swings in [0, 0.5), right in [0.5, 1)."""
        crouch = self.CROUCH if crouch is None else crouch
        r = self.SWAY * amp * math.sin(2 * math.pi * phi)  # > 0: weight on the right foot

        def shaped(u):
            return min(max((u - 0.15) / 0.7, 0.0), 1.0)

        out = {}
        for leg, offset in (("Left", 0.0), ("Right", 0.5)):
            u = (phi - offset) % 1.0
            if u < 0.5:  # swing: lift and move from back to front
                v = shaped(u / 0.5)
                lift, s = self.LIFT * amp * math.sin(math.pi * v), 0.5 - 0.5 * math.cos(math.pi * v)
            else:  # stance: the foot slides back under the body
                lift, s = 0.0, 1.0 - (u - 0.5) / 0.5
            fwd, lat = stride * amp * (2 * s - 1), side * amp * (2 * s - 1)
            hip, knee = crouch + lift + fwd, 2 * crouch + 2 * lift
            ankle = knee - hip  # keeps the foot parallel to the torso
            if leg == "Right":
                out.update(RightHipFB=60 - hip, RightKneeFlex=76 - knee, RightAnkleFB=110 + ankle,
                           RightHipLR=90 + r + lat, RightAnkleUD=90 - r - lat)
            else:
                out.update(LeftHipFB=120 + hip, LeftKneeFlex=104 + knee, LeftAnkleFB=70 - ankle,
                           LeftHipLR=90 + r + lat, LeftAnkleUD=90 - r - lat)
        if wave:
            swing = 15.0 * amp * math.sin(2 * math.pi * phi)
            out.update(RightShoulderRoll=90 - swing, LeftShoulderRoll=90 - swing)
        return out

    @staticmethod
    def describe(speed_v, speed_h):
        parts = []
        if speed_v:
            parts.append("forward" if speed_v > 0 else "backward")
        if speed_h:
            parts.append("left" if speed_h > 0 else "right")
        return "walking " + " and ".join(parts) if parts else "stepping in place"

    def _lean(self):
        """(lean about x, lean about y, rates) of the torso in deg and deg/s, model frame (SimRobot.torso_lean)."""
        return self.sim.torso_lean()

    def _balance(self, target, ref, amp):
        """Ankle strategy: lean the torso back towards ref = (lean x, lean y) measured in the starting crouch.
        Measured in the sim: +d on RightAnkleFB with -d on LeftAnkleFB pitches the torso forward (lean about x
        down); +r on both AnkleUD leans it to the left (lean about y down)."""
        if not (self.K_PITCH or self.K_ROLL or self.D_PITCH or self.D_ROLL):
            return
        ex, ey, wx, wy = self._lean()
        pitch = amp * (self.K_PITCH * (ex - ref[0]) + self.D_PITCH * wx)
        roll = amp * (self.K_ROLL * (ey - ref[1]) + self.D_ROLL * wy)
        target["RightAnkleFB"] += pitch
        target["LeftAnkleFB"] -= pitch
        target["RightAnkleUD"] += roll
        target["LeftAnkleUD"] += roll

    def _slew(self, target, last, dt):
        lim = self.MAX_DEG_S * dt
        out = {}
        for n, deg in target.items():
            deg = min(max(deg, LIMITS[n][0]), LIMITS[n][1])
            out[n] = deg if last is None or n not in last else last[n] + min(max(deg - last[n], -lim), lim)
        return out

    def _send(self, angles, ms):
        self.sim._send(angles, ms)  # exact piecewise-linear targets (set_servos_angles clamps to >= 200 ms)

    def run(self, params, stop_event, on_step=None):
        """Walk until params["steps"] steps are done (0 = until stop_event). params may change while walking
        (speed_v, speed_h, period, wave). Returns a status text."""
        sim, dt = self.sim, self.DT
        self.steps_done, self.status = 0, "walking"
        sim.set_servos_angles(self._slew(self.pose(0.0, amp=0.0), None, dt), 600)
        sim.wait(0.7)
        ref = self._lean()[:2]
        last = None
        phi, amp, t_prev = 0.0, 0.0, sim.data.time
        stopping, status = False, "done"
        while True:
            p = dict(params)
            steps = int(p.get("steps") or 0)
            if stop_event.is_set() and not stopping:
                stopping, status = True, "stopped"
            if steps and self.steps_done >= steps and not stopping:
                stopping, status = True, "done"
            if sim.torso_tilt_deg() > self.max_tilt:
                status = f"stopped: torso tilted over {self.max_tilt:g} deg"
                break
            T = self._cycle(p.get("period", 1))
            v = min(max(p.get("speed_v", 0), -5), 5)
            h = min(max(p.get("speed_h", 0), -5), 5)
            stride = self.STRIDE * (v if v >= 0 else v * self.BACK)
            side = self.SIDE * h * (self.DIAG if v else 1.0)
            t = sim.data.time
            dphi = max(0.0, t - t_prev) / T
            t_prev = t
            amp = max(0.0, amp - dt / self.STOP_S) if stopping else min(1.0, amp + dt / (T / 2))
            if stopping and amp <= 0.0:
                break
            old, phi = phi, (phi + dphi) % 1.0
            if (old < 0.5 <= phi) or phi < old:
                self.steps_done += 1
                if on_step:
                    on_step(self.steps_done)
            target = self.pose(phi + dt / T, stride, side, amp, bool(p.get("wave")))
            self._balance(target, ref, amp)
            last = self._slew(target, last, dt)
            self._send(last, dt * 1000)
            sim.wait(dt)
        sim.set_servos_angles({n: RESET_POSE[n] for n in self.pose(0.0, wave=True)}, 600)
        sim.wait(0.6)
        self.status = status
        return status


# -- sim battery ---------------------------------------------------------------------
class SimBattery:
    """Simulated battery: starts at 100 % and drains with time and actuator effort; charge() refills it.

    Roughly 1.5 h of light use from 100 % to the robot's 30 % motion cut-off."""

    IDLE_PER_S = 0.008  # %/s
    EFFORT_PER_NM_S = 0.004  # %/s per N*m of summed |actuator torque|

    def __init__(self, percent=100.0):
        self.percent = float(percent)
        self._lock = threading.Lock()

    def update(self, sim, dt):
        with sim._lock:
            effort = float(abs(sim.data.actuator_force).sum())
        with self._lock:
            self.percent = max(0.0, self.percent - dt * (self.IDLE_PER_S + self.EFFORT_PER_NM_S * effort))
        return self.percent

    def charge(self):
        with self._lock:
            self.percent = 100.0
        return self.percent


# -- sim vision --------------------------------------------------------------------
_HUES = (("red", 0, 10), ("orange", 10, 22), ("yellow", 22, 35), ("green", 35, 85), ("cyan", 85, 100),
         ("blue", 100, 130), ("purple", 130, 160), ("red", 160, 181))


def detect_colours(rgb, min_fraction=0.02):
    """Saturated colour regions in an RGB image, largest first, in the robot's color_detect shape:
    {"color": [{"name", "fraction", "x", "y"}]} (x, y = centroid in pixels)."""
    import numpy as np
    try:
        import cv2
        hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)  # H 0-180, S/V 0-255
        h, s, v = hsv[:, :, 0].astype(int), hsv[:, :, 1], hsv[:, :, 2]
    except ImportError:
        import colorsys  # slow fallback on a subsampled image
        small = rgb[::4, ::4] / 255.0
        hsv = np.array([[colorsys.rgb_to_hsv(*px) for px in row] for row in small])
        h, s, v = (hsv[:, :, 0] * 180).astype(int), hsv[:, :, 1] * 255, hsv[:, :, 2] * 255
    mask = (s > 110) & (v > 70)
    total = mask.size
    found = {}
    ys, xs = np.mgrid[0:mask.shape[0], 0:mask.shape[1]]
    for name, lo, hi in _HUES:
        m = mask & (h >= lo) & (h < hi)
        k = int(m.sum())
        if k:
            a = found.setdefault(name, [0, 0.0, 0.0])
            a[0] += k
            a[1] += float(xs[m].sum())
            a[2] += float(ys[m].sum())
    scale = rgb.shape[1] / mask.shape[1]
    out = [{"name": n, "fraction": round(k / total, 3), "x": int(sx / k * scale), "y": int(sy / k * scale)}
           for n, (k, sx, sy) in found.items() if k / total >= min_fraction]
    return {"color": sorted(out, key=lambda c: -c["fraction"])}


__all__ = ["SimRenderer", "SimGait", "SimBattery", "add_leds", "led_rgba", "detect_colours", "encode_jpeg",
           "load_builtin", "builtin_path", "has_builtin", "camera_name", "CAMERAS", "SIM_BUILTINS", "BUILTIN_DIR"]
