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

from .robot import LED_GEOMS, LED_RGB, ROOT, led_rgba, paint_led_model  # noqa: F401 - re-exported
from .servos import LIMITS, RESET_POSE

BUILTIN_DIR = os.path.join(ROOT, "motions", "builtin")

# -- LEDs ----------------------------------------------------------------------
_EYE_GROUP = 1  # the eye LEDs sit in front of the head camera: hidden in its view


def add_leds(spec):
    """MjSpec hook: chest (button) and eye (camera) LED discs, visual only (no mass, no contacts). Each disc has
    its own material so SimRobot.paint_leds can set colour and glow at run time (robot.paint_led_model); the
    viewer window and the offscreen cameras then show identical LEDs."""
    import mujoco
    disc = [0.7071068, 0.7071068, 0.0, 0.0]  # cylinder axis along y: the disc faces forward

    def led(body, name, pos, radius, group):
        mat = spec.add_material()
        mat.name, mat.rgba, mat.emission = name + "_mat", [0.15, 0.15, 0.15, 1.0], 0.0
        g = body.add_geom()
        g.name, g.type, g.pos, g.quat = name, mujoco.mjtGeom.mjGEOM_CYLINDER, pos, disc
        g.size = [radius, 0.0012, 0.0]
        g.rgba, g.material = [0.15, 0.15, 0.15, 1.0], mat.name
        g.contype = g.conaffinity = 0
        g.group, g.mass = group, 0.0

    base, head = spec.body("base_link"), spec.body("NeckLR")
    led(base, "led_chest", [0.0, 0.0586, 0.2913], 0.009, 2)  # the chest button ring
    for name, x in (("led_eye_left", -0.0092), ("led_eye_right", 0.0098)):
        led(head, name, [x, 0.0252, 0.0403], 0.0045, _EYE_GROUP)


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
            data, cams = mj.MjData(m), self._cameras()
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
                    mj.mj_copyData(data, m, self.sim.data)  # the LED colours live in the shared model (paint_leds)
                cams["free"].azimuth = -60.0 + 18.0 * (tick - t0)  # one turn every 20 s
                n += 1
                for cam in wanted:
                    renderer.update_scene(data, cams[cam], opt_head if cam == "head_camera" else opt)
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
    CYCLE = 0.75  # s per two steps at period 0; a cycle lasts CYCLE + CYCLE_PER_PERIOD * period (0.7 drifted
    # the heading by up to 60 deg in backward-right walks; 0.75 cut the mean drift from 15 to 6 deg)
    CYCLE_PER_PERIOD = 0.05
    K_PITCH = 0.6  # ankle pitch deg per deg of torso lean about x (+ = leaning back); 0.3 let 0.3 N.s pushes tip it
    K_ROLL = 0.0  # ankle roll deg per deg of torso lean about y (+ = leaning right)
    D_PITCH = 0.06  # ankle pitch deg per deg/s of lean rate about x (with K_PITCH 0.6: 0/384 push falls, was 24)
    D_ROLL = 0.06  # ankle roll deg per deg/s of lean rate about y
    MAX_DEG_S = 300.0  # servo slew limit on the targets (the servo family is rated about 0.2 s per 60 deg)
    STOP_S = 1.5  # s over which the stride fades out when the walk stops (0.4 s tipped it at full speed, 1.0 s at
    # some stop phases of fast walks; 2.0 s is worse: the shrinking steps stop clearing the floor). The stepping
    # itself (sway, lift) stays at full amplitude until the stride is half gone, and the ankle balance keeps its
    # full gain through the fade: the feet keep moving under the torso while its momentum is absorbed.
    TURN_DEG = 4.0  # deg of opposite fore/aft stride on the two legs while turning in place (see run())
    TURN_LIFT = 4.0  # deg of extra foot lift while turning: a foot that clears the floor pivots instead of dragging
    # (sweep: 4 deg stride + 4 deg lift turns 10-11 deg/s both ways at 9 deg tilt; 3 deg + no lift gave 3 deg/s)
    RAMP_S = 2.0  # s for a stride, side step or turn to go from zero to full: direction changes never jump
    # (1.5 s still tipped the robot in 2 of 3 full-speed diagonal-then-reverse chains with the GUI's renderer running)
    # Momentum: SimRobot.balance() gives the capture point (where the robot would have to step to come to rest).
    # In a steady walk it stays within about 3 cm outside the support circle (measured, speed 5). Beyond
    # BRAKE_M the commanded stride is wound down (over BRAKE_IN_S, released over BRAKE_OUT_S), and the swing
    # foot is placed towards the capture point (K_CP deg per m of excess offset, capped), the stepping reflex
    # that catches a push or a too-fast direction change before the torso tips.
    BRAKE_M = 0.035
    BRAKE_IN_S = 0.3
    BRAKE_OUT_S = 0.8
    CP_NORMAL_M = 0.05  # capture-point offset from the support centre that walking itself produces
    K_CP = 150.0  # deg of extra swing-foot stride per m of capture-point offset beyond CP_NORMAL_M
    K_CP_SIDE = 60.0  # deg of extra swing-foot side placement per m of sideways excess (small: the legs must not cross)
    CP_STEP_MAX = 12.0  # deg: cap on the extra swing stride
    CP_SIDE_MAX = 4.0  # deg: cap on the extra side placement
    DT = 0.02
    TUNABLE = ("crouch", "sway", "lift", "stride", "side", "back", "diag", "cycle", "cycle_per_period",
               "k_pitch", "k_roll", "d_pitch", "d_roll", "max_deg_s", "stop_s", "turn_deg", "ramp_s", "brake_m",
               "brake_in_s", "brake_out_s", "cp_normal_m", "k_cp", "k_cp_side", "cp_step_max", "cp_side_max",
               "turn_lift")

    def __init__(self, sim, max_tilt_deg=25.0, **tuning):
        self.sim, self.max_tilt = sim, max_tilt_deg
        for k, v in tuning.items():
            if k not in self.TUNABLE:
                raise TypeError(f"SimGait: unknown tuning parameter '{k}' (one of {self.TUNABLE})")
            setattr(self, k.upper(), float(v))
        self.steps_done = 0
        self.status = "idle"
        self.balance = None  # last SimRobot.balance() while walking
        self.braking = 0.0  # 0..1: how much of the commanded stride the momentum brake is taking away right now
        self.reflex = (0.0, 0.0)  # (deg forward, deg sideways) the swing foot is currently placed off its path
        self._fading = False  # the walk is stopping (or idling): pose() fades the stride before the stepping
        self._landed = {"Left": [0.0, 0.0], "Right": [0.0, 0.0]}  # where each foot landed off its path (fades in stance)

    @classmethod
    def cycle_time(cls, period):
        """Seconds per cycle (two steps) at gait period 1..5, with the class defaults."""
        return cls.CYCLE + cls.CYCLE_PER_PERIOD * min(max(period, 1), 5)

    def _cycle(self, period):
        return self.CYCLE + self.CYCLE_PER_PERIOD * min(max(period, 1), 5)

    def pose(self, phi, stride=0.0, side=0.0, amp=1.0, wave=False, crouch=None, turn=0.0, reflex=(0.0, 0.0)):
        """Leg (and arm) servo angles at cycle phase phi (0..1): left leg swings in [0, 0.5), right in [0.5, 1).
        turn (deg) is added to the right leg's stride and taken from the left one's: the robot has no hip yaw,
        so it pivots on friction and turns anticlockwise for turn > 0 (measured in the sim; slow, about 3 deg/s).
        reflex = (deg forward, deg to the left) is added to where the swinging foot lands (the capture-point
        step); the offset is remembered per foot and fades out while that foot is on the ground.
        While the walk fades out (self._fading, 0 < amp < 1) the stride, side step, turn and reflex shrink with amp
        but the stepping (sway, lift) stays at full amplitude until the stride is half gone, then fades: both feet
        landing flat while the torso still moves is what used to pitch it over at the end of a stop."""
        if 0.0 < amp < 1.0 and self._fading:
            amp_w = min(1.0, 2.0 * amp)
            k = amp / amp_w
            stride, side, turn, reflex, amp = stride * k, side * k, turn * k, (reflex[0] * k, reflex[1] * k), amp_w
        crouch = self.CROUCH if crouch is None else crouch
        r = self.SWAY * amp * math.sin(2 * math.pi * phi)  # > 0: weight on the right foot

        def shaped(u):
            return min(max((u - 0.15) / 0.7, 0.0), 1.0)

        out = {}
        for leg, offset in (("Left", 0.0), ("Right", 0.5)):
            u = (phi - offset) % 1.0
            landed = self._landed[leg]
            if u < 0.5:  # swing: lift and move from back to front, landing off its path by the reflex
                v = shaped(u / 0.5)
                lift_amp = self.LIFT + self.TURN_LIFT * min(abs(turn) / self.TURN_DEG, 1.0)  # higher steps while turning
                lift, s = lift_amp * amp * math.sin(math.pi * v), 0.5 - 0.5 * math.cos(math.pi * v)
                landed[0], landed[1] = reflex[0] * s, reflex[1] * s
                extra_f, extra_l = landed
            else:  # stance: the foot slides back under the body; the landing offset fades out
                lift, s = 0.0, 1.0 - (u - 0.5) / 0.5
                extra_f, extra_l = landed[0] * s, landed[1] * s
            leg_stride = stride + (turn if leg == "Right" else -turn)
            fwd, lat = leg_stride * amp * (2 * s - 1) + extra_f * amp, side * amp * (2 * s - 1) + extra_l * amp
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
    def describe(speed_v, speed_h, turn=0):
        parts = []
        if speed_v:
            parts.append("forward" if speed_v > 0 else "backward")
        if speed_h:
            parts.append("left" if speed_h > 0 else "right")
        text = "walking " + " and ".join(parts) if parts else ""
        if turn:
            spin = "turning " + ("anticlockwise" if turn > 0 else "clockwise")
            text = f"{text}, {spin}" if text else spin
        return text or "stepping in place"

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
        quick = bool(params.get("quick"))  # a balance reflex: no time to settle into the crouch first
        sim.set_servos_angles(self._slew(self.pose(0.0, amp=0.0), None, dt), 250 if quick else 600)
        sim.wait(0.25 if quick else 0.7)
        ref = self._lean()[:2]
        last = None
        phi, amp, t_prev = 0.0, 0.0, sim.data.time
        eff = {"stride": 0.0, "side": 0.0, "turn": 0.0}  # ramped towards the commanded values (RAMP_S)
        rate = {"stride": 5 * self.STRIDE / self.RAMP_S, "side": 5 * self.SIDE / self.RAMP_S,
                "turn": self.TURN_DEG / self.RAMP_S}
        stopping, status = False, "done"
        peak = 0.0  # the balance keeps the gain it reached: it must not fade with the stride during a stop
        self._fading = False
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
            t = sim.data.time
            step = min(max(t - t_prev, 0.0), 4 * dt)  # sim time since the last tick: in real time a tick may be late
            t_prev = t
            v = min(max(p.get("speed_v", 0), -5), 5)
            h = min(max(p.get("speed_h", 0), -5), 5)
            tn = p.get("turn", 0) or 0
            idle = not steps and not (v or h or tn)  # hold-to-walk with nothing held: stand in the crouch, do not step
            self._fading = stopping or idle
            want = {"stride": self.STRIDE * (v if v >= 0 else v * self.BACK),
                    "side": self.SIDE * h * (self.DIAG if v else 1.0),
                    "turn": self.TURN_DEG * (1 if tn > 0 else -1 if tn < 0 else 0)}  # the turn rate saturates
            for k, target in want.items():  # a new direction is reached gradually, so momentum stays small
                eff[k] += min(max(target - eff[k], -rate[k] * step), rate[k] * step)
            stride, side, turn = eff["stride"], eff["side"], eff["turn"]
            try:
                self.balance = sim.balance()
            except Exception:  # noqa: BLE001 - a model without soles or an odd state must not stop the walk
                self.balance = None
            if self.balance is not None:
                ox, oy = self.balance["offset_m"]  # capture point relative to the support centre (x right, y forward)
                over = self.balance["margin_m"] > self.BRAKE_M
                self.braking = min(self.braking + step / self.BRAKE_IN_S, 1.0) if over else \
                    max(self.braking - step / self.BRAKE_OUT_S, 0.0)
                exc_y = math.copysign(max(abs(oy) - self.CP_NORMAL_M, 0.0), oy)
                exc_x = math.copysign(max(abs(ox) - self.CP_NORMAL_M, 0.0), ox)
                self.reflex = (max(-self.CP_STEP_MAX, min(self.CP_STEP_MAX, self.K_CP * exc_y)),
                               max(-self.CP_SIDE_MAX, min(self.CP_SIDE_MAX, -self.K_CP_SIDE * exc_x)))
            if self.braking:  # momentum the feet cannot absorb: stop pushing, gradually
                stride *= 1.0 - self.braking
                side *= 1.0 - self.braking
            dphi = step / T
            amp = max(0.0, amp - step / self.STOP_S) if (stopping or idle) else min(1.0, amp + step / (T / 2))
            peak = max(peak, amp)
            if stopping and amp <= 0.0:
                break
            old, phi = phi, (phi + dphi) % 1.0
            if (old < 0.5 <= phi) or phi < old:
                self.steps_done += 1
                if on_step:
                    on_step(self.steps_done)
            target = self.pose(phi + dt / T, stride, side, amp, bool(p.get("wave")), turn=turn, reflex=self.reflex)
            self._balance(target, ref, peak)
            last = self._slew(target, last, max(step, dt))
            self._send(last, dt * 1000)
            sim.wait(dt)
        tilted = f"stopped: torso tilted over {self.max_tilt:g} deg"
        if sim.torso_tilt_deg() > self.max_tilt:
            status = tilted  # no point standing up a robot that is going over
        else:
            sim.set_servos_angles({n: RESET_POSE[n] for n in self.pose(0.0, wave=True)}, 600)
            t_end = sim.data.time + 1.0  # the stand-up and a settle: a fall here used to be reported as a clean stop
            while sim.data.time < t_end:
                sim.wait(dt)
                if sim.torso_tilt_deg() > self.max_tilt:
                    status = tilted + " while standing up"
                    break
        self._fading = False
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


__all__ = ["SimRenderer", "SimGait", "SimBattery", "add_leds", "led_rgba", "paint_led_model", "LED_GEOMS", "LED_RGB",
           "detect_colours", "encode_jpeg",
           "load_builtin", "builtin_path", "has_builtin", "camera_name", "CAMERAS", "SIM_BUILTINS", "BUILTIN_DIR"]
