"""Relay controller: the one object the web GUI drives (see docs/ARCHITECTURE.md sections 1, 2 and 4).

    from yanshee_sim.controller import Controller
    ctl = Controller()                          # mode "sim": commands go to the simulator
    ctl.play_action("wave", "right")
    ctl.connect("192.168.3.13"); ctl.set_mode("both")   # simulator and real robot together
    ctl.set_servos({"NeckLR": 60}, runtime=800)          # through the safety gate
    ctl.stop_all()

Modes: "sim" (simulator only), "robot" (real robot only; the sim shows the robot's servo readback at
4 Hz) and "both" (each command to both; for robot-only built-ins, gait and dances the sim follows the
robot's readback until the activity ends). Every command bound for the robot passes the safety gate:
servo limits (clamp + warning), self-collision along the path from the estimated in-flight pose
(refused), a speed cap (runtime stretched), motion-file validation and sweep from the current pose,
and a 30 % battery floor for motions. Robot errors never raise: methods return {"ok": False, "error"}
and log an event. Long operations run in one cancellable activity thread; a new one cancels the old.

Race rules: every motion command reads the STOP generation together with its targets before it waits for
anything (_snapshot). STOP, a mode switch and a disconnect bump the generation, so a command that was
waiting for the previous activity to end is refused afterwards instead of reaching the old targets; one
that slips past the check is undone (robot stops and holds). Commands that arrive while a STOP or mode
switch is still running are refused.
"""
import atexit
import itertools
import json
import logging
import math
import os
import re
import threading
import time
import weakref
from collections import deque

from . import motion as motion_mod
from . import sim_extras
from .mirror import SimToRobot
from .rest import SPEEDS, RobotUnreachable
from .robot import LED_COLORS, LED_MODES, ROOT, SERVO_MODES, RealRobot, SimRobot, is_system_motion
from .servos import LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, NAMES, RESET_POSE, check_angles, clamp_angles

log = logging.getLogger(__name__)

MODES = ("sim", "robot", "both")
MIN_BATTERY = 30  # % below which motions, built-ins and gait are refused (the robot itself says "lowbattery")
FALL_DEG = 45  # torso tilt that counts as fallen
TWIN_PERIOD_S = 0.25  # robot readback -> sim (4 Hz)
ROBOT_LOST_S = 5.0  # a robot activity ends with an error after this long without any answer from the robot
GAIT_LEASE_S = 1.5  # default dead-man lease for open-ended walking from the web GUI (refreshed by keepalives)
LEG_SERVOS = tuple(n for n in NAMES if any(k in n for k in ("Hip", "Knee", "Ankle")))
USER_MOTIONS_DIR = os.path.join(ROOT, "motions")
FALLBACK_MOTION_LIST = os.path.join(ROOT, "tests", "data", "motion_list_real.json")
FALLBACK_FIRST_LAYERED = "Track_TurnR"  # the captured list is hts motions, then layered (v2) ones
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")
# playback speed multiplier for the robot's speed words (the robot's own durations scale about like this)
SPEED_FACTOR = {"very slow": 0.5, "slow": 1 / 1.5, "normal": 1.0, "fast": 1 / 0.75, "very fast": 2.0}
SPEED_PERIOD = {"very slow": 5, "slow": 4, "normal": 2, "fast": 1, "very fast": 1}  # sim "walk" -> gait period
DANCES = {"WakaWaka", "LittleApple", "GangnamStyle", "HappyBirthday", "SorrySorry", "WeAreTakingOff",
          "SweetAndSour", "SiegeOfTroy", "CombinationOfSongs"}
# parametric built-ins: name, directions, category
PARAMETRIC = [("raise", ("left", "right", "both"), "gestures"), ("wave", ("left", "right", "both"), "gestures"),
              ("stretch", ("left", "right", "both"), "gestures"), ("come on", ("left", "right", "both"), "gestures"),
              ("bend", ("left", "right"), "gestures"), ("head", ("forward", "left", "right"), "gestures"),
              ("bow", (), "gestures"), ("crouch", (), "gestures"), ("reset", (), "gestures"),
              ("walk", ("forward", "backward", "left", "right"), "locomotion"),
              ("turn around", ("left", "right"), "locomotion")]
PARAM_BY_NAME = {p[0]: p for p in PARAMETRIC}
WALK_PARAMS = {"forward": (3, 0), "backward": (-3, 0), "left": (0, 3), "right": (0, -3)}  # (speed_v, speed_h)
PHOTO_RESOLUTIONS = ("320x240", "640x480", "1280x720", "1920x1080")  # robot camera (YanAPI: max 1920x1080)
VISION_TASKS = {**{f"face:{t}": ("face", t) for t in
                   ("recognition", "quantity", "age", "age_group", "gender", "expression", "mask", "glass")},
                "gesture": ("hand", "gesture"), "color": ("color", "color_detect"),
                "object": ("object", "recognition"), "qr": ("qr", "")}


class ModeError(ValueError):
    """The requested mode needs a connected robot, or is not one of MODES."""


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class _NoSim:
    """SimToRobot wants a sim to mirror; the gate is fed by the controller instead."""

    def jump_to(self, angles):
        pass

    def get_target_angles(self):
        return {}


class RobotGate(SimToRobot):
    """SimToRobot used as a command gate: tracks every command sent to the robot so the speed cap and
    the collision sweep start from the estimated in-flight pose. Not thread-safe: the controller locks it."""

    def __init__(self, robot, checker, max_speed_deg_s=90):
        super().__init__(_NoSim(), robot, max_speed_deg_s=max_speed_deg_s, deadband_deg=0.0,
                         min_runtime_ms=MIN_RUNTIME_MS, checker=checker)

    def sync(self, pose):
        """The robot is really at pose with nothing in flight (fresh readback)."""
        self._moves, self.sent = {}, {}
        self._set_sent({**RESET_POSE, **{n: int(round(d)) for n, d in clamp_angles(pose).items()}}, 0, self.clock())

    def busy(self, margin=0.0):
        """A commanded move is (or may still be) in flight. sync()'s zero-length entries are poses, not moves:
        counting them kept busy(margin) true right after every 4 Hz twin sync."""
        now = self.clock()
        return any(dur > 0 and now < t0 + dur + margin for _, _, t0, dur in self._moves.values())

    def note_sent(self, angles, runtime_ms):
        """Record a command sent outside command() (motion-file frames)."""
        self._set_sent({n: int(round(d)) for n, d in angles.items()}, runtime_ms, self.clock())

    def runtime_for(self, angles):
        """Runtime (ms) the speed cap needs to reach angles from the estimated in-flight pose (200-4000)."""
        est = {**self.sent, **self.estimated()}
        dist = max([abs(int(round(v)) - est.get(n, v)) for n, v in angles.items()] + [0])
        return int(min(max(math.ceil(dist / self.max_speed * 1000), MIN_RUNTIME_MS), MAX_RUNTIME_MS))

    def command(self, angles, runtime_ms):
        """Check and send. Returns (status, info, runtime_ms): status "sent", "blocked" (info = pairs) or "error"."""
        goal = {n: int(round(v)) for n, v in angles.items()}
        hits = self.path_contacts(goal)
        if hits:
            return "blocked", sorted(hits), runtime_ms
        est = {**self.sent, **self.estimated()}
        dist = max([abs(v - est.get(n, v)) for n, v in goal.items()] + [0])
        need = math.ceil(dist / self.max_speed * 1000)
        if need > MAX_RUNTIME_MS:
            return "error", f"move of {dist:.0f} deg is too far for {self.max_speed:g} deg/s in 4 s", runtime_ms
        runtime = int(min(max(runtime_ms, need, MIN_RUNTIME_MS), MAX_RUNTIME_MS))
        status, info = self._send(goal, runtime)
        return status, info, runtime


class _Fanout:
    """motion.play() target: each frame goes to the robot (tracked by the gate) and/or the sim.

    Robot-bound frames obey the gate's speed cap: a frame that would move a servo faster than max_speed from its
    estimated in-flight position gets a longer runtime (the sim gets the same one, so both stay in step), and the
    playback waits the extra time, so the motion keeps its shape, only slower."""

    def __init__(self, ctl, sim, robot, stop=None):
        self.ctl, self.sim, self.robot, self.stop = ctl, sim, robot, stop
        self.extra_s, self.stretched = 0.0, []

    def set_servos_angles(self, angles, runtime=200):
        if self.robot is not None:
            if self.ctl.robot is not self.robot:  # disconnected (or reconnected) while playing: stop here
                raise RuntimeError("robot disconnected")
            with self.ctl._gate_lock:
                gate = self.ctl._live_gate(self.robot)
                if gate is None:
                    raise RuntimeError("robot disconnected")
                need = gate.runtime_for(angles)
            if need > runtime:
                self.stretched.append((runtime, need))
                self.extra_s += (need - runtime) / 1000
                runtime = need
            self.robot.set_servos_angles(angles, runtime)  # raises on refusal: the activity stops
            with self.ctl._gate_lock:
                gate = self.ctl._live_gate(self.robot)
                if gate is not None:
                    gate.note_sent(angles, runtime)
        if self.sim is not None:
            self.sim.set_servos_angles(clamp_angles(angles), runtime)
        return {"code": 0}

    def wait(self, seconds):
        seconds, self.extra_s = max(0.0, seconds) + self.extra_s, 0.0
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            if self.stop is not None and self.stop.is_set():
                return
            time.sleep(min(left, 0.05))


class _Activity:
    def __init__(self, kind, name, target, duration=None, twin=False, detail=""):
        self.kind, self.name, self.target, self.duration, self.twin = kind, name, target, duration, twin
        self.detail, self.params, self.error = detail, {}, None
        self.started, self.mono = time.time(), time.monotonic()
        self.stop, self.done = threading.Event(), threading.Event()
        self.hard = False  # STOP: end a robot walk at once instead of decelerating first
        self.thread, self.gen = None, None
        self.lease_s = self.lease_until = None  # dead-man: stop unless keepalive() is called in time

    def keepalive(self):
        if self.lease_s:
            self.lease_until = time.monotonic() + self.lease_s

    def snapshot(self):
        progress = 0.0
        if self.duration:
            progress = min(1.0, (time.monotonic() - self.mono) / self.duration)
        return {"kind": self.kind, "name": self.name, "target": self.target, "started": round(self.started, 3),
                "progress": round(progress, 3), "detail": self.detail}


def _close_at_exit(obj):
    """Close obj when the interpreter exits (a renderer thread inside EGL at shutdown can crash Python)."""
    ref = weakref.ref(obj)

    def close():
        o = ref()
        if o is not None:
            try:
                o.close()
            except Exception:
                pass
    atexit.register(close)


class Controller:
    """Relay between the simulator and a real Yanshee, with a safety gate. All public methods are thread-safe."""

    def __init__(self, start_mode="sim", sim_viewer=False, fixed_base=False, ip=None, renderer=True,
                 render_size=(640, 480), fps=12, motions_dir=None, max_speed_deg_s=90, robot_timeout=3.0,
                 min_battery=MIN_BATTERY):
        if start_mode not in MODES:
            raise ModeError(f"mode must be one of {MODES}, not {start_mode!r}")
        if not _num(max_speed_deg_s) or max_speed_deg_s < 45:
            raise ValueError("max_speed_deg_s must be >= 45 (a full 180 deg move must fit in 4 s)")
        self.max_speed, self.robot_timeout, self.min_battery = max_speed_deg_s, robot_timeout, min_battery
        self.motions_dir = motions_dir or USER_MOTIONS_DIR
        self.events = deque(maxlen=500)
        self._ids = itertools.count(1)
        self._lock = threading.RLock()  # mode, robot, caches
        self._act_lock = threading.Lock()  # serialises activity starts
        self._gate_lock = threading.RLock()
        self._conn_lock = threading.RLock()  # connect / disconnect / set_mode
        self.mode = "sim"
        self.sim = SimRobot(viewer=sim_viewer, fixed_base=fixed_base, spec_hook=sim_extras.add_leds)
        self.sim.on_key = self._viewer_key  # WASD/arrows, Q/E, X/Esc in the viewer window (when there is one)
        self.checker = motion_mod.CollisionChecker()
        self.renderer = sim_extras.SimRenderer(self.sim, *render_size, fps=fps) if renderer else None
        self.battery = sim_extras.SimBattery()
        self.robot = self.gate = None
        self._robot_info = self._empty_robot_info()
        self._robot_servos, self._robot_read_t, self._robot_motions = None, 0.0, None
        self._twin_stop = None
        self._activity = self._side = self._music = None
        self._stop_gen = 0  # bumped by STOP, mode switches and disconnect (see _snapshot)
        self._stopping = 0  # > 0 while a STOP or mode switch runs: motion commands are refused
        self._gate_stale = False  # the robot moved outside the gate (built-in, gait, failed command): resync first
        self._settings = {"volume": 50, "language": "en", "fall_management": False}
        self._leds = {k: dict(v) for k, v in self.sim.led.items()}
        self._tilt, self._fallen, self._fallen_since = 0.0, False, None
        self._balance_margin = None  # m, from SimRobot.balance() at the monitor's rate
        # sim mode, idle and pushed: take a quick step towards the capture point. Off by default: in tests it did
        # not save the robot from a 0.5 N.s shove at the head (the step starts too late) and once made it worse.
        self.reflex_enabled = False
        self._reflex_at = 0.0
        self._running = True
        self._monitor = threading.Thread(target=self._monitor_loop, name="ctl-monitor", daemon=True)
        self._monitor.start()
        _close_at_exit(self)
        self._event("info", "simulator ready" + (" (fixed base)" if fixed_base else ""))
        if ip:
            res = self.connect(ip)
            if not res["ok"] and start_mode != "sim":
                raise ModeError(res["error"])
        if start_mode != "sim":
            self.set_mode(start_mode)

    # -- events and helpers ---------------------------------------------------------
    def _event(self, level, text):
        ev = {"id": next(self._ids), "t": round(time.time(), 3), "level": level, "text": text}
        self.events.append(ev)
        getattr(log, {"warn": "warning"}.get(level, level), log.info)(text)
        return ev

    def events_since(self, last_id=0):
        """Events with id > last_id (oldest first)."""
        return [e for e in list(self.events) if e["id"] > last_id]

    def _fail(self, text, level="error", **extra):
        self._event(level, text)
        return {"ok": False, "error": text, **extra}

    @staticmethod
    def _empty_robot_info():
        return {"connected": False, "ip": None, "battery": None, "charging": None, "versions": None,
                "online": False}

    def _targets(self):
        """(sim?, robot?) for the current mode, plus the robot object (None when not connected)."""
        with self._lock:
            mode, robot = self.mode, self.robot
        return mode in ("sim", "both"), mode in ("robot", "both") and robot is not None, robot

    def _live_gate(self, robot):
        """The safety gate if robot (from _targets()) is still the connected one, else None: disconnect() may run
        between _targets() and the command. Call with _gate_lock held."""
        gate = self.gate
        return gate if gate is not None and robot is not None and self.robot is robot else None

    @staticmethod
    def _call(what, fn):
        """Run a robot call: (reply, None) on success, (reply or None, error text) otherwise. Never raises."""
        try:
            res = fn()
        except Exception as e:  # robot unreachable, refused, malformed reply, bad argument
            return None, f"{what}: {e}"
        if isinstance(res, dict) and res.get("code", 0) != 0:
            hint = " (low battery: charge the robot)" if res.get("code") == 6 else ""
            return res, f"{what}: robot answered code {res.get('code')} {res.get('msg', '')}{hint}".rstrip()
        return res, None

    def _sim_angles(self):
        return self.sim.get_servos_angles()["data"]

    def _robot_angles(self, fresh=True):
        """Robot servo readback (fresh read, falling back to the twin cache). None if unavailable."""
        _, use_robot, robot = self._targets()
        if robot is None:
            return None
        if fresh:
            res, err = self._call("servo read", lambda: robot.get_servos_angles(NAMES))
            if not err:
                angles = clamp_angles(res["data"])
                with self._lock:
                    self._robot_servos, self._robot_read_t = angles, time.time()
                return dict(angles)
        with self._lock:
            return dict(self._robot_servos) if self._robot_servos else None

    def _current_pose(self):
        """The pose commands start from: robot readback when the robot is a target, else the sim."""
        _, use_robot, _ = self._targets()
        if use_robot:
            pose = self._robot_angles(fresh=True)
            if pose is not None:
                return {**RESET_POSE, **pose}
        return self._sim_angles()

    def _robot_battery(self, max_age=10.0):
        with self._lock:
            info, robot = self._robot_info, self.robot
            stale = time.monotonic() - info.get("_battery_t", 0.0) > max_age
        if robot is not None and stale:
            res, err = self._call("battery", robot.battery_info)
            if not err:
                with self._lock:
                    info.update(battery=res["percent"], charging=res["charging"], _battery_t=time.monotonic())
        return info.get("battery")

    def _battery_refusal(self, use_sim, use_robot):
        """Error text if a motion must be refused for low battery, else None."""
        if use_robot:
            b = self._robot_battery()
            if b is not None and b < self.min_battery:
                return (f"robot battery {b}% is below {self.min_battery}%: charge it (motions are refused; "
                        "servo commands, STOP and settings still work)")
        if use_sim and self.battery.percent < self.min_battery:
            return (f"sim battery {self.battery.percent:.0f}% is below {self.min_battery}%: "
                    "charge it (Settings, sim battery)")
        return None

    # -- background threads ------------------------------------------------------------
    def _monitor_loop(self):
        last = time.monotonic()
        while self._running:
            time.sleep(0.2)
            now = time.monotonic()
            try:
                self.battery.update(self.sim, now - last)
                tilt = self.sim.torso_tilt_deg()
                self._balance_margin = self.sim.balance()["margin_m"]
            except Exception as e:  # never let the monitor die
                log.warning("monitor: %s", e)
                continue
            last = now
            act = self._activity
            if act is not None and act.lease_until is not None and now > act.lease_until and \
                    not act.done.is_set() and not act.stop.is_set():
                act.stop.set()  # the activity thread ends the walk (robot: gait_exit)
                self._event("warn", f"{act.kind} '{act.name}' stopped: no keepalive for {act.lease_s:g} s "
                                    "(controller page closed or connection lost)")
            fallen = tilt > FALL_DEG and not self.sim.fixed_base
            margin = self._balance_margin
            if self.reflex_enabled and margin is not None and margin > self.REFLEX_MARGIN_M and not fallen and \
                    tilt < 30 and not self.sim.fixed_base and self.mode == "sim" and now - self._reflex_at > 2.0 and \
                    (act is None or act.done.is_set()) and not self._stopping:
                self._reflex_at = now  # pushed while standing: a quick step towards where it is falling
                self._event("info", f"balance reflex: stepping to catch {100 * margin:.1f} cm of momentum")
                threading.Thread(target=self.gait, args=(0, 0), kwargs={"steps": 2, "period": 1, "quick": True},
                                 name="ctl-reflex", daemon=True).start()
            if fallen and not self._fallen:
                self._event("warn", "the simulated robot fell over (press Get up)")
                self._fallen_since = now
            self._tilt, self._fallen = tilt, fallen
            if fallen and self._twin_active() and now - (self._fallen_since or now) > 2.0:
                self.sim.respawn()  # a fallen twin is useless: stand it up again
                self._event("info", "the twin fell in the sim and was stood up again")
                self._fallen_since = now

    def _twin_active(self):
        with self._lock:
            act = self._activity
            return self.robot is not None and (self.mode == "robot" or (
                self.mode == "both" and act is not None and act.twin and not act.done.is_set()))

    def _robot_activity_running(self):
        act = self._activity
        return act is not None and not act.done.is_set() and act.target in ("robot", "both")

    def _twin_loop(self, robot, stop):
        fails, last_poll = 0, time.monotonic()
        while not stop.wait(TWIN_PERIOD_S):
            try:
                angles = clamp_angles(robot.get_servos_angles(NAMES)["data"])
            except Exception as e:
                fails += 1
                if fails == 3:
                    self._event("error", f"robot is not answering servo reads ({e})")
                    with self._lock:
                        self._robot_info["online"] = False
                continue
            if fails >= 3:
                self._event("info", "robot is answering again")
            fails = 0
            with self._lock:
                if self.robot is not robot:
                    return
                self._robot_info["online"] = True
                self._robot_servos, self._robot_read_t = angles, time.time()
            if self._twin_active():
                try:
                    self.sim.set_servos_angles(angles, int(TWIN_PERIOD_S * 1000) + 50)
                except Exception as e:
                    log.warning("twin: %s", e)
            with self._gate_lock:
                if self.gate is not None and not self.gate.busy(margin=0.5) and not self._robot_activity_running():
                    self.gate.sync(angles)
                    self._gate_stale = False
            if time.monotonic() - last_poll > 10.0:
                last_poll = time.monotonic()
                self._robot_battery(max_age=0)
                music = self._music
                if music and music["target"] != "sim":
                    res, err = self._call("music state", robot.rest.music_state)
                    if not err and (res.get("data") or {}).get("status") == "idle" and \
                            time.time() - music["started"] > 3:
                        self._music = None

    # -- activities ------------------------------------------------------------------------
    def _cancel_activity(self, timeout=6.0):
        act = self._activity
        if act is not None and not act.done.is_set():
            act.stop.set()
            if act.thread is not threading.current_thread():
                act.done.wait(timeout)

    def _launch(self, act, fn, gen):
        """Start fn(act) in the activity thread (caller holds _act_lock and has cancelled the old one).
        Returns None, without starting, if STOP, a mode switch or a disconnect came since _snapshot() gave gen."""
        def run():
            try:
                fn(act)
            except Exception as e:
                act.error = str(e)
                self._event("error", f"{act.kind} '{act.name}' stopped: {e}")
                try:
                    self.sim.stop_all()
                except Exception:
                    pass
            finally:
                act.done.set()
        act.thread, act.gen = threading.Thread(target=run, name=f"activity-{act.kind}", daemon=True), gen
        act.keepalive()
        with self._lock:
            if gen is None or gen != self._stop_gen:
                act.done.set()
                return None
            self._activity = act
            if act.target != "sim" and act.kind != "motion":  # built-ins and gait move the robot outside the gate
                with self._gate_lock:
                    self._gate_stale = True
            act.thread.start()
        return act

    def _target_name(self, use_sim, use_robot):
        return "both" if use_sim and use_robot else ("robot" if use_robot else "sim")

    def _wait_robot_motion(self, act, robot, version, total_s):
        """Poll the robot's motion state until it is idle again. Sends stop if cancelled. True when done."""
        t0 = last_ok = time.monotonic()
        deadline = t0 + (max(total_s, 1.0) + 5.0 if total_s else 600.0)
        seen_run, last_poll = False, t0
        while not act.stop.wait(0.3):
            fn = robot.rest.motion_state if version == "v1" else robot.rest.layer_motion_state
            res, err = self._call("motion state", fn)
            now = time.monotonic()
            if res is not None:  # any answer, even an error code, means the robot is there
                last_ok = now
            elif now - last_ok > ROBOT_LOST_S:
                raise RuntimeError(f"robot stopped answering ({err})")
            if not err:
                data = res.get("data")
                states = [d.get("status") for d in data if isinstance(d, dict)] if isinstance(data, list) else \
                    [(data or {}).get("status")]
                running = "run" in states or "pause" in states
                if "pause" in states:  # a paused motion does not use up its time
                    deadline += now - last_poll
                seen_run = seen_run or running
                if not running and (seen_run or now - t0 > 1.0):
                    return True
            last_poll = now
            if now > deadline:
                return True
        self._call("motion stop", lambda: robot.rest.motion_control("stop", version=version))
        return False

    def _sim_play(self, act, mo, speed):
        ok = motion_mod.play(self.sim, mo, speed=speed, stop_event=act.stop)
        if not ok:
            self.sim.stop_all()
        return ok

    # -- command guard (see the module docstring) ------------------------------------------------
    def _snapshot(self):
        """(gen, use_sim, use_robot, robot), read together before any waiting. gen is None while a STOP or mode
        switch runs (the command must be refused); otherwise pass it to _begin(), _current() and _launch()."""
        with self._lock:
            gen = None if self._stopping else self._stop_gen
            mode, robot = self.mode, self.robot
        return gen, mode in ("sim", "both"), mode in ("robot", "both") and robot is not None, robot

    def _current(self, gen):
        """No STOP, mode switch or disconnect since _snapshot() returned gen (so its targets are still valid)."""
        with self._lock:
            return gen is not None and gen == self._stop_gen

    def _begin(self, gen, robot=None):
        """Cancel the running activity (waiting for it), then None if gen is still current, else why the command
        must be refused (robot: the one the command is aimed at, to name a disconnect). Call with _act_lock held."""
        if gen is None:
            return "refused: STOP or a mode switch is in progress"
        self._cancel_activity()
        if self._current(gen):
            return None
        if robot is not None and self.robot is not robot:
            return "not sent: robot disconnected"
        return "cancelled by STOP (or a mode switch)"

    def _sim_send(self, gen, fn):
        """Run fn() (a sim command) unless gen is stale. Under _lock: a STOP either comes first (refused here)
        or after (and its sim.stop_all() holds the pose)."""
        with self._lock:
            if gen is None or gen != self._stop_gen:
                return False
            fn()
            return True

    def _resync_gate(self, robot):
        """Fresh readback -> gate.sync(); None or the error text. Needed after the robot moved outside the gate."""
        res, err = self._call("servo read", lambda: robot.get_servos_angles(NAMES))
        if err:
            return err
        pose = clamp_angles(res["data"])
        with self._lock:
            if self.robot is robot:
                self._robot_servos, self._robot_read_t = pose, time.time()
        with self._gate_lock:
            gate = self._live_gate(robot)
            if gate is None:
                return "robot disconnected"
            gate.sync(pose)
            self._gate_stale = False
        return None

    def _hold_robot(self, robot, force=False):
        """Stop a servo move in flight on the robot: read the angles and command them over MIN_RUNTIME_MS. Only
        when the gate shows a move in flight, or force. Returns the held pose or None. Never raises."""
        with self._gate_lock:
            gate = self._live_gate(robot)
            busy = gate is not None and gate.busy()
        if not (busy or force) or robot is None:
            return None
        res, err = self._call("servo read", lambda: robot.get_servos_angles(NAMES))
        if err:
            self._event("warn", f"stop: cannot read the servos to hold them ({err})")
            return None
        pose = clamp_angles(res["data"])
        _, err = self._call("hold", lambda: robot.set_servos_angles(pose, MIN_RUNTIME_MS))
        if err:
            self._event("warn", f"stop: cannot hold the servos ({err})")
            return None
        with self._gate_lock:
            gate = self._live_gate(robot)
            if gate is not None:
                gate.sync(pose)
        return pose

    def _undo_robot(self, robot):
        """A command reached the robot after STOP / a mode switch: stop everything there and hold the servos."""
        robot.stop_all()
        self._hold_robot(robot, force=True)

    def _call_start(self, what, fn, undo):
        """_call() for a request that starts something on the robot. A request that timed out (not refused) may
        still reach the robot and run untracked: undo() is sent best-effort, now and again one robot timeout later
        (a delayed start can overtake the first stop), and the error says so."""
        try:
            res = fn()
        except RobotUnreachable as e:
            if "refused" in str(e).lower():
                return None, f"{what}: {e}"

            act0 = self._activity

            def undo_twice():
                self._call(f"{what} undo", undo)
                time.sleep(self.robot_timeout)
                if self._activity is act0:  # not if the user started something new meanwhile
                    self._call(f"{what} undo", undo)
            threading.Thread(target=undo_twice, name="ctl-undo", daemon=True).start()
            return None, f"{what}: {e} (the robot may still have received it: stop sent)"
        except Exception as e:
            return None, f"{what}: {e}"
        if isinstance(res, dict) and res.get("code", 0) != 0:
            hint = " (low battery: charge the robot)" if res.get("code") == 6 else ""
            return res, f"{what}: robot answered code {res.get('code')} {res.get('msg', '')}{hint}".rstrip()
        return res, None

    # -- modes and connection --------------------------------------------------------------
    def set_mode(self, mode):
        """Switch the relay. Stops everything first. Raises ModeError if mode needs a robot that is not connected."""
        if mode not in MODES:
            raise ModeError(f"mode must be one of {MODES}, not {mode!r}")
        with self._conn_lock:
            if mode != "sim" and self.robot is None:
                raise ModeError(f"mode '{mode}' needs a connected robot: connect(ip) first")
            with self._lock:
                self._stopping += 1  # commands are refused until the new mode is in place
            try:
                self.stop_all()
                with self._lock:
                    self.mode = mode
                    self._stop_gen += 1  # commands that read the old targets are refused
            finally:
                with self._lock:
                    self._stopping -= 1
            if mode != "sim":
                pose = self._robot_angles(fresh=True)
                if pose:
                    with self._gate_lock:
                        self.gate.sync(pose)
                    if mode == "both":  # start the sim in the robot's pose, so both run the same commands
                        self.sim.set_servos_angles(pose, 1000)
            self._event("info", f"mode: {mode}")
        return {"ok": True, "mode": mode}

    def connect(self, ip):
        """Connect the real robot (ip or ip:port). Does not change the mode."""
        if not isinstance(ip, str) or not ip.strip():
            return self._fail("connect: give the robot's IP address")
        ip = ip.strip()
        with self._conn_lock:
            if self.robot is not None:
                self.disconnect()
            self._event("info", f"connecting to {ip} ...")
            try:
                robot = RealRobot(ip, min_battery=0, timeout=self.robot_timeout)
            except Exception as e:
                return self._fail(f"cannot connect to {ip}: {e}")
            res, err = self._call("servo read", lambda: robot.get_servos_angles(NAMES))
            if err:
                return self._fail(f"connected to {ip} but cannot read the servos: {err}")
            angles = clamp_angles(res["data"])
            versions = {}
            for t in ("core", "servo", "sn"):
                r, e = self._call(f"version {t}", lambda t=t: robot.rest.versions(t))
                versions[t] = (r.get("data") or {}).get(t) if not e else None
            settings = {}
            for key, fn, field in (("volume", robot.rest.get_volume, "volume"),
                                   ("language", robot.rest.get_language, "language"),
                                   ("fall_management", robot.rest.get_fall_management, "enable")):
                r, e = self._call(key, fn)
                if not e and isinstance(r.get("data"), dict) and field in r["data"]:
                    settings[key] = r["data"][field]
            leds = {}
            r, e = self._call("leds", robot.rest.get_led)
            if not e and isinstance(r.get("data"), list):
                leds = {d["type"]: {"color": d.get("color"), "mode": d.get("mode")} for d in r["data"]
                        if isinstance(d, dict) and d.get("type") in LED_COLORS}
            gate = RobotGate(robot, self.checker, self.max_speed)
            gate.sync(angles)
            stop = threading.Event()
            with self._lock:
                self.robot, self._twin_stop = robot, stop
                with self._gate_lock:
                    self.gate = gate
                self._robot_info = {"connected": True, "ip": ip, "battery": robot.battery, "charging": None,
                                    "versions": versions, "online": True, "_battery_t": time.monotonic()}
                self._robot_servos, self._robot_read_t, self._robot_motions = angles, time.time(), None
                self._settings.update(settings)
                self._leds.update(leds)
            threading.Thread(target=self._twin_loop, args=(robot, stop), name="ctl-twin", daemon=True).start()
            self._event("info", f"connected to robot at {ip} (battery {robot.battery}%)")
            if robot.battery < self.min_battery:
                self._event("warn", f"robot battery {robot.battery}% is below {self.min_battery}%: "
                                    "motions will be refused until it is charged")
            return {"ok": True, "ip": ip, "battery": robot.battery, "versions": versions}

    def disconnect(self):
        """Forget the robot (switches to sim mode first if needed)."""
        with self._conn_lock:
            if self.robot is None:
                return {"ok": True}
            if self.mode != "sim":
                self.set_mode("sim")
            with self._lock:
                if self._twin_stop is not None:
                    self._twin_stop.set()
                ip = self._robot_info.get("ip")
                self.robot, self._twin_stop, self._robot_servos = None, None, None
                self._stop_gen += 1
                with self._gate_lock:
                    self.gate = None
                self._robot_info = self._empty_robot_info()
            self._event("info", f"disconnected from {ip}")
            return {"ok": True}

    # -- state ------------------------------------------------------------------------------
    def state(self):
        """JSON-serialisable snapshot (ARCHITECTURE.md section 4). Never touches the network."""
        with self._lock:
            robot_info = {k: v for k, v in self._robot_info.items() if not k.startswith("_")}
            robot_servos = dict(self._robot_servos) if self._robot_servos else None
            mode, act, side, music = self.mode, self._activity, self._side, self._music
            settings, leds = dict(self._settings), {k: dict(v) for k, v in self._leds.items()}
        if act is not None and not act.done.is_set():
            activity = act.snapshot()
        elif side is not None:
            activity = dict(side, progress=0.0)
        elif music is not None:
            activity = {"kind": "music", "name": music["name"], "target": music["target"],
                        "started": music["started"], "progress": 0.0, "detail": ""}
        else:
            activity = {"kind": "idle", "name": "", "target": "", "started": 0.0, "progress": 0.0, "detail": ""}
        renderer_error = self.renderer.error if self.renderer is not None else "renderer disabled"
        return {"mode": mode, "robot": robot_info,
                "sim": {"battery": round(self.battery.percent, 1), "tilt_deg": round(self._tilt, 1),
                        "fallen": self._fallen, "fixed_base": self.sim.fixed_base, "renderer_error": renderer_error,
                        "balance_margin_cm": None if self._balance_margin is None else round(100 * self._balance_margin, 1)},
                "servos": {"sim": self._sim_angles(), "robot": robot_servos},
                "activity": activity, "leds": leds, **settings,
                "limits": {n: list(v) for n, v in LIMITS.items()}, "reset_pose": dict(RESET_POSE),
                "events": list(self.events)[-50:]}

    # -- stop and reset -----------------------------------------------------------------------
    def stop_all(self):
        """STOP: cancel the activity, hold the sim pose, and send every stop to the robot (whatever the mode). A servo
        move still in flight on the robot (slider, motion-file frame) is halted by commanding the current readback."""
        with self._lock:
            self._stop_gen += 1
            self._stopping += 1
            act, robot = self._activity, self.robot
            self._music = None
        try:
            if act is not None:
                act.hard = True
                act.stop.set()
            self.sim.stop_all()
            res = robot.stop_all() if robot is not None else None
            self._cancel_activity(timeout=5.0)
            self.sim.stop_all()
            held = self._hold_robot(robot) if robot is not None else None
        finally:
            with self._lock:
                self._stopping -= 1
        if res is not None and res.get("code") != 0:
            self._event("warn", f"stop: some robot stops failed ({res.get('msg')})")
        else:
            self._event("info", "stopped" + (" (robot servos held where they were)" if held else ""))
        try:  # STOP holds the pose: say so when that pose is a bent one (a halted bow, crouch or squat)
            targets = self.sim.get_target_angles()
            off = max(abs(targets[n] - RESET_POSE[n]) for n in RESET_POSE if "Hip" in n or "Knee" in n or "Ankle" in n)
            if off > 10 or self.sim.torso_tilt_deg() > 10:
                self._event("info", "stopped in a bent pose: press Reset to stand straight")
        except Exception:  # noqa: BLE001 - a hint only
            pass
        return {"ok": True, "robot": res, "held": bool(held)}

    def reset(self):
        """Stand in the reset pose: the sim respawns if it fell, the robot moves through the gate."""
        gen, use_sim, use_robot, robot = self._snapshot()
        with self._act_lock:
            act = self._activity
            robot_was_busy = act is not None and not act.done.is_set() and act.target != "sim"
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"reset {why}", level="warn")
            warnings = []
            if use_robot:
                if robot_was_busy or self._gate_stale:
                    err = self._resync_gate(robot)
                    if err:
                        return self._fail(f"reset: cannot read the robot's pose ({err})")
                with self._gate_lock:
                    gate = self._live_gate(robot)
                    status, info, rt = (gate.command(dict(RESET_POSE), 1000) if gate is not None
                                        else ("error", "robot disconnected", 1000))
                if status == "blocked":  # the straight path collides: let the robot's own reset do it
                    warnings.append(f"straight path to reset collides ({info}); using the robot's reset motion")
                    res, err = self._call("reset", lambda: robot.play_builtin("reset"))
                    if err:
                        return self._fail(err)
                    with self._gate_lock:
                        self._gate_stale = True
                elif status != "sent":
                    with self._gate_lock:
                        self._gate_stale = True
                    return self._fail(f"reset: {info}")
                if not self._current(gen):
                    self._undo_robot(robot)
                    return self._fail("reset cancelled by STOP", level="warn")
            if use_sim:
                def sim_reset():
                    if self._fallen or self.sim.torso_tilt_deg() > FALL_DEG:
                        self.sim.respawn()
                    else:
                        self.sim.set_servos_angles(dict(RESET_POSE), 1000)
                if not self._sim_send(gen, sim_reset):
                    return self._fail("reset cancelled by STOP", level="warn")
            self._event("info", "reset")
        return {"ok": True, "warnings": warnings}

    # -- servos ----------------------------------------------------------------------------------
    def set_servos(self, angles, runtime=500):
        """Move servos {name: deg} over runtime ms on the current targets, through the safety gate.
        Returns {"ok", "sent_to", "warnings", "blocked", "runtime"}."""
        if not isinstance(angles, dict) or not angles:
            return self._fail("set_servos: angles must be a non-empty {servo: degrees} object", level="warn")
        bad = [p for p in check_angles(angles) if "outside" not in p]
        if bad:
            return self._fail("set_servos: " + "; ".join(bad), level="warn", sent_to=[], blocked=[])
        if not _num(runtime):
            return self._fail(f"set_servos: runtime {runtime!r} is not a number", level="warn")
        warnings = [f"{p}: clamped" for p in check_angles(angles)]
        target = clamp_angles(angles)
        rt = int(min(max(runtime, MIN_RUNTIME_MS), MAX_RUNTIME_MS))
        if rt != runtime:
            warnings.append(f"runtime {runtime} ms clamped to {rt} ms (YanAPI accepts 200-4000)")
        gen, use_sim, use_robot, robot = self._snapshot()
        sent_to = []
        with self._act_lock:
            act = self._activity
            running = act is not None and not act.done.is_set()
            if running:
                warnings.append(f"cancelled {act.kind} '{act.name}' (manual control takes over)")
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"set_servos {why}", level="warn", sent_to=[], blocked=[], warnings=warnings)
            if use_robot:
                if (running and act.target != "sim") or self._gate_stale:  # the robot moved outside the gate
                    err = self._resync_gate(robot)
                    if err:
                        return self._fail(f"robot servo command not sent: cannot read the robot's pose ({err})",
                                          blocked=[], sent_to=[], warnings=warnings)
                with self._gate_lock:
                    gate = self._live_gate(robot)
                    status, info, used = (gate.command(target, rt) if gate is not None
                                          else ("error", "robot disconnected", rt))
                    if status == "error":
                        self._gate_stale = True  # a timed-out command may still have moved the robot
                if status == "blocked":
                    text = ", ".join(f"{a} and {b}" for a, b in info)
                    return self._fail(f"refused: self-collision between {text}", level="warn",
                                      blocked=[list(p) for p in info], sent_to=[], warnings=warnings)
                if status != "sent":
                    return self._fail(f"robot servo command failed: {info}", blocked=[], sent_to=[],
                                      warnings=warnings)
                if used > rt:
                    warnings.append(f"runtime stretched from {rt} to {used} ms by the {self.max_speed:g} deg/s "
                                    "speed cap")
                rt = used
                sent_to.append("robot")
                if not self._current(gen):  # STOP or a mode switch while sending: halt it again
                    self._undo_robot(robot)
                    return self._fail("set_servos cancelled by STOP", level="warn", blocked=[], sent_to=[],
                                      warnings=warnings)
            if use_sim:
                if not self._sim_send(gen, lambda: self.sim.set_servos_angles(target, rt)):
                    return self._fail("set_servos cancelled by STOP", level="warn", blocked=[], sent_to=sent_to,
                                      warnings=warnings)
                sent_to.append("sim")
        for w in warnings:
            self._event("warn", w)
        return {"ok": True, "sent_to": sent_to, "warnings": warnings, "blocked": [], "runtime": rt}

    def get_servos(self):
        """{"ok", "sim": {...}, "robot": {...} or None} (robot = the latest readback, at most 0.25 s old)."""
        with self._lock:
            robot = dict(self._robot_servos) if self._robot_servos else None
        return {"ok": True, "sim": self._sim_angles(), "robot": robot}

    def set_servo_mode(self, mode, names=None):
        """PRP: "program" makes the servos limp (pose them by hand), "work" holds where they are."""
        if mode not in SERVO_MODES:
            return self._fail(f"servo mode must be one of {SERVO_MODES}, not {mode!r}", level="warn")
        names = list(NAMES if names is None else ([names] if isinstance(names, str) else names))
        unknown = [n for n in names if n not in LIMITS]
        if unknown:
            return self._fail(f"unknown servos {unknown}", level="warn")
        gen, use_sim, use_robot, robot = self._snapshot()
        warnings = []
        with self._act_lock:
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"servo mode {why}", level="warn")
            if use_robot:
                res, err = self._call(f"servo mode {mode}", lambda: robot.set_servos_mode(mode, names))
                if err:
                    return self._fail(err)
                legs = [n for n in names if n in LEG_SERVOS]
                if mode == "program" and legs:
                    warnings.append(f"{len(legs)} leg servos are limp on the real robot: it cannot stand "
                                    "(hold it or lay it down; Hold makes them stiff again)")
                if mode == "work":
                    pose = self._robot_angles(fresh=True)
                    if pose:
                        with self._gate_lock:
                            gate = self._live_gate(robot)
                            if gate is not None:
                                gate.sync(pose)
            if use_sim:
                self.sim.set_servos_mode(mode, names)
        self._event("info", f"servos {mode}: {', '.join(names) if len(names) < len(NAMES) else 'all'}")
        for w in warnings:
            self._event("warn", w)
        return {"ok": True, "mode": mode, "names": names, "warnings": warnings}

    # -- built-in actions ------------------------------------------------------------------------
    def _robot_motion_list(self):
        """[(name, layered, music)] from the robot (cached 60 s), or None when not connected / failing."""
        with self._lock:
            robot, cached = self.robot, self._robot_motions
        if robot is None:
            return None
        if cached and time.monotonic() - cached[0] < 60:
            return cached[1]
        res, err = self._call("motion list", robot.rest.motion_list)
        if err or not isinstance(res.get("data"), dict):
            return cached[1] if cached else None
        out = []
        for key, layered in (("system_hts_motions", False), ("system_layers_motions", True),
                             ("user_hts_motions", False), ("user_layers_motions", True)):
            for e in res["data"].get(key) or []:
                if isinstance(e, dict) and isinstance(e.get("name"), str):
                    out.append((os.path.splitext(e["name"])[0], layered, bool(e.get("music"))))
        with self._lock:
            self._robot_motions = (time.monotonic(), out)
        return out

    @staticmethod
    def _fallback_motion_list():
        try:
            with open(FALLBACK_MOTION_LIST, encoding="utf-8") as f:
                names = json.load(f)
        except (OSError, ValueError):
            return []
        k = names.index(FALLBACK_FIRST_LAYERED) if FALLBACK_FIRST_LAYERED in names else len(names)
        return [(n, i >= k, n in DANCES or not n.isascii()) for i, n in enumerate(names)]

    @staticmethod
    def _category(name, layered, music):
        n = name.lower().replace(" ", "")
        if layered:
            return "v2 layered"
        if music or name in DANCES or not name.isascii():
            return "dances"
        if n.startswith(("getup", "base_getup")):
            return "get-up"
        if n.startswith(("football", "goalkeeper", "shoot")) or "tackle" in n:
            return "football"
        if n.startswith(("fight", "knee")) or "punch" in n or "hit" in n:
            return "fight"
        if n.startswith(("forward", "backward", "leftward", "rightward", "turn", "onestep", "base_", "move",
                         "leftmove", "rightmove", "track")):
            return "locomotion"
        return "gestures"

    def list_actions(self):
        """[{name, label, directions, sim_directions, category, sim, robot, music, version, kind, source}].

        Parametric built-ins, then the robot's motion list (or the captured list as robot-only when not
        connected), then the user's motion files. System motions (Shutdown, calibration...) are hidden."""
        out = []
        for name, dirs, cat in PARAMETRIC:
            sim_dirs = [d for d in (dirs or ("",)) if sim_extras.has_builtin(name, d)]
            if name == "walk":
                sim_dirs = list(dirs)
            out.append({"name": name, "label": name.title(), "directions": list(dirs), "sim_directions": sim_dirs,
                        "category": cat, "sim": bool(sim_dirs), "robot": True, "music": False, "version": "v1",
                        "kind": "builtin", "source": "parametric"})
        live = self._robot_motion_list()
        entries, source = (live, "robot") if live is not None else (self._fallback_motion_list(), "fallback")
        seen = set(PARAM_BY_NAME)
        for name, layered, music in entries:
            low = name.lower()
            if low in seen or is_system_motion(name):
                continue
            seen.add(low)
            out.append({"name": name, "label": name.replace("_", " "), "directions": [], "sim_directions": [],
                        "category": self._category(name, layered, music), "sim": False, "robot": True,
                        "music": bool(music or name in DANCES), "version": "v2" if layered else "v1",
                        "kind": "builtin", "source": source})
        for m in self.motion_files():
            if "error" not in m:
                out.append({"name": m["name"], "label": m["name"].replace("_", " "), "directions": [],
                            "sim_directions": [""], "category": "user", "sim": True, "robot": True, "music": False,
                            "version": "v1", "kind": "file", "source": m["dir"]})
        return out

    def play_action(self, name, direction="", speed="normal", repeat=1, version="v1"):
        """Play a built-in (parametric, robot list or a user motion file) on the current targets."""
        if not isinstance(name, str) or not name.strip():
            return self._fail("play_action: give an action name", level="warn")
        name, direction = name.strip(), (direction or "").strip().lower()
        if is_system_motion(name):
            return self._fail(f"'{name}' is a robot system motion (power, calibration, burn-in): not available here",
                              level="warn")
        if speed not in SPEEDS:
            return self._fail(f"speed must be one of {SPEEDS}", level="warn")
        if not isinstance(repeat, int) or isinstance(repeat, bool) or not 1 <= repeat <= 100:
            return self._fail("repeat must be an integer 1-100", level="warn")
        if version not in ("v1", "v2"):
            return self._fail("version must be 'v1' or 'v2'", level="warn")
        param = PARAM_BY_NAME.get(name.lower())
        if param:
            name = param[0]
            if param[1] and direction not in param[1]:
                return self._fail(f"'{name}' needs a direction: one of {list(param[1])}", level="warn")
            if not param[1]:
                direction = ""
        elif NAME_RE.match(name) and self._motion_path(name) and not self._is_robot_motion(name):
            return self.play_motion(name, speed=SPEED_FACTOR[speed])
        gen, use_sim, use_robot, robot = self._snapshot()
        label = f"{name} {direction}".strip()
        sim_motion = sim_extras.load_builtin(name, direction) if param else None
        sim_walk = param is not None and name == "walk"
        if use_sim and not use_robot and sim_motion is None and not sim_walk:
            return self._fail(f"'{label}' is not available in simulation (connect a robot and use Robot or Both)",
                              level="warn")
        refusal = self._battery_refusal(use_sim and (sim_motion is not None or sim_walk), use_robot)
        if refusal:
            return self._fail(refusal, level="warn")
        if sim_motion is not None and repeat > 1:
            sim_motion = dict(sim_motion, frames=sim_motion["frames"] * repeat)
        if sim_motion is not None and use_sim:
            errors, _ = motion_mod.validate(sim_motion)
            if errors:
                return self._fail(f"sim version of '{label}' is broken: {errors[:3]}")
        target = self._target_name(use_sim, use_robot)
        with self._act_lock:
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"action '{label}' {why}", level="warn")
            total_s = None
            if use_robot:
                res, err = self._call_start(
                    f"action '{label}'", lambda: robot.play_builtin(name, direction, speed, repeat, version),
                    lambda: robot.rest.motion_control("stop", version=version))
                if err:
                    return self._fail(err)
                if not self._current(gen):
                    self._undo_robot(robot)
                    return self._fail(f"action '{label}' cancelled by STOP", level="warn")
                total_s = ((res.get("data") or {}).get("total_time") or 0) / 1000 or None
            twin = use_robot and use_sim and sim_motion is None
            if use_sim and not use_robot:
                total_s = motion_mod.duration_s(sim_motion, SPEED_FACTOR[speed]) if sim_motion else \
                    4 * repeat * sim_extras.SimGait.cycle_time(SPEED_PERIOD[speed]) / 2
            act = _Activity("action", label, target, duration=total_s, twin=twin,
                            detail="sim follows the robot" if twin else "")
            act.robot_name, act.version = name, version
            sim_job = None
            if use_sim and not twin:
                if sim_walk:
                    v, h = WALK_PARAMS[direction]
                    act.params = {"speed_v": v, "speed_h": h, "steps": 4 * repeat, "period": SPEED_PERIOD[speed]}
                    sim_job = lambda a: self._sim_gait(a)  # noqa: E731
                else:
                    sim_job = lambda a: self._sim_play(a, sim_motion, SPEED_FACTOR[speed])  # noqa: E731

            def run(a):
                if use_robot:
                    t = threading.Thread(target=sim_job, args=(a,), daemon=True) if sim_job else None
                    if t:
                        t.start()
                    self._wait_robot_motion(a, robot, version, total_s or 0)
                    if t:
                        t.join()
                elif sim_job:
                    sim_job(a)
            if self._launch(act, run, gen) is None:
                if use_robot:
                    self._undo_robot(robot)
                return self._fail(f"action '{label}' cancelled by STOP", level="warn")
        self._event("info", f"action {label} ({speed}, x{repeat}) on {target}")
        return {"ok": True, "target": target, "duration_s": total_s, "twin": twin}

    def _pause_resume(self, operation):
        act = self._activity
        if act is None or act.done.is_set() or act.kind != "action" or act.target == "sim":
            return self._fail(f"{operation}: no robot built-in action is playing (pause works for robot built-ins; "
                              "STOP ends anything)", level="warn")
        if act.target == "both" and not act.twin:
            return self._fail(f"{operation}: this action also plays its own sim version, which cannot pause; use STOP",
                              level="warn")
        with self._lock:
            robot = self.robot
        if robot is None:
            return self._fail(f"{operation}: no robot connected", level="warn")
        version = getattr(act, "version", "v1")
        res, err = self._call(f"{operation} '{act.name}'", lambda: robot.rest.motion_control(
            operation, act.robot_name, version=version))
        if err:
            return self._fail(err)
        act.detail = "paused" if operation == "pause" else ("sim follows the robot" if act.twin else "")
        self._event("info", f"{operation}d {act.name}")
        return {"ok": True, "name": act.name}

    def pause_action(self):
        """Pause the robot built-in that is playing (motion_control pause). Robot only; STOP still ends it."""
        return self._pause_resume("pause")

    def resume_action(self):
        """Resume a paused robot built-in."""
        return self._pause_resume("resume")

    def _is_robot_motion(self, name):
        live = self._robot_motion_list() or []
        return any(n.lower() == name.lower() for n, _, _ in live)

    # -- gait -------------------------------------------------------------------------------------
    def _sim_gait(self, act):
        gait = sim_extras.SimGait(self.sim)

        def text(p):
            return "sim: " + gait.describe(p.get("speed_v", 0), p.get("speed_h", 0), p.get("turn", 0))
        act.detail = text(act.params)

        def on_step(n):
            act.detail = text(act.params) + f" ({n} steps)"
        status = gait.run(act.params, act.stop, on_step)
        if status.startswith("stopped: torso"):
            self._event("warn", f"sim gait {status}")

    def gait(self, speed_v, speed_h, steps=0, period=1, wave=False, lease_s=None, turn=0, quick=False):
        """Walk (control_motion_gait). speed_v -5..5 (+ forward), speed_h -5..5 (+ left), steps 0 = until
        gait_stop(), period 1..5 (0 stops). Calling it while walking changes the speeds in place.
        turn -5..5 (+ anticlockwise) rotates in place, simulator only: the robot's gait cannot turn, it uses the
        built-in "turn around" motion instead (play_action). lease_s (dead-man, for steps=0): the walk stops
        unless gait() or gait_keepalive() is called again within lease_s seconds. None = no lease (the web
        server always sets one for steps=0)."""
        vals = (speed_v, speed_h, steps, period, turn)
        if not all(isinstance(v, int) and not isinstance(v, bool) for v in vals) or not (
                -5 <= speed_v <= 5 and -5 <= speed_h <= 5 and 0 <= steps <= 10000 and 0 <= period <= 5
                and -5 <= turn <= 5):
            return self._fail("gait: speed_v, speed_h and turn must be ints -5..5, steps >= 0, period 0..5",
                              level="warn")
        if lease_s is not None and (not _num(lease_s) or lease_s <= 0):
            return self._fail("gait: lease_s must be a number > 0 (or None)", level="warn")
        if period == 0:
            return self.gait_stop()
        wave = bool(wave)
        gen, use_sim, use_robot, robot = self._snapshot()
        if turn and use_robot:
            return self._fail("the robot's gait cannot turn in place: use the 'turn around' built-in "
                              "(play_action('turn around', 'left'|'right')); turning is simulator-only", level="warn")
        params = {"speed_v": speed_v, "speed_h": speed_h, "steps": steps, "period": period, "wave": wave,
                  "turn": turn, "quick": bool(quick)}
        target = self._target_name(use_sim, use_robot)
        lease = lease_s if not steps else None
        with self._act_lock:
            act = self._activity
            if act is not None and act.kind == "gait" and not act.done.is_set() and not act.stop.is_set() and \
                    act.target == target and gen is not None and act.gen == gen:
                # update in place: same targets and no STOP / mode switch since this walk started
                if use_robot:
                    res, err = self._call("gait", lambda: robot.rest.gait(speed_v, speed_h, steps, period, wave))
                    if err:
                        return self._fail(err)
                    if not self._current(gen) or act.stop.is_set():  # STOP or release while sending
                        self._call("gait exit", robot.rest.gait_exit)
                        return self._fail("gait update cancelled by STOP", level="warn")
                act.params.update(params)
                act.name = sim_extras.SimGait.describe(speed_v, speed_h, turn)
                act.lease_s = lease
                act.lease_until = None
                act.keepalive()
                return {"ok": True, "target": target, "updated": True}
            refusal = self._battery_refusal(use_sim and not use_robot, use_robot)
            if refusal:
                return self._fail(refusal, level="warn")
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"gait {why}", level="warn")
            if use_robot:
                res, err = self._call_start("gait", lambda: robot.rest.gait(speed_v, speed_h, steps, period, wave),
                                            robot.rest.gait_exit)
                if err:
                    return self._fail(err)
                if not self._current(gen):
                    self._undo_robot(robot)
                    return self._fail("gait cancelled by STOP", level="warn")
            duration = steps * sim_extras.SimGait.cycle_time(period) / 2 if steps else None
            act = _Activity("gait", sim_extras.SimGait.describe(speed_v, speed_h, turn), target, duration=duration,
                            twin=use_robot and use_sim, detail="sim follows the robot" if use_robot and use_sim else "")
            act.params, act.lease_s = params, lease

            def run(a):
                if use_robot:
                    self._wait_robot_gait(a, robot)
                else:
                    self._sim_gait(a)
            if self._launch(act, run, gen) is None:
                if use_robot:
                    self._undo_robot(robot)
                return self._fail("gait cancelled by STOP", level="warn")
        self._event("info", f"gait v={speed_v} h={speed_h}{f' turn={turn}' if turn else ''} "
                            f"steps={steps or 'until stopped'} on {target}"
                    + (f" (stops without a keepalive for {lease:g} s)" if lease else ""))
        return {"ok": True, "target": target, "updated": False}

    REFLEX_MARGIN_M = 0.015  # capture point this far outside the support circle while standing = about to tip

    def get_up(self):
        """Stand up after a fall. Robot (robot/both modes): the built-in GetupFront or GetupRear, chosen from the
        robot's own IMU (euler-x about 0 face down, about 180 on the back); the twin follows. Sim: the motion
        found by tools/tune_getup.py for that posture (motions/builtin/getup_front.json, getup_rear.json); when
        there is none, or the robot is still down 2 s after it, respawn() stands it up and an event says so."""
        gen, use_sim, use_robot, robot = self._snapshot()
        if use_robot:
            res, err = self._call("gyro", robot.rest.gyro)
            if err:
                return self._fail(f"get up: cannot read the robot's IMU ({err})")
            try:
                ex = float(((res.get("data") or {}).get("gyro") or [{}])[0].get("euler-x", 90))
            except (TypeError, ValueError):
                ex = 90.0
            return self.play_action("GetupFront" if abs(ex) < 90 else "GetupRear")
        if self.sim.fixed_base:
            return self._fail("get up: the simulated torso is held in the air (fixed base)", level="warn")
        if self.sim.torso_tilt_deg() < 30:
            return {"ok": True, "target": "sim", "standing": True}
        ex = self.sim.get_sensors_gyro()["data"]["gyro"][0]["euler-x"]
        posture = "front" if abs(ex) < 90 else "rear"
        mo = sim_extras.load_builtin("getup", posture)
        with self._act_lock:
            why = self._begin(gen, None)
            if why:
                return self._fail(f"get up {why}", level="warn")
            act = _Activity("action", f"get up ({posture})", "sim",
                            duration=(motion_mod.duration_s(mo) + 2.0) if mo else 1.0)

            def run(a):
                if mo is not None:
                    self._sim_play(a, mo, 1.0)
                    if not a.stop.is_set():
                        self.sim.wait(1.5)
                if a.stop.is_set():
                    return
                if self.sim.torso_tilt_deg() > 30:
                    self.sim.respawn()
                    self._event("warn", "the simulated robot could not get up by itself and was stood up"
                                + ("" if mo else f" (no getup_{posture} motion in motions/builtin)"))
                else:
                    self._event("info", f"the simulated robot got up from lying {'face down' if posture == 'front' else 'on its back'}")
            if self._launch(act, run, gen) is None:
                return self._fail("get up cancelled by STOP", level="warn")
        return {"ok": True, "target": "sim", "posture": posture, "motion": mo is not None}

    TIP_DIRECTIONS = {"front": (0.0, 1.0), "back": (0.0, -1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)}

    def tip_over(self, direction="front", force_n=9.0, seconds=0.5):
        """Simulator only: push the standing robot over gently (a small, sustained force at the head, so it folds
        rather than slams) to test fall detection and get_up(). Refused with a robot connected in robot/both mode."""
        gen, use_sim, use_robot, robot = self._snapshot()
        if use_robot:
            return self._fail("tip over is a simulator test: switch to Sim mode (the real robot is never pushed)", level="warn")
        if direction not in self.TIP_DIRECTIONS:
            return self._fail(f"tip over: direction must be one of {list(self.TIP_DIRECTIONS)}", level="warn")
        if self.sim.fixed_base:
            return self._fail("tip over: the torso is held in the air (fixed base)", level="warn")
        if self.sim.torso_tilt_deg() > 30:
            return {"ok": True, "target": "sim", "already_down": True}
        with self._act_lock:
            why = self._begin(gen, None)
            if why:
                return self._fail(f"tip over {why}", level="warn")
            act = _Activity("action", f"tip over ({direction})", "sim", duration=seconds + 2.0)
            dx, dy = self.TIP_DIRECTIONS[direction]

            def run(a):
                head = self.sim.model.body("NeckLR").id
                with self.sim._lock:
                    self.sim.data.xfrc_applied[head] = [force_n * dx, force_n * dy, 0, 0, 0, 0]
                try:
                    t_end = time.monotonic() + seconds
                    while time.monotonic() < t_end and not a.stop.is_set():
                        time.sleep(0.02)
                finally:
                    with self.sim._lock:
                        self.sim.data.xfrc_applied[head] = 0
                if not a.stop.is_set():
                    self.sim.wait(2.0)
                    tilt = self.sim.torso_tilt_deg()
                    self._event("info", f"tip over: torso at {tilt:.0f} deg" + (" (fallen: try Get up)" if tilt > 45 else " (it kept its feet)"))
            if self._launch(act, run, gen) is None:
                return self._fail("tip over cancelled by STOP", level="warn")
        self._event("info", f"tipping the simulated robot over {direction} ({force_n:g} N at the head for {seconds:g} s)")
        return {"ok": True, "target": "sim", "direction": direction}

    # keys in the MuJoCo viewer window (GLFW codes): WASD / arrows walk, Q / E turn, X / Esc STOP
    VIEWER_KEYS = {87: (1, 0, 0), 265: (1, 0, 0), 83: (-1, 0, 0), 264: (-1, 0, 0), 65: (0, 1, 0), 263: (0, 1, 0),
                   68: (0, -1, 0), 262: (0, -1, 0), 81: (0, 0, 1), 69: (0, 0, -1)}
    VIEWER_STOP_KEYS = (88, 256)
    VIEWER_SPEED = 3
    VIEWER_LEASE_S = 0.9  # a held key repeats every few tens of ms after about half a second: the walk ends this
    # long after the last repeat, so releasing the key stops it (the viewer reports no key releases)

    def _viewer_key(self, keycode):
        """Drive the walk from the MuJoCo viewer window (SimRobot.on_key). Called on the viewer's thread for every
        key press and repeat: a new direction starts a leased walk, a repeat renews the lease, X or Esc is STOP.
        In robot or both mode Q/E play the robot's turn-around built-in once per press."""
        if keycode in self.VIEWER_STOP_KEYS:
            with self._lock:
                stopping = self._stopping > 0
            if not stopping:  # a held key repeats every few tens of ms: one STOP is enough
                threading.Thread(target=self.stop_all, name="viewer-stop", daemon=True).start()
            return
        vec = self.VIEWER_KEYS.get(keycode)
        if vec is None:
            return
        v, h, t = (x * self.VIEWER_SPEED for x in vec)
        act = self._activity
        if t and self.mode != "sim":
            if act is not None and act.kind == "action" and not act.done.is_set() and "turn around" in act.name:
                return  # still turning: ignore the key repeats
            threading.Thread(target=self.play_action, args=("turn around", "left" if t > 0 else "right"),
                             name="viewer-turn", daemon=True).start()
            return
        wanted = {"speed_v": v, "speed_h": h, "turn": t}
        if act is not None and act.kind == "gait" and not act.done.is_set() and not act.stop.is_set() and \
                all(act.params.get(k, 0) == val for k, val in wanted.items()):
            act.keepalive()
            return
        threading.Thread(target=self.gait, args=(v, h), name="viewer-walk", daemon=True,
                         kwargs={"steps": 0, "period": 1, "lease_s": self.VIEWER_LEASE_S, "turn": t}).start()

    def gait_keepalive(self):
        """Refresh the dead-man lease of the running walk (see gait(lease_s=)). {"ok", "walking"}."""
        act = self._activity
        walking = act is not None and act.kind == "gait" and not act.done.is_set() and not act.stop.is_set()
        if walking:
            act.keepalive()
        return {"ok": True, "walking": walking}

    def _wait_robot_gait(self, act, robot):
        t0, seen = time.monotonic(), False
        last_ok = t0
        while not act.stop.wait(0.3):
            res, err = self._call("gait state", robot.rest.gait_state)
            if res is not None:
                last_ok = time.monotonic()
            elif time.monotonic() - last_ok > ROBOT_LOST_S:
                self._call("gait exit", robot.rest.gait_exit)
                raise RuntimeError(f"robot stopped answering ({err})")
            if err:
                continue
            status = (res.get("data") or {}).get("status")
            if status in (0, 1, 2):
                seen = True
            elif status in (3, 5, 7, 8) and (seen or time.monotonic() - t0 > 1.5):
                break
        p = act.params
        if act.stop.is_set() and not act.hard and (p.get("speed_v") or p.get("speed_h")):
            # a normal stop (release, gait_stop, lost lease): fade the speeds out first, an exit at speed can tip the robot
            self._call("gait stop", lambda: robot.rest.gait_decelerate(p.get("speed_v", 0), p.get("speed_h", 0),
                                                                        p.get("period", 1), p.get("wave", False)))
        else:
            self._call("gait exit", robot.rest.gait_exit)

    def gait_stop(self):
        """Stop walking gently: the sim fades its stride out (SimGait.STOP_S), a robot walk steps its speeds down over
        about a second before exit_motion_gait (rest.gait_decelerate), since an exit at speed can tip the robot.
        STOP (stop_all) exits at once instead. A connected robot gets gait_exit whatever the mode (a walk may have
        started just before a mode switch)."""
        act = self._activity
        running = act is not None and act.kind == "gait" and not act.done.is_set()
        if running:
            act.stop.set()
        with self._lock:
            robot = self.robot
        if running and robot is not None and act.target != "sim":
            act.done.wait(6.0)  # the activity thread decelerates and sends gait_exit
            return {"ok": True, "decelerated": True}
        if robot is not None:
            res, err = self._call("gait exit", robot.rest.gait_exit)
            if err:
                return self._fail(err)
        if running:
            act.done.wait(5.0)
        return {"ok": True}

    # -- music --------------------------------------------------------------------------------------
    def music_list(self):
        """{"ok", "music": [names], "source": "robot"|"sim"}. The sim has no audio: it lists the dance songs."""
        _, use_robot, robot = self._targets()
        if robot is not None:
            res, err = self._call("music list", robot.rest.music_list)
            if not err:
                data = res.get("data") or {}
                items = data.get("music") if isinstance(data, dict) else data
                names = [m["name"] if isinstance(m, dict) else str(m) for m in items or []]
                return {"ok": True, "music": names, "source": "robot"}
            self._event("warn", err)
        return {"ok": True, "music": sorted(DANCES), "source": "sim"}

    def music_play(self, name):
        if not isinstance(name, str) or not name.strip():
            return self._fail("music_play: give a song name", level="warn")
        use_sim, use_robot, robot = self._targets()
        target = self._target_name(use_sim, use_robot)
        if use_robot:
            res, err = self._call(f"music '{name}'", lambda: robot.rest.music_play(name))
            if err:
                return self._fail(err)
        with self._lock:
            self._music = {"name": name, "target": target, "started": round(time.time(), 3)}
        self._event("info", f"music {name} on {target}" + ("" if use_robot else " (no audio in the simulator)"))
        return {"ok": True, "target": target, "simulated": not use_robot}

    def music_stop(self):
        _, use_robot, robot = self._targets()
        with self._lock:
            self._music = None
        if use_robot:
            res, err = self._call("music stop", robot.rest.music_stop)
            if err:
                return self._fail(err)
        return {"ok": True}

    # -- motion files --------------------------------------------------------------------------------
    def _motion_path(self, name):
        """Existing file for a motion name (user folder, then recorded/; NAME.json, else the robot's NAME.hts), or None."""
        if not isinstance(name, str) or not NAME_RE.match(name):
            return None
        for d in (self.motions_dir, os.path.join(self.motions_dir, "recorded")):
            for ext in (".json", ".hts"):
                p = os.path.join(d, name + ext)
                if os.path.isfile(p):
                    return p
        return None

    def motion_files(self):
        """[{name, dir: "user"|"recorded", format: "json"|"hts", description, frames, duration_s}] (or {name, dir,
        error}). NAME.json and the robot's NAME.hts are both listed; a .hts next to a .json of the same name is skipped."""
        out = []
        for d, kind in ((self.motions_dir, "user"), (os.path.join(self.motions_dir, "recorded"), "recorded")):
            try:
                files = sorted(os.listdir(d))
            except OSError:
                continue
            for fn in files:
                stem, ext = os.path.splitext(fn)
                if ext not in (".json", ".hts") or not NAME_RE.match(stem) or (ext == ".hts" and stem + ".json" in files):
                    continue
                try:
                    mo = motion_mod.load(os.path.join(d, fn))
                    out.append({"name": stem, "dir": kind, "format": ext[1:], "description": mo.get("description", ""),
                                "frames": len(motion_mod._frames(mo)), "duration_s": round(motion_mod.duration_s(mo), 2)})
                except (OSError, ValueError, AttributeError) as e:
                    out.append({"name": stem, "dir": kind, "error": f"unreadable: {e}"})
        return out

    def load_motion(self, name):
        path = self._motion_path(name)
        if path is None:
            return self._fail(f"no motion file named {name!r}", level="warn")
        try:
            mo = motion_mod.load(path)
        except (OSError, ValueError) as e:
            return self._fail(f"cannot read {name}: {e}")
        if isinstance(mo, dict):
            mo["name"] = name
        return {"ok": True, "motion": mo}

    def save_motion(self, name, motion):
        """Write motions/NAME.json (names [A-Za-z0-9_-]+). Refuses malformed motions; returns the check report."""
        if not isinstance(name, str) or not NAME_RE.match(name):
            return self._fail("motion names may only use letters, digits, '_' and '-'", level="warn")
        if not isinstance(motion, dict) or not isinstance(motion.get("frames"), list):
            return self._fail("a motion is an object with a 'frames' list", level="warn")
        errors, warnings = motion_mod.validate(motion)
        if any("is not a number" in e or "must be an object" in e or "must be a list" in e for e in errors):
            return self._fail(f"cannot save {name}: {errors[:3]}", level="warn", errors=errors)
        mo = dict(motion, name=name)
        mo.setdefault("end_in_reset", True)
        path = os.path.join(self.motions_dir, name + ".json")
        try:
            os.makedirs(self.motions_dir, exist_ok=True)
            tmp = path + ".tmp"
            motion_mod.save(mo, tmp)
            os.replace(tmp, path)
        except OSError as e:
            return self._fail(f"cannot save {name}: {e}")
        self._event("info", f"saved motion {name}")
        return {"ok": True, "name": name, "errors": errors, "warnings": warnings}

    def delete_motion(self, name):
        path = self._motion_path(name)
        if path is None:
            return self._fail(f"no motion file named {name!r}", level="warn")
        try:
            os.remove(path)
        except OSError as e:
            return self._fail(f"cannot delete {name}: {e}")
        self._event("info", f"deleted motion {name}")
        return {"ok": True}

    def _resolve_motion(self, name_or_dict):
        if isinstance(name_or_dict, dict):
            return dict(name_or_dict), name_or_dict.get("name") or "unsaved motion"
        res = self.load_motion(name_or_dict)
        return (res["motion"], name_or_dict) if res["ok"] else (None, res["error"])

    def _speed_cap_plan(self, motion, start, speed):
        """Robot playback under the speed cap: (warnings, extra seconds) for the frames _Fanout will stretch."""
        pose, notes, extra = dict(start), [], 0.0
        moves = [(f"frame {i + 1}", motion_mod._angles(fr), motion_mod._times(fr)[0])
                 for i, fr in enumerate(motion_mod._frames(motion))]
        if moves and isinstance(motion, dict) and motion.get("end_in_reset"):
            moves.append(("return to reset", dict(RESET_POSE), motion_mod.RESET_RUNTIME_MS))
        for label, angles, runtime in moves:
            if not angles:
                continue
            eff = motion_mod.effective_runtime_ms(runtime, speed)
            dist = max(abs(d - pose.get(n, d)) for n, d in angles.items())
            need = min(max(math.ceil(dist / self.max_speed * 1000), MIN_RUNTIME_MS), MAX_RUNTIME_MS)
            if need > eff:
                notes.append(f"{label}: runtime {eff:.0f} ms stretched to {need} ms on the robot "
                             f"({dist:.0f} deg at the {self.max_speed:g} deg/s speed cap)")
                extra += (need - eff) / 1000
            pose.update(angles)
        return notes, extra

    def check_motion(self, motion, speed=1.0):
        """{"ok", "safe", "errors", "warnings", "collisions", "duration_s", "start"} starting from the current pose.
        collisions[].frame is 0-based (len(frames) = the end_in_reset return move) and collisions[].label names it
        for people ("frame 1", "return to reset"); "at" is the fraction of that move. With
        the robot as a target, frames faster than the speed cap are reported (they are slowed down on the robot)."""
        if isinstance(motion, str):
            motion, err = self._resolve_motion(motion)
            if motion is None:
                return {"ok": False, "error": err}
        _, use_robot, _ = self._targets()
        start = self._current_pose()
        errors, warnings = motion_mod.validate(motion, start_pose=start, speed=speed)
        cols = []
        if isinstance(motion, dict) and isinstance(motion.get("frames"), list):
            cols = [{"frame": f, "label": self._frame_label(motion, f), "at": s, "bodies": list(p)}
                    for f, s, p in motion_mod.self_collisions(motion, start_pose=start, checker=self.checker)]
        duration = motion_mod.duration_s(motion, speed)
        if use_robot and not errors and _num(speed) and speed > 0:
            notes, extra = self._speed_cap_plan(motion, start, speed)
            warnings += notes
            duration += extra
        return {"ok": True, "safe": not errors and not cols, "errors": errors, "warnings": warnings,
                "collisions": cols, "duration_s": round(duration, 2), "start": "robot" if use_robot else "sim"}

    @staticmethod
    def _frame_label(motion, i):
        return "return to reset" if i >= len(motion_mod._frames(motion)) else f"frame {i + 1}"

    def play_motion(self, name_or_dict, speed=1.0):
        """Play a motion file (name) or dict on the current targets, in the activity thread (cancellable).
        Robot-bound playback is refused on validation errors or self-collisions from the current pose, and frames
        are slowed to the robot speed cap."""
        mo, label = self._resolve_motion(name_or_dict)
        if mo is None:
            return {"ok": False, "error": label}
        if not _num(speed) or speed <= 0:
            return self._fail(f"speed must be a number > 0, not {speed!r}", level="warn")
        gen, use_sim, use_robot, robot = self._snapshot()
        report = self.check_motion(mo, speed)
        if report["errors"]:
            return self._fail(f"refusing to play '{label}': {'; '.join(report['errors'][:3])}", level="warn",
                              **{k: report[k] for k in ("errors", "warnings", "collisions")})
        warnings = list(report["warnings"])
        if report["collisions"]:
            text = "; ".join(f"{self._frame_label(mo, c['frame'])}: {c['bodies'][0]} hits {c['bodies'][1]}"
                             for c in report["collisions"][:3])
            if use_robot:
                return self._fail(f"refusing to send '{label}' to the robot: self-collision ({text})", level="warn",
                                  **{k: report[k] for k in ("errors", "warnings", "collisions")})
            warnings.append(f"self-collision in the sim: {text}")
        refusal = self._battery_refusal(use_sim, use_robot)
        if refusal:
            return self._fail(refusal, level="warn")
        target = self._target_name(use_sim, use_robot)
        with self._act_lock:
            why = self._begin(gen, robot if use_robot else None)
            if why:
                return self._fail(f"motion '{label}' {why}", level="warn")
            if use_robot:
                err = self._resync_gate(robot)
                if err == "robot disconnected":
                    return self._fail(f"'{label}' not played: robot disconnected", level="warn")
            act = _Activity("motion", label, target, duration=report["duration_s"])
            out = _Fanout(self, self.sim if use_sim else None, robot if use_robot else None, stop=act.stop)

            def run(a):
                if not motion_mod.play(out, mo, speed=speed, stop_event=a.stop):
                    self.sim.stop_all()
            if self._launch(act, run, gen) is None:
                return self._fail(f"motion '{label}' cancelled by STOP", level="warn")
        for w in warnings[:3]:
            self._event("warn", w)
        if len(warnings) > 3:
            self._event("warn", f"... and {len(warnings) - 3} more warnings (see check_motion)")
        self._event("info", f"motion {label} (speed {speed:g}) on {target}")
        return {"ok": True, "target": target, "duration_s": report["duration_s"], "warnings": warnings}

    def record_frame(self, runtime=500, hold=0):
        """PRP: capture the current angles as a keyframe (robot readback in robot/both, sim otherwise)."""
        if not _num(runtime) or not MIN_RUNTIME_MS <= runtime <= MAX_RUNTIME_MS:
            return self._fail(f"runtime must be {MIN_RUNTIME_MS}-{MAX_RUNTIME_MS} ms", level="warn")
        if not _num(hold) or hold < 0:
            return self._fail("hold must be a number >= 0", level="warn")
        _, use_robot, _ = self._targets()
        if use_robot:
            angles = self._robot_angles(fresh=True)
            if angles is None:
                return self._fail("cannot read the robot's servos")
        else:
            angles = self._sim_angles()
        frame = {"angles": {n: int(round(angles[n])) for n in NAMES if n in angles}, "runtime": int(runtime)}
        if hold:
            frame["hold"] = int(hold)
        return {"ok": True, "frame": frame, "source": "robot" if use_robot else "sim"}

    # -- device settings ---------------------------------------------------------------------------------
    def _setting(self, what, robot_fn, apply_local):
        _, use_robot, robot = self._targets()
        if use_robot:
            res, err = self._call(what, lambda: robot_fn(robot))
            if err:
                return self._fail(err)
        with self._lock:
            apply_local()
        self._event("info", what)
        return {"ok": True}

    def set_led(self, type, color, mode):
        """Chest ("button") or eye ("camera") LED. The sim always shows it (it is the twin's display too)."""
        if type not in LED_COLORS or color not in LED_COLORS[type] or mode not in LED_MODES[type]:
            return self._fail(f"bad LED setting {type}/{color}/{mode}: {type} colours {LED_COLORS.get(type)}, "
                              f"modes {LED_MODES.get(type)}", level="warn")

        def local():
            self._leds[type] = {"color": color, "mode": mode}
            self.sim.set_led(type, color, mode)
        return self._setting(f"led {type} {color} {mode}", lambda r: r.rest.set_led(type, color, mode), local)

    def set_volume(self, volume):
        if not isinstance(volume, int) or isinstance(volume, bool) or not 0 <= volume <= 100:
            return self._fail("volume must be an integer 0-100", level="warn")
        return self._setting(f"volume {volume}", lambda r: r.rest.set_volume(volume),
                             lambda: self._settings.update(volume=volume))

    def set_language(self, language):
        if language not in ("zh", "en"):
            return self._fail("language must be 'zh' or 'en'", level="warn")
        return self._setting(f"language {language}", lambda r: r.rest.set_language(language),
                             lambda: self._settings.update(language=language))

    def set_fall_management(self, enable):
        enable = bool(enable)
        return self._setting(f"fall management {'on' if enable else 'off'}",
                             lambda r: r.rest.set_fall_management(enable),
                             lambda: self._settings.update(fall_management=enable))

    def charge(self):
        """Refill the simulated battery."""
        self.battery.charge()
        self._event("info", "sim battery charged")
        return {"ok": True, "battery": 100.0}

    # -- info and sensors ----------------------------------------------------------------------------------
    def info(self):
        """Battery, versions and flags per target."""
        import mujoco
        out = {"ok": True, "mode": self.mode,
               "sim": {"battery": round(self.battery.percent, 1), "charging": False, "fixed_base": self.sim.fixed_base,
                       "versions": {"core": f"yanshee_sim (MuJoCo {mujoco.__version__})", "servo": "simulated",
                                    "sn": "SIM"}, "tilt_deg": round(self._tilt, 1), "fallen": self._fallen},
               "robot": None}
        with self._lock:
            robot, info = self.robot, dict(self._robot_info)
        if robot is not None:
            r = {"ip": info["ip"], "online": info["online"], "versions": info["versions"]}
            res, err = self._call("battery", robot.battery_info)
            r.update(res if not err else {"battery_error": err})
            res, err = self._call("mode", robot.rest.mode)
            r["flags"] = res.get("data") if not err else {"error": err}
            out["robot"] = r
        return out

    def sensors(self):
        """IMU per target (euler in degrees) plus the robot's add-on sensors."""
        sim = {"gyro": self.sim.get_sensors_gyro()["data"]["gyro"][0]}
        try:
            with self.sim._lock:
                sim["distance_m"] = {s: round(float(self.sim.data.sensor(f"distance_{s}").data[0]), 3)
                                     for s in ("left", "right")}
        except Exception:
            pass
        out = {"ok": True, "sim": sim, "robot": "not connected"}
        with self._lock:
            robot = self.robot
        if robot is not None:
            r = {}
            res, err = self._call("gyro", robot.rest.gyro)
            r["gyro"] = ((res.get("data") or {}).get("gyro") or [{}])[0] if not err else {"error": err}
            res, err = self._call("sensor list", robot.rest.sensors_list)
            addons = {}
            if not err:
                for s in (res.get("data") or {}).get("sensors") or []:
                    kind = s.get("type") if isinstance(s, dict) else None
                    fn = getattr(robot.rest, kind, None) if kind in (
                        "infrared", "ultrasonic", "touch", "pressure", "environment") else None
                    if fn is None:
                        continue
                    rr, e = self._call(kind, fn)
                    addons[kind] = ((rr.get("data") or {}).get(kind) if not e else {"error": e})
            else:
                addons = {"error": err}
            r["addons"] = addons
            out["robot"] = r
        return out

    # -- voice and vision ------------------------------------------------------------------------------------
    def tts(self, text):
        """Robot: the robot speaks. Sim: returns {"speak_in_browser": text} for the GUI's speechSynthesis."""
        if not isinstance(text, str) or not text.strip():
            return self._fail("tts: give some text", level="warn")
        text = text.strip()[:500]
        _, use_robot, robot = self._targets()
        if not use_robot:
            self._event("info", f"say (browser): {text}")
            return {"ok": True, "target": "sim", "speak_in_browser": text}
        res, err = self._call("tts", lambda: robot.rest.tts(text))
        if err:
            return self._fail(err)
        self._event("info", f"say (robot): {text}")
        return {"ok": True, "target": "robot"}

    def tts_stop(self):
        """Stop speaking: robot TTS (stop_voice_tts) when a robot is connected. Nothing else stops (STOP does)."""
        with self._lock:
            robot = self.robot
        if robot is None:
            return {"ok": True, "target": "sim"}
        res, err = self._call("tts stop", robot.rest.tts_stop)
        if err:
            return self._fail(err)
        return {"ok": True, "target": "robot"}

    def vision(self, task, timeout=8):
        """Run a vision task (VISION_TASKS). Robot: the real one; sim: 'color' from the head camera render."""
        if task not in VISION_TASKS:
            return self._fail(f"unknown vision task {task!r}; use one of {sorted(VISION_TASKS)}", level="warn")
        if not _num(timeout) or timeout <= 0:
            return self._fail("timeout must be a number > 0", level="warn")
        timeout = min(float(timeout), 30.0)
        _, use_robot, robot = self._targets()
        target = "robot" if use_robot else "sim"
        with self._lock:
            self._side = {"kind": "vision", "name": task, "target": target, "started": round(time.time(), 3),
                          "detail": ""}
        try:
            if use_robot:
                option, kind = VISION_TASKS[task]
                fn = (lambda: robot.rest.qr(timeout)) if option == "qr" else \
                    (lambda: robot.rest.vision(option, kind, timeout))
                res, err = self._call(f"vision {task}", fn)
                if err:
                    return self._fail(err, level="warn", task=task, target=target,
                                      data=(res or {}).get("data"))
                return {"ok": True, "task": task, "target": target, "data": res.get("data")}
            if task != "color":
                return self._fail(f"'{task}' is not available in simulation (only 'color'); connect a robot",
                                  level="warn", task=task, target=target)
            rgb = self.renderer.latest_rgb("head_camera", timeout=3.0, max_age=0.5) if self.renderer else None
            if rgb is None:
                return self._fail("no head-camera image from the simulator "
                                  f"({self.renderer.error if self.renderer else 'renderer disabled'})")
            data = sim_extras.detect_colours(rgb)
            self._event("info", f"vision color (sim): {[c['name'] for c in data['color']] or 'nothing'}")
            return {"ok": True, "task": task, "target": target, "data": data}
        finally:
            with self._lock:
                self._side = None

    def face_entry(self, name):
        """Enrol the face in front of the robot's camera under name (robot only)."""
        if not isinstance(name, str) or not name.strip():
            return self._fail("face_entry: give a name", level="warn")
        _, use_robot, robot = self._targets()
        if not use_robot:
            return self._fail("face entry needs the robot (not available in simulation)", level="warn")
        res, err = self._call("face entry", lambda: robot.rest.face_entry(name.strip()))
        if err:
            return self._fail(err)
        self._event("info", f"face entered: {name.strip()}")
        return {"ok": True}

    def photo(self, resolution="640x480"):
        """JPEG bytes from the robot's camera (robot/both, at resolution: PHOTO_RESOLUTIONS) or the sim head camera
        (its render size); {"ok": False, "error"} on failure."""
        if resolution not in PHOTO_RESOLUTIONS:
            return self._fail(f"photo resolution must be one of {PHOTO_RESOLUTIONS}", level="warn")
        _, use_robot, robot = self._targets()
        if use_robot:
            res, err = self._call("photo", lambda: robot.rest.photo(resolution))
            return res if not err else self._fail(err)
        jpeg = self.renderer.latest("head_camera", timeout=3.0, max_age=0.5) if self.renderer else None
        if jpeg is None:
            return self._fail("no head-camera image from the simulator "
                              f"({self.renderer.error if self.renderer else 'renderer disabled'})")
        return jpeg

    def open_robot_stream(self, resolution="640x480"):
        """Start the robot's MJPEG stream; returns {"ok", "url"} (http://IP:8000/stream.mjpg)."""
        with self._lock:
            robot = self.robot
        if robot is None:
            return self._fail("no robot connected", level="warn")
        try:
            res = robot.rest.stream_open(resolution)
        except Exception as e:
            return self._fail(f"stream: {e}")
        if res.get("code") not in (0, 20003):  # 20003: already open
            return self._fail(f"stream: robot answered code {res.get('code')} {res.get('msg', '')}")
        return {"ok": True, "url": robot.rest.stream_url}

    # -- shutdown ------------------------------------------------------------------------------------------
    def close(self):
        """Stop everything and release the simulator, renderer and robot."""
        if not self._running:
            return
        try:
            self.stop_all()
            self.disconnect()
        finally:
            self._running = False
            if self.renderer is not None:
                self.renderer.close()
            self._monitor.join(timeout=1)
            self.sim.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


__all__ = ["Controller", "ModeError", "RobotGate", "MODES", "PARAMETRIC", "VISION_TASKS", "SPEED_FACTOR",
           "GAIT_LEASE_S", "PHOTO_RESOLUTIONS"]
