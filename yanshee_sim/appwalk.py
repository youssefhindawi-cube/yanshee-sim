"""AppWalk in the simulator: UBTECH's firmware walk played back from the robot's own servo frames.

The robot walks with its firmware gait (gait_node), driven the way the mobile app drives it (AppWalk, controller.py).
That walk is open-loop: the same command sends the same frames every time (29 Oct 2026), so the simulator walks the
robot's way by sending them itself. model/appwalk.json (tools/extract_appwalk.py, from the robot's logs) holds per
direction: the start (20 ms frames), the cycle that repeats while walking, the wind-downs the firmware makes after a
stop (keyed by the point of the cycle where they begin, at least min_lag frames after the stop command) and the five
100 ms frames that stand it up; plus TurnLeft / TurnRight (looped while turning) and ResetSwitch (played on release),
the motions the app's turns play. A direction the robot has not been recorded walking is refused (unsupported()).

Not reproduced: the start the firmware makes from an unusual pose (after a turn or a get-up it starts from where the
servos are); the walking speed (the frames are the recorded speed whatever is asked); arm swing off (arms left alone).
"""
import json
import os
import threading

from .balance import Balance, lean_from_imu
from .robot_stream import KEEP, to_degrees
from .servos import NAMES

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(ROOT, "model", "appwalk.json")
ARMS = range(0, 6)  # servo order: right shoulder roll/flex, elbow, then the left arm
TURN_NAMES = {1: "TurnLeft", -1: "TurnRight"}
_cache, _cache_lock = {}, threading.Lock()


def load(path=TABLE):
    """The recorded walk (model/appwalk.json), read once; {} if it is missing."""
    with _cache_lock:
        if path not in _cache:
            try:
                with open(path) as f:
                    _cache[path] = json.load(f)
            except (OSError, ValueError):
                _cache[path] = {}
        return _cache[path]


def direction(v, h):
    if v and h:
        return ("forward" if v > 0 else "backward") + "_" + ("left" if h > 0 else "right")
    return "forward" if v > 0 else "backward" if v < 0 else "left" if h > 0 else "right" if h < 0 else None


class AppWalkTable:
    """One direction of the recorded walk: frame(k) is the k-th 20 ms frame of a walk that has not been stopped."""

    def __init__(self, t):
        self.start, self.cycle, self.stand = t["start"], t["cycle"], t["stand"]
        self.winds = {int(k): v for k, v in t["winddowns"].items()}
        self.lag, self.P, self.s = int(t["min_lag"]), len(t["cycle"]), len(t["start"])

    def frame(self, k):
        return self.start[k] if k < self.s else self.cycle[(k - self.s) % self.P]

    def stop_at(self, k):
        """(b, frames) for a stop at frame k: the walk goes on to frame b, then plays frames (the wind-down)."""
        b0 = max(k + self.lag, self.s)
        for b in range(b0, b0 + self.P):
            if (b - self.s) % self.P in self.winds:
                return b, self.winds[(b - self.s) % self.P]
        return b0, []

    def frames_for(self, n):
        """20 ms frames of a walk stopped at frame n, wind-down included (not the stand frames)."""
        b, wd = self.stop_at(n)
        return [self.frame(k) for k in range(b)] + list(wd)


class AppWalkSim:
    """Walks the simulator with the recorded firmware walk; run() has SimGait.run()'s interface."""

    FRAME_S = 0.02
    STAND_S = 0.1

    def __init__(self, sim, max_tilt_deg=25.0, table=None, balance=False):
        """balance: True (or a balance.Balance) adds IMU ankle corrections from the simulator's emulated IMU (19 per
        second, with noise, as the robot's) on top of the recorded frames, every FRAME_S."""
        self.sim, self.max_tilt = sim, max_tilt_deg
        self.data = table if table is not None else load()
        self.steps_done, self.status = 0, "idle"
        self.balance = Balance() if balance is True else balance or None

    def _assist(self):
        if self.balance is None:
            return
        from .robot import IMU_UPRIGHT_EULER_X
        g = self.sim.get_sensors_gyro()["data"]["gyro"][0]
        lean = lean_from_imu((g["euler-x"], g["euler-y"]), (g["gyro-x"], g["gyro-y"], g["gyro-z"]), IMU_UPRIGHT_EULER_X)
        self.sim.set_offsets(self.balance.update(*lean, t=self.sim.data.time))

    def _wait(self, seconds):
        """sim.wait, with the balance corrections updated every FRAME_S."""
        if self.balance is None:
            self.sim.wait(seconds)
            return
        end = self.sim.data.time + seconds
        while self.sim.data.time < end - 1e-6:
            self._assist()
            self.sim.wait(min(self.FRAME_S, end - self.sim.data.time))

    def _settle_balance(self):
        """The movement is over: fade the corrections out."""
        if self.balance is None:
            return
        for _ in range(40):
            offs = self.balance.fade(self.sim.data.time)
            self.sim.set_offsets(offs)
            if not any(offs.values()):
                break
            self.sim.wait(self.FRAME_S)
        self.balance.reset(self.balance.ref)  # keep the standing reference

    @staticmethod
    def describe(speed_v, speed_h, turn=0):
        from .sim_extras import SimGait
        return SimGait.describe(speed_v, speed_h, turn if not (speed_v or speed_h) else 0)

    @classmethod
    def cycle_time(cls, period=1):
        """Seconds per cycle (two steps) of the recorded forward walk (the firmware's, whatever the period)."""
        t = (load().get("walks") or {}).get("forward")
        return len(t["cycle"]) * cls.FRAME_S if t else 0.4

    @classmethod
    def unsupported(cls, speed_v, speed_h, turn=0, table=None):
        """Why the simulator cannot walk this command the robot's way (not recorded yet), or None."""
        data = table if table is not None else load()
        if not data:
            return "the simulator has no recording of the robot's walk (model/appwalk.json: tools/extract_appwalk.py)"
        d = direction(speed_v, speed_h)
        if d and d not in data.get("walks", {}):
            have = ", ".join(sorted(data.get("walks", {}))) or "none"
            return (f"the robot has not been recorded walking {d.replace('_', ' and ')} yet (recorded: {have}); walk "
                    f"it with the robot connected, then run tools/extract_appwalk.py")
        if not d and turn and TURN_NAMES[1 if turn > 0 else -1] not in data.get("turns", {}):
            return "the simulator has no copy of the robot's turn motions (tools/extract_appwalk.py)"
        return None

    def _tilted(self):
        return self.sim.torso_tilt_deg() > self.max_tilt

    def _send(self, units, ms, wave=True):
        deg = {n: v for n, v in to_degrees([KEEP if (not wave and i in ARMS) else u for i, u in enumerate(units)]).items()
               if n != "NeckLR"}  # hal ignores the head in the firmware walk's frames
        self.sim._send(deg, ms)
        self._wait(ms / 1000.0)

    def _turn(self, params, stop_event, name, on_step=None):
        """TurnLeft / TurnRight looped while the command lasts (or for params["steps"] steps of half a cycle), then
        ResetSwitch, as hal plays them for the app."""
        turns, step_s, t0 = self.data["turns"], self.cycle_time() / 2, self.sim.data.time

        def going():
            steps = dict(params).get("steps") or 0
            return not stop_event.is_set() and self._key(params) == ("turn", name) and not (
                steps and self.steps_done >= steps)
        while going():
            for f in turns[name]:
                if not going() or self._tilted():
                    break
                self.sim._send({n: d for n, d in f["angles"].items() if n in NAMES}, f["runtime"])
                self._wait((f["runtime"] + f.get("hold", 0)) / 1000.0)
                while self.sim.data.time - t0 >= (self.steps_done + 1) * step_s:
                    self.steps_done += 1
                    if on_step:
                        on_step(self.steps_done)
            if self._tilted():
                return False
        for f in turns.get("ResetSwitch", []):
            self.sim._send({n: d for n, d in f["angles"].items() if n in NAMES}, f["runtime"])
            self._wait((f["runtime"] + f.get("hold", 0)) / 1000.0)
        self._settle_balance()
        return not self._tilted()

    @staticmethod
    def _key(params):
        p = dict(params)
        v, h, t = p.get("speed_v", 0), p.get("speed_h", 0), p.get("turn", 0) or 0
        d = direction(v, h)
        if d:
            return ("walk", d)
        if t:
            return ("turn", TURN_NAMES[1 if t > 0 else -1])
        return ("idle", None)

    def run(self, params, stop_event, on_step=None):
        """Walk until params["steps"] steps are done (0 = until stop_event); params may change while walking (a new
        direction stops the walk the firmware's way and starts the new one). Returns a status text."""
        self.steps_done, self.status = 0, "walking"
        tilted = f"stopped: torso tilted over {self.max_tilt:g} deg"
        status = "stopped"
        while not stop_event.is_set():
            kind, name = self._key(params)
            if kind == "idle":
                if params.get("steps"):
                    status = "done"
                    break
                self.sim.wait(0.05)  # hold-to-walk with nothing held: stand
                continue
            if kind == "turn":
                if not self._turn(params, stop_event, name, on_step):
                    status = tilted
                    break
                if params.get("steps") and self.steps_done >= params["steps"]:
                    status = "done"
                    break
                continue
            t = AppWalkTable(self.data["walks"][name])
            self.sim.wait(self.data["walks"][name].get("start_delay_s", 0.3))  # the firmware takes this long to start
            k, half, why = 0, t.P // 2, None
            while why is None:
                p = dict(params)
                if self._tilted():
                    why = "tilted"
                elif stop_event.is_set():
                    why = "stopped"
                elif p.get("steps") and self.steps_done >= p["steps"]:
                    why = "done"
                elif self._key(p) != (kind, name):
                    why = "changed"
                if why:
                    break
                self._send(t.frame(k), 20, bool(p.get("wave", True)))
                k += 1
                if k >= t.s and (k - t.s) % half == 0:
                    self.steps_done += 1
                    if on_step:
                        on_step(self.steps_done)
            if why == "tilted":
                status = tilted
                break
            b, wd = t.stop_at(k)  # the firmware finishes its step, winds down and stands up
            wave = bool(params.get("wave", True))
            for f in [t.frame(j) for j in range(k, b)] + list(wd):
                self._send(f, 20, wave)
            for f in t.stand:
                self._send(f, 100, wave)
            self._settle_balance()
            if self._tilted():
                status = tilted
                break
            status = "done" if why == "done" else "stopped"
            if why in ("done", "stopped"):
                break
        if self.balance is not None:
            self.sim.set_offsets({})
            self.balance.reset()
        self.status = status
        return status
