"""Live mirroring between the simulator and the real robot.

SimToRobot: you move the simulated robot (viewer sliders or code) and the real robot follows.
RobotToSim: the real robot moves (built-in motion, app, by hand) and the simulator shows it.

Every pose sent to the robot is checked first: servo limits, a speed cap and self-collision.
"""
import logging
import math
import time

from .motion import CollisionChecker
from .servos import LIMITS, MAX_RUNTIME_MS, NAMES, RESET_POSE, clamp_angles

log = logging.getLogger(__name__)


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _readings(res):
    """{servo: deg} from a get_servos_angles reply: known servos, numbers only, clamped to the limits."""
    if not isinstance(res, dict) or res.get("code", 0) != 0 or not isinstance(res.get("data"), dict):
        raise RuntimeError(f"servo read failed: {res}")
    return clamp_angles({n: d for n, d in res["data"].items() if n in LIMITS and _num(d)})


class DryRunRobot:
    """Stands in for the real robot: prints commands instead of sending them."""

    def set_servos_angles(self, angles, runtime=200):
        print(f"[dry run] set_servos_angles({angles}, runtime={runtime})")
        return {"code": 0}

    def get_servos_angles(self, names=None):
        return {"code": 0, "data": {n: RESET_POSE[n] for n in (names or NAMES)}}

    def set_servos_mode(self, mode, names=None):
        print(f"[dry run] set_servos_mode({mode!r}, {names})")
        return {"code": 0}

    def wait(self, seconds):
        time.sleep(seconds)


class SimToRobot:
    """Forward simulator servo targets to the robot, only when they change and only if safe.

    Speed cap: each command's runtime comes from the distance between the new target and where each
    servo is estimated to be right now (interpolating the moves already sent), so a new command in the
    middle of a long move cannot rush the rest of it. A move that would need more than 4000 ms at
    max_speed_deg_s is sent in parts along the straight line to the target, one part per step().
    Self-collision: the path from the estimated pose to the new pose is swept; pairs already touching
    in the last sent pose (e.g. a start pose with an arm resting on the torso) are ignored.
    """

    def __init__(self, sim, robot, max_speed_deg_s=90, deadband_deg=1.0, min_runtime_ms=250, checker=None,
                 clock=time.monotonic):
        if not _num(max_speed_deg_s) or max_speed_deg_s < 1:
            raise ValueError(f"max_speed_deg_s must be >= 1 deg/s, got {max_speed_deg_s!r}")
        self.sim, self.robot = sim, robot
        self.max_speed, self.deadband, self.min_runtime = max_speed_deg_s, deadband_deg, min_runtime_ms
        self.checker = checker or CollisionChecker()
        self.clock = clock
        self.sent = {}  # last commanded target per servo
        self._moves = {}  # servo -> (start deg, target deg, t_send s, runtime s)
        self._base = set()  # contact pairs tolerated (touching in the last sent pose)
        self.start_contacts = set()  # pairs touching in the robot's pose at start()

    # -- state ----------------------------------------------------------------
    def estimated(self, now=None):
        """Where each servo should be now, assuming it follows the commanded linear moves."""
        now = self.clock() if now is None else now
        out = {}
        for n, (a, b, t0, dur) in self._moves.items():
            s = 1.0 if dur <= 0 or now >= t0 + dur else max(0.0, (now - t0) / dur)
            out[n] = a + (b - a) * s
        return out

    def _set_sent(self, pose, runtime_ms, now):
        est = self.estimated(now)
        for n, deg in pose.items():
            self._moves[n] = (est.get(n, deg), deg, now, runtime_ms / 1000)
        self.sent.update(pose)
        self._base = self.checker.baseline | self.checker.contacts(self.sent, baseline=set())

    def start(self):
        """Read the robot's current pose and put the simulator in the same pose, so nothing jumps.
        Servos the robot does not report start at RESET_POSE. Sets start_contacts (pairs already touching)."""
        read = _readings(self.robot.get_servos_angles(NAMES))
        missing = [n for n in NAMES if n not in read]
        if missing:
            log.warning("robot did not report %s; assuming the reset pose for them", missing)
        self.sent, self._moves = {}, {}
        self._set_sent({**RESET_POSE, **read}, 0, self.clock())
        self.start_contacts = self.checker.contacts(self.sent)
        if self.start_contacts:
            log.warning("start pose already has parts touching: %s", sorted(self.start_contacts))
        self.sim.jump_to(self.sent)
        return dict(self.sent)

    # -- planning -------------------------------------------------------------
    def _plan(self, goal, deadband):
        """Next command toward goal: (changed, runtime_ms, partial) or None when nothing to send."""
        changed = {n: v for n, v in goal.items() if abs(v - self.sent.get(n, v)) >= deadband}
        if not changed:
            return None
        est = self.estimated()
        dist = {n: abs(v - est.get(n, v)) for n, v in changed.items()}
        reach = self.max_speed * MAX_RUNTIME_MS / 1000 - 0.5  # rounding to whole degrees adds <= 0.5
        partial = max(dist.values()) > reach + 0.5
        if partial:  # too far for one command at this speed: go part of the way, on the line to the goal
            f = reach / max(dist.values())
            changed = {n: int(round(est.get(n, v) + (v - est.get(n, v)) * f)) for n, v in changed.items()}
            changed = {n: v for n, v in changed.items() if v != self.sent.get(n)}
            if not changed:  # the move in flight already goes this far
                return None
            dist = {n: abs(v - est.get(n, v)) for n, v in changed.items()}
        runtime = math.ceil(max(self.min_runtime, max(dist.values()) / self.max_speed * 1000))
        return changed, min(runtime, MAX_RUNTIME_MS), partial

    def path_contacts(self, goal, samples=None):
        """New contact pairs on the straight path from the estimated pose to {**sent, **goal}."""
        start = {**self.sent, **self.estimated()}
        end = {**self.sent, **goal}
        biggest = max([abs(end[n] - start[n]) for n in end] + [0])
        samples = samples or max(1, min(8, math.ceil(biggest / 10)))
        hits = set()
        for k in range(1, samples + 1):
            s = k / samples
            hits |= self.checker.contacts({n: start[n] + (end[n] - start[n]) * s for n in end}, baseline=self._base)
        return hits

    def _send(self, changed, runtime):
        now = self.clock()
        try:
            res = self.robot.set_servos_angles(changed, runtime)
        except Exception as e:  # robot unreachable or refused: report, keep the old state, keep going
            log.warning("robot error: %s", e)
            return "error", str(e)
        if isinstance(res, dict) and res.get("code", 0) != 0:
            log.warning("robot refused servo command: %s", res)
            return "error", f"robot refused servo command: {res}"
        self._set_sent(changed, runtime, now)
        return "sent", (changed, runtime)

    # -- public ---------------------------------------------------------------
    def step(self):
        """Send changed targets. Returns ("idle"|"sent"|"blocked"|"error", details)."""
        goal = {n: int(round(v)) for n, v in clamp_angles(self.sim.get_target_angles()).items()}
        plan = self._plan(goal, self.deadband)
        if plan is None:
            return "idle", None
        changed, runtime, _ = plan
        hits = self.path_contacts(changed)
        if hits:
            return "blocked", sorted(hits)
        return self._send(changed, runtime)

    def move_to(self, pose, wait=None, timeout_s=60.0):
        """Drive the robot to a pose under the same speed cap and collision check, and wait until done.
        The whole straight path is checked first. Returns ("sent"|"blocked"|"error", details)."""
        goal = {n: int(round(v)) for n, v in clamp_angles(pose).items()}
        hits = self.path_contacts(goal, samples=16)
        if hits:
            return "blocked", sorted(hits)
        wait = wait or getattr(self.robot, "wait", time.sleep)
        deadline = self.clock() + timeout_s
        while self.clock() < deadline:
            if all(self.sent.get(n) == v for n, v in goal.items()):
                wait(max([t0 + dur - self.clock() for _, _, t0, dur in self._moves.values()] + [0]))
                return "sent", goal
            plan = self._plan(goal, 1)
            if plan is None:  # still in flight: nothing new to send yet
                wait(0.05)
                continue
            changed, runtime, _ = plan
            status, info = self._send(changed, runtime)
            if status != "sent":
                return status, info
            wait(runtime / 1000)
        return "error", f"did not reach the pose within {timeout_s} s"


class RobotToSim:
    """Poll the robot's servo angles and drive the simulator to match.

    Readings are clamped to the servo limits (they can sit a degree outside); the sim's strict flag is untouched.
    """

    def __init__(self, sim, robot, period_s=0.25):
        self.sim, self.robot, self.period = sim, robot, period_s
        self.last_error = None

    def step(self):
        """Returns the clamped angles sent to the sim, or None if the robot could not be read (see last_error)."""
        try:
            angles = _readings(self.robot.get_servos_angles(NAMES))
        except Exception as e:
            log.warning("robot read failed: %s", e)
            self.last_error = str(e)
            return None
        self.last_error = None
        if angles:
            self.sim.set_servos_angles(angles, runtime=max(200, int(self.period * 1000)))
        return angles
