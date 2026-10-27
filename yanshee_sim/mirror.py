"""Live mirroring between the simulator and the real robot.

SimToRobot: you move the simulated robot (viewer sliders or code) and the real robot follows.
RobotToSim: the real robot moves (built-in motion, app, by hand) and the simulator shows it.

Every pose sent to the robot is checked first: servo limits, a speed cap and self-collision.
"""
from .motion import CollisionChecker
from .servos import MAX_RUNTIME_MS, NAMES, RESET_POSE, clamp_angles


class DryRunRobot:
    """Stands in for the real robot: prints commands instead of sending them."""

    def set_servos_angles(self, angles, runtime=200):
        print(f"[dry run] set_servos_angles({angles}, runtime={runtime})")
        return {"code": 0}

    def get_servos_angles(self, names=None):
        return {"code": 0, "data": {n: RESET_POSE[n] for n in (names or NAMES)}}


class SimToRobot:
    """Forward simulator servo targets to the robot, only when they change and only if safe."""

    def __init__(self, sim, robot, max_speed_deg_s=90, deadband_deg=1.0, min_runtime_ms=250, checker=None):
        self.sim, self.robot = sim, robot
        self.max_speed, self.deadband, self.min_runtime = max_speed_deg_s, deadband_deg, min_runtime_ms
        self.checker = checker or CollisionChecker()
        self.sent = {}

    def start(self):
        """Read the robot's current pose and put the simulator in the same pose, so nothing jumps."""
        res = self.robot.get_servos_angles(NAMES)
        self.sent = clamp_angles(res["data"])
        self.sim.jump_to(self.sent)
        return self.sent

    def step(self):
        """Send changed targets. Returns ("idle"|"sent"|"blocked", details)."""
        target = {n: int(round(v)) for n, v in clamp_angles(self.sim.get_target_angles()).items()}
        changed = {n: v for n, v in target.items() if abs(v - self.sent.get(n, v)) >= self.deadband}
        if not changed:
            return "idle", None
        pose = {**self.sent, **changed}
        hits = self.checker.contacts(pose)
        if hits:
            return "blocked", sorted(hits)
        biggest = max(abs(v - self.sent[n]) for n, v in changed.items())
        runtime = int(min(max(self.min_runtime, biggest / self.max_speed * 1000), MAX_RUNTIME_MS))
        self.robot.set_servos_angles(changed, runtime)
        self.sent.update(changed)
        return "sent", (changed, runtime)


class RobotToSim:
    """Poll the robot's servo angles and drive the simulator to match."""

    def __init__(self, sim, robot, period_s=0.25):
        self.sim, self.robot, self.period = sim, robot, period_s
        sim.strict = False  # readings can sit a degree outside the documented limits

    def step(self):
        res = self.robot.get_servos_angles(NAMES)
        angles = res["data"]
        self.sim.set_servos_angles(angles, runtime=max(200, int(self.period * 1000)))
        return angles
