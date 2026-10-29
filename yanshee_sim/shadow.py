"""The shadow simulator: a hidden physics copy of the robot that gets the same servo frames as the real robot (the
firmware walk's and SwayStep's, echoed by the robot link) while the visible simulator mirrors the real robot.
What the physics predicts and what the robot does are logged side by side (robot_dump/sim2real_*.jsonl) to measure
and later close the sim-to-real gap. Only walking is shadowed: built-in motions do not go through /hal_angles_set.
"""
import math
import threading

from .servos import NAMES

FALL_TILT_DEG = 50.0  # the shadow counts as fallen past this tilt (it stays down until the next reset)


class Shadow:
    def __init__(self, sim_factory=None):
        self._factory = sim_factory
        self.sim, self.lock = None, threading.Lock()
        self.fallen, self.frames, self.active = False, 0, False

    def _ensure(self):
        if self.sim is None:
            if self._factory is not None:
                self.sim = self._factory()
            else:
                from .robot import SimRobot
                self.sim = SimRobot(viewer=False, realtime=True)
            self.sim.imu_noise = False
        return self.sim

    def reset(self, pose):
        """Stand the shadow up with the real robot's joint angles ({servo: deg}) at the start of a walk."""
        with self.lock:
            sim = self._ensure()
            sim.respawn()
            sim.jump_to({n: d for n, d in pose.items() if n in NAMES})
            self.fallen, self.frames, self.active = False, 0, True

    def feed(self, angles, ms):
        """One servo frame as the robot got it ({servo: deg}, move time in ms)."""
        with self.lock:
            if not self.active or self.sim is None:
                return
            self.sim._send(angles, max(20, int(ms)))
            self.frames += 1

    def stop(self):
        with self.lock:
            self.active = False

    def sample(self):
        """The shadow's state: joints (deg), IMU as the robot would report it, tilt, fallen (once past
        FALL_TILT_DEG it stays fallen until reset). None before the first walk."""
        with self.lock:
            sim = self.sim
            if sim is None:
                return None
            g = sim.get_sensors_gyro()["data"]["gyro"][0]
            tilt = sim.torso_tilt_deg()
            self.fallen = self.fallen or tilt > FALL_TILT_DEG
            with sim._lock:
                joints = {n: round(math.degrees(float(sim.data.qpos[sim._qadr[n]])), 2) for n in NAMES}
            return {"active": self.active, "frames": self.frames, "tilt": round(tilt, 2), "fallen": self.fallen,
                    "euler": [round(g[k], 2) for k in ("euler-x", "euler-y", "euler-z")],
                    "gyro": [round(g[k], 2) for k in ("gyro-x", "gyro-y", "gyro-z")],
                    "accel": [round(g[k], 3) for k in ("accel-x", "accel-y", "accel-z")], "joints": joints}

    def close(self):
        with self.lock:
            if self.sim is not None:
                self.sim.close()
                self.sim = None
