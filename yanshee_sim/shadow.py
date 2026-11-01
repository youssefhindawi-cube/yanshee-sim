"""The shadow simulator: a hidden physics copy of the robot that gets the same servo frames as the real robot (the
firmware walk's and SwayStep's, echoed by the robot link) while the visible simulator mirrors the real robot.
What the physics predicts and what the robot does are logged side by side (robot_dump/sim2real_*.jsonl) to measure
and later close the sim-to-real gap. Only walking is shadowed: built-in motions do not go through /hal_angles_set.
"""
import heapq
import itertools
import math
import threading
import time

from .servos import NAMES

FALL_TILT_DEG = 50.0  # the shadow counts as fallen past this tilt (it stays down until the next reset)


class Shadow:
    def __init__(self, sim_factory=None):
        self._factory = sim_factory
        self.sim, self.lock = None, threading.Lock()
        self.fallen, self.frames, self.active = False, 0, False
        self._input_t = 0.0  # wall time of the last reset or frame applied
        self._due, self._seq, self._cv = [], itertools.count(), threading.Condition()  # frames waiting for their time
        threading.Thread(target=self._scheduler, name="shadow-frames", daemon=True).start()

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
        with self._cv:
            self._due.clear()
        with self.lock:
            sim = self._ensure()
            sim.respawn()
            sim.jump_to({n: d for n, d in pose.items() if n in NAMES})
            self.fallen, self.frames, self.active = False, 0, True
            self._input_t = time.time()

    def feed(self, angles, ms, due=None):
        """One servo frame as the robot got it ({servo: deg}, move time in ms). due (wall time): apply it then, not
        now: frames that crossed the Wi-Fi arrive bunched, and replayed bunched the simulator fell where the robot
        did not (29 Oct 2026); scheduled at the robot's own times plus a fixed delay they keep their rhythm."""
        if due is not None and due > time.time():
            with self._cv:
                heapq.heappush(self._due, (due, next(self._seq), angles, ms))
                self._cv.notify()
            return
        with self.lock:
            if not self.active or self.sim is None:
                return
            self.sim._send(angles, max(20, int(ms)))
            self.frames += 1
            self._input_t = time.time()

    # Quiet this long after the last frame: the robot settles, then the shadow stops reporting (with nothing to follow it
    # had logged 10 times a second for 45 min after the robot left the Wi-Fi, 29 Oct 2026).
    IDLE_S = 5.0
    LOOPING = ("TurnLeft", "TurnRight")  # AppWalk starts these with repeat 0: hal loops them until the next motion

    def play_motion(self, name, frames, due=None):
        """Play an .hts motion the robot started (frames: hts.load()["frames"]) from wall time due; a motion in LOOPING
        repeats until the next play_motion() or stop_motion()."""
        self.stop_motion()
        cancel = threading.Event()
        self._motion = cancel

        def run():
            if due is not None:
                cancel.wait(max(0.0, due - time.time()))
            while not cancel.is_set():
                for f in frames:
                    if cancel.is_set():
                        return
                    self.feed(f["angles"], f["runtime"])
                    cancel.wait((f["runtime"] + f.get("hold", 0)) / 1000.0)
                if name not in self.LOOPING:
                    return
        threading.Thread(target=run, name=f"shadow-{name}", daemon=True).start()

    def stop_motion(self, due=None):
        cancel = getattr(self, "_motion", None)
        if cancel is None:
            return
        if due is not None and due > time.time():
            threading.Timer(due - time.time(), cancel.set).start()
        else:
            cancel.set()

    def _scheduler(self):
        while True:
            with self._cv:
                while not self._due or self._due[0][0] > time.time():
                    self._cv.wait(None if not self._due else max(0.0, self._due[0][0] - time.time()))
                _, _, angles, ms = heapq.heappop(self._due)
            self.feed(angles, ms)

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
            active = self.active and time.time() - self._input_t < self.IDLE_S
            return {"active": active, "frames": self.frames, "tilt": round(tilt, 2), "fallen": self.fallen,
                    "euler": [round(g[k], 2) for k in ("euler-x", "euler-y", "euler-z")],
                    "gyro": [round(g[k], 2) for k in ("gyro-x", "gyro-y", "gyro-z")],
                    "accel": [round(g[k], 3) for k in ("accel-x", "accel-y", "accel-z")], "joints": joints}

    def close(self):
        with self.lock:
            if self.sim is not None:
                self.sim.close()
                self.sim = None
