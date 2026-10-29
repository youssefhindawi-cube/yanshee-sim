"""Walking statistics on the real robot, to compare walking methods (the firmware walk, our gait): every movement
switch, fall and get-up, appended to robot_dump/walk_stats.jsonl. tools/walk_stats.py prints the comparison."""
import datetime
import json
import os
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.environ.get("YANSHEE_WALK_STATS") or os.path.join(ROOT, "robot_dump", "walk_stats.jsonl")  # tests redirect it
MOVING = {"walk+", "walk-", "side", "diag", "turn_left", "turn_right"}


class WalkStats:
    """One session of one walking method. command(kind) with kind in MOVING or 'stop'; fall(posture);
    getup(ok, message)."""

    def __init__(self, method, note="", path=PATH):
        self.method, self.path, self.lock = method, path, threading.Lock()
        self.session = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
        self.action, self.switch_t, self.switch_kind, self.moving_since = "stop", None, None, None
        self.history = []  # (time, action, switch kind): what the robot was doing when it went down
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._write("session_start", note=note)

    def _write(self, kind, **kv):
        line = json.dumps({"session": self.session, "method": self.method, "t": round(time.time(), 3), "kind": kind, **kv})
        with self.lock, open(self.path, "a") as f:
            f.write(line + "\n")

    def command(self, kind):
        """A movement command; logs a switch when it differs from the current one."""
        if kind == self.action:
            return
        now = time.time()
        moved = round(now - self.moving_since, 2) if self.moving_since is not None else 0.0
        self.switch_t, self.switch_kind = now, f"{self.action}->{kind}"
        self.history = (self.history + [(now, kind, self.switch_kind)])[-50:]
        self._write("switch", switch=self.switch_kind, moved_s=moved if kind not in MOVING or self.action in MOVING else 0.0)
        if kind in MOVING and self.moving_since is None:
            self.moving_since = now
        elif kind not in MOVING and self.moving_since is not None:
            self._write("moving", seconds=moved)
            self.moving_since = None
        self.action = kind

    def fall(self, posture, fell=True, at=None):
        """A fall (fell) or the robot found lying after being laid down (not fell). at: when it went down (wall
        time); it is reported later, once it lies still, so what it was doing is looked up for that moment."""
        now = time.time()
        action, switch_t, switch_kind = self.action, self.switch_t, self.switch_kind
        if at is not None:
            past = [h for h in self.history if h[0] <= at]
            action, switch_t, switch_kind = (past[-1][1], past[-1][0], past[-1][2]) if past else ("stop", None, None)
        since = round((at or now) - switch_t, 2) if switch_t is not None else None
        if self.moving_since is not None:
            self._write("moving", seconds=round(now - self.moving_since, 2))
            self.moving_since = None
        self._write("fall" if fell else "lying", posture=posture, action=action, last_switch=switch_kind,
                    since_switch_s=since)
        self.action = "stop"

    def getup(self, ok, message):
        self._write("getup", ok=bool(ok), message=message)

    def close(self):
        self.command("stop")
        self._write("session_end")
