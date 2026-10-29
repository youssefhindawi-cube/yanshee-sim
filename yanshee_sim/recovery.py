"""Fall detection and recovery for the real robot.

The firmware's own fall management does not help on our robot (its on/off setting never reached the hal node, it
ignores falls it blames on people, and face down it plays GetupFront, which fails on mats: docs/GETUP_ON_MATS.md),
so this watches the IMU itself. A fall is the robot going from upright to lying (face down or on its back) within
FALL_S and then lying still: being laid down slowly by hand is reported but is not a fall, so an automatic get-up
never starts in someone's hands. The get-ups are the ones verified on the robot on 29 Oct 2026.
"""
import time

# get-up per posture: (robot motion, then reset). Base_GetupF ends in a wide crouch; on a mat closing it can
# stall-lock a hip roll, so the servos are checked before the reset (a locked one would not follow it).
GETUP = {"face_down": ("Base_GetupF", True), "on_back": ("GetupRear", False)}
FALL_S = 0.8        # from leaving the upright band (|euler-x| 60-120) to lying within this long: a fall. A falling
                    # robot crosses 30-60 deg off upright in about 0.3 s, hands laying it down take a second or more
STILL_S = 1.0       # then lying this long with the gyro under STILL_DPS before acting
STILL_DPS = 20.0


def posture(euler_x, euler_y=0.0):
    """'upright', 'face_down', 'on_back', 'side_right', 'side_left' or 'tilted' from the robot's euler-x (about 92
    upright, 0 face down, +-180 on its back) and euler-y (+ = leaning to its right); measured 29 Oct 2026."""
    if abs(euler_y) > 60:
        return "side_right" if euler_y > 0 else "side_left"
    a = abs(euler_x)
    if a < 30:
        return "face_down"
    if a > 150:
        return "on_back"
    return "upright" if 60 < a < 120 else "tilted"


ACTIVE_S = 2.0      # lying within this long of a command running (walking, turning) is a fall, however slow


class FallDetector:
    """Feed IMU samples; feed() returns {"posture", "fell"} once per time down, when the robot has lain still for
    STILL_S. fell is True if it went from upright to lying within FALL_S, or if it went down while it was being
    commanded (active) or within ACTIVE_S of that: on 29 Oct 2026 a walking robot leaned back for 0.9 s, still
    stepping, before it went over, which the time rule alone took for being laid down."""

    def __init__(self, fall_s=FALL_S, still_s=STILL_S, still_dps=STILL_DPS):
        self.fall_s, self.still_s, self.still_dps = fall_s, still_s, still_dps
        self.last_upright = self.down_since = self.still_since = self.last_active = None
        self.fell = self.reported = False

    def feed(self, t, euler_x, gyro_dps=0.0, euler_y=0.0, active=False):
        if active:
            self.last_active = t
        p = posture(euler_x, euler_y)
        if p == "upright":
            self.last_upright, self.down_since, self.still_since, self.reported = t, None, None, False
            return None
        if p == "tilted":
            return None
        if self.down_since is None:
            self.down_since = t
            self.fell = (self.last_upright is not None and t - self.last_upright <= self.fall_s) or \
                (self.last_active is not None and t - self.last_active <= ACTIVE_S)
        self.still_since = (self.still_since or t) if gyro_dps < self.still_dps else None
        if not self.reported and self.still_since is not None and t - self.still_since >= self.still_s:
            self.reported = True
            return {"posture": p, "fell": self.fell, "down_t": self.down_since}
        return None


def read_imu(rest, with_y=False):
    """(euler-x, gyro magnitude in deg/s) from the robot; with_y: (euler-x, euler-y, gyro magnitude)."""
    g = ((rest.gyro().get("data") or {}).get("gyro") or [{}])[0]
    rate = sum(float(g.get(k, 0.0)) ** 2 for k in ("gyro-x", "gyro-y", "gyro-z")) ** 0.5
    if with_y:
        return float(g.get("euler-x", 90.0)), float(g.get("euler-y", 0.0)), rate
    return float(g.get("euler-x", 90.0)), rate


def locked_servos(rest):
    """Servos that refuse a command to the angle they already report: stall-locked (their readback is then stale).
    Healthy servos are already there, so nothing moves. [] when the robot cannot be read."""
    from .servos import LIMITS, NAMES, MIN_RUNTIME_MS
    try:
        now = rest.get_angles(list(NAMES)).get("data") or {}
        ok = {k: int(v) for k, v in now.items() if k in LIMITS and LIMITS[k][0] <= v <= LIMITS[k][1]}
        res = rest.set_angles(ok, MIN_RUNTIME_MS)
    except Exception:  # noqa: BLE001 - a check only
        return []
    return sorted(k for k, v in (res.get("data") or {}).items() if v is False)


def wait_motion(rest, total_s, stop=None):
    """Wait until the robot's motion engine is idle again (at most total_s + 5 s). False if stopped."""
    deadline, seen = time.monotonic() + max(total_s, 1.0) + 5.0, False
    while time.monotonic() < deadline:
        if stop is not None and stop.is_set():
            return False
        time.sleep(0.3)
        try:
            st = (rest.motion_state().get("data") or {}).get("status")
        except Exception:  # noqa: BLE001 - keep waiting
            continue
        seen = seen or st == "run"
        if st == "idle" and (seen or time.monotonic() > deadline - total_s - 4.0):
            return True
    return True


def robot_get_up(rest, lying, stop=None):
    """Play the get-up for `lying` ('face_down' or 'on_back'), then reset if needed and no servo is locked.
    Returns (stood_up, message)."""
    motion, then_reset = GETUP[lying]
    res = rest.play_motion(motion)
    if res.get("code") != 0:
        return False, f"{motion} refused: {res.get('msg')} (code {res.get('code')})"
    if not wait_motion(rest, ((res.get("data") or {}).get("total_time") or 8000) / 1000, stop):
        return False, f"{motion} stopped"
    if then_reset:
        locked = locked_servos(rest)
        if locked:
            return False, (f"{', '.join(locked)} stall-locked after {motion} (grippy floor): no reset; lay the robot "
                           "down, switch those servos to program mode, then reset")
        r = rest.play_motion("reset")
        wait_motion(rest, ((r.get("data") or {}).get("total_time") or 1200) / 1000, stop)
    time.sleep(1.0)
    ex, _ = read_imu(rest)
    p = posture(ex)
    return p == "upright", f"{motion}{' + reset' if then_reset else ''}: {'standing' if p == 'upright' else 'still ' + p} (euler-x {ex:+.0f})"
