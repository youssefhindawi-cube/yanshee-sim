"""The robot link: a small ROS node on the robot, started over ssh (Python 2, ROS kinetic).

The robot's readings are LOGGED ON THE ROBOT, stamped with the robot's clock (/home/pi/yanshee_logs/robot_<session>.jsonl),
so the ssh/Wi-Fi delay does not distort them; the laptop logs the simulator side with its own clock
(robot_dump/sim2real_<session>.jsonl). Clock pings over the link (every PING_S) give the offset between the two clocks
so the logs line up later (tools/fetch_robot_logs.py brings the robot's logs over; tools/sim2real_report.py merges).
A copy of the IMU and encoders is also sent over the link, late by the link's delay, for the live mirror and SwayStep.

On the robot the node:
- publishes our servo frames on /hal_angles_set, the way the firmware gait does (decoded from its own messages on
  29 Oct 2026: 17 values in servo order, raw units of 4096 per turn so 1024 = 90 deg, 65535 = leave that servo alone,
  time = move time in ms, 20 at 50 per second). Measured 29 Oct 2026: hal applies them whatever the gait status, but
  IGNORES the head (NeckLR) on this topic: the firmware gait always sends it 90. Each of our frames carries a sequence
  number and the laptop's send time, so its delay over the link is in the robot's log;
- logs every frame on /hal_angles_set (the firmware walk's too, with the publisher's name), the IMU (/hal_gyro_report,
  19 per second, with the compass), the battery (/hal_battery_report) and every firmware log line (/rosout_agg);
- monitors through hal's read-only services (robot_dump/hal_services_2026-10-29.txt): the servo encoders
  (/hal_servo_read type 0: 17 x 4 hex digits, whole degrees, about 35 ms, enc_hz per second), once a second
  battery, board temperature, motion status, fall and shutdown flags and LEDs, and once at the start the servo
  calibration offsets, versions and extension sensors. It never calls hal's write services (serial number, upgrade,
  calibration, offsets, shutdown).
- sends to the laptop: IMU, encoders, the firmware's frames (for the shadow simulator), the monitor, the info.

RobotBody gives SimGait what it needs from a robot (send angles, wait, clock, torso lean and tilt) from the robot's IMU
instead of the physics, so the same gait code (SwayStep) walks the simulator and the robot.
"""
import json
import math
import os
import statistics
import subprocess
import threading
import time

from .servos import LIMITS, NAMES

UNITS_PER_DEG = 4096 / 360.0
KEEP = 65535  # "leave this servo alone"
UPRIGHT_EULER_X = 92.0  # the robot's euler-x standing (measured 29 Oct 2026)
REMOTE_PATH = "/tmp/yanshee_sway_stream.py"
ROBOT_LOG_DIR = "/home/pi/yanshee_logs"
ROS_ENV = "source /opt/ros/kinetic/setup.bash; source /opt/yanshee/setup.bash >/dev/null 2>&1; "
PING_S = 2.0  # clock pings over the link

REMOTE = r'''
import sys, os, json, threading, time, rospy
from ubt_msgs.msg import angles_set, gyro_report, battery_report
from ubt_msgs import srv as S
from rosgraph_msgs.msg import Log
ENC_HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
SESSION = sys.argv[2] if len(sys.argv) > 2 else time.strftime("%Y%m%d_%H%M%S")
LOG_DIR = sys.argv[3] if len(sys.argv) > 3 else "/home/pi/yanshee_logs"
if not os.path.isdir(LOG_DIR):
    os.makedirs(LOG_DIR)
LOG_PATH = os.path.join(LOG_DIR, "robot_%s.jsonl" % SESSION)
logf = open(LOG_PATH, "a")
rospy.init_node("yanshee_sway_stream", anonymous=True, disable_signals=True)
NODE = rospy.get_name()
pub = rospy.Publisher("/hal_angles_set", angles_set, queue_size=1)
lock, flock = threading.Lock(), threading.Lock()
def out(d):  # to the laptop
    with lock:
        sys.stdout.write(json.dumps(d) + "\n")
        sys.stdout.flush()
def rec(kind, d):  # to the robot's log, robot clock
    d = dict(d)
    d["k"] = kind
    d.setdefault("t", time.time())
    with flock:
        logf.write(json.dumps(d) + "\n")
def flusher():
    while True:
        time.sleep(1.0)
        with flock:
            logf.flush()
def on_imu(m):
    d = {"e": list(m.euler_data), "g": list(m.gyro_data), "a": list(m.accel_data), "m": list(m.compass_data), "t": time.time()}
    rec("imu", d)
    out(d)
rospy.Subscriber("/hal_gyro_report", gyro_report, on_imu, queue_size=5)
def on_cmd(m):
    t = time.time()
    src = (getattr(m, "_connection_header", None) or {}).get("callerid", "")
    if src == NODE:
        return  # ours: logged when published, with its sequence number
    rec("frame", {"c": list(m.angles), "ms": m.time, "src": src, "t": t})
    out({"c": list(m.angles), "ms": m.time, "src": src, "t": t})
rospy.Subscriber("/hal_angles_set", angles_set, on_cmd, queue_size=100)
def on_bat(m):
    d = {"charging": m.charging, "voltage": m.voltage, "percent": m.percent, "dc": m.dc}
    rec("battery", d)
    out({"b": d, "t": time.time()})
rospy.Subscriber("/hal_battery_report", battery_report, on_bat, queue_size=1)
LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}
def on_log(m):
    if m.name != NODE:
        rec("rosout", {"lvl": LEVELS.get(m.level, m.level), "node": m.name, "msg": m.msg[:400], "fn": m.function,
                       "st": m.header.stamp.to_sec()})
rospy.Subscriber("/rosout_agg", Log, on_log, queue_size=200)
def proxy(name, typ):
    return rospy.ServiceProxy(name, getattr(S, typ))
def fields(r):
    return dict((k, getattr(r, k)) for k in r.__slots__ if k != "rc")
def info():  # once: calibration offsets, versions, extension sensors (hal read services only)
    d = {}
    try:
        off = proxy("/hal_servo_offset_read", "servo_offset_read")
        d["servo_offsets"] = [off(i).offset for i in range(1, 18)]
        ver = proxy("/hal_version_read", "version_read")
        d["versions"] = dict((str(t), ver(t, 0).ver) for t in (0, 2))
        d["ext"] = fields(proxy("/hal_ext_read", "ext_read")(0))
    except Exception as e:
        d["error"] = str(e)
    rec("info", d)
    out({"info": d, "t": time.time()})
def encoders():  # the servo encoders straight from hal (whole degrees), ENC_HZ per second
    read = proxy("/hal_servo_read", "servo_read")
    while not rospy.is_shutdown():
        t0 = time.time()
        try:
            r = read(0, "")
            b = r.servo_angles
            d = {"s": [int(b[i:i + 4], 16) for i in range(0, min(len(b), 68), 4)], "ms": int((time.time() - t0) * 1000),
                 "rc": r.rc, "t": t0}
            rec("enc", d)
            out(d)
        except Exception as e:
            rec("enc_error", {"error": str(e)[:200], "t": t0})
        time.sleep(max(0.0, 1.0 / ENC_HZ - (time.time() - t0)))
def monitor():  # once a second: battery, board temperature, motion status, fall and shutdown flags, LEDs
    calls = [("battery", proxy("/hal_battery_read", "battery_read")), ("board", proxy("/hal_board_read", "board_read")),
             ("move", proxy("/hal_move_status", "move_status")), ("fall", proxy("/hal_robot_fall", "value_get")),
             ("shutdown", proxy("/hal_robot_shutdown_read", "value_get")), ("led", proxy("/hal_led_read", "led_reply"))]
    while not rospy.is_shutdown():
        t0, d = time.time(), {}
        for k, p in calls:
            try:
                d[k] = fields(p())
            except Exception as e:
                d[k] = {"error": str(e)[:120]}
        rec("hal", dict(d, t=t0))
        out({"hal": d, "t": t0})
        time.sleep(max(0.0, 1.0 - (time.time() - t0)))
t0 = time.time()
while pub.get_num_connections() == 0 and time.time() - t0 < 8:  # hal must be listening before the first command
    time.sleep(0.05)
time.sleep(0.3)
for fn in (info, encoders, monitor, flusher):
    th = threading.Thread(target=fn)
    th.daemon = True
    th.start()
rec("start", {"node": NODE, "session": SESSION, "enc_hz": ENC_HZ})
out({"ready": True, "subscribers": pub.get_num_connections(), "node": NODE, "log": LOG_PATH})
for line in iter(sys.stdin.readline, ""):
    p = line.split()
    if not p or p[0] == "Q":
        break
    if p[0] == "S" and len(p) == 21:  # S ms seq laptop_time 17 values
        t = time.time()
        msg = angles_set()
        msg.time = int(p[1])
        msg.angles = [int(x) for x in p[4:]]
        pub.publish(msg)
        rec("frame", {"c": msg.angles, "ms": msg.time, "src": "ours", "seq": int(p[2]), "lt": float(p[3]), "t": t})
    elif p[0] == "P" and len(p) == 3:  # clock ping: P seq laptop_time
        t = time.time()
        rec("clock", {"seq": int(p[1]), "lt": float(p[2]), "t": t})
        out({"pong": int(p[1]), "lt": float(p[2]), "rt": t})
rec("stop", {})
with flock:
    logf.flush()
    logf.close()
rospy.signal_shutdown("done")
'''


def to_degrees(units):
    """The 17 raw /hal_angles_set values -> {servo: deg} (servos left alone, 65535, are left out)."""
    return {n: u * 360.0 / 4096 for n, u in zip(NAMES, units) if u != KEEP}


def to_units(angles):
    """{servo: deg} -> the 17 raw values for /hal_angles_set (KEEP for servos not given), clamped to the limits."""
    out = [KEEP] * len(NAMES)
    for n, deg in angles.items():
        if n in LIMITS:
            lo, hi = LIMITS[n]
            out[NAMES.index(n)] = int(round(min(max(float(deg), lo), hi) * UNITS_PER_DEG))
    return out


class RobotStream:
    """The link: send(angles, ms) publishes one /hal_angles_set frame; imu is the latest IMU sample ({"e": euler deg,
    "g": gyro deg/s, "a": accel g, "age": seconds}, late by the link delay); battery the latest battery report.
    on_frame(degrees, ms, src, ours) is called for every frame on the robot: ours locally as they are sent, the
    firmware's as they arrive. log(kind, **data) gets the laptop-side records: link (the robot log's path), info,
    clock (offset robot - laptop and round trip, per ping). The robot's readings are in its own log (robot_log)."""

    def __init__(self, host, user="pi", password="raspberry", log=None, on_frame=None, timeout=20.0, enc_hz=5.0,
                 session=None):
        env = dict(os.environ, SSHPASS=password)
        self._ssh = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8", f"{user}@{host}"]
        self._env, self.host, self.user = env, host, user
        up = subprocess.run(self._ssh + [f"cat > {REMOTE_PATH}"], input=REMOTE, text=True, env=env, timeout=30,
                            capture_output=True)
        if up.returncode:
            raise RuntimeError(f"cannot upload the stream script to the robot: {up.stderr.strip()[:200]}")
        self.session = session or time.strftime("%Y%m%d_%H%M%S")
        self.proc = subprocess.Popen(self._ssh + [ROS_ENV + f"exec python2 -u {REMOTE_PATH} {float(enc_hz)} "
                                                  f"{self.session} {ROBOT_LOG_DIR}"], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1, env=env)
        self.log, self.on_frame, self._imu, self._imu_t, self.ready = log, on_frame, None, 0.0, threading.Event()
        self.battery, self.node, self.robot_log = None, None, None  # robot_log: the log's path on the robot
        self.encoders, self._enc_t, self.hal, self.info = None, 0.0, None, None
        self.subscribers = None  # how many nodes listen to /hal_angles_set (hal: 1)
        self.sent, self.lock, self._seq, self._ping = 0, threading.Lock(), 0, 0
        self.clock = []  # (round trip s, offset robot - laptop s) per ping
        threading.Thread(target=self._read, name="robot-stream", daemon=True).start()
        if not self.ready.wait(timeout):
            self.close()
            raise RuntimeError("the robot stream did not start (ROS on the robot not answering?)")
        threading.Thread(target=self._pinger, name="robot-stream-ping", daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            try:
                self._line(d)
            except Exception:  # noqa: BLE001 - a bad line or a failing callback must not end the link
                pass

    def _line(self, d):
        if d.get("ready"):
            self.subscribers, self.node, self.robot_log = d.get("subscribers"), d.get("node"), d.get("log")
            if self.log:
                self.log("link", robot_log=self.robot_log, node=self.node, session=self.session)
            self.ready.set()
        elif "e" in d:
            self._imu, self._imu_t = d, time.monotonic()
        elif "c" in d:
            if self.on_frame:
                self.on_frame(to_degrees(d["c"]), d.get("ms") or 20, d.get("src"), False)
        elif "b" in d:
            self.battery = d["b"]
        elif "s" in d:
            self.encoders = {n: float(v) for n, v in zip(NAMES, d["s"])}
            self._enc_t = time.monotonic()
        elif "hal" in d:
            self.hal = d["hal"]
        elif "pong" in d:
            back = time.time()
            rtt, off = back - d["lt"], d["rt"] - (d["lt"] + back) / 2
            self.clock = (self.clock + [(rtt, off)])[-200:]
            if self.log:
                self.log("clock", seq=d["pong"], rtt=round(rtt, 4), offset=round(off, 4))
        elif "info" in d:
            self.info = d["info"]
            if self.log:
                self.log("info", **d["info"])

    def _pinger(self):
        while self.alive:
            with self.lock:
                if not self.alive:
                    return
                self._ping += 1
                self.proc.stdin.write(f"P {self._ping} {time.time():.4f}\n")
                self.proc.stdin.flush()
            time.sleep(PING_S)

    def clock_offset(self):
        """Robot clock minus laptop clock in s, from the quickest third of the pings (None before the first)."""
        if not self.clock:
            return None
        best = sorted(self.clock)[:max(1, len(self.clock) // 3)]
        return statistics.median(o for _, o in best)

    @property
    def alive(self):
        return self.proc.poll() is None

    def fresh_encoders(self, max_age=0.5):
        """The latest encoder reading ({servo: deg}) if younger than max_age seconds, else None."""
        return dict(self.encoders) if self.encoders and time.monotonic() - self._enc_t < max_age else None

    @property
    def imu(self):
        if self._imu is None:
            return None
        return {**self._imu, "age": time.monotonic() - self._imu_t}

    def send(self, angles, ms=20):
        units = to_units(angles)
        with self.lock:
            if not self.alive:
                raise RuntimeError("the robot stream has ended")
            self._seq += 1
            self.proc.stdin.write(f"S {int(max(20, ms))} {self._seq} {time.time():.4f} " + " ".join(map(str, units)) + "\n")
            self.proc.stdin.flush()
            self.sent += 1
        if self.on_frame:
            self.on_frame(to_degrees(units), max(20, int(ms)), "ours", True)

    def fetch_log(self, dest_dir):
        """Copy the robot's log of this link to dest_dir (scp). Returns the local path."""
        if not self.robot_log:
            raise RuntimeError("no robot log yet")
        return fetch(self.host, self.robot_log, dest_dir, user=self.user, env=self._env)

    def close(self):
        try:
            with self.lock:
                if self.alive:
                    self.proc.stdin.write("Q\n")
                    self.proc.stdin.flush()
            self.proc.wait(3)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def fetch(host, remote_path, dest_dir, user="pi", password="raspberry", env=None):
    """scp one file from the robot into dest_dir; returns the local path."""
    os.makedirs(dest_dir, exist_ok=True)
    local = os.path.join(dest_dir, os.path.basename(remote_path))
    env = env or dict(os.environ, SSHPASS=password)
    r = subprocess.run(["sshpass", "-e", "scp", "-q", "-o", "StrictHostKeyChecking=no", f"{user}@{host}:{remote_path}", local],
                       env=env, timeout=120, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(f"cannot copy {remote_path} from the robot: {r.stderr.strip()[:200]}")
    return local


class _Clock:
    """sim.data stand-in for SimGait: wall-clock time and no actuator forces."""

    def __init__(self):
        self._t0 = time.monotonic()
        self.actuator_force = [0.0] * len(NAMES)

    @property
    def time(self):
        return time.monotonic() - self._t0


class RobotBody:
    """What SimGait needs from a robot, on the real Yanshee: servo commands through RobotStream, torso lean and tilt
    from its IMU (euler-x about 92 standing, + leaning back; euler-y + leaning right; gyro-x the back-lean rate,
    -gyro-z the right-lean rate: all measured 29 Oct 2026). balance() is not available (it needs the body's velocity):
    SimGait then walks without its capture-point brake and foot placement."""

    FRAME_MS = 20  # the firmware gait's frame: a long move is sent as 20 ms frames (as it does), not as one frame

    def __init__(self, stream, pose):
        """pose: the robot's servo angles now ({servo: deg}); long moves start from it."""
        self.stream, self.data, self._lock = stream, _Clock(), threading.Lock()
        self.pose = {n: float(d) for n, d in pose.items() if n in LIMITS}

    def _euler(self):
        imu = self.stream.imu
        if imu is None or imu["age"] > 0.5:
            raise RuntimeError("no fresh IMU reading from the robot")
        return imu

    def _send(self, angles, ms):
        self.stream.send(angles, ms)
        self.pose.update(angles)

    def set_servos_angles(self, angles, runtime):
        """A move over runtime ms, sent as 20 ms frames from the current pose (blocks until sent)."""
        start = {n: self.pose.get(n, d) for n, d in angles.items()}
        n_frames = max(1, int(round(runtime / self.FRAME_MS)))
        t0 = time.monotonic()
        for i in range(1, n_frames + 1):
            k = i / n_frames
            self._send({n: start[n] + (d - start[n]) * k for n, d in angles.items()}, self.FRAME_MS)
            time.sleep(max(0.0, t0 + i * self.FRAME_MS / 1000 - time.monotonic()))
        return {"code": 0, "data": {n: True for n in angles}, "msg": "success"}

    def wait(self, seconds):
        time.sleep(max(0.0, seconds))

    def torso_lean(self):
        imu = self._euler()
        ex, ey = imu["e"][0], imu["e"][1]
        return (ex - UPRIGHT_EULER_X, ey, imu["g"][0], -imu["g"][2])

    def torso_tilt_deg(self):
        """Tilt from upright, from the fused euler angles (the accelerometer swings by tenths of a g while walking)."""
        imu = self._euler()
        c = math.cos(math.radians(imu["e"][0] - UPRIGHT_EULER_X)) * math.cos(math.radians(imu["e"][1]))
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))

    def balance(self):
        raise NotImplementedError("the robot gives no body velocity: SimGait walks without the capture point")
