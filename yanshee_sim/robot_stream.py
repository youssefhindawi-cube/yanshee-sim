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
  (/hal_servo_read type 0: 17 x 4 hex digits, whole degrees, about 35 ms, enc_hz per second; paused while hal
  plays a motion, when the reads filled its buffers), once a second
  battery, board temperature, motion status, fall and shutdown flags and LEDs, and once at the start the servo
  calibration offsets, versions and extension sensors. It never calls hal's write services (serial number, upgrade,
  calibration, offsets, shutdown).
- sends to the laptop: IMU, encoders, the firmware's frames (for the shadow simulator), the monitor, the info
  (through a queue: a stalled link drops them instead of holding the node up);
- passes REST calls sent through the link to the robot's REST server on the robot (AppWalk's walk, turn and stop
  commands: no new connection over the Wi-Fi for each, one kept-open connection on the robot);
- plays TurnLeft / TurnRight itself when asked (T name ... T 0), from the robot's own .hts frames sent at the start
  (TF), streamed in 20 ms frames with the balance's ankle offsets, looped until released, then ResetSwitch: hal
  plays the .hts itself, where nothing can be added to its frames. It stops by itself if the laptop goes silent;
- with the balance on (B 1), adds ankle offsets from the IMU to every firmware gait frame (an ankle-only frame right
  after it) and fades them out after the walk;
- WATCHDOG: if the laptop goes silent (it sends a heartbeat every HEARTBEAT_S) for watchdog_s while the firmware gait
  walks or a TurnLeft/TurnRight loops, it stops the robot itself, the mobile app's way (a gait command with every
  value 0), and exits the gait if it still walks 3 s later; the laptop's own dead-man cannot reach a robot whose
  Wi-Fi has gone. After a minute of silence the node exits (a new link also ends any old node first).

RobotBody gives SimGait what it needs from a robot (send angles, wait, clock, torso lean and tilt) from the robot's IMU
instead of the physics, so the same gait code (SwayStep) walks the simulator and the robot.
"""
import glob
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
HEARTBEAT_S = 0.25  # the laptop's heartbeat over the link, for the robot's watchdog
WATCHDOG_S = 1.5  # robot watchdog: the laptop silent this long while walking or turning -> the robot stops itself
RELAY_SILENT_S = 1.0  # no REST through a link that has said nothing for this long (the IMU alone comes 19 per second)
SSH_ALIVE = ["-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3"]  # a dead connection ends ssh in about 6 s
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

REMOTE = r'''
UNITS = 4096 / 360.0
import sys, os, json, threading, time, rospy
try:
    import httplib
    import Queue as queue
except ImportError:  # Python 3 (the tests' fake ROS)
    import http.client as httplib
    import queue
from ubt_msgs.msg import angles_set, gyro_report, battery_report
from ubt_msgs import srv as S
from rosgraph_msgs.msg import Log
ENC_HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
SESSION = sys.argv[2] if len(sys.argv) > 2 else time.strftime("%Y%m%d_%H%M%S")
LOG_DIR = sys.argv[3] if len(sys.argv) > 3 else "/home/pi/yanshee_logs"
REST_PORT = int(sys.argv[4]) if len(sys.argv) > 4 else 9090  # the robot's REST server, on this machine
WATCHDOG_S = float(sys.argv[5]) if len(sys.argv) > 5 else 1.5  # laptop silent this long while moving: stop (0: never)
LINGER_S = 60.0  # laptop silent this long: this node exits (a link that died without closing)
if not os.path.isdir(LOG_DIR):
    os.makedirs(LOG_DIR)
LOG_PATH = os.path.join(LOG_DIR, "robot_%s.jsonl" % SESSION)
logf = open(LOG_PATH, "a")
rospy.init_node("yanshee_sway_stream", anonymous=True, disable_signals=True)
NODE = rospy.get_name()
pub = rospy.Publisher("/hal_angles_set", angles_set, queue_size=1)
flock = threading.Lock()
outq = queue.Queue(2000)
def out(d):  # to the laptop, through a queue: a stalled link must not hold up the callbacks or the watchdog
    try:
        outq.put_nowait(json.dumps(d) + "\n")
    except queue.Full:
        pass
def writer():
    while True:
        line = outq.get()
        sys.stdout.write(line)
        sys.stdout.flush()
WD = {"rx": time.time(), "gait": 0.0, "turn": None, "stop": 0.0, "exit": False}  # the watchdog's view
BAL = {"b": None, "off": False, "last": None}  # balance on the firmware walk: Balance, fading out, last gait frame
TURN = {"frames": {}, "want": None, "playing": False, "abort": False}  # our own TurnLeft/TurnRight (with the balance)
ENC = {"s": None, "t": 0.0}  # the latest encoder reading (whole degrees), where a turn starts from
def with_offsets(c, offs):  # a whole frame (raw units) with the balance offsets on its ankles
    fr = list(c)
    for n, o in offs.items():
        i = SERVO_INDEX[n]
        if fr[i] != 65535:
            lo, hi = SERVO_LIMITS[n]
            fr[i] = int(round(min(max(fr[i] / UNITS + o, lo), hi) * UNITS))
    return fr
def turn_publish(c, ms, t):
    b = BAL["b"]
    fr = with_offsets(c, b.offsets()) if b is not None and not BAL["off"] else list(c)
    msg = angles_set()
    msg.time, msg.angles = int(ms), fr
    pub.publish(msg)
    rec("frame", {"c": fr, "ms": int(ms), "src": "turn", "t": t})
    out({"c": fr, "ms": int(ms), "src": "turn", "t": t})
def play_key(cur, target, runtime, hold):
    """One keyframe as hal plays an .hts one: a timed move to target over runtime ms, then held hold ms; streamed
    here in 20 ms frames (from cur, the last target; the first one as a single timed frame) so the balance can
    correct every frame. Returns the new target, or None if aborted."""
    t0 = time.time()
    if cur is None:
        turn_publish(target, runtime, t0)
        steps = []
    else:
        n = max(1, int(runtime) // 20)
        steps = [[c if (c == 65535 or g == 65535) else int(round(c + (g - c) * float(i) / n)) for c, g in zip(cur, target)]
                 for i in range(1, n + 1)]
    end, k = t0 + (runtime + hold) / 1000.0, 0
    while True:
        if TURN["abort"]:
            return None
        now = time.time()
        if now >= end - 0.005:
            break
        if k < len(steps):
            turn_publish(steps[k], 20, now)
            k += 1
        elif cur is not None or now >= t0 + runtime / 1000.0:
            turn_publish(target, 20, now)  # holding (the balance still corrects it)
        time.sleep(max(0.0, t0 + 0.02 * k - time.time()) if k < len(steps) else 0.02)
    return target
def turner():  # plays TURN["want"] looped until it changes, then ResetSwitch; the laptop silent -> it stops by itself
    while True:
        time.sleep(0.02)
        name = TURN["want"]
        if name is None:
            continue
        seq, reset = TURN["frames"].get(name), TURN["frames"].get("ResetSwitch") or []
        if not seq or time.time() - WD["gait"] < 0.3:  # unknown, or the firmware gait is walking
            TURN["want"] = None
            out({"turn": "refused", "name": name, "t": time.time()})
            continue
        TURN["playing"], TURN["abort"] = True, False
        rec("turn", {"name": name, "st": "start", "t": time.time()})
        out({"turn": "start", "name": name, "t": time.time()})
        cur, k, why = None, 0, "released"
        if ENC["s"] is not None and time.time() - ENC["t"] < 1.5:  # stream the first move too, from the encoders
            cur = [int(round(deg * UNITS)) for deg in ENC["s"]]
        while TURN["want"] == name:
            if WATCHDOG_S and time.time() - WD["rx"] > WATCHDOG_S:
                why = "laptop silent"
                break
            f = seq[k % len(seq)]
            cur = play_key(cur, f[0], f[1], f[2])
            if cur is None:
                break
            k += 1
        if cur is not None:
            for f in reset:
                cur = play_key(cur, f[0], f[1], f[2])
                if cur is None:
                    break
        if cur is None:
            why = "aborted"
        BAL["last"] = cur
        if TURN["want"] == name:
            TURN["want"] = None
        TURN["playing"] = False
        rec("turn", {"name": name, "st": "finish", "why": why, "t": time.time()})
        out({"turn": "finish", "name": name, "why": why, "t": time.time()})
def bal_publish(c, offs, ms, t):  # one frame for the ankles only: the gait's angles (c) plus the balance offsets
    fr = [65535] * len(c)
    for n, o in offs.items():
        i = SERVO_INDEX[n]
        if c[i] != 65535:
            lo, hi = SERVO_LIMITS[n]
            fr[i] = int(round(min(max(c[i] / UNITS + o, lo), hi) * UNITS))
    msg = angles_set()
    msg.time, msg.angles = ms, fr
    pub.publish(msg)
    rec("frame", {"c": fr, "ms": ms, "src": "balance", "t": t})
    out({"c": fr, "ms": ms, "src": "balance", "t": t})
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
    b = BAL["b"]
    if b is not None and not BAL["off"]:
        lean = lean_from_imu(d["e"], d["g"])
        if d["t"] - WD["gait"] < 0.3 or TURN["playing"]:  # walking or turning: new offsets for the next frames
            b.update(lean[0], lean[1], lean[2], lean[3], d["t"])
            rec("bal", {"p": round(b.pitch, 2), "r": round(b.roll, 2), "t": d["t"]})
        elif not (b.pitch or b.roll):
            b.stand(*lean)  # standing still: follow the standing lean
    rec("imu", d)
    out(d)
rospy.Subscriber("/hal_gyro_report", gyro_report, on_imu, queue_size=5)
def on_cmd(m):
    t = time.time()
    src = (getattr(m, "_connection_header", None) or {}).get("callerid", "")
    if src == NODE:
        return  # ours: logged when published, with its sequence number
    WD["gait"] = t  # the firmware gait is walking
    b = BAL["b"]
    if b is not None and not BAL["off"]:
        BAL["last"] = list(m.angles)
        offs = b.offsets()
        if any(offs.values()):
            bal_publish(list(m.angles), offs, m.time, t)
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
        t = time.time()
        rec("rosout", {"lvl": LEVELS.get(m.level, m.level), "node": m.name, "msg": m.msg[:400], "fn": m.function,
                       "st": m.header.stamp.to_sec(), "t": t})
        if m.msg.startswith("Publish action[") and "status[" in m.msg:  # hal starts / ends an .hts motion
            path, st = m.msg.split("[", 1)[1].split("]", 1)[0], m.msg.split("status[", 1)[1].split("]", 1)[0]
            if "TurnLeft" in path or "TurnRight" in path:  # AppWalk's turns loop until stopped
                WD["turn"] = path if st == "start" else None
            out({"mo": path, "st": st, "t": t})
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
    read, moving = proxy("/hal_servo_read", "servo_read"), proxy("/hal_move_status", "move_status")
    paused = False
    while not rospy.is_shutdown():
        t0 = time.time()
        try:  # not while hal plays a motion (.hts): reads then filled hal's buffers (29 Oct 2026, during GetupRear)
            busy = "run" in moving().status
        except Exception:
            busy = False
        if busy != paused:
            paused = busy
            rec("enc_pause" if busy else "enc_resume", {"t": t0})
        if busy:
            time.sleep(max(0.0, 1.0 / ENC_HZ - (time.time() - t0)))
            continue
        try:
            r = read(0, "")
            b = r.servo_angles
            d = {"s": [int(b[i:i + 4], 16) for i in range(0, min(len(b), 68), 4)], "ms": int((time.time() - t0) * 1000),
                 "rc": r.rc, "t": t0}
            if len(d["s"]) == len(SERVO_INDEX):
                ENC["s"], ENC["t"] = d["s"], t0
            rec("enc", d)
            out(d)
        except Exception as e:
            rec("enc_error", {"error": str(e)[:200], "t": t0})
        time.sleep(max(0.0, 1.0 / ENC_HZ - (time.time() - t0)))
def http(method, path, body=None, conn=None, timeout=5.0):
    """One call to the robot's REST server here: (connection, status, content type, body, server closes it)."""
    c = conn or httplib.HTTPConnection("127.0.0.1", REST_PORT, timeout=timeout)
    c.request(method, "/v1/" + path, body, {"Content-Type": "application/json"})
    r = c.getresponse()
    data = r.read()
    return c, r.status, r.getheader("content-type", ""), data, r.getheader("connection", "").lower() == "close" or r.version == 10
relq = queue.Queue()
def relay():  # REST calls sent through the link (AppWalk's walk, turn and stop commands), over one kept-open connection
    conn = None
    while True:
        rid, method, path, body = relq.get()
        t0, d = time.time(), {"rr": rid}
        for attempt in (0, 1):
            reused = conn is not None
            try:
                conn, st, ct, data, close = http(method, path, body, conn)
                d.update(st=st, ct=ct, b=data.decode("utf-8", "replace"))
                if close:
                    conn.close()
                    conn = None
                break
            except Exception as e:
                if conn is not None:
                    conn.close()
                conn = None
                if not reused or attempt:  # a kept-open connection the server had closed: once more, new
                    d["err"] = str(e)[:200]
                    break
        d["ms"] = int((time.time() - t0) * 1000)
        rec("relay", {"m": method, "p": path, "st": d.get("st"), "err": d.get("err"), "ms": d["ms"], "t": t0})
        out(d)
def balancer():  # after the walk (or "B 0"): fade the offsets out on the gait's last angles, 50 per second
    while True:
        time.sleep(0.02)
        b, last, now = BAL["b"], BAL["last"], time.time()
        if b is None or ((now - WD["gait"] < 0.25 or TURN["playing"]) and not BAL["off"]):
            continue
        if b.pitch or b.roll:
            offs = b.fade(now)
            if last is not None:
                bal_publish(last, offs, 20, now)
        elif BAL["off"]:
            BAL["b"], BAL["off"] = None, False
            rec("balance", {"on": False, "t": now})
def watchdog():  # the laptop silent while the robot walks or turns (Wi-Fi gone): stop it here, the mobile app's way
    while True:
        time.sleep(0.1)
        now = time.time()
        silent = now - WD["rx"]
        if silent > LINGER_S:
            rec("linger_exit", {"silent": silent})
            with flock:
                logf.flush()
            os._exit(0)
        if not WATCHDOG_S or silent < WATCHDOG_S:
            WD["stop"], WD["exit"] = 0.0, False
            continue
        moving = now - WD["gait"] < 0.5 or WD["turn"] is not None
        if not moving:
            continue
        act = None
        if not WD["stop"]:  # the app's stop: every value 0 (the walk winds down, a turn ends with ResetSwitch)
            act, WD["stop"] = "stop", now
            body = json.dumps({"speed_v": 0, "speed_h": 0, "steps": 0, "period": 0, "timestamp": int(now * 1000), "wave": False})
            call = ("PUT", "motions/gait", body)
        elif now - WD["stop"] > 3.0 and not WD["exit"]:  # still walking 3 s later: exit the gait
            act, WD["exit"] = "exit", True
            call = ("DELETE", "motions/gait", json.dumps({"timestamp": 0}))
        if act:
            try:
                st = http(*call, timeout=3.0)[1]
            except Exception as e:
                st = str(e)[:120]
            rec("watchdog", {"silent": round(silent, 2), "action": act, "reply": st, "turn": WD["turn"], "t": now})
            out({"wd": act, "silent": round(silent, 2), "reply": st, "t": now})
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
for fn in (writer, info, encoders, monitor, flusher, relay, watchdog, balancer, turner):
    th = threading.Thread(target=fn)
    th.daemon = True
    th.start()
rec("start", {"node": NODE, "session": SESSION, "enc_hz": ENC_HZ, "watchdog_s": WATCHDOG_S})
out({"ready": True, "subscribers": pub.get_num_connections(), "node": NODE, "log": LOG_PATH})
for line in iter(sys.stdin.readline, ""):
    WD["rx"] = time.time()  # anything from the laptop (H: its heartbeat) keeps the watchdog quiet
    p = line.split()
    if not p or p[0] == "Q":
        break
    if p[0] == "R" and len(p) >= 4:  # REST through the link: R id METHOD path [json body]
        rest = line.rstrip("\n").split(" ", 4)
        relq.put((int(p[1]), p[2], p[3], rest[4] if len(rest) > 4 and rest[4] else None))
        continue
    if p[0] == "TF" and len(p) >= 2:  # the turn motions: TF {name: [[17 raw values, runtime ms, hold ms], ...]}
        TURN["frames"] = json.loads(line.split(" ", 1)[1])
        out({"turn_frames": sorted(TURN["frames"])})
        continue
    if p[0] == "T" and len(p) == 2:  # T TurnLeft / T TurnRight: play it until T 0 (then ResetSwitch); T !: stop now
        if p[1] == "!":
            TURN["want"], TURN["abort"] = None, TURN["playing"]
        else:
            TURN["want"] = None if p[1] == "0" else p[1]
        continue
    if p[0] == "B" and len(p) >= 2:  # balance on the firmware walk: B 1 [gains json] / B 0
        if p[1] == "1":
            gains = json.loads(line.split(" ", 2)[2]) if len(p) > 2 else {}
            BAL["b"], BAL["off"] = Balance(**gains), False
            rec("balance", {"on": True, "gains": BAL["b"].g, "t": time.time()})
        elif BAL["b"] is not None:
            BAL["off"] = True
        out({"balance": p[1] == "1"})
        continue
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


def _node_source():
    """What runs on the robot: yanshee_sim/balance.py, the servo table, then REMOTE."""
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "balance.py")) as f:
        bal = f.read()
    return (bal + "\nSERVO_INDEX = %r\nSERVO_LIMITS = %r\n" % ({n: i for i, n in enumerate(NAMES)}, dict(LIMITS))
            + REMOTE)


NODE_SOURCE = _node_source()


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
    on_frame(degrees, ms, src, ours, rt=robot time or None) is called for every frame on the robot: ours locally as
    they are sent, the firmware's as they arrive (late and bunched by the link: rt says when the robot published it). log(kind, **data) gets the laptop-side records: link (the robot log's path), info,
    clock (offset robot - laptop and round trip, per ping). The robot's readings are in its own log (robot_log).
    rest(method, path, body) makes a REST call on the robot through the link; a heartbeat keeps the robot's watchdog
    quiet (watchdog_s, 0 = off), on_watchdog(dict) hears when it stopped the robot; silence is how long the robot has
    said nothing. launcher (tests): args -> the command running the node, instead of ssh to host."""

    def __init__(self, host, user="pi", password="raspberry", log=None, on_frame=None, timeout=20.0, enc_hz=5.0,
                 session=None, on_motion=None, watchdog_s=WATCHDOG_S, on_watchdog=None, rest_port=9090, launcher=None):
        env = dict(os.environ, SSHPASS=password)
        self._ssh = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8", f"{user}@{host}"]
        self._env, self.host, self.user = env, host, user
        self.session = session or time.strftime("%Y%m%d_%H%M%S")
        args = [str(float(enc_hz)), self.session, ROBOT_LOG_DIR, str(int(rest_port)), str(float(watchdog_s))]
        if launcher is None:
            up = subprocess.run(self._ssh + [f"cat > {REMOTE_PATH}"], input=NODE_SOURCE, text=True, env=env, timeout=30,
                                capture_output=True)
            if up.returncode:
                raise RuntimeError(f"cannot upload the stream script to the robot: {up.stderr.strip()[:200]}")
            # one node at a time: end any left from a link that died without closing ([p]: not this shell itself)
            cmd = self._ssh[:-1] + SSH_ALIVE + self._ssh[-1:] + [
                ROS_ENV + f"pkill -f '[p]ython2 -u {REMOTE_PATH}'; exec python2 -u {REMOTE_PATH} " + " ".join(args)]
        else:
            cmd = launcher(args)
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     text=True, bufsize=1, env=env)
        self.on_watchdog, self._rx_t, self._rid, self._pending = on_watchdog, time.monotonic(), 0, {}
        self.turning = False  # the node is playing a turn of its own (turn())
        self.log, self.on_frame, self._imu, self._imu_t, self.ready = log, on_frame, None, 0.0, threading.Event()
        self.on_motion = on_motion  # on_motion(robot path of the .hts, "start" | "finish", rt=robot time)
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
        threading.Thread(target=self._heartbeat, name="robot-stream-heartbeat", daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self._rx_t = time.monotonic()
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
        elif "rr" in d:
            slot = self._pending.get(d["rr"])
            if slot is not None:
                slot[1] = d
                slot[0].set()
        elif "turn" in d:
            self.turning = d["turn"] == "start"
            if self.log:
                self.log("node_turn", **{k: d.get(k) for k in ("turn", "name", "why", "t")})
        elif "wd" in d:
            if self.log:
                self.log("watchdog", **{k: d.get(k) for k in ("wd", "silent", "reply", "t")})
            if self.on_watchdog:
                self.on_watchdog(d)
        elif "e" in d:
            self._imu, self._imu_t = d, time.monotonic()
        elif "c" in d:
            if self.on_frame:
                self.on_frame(to_degrees(d["c"]), d.get("ms") or 20, d.get("src"), False, rt=d.get("t"))
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
        elif "mo" in d:
            if self.on_motion:
                self.on_motion(d["mo"], d.get("st"), rt=d.get("t"))
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

    def _heartbeat(self):
        while self.alive:
            try:
                with self.lock:
                    if not self.alive:
                        return
                    self.proc.stdin.write("H\n")
                    self.proc.stdin.flush()
            except (OSError, ValueError):
                return
            time.sleep(HEARTBEAT_S)

    @property
    def silence(self):
        """Seconds since the robot last sent anything (IMU 19 per second, so normally under 0.1)."""
        return time.monotonic() - self._rx_t

    def rest(self, method, path, body=None, timeout=4.0):
        """A REST call on the robot (its server on port 9090 there) through the link: path as in rest.py
        ('motions/gait'), body a JSON string or None. Returns (status, content bytes, content type, ms on the robot).
        Raises RuntimeError when the link is down or quiet (then use the direct connection) or on no reply."""
        if not self.alive or self.silence > RELAY_SILENT_S:
            raise RuntimeError("the robot link is down or quiet")
        if body is not None and "\n" in body:
            raise ValueError("body must be one line of JSON")
        slot = [threading.Event(), None]
        with self.lock:
            self._rid += 1
            rid = self._rid
            self._pending[rid] = slot
            self.proc.stdin.write(f"R {rid} {method} {path}" + (f" {body}" if body is not None else "") + "\n")
            self.proc.stdin.flush()
        try:
            if not slot[0].wait(timeout):
                raise RuntimeError(f"no reply through the robot link in {timeout:g} s")
        finally:
            self._pending.pop(rid, None)
        d = slot[1]
        if d.get("err"):
            raise RuntimeError(f"the robot's REST server: {d['err']}")
        return d["st"], d.get("b", "").encode("utf-8"), d.get("ct", ""), d.get("ms")

    def load_turns(self, turns):
        """The turn motions the node plays itself (turn()): {name: [{"angles": {servo: deg}, "runtime": ms,
        "hold": ms}, ...]} as in model/appwalk.json (TurnLeft, TurnRight, ResetSwitch: hal's own .hts files)."""
        frames = {n: [[to_units(f["angles"]), int(f["runtime"]), int(f.get("hold", 0))] for f in fs] for n, fs in turns.items()}
        self._write("TF " + json.dumps(frames))

    def turn(self, name):
        """Play TurnLeft / TurnRight on the robot from the node, looped until turn(None) (then its ResetSwitch), with
        the balance if it is on; turn("!") stops it at once (STOP). turning: the node is playing one."""
        self._write(f"T {name or 0}")

    def _write(self, line):
        with self.lock:
            if not self.alive:
                raise RuntimeError("the robot stream has ended")
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()

    def set_balance(self, on, gains=None):
        """Balance (yanshee_sim.balance) on the robot's firmware walk: on the robot, the gait's own frames get ankle
        corrections from its IMU (no Wi-Fi in the loop); off fades them out."""
        with self.lock:
            if not self.alive:
                raise RuntimeError("the robot stream has ended")
            self.proc.stdin.write("B 1 " + json.dumps(gains or {}) + "\n" if on else "B 0\n")
            self.proc.stdin.flush()

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


def hts_copy(robot_path, host, user="pi", password="raspberry"):
    """A local copy of one of the robot's .hts motion files (e.g. /opt/yanshee/lib/move/hts/TurnLeft.hts): the ones
    tools/robot_checks.py inspect copied (robot_dump/inspect_*/hts/), else fetched from the robot once into
    robot_dump/hts_cache/. Both named after the full path, / -> __."""
    flat = robot_path.strip("/").replace("/", "__")
    cache = os.path.join(ROOT, "robot_dump", "hts_cache")
    hits = glob.glob(os.path.join(ROOT, "robot_dump", "inspect_*", "hts", flat)) + glob.glob(os.path.join(cache, flat))
    if hits:
        return hits[0]
    got = fetch(host, robot_path, cache, user=user, password=password)
    os.replace(got, os.path.join(cache, flat))
    return os.path.join(cache, flat)


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
