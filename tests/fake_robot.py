#!/usr/bin/env python3
"""Fake Yanshee REST server for tests and GUI work without hardware (stdlib only).

    python3 tests/fake_robot.py --port 9191                 # then connect to 127.0.0.1:9191
    python3 tests/fake_robot.py --port 9191 --battery 20    # low battery: motions refused with code 6
    python3 tests/fake_robot.py --port 9191 --refuse        # code 1 on every motion/servo command
    python3 tests/fake_robot.py --port 9191 --slow 5        # every reply delayed 5 s

From Python:

    import fake_robot
    srv = fake_robot.start(9191, battery=80)    # serves in a background thread
    srv.state.log                               # [(method, path, query, body), ...]
    srv.state.dead = True                       # stops answering (connections dropped) but keeps the port
    srv.stop()

Behaviour: servos move linearly over their runtime; motions, gait, music, TTS and vision tasks go
run -> idle after a duration; the motion list is the one captured from a real robot
(tests/data/motion_list_real.json). Only binds where told (default 127.0.0.1).
"""
import argparse
import base64
import io
import json
import math
import os
import random
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from yanshee_sim.servos import LIMITS, NAMES, RESET_POSE  # noqa: E402

MOTION_LIST = os.path.join(HERE, "data", "motion_list_real.json")
FIRST_LAYERS_MOTION = "Track_TurnR"  # the captured list is hts motions, then layered (v2) ones
DANCES = {"WakaWaka", "LittleApple", "GangnamStyle", "HappyBirthday", "SorrySorry", "WeAreTakingOff",
          "SweetAndSour", "SiegeOfTroy", "CombinationOfSongs"}
PARAMETRIC = {"raise", "stretch", "come on", "wave", "bend", "turn around", "walk", "head", "reset", "crouch", "bow"}
SPEED_FACTOR = {"very slow": 2.0, "slow": 1.5, "normal": 1.0, "fast": 0.75, "very fast": 0.5}
LED_COLORS = {"button": {"white", "red", "green", "blue", "yellow", "purple", "cyan"}, "camera": {"red", "green", "blue"}}
LED_MODES = {"button": {"on", "off", "blink", "breath"}, "camera": {"on", "off", "blink"}}
CANNED_VISION = {
    ("face", "recognition"): {"recognition": {"name": "visitor"}},
    ("face", "quantity"): {"quantity": 1},
    ("face", "age"): {"analysis": {"age": 30}},
    ("face", "age_group"): {"analysis": {"group": "youth"}},
    ("face", "gender"): {"analysis": {"gender": "female"}},
    ("face", "expression"): {"analysis": {"expression": "happy"}},
    ("face", "mask"): {"analysis": {"mask": "unmasked"}},
    ("face", "glass"): {"analysis": {"glass": "noglass"}},
    ("face", "tracking"): {"tracking": {"x": 320, "y": 240}},
    ("hand", "gesture"): {"gesture": "ok"},
    ("color", "color_detect"): {"color": [{"name": "red"}]},
    ("object", "recognition"): {"recognition": {"name": "cup"}},
}
_JPEG_B64 = (  # 32x24 fallback photo when Pillow is missing
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA0JCgsKCA0LCgsODg0PEyAVExISEyccHhcgLikxMC4pLSwzOko+MzZGNywtQFdBRkxOUlNSMj5a"
    "YVpQYEpRUk//2wBDAQ4ODhMREyYVFSZPNS01T09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0//wAAR"
    "CAAYACADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEG"
    "E1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWG"
    "h4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEB"
    "AQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYk"
    "NOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0"
    "tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDDq19guv8Anl/48P8AGqtdLWuY42pheXkS"
    "1vv8hYDCQxHNzt6W2+ZifYLr/nl/48P8aq10tc1Rl2NqYrm50tLbfMMfhIYfl5G9b7/IKtfb7r/nr/46P8KKK9CdKnU+OKfqjihVnD4G16B9"
    "vuv+ev8A46P8Kq0UUQpU6fwRS9EE6s5/G2/U/9k=")


def make_jpeg(resolution="640x480", text=""):
    """A small test-card JPEG (Pillow if available)."""
    try:
        from PIL import Image, ImageDraw
        w, h = (int(v) for v in resolution.lower().split("x"))
        w, h = max(16, min(w, 640)), max(12, min(h, 480))
        im = Image.new("RGB", (w, h), (40, 90, 160))
        d = ImageDraw.Draw(im)
        d.rectangle([w // 4, h // 4, 3 * w // 4, 3 * h // 4], fill=(230, 120, 30))
        d.text((8, 8), text or "fake yanshee", fill=(255, 255, 255))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=70)
        return buf.getvalue()
    except Exception:
        return base64.b64decode(_JPEG_B64)


def _load_motions():
    with open(MOTION_LIST, encoding="utf-8") as f:
        names = json.load(f)
    k = names.index(FIRST_LAYERS_MOTION) if FIRST_LAYERS_MOTION in names else len(names)
    return names[:k], names[k:]


class FakeState:
    """Everything the fake robot remembers. Times are time.monotonic() seconds."""

    def __init__(self, battery=90, refuse=False, slow=0.0, addons=True, motion_seconds=3.0, vision_seconds=1.0,
                 music_seconds=5.0):
        self.lock = threading.RLock()
        self.battery, self.refuse, self.slow, self.addons = battery, refuse, slow, addons
        self.motion_seconds, self.vision_seconds, self.music_seconds = motion_seconds, vision_seconds, music_seconds
        self.log = deque(maxlen=2000)
        self.overrides = {}  # test hook: (method, path) -> reply dict, or (bytes, content type) for a raw reply
        self.dead = False  # test hook: drop every connection without a reply (a robot that died; port stays bound)
        now = time.monotonic()
        self.servos = {n: [float(d), float(d), now, now] for n, d in RESET_POSE.items()}  # start, target, t0, t1
        self.modes = {n: "work" for n in NAMES}
        self.hts, self.layers = _load_motions()
        self.motion = {"name": "", "status": "idle", "timestamp": 0, "end": 0.0, "start": 0.0, "version": "v1",
                       "left": 0.0}
        self.gait = {"status": 8, "timestamp": 0, "end": None, "start": 0.0}
        self.music = {"name": "", "status": "idle", "end": 0.0}
        self.tts = {"status": "idle", "timestamp": 0, "end": 0.0, "text": ""}
        self.voice = {k: {"status": "idle", "timestamp": 0, "end": 0.0} for k in ("asr", "iat")}
        self.vision = {}  # (option, type) -> {"timestamp", "end"}
        self.qr = {"status": "idle", "end": 0.0}
        self.leds = {"button": {"color": "blue", "mode": "breath"}, "camera": {"color": "green", "mode": "on"}}
        self.volume, self.language, self.fall = 50, "en", False
        self.photos, self.samples, self.tags = {}, set(), {}
        self.stream = False

    # -- helpers ----------------------------------------------------------------
    def angle(self, n, now):
        a, b, t0, t1 = self.servos[n]
        s = 1.0 if now >= t1 else max(0.0, (now - t0) / (t1 - t0))
        deg = a + (b - a) * s
        if self._motion_running(now) and n in ("RightShoulderFlex", "LeftShoulderFlex"):  # show activity
            deg += 25 * math.sin(2 * math.pi * (now - self.motion["start"]) / 1.5) * (1 if n[0] == "R" else -1)
        if self._gait_running(now) and n in ("RightKneeFlex", "LeftKneeFlex"):
            deg += 8 * math.sin(2 * math.pi * (now - self.gait["start"]))
        lo, hi = LIMITS[n]
        return int(round(min(max(deg, lo), hi)))

    def move_by_hand(self, angles):
        """Test hook: pose servos instantly, as if someone moved them in program (limp) mode."""
        now = time.monotonic()
        with self.lock:
            for n, d in angles.items():
                self.servos[n] = [float(d), float(d), now, now]

    def _motion_running(self, now):
        m = self.motion
        return m["status"] == "run" and now < m["end"]

    def _gait_running(self, now):
        g = self.gait
        return 0 <= g["status"] <= 2 and (g["end"] is None or now < g["end"])

    def refresh(self, now):
        m = self.motion
        if m["status"] == "run" and now >= m["end"]:
            m["status"] = "idle"
        g = self.gait
        if 0 <= g["status"] <= 2 and g["end"] is not None and now >= g["end"]:
            g["status"] = 8
        for d in (self.music, self.tts, *self.voice.values()):
            if d["status"] == "run" and now >= d["end"]:
                d["status"] = "idle"

    def motion_names(self):
        return {n.lower() for n in self.hts + self.layers} | PARAMETRIC


def ok(data=None, **extra):
    return dict({"code": 0, "data": {} if data is None else data, "msg": "success"}, **extra)


def err(code, msg, **extra):
    return dict({"code": code, "data": {}, "msg": msg}, **extra)


def _first(q, key, default=None):
    v = q.get(key)
    return v[0] if v else default


def _voice_reply(status, ts, data):
    return {"code": 0, "status": status, "timestamp": ts, "data": json.dumps(data) + "\x00\x00", "msg": "success"}


# -- endpoint handlers: (state, now, query, body) -> dict | (bytes, content type) ------------------------
def h_battery(s, now, q, b):
    return ok({"percent": int(s.battery), "charging": 0, "voltage": int(6800 + 14 * s.battery)})


def h_versions(s, now, q, b):
    info = {"core": "fake-1.0 (yanshee_sim tests)", "servo": "fake-servo-1.0", "sn": "FAKE00000001"}
    t = _first(q, "type", "core")
    return ok({t: info[t]}) if t in info else err(1, f"unknown version type '{t}'")


def h_mode(s, now, q, b):
    return ok({"energy_saving_mode": False, "calibration_mode": False})


def h_get_volume(s, now, q, b):
    return ok({"volume": s.volume})


def h_set_volume(s, now, q, b):
    v = b.get("volume")
    if not isinstance(v, int) or not 0 <= v <= 100:
        return err(1, "volume must be an int 0-100")
    s.volume = v
    return ok()


def h_get_language(s, now, q, b):
    return ok({"language": s.language})


def h_set_language(s, now, q, b):
    if b.get("language") not in ("zh", "en"):
        return err(1, "language must be zh or en")
    s.language = b["language"]
    return ok()


def h_get_led(s, now, q, b):
    return ok([{"type": t, **v} for t, v in s.leds.items()])


def h_set_led(s, now, q, b):
    t, c, m = b.get("type"), b.get("color"), b.get("mode")
    if t not in LED_COLORS or c not in LED_COLORS[t] or m not in LED_MODES[t]:
        return err(1, f"bad led setting {t}/{c}/{m}")
    s.leds[t] = {"color": c, "mode": m}
    return ok()


def h_get_fall(s, now, q, b):
    return ok({"enable": s.fall})


def h_set_fall(s, now, q, b):
    s.fall = bool(b.get("enable"))
    return ok()


def h_get_angles(s, now, q, b):
    names = q.get("names") or NAMES
    return ok({n: s.angle(n, now) for n in names if n in s.servos})


def h_set_angles(s, now, q, b):
    if s.refuse:
        return err(1, "refused (fake robot --refuse)")
    angles, runtime = b.get("angles") or {}, b.get("runtime", 200)
    if not isinstance(angles, dict) or not isinstance(runtime, int):
        return err(1, "bad servo command")
    runtime = min(max(runtime, 200), 4000)
    res = {}
    for n, d in angles.items():
        if n not in s.servos or not isinstance(d, (int, float)) or s.modes[n] != "work":
            res[n] = False
            continue
        cur = s.angle(n, now)
        lo, hi = LIMITS[n]
        s.servos[n] = [float(cur), float(min(max(d, lo), hi)), now, now + runtime / 1000]
        res[n] = True
    return ok(res)


def h_get_mode(s, now, q, b):
    return ok({n: s.modes[n] for n in (q.get("names") or NAMES) if n in s.modes})


def h_set_mode(s, now, q, b):
    if s.refuse:
        return err(1, "refused (fake robot --refuse)")
    mode = b.get("mode")
    if mode not in ("work", "program"):
        return err(1, "mode must be work or program")
    res = {}
    for item in b.get("servos") or []:
        n = item.get("name") if isinstance(item, dict) else None
        if n in s.modes:
            cur = s.angle(n, now)
            s.servos[n] = [float(cur), float(cur), now, now]  # holds where it is when switching
            s.modes[n] = mode
        res[n] = n in s.modes
    return ok(res)


def h_motion_list(s, now, q, b):
    def entries(names, ext):
        return [{"name": n + ext, "music": n in DANCES or not n.isascii()} for n in names]
    return ok({"system_hts_motions": entries(s.hts, ".hts"), "system_layers_motions": entries(s.layers, ".layers"),
               "user_hts_motions": [], "user_layers_motions": []})


def h_motion_control(s, now, q, b):
    op, motion = b.get("operation"), b.get("motion") or {}
    m = s.motion
    if op in ("start", "start_async"):  # start_async: what the Yanshee mobile app sends (captured 29 Oct 2026)
        if s.refuse:
            return err(1, "refused (fake robot --refuse)")
        if s.battery < 30:
            return err(6, "lowbattery")
        name, speed, repeat = motion.get("name", ""), motion.get("speed", "normal"), motion.get("repeat", 1)
        if name.lower() not in s.motion_names():
            return err(2, f"motion '{name}' not found")
        if speed not in SPEED_FACTOR or not isinstance(repeat, int) or repeat < 0:  # 0: the mobile app's turns
            return err(1, "bad speed or repeat")
        total = s.motion_seconds * SPEED_FACTOR[speed] * repeat
        s.motion.update(name=name, status="run", timestamp=b.get("timestamp", 0), start=now, end=now + total,
                        version=b.get("version", "v1"))
        return ok({"total_time": int(round(total * 1000))})
    if op == "stop":
        m["status"] = "idle"
        return ok()
    if op == "pause" and m["status"] == "run":
        m.update(status="pause", left=max(0.0, m["end"] - now))
        return ok()
    if op == "resume" and m["status"] == "pause":
        m.update(status="run", end=now + m["left"])
        return ok()
    if op in ("pause", "resume"):
        return ok()
    return err(1, f"unknown operation '{op}'")


def h_motion_state(s, now, q, b):
    m = s.motion
    return ok({"name": m["name"], "status": m["status"], "timestamp": m["timestamp"]})


def h_layer_state(s, now, q, b):
    m = s.motion
    if m["version"] != "v2" or not m["name"]:
        return ok([])
    return ok([{"name": m["name"] + ".layers", "status": m["status"], "timestamp": m["timestamp"]}])


def h_gait(s, now, q, b):
    if s.refuse:
        return err(1, "refused (fake robot --refuse)")
    if s.battery < 30:
        return err(6, "lowbattery")
    vals = [b.get(k, 0) for k in ("speed_v", "speed_h", "steps", "period")]
    if not all(isinstance(v, int) for v in vals) or not (-5 <= vals[0] <= 5 and -5 <= vals[1] <= 5
                                                          and vals[2] >= 0 and 0 <= vals[3] <= 5):
        return err(1, "bad gait parameters")
    g = s.gait
    if vals[3] == 0:
        g.update(status=8, end=None)
    else:
        g.update(status=2, timestamp=b.get("timestamp", 0), start=now,
                 end=now + vals[2] * 0.5 * vals[3] if vals[2] else None)
    return ok()


def h_gait_state(s, now, q, b):
    return ok({"status": s.gait["status"], "timestamp": s.gait["timestamp"]})


def h_gait_exit(s, now, q, b):
    s.gait.update(status=8, end=None)
    return ok()


def h_music_list(s, now, q, b):
    return ok({"music": [{"name": n + ".mp3"} for n in sorted(DANCES)]})


def h_music(s, now, q, b):
    if b.get("operation") == "start":
        name = b.get("name", "SorrySorry.mp3")
        s.music.update(name=name, status="run", end=now + s.music_seconds)
        return ok()
    if b.get("operation") == "stop":
        s.music["status"] = "idle"
        return ok()
    return err(1, "operation must be start or stop")


def h_music_state(s, now, q, b):
    return ok({"name": s.music["name"], "status": s.music["status"]})


def h_tts(s, now, q, b):
    text = b.get("tts", "")
    if not isinstance(text, str):
        return err(1, "tts must be a string")
    s.tts.update(status="run", timestamp=b.get("timestamp", 0), text=text, end=now + 0.5 + 0.06 * len(text))
    return ok()


def h_tts_state(s, now, q, b):
    ts = _first(q, "timestamp")
    t = s.tts
    if ts is not None and str(t["timestamp"]) != ts:
        return _voice_reply("idle", int(ts) if ts.lstrip("-").isdigit() else ts, {})
    return _voice_reply(t["status"], t["timestamp"], {"text": t["text"]})


def h_tts_stop(s, now, q, b):
    s.tts["status"] = "idle"
    return ok()


def _voice_start(kind):
    def h(s, now, q, b):
        s.voice[kind].update(status="run", timestamp=b.get("timestamp", 0), end=now + s.vision_seconds)
        return ok()
    return h


def _voice_get(kind, result):
    def h(s, now, q, b):
        v = s.voice[kind]
        return _voice_reply(v["status"], v["timestamp"], result if v["status"] == "idle" else {})
    return h


def _voice_stop(kind):
    def h(s, now, q, b):
        s.voice[kind]["status"] = "idle"
        return ok()
    return h


def h_vision_control(s, now, q, b):
    key = (b.get("option"), b.get("type"))
    if key not in CANNED_VISION:
        return err(1, f"unknown vision task {key[0]}/{key[1]}")
    if b.get("operation") == "start":
        s.vision[key] = {"timestamp": b.get("timestamp", 0), "end": now + s.vision_seconds}
        return ok()
    if b.get("operation") == "stop":
        s.vision.pop(key, None)
        return ok()
    return err(1, "operation must be start or stop")


def h_vision_result(s, now, q, b):
    key = (_first(q, "option"), _first(q, "type"))
    task = s.vision.get(key)
    if task is None:
        return {"code": 0, "type": key[1], "status": "idle", "timestamp": 0, "data": {}, "msg": "success"}
    if now < task["end"]:
        return {"code": 0, "type": key[1], "status": "run", "timestamp": task["timestamp"], "data": {}, "msg": "success"}
    return {"code": 0, "type": key[1], "status": "idle", "timestamp": task["timestamp"],
            "data": CANNED_VISION[key], "msg": "success"}


def h_qr(s, now, q, b):
    if b.get("operation") == "start":
        s.qr.update(status="run", end=now + s.vision_seconds)
    else:
        s.qr["status"] = "idle"
    return ok()


def h_qr_state(s, now, q, b):
    done = s.qr["status"] == "run" and now >= s.qr["end"]
    return {"code": 0, "status": "idle" if done or s.qr["status"] == "idle" else "run",
            "data": {"contents": ["https://example.org/yanshee"] if done else []}, "msg": "success"}


def h_take_photo(s, now, q, b):
    res = b.get("resolution", "640x480")
    name = time.strftime("img_%Y%m%d_%H%M%S") + f"_{random.randint(1000, 9999)}.jpg"
    s.photos[name] = make_jpeg(res, name)
    return ok({"name": name})


def h_get_photo(s, now, q, b):
    name = _first(q, "body", "")
    if name not in s.photos:
        return err(1, f"photo '{name}' not found")
    return s.photos[name], "image/jpeg"


def h_delete_photo(s, now, q, b):
    return ok() if s.photos.pop(b.get("name"), None) is not None else err(1, "photo not found")


def h_photo_list(s, now, q, b):
    return ok([{"name": n} for n in s.photos])


def h_samples(s, now, q, b):
    return ok([{"name": n} for n in sorted(s.samples)])


def h_upload_sample(s, now, q, b):
    raw = b.get("_raw", b"")
    marker = b'filename="'
    i = raw.find(marker)
    if i < 0 or b"\xff\xd8" not in raw:
        return err(1, "expected a multipart JPEG upload in field 'file'")
    name = raw[i + len(marker):raw.index(b'"', i + len(marker))].decode("utf-8", "replace")
    s.samples.add(name)
    return ok()


def h_delete_sample(s, now, q, b):
    s.samples.discard(b.get("name"))
    return ok()


def h_get_tags(s, now, q, b):
    return ok([{"resources": r, "tags": t} for t, r in s.tags.items()])


def h_set_tag(s, now, q, b):
    res, tag = b.get("resources"), b.get("tags")
    if not isinstance(res, list) or not tag or any(r not in s.samples for r in res):
        return err(1, "upload the samples before tagging them")
    s.tags.setdefault(tag, []).extend(r for r in res if r not in s.tags.get(tag, []))
    return ok()


def h_delete_tag(s, now, q, b):
    tag = b.get("tags")
    if tag not in s.tags:
        return err(1, f"tag '{tag}' not found")
    removed = s.tags.pop(tag)
    if b.get("mode", "all") == "all":
        s.samples.difference_update(removed)
    return ok()


def h_stream_open(s, now, q, b):
    if s.stream:
        return err(20003, "stream already open", streamUrl="http://127.0.0.1:8000")
    s.stream = True
    return ok()


def h_stream_close(s, now, q, b):
    s.stream = False
    return ok()


ADDONS = [{"id": 1, "slot": 1, "type": "infrared", "version": 1}, {"id": 2, "slot": 2, "type": "ultrasonic", "version": 1},
          {"id": 3, "slot": 3, "type": "touch", "version": 1}, {"id": 4, "slot": 4, "type": "pressure", "version": 1},
          {"id": 5, "slot": 5, "type": "environment", "version": 1}]


def h_sensors_list(s, now, q, b):
    return ok({"sensors": ADDONS if s.addons else []})


def h_gyro(s, now, q, b):
    n = lambda: random.uniform(-0.01, 0.01)  # noqa: E731
    # upright, as measured: accel about +1 g on the up (y) axis, euler-x about 92 (0 face down, 180 on the back)
    return ok({"gyro": [{"id": 1, "gyro-x": n(), "gyro-y": n(), "gyro-z": n(), "accel-x": n(), "accel-y": 1.0 + n(),
                         "accel-z": n(), "compass-x": 0.2, "compass-y": -0.1, "compass-z": 0.4,
                         "euler-x": 89.5 + n(), "euler-y": 0.3 + n(), "euler-z": 12.0 + n()}]})


def _addon(kind, value):
    def h(s, now, q, b):
        if not s.addons:
            return ok({kind: []})
        a = next(x for x in ADDONS if x["type"] == kind)
        return ok({kind: [{"id": a["id"], "slot": a["slot"], **value()}]})
    return h


ROUTES = {
    ("GET", "devices/battery"): h_battery,
    ("GET", "devices/versions"): h_versions,
    ("GET", "devices/mode"): h_mode,
    ("GET", "devices/volume"): h_get_volume,
    ("PUT", "devices/volume"): h_set_volume,
    ("GET", "devices/languages"): h_get_language,
    ("PUT", "devices/languages"): h_set_language,
    ("GET", "devices/led"): h_get_led,
    ("PUT", "devices/led"): h_set_led,
    ("GET", "devices/fall_management"): h_get_fall,
    ("PUT", "devices/fall_management"): h_set_fall,
    ("GET", "servos/angles"): h_get_angles,
    ("PUT", "servos/angles"): h_set_angles,
    ("GET", "servos/mode"): h_get_mode,
    ("PUT", "servos/mode"): h_set_mode,
    ("GET", "motions/list"): h_motion_list,
    ("PUT", "motions"): h_motion_control,
    ("GET", "motions"): h_motion_state,
    ("GET", "motions/all"): h_layer_state,
    ("PUT", "motions/gait"): h_gait,
    ("GET", "motions/gait"): h_gait_state,
    ("DELETE", "motions/gait"): h_gait_exit,
    ("GET", "media/music/list"): h_music_list,
    ("PUT", "media/music"): h_music,
    ("GET", "media/music"): h_music_state,
    ("PUT", "voice/tts"): h_tts,
    ("GET", "voice/tts"): h_tts_state,
    ("DELETE", "voice/tts"): h_tts_stop,
    ("PUT", "voice/asr"): _voice_start("asr"),
    ("GET", "voice/asr"): _voice_get("asr", {"question": "你好", "answer": "你好，我是Yanshee"}),
    ("DELETE", "voice/asr"): _voice_stop("asr"),
    ("PUT", "voice/iat"): _voice_start("iat"),
    ("GET", "voice/iat"): _voice_get("iat", {"text": "hello robot"}),
    ("DELETE", "voice/iat"): _voice_stop("iat"),
    ("PUT", "visions"): h_vision_control,
    ("GET", "visions"): h_vision_result,
    ("PUT", "visions/QR"): h_qr,
    ("GET", "visions/QR"): h_qr_state,
    ("POST", "visions/photos"): h_take_photo,
    ("GET", "visions/photos"): h_get_photo,
    ("DELETE", "visions/photos"): h_delete_photo,
    ("GET", "visions/photos/list"): h_photo_list,
    ("GET", "visions/photosamples"): h_samples,
    ("POST", "visions/photosamples"): h_upload_sample,
    ("DELETE", "visions/photosamples"): h_delete_sample,
    ("GET", "visions/tags"): h_get_tags,
    ("PUT", "visions/tags"): h_set_tag,
    ("DELETE", "visions/tags"): h_delete_tag,
    ("POST", "visions/streams"): h_stream_open,
    ("DELETE", "visions/streams"): h_stream_close,
    ("GET", "sensors/list"): h_sensors_list,
    ("GET", "sensors/gyro"): h_gyro,
    ("GET", "sensors/infrared"): _addon("infrared", lambda: {"value": random.randint(240, 260)}),
    ("GET", "sensors/ultrasonic"): _addon("ultrasonic", lambda: {"value": random.randint(290, 310)}),
    ("GET", "sensors/touch"): _addon("touch", lambda: {"value": 0}),
    ("GET", "sensors/pressure"): _addon("pressure", lambda: {"value": random.randint(0, 3)}),
    ("GET", "sensors/environment"): _addon("environment", lambda: {"temperature": 24, "humidity": 45,
                                                                   "pressure": 101325}),
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):  # quiet
        pass

    def _reply(self, status, payload, ctype="application/json"):
        body = payload if isinstance(payload, bytes) else json.dumps(payload, ensure_ascii=False).encode()
        try:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):  # the client gave up (its timeout < --slow)
            self.close_connection = True

    def _handle(self, method):
        state = self.server.state
        if state.dead:
            self.close_connection = True
            return
        url = urlsplit(self.path)
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        if state.slow:
            time.sleep(state.slow)
        path = url.path[len("/v1/"):] if url.path.startswith("/v1/") else None
        query = parse_qs(url.query)
        body = {}
        if raw and "multipart" in (self.headers.get("Content-Type") or ""):
            body = {"_raw": raw}
        elif raw:
            try:
                body = json.loads(raw.decode("utf-8"))
            except ValueError:
                return self._reply(400, err(400, "body is not JSON"))
            if not isinstance(body, dict):
                return self._reply(400, err(400, "body must be a JSON object"))
        handler = ROUTES.get((method, path))
        if (method, path) in state.overrides:
            res = state.overrides[(method, path)]
            handler = lambda *a: res  # noqa: E731
        state.log.append((method, path or url.path, {k: v if len(v) > 1 else v[0] for k, v in query.items()},
                          {k: v for k, v in body.items() if k != "_raw"}))
        if handler is None:
            return self._reply(404, err(404, f"no such endpoint {method} {url.path}"))
        with state.lock:
            now = time.monotonic()
            state.refresh(now)
            res = handler(state, now, query, body)
        if isinstance(res, tuple):
            return self._reply(200, res[0], res[1])
        self._reply(200, res)

    def do_GET(self):
        self._handle("GET")

    def do_PUT(self):
        self._handle("PUT")

    def do_POST(self):
        self._handle("POST")

    def do_DELETE(self):
        self._handle("DELETE")


class FakeRobotServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        """A client that timed out and closed its socket is normal here (--slow): no traceback for it."""
        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)

    def __init__(self, port=9191, host="127.0.0.1", **options):
        self.state = FakeState(**options)
        super().__init__((host, port), Handler)
        self.thread = None

    @property
    def address(self):
        """'127.0.0.1:PORT', ready for YansheeRest / RealRobot."""
        return f"{self.server_address[0]}:{self.server_address[1]}"

    def stop(self):
        self.shutdown()
        self.server_close()
        if self.thread is not None:
            self.thread.join(timeout=2)


def start(port=9191, host="127.0.0.1", **options):
    """Serve in a daemon thread and return the server (options: battery, refuse, slow, addons, motion_seconds,
    vision_seconds, music_seconds). port=0 picks a free port; see server.address."""
    srv = FakeRobotServer(port, host, **options)
    srv.thread = threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    srv.thread.start()
    return srv


def stop(server):
    server.stop()


def main():
    ap = argparse.ArgumentParser(description="Fake Yanshee REST server (no hardware needed).")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9191)
    ap.add_argument("--battery", type=int, default=90, help="battery percent (below 30: motions refused, code 6)")
    ap.add_argument("--refuse", action="store_true", help="answer code 1 to every motion/servo command")
    ap.add_argument("--slow", type=float, default=0.0, metavar="SECONDS", help="delay every reply")
    ap.add_argument("--no-addons", action="store_true", help="no add-on sensors attached")
    ap.add_argument("--motion-seconds", type=float, default=3.0, help="duration of every built-in motion")
    ap.add_argument("--vision-seconds", type=float, default=1.0, help="time to a vision/voice result")
    args = ap.parse_args()
    srv = FakeRobotServer(args.port, args.host, battery=args.battery, refuse=args.refuse, slow=args.slow,
                          addons=not args.no_addons, motion_seconds=args.motion_seconds,
                          vision_seconds=args.vision_seconds)
    print(f"fake Yanshee on http://{srv.address}/v1/  (battery {args.battery}%"
          f"{', refusing' if args.refuse else ''}{f', {args.slow:g} s delay' if args.slow else ''}); Ctrl+C stops")
    try:
        srv.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
