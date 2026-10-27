"""Per-instance REST client for the Yanshee robot (http://HOST:9090/v1/...), with a timeout on every request.

    from yanshee_sim.rest import YansheeRest
    r = YansheeRest("192.168.3.13")          # or "127.0.0.1:9191" for tests/fake_robot.py
    r.battery()                               # {"code": 0, "data": {"percent": 87, ...}, "msg": "success"}
    r.set_angles({"NeckLR": 60}, runtime=800)
    jpeg = r.photo()

Endpoint paths and payloads follow the official YanAPI.py, but nothing here uses that module: there is
no global IP, so several robots can be driven from one process, and no call can hang forever.
Methods return the robot's JSON dict unchanged (check "code" == 0). Transport failures raise
RobotUnreachable; replies that are not JSON (or not the expected shape) raise RobotProtocolError.
"""
import http.client
import json
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

DEFAULT_PORT = 9090
STREAM_PORT = 8000
SPEEDS = ("very slow", "slow", "normal", "fast", "very fast")
VISION_TYPES = {  # option -> task types accepted by PUT /visions
    "face": ("recognition", "quantity", "age", "age_group", "gender", "expression", "mask", "glass", "tracking"),
    "hand": ("gesture",),
    "color": ("color_detect",),
    "object": ("recognition",),
}


class RobotError(Exception):
    """Base class for robot communication errors."""


class RobotUnreachable(RobotError, ConnectionError):
    """No answer: connection refused, timeout, DNS or network failure."""


class RobotProtocolError(RobotError):
    """The robot answered, but not with the JSON (or JPEG) the API documents."""


def split_host(host, default_port=DEFAULT_PORT):
    """'192.168.3.13' -> ('192.168.3.13', 9090); '127.0.0.1:9191' -> ('127.0.0.1', 9191)."""
    host = str(host).strip()
    for prefix in ("http://", "https://"):
        if host.startswith(prefix):
            host = host[len(prefix):]
    host = host.split("/", 1)[0]
    if not host:
        raise ValueError("robot address is empty")
    if host.count(":") == 1:
        name, port = host.split(":")
        if not port.isdigit() or not 0 < int(port) < 65536:
            raise ValueError(f"bad port in robot address '{host}'")
        return name, int(port)
    return host, default_port


def _decode_voice(res):
    """Voice GETs return data as a NUL-padded JSON string; decode it like YanAPI does."""
    data = res.get("data") if isinstance(res, dict) else None
    if isinstance(data, str):
        text = data.strip("\x00").strip()
        try:
            res["data"] = json.loads(text) if text else {}
        except ValueError:
            res["data"] = {"text": text}
    return res


class YansheeRest:
    """REST client for one robot. host is 'IP' or 'IP:PORT' (default port 9090); timeout in seconds."""

    def __init__(self, host, timeout=4.0):
        self.host, self.port = split_host(host)
        self.timeout = float(timeout)
        self.base = f"http://{self.host}:{self.port}/v1/"
        self.poll_interval = 0.3  # seconds between polls in vision/asr/iat

    def __repr__(self):
        return f"YansheeRest('{self.host}:{self.port}')"

    @property
    def stream_url(self):
        """MJPEG camera stream, served by the robot after stream_open()."""
        return f"http://{self.host}:{STREAM_PORT}/stream.mjpg"

    # -- transport --------------------------------------------------------------
    def _request(self, method, path, params=None, body=None, timeout=None, raw=False, data=None, headers=None):
        url = self.base + path
        if params:
            url += "?" + urllib.parse.urlencode(params, doseq=True)  # lists repeat the key, like requests
        hdrs = {"Content-Type": "application/json"}
        if body is not None:
            data = json.dumps(body).encode()
        if headers:
            hdrs.update(headers)
        req = urllib.request.Request(url, data=data, method=method, headers=hdrs)
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as resp:
                content, status, ctype = resp.read(), resp.status, resp.headers.get("Content-Type", "")
        except urllib.error.HTTPError as e:  # the robot answered with an HTTP error status
            try:
                content = e.read()
            except Exception:
                content = b""
            status, ctype = e.code, e.headers.get("Content-Type", "") if e.headers else ""
            try:
                res = json.loads(content.decode("utf-8"))
                if isinstance(res, dict):
                    return res
            except ValueError:
                pass
            raise RobotProtocolError(f"{method} {path}: HTTP {status} from {self.host}:{self.port}: "
                                     f"{content[:120]!r}") from None
        except (urllib.error.URLError, OSError) as e:  # refused, timeout, reset, DNS, no route
            reason = getattr(e, "reason", None) or e or type(e).__name__
            raise RobotUnreachable(f"{method} {path}: no answer from {self.host}:{self.port} ({reason})") from None
        except http.client.HTTPException as e:  # garbage instead of an HTTP reply
            raise RobotProtocolError(f"{method} {path}: bad HTTP reply from {self.host}:{self.port} "
                                     f"({type(e).__name__}: {e})") from None
        if raw:
            return content, ctype
        try:
            res = json.loads(content.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            raise RobotProtocolError(f"{method} {path}: reply from {self.host}:{self.port} is not JSON "
                                     f"(HTTP {status}, {ctype or 'no content type'}): {content[:120]!r}") from None
        if not isinstance(res, dict):
            raise RobotProtocolError(f"{method} {path}: expected a JSON object, got {type(res).__name__}")
        return res

    def _get(self, path, params=None, **kw):
        return self._request("GET", path, params=params, **kw)

    def _put(self, path, body, **kw):
        return self._request("PUT", path, body=body, **kw)

    def _post(self, path, body, **kw):
        return self._request("POST", path, body=body, **kw)

    def _delete(self, path, body=None, **kw):
        return self._request("DELETE", path, body=body, **kw)

    # -- device -----------------------------------------------------------------
    def battery(self):
        """data: {percent, charging (0/1), voltage (mV)}."""
        return self._get("devices/battery")

    def versions(self, type="core"):
        """type: core | servo | sn."""
        return self._get("devices/versions", {"type": type})

    def mode(self):
        """data: {energy_saving_mode, calibration_mode}."""
        return self._get("devices/mode")

    def get_volume(self):
        return self._get("devices/volume")

    def set_volume(self, volume):
        return self._put("devices/volume", {"volume": int(volume)})

    def get_language(self):
        return self._get("devices/languages")

    def set_language(self, language):
        """'zh' or 'en'."""
        return self._put("devices/languages", {"language": language})

    def get_led(self):
        """data: [{type: button|camera, color, mode}]."""
        return self._get("devices/led")

    def set_led(self, type, color, mode):
        """type 'button' (white/red/green/blue/yellow/purple/cyan; on/off/blink/breath)
        or 'camera' (red/green/blue; on/off/blink)."""
        return self._put("devices/led", {"type": type, "color": color, "mode": mode})

    def get_fall_management(self):
        return self._get("devices/fall_management")

    def set_fall_management(self, enable):
        return self._put("devices/fall_management", {"enable": bool(enable)})

    # -- servos -----------------------------------------------------------------
    def get_angles(self, names):
        """data: {name: degrees}."""
        return self._get("servos/angles", {"names": list(names)})

    def set_angles(self, angles, runtime=200):
        """Move {name: degrees} over runtime ms (the robot accepts 200-4000). Returns immediately."""
        return self._put("servos/angles", {"angles": {n: int(round(d)) for n, d in angles.items()},
                                           "runtime": int(runtime)})

    def get_servo_mode(self, names):
        """data: {name: work|program|unknown|nonsupport}."""
        return self._get("servos/mode", {"names": list(names)})

    def set_servo_mode(self, mode, names):
        """mode 'program' = limp and readable by hand (PRP), 'work' = holding position."""
        return self._put("servos/mode", {"mode": mode, "servos": [{"name": n} for n in names]})

    # -- motions ----------------------------------------------------------------
    def motion_list(self):
        """data: {system_hts_motions, system_layers_motions, user_hts_motions, user_layers_motions: [{name, music}]}."""
        return self._get("motions/list")

    def motion_control(self, operation, name="", version="v1", direction="", speed="normal", repeat=1, timestamp=0):
        """operation: start | pause | resume | stop. Start replies data.total_time (ms)."""
        motion = {"name": name, "repeat": int(repeat), "speed": speed}
        if direction:
            motion["direction"] = direction
        return self._put("motions", {"operation": operation, "motion": motion, "timestamp": int(timestamp),
                                     "version": version})

    def play_motion(self, name, direction="", speed="normal", repeat=1, version="v1", timestamp=None):
        """Start a built-in motion (v1 = .hts, v2 = layered). Returns immediately with data.total_time (ms)."""
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")
        ts = int(time.time() * 1000) if timestamp is None else timestamp
        return self.motion_control("start", name, version, direction, speed, repeat, ts)

    def motion_state(self):
        """data: {name, status: run|idle, timestamp}."""
        return self._get("motions")

    def layer_motion_state(self):
        """data: [{name: 'X.layers', status, timestamp}] for v2 motions."""
        return self._get("motions/all")

    # -- gait -------------------------------------------------------------------
    def gait(self, speed_v=0, speed_h=0, steps=0, period=1, wave=False):
        """Walk: speed_v/speed_h in -5..5, steps 0 = until stopped, period 1..5 (0 stops)."""
        return self._put("motions/gait", {"speed_v": int(speed_v), "speed_h": int(speed_h), "steps": int(steps),
                                          "period": int(period), "timestamp": int(time.time() * 1000),
                                          "wave": bool(wave)})

    def gait_state(self):
        """data: {status 0-8 (8 = idle), timestamp}."""
        return self._get("motions/gait")

    def gait_exit(self):
        return self._delete("motions/gait", {"timestamp": 0})

    def gait_decelerate(self, speed_v, speed_h=0, period=1, wave=False, stage_s=0.5):
        """Stop an endless walk gently: step the speeds down (|5| -> 3 -> 1, |3| -> 1) with stage_s between the
        updates, then gait_exit. exit_motion_gait at full speed freezes the legs mid-step, which can tip the
        robot over. Returns the gait_exit reply."""
        for stage in (3, 1):
            v, h = max(-stage, min(stage, int(speed_v))), max(-stage, min(stage, int(speed_h)))
            if (v, h) == (speed_v, speed_h):
                continue
            speed_v, speed_h = v, h
            self.gait(v, h, 0, period, wave)
            time.sleep(stage_s)
        return self.gait_exit()

    # -- music ------------------------------------------------------------------
    def music_list(self):
        """data: {music: [{name}]} on current firmware."""
        return self._get("media/music/list")

    def music_play(self, name):
        body = {"operation": "start"}
        if name:
            body["name"] = name
        return self._put("media/music", body)

    def music_stop(self):
        return self._put("media/music", {"operation": "stop"})

    def music_state(self):
        """data: {name, status: run|idle}."""
        return self._get("media/music")

    # -- voice ------------------------------------------------------------------
    def tts(self, text, interrupt=True, timestamp=None):
        """Speak text. Returns immediately; poll tts_state(timestamp) for run/idle."""
        ts = int(time.time()) if timestamp is None else timestamp
        return self._put("voice/tts", {"tts": text, "interrupt": bool(interrupt), "timestamp": ts})

    def tts_state(self, timestamp=None):
        """status: idle | run | build | wait."""
        return _decode_voice(self._get("voice/tts", {"timestamp": timestamp} if timestamp is not None else None))

    def tts_stop(self):
        return self._delete("voice/tts")

    def _start_and_poll(self, start, poll, stop, timeout, ts):
        res = start()
        if res.get("code") != 0:
            return res
        deadline = time.monotonic() + timeout
        last = res
        while time.monotonic() < deadline:
            time.sleep(self.poll_interval)
            last = poll()
            if last.get("code", 0) != 0:
                return last
            if last.get("timestamp") == ts and last.get("status") == "idle":
                return last
        try:
            stop()
        except RobotError:
            pass
        return {"code": -1, "status": "timeout", "timestamp": ts, "data": last.get("data", {}),
                "msg": f"no result within {timeout:g} s (task stopped)"}

    def asr(self, timeout=10):
        """One round of speech understanding (Chinese on the robot). On timeout: code -1, status 'timeout'."""
        ts = int(time.time())
        return self._start_and_poll(lambda: self._put("voice/asr", {"continues": False, "timestamp": ts}),
                                    lambda: _decode_voice(self._get("voice/asr")),
                                    lambda: self._delete("voice/asr"), timeout, ts)

    def iat(self, timeout=10):
        """One round of dictation. On timeout: code -1, status 'timeout'."""
        ts = int(time.time())
        return self._start_and_poll(lambda: self._put("voice/iat", {"timestamp": ts}),
                                    lambda: _decode_voice(self._get("voice/iat")),
                                    lambda: self._delete("voice/iat"), timeout, ts)

    # -- vision -----------------------------------------------------------------
    def vision_task(self, option, type, operation="start", timestamp=0):
        return self._put("visions", {"option": option, "type": type, "operation": operation,
                                     "timestamp": int(timestamp)})

    def vision_result(self, option, type):
        res = self._get("visions", {"option": option, "type": type})
        if isinstance(res.get("data"), str):  # the robot sometimes sends "{}" as a string
            try:
                res["data"] = json.loads(res["data"] or "{}")
            except ValueError:
                pass
        return res

    def vision(self, option, type, timeout=8):
        """Run one vision task and wait for its result.

        option/type: face/(recognition|quantity|age|age_group|gender|expression|mask|glass), hand/gesture,
        color/color_detect, object/recognition. The task is stopped if no result arrives within timeout
        seconds; the reply then has code -1 and status 'timeout'.
        """
        if type not in VISION_TYPES.get(option, ()):
            raise ValueError(f"unknown vision task {option}/{type}")
        ts = int(time.time())
        return self._start_and_poll(lambda: self.vision_task(option, type, "start", ts),
                                    lambda: self.vision_result(option, type),
                                    lambda: self.vision_task(option, type, "stop", ts), timeout, ts)

    def qr(self, timeout=8):
        """Scan for a QR code (data.contents). Stops the task on timeout."""
        res = self._put("visions/QR", {"operation": "start", "remote_stream_enable": False})
        if res.get("code") not in (0, 20003):
            return res
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(self.poll_interval)
            last = self._get("visions/QR")
            data = last.get("data") if isinstance(last.get("data"), dict) else {}
            if last.get("status") == "idle" or data.get("contents"):
                return last
        try:
            self._put("visions/QR", {"operation": "stop"})
        except RobotError:
            pass
        return {"code": -1, "status": "timeout", "data": {}, "msg": f"no QR code within {timeout:g} s"}

    def take_photo(self, resolution="640x480"):
        """data: {name} of the photo stored on the robot."""
        return self._post("visions/photos", {"resolution": resolution})

    def download_photo(self, name):
        """JPEG bytes of a photo stored on the robot."""
        content, ctype = self._request("GET", "visions/photos", {"body": name}, raw=True)
        if not content.startswith(b"\xff\xd8"):
            raise RobotProtocolError(f"photo '{name}' is not a JPEG ({ctype}): {content[:120]!r}")
        return content

    def delete_photo(self, name):
        return self._delete("visions/photos", {"name": name})

    def photo(self, resolution="640x480", keep=False):
        """Take a photo with the head camera and return the JPEG bytes (deleted on the robot unless keep)."""
        res = self.take_photo(resolution)
        if res.get("code") != 0:
            raise RobotProtocolError(f"robot could not take a photo: code {res.get('code')} {res.get('msg', '')}")
        name = (res.get("data") or {}).get("name")
        if not name:
            raise RobotProtocolError(f"photo reply has no name: {res}")
        jpeg = self.download_photo(name)
        if not keep:
            try:
                self.delete_photo(name)
            except RobotError:
                pass
        return jpeg

    def stream_open(self, resolution="640x480"):
        """Start the MJPEG stream at stream_url."""
        return self._post("visions/streams", {"resolution": resolution})

    def stream_close(self):
        return self._delete("visions/streams")

    def upload_photo_sample(self, name, jpeg):
        """Upload JPEG bytes as a face sample (multipart field 'file')."""
        boundary = uuid.uuid4().hex
        data = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"{name}\"\r\n"
                f"Content-Type: image/jpeg\r\n\r\n").encode() + jpeg + f"\r\n--{boundary}--\r\n".encode()
        return self._request("POST", "visions/photosamples", data=data,
                             headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})

    def set_vision_tag(self, resources, tag):
        return self._put("visions/tags", {"resources": list(resources), "tags": tag})

    def face_entry(self, name):
        """Enrol the face in front of the camera under name (photo, upload as sample, tag). Returns the tag reply."""
        res = self.take_photo()
        if res.get("code") != 0:
            return res
        photo = res["data"]["name"]
        jpeg = self.download_photo(photo)
        res = self.upload_photo_sample(photo, jpeg)
        try:
            self.delete_photo(photo)
        except RobotError:
            pass
        if res.get("code") != 0:
            return res
        return self.set_vision_tag([photo], name)

    def vision_tags(self):
        """data: [{resources: [sample names], tags: name}]."""
        return self._get("visions/tags")

    def delete_vision_tag(self, tag, mode="all"):
        """mode 'all' deletes the tag and its samples, 'tags' only the tag."""
        return self._delete("visions/tags", {"tags": tag, "mode": mode})

    # -- sensors ----------------------------------------------------------------
    def sensors_list(self):
        """data: {sensors: [{id, slot, type, version}]} for the attached add-on sensors."""
        return self._get("sensors/list")

    def gyro(self):
        """data: {gyro: [{id, gyro-x/y/z, accel-x/y/z, compass-x/y/z, euler-x/y/z}]}."""
        return self._get("sensors/gyro")

    @staticmethod
    def _ids(id, slot):
        p = {}
        if id is not None:
            p["id"] = id
        if slot is not None:
            p["slot"] = slot
        return p or None

    def infrared(self, id=None, slot=None):
        return self._get("sensors/infrared", self._ids(id, slot))

    def ultrasonic(self, id=None, slot=None):
        return self._get("sensors/ultrasonic", self._ids(id, slot))

    def touch(self, id=None, slot=None):
        return self._get("sensors/touch", self._ids(id, slot))

    def pressure(self, id=None, slot=None):
        return self._get("sensors/pressure", self._ids(id, slot))

    def environment(self):
        """data: {environment: [{id, temperature, humidity, pressure}]}."""
        return self._get("sensors/environment")


__all__ = ["YansheeRest", "RobotError", "RobotUnreachable", "RobotProtocolError", "split_host", "SPEEDS",
           "VISION_TYPES", "DEFAULT_PORT"]
