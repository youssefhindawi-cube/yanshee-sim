"""Drop-in stand-in for YanAPI that drives the simulator, so existing robot scripts run unmodified.

    python3 scripts/run_in_sim.py path/to/your_script.py

Servo, motion-file and reset calls are simulated. Speech, LEDs, music and volume are printed.
Vision, voice recognition and sensors other than the gyro are not simulated: they return the
real API's failure value (an error dict {"code": -1, ...}, -1, "", False, [] or None, by the
function's documented return type) and print a warning once, so guard for them the same way
you guard for a real timeout.

Differences from the real robot, on purpose:
- Bad servo commands (unknown name, angle outside the limits) are not sent. The call prints a
  warning and returns {"code": -1, "msg": ...} (get_servo_angle_value returns -1).
  A runtime outside 200-4000 ms prints a warning and is clamped.
- One motion plays at a time. start_play_motion cancels the motion that is playing;
  stop_play_motion cancels it and holds the current pose; start_play_motion("reset") cancels it
  and returns to the reset pose. A motion file that fails motion.validate() is not played
  (code -1, the errors are in msg).
"""
import atexit
import inspect
import math
import os
import sys
import threading
from enum import Enum, unique

from . import motion as _motion
from .robot import ROOT, SimRobot
from .servos import LIMITS, MAX_RUNTIME_MS, MIN_RUNTIME_MS, NAMES, RESET_POSE, check_angles, clamp_angles

_robot = None
_closed = False
_warned = set()
_state = {"volume": 50}
MOTION_DIRS = [os.path.join(ROOT, "motions", "recorded"), os.path.join(ROOT, "motions", "builtin"),
               os.path.join(ROOT, "motions")]
SPEEDS = {"very slow": 0.5, "slow": 0.75, "normal": 1.0, "fast": 1.25, "very fast": 1.5}

# Module globals of the real YanAPI (some scripts read them).
ip = "127.0.0.1"
basic_url = "http://127.0.0.1:9090/v1/"
headers = {"Content-Type": "application/json"}


def _bot():
    global _robot
    if _robot is None:
        _robot = SimRobot(viewer=os.environ.get("YANSHEE_SIM_HEADLESS") != "1",
                          fixed_base=os.environ.get("YANSHEE_SIM_FIXED") == "1")
        atexit.register(_shutdown)  # a script that exits without cleanup must not crash the interpreter
    return _robot


def _shutdown():
    """Cancel playback and close the simulator (idempotent; also registered with atexit)."""
    global _closed
    _cancel_playback()
    if _robot is not None and not _closed:
        _closed = True
        _robot.close()


def _ok(data=None):
    return {"code": 0, "data": data if data is not None else {}, "msg": "success"}


def _err(msg):
    return {"code": -1, "data": {}, "msg": msg}


def _say(kind, text):
    print(f"[sim {kind}] {text}", file=sys.stderr)


def _warn_once(key, text):
    if key not in _warned:
        _warned.add(key)
        _say("warning", text)


def _val(x):
    """Accept the YanAPI enums (RobotMotionSpeed.fast, ...) wherever a string is expected."""
    return x.value if isinstance(x, Enum) else x


def yan_api_init(robot_ip):
    global ip, basic_url
    ip, basic_url = str(robot_ip), f"http://{robot_ip}:9090/v1/"
    _say("init", f"YanAPI calls for {robot_ip} are routed to the simulator")
    _bot()


# servos ---------------------------------------------------------------------
def _check_servo_command(angles, runtime):
    """Return an error dict for a command the shim refuses, else None. Warns about clamped runtimes."""
    try:
        problems = check_angles(angles)
    except (AttributeError, TypeError) as e:
        problems = [f"bad angles argument {angles!r} ({e})"]
    if problems:
        _say("warning", "servo command not sent: " + "; ".join(problems))
        return _err("; ".join(problems))
    if not MIN_RUNTIME_MS <= runtime <= MAX_RUNTIME_MS:
        _say("warning", f"runtime {runtime} ms is outside {MIN_RUNTIME_MS}-{MAX_RUNTIME_MS}; clamped")
    return None


def set_servos_angles(angles, runtime=200):
    return _check_servo_command(angles, runtime) or _bot().set_servos_angles(angles, runtime)


def sync_set_servo_rotate(angles, runtime=200):
    return _check_servo_command(angles, runtime) or _bot().sync_set_servo_rotate(angles, runtime)


def get_servos_angles(names):
    names = [names] if isinstance(names, str) else list(names)
    unknown = [n for n in names if n not in LIMITS]
    if unknown:
        _say("warning", f"get_servos_angles: unknown servo(s) {unknown}")
        return _err(f"unknown servo(s) {unknown}")
    return _bot().get_servos_angles(names)


def get_servo_angle_value(name):
    if name not in LIMITS:
        _say("warning", f"get_servo_angle_value: unknown servo '{name}'")
        return -1
    return _bot().get_servo_angle_value(name)


# motions --------------------------------------------------------------------
class _Playback:
    def __init__(self, name):
        self.name, self.stop, self.ok, self.thread = name, threading.Event(), None, None


_play_lock = threading.Lock()
_playback = None


def _run_playback(pb, bot, mo, rate):
    try:
        pb.ok = _motion.play(bot, mo, rate, stop_event=pb.stop) is not False
    except Exception as e:  # report instead of dying silently in the thread
        pb.ok = False
        _say("motion", f"'{pb.name}' failed: {e}")


def _cancel_playback(timeout=5.0):
    """Stop the motion that is playing (if any) and wait for its thread."""
    global _playback
    with _play_lock:
        pb, _playback = _playback, None
    if pb is not None and pb.thread.is_alive():
        pb.stop.set()
        if pb.thread is not threading.current_thread():
            pb.thread.join(timeout)


def _hold_pose(bot):
    """Freeze every servo at its current target (what the robot does when a motion is stopped)."""
    if hasattr(bot, "stop_all"):
        bot.stop_all()
    else:
        bot.set_servos_angles(clamp_angles(bot.get_target_angles()), MIN_RUNTIME_MS)


def _find_motion(name, direction=""):
    stems = [f"{name}_{direction}", name] if direction else [name]
    for stem in stems:
        for d in MOTION_DIRS:
            for s in dict.fromkeys((stem, stem.replace(" ", "_"))):
                p = os.path.join(d, s + ".json")
                if os.path.exists(p):
                    return p
    return None


def start_play_motion(name="reset", direction="", speed="normal", repeat=1, timestamp=0, version="v1"):
    global _playback
    name, direction, speed = _val(name), _val(direction) or "", _val(speed)
    _cancel_playback()
    if name in ("reset", "Stop", "stop"):
        _bot().set_servos_angles(dict(RESET_POSE), 1000)
        return _ok({"total_time": 1000})
    path = _find_motion(name, direction)
    if not path:
        _say("motion", f"built-in motion '{name}' is not available in simulation "
                       f"(record it from the robot with scripts/record_real_motion.py)")
        return _err(f"motion '{name}' not simulated")
    try:
        mo = _motion.load(path)
        errors, _ = _motion.validate(mo)
    except (OSError, ValueError, AttributeError, TypeError) as e:
        errors = [f"cannot read {path}: {e}"]
    if errors:
        _say("motion", f"'{name}' not played, {path} has errors: " + "; ".join(errors))
        return _err("; ".join(errors))
    rate = SPEEDS.get(speed, 1.0)
    mo = {**mo, "frames": mo["frames"] * max(1, int(repeat))}
    pb = _Playback(name)
    pb.thread = threading.Thread(target=_run_playback, args=(pb, _bot(), mo, rate), daemon=True)
    with _play_lock:
        _playback = pb
    pb.thread.start()
    return _ok({"total_time": int(round(_motion.duration_s(mo, rate) * 1000))})


def sync_play_motion(name="reset", direction="", speed="normal", repeat=1, version="v1"):
    """Play and return when the motion ends (True), or False if it could not play or failed."""
    res = start_play_motion(name, direction, speed, repeat)
    if res["code"] != 0:
        return False
    with _play_lock:
        pb = _playback
    if pb is None:  # reset: a timed servo move
        _bot().wait(res["data"]["total_time"] / 1000)
        return True
    while pb.thread.is_alive():
        pb.thread.join(0.1)
    return pb.ok is not False or pb.stop.is_set()  # stopped by another call still counts as played


def stop_play_motion(name="", timestamp=0, version="v1"):
    """Cancel the motion that is playing and hold the current pose."""
    _cancel_playback()
    if _robot is not None and not _closed:
        _hold_pose(_robot)
    return _ok()


def get_motion_list_value():
    found = set()
    for d in MOTION_DIRS:
        if os.path.isdir(d):
            found |= {f[:-5] for f in os.listdir(d) if f.endswith(".json")}
    return ["reset"] + sorted(found - {"reset"})


# device stubs ---------------------------------------------------------------
def get_robot_battery_value():
    return 100


def get_robot_battery_info():
    return _ok({"percent": 100, "charging": 0, "voltage": 8400})


def start_voice_tts(tts="", interrupt=True, timestamp=0):
    _say("tts", tts)
    return _ok()


def sync_do_tts(tts="", interrupt=True):
    _say("tts", tts)
    _bot().wait(max(1.0, 0.4 * len(str(tts).split())))
    return _ok()


def stop_voice_tts():
    _say("tts", "stop")
    return _ok()


def set_robot_led(type, color, mode):
    _say("led", f"{_val(type)} {_val(color)} {_val(mode)}")
    return _ok()


def sync_set_led(type, color, mode):
    return set_robot_led(type, color, mode)["code"] == 0


def set_robot_volume(volume):
    _say("volume", volume)
    _state["volume"] = int(volume)
    return _ok()


def set_robot_volume_value(volume):
    return set_robot_volume(volume)["code"] == 0


def get_robot_volume():
    return _ok({"volume": _state["volume"]})


def get_robot_volume_value():
    return _state["volume"]


def start_play_music(name=""):
    _say("music", f"play {name}")
    return _ok()


def sync_play_music(name=""):
    start_play_music(name)
    return True


def stop_play_music():
    _say("music", "stop")
    return _ok()


def get_sensors_gyro():
    """Simulated IMU with the real field set. Euler angles are degrees in the model frame (x roll,
    y pitch, z yaw); the real robot's IMU frame differs (see README). Compass is not simulated (0)."""
    bot = _bot()
    res = bot.get_sensors_gyro()
    entry = dict(res["data"]["gyro"][0])
    with bot._lock:
        w, x, y, z = (float(v) for v in bot.data.sensor("imu_quat").data)
    euler = {"euler-x": math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
             "euler-y": math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
             "euler-z": math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))}
    out = {"id": 1}
    for k in ("gyro-x", "gyro-y", "gyro-z", "accel-x", "accel-y", "accel-z"):
        out[k] = float(entry.get(k, 0.0))
    out.update({"compass-x": 0.0, "compass-y": 0.0, "compass-z": 0.0})
    out.update({k: float(entry.get(k, v)) for k, v in euler.items()})
    return {"code": 0, "data": {"gyro": [out]}, "msg": "success"}


# YanAPI enums (copied from YanAPI.py; they have no dependencies) --------------
@unique
class ChargingState(Enum):
    Uncharged = 0
    Charging = 1


@unique
class RobotLanguage(Enum):
    zh = "zh"
    en = "en"


@unique
class RobotButtonLedColor(Enum):
    white = "white"
    red = "red"
    green = "green"
    blue = "blue"
    yellow = "yellow"
    purple = "purple"
    cyan = "cyan"


@unique
class RobotButtonLedMode(Enum):
    on = "on"
    off = "off"
    blink = "blink"
    breath = "breath"


@unique
class RobotEyeLedColor(Enum):
    red = "red"
    green = "green"
    blue = "blue"


@unique
class RobotEyeLedMode(Enum):
    on = "on"
    off = "off"
    blink = "blink"


@unique
class RobotBuiltInMotion(Enum):
    reset = "reset"
    handsUp = "raise"
    crouch = "crouch"
    comeOn = "come on"
    stretch = "stretch"
    wave = "wave"
    bend = "bend"
    walk = "walk"
    turnAround = "turn around"
    head = "head"
    bow = "bow"


@unique
class RobotMotionDirection(Enum):
    none = ""
    left = "left"
    right = "right"
    both = "both"
    forward = "forward"
    backward = "backward"


@unique
class RobotMotionSpeed(Enum):
    verySlow = "very slow"
    slow = "slow"
    normal = "normal"
    fast = "fast"
    veryFast = "very fast"


@unique
class RobotFaceRecognitionType(Enum):
    tracking = "tracking"
    recognition = "recognition"
    quantity = "quantity"
    age_group = "age_group"
    gender = "gender"
    age = "age"
    expression = "expression"
    mask = "mask"
    glass = "glass"


RobotJointType = unique(Enum("RobotJointType", {f"No{i + 1}": n for i, n in enumerate(NAMES)}, module=__name__))

GamepadKey = unique(Enum("GamepadKey", {k: "key_" + k for k in (
    "L1 L2 R1 R2 A B X Y DPAD_UP DPAD_DOWN DPAD_LEFT DPAD_RIGHT DPAD_UP_LEFT DPAD_UP_RIGHT DPAD_DOWN_LEFT "
    "DPAD_DOWN_RIGHT L_STICK R_STICK L_STICK_UP L_STICK_DOWN L_STICK_LEFT L_STICK_RIGHT R_STICK_UP "
    "R_STICK_DOWN R_STICK_LEFT R_STICK_RIGHT BT START POWER RESERVE").split()}, module=__name__))


# everything else: the real API's failure value -------------------------------
# Return types from YanAPI.py for the calls that do not return a {code, data, msg} dict.
_STR_FUNCS = {"get_robot_version_info_value", "get_button_led_color_value", "get_button_led_mode_value",
              "get_eye_led_color_value", "get_eye_led_mode_value", "sync_do_voice_iat_value"}
_BOOL_FUNCS = {"sync_do_motion_gait", "do_face_entry"}
_LIST_FUNCS = {"get_sensors_list_value"}
_NONE_FUNCS = {"sync_do_voice_asr_value", "sync_do_face_recognition_value"}
_stub_classes = {}


def _failure_value(name):
    if name in _STR_FUNCS:
        return ""
    if name in _BOOL_FUNCS:
        return False
    if name in _LIST_FUNCS:
        return []
    if name in _NONE_FUNCS:
        return None
    if name.endswith("_value"):
        return -1
    return {"code": -1, "data": {}, "msg": "not simulated"}


def _not_simulated(name, owner="YanAPI"):
    def not_simulated(*args, **kwargs):
        value = _failure_value(name)
        _warn_once(f"{owner}.{name}", f"{owner}.{name}() is not simulated; returns {value!r}")
        return value
    not_simulated.__name__ = name
    return not_simulated


def _stub_class(cls_name):
    """A class whose instances accept any arguments and whose methods return the failure value."""
    def __init__(self, *args, **kwargs):
        _warn_once(f"YanAPI.{cls_name}", f"YanAPI.{cls_name} is not simulated; its methods return failure values")

    def __getattr__(self, attr):
        if attr.startswith("_"):
            raise AttributeError(attr)
        return _not_simulated(attr, cls_name)

    return type(cls_name, (), {"__init__": __init__, "__getattr__": __getattr__, "__module__": __name__})


def __getattr__(name):
    if name.startswith("_"):  # dunders (hasattr checks) and private names are not part of the API
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    if name[:1].isupper():
        return _stub_classes.setdefault(name, _stub_class(name))
    return _not_simulated(name)


__all__ = ["ip", "basic_url", "headers"] + [
    n for n, v in list(globals().items())
    if not n.startswith("_") and getattr(v, "__module__", None) == __name__ and (inspect.isfunction(v) or isinstance(v, type))]
