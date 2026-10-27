"""Drop-in stand-in for YanAPI that drives the simulator, so existing robot scripts run unmodified.

    python3 scripts/run_in_sim.py path/to/your_script.py

Servo, motion-file and reset calls are simulated. Speech, LEDs, music and volume are printed.
Vision, voice recognition and sensors other than the gyro return {"code": -1} because the
simulator has no AI services; guard for that the same way you guard for a real timeout.
"""
import os
import sys

from . import motion as _motion
from .robot import ROOT, SimRobot
from .servos import NAMES, RESET_POSE

_robot = None
_warned = set()
MOTION_DIRS = [os.path.join(ROOT, "motions", "recorded"), os.path.join(ROOT, "motions")]


def _bot():
    global _robot
    if _robot is None:
        _robot = SimRobot(viewer=os.environ.get("YANSHEE_SIM_HEADLESS") != "1",
                          fixed_base=os.environ.get("YANSHEE_SIM_FIXED") == "1")
    return _robot


def _ok(data=None):
    return {"code": 0, "data": data if data is not None else {}, "msg": "success"}


def _say(kind, text):
    print(f"[sim {kind}] {text}", file=sys.stderr)


def yan_api_init(robot_ip):
    _say("init", f"YanAPI calls for {robot_ip} are routed to the simulator")
    _bot()


# servos ---------------------------------------------------------------------
def set_servos_angles(angles, runtime=200):
    return _bot().set_servos_angles(angles, runtime)


def sync_set_servo_rotate(angles, runtime=200):
    return _bot().sync_set_servo_rotate(angles, runtime)


def get_servos_angles(names):
    return _bot().get_servos_angles(names)


def get_servo_angle_value(name):
    return _bot().get_servo_angle_value(name)


# motions --------------------------------------------------------------------
def _find_motion(name):
    for d in MOTION_DIRS:
        p = os.path.join(d, name + ".json")
        if os.path.exists(p):
            return p
    return None


def start_play_motion(name="reset", direction="", speed="normal", repeat=1, timestamp=0, version="v1"):
    import threading
    if name in ("reset", "Stop", "stop"):
        _bot().set_servos_angles(dict(RESET_POSE), 1000)
        return _ok({"total_time": 1000})
    path = _find_motion(name if not direction else f"{name}_{direction}") or _find_motion(name)
    if not path:
        _say("motion", f"built-in motion '{name}' is not available in simulation "
                       f"(record it from the robot with scripts/record_real_motion.py)")
        return {"code": -1, "data": {}, "msg": f"motion '{name}' not simulated"}
    mo = _motion.load(path)
    rate = {"very slow": 0.5, "slow": 0.75, "normal": 1.0, "fast": 1.25, "very fast": 1.5}.get(speed, 1.0)
    frames = mo["frames"] * max(1, int(repeat))
    threading.Thread(target=_motion.play, args=(_bot(), {**mo, "frames": frames}, rate), daemon=True).start()
    return _ok({"total_time": int(_motion.duration_s({**mo, "frames": frames}) * 1000 / rate)})


def sync_play_motion(name="reset", direction="", speed="normal", repeat=1, version="v1"):
    res = start_play_motion(name, direction, speed, repeat)
    if res["code"] == 0:
        _bot().wait(res["data"]["total_time"] / 1000)
    return res["code"] == 0


def stop_play_motion(name="", timestamp=0, version="v1"):
    return start_play_motion("reset")


def get_motion_list_value():
    found = []
    for d in MOTION_DIRS:
        if os.path.isdir(d):
            found += [f[:-5] for f in os.listdir(d) if f.endswith(".json")]
    return ["reset"] + sorted(found)


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


def set_robot_led(type, color, mode):
    _say("led", f"{type} {color} {mode}")
    return _ok()


def set_robot_volume(volume):
    _say("volume", volume)
    return _ok()


def start_play_music(name=""):
    _say("music", f"play {name}")
    return _ok()


def stop_play_music():
    _say("music", "stop")
    return _ok()


def get_sensors_gyro():
    return _bot().get_sensors_gyro()


def __getattr__(name):
    def not_simulated(*args, **kwargs):
        if name not in _warned:
            _warned.add(name)
            _say("not simulated", f"YanAPI.{name}() returns code -1 in simulation")
        return {"code": -1, "data": {}, "msg": "not simulated"}
    return not_simulated


__all__ = ["yan_api_init", "set_servos_angles", "get_servos_angles", "start_play_motion", "NAMES"]
