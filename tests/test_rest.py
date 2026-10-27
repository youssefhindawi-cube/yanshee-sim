"""YansheeRest and RealRobot against tests/fake_robot.py (no hardware). Run: python3 tests/test_rest.py"""
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import fake_robot  # noqa: E402
from yanshee_sim.rest import RobotProtocolError, RobotUnreachable, YansheeRest, split_host  # noqa: E402
from yanshee_sim.robot import RealRobot, RobotCommandError, ServoCommandError  # noqa: E402
from yanshee_sim.servos import NAMES, RESET_POSE  # noqa: E402

PORTS = range(9191, 9300)  # only ever 127.0.0.1
YANAPI_DIR = os.path.join(os.path.dirname(ROOT), "yanshee robot", "yanshee", "yanshee_examples", "yanshee_examples")


def start_fake(**options):
    """Fake robot on the first free port in PORTS."""
    options.setdefault("motion_seconds", 0.6)
    options.setdefault("vision_seconds", 0.4)
    for port in PORTS:
        try:
            return fake_robot.start(port, **options)
        except OSError:
            continue
    raise RuntimeError("no free port in 9191-9299")


def free_port():
    """A port in PORTS with nothing listening (connections are refused)."""
    for port in PORTS:
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port
    raise RuntimeError("no free port in 9191-9299")


class _Fixture:
    def __init__(self, **options):
        self.srv = start_fake(**options)
        self.rest = YansheeRest(self.srv.address, timeout=2)

    def __enter__(self):
        return self.srv, self.rest

    def __exit__(self, *exc):
        self.srv.stop()


def fake(**options):
    return _Fixture(**options)


def wait_for(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


# -- addressing -------------------------------------------------------------------
def test_split_host():
    assert split_host("192.168.3.13") == ("192.168.3.13", 9090)
    assert split_host("127.0.0.1:9191") == ("127.0.0.1", 9191)
    assert split_host("http://10.0.0.5:9090/v1/") == ("10.0.0.5", 9090)
    for bad in ("", "1.2.3.4:x", "1.2.3.4:70000"):
        try:
            split_host(bad)
        except ValueError:
            continue
        raise AssertionError(bad)


# -- endpoints --------------------------------------------------------------------
def test_device_endpoints():
    with fake(battery=77) as (srv, r):
        assert r.battery()["data"]["percent"] == 77
        assert r.versions("sn")["data"]["sn"].startswith("FAKE")
        assert r.mode()["data"]["calibration_mode"] is False
        assert r.set_volume(35)["code"] == 0 and r.get_volume()["data"]["volume"] == 35
        assert r.set_language("zh")["code"] == 0 and r.get_language()["data"]["language"] == "zh"
        assert r.set_led("button", "red", "blink")["code"] == 0
        assert {"type": "button", "color": "red", "mode": "blink"} in r.get_led()["data"]
        assert r.set_led("camera", "purple", "on")["code"] != 0  # camera LEDs are red/green/blue only
        assert r.set_fall_management(True)["code"] == 0 and r.get_fall_management()["data"]["enable"] is True


def test_servos_move_linearly_and_modes():
    with fake() as (srv, r):
        assert r.get_angles(NAMES)["data"] == RESET_POSE
        assert r.set_angles({"NeckLR": 50}, runtime=1000)["data"] == {"NeckLR": True}
        time.sleep(0.5)
        mid = r.get_angles(["NeckLR"])["data"]["NeckLR"]
        assert 60 <= mid <= 80, mid  # halfway from 90 to 50
        time.sleep(0.6)
        assert r.get_angles(["NeckLR"])["data"]["NeckLR"] == 50
        assert r.set_servo_mode("program", ["RightShoulderFlex"])["data"] == {"RightShoulderFlex": True}
        assert r.get_servo_mode(["RightShoulderFlex", "NeckLR"])["data"] == {"RightShoulderFlex": "program",
                                                                            "NeckLR": "work"}
        srv.state.move_by_hand({"RightShoulderFlex": 100})
        assert r.get_angles(["RightShoulderFlex"])["data"]["RightShoulderFlex"] == 100
        assert r.set_angles({"RightShoulderFlex": 50})["data"] == {"RightShoulderFlex": False}  # limp servo


def test_motions_list_play_pause_stop():
    with fake() as (srv, r):
        data = r.motion_list()["data"]
        hts = [m["name"] for m in data["system_hts_motions"]]
        layers = [m["name"] for m in data["system_layers_motions"]]
        assert "WakaWaka.hts" in hts and "H_Wave_R.layers" in layers and "Track_TurnR.hts" not in hts
        assert next(m for m in data["system_hts_motions"] if m["name"] == "WakaWaka.hts")["music"] is True
        res = r.play_motion("wave", "left", speed="slow", repeat=2)
        assert res["code"] == 0 and res["data"]["total_time"] == 1800  # 0.6 s * 1.5 * 2
        st = r.motion_state()["data"]
        assert st["status"] == "run" and st["name"] == "wave"
        assert r.motion_control("pause", "wave")["code"] == 0 and r.motion_state()["data"]["status"] == "pause"
        assert r.motion_control("resume", "wave")["code"] == 0 and r.motion_state()["data"]["status"] == "run"
        assert r.motion_control("stop")["code"] == 0 and r.motion_state()["data"]["status"] == "idle"
        assert r.play_motion("NoSuchMotion")["code"] != 0
        r.play_motion("H_Wave_R", version="v2")
        layered = r.layer_motion_state()["data"]
        assert layered[0]["name"] == "H_Wave_R.layers" and layered[0]["status"] == "run"
        assert wait_for(lambda: r.motion_state()["data"]["status"] == "idle")
        try:
            r.play_motion("wave", speed="warp")
        except ValueError:
            pass
        else:
            raise AssertionError("bad speed accepted")


def test_gait_music_tts():
    with fake() as (srv, r):
        assert r.gait(speed_v=2, steps=2, period=1)["code"] == 0
        assert r.gait_state()["data"]["status"] == 2
        assert wait_for(lambda: r.gait_state()["data"]["status"] == 8)
        r.gait(speed_v=1)
        assert r.gait_exit()["code"] == 0 and r.gait_state()["data"]["status"] == 8
        assert {"name": "WakaWaka.mp3"} in r.music_list()["data"]["music"]
        r.music_play("WakaWaka.mp3")
        assert r.music_state()["data"] == {"name": "WakaWaka.mp3", "status": "run"}
        r.music_stop()
        assert r.music_state()["data"]["status"] == "idle"
        assert r.tts("hello there", timestamp=42)["code"] == 0
        st = r.tts_state(42)
        assert st["status"] == "run" and st["data"]["text"] == "hello there"  # NUL-padded JSON decoded
        r.tts_stop()
        assert r.tts_state()["status"] == "idle"


def test_vision_voice_and_timeouts():
    with fake() as (srv, r):
        res = r.vision("face", "age", timeout=3)
        assert res["code"] == 0 and res["status"] == "idle" and res["data"]["analysis"]["age"] == 30
        assert r.vision("color", "color_detect")["data"]["color"][0]["name"] == "red"
        assert r.vision("hand", "gesture")["data"]["gesture"] == "ok"
        assert r.qr(timeout=3)["data"]["contents"]
        assert r.asr(timeout=3)["data"]["question"] == "你好"
        assert r.iat(timeout=3)["data"]["text"] == "hello robot"
        try:
            r.vision("face", "colour")
        except ValueError:
            pass
        else:
            raise AssertionError("bad vision task accepted")
        srv.state.vision_seconds = 60  # the robot never answers in time
        t0 = time.monotonic()
        res = r.vision("object", "recognition", timeout=0.8)
        assert res["code"] == -1 and res["status"] == "timeout" and time.monotonic() - t0 < 2
        stops = [e for e in srv.state.log if e[0] == "PUT" and e[1] == "visions" and e[3].get("operation") == "stop"]
        assert stops and stops[-1][3]["option"] == "object", "vision task not stopped on timeout"
        assert r.iat(timeout=0.5)["status"] == "timeout"
        assert any(e[:2] == ("DELETE", "voice/iat") for e in srv.state.log)


def test_photo_stream_face_entry_sensors():
    with fake() as (srv, r):
        jpeg = r.photo()
        assert jpeg[:2] == b"\xff\xd8" and len(jpeg) > 200
        assert srv.state.photos == {}, "photo not deleted on the robot"
        assert r.stream_open()["code"] == 0 and r.stream_url.endswith(":8000/stream.mjpg")
        assert r.stream_open()["code"] == 20003  # already running
        assert r.stream_close()["code"] == 0
        assert r.face_entry("ada")["code"] == 0
        tags = r.vision_tags()["data"]
        assert tags[0]["tags"] == "ada" and tags[0]["resources"][0].endswith(".jpg")
        assert r.delete_vision_tag("ada")["code"] == 0 and r.vision_tags()["data"] == []
        g = r.gyro()["data"]["gyro"][0]
        assert {"id", "gyro-x", "accel-z", "compass-x", "euler-x", "euler-y", "euler-z"} <= set(g)
        assert len(r.sensors_list()["data"]["sensors"]) == 5
        assert r.infrared()["data"]["infrared"][0]["value"] > 0
        assert r.ultrasonic(id=[2])["data"]["ultrasonic"]
        assert r.touch()["code"] == 0 and r.pressure()["code"] == 0
        assert r.environment()["data"]["environment"][0]["temperature"] == 24
        srv.state.overrides[("GET", "visions/photos")] = (b"<html>oops</html>", "text/html")
        try:
            r.photo()
        except RobotProtocolError:
            pass
        else:
            raise AssertionError("HTML accepted as a photo")


def test_payloads_match_official_yanapi():
    """Every request the client builds equals what YanAPI_en.py sends (timestamps aside)."""
    try:
        sys.path.insert(0, YANAPI_DIR)
        import YanAPI_en as Y
    except Exception as e:  # requests/nest_asyncio/cv2 missing or SDK not next to the repo
        print("skip test_payloads_match_official_yanapi:", e)
        return

    def strip(x):
        if isinstance(x, dict):
            return {k: strip(v) for k, v in x.items() if k != "timestamp"}
        return x

    with fake() as (srv, r):
        Y.basic_url = f"http://{srv.address}/v1/"
        pairs = [
            (Y.get_robot_battery_info, r.battery), (lambda: Y.get_robot_version_info("servo"), lambda: r.versions("servo")),
            (Y.get_robot_mode, r.mode), (Y.get_robot_volume, r.get_volume),
            (lambda: Y.set_robot_volume(40), lambda: r.set_volume(40)), (Y.get_robot_language, r.get_language),
            (lambda: Y.set_robot_language("en"), lambda: r.set_language("en")), (Y.get_robot_led, r.get_led),
            (lambda: Y.set_robot_led("button", "red", "blink"), lambda: r.set_led("button", "red", "blink")),
            (Y.get_robot_fall_management_state, r.get_fall_management),
            (lambda: Y.set_robot_fall_management_state(True), lambda: r.set_fall_management(True)),
            (lambda: Y.get_servos_angles(["NeckLR", "RightHipFB"]), lambda: r.get_angles(["NeckLR", "RightHipFB"])),
            (lambda: Y.set_servos_angles({"NeckLR": 80}, 500), lambda: r.set_angles({"NeckLR": 80}, 500)),
            (lambda: Y.get_servos_mode(["NeckLR"]), lambda: r.get_servo_mode(["NeckLR"])),
            (lambda: Y.set_servos_mode("work", ["NeckLR", "LeftHipFB"]), lambda: r.set_servo_mode("work", ["NeckLR", "LeftHipFB"])),
            (Y.get_motion_list, r.motion_list),
            (lambda: Y.start_play_motion("wave", "left", "slow", 2), lambda: r.play_motion("wave", "left", "slow", 2)),
            (lambda: Y.start_play_motion("reset"), lambda: r.play_motion("reset")),
            (lambda: Y.pause_play_motion(), lambda: r.motion_control("pause")),
            (lambda: Y.resume_play_motion(), lambda: r.motion_control("resume")),
            (lambda: Y.stop_play_motion(version="v2"), lambda: r.motion_control("stop", version="v2")),
            (Y.get_current_motion_play_state, r.motion_state), (Y.get_current_layer_motion_play_state, r.layer_motion_state),
            (lambda: Y.control_motion_gait(1, -1, 4, 2, True), lambda: r.gait(1, -1, 4, 2, True)),
            (Y.get_motion_gait_state, r.gait_state), (Y.exit_motion_gait, r.gait_exit),
            (Y.get_media_music_list, r.music_list), (Y.get_media_music_state, r.music_state),
            (lambda: Y.start_play_music("WakaWaka.mp3"), lambda: r.music_play("WakaWaka.mp3")),
            (Y.stop_play_music, r.music_stop),
            (lambda: Y.start_voice_tts("hi", False, 5), lambda: r.tts("hi", False, 5)), (Y.stop_voice_tts, r.tts_stop),
            (lambda: Y.start_face_recognition("age", 5), lambda: r.vision_task("face", "age", "start", 5)),
            (lambda: Y.get_visual_task_result("face", "age"), lambda: r.vision_result("face", "age")),
            (lambda: Y.take_vision_photo("320x240"), lambda: r.take_photo("320x240")),
            (Y.get_vision_photo_list, lambda: r._get("visions/photos/list")),
            (lambda: Y.open_vision_stream("320x240"), lambda: r.stream_open("320x240")),
            (Y.close_vision_stream, r.stream_close), (Y.get_vision_tags, r.vision_tags),
            (lambda: Y.set_vision_tag(["a.jpg"], "x"), lambda: r.set_vision_tag(["a.jpg"], "x")),
            (lambda: Y.delete_vision_tag("x"), lambda: r.delete_vision_tag("x")),
            (Y.get_sensors_list, r.sensors_list), (Y.get_sensors_gyro, r.gyro), (Y.get_sensors_infrared, r.infrared),
            (Y.get_sensors_ultrasonic, r.ultrasonic), (Y.get_sensors_touch, r.touch),
            (Y.get_sensors_pressure, r.pressure), (Y.get_sensors_environment, r.environment),
            (lambda: Y.start_voice_asr(timestamp=7), lambda: r.asr(timeout=0.01)),
            (lambda: Y.start_voice_iat(timestamp=7), lambda: r.iat(timeout=0.01)),
            (lambda: Y.start_QR_code_recognition(), lambda: r.qr(timeout=0.01)),
        ]
        for official, ours in pairs:
            srv.state.log.clear()
            official()
            want = [(m, p, q, strip(b)) for m, p, q, b in srv.state.log]
            srv.state.log.clear()
            ours()
            got = [(m, p, q, strip(b)) for m, p, q, b in srv.state.log][:len(want)]
            assert got == want, f"{got} != {want}"
        srv.state.log.clear()
        name = Y.take_vision_photo()["data"]["name"]
        srv.state.log.clear()
        r.download_photo(name)  # YanAPI.get_vision_photo sends GET visions/photos?body=NAME (and writes a file)
        assert srv.state.log[-1][:3] == ("GET", "visions/photos", {"body": name})


# -- transport errors -------------------------------------------------------------
def test_unreachable_and_timeouts():
    port = free_port()
    r = YansheeRest(f"127.0.0.1:{port}", timeout=1)
    t0 = time.monotonic()
    try:
        r.battery()
    except RobotUnreachable as e:
        assert isinstance(e, ConnectionError) and str(port) in str(e)
    else:
        raise AssertionError("no error for a closed port")
    try:
        RealRobot(f"127.0.0.1:{port}")
    except ConnectionError as e:
        assert "cannot reach" in str(e)
    else:
        raise AssertionError("RealRobot connected to nothing")
    assert time.monotonic() - t0 < 3
    with fake(slow=3) as (srv, _):
        r = YansheeRest(srv.address, timeout=0.5)
        t0 = time.monotonic()
        try:
            r.get_angles(["NeckLR"])
        except RobotUnreachable:
            assert time.monotonic() - t0 < 1.5
        else:
            raise AssertionError("slow robot did not time out")
        t0 = time.monotonic()
        try:
            RealRobot(srv.address, timeout=0.5)
        except RobotUnreachable as e:
            assert "did not answer" in str(e) and time.monotonic() - t0 < 2.5
        else:
            raise AssertionError("RealRobot accepted a robot that never answers")


class _HtmlHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        body = b"<html><body>502 Bad Gateway</body></html>"
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def test_non_json_reply():
    srv = None
    for port in PORTS:
        try:
            srv = ThreadingHTTPServer(("127.0.0.1", port), _HtmlHandler)
            break
        except OSError:
            continue
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        try:
            YansheeRest(f"127.0.0.1:{port}").battery()
        except RobotProtocolError as e:
            assert "not JSON" in str(e)
        else:
            raise AssertionError("HTML accepted")
        try:
            RealRobot(f"127.0.0.1:{port}")
        except RobotProtocolError as e:
            assert "is this a Yanshee" in str(e)
        else:
            raise AssertionError("RealRobot accepted HTML")
    finally:
        srv.shutdown()
        srv.server_close()


# -- RealRobot over the fake ------------------------------------------------------------
def test_real_robot_basics():
    with fake(battery=64) as (srv, _):
        bot = RealRobot(srv.address, timeout=2)
        assert bot.battery == 64 and bot.rest.port == srv.server_address[1]
        assert bot.battery_info()["percent"] == 64
        bot.set_servos_angles({"NeckLR": 120}, runtime=200)
        time.sleep(0.3)
        assert bot.get_servo_angle_value("NeckLR") == 120
        assert bot.get_servos_angles()["data"]["LeftHipFB"] == RESET_POSE["LeftHipFB"]
        for bad in (lambda: bot.get_servo_angle_value("Foo"), lambda: bot.get_servos_angles(["Foo"]),
                    lambda: bot.set_servos_angles({"NeckLR": float("nan")}), lambda: bot.set_servos_mode("limp", ["NeckLR"])):
            try:
                bad()
            except ServoCommandError:
                continue
            raise AssertionError("bad input accepted")
        bot.strict = False
        try:
            bot.set_servos_angles({"NeckLR": float("nan")})
        except ServoCommandError:
            pass
        else:
            raise AssertionError("NaN accepted in non-strict mode")
        assert bot.set_servos_mode("program", ["LeftShoulderFlex"])["code"] == 0
        assert bot.get_servos_mode(["LeftShoulderFlex"])["data"] == {"LeftShoulderFlex": "program"}
        res = bot.play_builtin("WakaWaka", speed="fast")
        assert res["data"]["total_time"] == 450
        assert srv.state.motion["status"] == "run"
        r = bot.stop_all()
        assert r["code"] == 0 and set(r["data"]) == {"motion_v1", "motion_v2", "gait", "music", "tts"}
        assert srv.state.motion["status"] == "idle"
        assert bot.stop()["code"] == 0
        bot.wait(-1)  # negative waits are ignored, not an error
        bot.close()


def test_real_robot_low_battery_and_refusals():
    with fake(battery=20) as (srv, _):
        try:
            RealRobot(srv.address)
        except RuntimeError as e:
            assert "20%" in str(e)
        else:
            raise AssertionError("low battery accepted")
        bot = RealRobot(srv.address, min_battery=10)
        try:
            bot.play_builtin("wave", "right")
        except RobotCommandError as e:
            assert e.code == 6 and "low battery" in str(e)
        else:
            raise AssertionError("robot motion at 20% accepted")
        bot.set_servos_angles({"NeckLR": 80})  # servo commands still work
    with fake(refuse=True) as (srv, _):
        bot = RealRobot(srv.address)
        for cmd in (lambda: bot.set_servos_angles({"NeckLR": 80}), lambda: bot.play_builtin("bow"),
                    lambda: bot.set_servos_mode("program", ["NeckLR"])):
            try:
                cmd()
            except RobotCommandError as e:
                assert e.code == 1 and "refused" in str(e)
            else:
                raise AssertionError("refusal not raised")
        assert bot.stop_all()["code"] == 0  # stops are never refused
    with fake() as (srv, _):
        srv.state.overrides[("GET", "devices/battery")] = {"code": 3, "msg": "busy", "data": {}}
        try:
            RealRobot(srv.address)
        except ConnectionError as e:
            assert "error code 3: busy" in str(e)
        else:
            raise AssertionError("battery error code ignored")


def test_two_robots_stay_separate():
    with fake() as (a_srv, _), fake() as (b_srv, _):
        a, b = RealRobot(a_srv.address), RealRobot(b_srv.address)
        a.set_servos_angles({"NeckLR": 60})
        b.set_servos_angles({"NeckLR": 120})
        time.sleep(0.3)
        assert a.get_servo_angle_value("NeckLR") == 60 and b.get_servo_angle_value("NeckLR") == 120


def test_stop_all_when_robot_vanishes():
    srv = start_fake()
    bot = RealRobot(srv.address, timeout=0.5)
    srv.stop()
    t0 = time.monotonic()
    res = bot.stop_all()  # never raises
    assert res["code"] == -1 and time.monotonic() - t0 < 2
    try:
        bot.get_servos_angles()
    except RobotUnreachable:
        pass
    else:
        raise AssertionError("dead robot not reported")


def test_api_is_lazy_and_aimed_at_this_robot():
    with fake() as (srv, _):
        bot = RealRobot(srv.address)
        assert bot._api is None
        try:
            api = bot.api
        except ImportError as e:
            print("skip .api check:", e)
            return
        assert api.basic_url == f"http://{srv.address}/v1/"
        assert api.get_robot_battery_value() == 90


def test_fake_robot_cli_help():
    import subprocess
    out = subprocess.run([sys.executable, os.path.join(HERE, "fake_robot.py"), "--help"], capture_output=True,
                         text=True, timeout=30)
    assert out.returncode == 0 and "--refuse" in out.stdout and "--slow" in out.stdout


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
