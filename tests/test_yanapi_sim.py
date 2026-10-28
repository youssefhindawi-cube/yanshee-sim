"""YanAPI shim, run_in_sim and the motion tools, headless. Run: python3 tests/test_yanapi_sim.py"""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
os.environ["YANSHEE_SIM_HEADLESS"] = "1"
os.environ["YANSHEE_SIM_FIXED"] = "1"
os.environ.setdefault("MUJOCO_GL", "egl")
from yanshee_sim import RESET_POSE, SimRobot  # noqa: E402
from yanshee_sim import yanapi_sim as Y  # noqa: E402

EXAMPLES = os.path.join(os.path.dirname(ROOT), "yanshee robot", "yanshee", "yanshee_examples", "yanshee_examples")
ENV = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
NOD = {"name": "nod", "frames": [{"angles": {"NeckLR": 40}, "runtime": 400},
                                 {"angles": {"NeckLR": 140}, "runtime": 400}] * 3, "end_in_reset": True}


def _script(path, *args, stdin=None, timeout=180):
    return subprocess.run([sys.executable, *path, *args], input=stdin, capture_output=True, text=True,
                          env=ENV, timeout=timeout, cwd=ROOT)


def _run_in_sim(*args, timeout=180):
    return _script([os.path.join(ROOT, "scripts", "run_in_sim.py")], *args, timeout=timeout)


def _load_script(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, "scripts", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _MotionDir:
    """Point the shim at a temporary motion folder."""

    def __enter__(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved = list(Y.MOTION_DIRS)
        Y.MOTION_DIRS[:] = [self.tmp.name]
        return self.tmp.name

    def __exit__(self, *exc):
        Y.stop_play_motion()
        Y.MOTION_DIRS[:] = self.saved
        self.tmp.cleanup()


def _write(folder, name, mo):
    with open(os.path.join(folder, name + ".json"), "w") as f:
        json.dump(mo, f)


# -- shim, in process ------------------------------------------------------------
def test_star_import_and_all():
    ns = {}
    exec("from yanshee_sim.yanapi_sim import *", ns)
    for n in ("sync_play_motion", "stop_play_motion", "sync_set_servo_rotate", "get_servo_angle_value",
              "get_motion_list_value", "sync_do_tts", "get_sensors_gyro", "RobotMotionSpeed", "basic_url"):
        assert n in ns, n
    assert "NAMES" not in ns and "SimRobot" not in ns and "os" not in ns


def test_fallback_defaults_and_classes():
    assert Y.get_robot_version_info_value("core") == ""
    assert Y.get_sensors_infrared_value() == -1
    assert Y.do_face_entry("bob") is False and Y.sync_do_motion_gait(1) is False
    assert Y.get_sensors_list_value() == []
    assert Y.sync_do_voice_asr_value() is None
    res = Y.sync_do_face_recognition("quantity")
    assert res["code"] == -1 and res["data"] == {}
    assert not hasattr(Y, "__wrapped__") and not hasattr(Y, "_no_such_private")
    v = Y.Voice()
    assert v.sync_do_voice_asr()["code"] == -1 and Y.Voice is Y.Voice
    assert Y.RobotMotionSpeed.veryFast.value == "very fast" and Y.RobotJointType.No17.value == "NeckLR"
    assert Y.GamepadKey.A.value == "key_A"
    assert Y.set_robot_volume_value(30) is True and Y.get_robot_volume_value() == 30
    assert Y.sync_set_led("button", "red", "on") is True


def test_servo_calls_return_errors_instead_of_raising():
    Y.yan_api_init("192.168.3.13")
    assert Y.set_servos_angles({"NeckLR": 300})["code"] == -1
    assert Y.set_servos_angles({"Bogus": 90})["code"] == -1
    assert Y.sync_set_servo_rotate({"NeckLR": 300})["code"] == -1
    assert Y.get_servos_angles(["Bogus"])["code"] == -1
    assert Y.get_servo_angle_value("Bogus") == -1
    assert Y.sync_set_servo_rotate({"NeckLR": 60}, 400)["code"] == 0
    Y._bot().wait(0.4)
    assert abs(Y.get_servo_angle_value("NeckLR") - 60) <= 2
    assert Y.get_servos_angles(["NeckLR"])["data"]["NeckLR"] == Y.get_servo_angle_value("NeckLR")
    Y.sync_play_motion("reset")


def test_gyro_has_real_field_set():
    g = Y.get_sensors_gyro()["data"]["gyro"][0]
    keys = {"id"} | {f"{k}-{a}" for k in ("gyro", "accel", "compass", "euler") for a in "xyz"}
    assert keys <= set(g), keys - set(g)
    assert all(type(g[k]) is float for k in keys - {"id"})
    assert abs(g["euler-x"] - 90) < 5 and abs(g["euler-y"]) < 5  # torso held upright (euler-x: 0 = face down)


def test_motion_list_dedup():
    with _MotionDir() as d:
        other = tempfile.mkdtemp()
        Y.MOTION_DIRS.append(other)
        for folder in (d, other):
            _write(folder, "nod", NOD)
            _write(folder, "reset", NOD)
        assert Y.get_motion_list_value() == ["reset", "nod"]
        from yanshee_sim import hts
        hts.save(NOD, os.path.join(d, "nod_hts.hts"))  # the robot's own files are found too
        assert Y.get_motion_list_value() == ["reset", "nod", "nod_hts"]
        assert Y._find_motion("nod_hts").endswith("nod_hts.hts") and Y._find_motion("nod").endswith("nod.json")


def test_stop_cancels_playback_and_holds():
    with _MotionDir() as d:
        _write(d, "nod", NOD)
        assert Y.start_play_motion("nod")["code"] == 0
        time.sleep(0.6)
        t = time.time()
        assert Y.stop_play_motion()["code"] == 0
        assert time.time() - t < 0.5
        assert Y._playback is None
        Y._bot().wait(0.4)
        held = Y.get_servo_angle_value("NeckLR")
        Y._bot().wait(1.0)
        assert abs(Y.get_servo_angle_value("NeckLR") - held) <= 2, "motion kept playing after stop"


def test_new_motion_and_reset_cancel_the_old_one():
    with _MotionDir() as d:
        _write(d, "nod", NOD)
        Y.start_play_motion("nod")
        first = Y._playback
        time.sleep(0.3)
        Y.start_play_motion("nod")
        assert not first.thread.is_alive() and first.stop.is_set()
        second = Y._playback
        Y.start_play_motion("reset")
        assert not second.thread.is_alive()
        Y._bot().wait(1.5)
        assert abs(Y.get_servo_angle_value("NeckLR") - RESET_POSE["NeckLR"]) <= 2


def test_sync_play_waits_for_the_motion():
    with _MotionDir() as d:
        _write(d, "nod", NOD)
        for speed in ("fast", "very slow"):
            total = Y.start_play_motion("nod", speed=speed)["data"]["total_time"]
            Y.stop_play_motion()
            t = time.time()
            assert Y.sync_play_motion("nod", speed=speed) is True
            took = time.time() - t
            assert abs(took * 1000 - total) < 300, (speed, took, total)
        assert Y.start_play_motion("nod", speed="fast")["data"]["total_time"] == 6 * 320 + 800  # clamps like play()
        Y.stop_play_motion()


def test_invalid_motion_is_not_played():
    with _MotionDir() as d:
        _write(d, "bad", {"frames": [{"angles": {"NeckLR": 200}, "runtime": 400}]})
        res = Y.start_play_motion("bad")
        assert res["code"] == -1 and "NeckLR" in res["msg"] and Y._playback is None
        assert Y.sync_play_motion("bad") is False
        assert Y.sync_play_motion("no_such_motion") is False


# -- run_in_sim ------------------------------------------------------------------
def test_run_in_sim_exit_codes_and_script_dir():
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "helper_pose.py"), "w") as f:
            f.write("POSE = {'NeckLR': 70}\n")
        with open(os.path.join(d, "show.py"), "w") as f:
            f.write("import sys\nfrom YanAPI import *\nfrom helper_pose import POSE\n"
                    "yan_api_init('1.2.3.4')\nsync_set_servo_rotate(POSE, 300)\n"
                    "assert sync_play_motion('reset') is True\nstop_play_motion()\n"
                    "assert get_servo_angle_value('NeckLR') > 0\nprint('argv', sys.argv[1:])\n")
        with open(os.path.join(d, "quit3.py"), "w") as f:
            f.write("from YansheeRobot import YanAPI\nYanAPI.yan_api_init('x')\nimport sys\nsys.exit(3)\n")
        with open(os.path.join(d, "boom.py"), "w") as f:
            f.write("import YanAPI\nYanAPI.yan_api_init('x')\nraise KeyError('quantity')\n")
        r = _run_in_sim(os.path.join(d, "show.py"), "--speed", "2")
        assert r.returncode == 0 and "argv ['--speed', '2']" in r.stdout, r.stderr
        assert _run_in_sim(os.path.join(d, "quit3.py")).returncode == 3
        r = _run_in_sim(os.path.join(d, "boom.py"))
        assert r.returncode == 1 and "KeyError" in r.stderr, (r.returncode, r.stderr)
    r = _run_in_sim("--help")
    assert r.returncode == 0 and "run_in_sim" in r.stdout
    assert _run_in_sim("no/such/script.py").returncode == 2


def test_official_examples_run_in_sim():
    if not os.path.isdir(EXAMPLES):
        print("skip: official examples not found at", EXAMPLES)
        return
    for rel, code in [("0custom0/singleJoint.py", 0),                    # servo moves
                      ("devices_samples/devices_samples_simple.py", 0),  # int/str getters
                      ("voice_samples/voice_class_sample.py", 0),        # YanAPI.Voice class
                      ("face_number/face_number.py", 1)]:                # its own KeyError: rc 1, not a segfault
        r = _run_in_sim(os.path.join(EXAMPLES, rel))
        assert r.returncode == code, (rel, r.returncode, r.stderr[-2000:])


# -- record_real_motion ------------------------------------------------------------
class _FakeRest:
    def __init__(self, readings, fail_after=None):
        self.readings, self.calls, self.fail_after, self.status = readings, 0, fail_after, "idle"

    def get_angles(self, names):
        self.calls += 1
        if self.fail_after is not None and self.calls > self.fail_after:
            raise ConnectionError("robot stopped answering")
        return {"code": 0, "data": dict(self.readings)}

    def motion_state(self):
        return {"code": 0, "data": {"name": "", "status": self.status, "timestamp": 0}}


def _fake_real_robot(rest, refuse=False, run_s=1.2):
    class FakeRealRobot:
        def __init__(self, ip, **kw):
            self.rest = rest

        def play_builtin(self, name, direction="", **kw):
            if refuse and name != "reset":
                return {"code": 6, "msg": "lowbattery"}
            rest.status = "run"
            end = time.time() + (0.2 if name == "reset" else run_s)
            import threading
            threading.Timer(end - time.time(), lambda: setattr(rest, "status", "idle")).start()
            return {"code": 0, "data": {"total_time": 1000}}
    return FakeRealRobot


def test_record_clamps_and_writes_atomically():
    rec = _load_script("record_real_motion")
    readings = {**RESET_POSE, "LeftAnkleUD": 116, "NeckLR": 10}
    rec.RealRobot = _fake_real_robot(_FakeRest(readings))
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "sub", "wave.json")
        assert rec.main(["--ip", "127.0.0.1", "--name", "wave", "--hz", "5", "--out", out]) == 0
        mo = json.load(open(out))
        assert len(mo["frames"]) >= 2 and not os.path.exists(out + ".tmp")
        assert all(f["angles"]["LeftAnkleUD"] == 115 and f["angles"]["NeckLR"] == 15 for f in mo["frames"])
        assert all(200 <= f["runtime"] <= 4000 for f in mo["frames"])


def test_record_failures_write_nothing():
    rec = _load_script("record_real_motion")
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "wave.json")
        with open(out, "w") as f:
            f.write("KEEP")
        rec.RealRobot = _fake_real_robot(_FakeRest(RESET_POSE), refuse=True)
        assert rec.main(["--ip", "127.0.0.1", "--name", "wave", "--out", out]) == 1
        rec.RealRobot = _fake_real_robot(_FakeRest(RESET_POSE, fail_after=1))
        assert rec.main(["--ip", "127.0.0.1", "--name", "wave", "--out", out]) == 1
        rec.RealRobot = _fake_real_robot(_FakeRest(RESET_POSE), run_s=0.0)
        assert rec.main(["--ip", "127.0.0.1", "--name", "wave", "--hz", "0.5", "--out", out]) == 1  # too few samples
        assert open(out).read() == "KEEP"
    for hz in ("0", "-1", "10"):
        r = _script([os.path.join(ROOT, "scripts", "record_real_motion.py")], "--ip", "127.0.0.1", "--name", "x",
                    "--hz", hz)
        assert r.returncode == 2 and "--hz" in r.stderr


def test_record_against_fake_robot_server():
    """End to end with tests/fake_robot.py and the real RealRobot, once it has .rest and play_builtin."""
    from yanshee_sim import RealRobot
    if not hasattr(RealRobot, "play_builtin"):
        print("skip: RealRobot.play_builtin not available yet")
        return
    import fake_robot
    srv = None
    for port in range(9240, 9300):
        try:
            srv = fake_robot.start(port, motion_seconds=1.5)
            break
        except OSError:
            continue
    try:
        rec = _load_script("record_real_motion")
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "wave.json")
            assert rec.main(["--ip", f"127.0.0.1:{port}", "--name", "wave", "--direction", "both", "--out", out]) == 0
            assert len(json.load(open(out))["frames"]) >= 2
            assert rec.main(["--ip", f"127.0.0.1:{port}", "--name", "NoSuchMotion", "--out", out + "2"]) == 1
            assert not os.path.exists(out + "2")
    finally:
        srv.stop()


# -- pose_editor and view ------------------------------------------------------------
def test_pose_editor_validates_saves_and_reloads():
    editor = [os.path.join(ROOT, "scripts", "pose_editor.py")]
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "new", "dir", "hello.json")
        r = _script(editor, "--out", out, "--fixed",
                    stdin="time 50\nset NeckLR\nundo\nset NeckLR 60\nadd\nsave\nquit\n")
        assert r.returncode == 0, r.stderr
        assert "time must be 200-4000" in r.stdout and "usage: set" in r.stdout and "nothing to undo" in r.stdout
        mo = json.load(open(out))
        assert mo["frames"] == [{"angles": {"NeckLR": 60.0}, "runtime": 600, "hold": 0}]
        r = _script(editor, "--out", out, "--fixed", stdin="set RightElbowFlex 120\nadd\nsave\nquit\n")
        assert r.returncode == 0 and "loaded 1 keyframes" in r.stdout, r.stderr
        frames = json.load(open(out))["frames"]
        assert frames[1]["angles"] == {"RightElbowFlex": 120.0}, frames  # NeckLR stays at 60, not reset
        os.chmod(d, 0o500)
        try:
            r = _script(editor, "--out", os.path.join(d, "x", "y.json"), "--fixed",
                        stdin="set NeckLR 70\nadd\nsave\nshow\nquit\n")
        finally:
            os.chmod(d, 0o700)
        assert r.returncode == 0 and "error:" in r.stdout and "1: {'angles': {'NeckLR': 70.0}" in r.stdout, r.stdout


def test_view_reset_restores_reset_pose():
    import mujoco
    view = _load_script("view")
    from yanshee_sim import sim_extras
    bot = SimRobot(viewer=False, realtime=False, spec_hook=sim_extras.add_leds)
    m, d = bot.model, bot.data
    mujoco.set_mjcb_control(view.reset_hold_callback(bot))  # takes the SimRobot now (it also paints the LEDs)
    try:
        for _ in range(250):
            mujoco.mj_step(m, d)
        mujoco.mj_resetData(m, d)  # what the viewer's Backspace / Reset does
        for _ in range(int(2 / m.opt.timestep)):
            mujoco.mj_step(m, d)
    finally:
        mujoco.set_mjcb_control(None)
    assert bot.torso_tilt_deg() < 10
    got = bot.get_servos_angles(["RightShoulderFlex", "NeckLR"])["data"]
    assert abs(got["RightShoulderFlex"] - 140) <= 3 and abs(got["NeckLR"] - 90) <= 3, got


if __name__ == "__main__":
    try:
        for name, fn in list(globals().items()):
            if name.startswith("test_"):
                fn()
                print("ok", name)
    finally:
        Y._shutdown()
