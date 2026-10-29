"""Relay controller: sim mode, robot and both modes against tests/fake_robot.py, the safety gate, the
renderer, the sim built-ins and gait, concurrency and a 60 s soak. Run: python3 tests/test_controller.py

Headless: MUJOCO_GL=egl is set when there is no display. YANSHEE_SOAK_S shortens the soak (default 60 s).
"""
import json
import os
import random
import shutil
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")
import fake_robot  # noqa: E402
from yanshee_sim import motion, sim_extras  # noqa: E402
from yanshee_sim.controller import Controller, ModeError  # noqa: E402
from yanshee_sim.robot import SimRobot  # noqa: E402
from yanshee_sim.servos import RESET_POSE  # noqa: E402

LEGS_CROSS = {"RightHipLR": 120, "LeftHipLR": 60}
# Each pose is clear, but the straight line back to reset hits the hip (see test_motion.py). The contacts are grazing:
# 1-2 deg of start-pose jitter (the free-standing sim) can hide them, so sim-mode checks pin the start (exact_start).
RESET_GAP = {"frames": [{"angles": {"LeftShoulderFlex": 66, "RightShoulderRoll": 113}, "runtime": 2000},
                        {"angles": {"RightElbowFlex": 155, "RightHipLR": 36}, "runtime": 2000}],
             "end_in_reset": True}
_CTL = {}


class exact_start:
    """with exact_start(c): sweeps start from RESET_POSE exactly instead of the sim's measured (jittery) pose."""

    def __init__(self, c):
        self.c = c

    def __enter__(self):
        self.c._current_pose = lambda: dict(RESET_POSE)

    def __exit__(self, *exc):
        del self.c._current_pose


def ctl():
    """One shared sim-mode controller (creating one compiles the model and starts a renderer)."""
    if "c" not in _CTL:
        _CTL["dir"] = tempfile.mkdtemp(prefix="yanshee_ctl_")
        shutil.copy(os.path.join(ROOT, "motions", "wave_right.json"), _CTL["dir"])
        _CTL["c"] = Controller(motions_dir=_CTL["dir"])
    c = _CTL["c"]
    if c.robot is not None:
        c.disconnect()
    c.stop_all()
    if c._fallen:
        c.reset()
    c.charge()
    return c


def start_fake(**options):
    """Fake robot on an OS-assigned port (a fixed range collided with parallel test runs and manual fakes)."""
    options.setdefault("motion_seconds", 1.5)
    options.setdefault("vision_seconds", 0.3)
    return fake_robot.start(0, **options)


def wait_until(cond, timeout=5.0, step=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


def requests(srv, method=None, path=None):
    return [e for e in list(srv.state.log) if (method is None or e[0] == method) and (path is None or e[1] == path)]


class Connected:
    """with Connected(mode="robot", battery=20) as (c, srv): ... (shared controller connected to a fake robot)."""

    def __init__(self, mode="robot", **fake_options):
        self.mode, self.options = mode, fake_options

    def __enter__(self):
        self.c = ctl()
        self.srv = start_fake(**self.options)
        res = self.c.connect(self.srv.address)
        assert res["ok"], res
        self.c.set_mode(self.mode)
        return self.c, self.srv

    def __exit__(self, *exc):
        try:
            self.c.disconnect()
        finally:
            self.srv.stop()


# -- sim mode ---------------------------------------------------------------------------------
def test_state_snapshot_shape():
    c = ctl()
    st = c.state()
    json.dumps(st)
    for key in ("mode", "robot", "sim", "servos", "activity", "leds", "volume", "language", "fall_management",
                "limits", "reset_pose", "events"):
        assert key in st, key
    assert st["mode"] == "sim" and st["robot"]["connected"] is False and st["servos"]["robot"] is None
    assert set(st["servos"]["sim"]) == set(RESET_POSE) and st["activity"]["kind"] == "idle"
    assert st["sim"]["fixed_base"] is False and 0 <= st["sim"]["battery"] <= 100
    assert st["limits"]["NeckLR"] == [15, 165] and st["reset_pose"]["NeckLR"] == 90
    assert all({"t", "level", "text"} <= set(e) for e in st["events"])


def test_mode_needs_robot():
    c = ctl()
    for mode in ("robot", "both"):
        try:
            c.set_mode(mode)
        except ModeError as e:
            assert "connect" in str(e)
        else:
            raise AssertionError("robot mode without a robot was accepted")
    try:
        c.set_mode("nope")
    except ModeError:
        pass
    else:
        raise AssertionError("bad mode accepted")
    assert c.mode == "sim"


def test_sim_servos_limits_and_bad_input():
    c = ctl()
    res = c.set_servos({"NeckLR": 60, "LeftAnkleUD": 130}, runtime=400)
    assert res["ok"] and res["sent_to"] == ["sim"] and any("LeftAnkleUD" in w for w in res["warnings"])
    assert wait_until(lambda: abs(c.get_servos()["sim"]["NeckLR"] - 60) <= 2, 3)
    assert c.get_servos()["robot"] is None
    for bad in ({"Nope": 3}, {"NeckLR": float("nan")}, {"NeckLR": "x"}, {}, "NeckLR"):
        r = c.set_servos(bad)
        assert r["ok"] is False and r["error"], bad
    r = c.set_servos({"NeckLR": 90}, runtime=50)
    assert r["ok"] and r["runtime"] == 200 and r["warnings"]
    assert c.set_servo_mode("program", ["NeckLR"])["ok"]
    assert c.sim.get_servos_mode(["NeckLR"])["data"]["NeckLR"] == "program"
    assert c.set_servo_mode("work", ["NeckLR"])["ok"]
    assert c.set_servo_mode("limp")["ok"] is False and c.set_servo_mode("work", ["Bad"])["ok"] is False
    rec = c.record_frame(600, 100)
    assert rec["ok"] and rec["source"] == "sim" and set(rec["frame"]["angles"]) == set(RESET_POSE)
    assert rec["frame"]["runtime"] == 600 and rec["frame"]["hold"] == 100
    assert c.record_frame(50)["ok"] is False and c.record_frame(500, -1)["ok"] is False
    c.reset()


def test_list_actions():
    c = ctl()
    acts = c.list_actions()
    by = {a["name"]: a for a in acts}
    for a in acts:
        assert {"name", "directions", "category", "sim", "robot", "music"} <= set(a), a
    for name, dirs in (("raise", 3), ("wave", 3), ("stretch", 3), ("come on", 3), ("bend", 2), ("turn around", 2),
                       ("walk", 4), ("head", 3), ("bow", 0), ("crouch", 0), ("reset", 0)):
        assert len(by[name]["directions"]) == dirs, name
    assert by["wave"]["sim"] and by["walk"]["sim"] and not by["turn around"]["sim"]
    for dance in ("WakaWaka", "LittleApple", "GangnamStyle", "HappyBirthday", "SorrySorry", "WeAreTakingOff"):
        assert by[dance]["category"] == "dances" and by[dance]["music"] and by[dance]["robot"]
        assert not by[dance]["sim"] and by[dance]["source"] == "fallback"
    for hidden in ("Shutdown", "ActionAging", "ActionAging1", "calibration", "calibration_new", "ResetSwitch"):
        assert hidden not in by, hidden
    cats = {a["category"] for a in acts}
    assert {"gestures", "locomotion", "fight", "football", "get-up", "dances", "v2 layered", "user"} <= cats, cats
    assert by["GetupFront"]["category"] == "get-up" and by["Football_LKick"]["category"] == "football"
    assert by["Fight_LHit"]["category"] == "fight" and by["H_Wave_R"]["version"] == "v2"
    assert by["wave_right"]["category"] == "user" and by["wave_right"]["kind"] == "file"


def test_sim_actions_and_cancel():
    c = ctl()
    r = c.play_action("wave", "right")
    assert r["ok"] and r["target"] == "sim" and r["duration_s"] > 1
    assert wait_until(lambda: c.state()["activity"]["kind"] == "action", 2)
    assert wait_until(lambda: c.get_servos()["sim"]["RightShoulderFlex"] < 100, 3)
    first = c._activity
    r = c.play_action("head", "left", speed="fast")  # a new activity cancels the old one
    assert r["ok"] and first.done.is_set() and first.stop.is_set()
    assert wait_until(lambda: c.get_servos()["sim"]["NeckLR"] < 60, 3)
    assert c.play_action("WakaWaka")["ok"] is False
    assert "not available in simulation" in c.play_action("turn around", "left")["error"]
    assert c.play_action("wave", "up")["ok"] is False
    assert c.play_action("wave", "left", speed="warp")["ok"] is False
    assert c.play_action("wave", "left", repeat=0)["ok"] is False
    r = c.play_action("wave_right")  # a user motion file through play_action
    assert r["ok"] and c.state()["activity"]["kind"] == "motion"
    c.stop_all()
    assert c.state()["activity"]["kind"] == "idle"
    r = c.play_action("crouch", speed="very fast", repeat=2)
    assert r["ok"]
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 10)
    c.reset()


def test_builtin_files_safe_and_upright():
    """Every sim built-in played from standing passes validate and self_collisions and keeps the free-standing sim
    upright. The get-ups start from lying: test_getup_builtins_are_the_robots."""
    files = sorted(f for f in os.listdir(sim_extras.BUILTIN_DIR) if f.endswith(".json") and not f.startswith("getup_"))
    expected = {sim_extras.builtin_path(n, d) for n, dirs in sim_extras.SIM_BUILTINS.items() for d in dirs}
    assert {os.path.join(sim_extras.BUILTIN_DIR, f) for f in files} == expected
    checker = motion.CollisionChecker()
    for f in files:
        mo = motion.load(os.path.join(sim_extras.BUILTIN_DIR, f))
        errors, warnings = motion.validate(mo)
        assert not errors and not warnings, (f, errors, warnings)
        assert motion.self_collisions(mo, checker=checker) == [], f
        sim = SimRobot(viewer=False, realtime=False)
        tilt = [0.0]

        class Watch:
            def set_servos_angles(self, angles, runtime):
                return sim.set_servos_angles(angles, runtime)

            def wait(self, seconds):
                for _ in range(int(round(seconds / 0.02))):
                    sim.wait(0.02)
                    tilt[0] = max(tilt[0], sim.torso_tilt_deg())
        assert motion.play(Watch(), mo)
        Watch().wait(1.0)
        sim.close()
        assert tilt[0] < 15, (f, tilt[0])


def test_getup_builtins_are_the_robots():
    """getup_front / getup_rear are the robot's own Base_GetupF and GetupRear (verified on the robot on 29 Oct 2026):
    every frame sets all 17 servos and validates without errors. Their fast frames and tight leg folds trip the
    speed and collision checks meant for hand-made motions, so those are not applied; get_up() tests standing up."""
    for posture, source in (("front", "Base_GetupF"), ("rear", "GetupRear")):
        mo = sim_extras.load_builtin("getup", posture)
        assert mo is not None and source in mo["source"], posture
        assert all(set(fr["angles"]) == set(RESET_POSE) for fr in mo["frames"]), posture
        assert motion.validate(mo)[0] == [], posture


def test_sim_gait_generator_upright():
    """Open-loop gait in every direction: steps, moves the right way and never tilts past 20 deg."""
    import numpy as np
    for params, axis, sign in ((dict(speed_v=5, steps=8), 1, 1), (dict(speed_v=-5, steps=6), 1, -1),
                               (dict(speed_h=5, steps=6), 0, -1), (dict(speed_h=-5, steps=6), 0, 1),
                               (dict(steps=4, wave=True), None, 0), (dict(speed_v=5, speed_h=5, period=1, steps=6),
                                                                     1, 1)):
        sim = SimRobot(viewer=False, realtime=False)
        tilt, wait = [0.0], sim.wait

        def watched(seconds, wait=wait, sim=sim):
            wait(seconds)
            tilt[0] = max(tilt[0], sim.torso_tilt_deg())
        sim.wait = watched
        p0 = sim.data.qpos[:2].copy()
        g = sim_extras.SimGait(sim)
        status = g.run(params, threading.Event())
        sim.wait(1.0)
        moved = sim.data.qpos[:2] - p0
        sim.close()
        assert status == "done" and g.steps_done >= params["steps"], (params, status)
        assert tilt[0] < 20, (params, tilt[0])
        if axis is not None:  # x = robot's right, y = forward
            assert moved[axis] * sign > 0.02, (params, moved)
        else:
            assert np.abs(moved).max() < 0.06, moved


def test_controller_gait_update_and_stop():
    c = ctl()
    r = c.gait(3, 0, steps=0)
    assert r["ok"] and not r["updated"]
    assert wait_until(lambda: "steps" in c.state()["activity"]["detail"], 5)
    assert c.gait(-2, 1)["updated"]
    assert c._activity.params["speed_v"] == -2
    assert c.gait(9, 0)["ok"] is False and c.gait(1.5, 0)["ok"] is False
    assert c.gait_stop()["ok"]
    assert c.state()["activity"]["kind"] == "idle" and not c._fallen
    assert c.gait(2, 0, steps=2)["ok"] and wait_until(lambda: c.state()["activity"]["kind"] == "idle", 12)
    assert c.gait(0, 0, period=0)["ok"]


def test_motion_files_check_and_play():
    c = ctl()
    names = [m["name"] for m in c.motion_files()]
    assert "wave_right" in names
    mo = c.load_motion("wave_right")["motion"]
    assert c.save_motion("my_wave", mo)["ok"] and "my_wave" in [m["name"] for m in c.motion_files()]
    for bad in ("../x", "a b", "", "x/y"):
        assert c.save_motion(bad, mo)["ok"] is False and c.load_motion(bad)["ok"] is False
    assert c.save_motion("broken", {"frames": "no"})["ok"] is False
    rep = c.check_motion(mo)
    assert rep["ok"] and rep["safe"] and rep["duration_s"] > 2 and rep["start"] == "sim"
    with exact_start(c):
        rep = c.check_motion(RESET_GAP)
    assert not rep["safe"] and rep["collisions"] and rep["collisions"][0]["frame"] == 2
    rep = c.check_motion({"frames": [{"angles": {"NeckLR": 500}}]})
    assert rep["errors"] and not rep["safe"]
    r = c.play_motion("my_wave", speed=2.0)
    assert r["ok"] and c.state()["activity"]["kind"] == "motion"
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 6)
    assert c.play_motion({"frames": [{"angles": {"NeckLR": 999}}]})["ok"] is False
    assert c.play_motion("nope")["ok"] is False and c.play_motion("my_wave", speed=0)["ok"] is False
    with exact_start(c):
        r = c.play_motion(RESET_GAP, speed=4)  # sim only: collisions are a warning, not a refusal
    assert r["ok"] and any("self-collision" in w for w in r["warnings"])
    c.stop_all()
    assert c.delete_motion("my_wave")["ok"] and c.delete_motion("my_wave")["ok"] is False
    c.reset()


def test_sim_settings_info_sensors_voice_vision():
    c = ctl()
    assert c.set_led("button", "red", "blink")["ok"] and c.sim.led["button"] == {"color": "red", "mode": "blink"}
    assert c.state()["leds"]["button"]["color"] == "red"
    assert c.set_led("camera", "purple", "on")["ok"] is False
    assert c.set_volume(70)["ok"] and c.state()["volume"] == 70 and c.set_volume(101)["ok"] is False
    assert c.set_language("zh")["ok"] and c.state()["language"] == "zh" and c.set_language("fr")["ok"] is False
    assert c.set_fall_management(True)["ok"] and c.state()["fall_management"] is True
    c.set_language("en")
    info = c.info()
    assert info["ok"] and info["robot"] is None and info["sim"]["versions"]["sn"] == "SIM"
    s = c.sensors()
    assert s["robot"] == "not connected" and "euler-x" in s["sim"]["gyro"]
    assert c.tts("hello")["speak_in_browser"] == "hello" and c.tts("")["ok"] is False
    v = c.vision("color")
    assert v["ok"] and v["target"] == "sim" and isinstance(v["data"]["color"], list)
    assert "not available in simulation" in c.vision("face:age")["error"]
    assert c.vision("telepathy")["ok"] is False
    assert c.face_entry("bob")["ok"] is False
    jpeg = c.photo()
    assert isinstance(jpeg, bytes) and jpeg[:2] == b"\xff\xd8"
    assert c.music_list()["source"] == "sim" and "WakaWaka" in c.music_list()["music"]
    assert c.music_play("WakaWaka")["simulated"] and c.state()["activity"]["kind"] == "music"
    assert c.music_stop()["ok"] and c.state()["activity"]["kind"] == "idle"
    c.battery.percent = 20
    assert "battery" in c.play_action("wave", "left")["error"]
    assert c.set_servos({"NeckLR": 80})["ok"]  # servo commands still work
    assert c.charge()["battery"] == 100.0


def test_renderer_cameras_and_leds():
    import numpy as np
    c = ctl()
    import cv2
    for cam in ("head", "head_camera", "front", "side", "free"):
        jpeg = c.renderer.latest(cam, timeout=10)
        assert jpeg and jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9", cam
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert img.shape == (480, 640, 3) and img.std() > 5, cam
    try:
        c.renderer.latest("ceiling")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown camera accepted")
    n = c.renderer.frame_number("front")
    jpeg, n2 = c.renderer.wait_frame("front", n, timeout=3)
    assert jpeg and n2 > n
    # LED markers exist and are animated: blink is on, then off
    for g in ("led_chest", "led_eye_left", "led_eye_right"):
        c.sim.model.geom(g)
    on, _ = sim_extras.led_rgba({"color": "red", "mode": "blink"}, 0.1)
    off, _ = sim_extras.led_rgba({"color": "red", "mode": "blink"}, 0.6)
    assert on[0] > 0.9 and off[0] < 0.2
    assert sim_extras.led_rgba({"color": "green", "mode": "off"}, 0)[0][1] < 0.2
    img = np.zeros((48, 64, 3), np.uint8)
    img[:, :32] = (220, 20, 20)
    img[:24, 32:] = (20, 40, 230)
    names = [col["name"] for col in sim_extras.detect_colours(img)["color"]]
    assert names == ["red", "blue"], names


# -- robot and both modes against the fake robot -------------------------------------------------
def test_unreachable_robot():
    c = ctl()
    import socket
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    t = time.monotonic()
    r = c.connect(f"127.0.0.1:{port}")
    assert r["ok"] is False and "cannot connect" in r["error"] and time.monotonic() - t < 5
    assert c.connect("")["ok"] is False and c.state()["robot"]["connected"] is False


def test_gait_turn_and_viewer_keys():
    """turn= rotates the sim in place; the robot refuses it (its built-in turn-around is the way); keys from the
    MuJoCo viewer window drive a leased walk, renew it on repeats and STOP on X/Esc."""
    c = ctl()
    assert c.gait(0, 0, steps=0, turn=9)["ok"] is False
    r = c.gait(0, 0, steps=0, turn=3)
    assert r["ok"] and wait_until(lambda: "anticlockwise" in c.state()["activity"]["detail"], 5)
    assert c.gait(0, 0, steps=0, turn=-2)["updated"] and c._activity.params["turn"] == -2
    assert c.gait_stop()["ok"] and wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
    # viewer keys: W = forward with a 0.9 s lease; a repeat renews it; X stops
    c._viewer_key(87)
    assert wait_until(lambda: c.state()["activity"]["kind"] == "gait", 5)
    act = c._activity
    assert act.params["speed_v"] == Controller.VIEWER_SPEED and act.lease_s == Controller.VIEWER_LEASE_S
    time.sleep(0.6)
    c._viewer_key(87)  # a key repeat
    time.sleep(0.6)
    assert c.state()["activity"]["kind"] == "gait" and c._activity is act  # renewed, not restarted
    c._viewer_key(81)  # Q: turn anticlockwise instead (the walk is updated in place)
    assert wait_until(lambda: c._activity is act and act.params.get("turn") == 3 and act.params["speed_v"] == 0, 5)
    c._viewer_key(88)  # X = STOP
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
    c._viewer_key(265)  # arrow up, then let the lease run out
    assert wait_until(lambda: c.state()["activity"]["kind"] == "gait", 5)
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 6)
    assert c._viewer_key(9999) is None  # unknown keys are ignored
    with Connected("robot") as (c2, srv):
        r = c2.gait(0, 0, steps=0, turn=2, lease_s=1.0)  # AppWalk: the robot turns like the mobile app
        assert r["ok"], r
        turns = lambda: [e[3] for e in requests(srv, "PUT", "motions") if (e[3].get("motion") or {}).get("name") == "TurnLeft"]  # noqa: E731
        assert wait_until(lambda: turns(), 5) and turns()[0]["operation"] == "start_async" and turns()[0]["motion"]["repeat"] == 0
        assert c2.gait(0, 0, steps=0, turn=2, lease_s=1.0)["updated"]  # held: resent at most every 0.4 s
        assert wait_until(lambda: c2.state()["activity"]["kind"] == "idle", 6)  # lease ran out: the app's stop
        stops = [e[3] for e in requests(srv, "PUT", "motions/gait") if e[3].get("period") == 0]
        assert stops and stops[-1]["speed_v"] == 0 and stops[-1]["speed_h"] == 0
        c2._viewer_key(69)  # E in robot mode: TurnRight the same way
        assert wait_until(lambda: any((e[3].get("motion") or {}).get("name") == "TurnRight" for e in requests(srv, "PUT", "motions")), 5)


def test_get_up_and_balance():
    """get_up: the sim plays its get-up motion (if there is one) and is stood up when still down; the robot gets
    GetupFront/GetupRear by its IMU. balance_margin_cm is in the state; the standing reflex is off by default."""
    c = ctl()
    assert c.reflex_enabled is False
    st = c.state()["sim"]
    assert st["balance_margin_cm"] is None or st["balance_margin_cm"] < 0  # standing: the capture point is inside
    assert c.get_up() == {"ok": True, "target": "sim", "standing": True}
    head = c.sim.model.body("NeckLR").id
    with c.sim._lock:
        c.sim.data.xfrc_applied[head] = [0, 12.0, 0, 0, 0, 0]  # a push at the head: it falls on its face
    time.sleep(0.4)
    with c.sim._lock:
        c.sim.data.xfrc_applied[head] = 0
    assert wait_until(lambda: c.state()["sim"]["fallen"], 6)
    r = c.get_up()
    assert r["ok"] and r["posture"] == "front", r
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle" and not c.state()["sim"]["fallen"], 20)
    assert c.sim.torso_tilt_deg() < 15
    assert any("got up" in e["text"] or "stood up" in e["text"] for e in c.events_since(0))
    with Connected("robot") as (c2, srv):
        r = c2.get_up()  # the fake robot reports euler-x 89.5 (upright) -> |x| < 90 -> face down -> Base_GetupF
        assert r["ok"] and r["motion"] == "Base_GetupF", r
        names = lambda: [e[3].get("motion", {}).get("name") or e[3].get("name") for e in requests(srv, "PUT", "motions")]  # noqa: E731
        assert wait_until(lambda: "Base_GetupF" in names(), 5)
        assert wait_until(lambda: "reset" in names(), 15), names()  # it ends crouched: reset follows when nothing is locked


def test_tip_over_then_get_up():
    """The simulator test button: a gentle push fells the robot (front/back), get_up() then stands it up; refused
    with a robot connected."""
    c = ctl()
    assert c.tip_over("sideways")["ok"] is False
    for direction in ("front", "back"):
        r = c.tip_over(direction)
        assert r["ok"] and r["direction"] == direction, r
        assert wait_until(lambda: c.state()["sim"]["fallen"], 8), direction
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
        g = c.sim.get_sensors_gyro()["data"]["gyro"][0]
        assert (abs(g["euler-x"]) < 60) == (direction == "front"), (direction, g["euler-x"])  # face down ~0, on the back ~180
        r = c.get_up()
        assert r["ok"] and r["posture"] == ("front" if direction == "front" else "rear"), r
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle" and not c.state()["sim"]["fallen"], 20)
    assert c.tip_over("front")["ok"]  # standing again: a new push is accepted
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
    assert c.get_up()["ok"]
    assert wait_until(lambda: not c.state()["sim"]["fallen"], 20)
    with Connected("robot") as (c2, srv):
        assert c2.tip_over("front")["ok"] is False


def test_gait_direction_changes_do_not_tip():
    """A second key during a walk, or a reversal, must not throw the robot: the gait ramps every change."""
    c = ctl()
    assert c.gait(5, 0, steps=0, lease_s=30)["ok"]
    time.sleep(2.0)
    assert c.gait(5, -5, steps=0, lease_s=30)["updated"]  # forward -> forward + right
    time.sleep(2.0)
    assert c.gait(-5, 0, steps=0, lease_s=30)["updated"]  # -> backward
    time.sleep(2.5)
    assert c.gait_stop()["ok"] and wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
    assert not c.state()["sim"]["fallen"] and c.sim.torso_tilt_deg() < 15


def test_robot_gait_stop_like_the_app():
    """A normal stop is the mobile app's: one gait command with every value 0 (AppWalk); STOP exits at once."""
    with Connected("robot") as (c, srv):
        assert c.gait(5, 2, steps=0)["ok"]
        assert wait_until(lambda: c.state()["activity"]["kind"] == "gait", 5)
        srv.state.log.clear()
        r = c.gait_stop()
        assert r["ok"], r
        sent = [(e[3]["speed_v"], e[3]["speed_h"], e[3]["period"]) for e in requests(srv, "PUT", "motions/gait")]
        assert sent == [(0, 0, 0)], sent
        assert not requests(srv, "DELETE", "motions/gait") and c.state()["activity"]["kind"] == "idle"
        assert c.gait(-4, 0, steps=0)["ok"]
        assert wait_until(lambda: c.state()["activity"]["kind"] == "gait", 5)
        srv.state.log.clear()
        assert c.stop_all()["ok"]
        assert not requests(srv, "PUT", "motions/gait") and requests(srv, "DELETE", "motions/gait")
        assert c.gait(1, 0, steps=0)["ok"]
        assert wait_until(lambda: c.state()["activity"]["kind"] == "gait", 5)
        srv.state.log.clear()
        assert c.gait_stop()["ok"] and [e[3]["period"] for e in requests(srv, "PUT", "motions/gait")] == [0]


def test_robot_mode_gate_twin_and_stop():
    with Connected("robot") as (c, srv):
        st = c.state()
        assert st["robot"]["connected"] and st["robot"]["versions"]["sn"] == "FAKE00000001"
        assert st["servos"]["robot"]["NeckLR"] == 90
        srv.state.log.clear()
        # servo command through the gate: sent to the robot only, runtime stretched by the speed cap
        r = c.set_servos({"NeckLR": 20}, runtime=200)
        assert r["ok"] and r["sent_to"] == ["robot"] and r["runtime"] >= 70 / 90 * 1000
        assert any("stretched" in w for w in r["warnings"])
        put = requests(srv, "PUT", "servos/angles")
        assert put and put[-1][3]["angles"] == {"NeckLR": 20} and put[-1][3]["runtime"] == r["runtime"]
        assert wait_until(lambda: c.get_servos()["robot"]["NeckLR"] == 20, 3)
        # twin: the sim follows the robot's readback
        assert wait_until(lambda: abs(c.get_servos()["sim"]["NeckLR"] - 20) <= 3, 4)
        srv.state.move_by_hand({"LeftShoulderFlex": 100})
        assert wait_until(lambda: abs(c.get_servos()["sim"]["LeftShoulderFlex"] - 100) <= 4, 4)
        srv.state.move_by_hand({"LeftShoulderFlex": 40})
        # a self-collision is refused and nothing reaches the robot
        assert wait_until(lambda: not c.gate.busy(margin=0.5), 3)
        n = len(requests(srv, "PUT", "servos/angles"))
        r = c.set_servos(LEGS_CROSS)
        assert r["ok"] is False and r["blocked"] and "self-collision" in r["error"]
        assert len(requests(srv, "PUT", "servos/angles")) == n
        # limits are clamped with a warning
        r = c.set_servos({"RightAnkleUD": 10, "NeckLR": 90}, 1500)
        assert r["ok"] and requests(srv, "PUT", "servos/angles")[-1][3]["angles"]["RightAnkleUD"] == 65
        # built-in action on the robot; STOP reaches every robot target
        r = c.play_action("WakaWaka")
        assert r["ok"] and r["target"] == "robot" and srv.state.motion["status"] == "run"
        assert c.state()["activity"]["kind"] == "action"
        srv.state.log.clear()
        assert c.stop_all()["ok"]
        log = srv.state.log
        stops = [e for e in log if e[:2] == ("PUT", "motions") and e[3].get("operation") == "stop"]
        assert {e[3].get("version") for e in stops} >= {"v1", "v2"}
        for path in ("motions/gait", "media/music", "voice/tts"):
            assert any(e[1] == path and e[0] in ("DELETE", "PUT") for e in log), path
        assert srv.state.motion["status"] == "idle" and c.state()["activity"]["kind"] == "idle"
        # a robot built-in that ends by itself ends the activity
        assert c.play_action("wave", "left", speed="very fast")["ok"]
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 6)
        # record a frame from the robot's readback
        srv.state.move_by_hand({"NeckLR": 120})
        rec = c.record_frame(800)
        assert rec["ok"] and rec["source"] == "robot" and rec["frame"]["angles"]["NeckLR"] == 120
        # servo modes on the robot (PRP)
        assert c.set_servo_mode("program", ["LeftShoulderFlex"])["ok"] and srv.state.modes["LeftShoulderFlex"] == "program"
        assert c.set_servo_mode("work", ["LeftShoulderFlex"])["ok"] and srv.state.modes["LeftShoulderFlex"] == "work"


def test_robot_mode_motion_files_gait_music_settings():
    with Connected("robot") as (c, srv):
        assert c.reset()["ok"]
        assert wait_until(lambda: not c.gate.busy(), 4)
        srv.state.log.clear()
        r = c.play_motion("wave_right", speed=2)
        assert r["ok"] and r["target"] == "robot"
        assert wait_until(lambda: len(requests(srv, "PUT", "servos/angles")) >= 3, 4)
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
        sent = [e[3]["angles"] for e in requests(srv, "PUT", "servos/angles")]
        assert sent[0]["RightShoulderFlex"] == 30 and sent[-1] == RESET_POSE
        r = c.play_motion(RESET_GAP)  # sweep includes the end_in_reset move from the current pose
        assert r["ok"] is False and r["collisions"] and "self-collision" in r["error"]
        # gait: start, change speed in place, stop
        srv.state.log.clear()
        assert c.gait(2, 0)["ok"] and srv.state.gait["status"] == 2
        assert c.gait(3, -1)["updated"]
        gaits = requests(srv, "PUT", "motions/gait")
        assert gaits[-1][3]["speed_v"] == 3 and gaits[-1][3]["speed_h"] == -1
        assert c.gait_stop()["ok"] and srv.state.gait["status"] == 8
        assert [e for e in requests(srv, "PUT", "motions/gait") if e[3]["period"] == 0]  # the app's stop
        # music
        ml = c.music_list()
        assert ml["source"] == "robot" and "WakaWaka.mp3" in ml["music"]
        assert c.music_play("WakaWaka.mp3")["ok"] and srv.state.music["status"] == "run"
        assert c.music_stop()["ok"] and srv.state.music["status"] == "idle"
        # settings reach the robot
        assert c.set_led("button", "green", "on")["ok"] and srv.state.leds["button"] == {"color": "green", "mode": "on"}
        assert c.set_volume(33)["ok"] and srv.state.volume == 33
        assert c.set_language("zh")["ok"] and srv.state.language == "zh"
        assert c.set_fall_management(True)["ok"] and srv.state.fall is True
        c.set_language("en")
        info = c.info()
        assert info["robot"]["percent"] == 90 and info["robot"]["flags"]["calibration_mode"] is False
        s = c.sensors()
        assert "euler-x" in s["robot"]["gyro"] and s["robot"]["addons"]["environment"][0]["temperature"] == 24
        assert c.tts("hello there")["target"] == "robot" and srv.state.tts["text"] == "hello there"
        v = c.vision("face:age", timeout=5)
        assert v["ok"] and v["data"] == {"analysis": {"age": 30}}
        assert c.vision("qr", timeout=5)["data"]["contents"]
        assert c.vision("color", timeout=5)["data"]["color"][0]["name"] == "red"
        jpeg = c.photo()
        assert isinstance(jpeg, bytes) and jpeg[:2] == b"\xff\xd8"
        assert c.face_entry("visitor")["ok"] and "visitor" in srv.state.tags
        assert c.open_robot_stream()["url"].endswith(":8000/stream.mjpg")
        acts = {a["name"]: a for a in c.list_actions()}
        assert acts["WakaWaka"]["source"] == "robot" and acts["H_Wave_R"]["version"] == "v2"
        assert c.play_action("H_Wave_R", version="v2")["ok"] and srv.state.motion["version"] == "v2"
        c.stop_all()


def test_mode_switch_stops_activity():
    with Connected("robot") as (c, srv):
        assert c.play_action("GangnamStyle")["ok"] and srv.state.motion["status"] == "run"
        c.set_mode("sim")
        assert srv.state.motion["status"] == "idle" and c.state()["activity"]["kind"] == "idle"
        assert c.mode == "sim" and c.play_action("GangnamStyle")["ok"] is False


def test_both_mode():
    with Connected("both", motion_seconds=6) as (c, srv):
        r = c.set_servos({"NeckLR": 70}, 600)
        assert r["ok"] and set(r["sent_to"]) == {"sim", "robot"}
        assert wait_until(lambda: abs(c.get_servos()["sim"]["NeckLR"] - 70) <= 2, 3)
        assert wait_until(lambda: c.get_servos()["robot"]["NeckLR"] == 70, 3)
        # a built-in with a sim version runs on both: the robot's own, and the sim file
        r = c.play_action("raise", "right")
        assert r["ok"] and r["target"] == "both" and not r["twin"]
        assert srv.state.motion["name"] == "raise"
        assert wait_until(lambda: c.get_servos()["sim"]["RightShoulderFlex"] < 60, 3)
        c.stop_all()
        # robot-only built-in: the sim follows the robot's readback while it runs
        assert c.get_servos()["sim"]["RightShoulderFlex"] < 100  # held where STOP left the sim's raise
        r = c.play_action("LittleApple")
        assert r["ok"] and r["twin"] and c._twin_active()

        def follows():
            sim, robot = c.get_servos()["sim"], c.get_servos()["robot"]
            return abs(sim["RightShoulderFlex"] - robot["RightShoulderFlex"]) <= 15 and sim["RightShoulderFlex"] > 100
        assert wait_until(follows, 4)
        c.stop_all()
        assert c._twin_active()  # idle with a robot connected: the sim mirrors it
        r = c.play_motion("wave_right", speed=2)  # the sim's own version: it stops mirroring and plays it
        assert r["ok"] and r["target"] == "both"
        assert wait_until(lambda: c.get_servos()["sim"]["RightShoulderFlex"] < 60, 3)
        c.stop_all()
        assert c.tts("hi")["target"] == "robot"


def test_low_battery_refused():
    with Connected("robot", battery=20) as (c, srv):
        assert any("below 30%" in e["text"] for e in c.events_since(0))
        r = c.play_action("wave", "right")
        assert r["ok"] is False and "battery 20%" in r["error"]
        assert c.play_motion("wave_right")["ok"] is False and c.gait(2, 0)["ok"] is False
        assert not [e for e in srv.state.log if e[:2] == ("PUT", "motions") and e[3].get("operation") == "start"]
        assert c.set_servos({"NeckLR": 80}, 500)["ok"]  # servo commands still work
        assert c.set_volume(40)["ok"] and c.stop_all()["ok"]


def test_refusing_robot_is_reported():
    with Connected("robot", refuse=True) as (c, srv):
        r = c.set_servos({"NeckLR": 80}, 500)
        assert r["ok"] is False and "refused" in r["error"]
        r = c.play_action("wave", "right")
        assert r["ok"] is False and "code 1" in r["error"]
        assert c.set_servo_mode("program", ["NeckLR"])["ok"] is False
        assert c.play_motion("wave_right")["ok"]  # starts; the first frame fails in the activity thread
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 4)
        assert any("motion 'wave_right' stopped" in e["text"] for e in c.events_since(0))
        assert c.stop_all()["ok"]


def test_robot_disappears():
    with Connected("robot") as (c, srv):
        srv.state.dead = True  # stops answering but keeps its port (a freed port could be taken by another fake)
        assert c.set_servos({"NeckLR": 80}, 500)["ok"] is False
        assert c.play_action("wave", "right")["ok"] is False
        assert c.info()["ok"] and "battery_error" in c.info()["robot"]
        assert c.stop_all()["ok"]
        assert wait_until(lambda: c.state()["robot"]["online"] is False, 8)
        assert any("not answering" in e["text"] for e in c.events_since(0))


def test_robot_dies_mid_activity():
    """A robot built-in or gait whose robot stops answering ends (with an error event) instead of polling for minutes."""
    for start in (lambda c: c.play_action("WakaWaka"), lambda c: c.gait(2, 0)):
        with Connected("both", motion_seconds=60) as (c, srv):
            assert start(c)["ok"]
            assert c.state()["activity"]["kind"] in ("action", "gait")
            last = c.events[-1]["id"] if c.events else 0
            srv.state.dead = True  # dead robot, port kept: nothing else can answer in its place
            assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 12)
            assert any("stopped answering" in e["text"] and e["level"] == "error" for e in c.events_since(last))


# -- races and safety fixes (review findings) ---------------------------------------------------------
def test_stop_while_command_waits_refuses_it():
    """A command waiting for the previous activity to end is refused when STOP lands meanwhile (sim)."""
    c = ctl()
    assert c.gait(3, 0)["ok"]
    out = {}
    t = threading.Thread(target=lambda: out.update(r=c.play_action("wave", "right")))
    t.start()
    time.sleep(0.15)
    assert c.stop_all()["ok"]
    t.join(10)
    assert out["r"]["ok"] is False and "STOP" in out["r"]["error"], out
    assert c.state()["activity"]["kind"] == "idle"
    c.reset()


def test_mode_switch_race_sends_nothing_after_the_stops():
    """play_action / set_servos issued just before a switch to Sim never reach the robot after the stops."""
    for call in (lambda c: c.play_action("bow"), lambda c: c.set_servos({"NeckLR": 30}, 4000)):
        with Connected("robot", motion_seconds=20) as (c, srv):
            assert c.play_action("wave", "right")["ok"]
            time.sleep(0.4)
            srv.state.slow = 0.6
            out = {}
            t = threading.Thread(target=lambda: out.update(r=call(c)))
            t.start()
            time.sleep(0.2)
            n0 = len(srv.state.log)
            assert c.set_mode("sim")["mode"] == "sim"
            t.join(15)
            srv.state.slow = 0
            assert out["r"]["ok"] is False, out
            later = [e for e in list(srv.state.log)[n0:] if e[0] != "GET"]
            assert not any(e[1] == "motions" and e[3].get("operation") == "start" for e in later), later
            assert not any(e[1] == "servos/angles" and e[3]["angles"].get("NeckLR") == 30 for e in later), later
            assert c.state()["activity"]["kind"] == "idle" and srv.state.motion["status"] == "idle"


def test_gait_update_during_stop_is_refused():
    """A d-pad change arriving while STOP winds the walk down must not restart it (robot, slow link)."""
    with Connected("robot") as (c, srv):
        assert c.gait(2, 0)["ok"] and srv.state.gait["status"] == 2
        srv.state.slow = 0.5
        t = threading.Thread(target=c.stop_all)
        t.start()
        time.sleep(0.15)
        r = c.gait(3, 0)
        t.join(10)
        srv.state.slow = 0
        assert r["ok"] is False and "STOP" in r["error"], r
        time.sleep(0.4)
        assert srv.state.gait["status"] == 8 and c.state()["activity"]["kind"] == "idle"


def test_gait_stop_reaches_robot_in_sim_mode():
    with Connected("robot") as (c, srv):
        c.set_mode("sim")
        n = len(requests(srv, "DELETE", "motions/gait"))
        assert c.gait_stop()["ok"] and len(requests(srv, "DELETE", "motions/gait")) == n + 1


def test_gait_lease_dead_man():
    """An open-ended walk with a lease stops by itself when keepalives stop (sim and robot)."""
    c = ctl()
    assert c.gait(2, 0, lease_s=0.8)["ok"]
    for _ in range(4):  # keepalives hold it
        time.sleep(0.4)
        assert c.gait_keepalive()["walking"]
    assert c.state()["activity"]["kind"] == "gait"
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 5)
    assert any("no keepalive" in e["text"] for e in c.events_since(0))
    assert c.gait_keepalive()["walking"] is False
    assert c.gait(2, 0, lease_s=0)["ok"] is False
    c.reset()
    with Connected("robot") as (c, srv):
        assert c.gait(3, 0, lease_s=0.6)["ok"] and srv.state.gait["status"] == 2
        assert wait_until(lambda: srv.state.gait["status"] == 8, 4)
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 3)


def test_system_motions_refused():
    with Connected("robot") as (c, srv):
        names = {a["name"].lower() for a in c.list_actions()}
        for name in ("Shutdown", "ActionAging", "calibration", "ResetSwitch", "shut down", "EnterEnergySaving"):
            assert name.lower() not in names
            r = c.play_action(name)
            assert r["ok"] is False and "system motion" in r["error"], (name, r)
        assert not [e for e in requests(srv, "PUT", "motions") if e[3].get("operation") == "start"]
        try:
            c.robot.play_builtin("Shutdown")
        except ValueError:
            pass
        else:
            raise AssertionError("RealRobot.play_builtin started a system motion")


def test_stop_holds_a_servo_move_in_flight():
    with Connected("robot") as (c, srv):
        r = c.set_servos({"RightShoulderFlex": 0}, 4000)
        assert r["ok"] and r["sent_to"] == ["robot"]
        time.sleep(0.5)
        res = c.stop_all()
        assert res["ok"] and res["held"]
        at_stop = srv.state.angle("RightShoulderFlex", time.monotonic())
        time.sleep(1.0)
        assert abs(srv.state.angle("RightShoulderFlex", time.monotonic()) - at_stop) <= 3
        assert not c.gate.busy(margin=0.3)


def test_motion_file_obeys_the_robot_speed_cap():
    fast = {"frames": [{"angles": {"RightShoulderFlex": 0, "LeftShoulderFlex": 180}, "runtime": 200}],
            "end_in_reset": True}
    with Connected("robot") as (c, srv):
        assert c.reset()["ok"]
        assert wait_until(lambda: not c.gate.busy(), 4)
        rep = c.check_motion(fast)
        assert rep["safe"] and any("speed cap" in w for w in rep["warnings"]) and rep["duration_s"] > 2.5, rep
        srv.state.log.clear()
        assert c.play_motion(fast)["ok"]
        assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 10)
        sent = [e[3] for e in requests(srv, "PUT", "servos/angles")]
        assert len(sent) == 2, sent
        for cmd in sent:  # 140 deg moves: never faster than 90 deg/s
            assert cmd["runtime"] >= 140 / c.max_speed * 1000 - 1, cmd


def test_set_servos_after_builtin_starts_from_fresh_pose():
    with Connected("robot", motion_seconds=20) as (c, srv):
        assert c.reset()["ok"]
        assert wait_until(lambda: not c.gate.busy(), 4)
        assert c.play_action("wave", "right")["ok"]
        time.sleep(0.4)
        srv.state.move_by_hand({"NeckLR": 20})  # the built-in moved a servo the gate did not see
        r = c.set_servos({"NeckLR": 160}, 200)
        assert r["ok"] and r["runtime"] >= 140 / c.max_speed * 1000 - 1, r


def test_timed_out_start_is_stopped():
    """A built-in whose start request timed out (slow link) gets a best-effort stop instead of running untracked."""
    with Connected("robot", motion_seconds=20) as (c, srv):
        c.robot.rest.timeout = 0.5
        srv.state.slow = 0.8
        r = c.play_action("wave", "right")
        assert r["ok"] is False and "may still" in r["error"], r
        srv.state.slow = 0
        c.robot.rest.timeout = c.robot_timeout
        assert wait_until(lambda: srv.state.motion["name"] == "wave", 3)
        assert wait_until(lambda: srv.state.motion["status"] == "idle", c.robot_timeout + 4)


def test_pause_resume_and_tts_stop():
    with Connected("robot", motion_seconds=20) as (c, srv):
        assert c.pause_action()["ok"] is False  # nothing playing
        assert c.play_action("WakaWaka")["ok"]
        assert c.pause_action()["ok"] and srv.state.motion["status"] == "pause"
        assert c.state()["activity"]["detail"] == "paused"
        time.sleep(0.8)
        assert c.state()["activity"]["kind"] == "action"
        assert c.resume_action()["ok"] and srv.state.motion["status"] == "run"
        assert c.tts("hello there")["ok"] and srv.state.tts["status"] == "run"
        assert c.tts_stop()["ok"] and srv.state.tts["status"] == "idle"
        assert srv.state.motion["status"] == "run" and c.state()["activity"]["kind"] == "action"  # dance goes on
        assert c.stop_all()["ok"]
    assert ctl().tts_stop() == {"ok": True, "target": "sim"}


def test_leg_limp_warns_on_robot():
    with Connected("robot") as (c, srv):
        r = c.set_servo_mode("program")
        assert r["ok"] and any("leg servos" in w for w in r["warnings"])
        assert not c.set_servo_mode("program", ["NeckLR"])["warnings"]
        assert c.set_servo_mode("work")["ok"]


def test_check_report_labels_frames_from_one():
    c = ctl()
    with exact_start(c):
        rep = c.check_motion(RESET_GAP)
    assert rep["collisions"] and rep["collisions"][0]["label"] == "return to reset"
    errors, _ = motion.validate({"frames": [{"angles": {"NeckLR": 90}, "runtime": 50}]})
    assert errors[0].startswith("frame 1:"), errors


# -- concurrency and soak ------------------------------------------------------------------------
def test_concurrency_smoke():
    c = ctl()
    errors, results = [], []
    calls = [lambda: c.state(), lambda: c.get_servos(), lambda: c.list_actions(),
             lambda: c.set_servos({"NeckLR": random.randint(40, 140)}, 300),
             lambda: c.play_action("wave", random.choice(["left", "right", "both"]), speed="very fast"),
             lambda: c.play_action("head", random.choice(["left", "right", "forward"])),
             lambda: c.gait(random.randint(-2, 3), 0, steps=2), lambda: c.gait_stop(), lambda: c.stop_all(),
             lambda: c.set_led("button", random.choice(["red", "blue"]), "on"), lambda: c.sensors(),
             lambda: c.check_motion("wave_right"), lambda: c.record_frame(500), lambda: c.info(),
             lambda: c.music_play("WakaWaka"), lambda: c.events_since(0)]
    stop_at = time.monotonic() + 8

    def worker():
        while time.monotonic() < stop_at:
            try:
                results.append(random.choice(calls)())
            except Exception as e:  # noqa: BLE001
                errors.append(repr(e))
    threads = [threading.Thread(target=worker) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(30)
    assert not any(t.is_alive() for t in threads), "deadlock"
    assert not errors, errors[:5]
    assert len(results) > 40
    json.dumps(c.state())
    c.stop_all()
    assert c.state()["activity"]["kind"] == "idle"
    c.reset()


def test_disconnect_races_robot_commands():
    """disconnect() between _targets() and the safety gate gives {ok: False}, never an exception (was AttributeError
    on gate None -> HTTP 500)."""
    c, srv = ctl(), start_fake()
    small = {"frames": [{"angles": {"NeckLR": 70}, "runtime": 300}], "end_in_reset": False}
    calls = {"set_servos": lambda: c.set_servos({"NeckLR": 80}, 300), "reset": lambda: c.reset(),
             "set_servo_mode": lambda: c.set_servo_mode("work", ["NeckLR"]), "play_motion": lambda: c.play_motion(small)}
    try:
        for name, call in calls.items():
            assert c.connect(srv.address)["ok"]
            c.set_mode("both")
            fired = []

            def racing(orig=c._snapshot):  # the race, made deterministic: disconnect right after the targets are read
                res = orig()
                if not fired:
                    fired.append(1)
                    c.disconnect()
                return res
            c._snapshot = racing
            try:
                r = call()
            finally:
                del c._snapshot
            assert fired and isinstance(r, dict), name
            assert r["ok"] is False and "disconnected" in r["error"], (name, r)
            assert c.robot is None and c.gate is None and c.mode == "sim", name
        # and under real contention
        errors, stop_at = [], time.monotonic() + 3

        def worker():
            while time.monotonic() < stop_at:
                try:
                    random.choice(list(calls.values()))()
                except Exception as e:  # noqa: BLE001
                    errors.append(repr(e))
        threads = [threading.Thread(target=worker) for _ in range(4)]
        for th in threads:
            th.start()
        while time.monotonic() < stop_at:
            assert c.connect(srv.address)["ok"]
            c.set_mode("both")
            c.disconnect()
        for th in threads:
            th.join(30)
        assert not errors, errors[:5]
    finally:
        c.disconnect()
        srv.stop()
    c.stop_all()
    c.reset()


def test_soak_sim():
    """Play actions and gait back to back (default 60 s): no exceptions, no fall."""
    c = ctl()
    seconds = float(os.environ.get("YANSHEE_SOAK_S", "60"))
    first_id = c.events_since(0)[-1]["id"] if c.events else 0
    plan = [("action", ("wave", "both")), ("gait", (4, 0, 6)), ("action", ("crouch", "")),
            ("action", ("bend", "left")), ("gait", (0, 3, 4)), ("action", ("raise", "both")),
            ("gait", (-3, 0, 4)), ("action", ("bow", "")), ("action", ("come on", "right")),
            ("gait", (2, -2, 4)), ("action", ("stretch", "left")), ("action", ("head", "right")),
            ("action", ("reset", ""))]
    max_tilt, i, t_end = 0.0, 0, time.monotonic() + seconds
    while time.monotonic() < t_end:
        kind, args = plan[i % len(plan)]
        i += 1
        r = c.play_action(*args, speed="fast") if kind == "action" else c.gait(args[0], args[1], steps=args[2])
        assert r["ok"], r
        while c.state()["activity"]["kind"] != "idle" and time.monotonic() < t_end + 20:
            st = c.state()
            max_tilt = max(max_tilt, st["sim"]["tilt_deg"])
            assert not st["sim"]["fallen"], (kind, args, st["sim"])
            time.sleep(0.1)
    bad = [e for e in c.events_since(first_id) if e["level"] == "error"]
    assert not bad, bad
    assert max_tilt < 25 and i >= min(5, seconds / 12), (max_tilt, i)


def test_zz_close():
    if "c" in _CTL:
        _CTL.pop("c").close()
        shutil.rmtree(_CTL.pop("dir"), ignore_errors=True)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            t = time.monotonic()
            fn()
            print("ok", name, f"({time.monotonic() - t:.1f} s)")


def test_both_mode_mirrors_the_robot():
    """With a robot connected the sim mirrors it (encoders and IMU, physics paused), also while walking in Both mode:
    the robot walks (AppWalk by default) and the sim shows it instead of walking on its own."""
    with Connected("both") as (c, srv):
        assert wait_until(lambda: c.sim.mirroring, 5), "the sim does not mirror the connected robot"
        x0 = float(c.sim.data.qpos[1])
        assert c.gait(2, 0, steps=0, lease_s=3.0)["ok"]
        assert wait_until(lambda: any(e[3].get("speed_v") == 2 for e in requests(srv, "PUT", "motions/gait")), 5)
        time.sleep(1.5)
        assert c.sim.mirroring and abs(float(c.sim.data.qpos[1]) - x0) < 0.02  # the sim did not walk off by itself
        st = c.state()
        assert st["robot"]["walk_method"] == "app" and st["robot"]["mirroring"] is True
        assert c.gait_stop()["ok"]
        c.set_mode("sim")
        assert wait_until(lambda: not c.sim.mirroring, 3), "Sim mode must give the physics back"


def test_walk_method_setting():
    c = ctl()
    assert c.set_walk_method("jog")["ok"] is False
    assert c.set_walk_method("sway")["walk_method"] == "sway" and c.walk_method == "sway"
    assert c.gait(1, 0, steps=0, lease_s=2.0)["ok"]  # sim mode: SwayStep in the sim whatever the robot method
    assert c.set_walk_method("app")["ok"] is False  # not while walking
    c.gait_stop()
    assert wait_until(lambda: c.state()["activity"]["kind"] == "idle", 8)
    assert c.set_walk_method("app")["ok"] and c.walk_method == "app"
