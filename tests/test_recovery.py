"""Fall detection and recovery for the real robot (yanshee_sim/recovery.py) and the controller's auto get-up switch.
Run: python3 -m pytest tests/test_recovery.py -q"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from yanshee_sim import recovery  # noqa: E402
from yanshee_sim.rest import YansheeRest  # noqa: E402
import fake_robot  # noqa: E402


def feed(det, samples):
    out = [det.feed(t, ex, g) for t, ex, g in samples]
    return [{k: e[k] for k in ("posture", "fell")} for e in out if e]  # also carries down_t: when it went down


def test_posture_bands():
    assert [recovery.posture(x) for x in (92, -1, 179, -178, 45, 130)] == \
        ["upright", "face_down", "on_back", "on_back", "tilted", "tilted"]


def test_fast_fall_is_reported_once_when_still():
    det = recovery.FallDetector()
    up = [(t / 10, 92.0, 1.0) for t in range(10)]
    fall = [(1.0, 60.0, 150.0), (1.2, 25.0, 200.0), (1.4, 2.0, 80.0)]  # 90 deg in 0.4 s
    still = [(1.4 + t / 10, 1.0, 2.0) for t in range(1, 15)]
    events = feed(det, up + fall + still)
    assert events == [{"posture": "face_down", "fell": True}]
    assert feed(det, [(3.0, 1.0, 2.0), (3.5, 1.0, 2.0)]) == []  # reported once per time down
    feed(det, [(4.0, 92.0, 1.0)])  # upright again: the next fall is new
    assert feed(det, [(4.5, 178.0, 50.0)] + [(4.5 + t / 10, 179.0, 1.0) for t in range(1, 15)]) == \
        [{"posture": "on_back", "fell": True}]


def test_slow_lay_down_is_not_a_fall():
    det = recovery.FallDetector()
    slow = [(t / 10, 92.0 - t * 3.0, 15.0) for t in range(31)]  # 90 deg over 3 s, by hand
    still = [(3.0 + t / 10, 0.5, 1.0) for t in range(1, 15)]
    assert feed(det, slow + still) == [{"posture": "face_down", "fell": False}]


def test_not_still_waits():
    det = recovery.FallDetector()
    events = feed(det, [(0.0, 92.0, 1.0), (0.5, 2.0, 120.0)] + [(0.5 + t / 10, 2.0, 60.0) for t in range(1, 20)])
    assert events == []  # still moving (being carried, rolling): nothing yet


def test_robot_get_up_against_fake_robot():
    srv = fake_robot.start(port=0)
    try:
        rest = YansheeRest(f"127.0.0.1:{srv.server_address[1]}", timeout=4.0)
        ok, msg = recovery.robot_get_up(rest, "face_down")  # the fake robot stands upright: euler-x 89.5
        names = [(e[3].get("motion") or {}).get("name") or e[3].get("name") for e in srv.state.log if e[0] == "PUT" and e[1] == "motions"]
        assert ok and "Base_GetupF + reset" in msg, msg
        assert names[:2] == ["Base_GetupF", "reset"], names
        ok, msg = recovery.robot_get_up(rest, "on_back")
        assert ok and msg.startswith("GetupRear:"), msg
        assert recovery.locked_servos(rest) == []
    finally:
        srv.shutdown()


def test_auto_get_up_switch_and_state():
    from test_controller import Connected
    with Connected("robot") as (c, srv):
        assert c.state()["robot"]["auto_get_up"] is False
        assert c.set_auto_get_up(True)["auto_get_up"] is True
        st = c.state()["robot"]
        assert st["auto_get_up"] is True and st["posture"] in (None, "upright")
        c.set_auto_get_up(False)


def test_walk_stats_and_report(tmp_path=None):
    import json
    import subprocess
    import tempfile
    from yanshee_sim.walkstats import WalkStats
    path = os.path.join(tempfile.mkdtemp(), "stats.jsonl")
    s = WalkStats("firmware", "smooth", path=path)
    s.command("walk+")
    s.command("walk+")  # same command: no switch
    s.command("turn_left")
    s.fall("face_down")
    s.getup(True, "Base_GetupF + reset: standing")
    s.command("walk-")
    s.fall("side_left")
    s.close()
    rows = [json.loads(line) for line in open(path)]
    kinds = [r["kind"] for r in rows]
    assert kinds.count("switch") == 3 and kinds.count("fall") == 2 and kinds.count("getup") == 1, kinds
    fall = next(r for r in rows if r["kind"] == "fall")
    assert fall["last_switch"] == "walk+->turn_left" and fall["action"] == "turn_left" and fall["since_switch_s"] < 1
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "tools", "walk_stats.py"), "--file", path, "--falls"],
                         capture_output=True, text=True, check=True).stdout
    assert "firmware" in out and "walk+->turn_left x1" in out and "side_left" in out, out


def test_slow_fall_while_walking_is_a_fall():
    """A robot that leans over slowly while it is being walked (0.9 s from upright to on its back) fell: nobody lays
    a walking robot down. The same slow change with nothing commanded is a lay-down."""
    samples = [(0.0, 92.0, 5.0), (0.25, 110.0, 30.0), (0.5, 118.0, 40.0), (0.75, 125.0, 60.0), (1.0, 140.0, 90.0),
               (1.25, 170.0, 120.0)] + [(1.25 + t / 10, 178.0, 2.0) for t in range(1, 15)]
    walking = recovery.FallDetector()
    got = [e for e in (walking.feed(t, x, g, active=t < 1.0) for t, x, g in samples) if e]
    assert [(e["posture"], e["fell"]) for e in got] == [("on_back", True)] and 1.0 < got[0]["down_t"] < 1.3
    idle = recovery.FallDetector()
    slow = [(t * 2, x, g / 3) for t, x, g in samples]  # twice as slow, nothing commanded
    assert [(e["posture"], e["fell"]) for e in (idle.feed(t, x, g) for t, x, g in slow) if e] == [("on_back", False)]


def test_robot_stream_units_and_swaystep_body():
    """SwayStep on the robot: angles go to /hal_angles_set in raw units (4096 per turn, 65535 = leave alone), and the
    simulator's gait code runs against RobotBody, its lean and tilt from the robot's IMU."""
    import threading
    import time
    from yanshee_sim import robot_stream, sim_extras
    from yanshee_sim.servos import NAMES
    u = robot_stream.to_units({"NeckLR": 90, "RightKneeFlex": 75.9, "LeftHipLR": 500})
    assert u[NAMES.index("NeckLR")] == 1024 and u[NAMES.index("RightKneeFlex")] == 864
    assert u[NAMES.index("LeftHipLR")] == round(180 * robot_stream.UNITS_PER_DEG)  # clamped to the limit
    assert u.count(robot_stream.KEEP) == len(NAMES) - 3

    class FakeStream:
        def __init__(self):
            self.sent = []
            self.imu = {"e": [92.0, 0.0, 10.0], "g": [0.0, 0.0, 0.0], "a": [0.0, 1.0, 0.0], "age": 0.0}

        def send(self, angles, ms=20):
            self.sent.append((time.monotonic(), dict(angles), ms))
    fs = FakeStream()
    from yanshee_sim.servos import RESET_POSE
    body = robot_stream.RobotBody(fs, RESET_POSE)
    assert body.torso_lean() == (0.0, 0.0, 0.0, -0.0) and body.torso_tilt_deg() < 0.01
    fs.imu["e"] = [112.0, 0.0, 10.0]
    assert abs(body.torso_tilt_deg() - 20) < 0.01
    fs.imu["e"] = [92.0, 0.0, 10.0]
    stop = threading.Event()
    gait = sim_extras.SimGait(body, max_tilt_deg=35)
    t = threading.Thread(target=gait.run, args=({"speed_v": 2, "speed_h": 0, "steps": 0, "period": 1}, stop))
    t.start()
    time.sleep(2.5)
    stop.set()
    t.join(10)
    walk = [s for s in fs.sent if s[2] == round(sim_extras.SimGait.DT * 1000)]
    assert all(s[2] == 20 for s in fs.sent)  # long moves go out as 20 ms frames, like the firmware gait's
    span = walk[-1][0] - walk[0][0]
    assert len(walk) / span > 35, (len(walk), span)  # about 50 commands a second
    assert {"RightHipFB", "LeftKneeFlex", "RightAnkleFB"} <= set(walk[len(walk) // 2][1])
    knees = [s[1]["RightKneeFlex"] for s in walk]
    assert max(knees) - min(knees) > 5  # it steps


def _puts(srv, path):
    return [e for e in list(srv.state.log) if e[0] == "PUT" and e[1] == path]


def test_settle_gate_after_a_stop():
    """A walk restarted right after a stop waits until the robot has settled (SETTLE_MIN_S, IMU steady), as the
    robot's log of 29 Oct 2026 showed restarts within 1.5 s of a stop swing it back 11-38 deg."""
    import time
    from test_controller import Connected, wait_until
    with Connected("robot") as (c, srv):
        time.sleep(1.0)  # some IMU samples for the standing lean
        assert c.gait(2, 0, steps=0, lease_s=5.0)["ok"]
        assert wait_until(lambda: any(e[3].get("speed_v") == 2 for e in _puts(srv, "motions/gait")), 3)
        assert c.gait_stop()["ok"]
        t_stop = time.monotonic()
        n = len(_puts(srv, "motions/gait"))
        assert c.gait(2, 0, steps=0, lease_s=5.0)["ok"]
        time.sleep(0.6)
        assert not any(e[3].get("speed_v") == 2 for e in _puts(srv, "motions/gait")[n:]), "restarted before settling"
        assert wait_until(lambda: any(e[3].get("speed_v") == 2 for e in _puts(srv, "motions/gait")[n:]), 4)
        waited = time.monotonic() - t_stop
        assert c.SETTLE_MIN_S - 0.1 <= waited <= c.SETTLE_MAX_S + 1.0, waited
        c.gait_stop()


def test_walk_to_turn_switch_stops_then_turns_once_settled():
    import time
    from test_controller import Connected, wait_until
    with Connected("robot") as (c, srv):
        time.sleep(1.0)
        assert c.gait(2, 0, steps=0, lease_s=5.0)["ok"]
        assert wait_until(lambda: any(e[3].get("speed_v") == 2 for e in _puts(srv, "motions/gait")), 3)
        n_gait, n_mot = len(_puts(srv, "motions/gait")), len(_puts(srv, "motions"))
        t_switch = time.monotonic()
        assert c.gait(0, 0, steps=0, lease_s=5.0, turn=2)["updated"]
        stops = _puts(srv, "motions/gait")[n_gait:]
        assert stops and all(not stops[0][3].get(k) for k in ("speed_v", "speed_h", "period")), stops  # the app's stop now
        turned = lambda: [e for e in _puts(srv, "motions")[n_mot:] if (e[3].get("motion") or {}).get("name") == "TurnLeft"]  # noqa: E731
        time.sleep(0.6)
        assert not turned(), "turned before the robot settled"
        assert wait_until(lambda: bool(turned()), 4)
        assert time.monotonic() - t_switch >= c.SETTLE_MIN_S - 0.1
        c.gait_stop()


def test_auto_get_up_leaves_a_firmware_get_up_alone():
    """With fall management on, hal stands the robot up by itself; our auto get-up waits for it instead of being
    refused (29 Oct 2026) and plays nothing when the robot ends upright."""
    import time
    from test_controller import Connected, wait_until
    with Connected("robot") as (c, srv):
        c.set_auto_get_up(True)
        time.sleep(1.0)
        lying = {"code": 0, "msg": "success", "data": {"gyro": [{"id": 1, "euler-x": 179.0, "euler-y": 0.0, "euler-z": 0.0,
                 "gyro-x": 0.0, "gyro-y": 0.0, "gyro-z": 0.0, "accel-x": 0.0, "accel-y": 0.0, "accel-z": -1.0}]}}
        srv.state.overrides[("GET", "sensors/gyro")] = lying
        srv.state.overrides[("GET", "motions")] = {"code": 0, "msg": "success",
                                                   "data": {"name": "GetupRear", "status": "run", "timestamp": 0}}
        assert wait_until(lambda: c._recovering, 6), "the fall was not noticed"
        time.sleep(2.0)
        assert not [e for e in _puts(srv, "motions") if (e[3].get("motion") or e[3]).get("name") == "GetupRear"]
        srv.state.overrides.pop(("GET", "sensors/gyro"))  # the firmware's get-up finished: standing again
        srv.state.overrides.pop(("GET", "motions"))
        assert wait_until(lambda: not c._recovering, 8)
        assert any("own fall manager stood it up" in e["text"] for e in c.events_since(0))
        assert not [e for e in _puts(srv, "motions") if (e[3].get("motion") or e[3]).get("name") == "GetupRear"]
        c.set_auto_get_up(False)


def test_fall_reports_when_it_began():
    """up_t: the last time the robot was upright, i.e. when the fall began (what it was doing then caused it)."""
    det = recovery.FallDetector()
    ev = [det.feed(t, x, g) for t, x, g in [(0.0, 92, 1), (0.5, 92, 1), (0.75, 60, 150), (1.0, 2, 80)] +
          [(1.0 + i / 10, 1.0, 2.0) for i in range(1, 15)]]
    ev = [e for e in ev if e]
    assert len(ev) == 1 and ev[0]["up_t"] == 0.5 and ev[0]["down_t"] == 1.0


def test_recenter_steps_mirror_and_end_in_the_reset_pose():
    """Stepping a splayed leg in: the weight goes onto the other leg, the free leg lifts and swings in, everything
    ends in the reset pose; the left leg's moves mirror the right's (180 - x, ankle rolls keep the sign of the lean)."""
    from yanshee_sim.servos import RESET_POSE
    pose = dict(RESET_POSE, RightHipLR=50, LeftHipLR=97)
    assert recovery.splayed_legs(pose) == ["Right", "Left"] and recovery.splayed_legs(RESET_POSE) == []
    right, left = recovery.recenter_steps(pose, "Right"), recovery.recenter_steps(pose, "Left")
    assert right[-1][0] == RESET_POSE and right[2][0] == {"RightHipLR": RESET_POSE["RightHipLR"]}
    assert right[0][0]["RightAnkleUD"] > 90 and left[0][0]["RightAnkleUD"] < 90  # lean left / right
    for n, d in right[1][0].items():  # the lift, mirrored
        m = n.replace("Right", "Left")
        assert abs((180 - d) - left[1][0][m]) < 1e-9, (n, d, left[1][0][m])
    assert all(ms >= 600 for _, ms, _ in right)  # the slow version
