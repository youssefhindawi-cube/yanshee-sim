"""The robot link's node (robot_stream.REMOTE) on the laptop against a fake ROS (tests/fake_ros) and the fake robot:
REST through the link, the robot's watchdog, and the controller bringing a lost link back.
Run: python3 -m pytest tests/test_link.py -q"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import fake_robot  # noqa: E402
from yanshee_sim import robot_stream  # noqa: E402
from yanshee_sim.controller import Controller  # noqa: E402
from yanshee_sim.rest import YansheeRest  # noqa: E402

FAKE_ROS = os.path.join(HERE, "fake_ros")


def _node_files():
    d = tempfile.mkdtemp()
    script, feed = os.path.join(d, "node.py"), os.path.join(d, "feed.txt")
    with open(script, "w") as f:
        f.write(robot_stream.NODE_SOURCE)
    open(feed, "w").close()
    env = dict(os.environ, PYTHONPATH=FAKE_ROS, FAKE_ROS_FEED=feed)
    return d, script, feed, env


def _feed(feed, line, n=1, every=0.02):
    for _ in range(n):
        with open(feed, "a") as f:
            f.write(line + "\n")
        time.sleep(every)


def _gait_calls(srv):
    return [(m, b) for m, p, q, b in list(srv.state.log) if p == "motions/gait" and m in ("PUT", "DELETE")]


def test_rest_through_the_link_and_falling_back():
    srv = fake_robot.start(port=0)
    d, script, feed, env = _node_files()
    os.environ.update(PYTHONPATH=FAKE_ROS, FAKE_ROS_FEED=feed)  # the launcher's process inherits them
    try:
        st = robot_stream.RobotStream("127.0.0.1", rest_port=srv.server_address[1], timeout=10,
                                      launcher=lambda a: [sys.executable, "-u", script, a[0], a[1], d] + a[3:])
        try:
            status, body, ctype, ms = st.rest("GET", "devices/battery")
            assert status == 200 and json.loads(body)["data"]["percent"] == 90 and ms is not None
            rest = YansheeRest(f"127.0.0.1:{srv.server_address[1]}")
            rest.relay = st.rest
            n = st._rid
            assert rest.gait(2, 0)["code"] == 0 and st._rid == n + 1  # through the link
            rest.relay = lambda *a: (_ for _ in ()).throw(RuntimeError("link down"))
            assert rest.gait(0, 0, 0, 0)["code"] == 0  # direct
            assert [b["speed_v"] for m, b in _gait_calls(srv)] == [2, 0]
            assert st.silence < 1.0
        finally:
            st.close()
    finally:
        for k in ("PYTHONPATH", "FAKE_ROS_FEED"):
            os.environ.pop(k, None)
        fake_robot.stop(srv)


@pytest.mark.parametrize("python", [sys.executable] + (["python2"] if shutil.which("python2") else []))
def test_watchdog_stops_a_walk_and_a_turn_when_the_laptop_goes_quiet(python):
    """In Python 2 as well where there is one: the robot runs the node in Python 2.7."""
    srv = fake_robot.start(port=0)
    d, script, feed, env = _node_files()
    p = subprocess.Popen([python, "-u", script, "1", "wd", d, str(srv.server_address[1]), "0.5"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
                         env=env)
    got = []
    threading.Thread(target=lambda: [got.append(json.loads(x)) for x in p.stdout if x.startswith("{")], daemon=True).start()
    try:
        t0 = time.time()
        while not any(g.get("ready") for g in got) and time.time() - t0 < 10:
            time.sleep(0.05)
        for _ in range(40):  # 1 s of walking with the laptop's heartbeat: nothing happens
            p.stdin.write("H\n")
            p.stdin.flush()
            _feed(feed, "gait", every=0.025)
        assert not _gait_calls(srv) and not [g for g in got if "wd" in g]
        _feed(feed, "gait", n=200)  # the laptop goes quiet while it walks (4 s): the app's stop, then exit
        calls = _gait_calls(srv)
        assert [m for m, b in calls] == ["PUT", "DELETE"], calls
        assert calls[0][1]["speed_v"] == 0 and calls[0][1]["period"] == 0
        assert [g["wd"] for g in got if "wd" in g] == ["stop", "exit"]
        p.stdin.write("H\n")  # the laptop is back: rearmed; a looping turn with the laptop quiet again
        p.stdin.flush()
        time.sleep(0.2)
        _feed(feed, "turn start TurnLeft")
        time.sleep(1.0)
        assert [m for m, b in _gait_calls(srv)] == ["PUT", "DELETE", "PUT"]
    finally:
        p.kill()
        fake_robot.stop(srv)


class _Stream:
    def __init__(self):
        self.alive, self.silence, self.closed = True, 0.0, False

    def rest(self, *a):
        return None

    def close(self):
        self.closed = True


def test_controller_brings_a_lost_link_back():
    """Controller._link_keeper: a link that fails to open is retried; one that goes silent is dropped (relay off)
    and opened again."""
    robot = types.SimpleNamespace(rest=types.SimpleNamespace(relay=None))
    events, streams, fails = [], [], [1]
    c = types.SimpleNamespace(robot=robot, _link=None, _link_lost=False, _lock=threading.Lock(), link_watchdog_s=1.5, balance=False,
                              LINK_SILENT_S=0.3, LINK_RETRY_S=(0.05,), LINK_CHECK_S=0.05)
    c._event = lambda level, msg: events.append(msg)
    c._s2r_write = lambda kind, **d: None
    c._shadow_loop = lambda robot, stop: None
    c._drop_link = types.MethodType(Controller._drop_link, c)

    def open_link(r):
        if fails[0]:
            fails[0] -= 1
            raise RuntimeError("ssh: no route to host")
        s = _Stream()
        streams.append(s)
        c._link, r.rest.relay = (r, s), s.rest
    c._robot_link = open_link
    stop = threading.Event()
    threading.Thread(target=Controller._link_keeper, args=(c, robot, stop), daemon=True).start()
    try:
        t0 = time.time()
        while len(streams) < 1 and time.time() - t0 < 3:
            time.sleep(0.02)
        assert len(streams) == 1 and "not available" in events[0] and not c._link_lost
        streams[0].silence = 10.0  # the Wi-Fi drops
        while len(streams) < 2 and time.time() - t0 < 5:
            time.sleep(0.02)
        time.sleep(0.1)
        assert len(streams) == 2 and streams[0].closed and c._link[1] is streams[1] and robot.rest.relay == streams[1].rest
        assert any("link lost" in e for e in events) and events[-1] == "robot link is back" and not c._link_lost
    finally:
        stop.set()


@pytest.mark.parametrize("python", [sys.executable] + (["python2"] if shutil.which("python2") else []))
def test_balance_on_the_firmware_walk(python):
    """B 1: the gait's frames get ankle-only frames with the balance offsets (leaning back -> RightAnkleFB up,
    LeftAnkleFB down); after the walk they fade back to the gait's angles."""
    from yanshee_sim.servos import NAMES
    d, script, feed, env = _node_files()
    p = subprocess.Popen([python, "-u", script, "1", "bal", d, "9", "0"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.DEVNULL, text=True, bufsize=1, env=env)
    got = []
    threading.Thread(target=lambda: [got.append(json.loads(x)) for x in p.stdout if x.startswith("{")], daemon=True).start()
    try:
        t0 = time.time()
        while not any(g.get("ready") for g in got) and time.time() - t0 < 10:
            time.sleep(0.05)
        p.stdin.write("B 1 {}\n")
        p.stdin.flush()
        _feed(feed, "imu 92 0 0 0 0", n=5, every=0.05)  # standing upright: the reference
        for _ in range(25):  # walking, leaning back 8 deg
            _feed(feed, "gait 1024", every=0.0)
            _feed(feed, "imu 100 0 0 0 0", every=0.04)
        time.sleep(1.0)  # the walk is over: fade
        bal = [g["c"] for g in got if g.get("src") == "balance"]
        rf, lf, ru = NAMES.index("RightAnkleFB"), NAMES.index("LeftAnkleFB"), NAMES.index("RightKneeFlex")
        assert bal and max(c[rf] for c in bal) > 1024 + 20 and min(c[lf] for c in bal) < 1024 - 20, bal[:3]
        assert all(c[ru] == 65535 for c in bal)  # ankles only
        assert bal[-1][rf] == 1024 and bal[-1][lf] == 1024  # faded back to the gait's angles
    finally:
        p.kill()


def _start_node(python, d, script, env, watchdog="0"):
    p = subprocess.Popen([python, "-u", script, "1", "turn", d, "9", watchdog], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1, env=env)
    got = []
    threading.Thread(target=lambda: [got.append(json.loads(x)) for x in p.stdout if x.startswith("{")], daemon=True).start()
    t0 = time.time()
    while not any(g.get("ready") for g in got) and time.time() - t0 < 10:
        time.sleep(0.05)
    return p, got


def _say(p, line):
    p.stdin.write(line + "\n")
    p.stdin.flush()


TURNS = {"TurnLeft": [[[1024] * 17, 100, 20], [[1100] * 17, 100, 20]], "ResetSwitch": [[[1000] * 17, 200, 20]]}


@pytest.mark.parametrize("python", [sys.executable] + (["python2"] if shutil.which("python2") else []))
def test_node_plays_a_balanced_turn(python):
    """T TurnLeft: the node streams the turn's keyframes (20 ms frames) with the balance's ankle offsets, loops until
    T 0, then plays ResetSwitch and fades the offsets back to its pose."""
    from yanshee_sim.servos import NAMES
    d, script, feed, env = _node_files()
    p, got = _start_node(python, d, script, env)
    try:
        _say(p, "TF " + json.dumps(TURNS))
        _say(p, "B 1 {}")
        _feed(feed, "imu 92 0 0 0 0", n=5, every=0.05)
        _say(p, "T TurnLeft")
        _feed(feed, "imu 100 0 0 0 0", n=20, every=0.04)  # 0.8 s turning, leaning back 8 deg
        _say(p, "T 0")
        time.sleep(1.2)
        turn = [g["c"] for g in got if g.get("src") == "turn"]
        rf, knee = NAMES.index("RightAnkleFB"), NAMES.index("RightKneeFlex")
        assert len(turn) > 30 and {g.get("turn") for g in got if "turn" in g} >= {"start", "finish"}
        assert any(c[knee] == 1100 for c in turn) and any(c[knee] == 1000 for c in turn)  # looped, then ResetSwitch
        assert max(c[rf] - c[knee] for c in turn) > 20  # the ankle got the balance offset on top
        fin = next(g for g in got if g.get("turn") == "finish")
        assert fin["why"] == "released"
        bal = [g["c"] for g in got if g.get("src") == "balance"]
        assert bal and bal[-1][rf] == 1000  # faded back to ResetSwitch's pose
    finally:
        p.kill()


def test_node_turn_stops_when_the_laptop_goes_quiet_or_on_stop():
    d, script, feed, env = _node_files()
    p, got = _start_node(sys.executable, d, script, env, watchdog="0.5")
    try:
        _say(p, "TF " + json.dumps(TURNS))
        _say(p, "T TurnLeft")  # then nothing from the laptop
        time.sleep(2.0)
        fin = [g for g in got if g.get("turn") == "finish"]
        assert fin and fin[0]["why"] == "laptop silent"
        _say(p, "T TurnLeft")
        for _ in range(4):
            _say(p, "H")
            time.sleep(0.1)
        _say(p, "T !")
        time.sleep(0.3)
        fin = [g for g in got if g.get("turn") == "finish"]
        assert len(fin) == 2 and fin[1]["why"] == "aborted"
    finally:
        p.kill()


def test_controller_sends_turns_to_the_node_with_the_balance_on():
    c = Controller.__new__(Controller)  # only what _robot_walk_send and _robot_turn_release use
    calls = []

    class Stream:
        alive, turns_loaded, turning = True, True, False

        def turn(self, name):
            calls.append(("node", name))

    class Rest:
        def motion_control(self, *a):
            calls.append(("hal", a[1]))
            return {"code": 0}

        def gait(self, *a):
            calls.append(("gait", a))
            return {"code": 0}
    robot = types.SimpleNamespace(rest=Rest())
    c._link, c._balance_robot, c._s2r_write = (robot, Stream()), "on", lambda *a, **k: None
    p = dict(speed_v=0, speed_h=0, turn=1, period=1, wave=False)
    c._robot_walk_send(robot, {}, p, True)
    c._robot_turn_release(robot, p)
    assert calls == [("node", "TurnLeft"), ("node", None)]
    c._balance_robot, calls[:] = "off: run tools/robot_checks.py balance", []
    p = dict(speed_v=0, speed_h=0, turn=-1, period=1, wave=False)
    c._robot_walk_send(robot, {}, p, True)
    c._robot_turn_release(robot, p)
    assert calls == [("hal", "TurnRight"), ("gait", (0, 0, 0, 0, False))]


def test_launch_command_ends_the_old_node_not_itself():
    """The command that starts the node on the robot ends a node left running (by its pid file) and starts the new
    one; matching by name (pkill -f) killed the command itself on the robot on 2 Nov 2026."""
    d = tempfile.mkdtemp()
    old = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", "yanshee_sway_stream_old"])
    pid_path, script = os.path.join(d, "node.pid"), os.path.join(d, "yanshee_sway_stream.py")
    with open(pid_path, "w") as f:
        f.write(str(old.pid))
    with open(script, "w") as f:
        f.write("import sys; print('started ' + ' '.join(sys.argv[1:]))\n")
    try:
        out = subprocess.run(["bash", "-c", robot_stream.launch_command(["5.0", "s"], path=script, pid_path=pid_path,
                                                                        python=sys.executable)],
                             capture_output=True, text=True, timeout=10).stdout
        assert out.strip() == "started 5.0 s"
        assert old.wait(5) is not None  # the old node was ended
    finally:
        old.kill()
