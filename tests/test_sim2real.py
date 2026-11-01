"""Sim-to-real tools: the shadow simulator and the session report (tools/sim2real_report.py).
Run: python3 -m pytest tests/test_sim2real.py -q"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from yanshee_sim import robot_stream  # noqa: E402
from yanshee_sim.servos import NAMES, RESET_POSE  # noqa: E402
from yanshee_sim.shadow import Shadow  # noqa: E402


def test_frames_round_trip():
    deg = {"RightKneeFlex": 75.9, "LeftShoulderRoll": 48.7}
    back = robot_stream.to_degrees(robot_stream.to_units(deg))
    assert set(back) == set(deg) and all(abs(back[n] - deg[n]) < 0.1 for n in deg)


def test_shadow_follows_frames_and_reports():
    from yanshee_sim.robot import SimRobot
    sh = Shadow(sim_factory=lambda: SimRobot(viewer=False, realtime=False))
    assert sh.sample() is None
    pose = dict(RESET_POSE, RightShoulderRoll=80)
    sh.reset(pose)
    for i in range(25):  # half a second of 20 ms frames raising the right arm sideways
        sh.feed({"RightShoulderRoll": 80 - i}, 20)
        sh.sim.wait(0.02)
    sh.sim.wait(0.3)
    smp = sh.sample()
    assert smp["active"] and smp["frames"] == 25 and not smp["fallen"]
    assert abs(smp["joints"]["RightShoulderRoll"] - 56) < 4, smp["joints"]["RightShoulderRoll"]
    assert abs(smp["euler"][0] - 90) < 5 and smp["tilt"] < 5  # standing, as the robot reports it (euler-x ~90)
    sh.IDLE_S = 0.05  # with no more frames (the robot gone) it stops reporting
    time.sleep(0.1)
    assert not sh.sample()["active"]
    sh.close()


def test_report_merges_robot_and_laptop_logs():
    """The robot's log (robot clock, 3 s ahead) and the laptop's line up through the clock pings."""
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "robot_logs"))
    OFF, LINK = 3.0, 0.015  # robot clock - laptop clock; frames reach the robot 15 ms after they are sent
    t0, lap, rob = 1000.0, [], []
    lap.append({"t": t0, "kind": "link", "robot_log": "/home/pi/yanshee_logs/robot_test.jsonl", "session": "test"})
    for i in range(6):
        lap.append({"t": t0 + i, "kind": "clock", "seq": i, "rtt": 0.03 + 0.01 * i, "offset": OFF + 0.001 * i})
    rob.append({"t": t0 + OFF, "k": "info", "servo_offsets": list(range(17)), "versions": {"0": "HW"}})
    lap.append({"t": t0 + 0.1, "kind": "shadow_reset", "pose": RESET_POSE, "method": "SwayStep"})
    for i in range(100):  # 2 s of frames: the knee swings; the encoders follow 60 ms later
        sent = t0 + 0.2 + i * 0.02
        knee = 76 + 10 * (i % 20) / 20
        units = robot_stream.to_units({"RightKneeFlex": knee})
        rob.append({"t": sent + LINK + OFF, "k": "frame", "src": "ours", "seq": i + 1, "lt": sent, "ms": 20, "c": units})
        if i % 5 == 0:
            rob.append({"t": sent + LINK + 0.06 + OFF, "k": "enc", "s": [round(knee) if n == "RightKneeFlex" else 90 for n in NAMES]})
            rob.append({"t": sent + 0.06 + OFF, "k": "imu", "e": [92 + i / 10, 0.5, 0], "g": [0, 0, 0], "a": [0, 1, 0]})
            lap.append({"t": sent + 0.07, "kind": "shadow", "active": True, "frames": i, "tilt": i / 20, "fallen": False,
                        "euler": [90, 0, 0], "gyro": [0, 0, 0], "accel": [0, 1, 0],
                        "joints": {"RightKneeFlex": round(knee) + 2}})
    rob.append({"t": t0 + 2.5 + OFF, "k": "hal", "battery": {"voltage": 7600, "charging": 0}, "board": {"iBoardTemperature": 44},
                "fall": {"data": 0}})
    rob.append({"t": t0 + 2.6 + OFF, "k": "rosout", "lvl": "WARN", "node": "/hal", "msg": "Read servo angle status timeout!"})
    for name, rows in (("sim2real_test.jsonl", lap), (os.path.join("robot_logs", "robot_test.jsonl"), rob)):
        with open(os.path.join(d, name), "w") as f:
            f.writelines(json.dumps(r) + "\n" for r in rows)
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "tools", "sim2real_report.py"),
                          os.path.join(d, "sim2real_test.jsonl"), "--joints"], capture_output=True, text=True, check=True).stdout
    walk = next(line for line in out.splitlines() if line.strip().startswith("1 "))
    assert "SwayStep" in walk and "ours 100" in walk and "60ms" in walk, out
    assert "robot - laptop = +300" in out and ("late by median 14 ms" in out or "late by median 15 ms" in out), out
    assert "Read servo angle status timeout" in out and "7600-7600 mV" in out and "RightKneeFlex 8," in out, out


def test_shadow_plays_a_motion_and_stops_a_looping_one():
    """The shadow plays the .hts motions hal plays (turns loop until the next motion), on time."""
    from yanshee_sim.robot import SimRobot
    sh = Shadow(sim_factory=lambda: SimRobot(viewer=False, realtime=True))
    sh.reset(RESET_POSE)
    frames = [{"angles": {"RightShoulderRoll": 60}, "runtime": 200, "hold": 0},
              {"angles": {"RightShoulderRoll": 90}, "runtime": 200, "hold": 0}]
    sh.play_motion("TurnLeft", frames, due=time.time() + 0.1)  # looping: still going after one pass
    time.sleep(1.2)
    n = sh.frames
    assert n >= 4, n
    sh.stop_motion()
    time.sleep(0.5)
    assert sh.frames <= n + 1  # stopped
    sh.play_motion("ResetSwitch", frames)  # not looping: two frames
    time.sleep(0.8)
    assert sh.frames - n <= 3
    sh.close()


def test_replay_tool_on_a_synthetic_session():
    """tools/sim2real_replay.py: a robot that stood still while frames held its pose replays as standing."""
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "robot_logs"))
    t0 = 1000.0
    lap = [{"t": t0, "kind": "link", "robot_log": "/home/pi/yanshee_logs/robot_syn.jsonl", "session": "syn"},
           {"t": t0, "kind": "clock", "seq": 1, "rtt": 0.01, "offset": 0.0},
           {"t": t0 + 1.0, "kind": "shadow_reset", "pose": RESET_POSE, "method": "AppWalk"}]
    units = robot_stream.to_units(RESET_POSE)
    rob = [{"t": t0 + 0.5, "k": "enc", "s": [round(RESET_POSE[n]) for n in NAMES]}]
    rob += [{"t": t0 + 0.4 + i * 0.05, "k": "imu", "e": [92.0, 0.0, 10.0], "g": [0, 0, 0], "a": [0, 1, 0]} for i in range(60)]
    rob += [{"t": t0 + 1.1 + i * 0.02, "k": "frame", "src": "/gait_node", "ms": 20, "c": units} for i in range(50)]
    for name, rows in (("sim2real_syn.jsonl", lap), (os.path.join("robot_logs", "robot_syn.jsonl"), rob)):
        with open(os.path.join(d, name), "w") as f:
            f.writelines(json.dumps(r) + "\n" for r in rows)
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "tools", "sim2real_replay.py"),
                          os.path.join(d, "sim2real_syn.jsonl"), "--jobs", "1"], capture_output=True, text=True,
                         timeout=300, env=dict(os.environ, MUJOCO_GL="egl")).stdout
    assert "falls agree 1/1" in out and "robot up" in out and "sim up" in out, out


def test_report_follows_the_robot_clock_drift():
    """The robot's clock drifts 60 ms in 20 min (as on 29 Oct 2026): frames that took 4 ms still show 4 ms."""
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
    import sim2real_report as rep
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "robot_logs"))
    t0 = 1000.0

    def off(t):
        return 0.06 * (t - t0) / 1200

    lap = [{"t": t0, "kind": "link", "robot_log": "/home/pi/yanshee_logs/robot_drift.jsonl", "session": "drift"}]
    lap += [{"t": t0 + 2 * i + 0.004, "kind": "clock", "seq": i, "rtt": 0.004 + 0.002 * (i % 3), "offset": off(t0 + 2 * i)}
            for i in range(600)]
    rob = [{"t": t0 + i + 0.004 + off(t0 + i), "k": "frame", "src": "ours", "seq": i, "lt": t0 + i, "ms": 20,
            "c": [65535] * len(NAMES)} for i in range(0, 1200, 10)]
    for name, rows in (("sim2real_drift.jsonl", lap), (os.path.join("robot_logs", "robot_drift.jsonl"), rob)):
        with open(os.path.join(d, name), "w") as f:
            f.writelines(json.dumps(r) + "\n" for r in rows)
    late = [(r["t"] - r["lt"]) * 1000 for r in rep.load(os.path.join(d, "sim2real_drift.jsonl"))[1] if r["kind"] == "frame"]
    assert len(late) == 120 and max(abs(x - 4) for x in late) < 2, (min(late), max(late))
