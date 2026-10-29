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
