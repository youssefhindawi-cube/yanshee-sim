"""AppWalk in the simulator: the robot's recorded firmware walk (model/appwalk.json) played back.
Run: python3 -m pytest tests/test_appwalk.py -q"""
import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from yanshee_sim import appwalk  # noqa: E402

KEEP = 65535


def frame(x):
    return [x] * 16 + [KEEP]


# a made-up walk: 4 start frames, a 4-frame cycle, stops recorded at cycle points 1 and 3, two stand frames
TABLE = {"walks": {"forward": {"start": [frame(1000 + i) for i in range(4)], "cycle": [frame(2000 + i) for i in range(4)],
                               "winddowns": {"1": [frame(3001)], "3": [frame(3003), frame(3004)]}, "min_lag": 2,
                               "stand": [frame(1024), frame(1024)], "start_delay_s": 0.0}},
         "turns": {"TurnLeft": [{"angles": {"RightHipLR": 90}, "runtime": 100}]}}


def test_table_loops_the_cycle_and_stops_where_the_firmware_does():
    t = appwalk.AppWalkTable(TABLE["walks"]["forward"])
    assert [t.frame(k)[0] for k in range(10)] == [1000, 1001, 1002, 1003, 2000, 2001, 2002, 2003, 2000, 2001]
    # a stop at frame 6 (cycle point 2): on for min_lag 2 frames to frame 8 (point 0), on to point 1, wind down there
    assert t.stop_at(6) == (9, [frame(3001)])
    assert [f[0] for f in t.frames_for(6)] == [1000, 1001, 1002, 1003, 2000, 2001, 2002, 2003, 2000, 3001]
    assert t.stop_at(0)[0] == 5  # a stop before the walk reached its cycle winds down at the first point in it


def test_unrecorded_directions_are_refused():
    u = appwalk.AppWalkSim.unsupported
    assert u(3, 0, table=TABLE) is None and u(0, 0, 2, table=TABLE) is None
    assert "backward" in u(-3, 0, table=TABLE) and "forward and left" in u(3, 2, table=TABLE)
    assert "turn motions" in u(0, 0, -2, table=TABLE)  # no TurnRight copy
    assert "no recording" in u(3, 0, table={})


def test_the_recorded_walk_walks_the_simulator_forward():
    from yanshee_sim.robot import SimRobot
    if not appwalk.load().get("walks", {}).get("forward"):
        return  # no recording in this checkout
    sim = SimRobot(viewer=False, realtime=False)
    sim.imu_noise = False
    sim.respawn()
    sim.wait(0.5)
    y0 = float(sim.data.xpos[sim.model.body("base_link").id][1])
    stop, steps = threading.Event(), []
    status = appwalk.AppWalkSim(sim).run({"speed_v": 3, "speed_h": 0, "steps": 8, "wave": True}, stop, steps.append)
    assert status == "done" and steps[-1] == 8 and sim.torso_tilt_deg() < 15
    assert float(sim.data.xpos[sim.model.body("base_link").id][1]) - y0 > 0.05
    sim.close()


def test_balance_keeps_the_walk_and_clears_its_offsets():
    """AppWalkSim with balance: the recorded walk still goes forward, the corrections are faded out after it."""
    import threading
    from yanshee_sim.robot import SimRobot
    from yanshee_sim.servos import RESET_POSE
    sim = SimRobot(viewer=False, realtime=False)
    sim.respawn()
    sim.set_servos_angles(RESET_POSE, 400)
    sim.wait(1.0)
    y0 = sim.data.body("base_link").xpos[1]
    walk = appwalk.AppWalkSim(sim, balance=True)
    assert walk.run(dict(speed_v=2, speed_h=0, turn=0, steps=10, period=1, wave=True), threading.Event()) == "done"
    assert sim.data.body("base_link").xpos[1] - y0 > 0.1 and sim.torso_tilt_deg() < 15 and sim._offsets == {}


def test_servo_delay_and_imu_latency_match_the_robot():
    """The simulator's servos hear a command SERVO_DELAY_S late (the robot: first motion 76 ms, settled 124 ms for a
    15 deg move) and its IMU samples are IMU_LATENCY_S old; both switch off with 0."""
    import math
    from yanshee_sim.robot import SimRobot
    from yanshee_sim.servos import RESET_POSE
    for delay, lo, hi in ((0.06, 0.05, 0.10), (0.0, 0.0, 0.03)):
        sim = SimRobot(viewer=False, realtime=False)
        sim.servo_delay = delay
        sim.respawn()
        sim.set_servos_angles(dict(RESET_POSE), 400)
        sim.wait(1.0)
        base = math.degrees(sim.data.qpos[sim._qadr["LeftShoulderFlex"]])
        t0 = sim.data.time
        sim._send({"LeftShoulderFlex": RESET_POSE["LeftShoulderFlex"] + 15}, 20)
        first = None
        while sim.data.time - t0 < 0.4:
            sim.wait(0.005)
            if first is None and abs(math.degrees(sim.data.qpos[sim._qadr["LeftShoulderFlex"]]) - base) >= 3:
                first = sim.data.time - t0
        assert first is not None and lo <= first <= hi, (delay, first)
        assert abs(math.degrees(sim.data.qpos[sim._qadr["LeftShoulderFlex"]]) - base - 15) < 2.5
        sim.close()
    sim = SimRobot(viewer=False, realtime=False)
    sim.imu_noise = False
    sim.respawn()
    sim.wait(0.3)
    head = sim.model.body("NeckLR").id
    with sim._lock:
        sim.data.xfrc_applied[head] = [0, 6.0, 0, 0, 0, 0]  # a push: the torso starts leaning
    sim.wait(0.25)
    lean_now = sim.torso_lean()[0]
    served = sim.get_sensors_gyro()["data"]["gyro"][0]["euler-x"] - 90.0
    sim.imu_latency = 0.0
    sim._imu_hold = None
    served_now = sim.get_sensors_gyro()["data"]["gyro"][0]["euler-x"] - 90.0
    assert abs(served_now - lean_now) < 0.5 and abs(served - lean_now) > abs(served_now - lean_now), (lean_now, served, served_now)
    sim.close()
