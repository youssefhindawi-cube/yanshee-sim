"""Mirror logic with a fake robot. Run: python3 tests/test_mirror.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import RESET_POSE, SimRobot  # noqa: E402
from yanshee_sim.mirror import DryRunRobot, RobotToSim, SimToRobot  # noqa: E402


class Clock:
    """Fake monotonic clock: tests move time by hand."""

    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class RecordingRobot(DryRunRobot):
    def __init__(self, pose=None, clock=None, fail=0):
        self.log, self.pose, self.clock, self.fail = [], dict(pose or RESET_POSE), clock, fail

    def set_servos_angles(self, angles, runtime=200):
        if self.fail:
            self.fail -= 1
            raise ConnectionError("robot stopped answering")
        self.log.append((dict(angles), runtime))
        return {"code": 0}

    def get_servos_angles(self, names=None):
        return {"code": 0, "data": dict(self.pose)}

    def wait(self, seconds):
        self.clock.t += seconds


def _link(robot_pose=None, **kw):
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True, manual=True)
    clock = Clock()
    robot = RecordingRobot(robot_pose, clock, fail=kw.pop("fail", 0))
    link = SimToRobot(sim, robot, clock=clock, **kw)
    link.start()
    return sim, robot, link, clock


def _drag(sim, **angles):
    for n, deg in angles.items():
        sim.data.ctrl[sim._act[n]] = math.radians(deg)


def test_sim_to_robot_sends_changes_with_speed_cap():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True, manual=True)
    robot = RecordingRobot()
    link = SimToRobot(sim, robot, max_speed_deg_s=90)
    link.start()
    _drag(sim, RightShoulderFlex=30)  # 110 deg change
    assert link.step() == ("sent", ({"RightShoulderFlex": 30}, 1223))  # 110 deg at 90 deg/s, rounded up
    assert link.step() == ("idle", None)
    sim.close()


def test_sim_to_robot_blocks_self_collision():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True, manual=True)
    robot = RecordingRobot()
    link = SimToRobot(sim, robot)
    link.start()
    _drag(sim, RightHipLR=120, LeftHipLR=60)  # legs cross
    status, _ = link.step()
    assert status == "blocked" and robot.log == []
    sim.close()


def test_start_matches_robot_pose():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True, manual=True)
    robot = RecordingRobot({**RESET_POSE, "NeckLR": 120})
    SimToRobot(sim, robot).start()
    assert sim.get_servos_angles(["NeckLR"])["data"]["NeckLR"] == 120
    sim.close()


def test_robot_to_sim_follows_and_clamps():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True)
    robot = RecordingRobot({**RESET_POSE, "RightElbowFlex": 100, "LeftAnkleUD": 116})
    RobotToSim(sim, robot, period_s=0.25).step()
    sim.wait(1.0)
    got = sim.get_servos_angles(["RightElbowFlex", "LeftAnkleUD"])["data"]
    assert got == {"RightElbowFlex": 100, "LeftAnkleUD": 115}
    sim.close()


def test_speed_cap_uses_in_flight_position():
    sim, robot, link, clock = _link(max_speed_deg_s=90)
    _drag(sim, RightShoulderFlex=30)
    assert link.step() == ("sent", ({"RightShoulderFlex": 30}, 1223))
    clock.t += 0.25  # the servo is only about 22 deg into the 110 deg move
    est = link.estimated()["RightShoulderFlex"]
    assert 117 < est < 118
    _drag(sim, RightShoulderFlex=29)
    status, (angles, runtime) = link.step()
    assert status == "sent" and angles == {"RightShoulderFlex": 29}
    assert (est - 29) / (runtime / 1000) <= 90 + 1e-6, runtime  # not the old 250 ms
    sim.close()


def test_slow_cap_sends_partial_moves():
    sim, robot, link, clock = _link(max_speed_deg_s=20)
    _drag(sim, RightShoulderFlex=0)  # 140 deg needs 7 s at 20 deg/s; one command can only last 4 s
    sent = []
    for _ in range(4):
        status, info = link.step()
        if status == "sent":
            est = link._moves["RightShoulderFlex"][0]
            sent.append(info)
            assert abs(info[0]["RightShoulderFlex"] - est) / (info[1] / 1000) <= 20 + 1e-6, (info, est)
        clock.t += 4.0
    assert sent[0] == ({"RightShoulderFlex": 60}, 4000)
    assert sent[-1][0] == {"RightShoulderFlex": 0} and len(sent) == 2
    assert link.step() == ("idle", None)
    sim.close()


def test_start_pose_in_contact_does_not_block_other_servos():
    touching = {**RESET_POSE, "RightElbowFlex": 0, "RightShoulderRoll": 170, "RightShoulderFlex": 0}
    sim, robot, link, clock = _link(touching)
    assert link.start_contacts  # the arm rests on the torso
    _drag(sim, NeckLR=120)
    status, info = link.step()
    assert status == "sent" and info[0] == {"NeckLR": 120}
    sim.close()


def test_robot_errors_are_reported_and_retried():
    sim, robot, link, clock = _link(fail=1)
    _drag(sim, NeckLR=120)
    status, info = link.step()
    assert status == "error" and "stopped answering" in info
    assert link.sent["NeckLR"] == 90 and robot.log == []
    status, info = link.step()
    assert status == "sent" and info[0] == {"NeckLR": 120}
    sim.close()


def test_move_to_reset_is_speed_capped():
    sim, robot, link, clock = _link({**RESET_POSE, "RightShoulderFlex": 0}, max_speed_deg_s=30)
    assert link.move_to(RESET_POSE) == ("sent", RESET_POSE)
    prev = 0
    for angles, runtime in robot.log:
        assert abs(angles["RightShoulderFlex"] - prev) / (runtime / 1000) <= 30 + 1e-6
        prev = angles["RightShoulderFlex"]
    assert len(robot.log) == 2 and link.sent == RESET_POSE
    sim.close()


def test_move_to_refuses_colliding_path():
    # each pose is clear, but the straight line back to reset drives the right elbow into the hip
    pose = {**RESET_POSE, "LeftShoulderFlex": 66, "RightShoulderRoll": 113, "RightElbowFlex": 155, "RightHipLR": 36}
    sim, robot, link, clock = _link(pose)
    status, hits = link.move_to(RESET_POSE)
    assert status == "blocked" and hits and robot.log == []
    sim.close()


def test_robot_to_sim_keeps_strict_and_survives_errors():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True)
    robot = RecordingRobot({**RESET_POSE, "LeftAnkleUD": 116})
    link = RobotToSim(sim, robot)
    assert link.step()["LeftAnkleUD"] == 115 and sim.strict is True
    robot.get_servos_angles = lambda names=None: {"code": -1, "msg": "busy"}
    assert link.step() is None and "busy" in link.last_error
    sim.close()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
