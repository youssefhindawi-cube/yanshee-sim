"""Mirror logic with a fake robot. Run: python3 tests/test_mirror.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import RESET_POSE, SimRobot  # noqa: E402
from yanshee_sim.mirror import DryRunRobot, RobotToSim, SimToRobot  # noqa: E402


class RecordingRobot(DryRunRobot):
    def __init__(self, pose=None):
        self.log, self.pose = [], dict(pose or RESET_POSE)

    def set_servos_angles(self, angles, runtime=200):
        self.log.append((dict(angles), runtime))
        return {"code": 0}

    def get_servos_angles(self, names=None):
        return {"code": 0, "data": dict(self.pose)}


def _drag(sim, **angles):
    for n, deg in angles.items():
        sim.data.ctrl[sim._act[n]] = math.radians(deg)


def test_sim_to_robot_sends_changes_with_speed_cap():
    sim = SimRobot(viewer=False, realtime=False, fixed_base=True, manual=True)
    robot = RecordingRobot()
    link = SimToRobot(sim, robot, max_speed_deg_s=90)
    link.start()
    _drag(sim, RightShoulderFlex=30)  # 110 deg change
    assert link.step() == ("sent", ({"RightShoulderFlex": 30}, 1222))
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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
