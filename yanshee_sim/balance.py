"""IMU balance on top of an open-loop walk or turn (the firmware's AppWalk, its TurnLeft/TurnRight): ankle pitch and
roll offsets from the torso's lean and lean rate, added to whatever the walk commands. Python 2 and 3 with nothing
but math: the simulator uses it (appwalk.AppWalkSim) and the robot link's node carries this file's source.

The walk sways on purpose (weight from foot to foot, about 2.5 steps a second), so only lean beyond a deadband around
the standing lean and the lean rate are corrected; offsets are limited in size and speed.

Signs, measured in the simulator (1 Nov 2026) with the feet flat at the reset pose: RightAnkleFB +d with LeftAnkleFB
-d leans the torso forward by about 1.6 d; both AnkleUD +d lean it left by about 0.55 d. On the robot the ankle pitch
direction is still to verify (test plan B3) before this runs there.
"""
import math

UPRIGHT_EULER_X = 92.0  # the robot's euler-x standing (measured 29 Oct 2026; the simulator's IMU emulates it)


def lean_from_imu(euler, gyro, upright=UPRIGHT_EULER_X):
    """The robot's IMU (euler-x/y/z deg, gyro deg/s on the chip axes right, up, back) -> (lean back, lean right,
    their rates): euler-x is about 0 face down and upright when standing, euler-y + leaning right."""
    return euler[0] - upright, euler[1], gyro[0], -gyro[2]


def _dead(x, band):
    return x - band if x > band else x + band if x < -band else 0.0


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class Balance(object):
    """update(lean back, lean right, rate back, rate right, t) -> {servo: offset deg}. The first update (or reset())
    takes the standing lean as the reference."""
    PITCH = (("RightAnkleFB", 1.0), ("LeftAnkleFB", -1.0))  # + leans the torso forward
    ROLL = (("RightAnkleUD", 1.0), ("LeftAnkleUD", 1.0))  # + leans it left
    GAINS = {"kp": 0.6, "kd": 0.06, "kr": 0.5, "krd": 0.03, "deadband": 2.0, "roll_deadband": 4.0, "limit": 8.0,
             "rate_limit": 120.0}

    def __init__(self, **gains):
        self.g = dict(self.GAINS)
        self.g.update(gains)
        self.reset()

    def reset(self, ref=None):
        self.ref, self.pitch, self.roll, self.t = ref, 0.0, 0.0, None

    def update(self, back, right, rate_back, rate_right, t):
        g = self.g
        if self.ref is None:
            self.ref = (back, right)
        want_p = _clamp(g["kp"] * _dead(back - self.ref[0], g["deadband"]) + g["kd"] * rate_back, g["limit"])
        want_r = _clamp(g["kr"] * _dead(right - self.ref[1], g["roll_deadband"]) + g["krd"] * rate_right, g["limit"])
        step = g["rate_limit"] * (min(0.2, t - self.t) if self.t is not None else 0.05)
        self.pitch += _clamp(want_p - self.pitch, step)
        self.roll += _clamp(want_r - self.roll, step)
        self.t = t
        return self.offsets()

    def stand(self, back, right, rate_back, rate_right, quiet=15.0, a=0.1):
        """Standing still between movements: follow the standing lean (the reference) while the torso is quiet."""
        if abs(rate_back) < quiet and abs(rate_right) < quiet:
            self.ref = (back, right) if self.ref is None else (
                self.ref[0] + a * (back - self.ref[0]), self.ref[1] + a * (right - self.ref[1]))

    def offsets(self):
        d = dict((n, s * self.pitch) for n, s in self.PITCH)
        d.update((n, s * self.roll) for n, s in self.ROLL)
        return d

    def fade(self, t, seconds=0.3):
        """Back towards no offset (the walk has stopped): call until offsets() are all 0."""
        step = self.g["limit"] * (min(0.2, t - self.t) if self.t is not None else 0.05) / seconds
        self.pitch -= _clamp(self.pitch, step)
        self.roll -= _clamp(self.roll, step)
        self.t = t
        return self.offsets()
