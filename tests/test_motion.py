"""Motion checks, playback and the play_motion script. Run: python3 tests/test_motion.py"""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import RESET_POSE, motion  # noqa: E402

# Each pose is clear, but the straight line from the last frame back to reset hits the hip.
RESET_GAP = {"frames": [{"angles": {"LeftShoulderFlex": 66, "RightShoulderRoll": 113}, "runtime": 2000},
                        {"angles": {"RightElbowFlex": 155, "RightHipLR": 36}, "runtime": 2000}],
             "end_in_reset": True}
WAVE = os.path.join(ROOT, "motions", "wave_right.json")


class LogRobot:
    """Records commands; wait() adds to a clock and can fire a callback."""

    def __init__(self, on_wait=None):
        self.log, self.t, self.on_wait = [], 0.0, on_wait

    def set_servos_angles(self, angles, runtime=200):
        self.log.append(("send", dict(angles), runtime))
        return {"code": 0}

    def wait(self, seconds):
        self.t += seconds
        if self.on_wait:
            self.on_wait(self)

    def reset(self, runtime=1000):
        raise AssertionError("play() must send the reset itself so it can be cancelled")


def test_play_completes_and_scales_final_reset():
    bot = LogRobot()
    mo = {"frames": [{"angles": {"NeckLR": 60}, "runtime": 800, "hold": 200}], "end_in_reset": True}
    assert motion.play(bot, mo, speed=0.5) is True
    assert bot.log == [("send", {"NeckLR": 60}, 1600), ("send", RESET_POSE, 2000)]
    assert abs(bot.t - motion.duration_s(mo, 0.5)) < 1e-6 and abs(bot.t - 4.0) < 1e-6


def test_play_rejects_bad_speed():
    for speed in (0, -1, float("nan"), "fast"):
        try:
            motion.play(LogRobot(), {"frames": []}, speed=speed)
        except ValueError:
            continue
        raise AssertionError(f"speed {speed!r} accepted")


def test_play_stop_event_between_frames_and_during_waits():
    mo = json.load(open(WAVE))
    stop = threading.Event()
    bot = LogRobot()
    done = motion.play(bot, mo, on_frame=lambda i, f: i == 2 and stop.set(), stop_event=stop)
    assert done is False and len(bot.log) == 3  # frame 2 was sent, nothing after it, no reset

    stop.clear()
    bot = LogRobot(on_wait=lambda b: b.t >= 0.3 and stop.set())  # cancel inside frame 0's 0.9 s wait
    assert motion.play(bot, mo, stop_event=stop) is False
    assert len(bot.log) == 1 and bot.t < 0.4


def test_play_pause_frames_and_malformed_frames():
    bot = LogRobot()
    mo = {"frames": [{"hold": 500}, {"angles": {"NeckLR": 60, "Bogus": 3, "RightElbowFlex": "x"}}], "end_in_reset": False}
    assert motion.play(bot, mo) is True
    assert bot.log == [("send", {"NeckLR": 60}, 500)]
    assert abs(bot.t - 1.5) < 1e-6  # pause: 500 runtime + 500 hold; then 500


def test_validate_never_raises_on_malformed_input():
    cases = [None, [], "x", {}, {"frames": None}, {"frames": "x"}, {"frames": [None, 3, "x"]},
             {"frames": [{"angles": [1, 2]}]}, {"frames": [{"angles": {"NeckLR": None}}]},
             {"frames": [{"angles": {"NeckLR": "90"}, "runtime": "500"}]},
             {"frames": [{"angles": {"NeckLR": 90}, "runtime": None, "hold": -5}]},
             {"frames": [{"angles": {"NeckLR": 90}, "runtime": True}]}]
    for mo in cases:
        errors, _ = motion.validate(mo)
        assert errors, mo
        motion.duration_s(mo)  # must not raise either
        motion.self_collisions(mo, samples_per_frame=1, checker=CHECKER)
    assert motion.validate({"frames": [{"angles": {"NeckLR": 90}}]}, speed=0)[0]
    assert motion.validate({"frames": [{"hold": 300}]}) == ([], [])  # a pause frame is fine


def test_validate_checks_requested_speed_and_reset_move():
    mo = json.load(open(WAVE))
    assert motion.validate(mo) == ([], [])
    errors, warnings = motion.validate(mo, speed=3)
    assert not errors and any("RightShoulderFlex moves at" in w for w in warnings)
    _, warnings = motion.validate({"frames": [{"angles": {"NeckLR": 60}, "runtime": 3000}]}, speed=0.5)
    assert any("clamped to 4000" in w for w in warnings)
    end_far = {"frames": [{"angles": {"RightShoulderFlex": 0}, "runtime": 2000}], "end_in_reset": True}
    _, warnings = motion.validate(end_far, speed=4)
    assert warnings and all(w.startswith("return to reset") for w in warnings)
    # the start pose is where speeds are measured from
    _, warnings = motion.validate({"frames": [{"angles": {"NeckLR": 160}, "runtime": 200}]}, start_pose={"NeckLR": 20})
    assert warnings


def test_self_collisions_sweeps_return_to_reset():
    hits = motion.self_collisions(RESET_GAP, checker=CHECKER)
    assert hits and {h[0] for h in hits} == {2}  # frame len(frames) = the end_in_reset move
    assert motion.self_collisions({**RESET_GAP, "end_in_reset": False}, checker=CHECKER) == []


def test_self_collisions_start_pose():
    end = {**RESET_POSE, "LeftShoulderFlex": 66, "RightShoulderRoll": 113, "RightElbowFlex": 155, "RightHipLR": 36}
    mo = {"frames": [{"angles": dict(RESET_POSE), "runtime": 1000}], "end_in_reset": False}
    assert motion.self_collisions(mo, checker=CHECKER, start_pose=end)
    assert motion.self_collisions(mo, checker=CHECKER) == []


def test_contacts_baseline_override():
    touching = {**RESET_POSE, "RightElbowFlex": 0, "RightShoulderRoll": 170, "RightShoulderFlex": 0}
    pairs = CHECKER.contacts(touching)
    assert pairs
    assert CHECKER.contacts(touching, baseline=pairs | CHECKER.baseline) == set()
    assert CHECKER.contacts(RESET_POSE, baseline=set()) == CHECKER.baseline


def _script(*args):
    env = {**os.environ, "MUJOCO_GL": "egl", "YANSHEE_SIM_HEADLESS": "1"}
    return subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "play_motion.py"), *args],
                          capture_output=True, text=True, timeout=120, env=env)


def test_play_motion_script_argument_checks():
    for args in (["--speed", "0"], ["--speed", "-1"], ["--headless", "--loop"], ["--real"]):
        r = _script(WAVE, *args)
        assert r.returncode == 2, (args, r.stderr)


def test_play_motion_headless_reports_tilt_and_falls():
    r = _script(WAVE, "--headless")
    assert r.returncode == 0 and "max torso tilt during playback" in r.stdout, r.stdout + r.stderr
    with tempfile.TemporaryDirectory() as d:
        fall = os.path.join(d, "fall.json")
        motion.save({"name": "fall", "frames": [{"angles": {"RightHipFB": 10, "LeftHipFB": 170}, "runtime": 300,
                                                 "hold": 1500}], "end_in_reset": False}, fall)
        r = _script(fall, "--headless")
        assert r.returncode == 1 and "FALL" in r.stdout, r.stdout + r.stderr
        bad = os.path.join(d, "bad.json")
        motion.save({"frames": [{"angles": {"NeckLR": 200}}]}, bad)
        r = _script(bad, "--headless")
        assert r.returncode == 1 and "ERROR" in r.stdout, r.stdout + r.stderr


def test_play_motion_real_moves_to_reset_first():
    spec = importlib.util.spec_from_file_location("play_motion_script", os.path.join(ROOT, "scripts", "play_motion.py"))
    pm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pm)

    class Bot(LogRobot):
        def __init__(self, pose):
            super().__init__()
            self.pose = pose

        def get_servos_angles(self, names=None):
            return {"code": 0, "data": dict(self.pose)}

    bot = Bot({**RESET_POSE, "RightShoulderFlex": 20})
    pm.move_to_reset(bot)
    assert bot.log == [("send", RESET_POSE, 2000)] and bot.t == 2.0  # 120 deg at 60 deg/s
    bot = Bot({**RESET_POSE, "LeftShoulderFlex": 66, "RightShoulderRoll": 113, "RightElbowFlex": 155, "RightHipLR": 36})
    try:
        pm.move_to_reset(bot)
    except RuntimeError as e:
        assert "self-collide" in str(e) and bot.log == []
    else:
        raise AssertionError("colliding reset path was sent")


CHECKER = motion.CollisionChecker()

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
