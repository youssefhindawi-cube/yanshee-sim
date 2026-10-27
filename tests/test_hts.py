"""The robot's .hts motion format (yanshee_sim/hts.py): round trips, the record layout, checksums, limits,
and motion.load()/save() dispatch on the extension.
Run: python3 -m pytest tests/test_hts.py -q  (or python3 tests/test_hts.py)"""
import os
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from yanshee_sim import hts, motion  # noqa: E402
from yanshee_sim.servos import NAMES, RESET_POSE  # noqa: E402

WAVE = {"name": "wave", "frames": [{"angles": {"RightShoulderFlex": 30, "RightElbowFlex": 90}, "runtime": 800, "hold": 100},
                                   {"angles": {"RightShoulderFlex": 60}, "runtime": 500},
                                   {"angles": {"RightShoulderFlex": 30, "NeckLR": 110}, "runtime": 460, "hold": 0}],
        "end_in_reset": True}


def _raises(fn, exc=hts.HtsError):
    try:
        fn()
    except exc:
        return True
    return False


def test_layout():
    data = hts.build(WAVE)
    assert len(data) == 33 * (4 + 2) and data[:33] == bytes(33)  # 3 frames + the reset frame
    assert struct.unpack("<I", data[-4:])[0] == 900 + 500 + 460 + 1000  # 20 ms ticks
    f = data[33:66]
    assert f[0] == 0xFB and f[1] == 0xBF and f[2] == hts.KIND_APP and f[3] == 1 and f[32] == 0xED
    assert struct.unpack("<H", f[4:6])[0] == 4 and struct.unpack("<H", f[6:8])[0] == 0
    assert f[8 + NAMES.index("RightShoulderFlex")] == 30 and f[8 + NAMES.index("LeftHipLR")] == 0xFF
    assert f[28] == 40 and (f[29] << 8 | f[30]) == 45 and f[31] == sum(f[2:31]) & 0xFF
    assert data[66 + 3] == 2 and data[99 + 3] == 2 and data[132 + 3] == 3
    last = data[132:165]
    assert all(last[8 + k] == RESET_POSE[n] for k, n in enumerate(NAMES)) and last[28] == 50


def test_round_trip_and_limits():
    mo = hts.parse(hts.build(WAVE), "wave")
    assert [fr["angles"] for fr in mo["frames"]] == [WAVE["frames"][0]["angles"], WAVE["frames"][1]["angles"],
                                                     WAVE["frames"][2]["angles"], RESET_POSE]
    assert [(fr["runtime"], fr["hold"]) for fr in mo["frames"]] == [(800, 100), (500, 0), (460, 0), (1000, 0)]
    assert mo["hts_total_ms"] == 2860 and mo["end_in_reset"] is False
    assert not motion.validate(mo)[0], motion.validate(mo)  # no errors
    assert hts.build(mo) == hts.build(WAVE)  # stable
    wild = {"frames": [{"angles": {"NeckLR": 200.4, "RightHipLR": -3}, "runtime": 9000},
                       {"angles": {"NeckLR": 90.6}, "runtime": 50}]}
    back = hts.parse(hts.build(wild))  # the writer keeps 0-180; the reader clamps to the servo limits
    assert back["frames"][0]["angles"] == {"NeckLR": 165, "RightHipLR": 0} and back["frames"][0]["runtime"] == 4000
    assert back["frames"][1]["angles"] == {"NeckLR": 91} and back["frames"][1]["runtime"] == 200
    assert back["clamped"] == ["frame 1: NeckLR 180 -> 165"] and "1 angle(s) clamped" in back["description"]
    app = hts.parse(hts.build({"frames": [{"angles": {"LeftHipFB": 171}, "runtime": 600}]}))  # as app files do
    assert app["frames"][0]["angles"] == {"LeftHipFB": 170} and not motion.validate(app)[0]
    assert _raises(lambda: hts.build({"frames": []}))


def test_bad_files_rejected():
    data = bytearray(hts.build(WAVE))
    assert _raises(lambda: hts.parse(bytes(data[:-1])))  # size
    bad = bytearray(data)
    bad[33 + 10] ^= 0x01  # an angle changed without the checksum
    assert _raises(lambda: hts.parse(bytes(bad)))
    bad = bytearray(data)
    bad[33] = 0x00  # marker
    assert _raises(lambda: hts.parse(bytes(bad)))
    sdk = bytearray(data)  # 2018 SDK files use kind 1: still readable
    for i in range(1, len(sdk) // 33 - 1):
        sdk[i * 33 + 2] = 1
        sdk[i * 33 + 31] = sum(sdk[i * 33 + 2:i * 33 + 31]) & 0xFF
    assert hts.parse(bytes(sdk))["frames"][0]["angles"] == WAVE["frames"][0]["angles"]


def test_motion_module_dispatch_and_cli():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "wave_test.hts")
        motion.save(WAVE, p)
        mo = motion.load(p)
        assert mo["name"] == "wave_test" and len(mo["frames"]) == 4 and mo["frames"][0]["runtime"] == 800
        j = os.path.join(d, "back.json")
        assert hts.main([p, j]) == 0 and motion.load(j)["frames"][3]["angles"] == RESET_POSE
        h2 = os.path.join(d, "again.hts")
        assert hts.main([j, h2]) == 0 and open(h2, "rb").read() == open(p, "rb").read()
        assert hts.main([p]) == 0 and hts.main(["x.txt", "y.txt"]) == 2 and hts.main([]) == 2


def test_controller_lists_and_plays_hts():
    """The web GUI's motion folder accepts the robot's .hts files next to JSON ones."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    from yanshee_sim.controller import Controller
    with tempfile.TemporaryDirectory() as d:
        motion.save(WAVE, os.path.join(d, "wave_hts.hts"))
        motion.save(WAVE, os.path.join(d, "both.json"))
        motion.save(WAVE, os.path.join(d, "both.hts"))  # shadowed by both.json
        c = Controller(motions_dir=d)
        try:
            files = {m["name"]: m for m in c.motion_files()}
            assert files["wave_hts"]["format"] == "hts" and files["wave_hts"]["frames"] == 4
            assert files["both"]["format"] == "json" and files["both"]["frames"] == 3
            mo = c.load_motion("wave_hts")
            assert mo["ok"] and mo["motion"]["frames"][0]["runtime"] == 800
            r = c.play_motion("wave_hts")
            assert r["ok"], r
            assert c.delete_motion("wave_hts")["ok"] and not os.path.exists(os.path.join(d, "wave_hts.hts"))
        finally:
            c.close()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
