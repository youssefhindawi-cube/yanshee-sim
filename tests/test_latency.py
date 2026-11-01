"""tools/latency_test.py without the robot: the link echo and REST timing locally, Wi-Fi parsing, the servo timing.
Run: python3 -m pytest tests/test_latency.py -q"""
import json
import os
import shutil
import sys
import tempfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
sys.path.insert(0, HERE)
import fake_robot  # noqa: E402
import latency_test as lt  # noqa: E402
from yanshee_sim import robot_stream  # noqa: E402
from yanshee_sim.servos import NAMES  # noqa: E402


def test_stats():
    s = lt.stats([5, 7, 6, 250, 8], tried=6)
    assert s["n"] == 5 and s["lost"] == 1 and s["median"] == 7 and s["max"] == 250 and s["stalls"] == 1
    assert "lost" not in lt.stats([1.0]) and lt.stats([], tried=3) == {"n": 0, "lost": 3}


def test_link_and_rest_locally():
    link = lt.link_test([sys.executable, "-u", "-c", lt.ECHO], 1.0, interval=0.05)
    assert link["n"] >= 15 and link["lost"] == 0 and link["median"] < 50, link
    srv = fake_robot.start(port=0)
    try:
        rest = lt.rest_test(f"127.0.0.1:{srv.server_address[1]}", 2.0)
    finally:
        fake_robot.stop(srv)
    assert rest["new_connection"]["n"] >= 2 and rest["kept_open"]["n"] >= 2, rest


@pytest.mark.skipif(not shutil.which("ping"), reason="no ping")
def test_ping_localhost():
    s = lt.ping_test("127.0.0.1", 1)
    assert s["n"] >= 3 and s["median"] < 5, s


def test_parse_wifi():
    iw = ("Connected to b8:27:eb:00:00:01 (on wlan0)\n\tSSID: show-net\n\tfreq: 2437\n\tsignal: -61 dBm\n"
          "\ttx bitrate: 65.0 MBit/s\nPower save: on\nInterface wlan0\n\tchannel 6 (2437 MHz), width: 20 MHz\n"
          "load 1.20 0.90 0.80 2/150 1234\n")
    w = lt.parse_wifi(iw)
    assert w == {"ssid": "show-net", "signal_dbm": -61, "tx_mbps": 65, "freq_mhz": 2437, "channel": 6,
                 "power_save": "on", "load": "1.20 0.90 0.80"}, w
    old = "wlan0  IEEE 802.11  ESSID:\"x\"\n  Bit Rate=72.2 Mb/s\n  Power Management:off\n  Link Quality=50/70  Signal level=-48 dBm\n"
    w = lt.parse_wifi(old)
    assert w["signal_dbm"] == -48 and w["tx_mbps"] == 72.2 and w["power_save"] == "off", w


def test_move_stats():
    """Frame at t; encoders every 40 ms; the arm starts moving 60 ms after the frame and settles by 180 ms."""
    i, t0, rows = NAMES.index(lt.MOVE_JOINT), 100.0, []
    for k in range(2):  # out to 55 deg and back to 40
        tf, a, b = t0 + 1.2 * k, (40, 55)[k], (55, 40)[k]
        rows.append({"k": "frame", "src": "ours", "seq": k + 1, "t": tf, "c": robot_stream.to_units({lt.MOVE_JOINT: b})})
        for j in range(-3, 28):
            dt = j * 0.04
            deg = a if dt < 0.06 else b if dt >= 0.18 else (a + b) / 2
            s = [90] * len(NAMES)
            s[i] = round(deg)
            rows.append({"k": "enc", "t": tf + dt + 0.001, "ms": 30, "s": s})
    path = os.path.join(tempfile.mkdtemp(), "robot_latency.jsonl")
    with open(path, "w") as f:
        f.writelines(json.dumps(r) + "\n" for r in sorted(rows, key=lambda r: r["t"]))
    m = lt.move_stats(path)
    assert m["moves"] == 2 and m["encoder_every_ms"] == 40 and m["settled_off_command_deg"] <= 1, m
    assert 70 <= m["frame_to_first_motion"]["median"] <= 100 and 190 <= m["frame_to_settled"]["median"] <= 220, m


def test_relay_test():
    class St:
        def __init__(self):
            self.n = 0

        def rest(self, method, path, timeout=4.0):
            self.n += 1
            if self.n == 3:
                raise RuntimeError("no reply through the robot link")
            return 200, b"{}", "application/json", 60
    r = lt.relay_test(St(), 4)
    assert r["through_link"]["n"] == 3 and r["through_link"]["lost"] == 1 and r["rest_server"]["median"] == 60, r
