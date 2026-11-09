"""tools/find_robot.py without the robot: the network list, the ARP parsing, the CLI.
Run: python3 -m pytest tests/test_find_robot.py -q"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
import find_robot as fr  # noqa: E402


def test_networks_skip_virtual_ones_and_cap_the_size(monkeypatch):
    text = ("2: wlo1    inet 192.168.89.103/23 brd 192.168.89.255 scope global dynamic noprefixroute wlo1\n"
            "5: tailscale0    inet 100.87.252.92/32 scope global tailscale0\n"
            "6: br-3db6b2b63000    inet 192.168.56.1/24 brd 192.168.56.255 scope global br-3db6b2b63000\n"
            "7: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0\n"
            "8: eth0    inet 10.0.0.7/16 brd 10.0.255.255 scope global eth0\n")
    monkeypatch.setattr(fr.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": text})())
    nets = fr.networks()
    assert [d for d, n in nets] == ["wlo1", "eth0"]
    assert str(nets[0][1]) == "192.168.88.0/23" and str(nets[1][1]) == "10.0.0.0/22"  # a /16 is capped to a /22


def test_arp_parsing(monkeypatch):
    text = ("192.168.89.99 dev wlo1 lladdr b8:27:eb:20:d3:af REACHABLE\n"
            "192.168.89.1 dev wlo1 lladdr 3c:64:cf:7d:e6:e0 STALE\n"
            "192.168.89.50 dev wlo1  FAILED\n")
    monkeypatch.setattr(fr.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": text})())
    assert fr.arp() == {"192.168.89.99": "b8:27:eb:20:d3:af", "192.168.89.1": "3c:64:cf:7d:e6:e0"}


def test_help():
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "tools", "find_robot.py"), "--help"],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0 and "--wait" in out.stdout and "--vnc" in out.stdout and fr.ROBOT_MAC in open(fr.__file__).read()


def test_avahi_ipv6_parsing():
    assert fr.parse_avahi("raspberrypi.local\tfe80::46d4:2eca:22be:f582\n") == "fe80::46d4:2eca:22be:f582"
    assert fr.parse_avahi("raspberrypi.local\t169.254.133.10\n") is None  # a self-assigned IPv4: not reachable, not used
    assert fr.parse_avahi("Failed to resolve host name 'raspberrypi.local': Timeout reached\n") is None


def test_ipv6_neighbour_mac(monkeypatch):
    text = "fe80::46d4:2eca:22be:f582 lladdr b8:27:eb:20:d3:af REACHABLE\nfe80::1 lladdr 3c:64:cf:7d:e6:e0 STALE\n"
    monkeypatch.setattr(fr.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": text})())
    assert fr.mac6("FE80::46D4:2ECA:22BE:F582", "wlo1") == fr.ROBOT_MAC
    assert fr.mac6("fe80::2", "wlo1") is None


def test_vnc_target_brackets_ipv6():
    assert fr.vnc_target("192.168.89.80") == "192.168.89.80:5900"
    assert fr.vnc_target("fe80::1%wlo1") == "[fe80::1%wlo1]:5900"


def test_saved_vnc_entry_and_config(tmp_path):
    store = tmp_path / "ViewerStore"
    store.mkdir()
    (store / "a.vnc").write_text("ConnMethod=tcp\nConnTime=2026-11-04T10:00:00Z\nHost=192.168.89.99:5900\nUserName=pi\nUuid=aaaa\n")
    (store / "b.vnc").write_text("ConnMethod=tcp\nConnTime=2026-11-09T11:24:34Z\nHost=[fe80::1%wlo1]:5900\nIdentity=00ff\nUserName=pi\nUuid=bbbb\n")
    (store / "c.vnc").write_text("ConnTime=2026-11-10T00:00:00Z\nHost=10.0.0.1:5900\nUserName=someone\nUuid=cccc\n")
    entry = fr.saved_vnc_entry(str(store))
    assert entry["Uuid"] == "bbbb"  # the newest entry for the robot's user, not another user's
    path = fr.vnc_config(entry, "192.168.89.80", str(tmp_path / "robot.vnc"))
    text = (tmp_path / "robot.vnc").read_text()
    assert text.splitlines()[0] == "Host=192.168.89.80:5900" and "Uuid=bbbb" in text and "Identity=00ff" in text
    assert "ConnTime" not in text and path.endswith("robot.vnc")
    assert fr.saved_vnc_entry(str(tmp_path / "missing")) is None
