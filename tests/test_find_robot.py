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
