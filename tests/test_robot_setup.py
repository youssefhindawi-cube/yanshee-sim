"""tools/robot_setup.py: the files it installs on the robot's Pi are sound (the robot itself is not needed).
Run: python3 -m pytest tests/test_robot_setup.py -q"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
import robot_setup  # noqa: E402


def test_installed_files_parse_and_carry_the_marker():
    for src, dst, mode in robot_setup.FILES:
        path = os.path.join(robot_setup.HERE, src)
        text = open(path).read()
        assert robot_setup.MARK in text, src  # --remove deletes only files with it
        if src.endswith(".sh") or src.endswith("exit-hook"):
            subprocess.run(["sh", "-n", path], check=True)
        if src.endswith(".service"):
            assert f"ExecStart={robot_setup.FILES[0][1]}" in text and "WantedBy=multi-user.target" in text
        assert mode in ("644", "755") and dst.startswith("/")


def test_help_and_status_command():
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "tools", "robot_setup.py"), "--help"],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0 and "--remove" in out.stdout
    assert "power_save" in robot_setup.STATUS and "synchronized" in robot_setup.STATUS
