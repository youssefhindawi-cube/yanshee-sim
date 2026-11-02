#!/usr/bin/env python3
"""Set the robot's Raspberry Pi up for the exhibition (files in tools/robot_setup/):
- Wi-Fi power saving off, at boot and after every reconnection (it added 0.3-2 s stalls, docs/SIM2REAL_FINDINGS.md
  "Link and Wi-Fi measured on the robot");
- the clock synced (systemd-timesyncd) as soon as the Wi-Fi is up, so it does not jump during a logged session.

    python3 tools/robot_setup.py --ip IP            # install (or update) and run it now
    python3 tools/robot_setup.py --ip IP --status   # what is installed, power saving, clock sync, the boot log line
    python3 tools/robot_setup.py --ip IP --remove   # back to how the robot came

Installs /usr/local/bin/yanshee-net-tune.sh, /etc/systemd/system/yanshee-net-tune.service and /etc/dhcpcd.exit-hook
(an existing exit hook that is not ours is left alone and reported). Nothing else on the robot is changed.
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_setup")
FILES = [("yanshee-net-tune.sh", "/usr/local/bin/yanshee-net-tune.sh", "755"),
         ("yanshee-net-tune.service", "/etc/systemd/system/yanshee-net-tune.service", "644"),
         ("dhcpcd.exit-hook", "/etc/dhcpcd.exit-hook", "644")]
MARK = "Installed by yanshee_sim"
STATUS = ("for f in " + " ".join(dst for _, dst, _ in FILES) + "; do [ -e $f ] && echo \"$f: $(grep -c '" + MARK +
          "' $f) ours\" || echo \"$f: missing\"; done; echo \"service: $(systemctl is-enabled yanshee-net-tune 2>&1) "
          "$(systemctl is-active yanshee-net-tune 2>&1)\"; /sbin/iw dev wlan0 get power_save; "
          "timedatectl | grep -i synchronized; journalctl -t yanshee-net-tune --no-pager -n 2 2>/dev/null | tail -2")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--status", action="store_true")
    g.add_argument("--remove", action="store_true")
    args = ap.parse_args()
    env = dict(os.environ, SSHPASS=args.password)
    opts = ["-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8"]
    host = f"{args.user}@{args.ip}"

    def ssh(cmd):
        return subprocess.run(["sshpass", "-e", "ssh", *opts, host, cmd], env=env, text=True, timeout=180)

    if args.status:
        return ssh(STATUS).returncode
    if args.remove:
        cmd = ("sudo systemctl disable --now yanshee-net-tune 2>/dev/null; " +
               "; ".join(f"grep -q '{MARK}' {dst} 2>/dev/null && sudo rm -f {dst}" for _, dst, _ in FILES) +
               "; sudo systemctl daemon-reload; echo removed; " + STATUS)
        return ssh(cmd).returncode
    tmp = "/tmp/yanshee_setup"
    if ssh(f"mkdir -p {tmp}").returncode:
        return 1
    r = subprocess.run(["sshpass", "-e", "scp", "-q", *opts, *[os.path.join(HERE, src) for src, _, _ in FILES],
                        f"{host}:{tmp}/"], env=env, timeout=60)
    if r.returncode:
        return r.returncode
    steps = []
    for src, dst, mode in FILES:  # never overwrite a file that someone else put there
        steps.append(f"if [ -e {dst} ] && ! grep -q '{MARK}' {dst}; then echo '{dst} exists and is not ours: left alone'; "
                     f"else sudo install -m {mode} -o root -g root {tmp}/{src} {dst}; fi")
    steps += ["sudo systemctl daemon-reload", "sudo systemctl enable yanshee-net-tune 2>/dev/null",
              "sudo systemctl restart yanshee-net-tune", f"rm -rf {tmp}", "sleep 6", STATUS]
    return ssh("; ".join(steps)).returncode


if __name__ == "__main__":
    sys.exit(main())
