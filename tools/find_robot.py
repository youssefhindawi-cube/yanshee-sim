#!/usr/bin/env python3
"""Find the Yanshee robot on the network this laptop is on, and say whether it is reachable.

    python3 tools/find_robot.py            # scan, print the robot's IP, check its REST server (9090) and ssh (22)
    python3 tools/find_robot.py --json     # the same as one JSON line (for other tools)
    python3 tools/find_robot.py --wait     # keep the window open at the end (the Desktop launcher uses this)
    python3 tools/find_robot.py --vnc      # ... and open the robot's screen in RealVNC Viewer (login pi / raspberry)

The robot's Wi-Fi MAC address (ROBOT_MAC) is the thing that does not change; its IP does with every network. The
quick ways first (the address it had last time, the addresses it has had, raspberrypi.local over mDNS), then a ping
sweep of the laptop's networks (up to /22, about 1000 addresses, a few seconds) and the ARP table. A find is confirmed
by GET /v1/devices/battery on port 9090 (battery, charging) and a TCP connect to port 22; the address is remembered
in ~/.yanshee_robot_ip. Exit code 0 found, 1 not found.
"""
import argparse
import concurrent.futures
import ipaddress
import json
import os
import re
import socket
import subprocess
import sys
import time
import urllib.request

ROBOT_MAC = "b8:27:eb:20:d3:af"
ROBOT_USER = "pi"
PI_OUIS = ("b8:27:eb", "dc:a6:32", "e4:5f:01", "28:cd:c1", "d8:3a:dd")  # Raspberry Pi Foundation prefixes
KNOWN_IPS = ("192.168.89.137", "192.168.89.74", "192.168.89.99")  # addresses it has had (office network)
MDNS_NAME = "raspberrypi.local"
REMEMBER = os.path.expanduser("~/.yanshee_robot_ip")
REST_PORT, SSH_PORT, VNC_PORT = 9090, 22, 5900
# The robot runs RealVNC Server (Raspbian's) as delivered: its desktop on port 5900, login with the robot's own user
# and password. Only RealVNC's viewer can log in to it (TigerVNC and Remmina cannot: the server offers RealVNC's
# authentication only), so --vnc opens RealVNC Connect Viewer, installed from realvnc.com (package
# realvnc-rvncconnect-viewer). Nothing is changed on the robot.
REALVNC_VIEWERS = ("/usr/lib/rvncconnect/rvncconnect", "/usr/bin/vncviewer-realvnc", "/usr/lib/vncviewer/vncviewer")
MAX_PREFIX = 22  # never sweep more than a /22


def networks():
    """[(interface, IPv4Network)] of the laptop's real networks (no docker, bridges, VPNs, loopback, /32)."""
    out = []
    try:
        text = subprocess.run(["ip", "-4", "-o", "addr", "show", "scope", "global"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return out
    for line in text.splitlines():
        p = line.split()
        if len(p) < 4:
            continue
        dev, cidr = p[1], p[3]
        if dev == "lo" or dev.startswith(("docker", "br-", "veth", "tailscale", "virbr", "tun", "wg")):
            continue
        try:
            net = ipaddress.ip_interface(cidr).network
        except ValueError:
            continue
        if net.prefixlen >= 31:
            continue
        if net.prefixlen < MAX_PREFIX:  # a huge network: sweep the /22 around the laptop's own address
            net = ipaddress.ip_network(f"{ipaddress.ip_interface(cidr).ip}/{MAX_PREFIX}", strict=False)
        out.append((dev, net))
    return out


def ping(ip, wait_s=1.0):
    try:
        return subprocess.run(["ping", "-c", "1", "-W", str(int(wait_s)), str(ip)], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=wait_s + 2).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def sweep(net, workers=128):
    """Ping every address of the network (in parallel) so the ARP table fills; returns the ones that answered."""
    hosts = list(net.hosts())
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        return [str(ip) for ip, up in zip(hosts, ex.map(ping, hosts)) if up]


def arp():
    """{ip: mac} from the kernel's neighbour table."""
    out = {}
    try:
        text = subprocess.run(["ip", "-4", "neigh"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return out
    for line in text.splitlines():
        m = re.match(r"(\S+) .*lladdr ([0-9a-f:]{17})", line)
        if m:
            out[m.group(1)] = m.group(2).lower()
    return out


def mac_of(ip):
    """The MAC the laptop has for ip (a ping first, so the table has it)."""
    ping(ip, 1.0)
    return arp().get(ip)


def rest_battery(ip, timeout=3.0):
    """The robot's battery over REST: {percent, charging, voltage} or None."""
    try:
        with urllib.request.urlopen(f"http://{ip}:{REST_PORT}/v1/devices/battery", timeout=timeout) as r:
            d = json.loads(r.read().decode())
        return d.get("data") if d.get("code") == 0 else None
    except Exception:  # noqa: BLE001 - unreachable, refused, not the robot
        return None


def port_open(ip, port, timeout=2.0):
    try:
        socket.create_connection((ip, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def mdns():
    try:
        return socket.gethostbyname(MDNS_NAME)
    except OSError:
        return None


def candidates():
    out = []
    try:
        with open(REMEMBER) as f:
            out.append(("last time", f.read().strip()))
    except OSError:
        pass
    m = mdns()
    if m:
        out.append((MDNS_NAME, m))
    out += [("known address", ip) for ip in KNOWN_IPS]
    seen, uniq = set(), []
    for how, ip in out:
        if ip and ip not in seen:
            seen.add(ip)
            uniq.append((how, ip))
    return uniq


def find(verbose=True):
    """Where the robot is: {"ip", "mac", "how", "battery", "ssh"} or None; other Raspberry Pis seen go in "pis"."""
    say = (lambda *a: print(*a, flush=True)) if verbose else (lambda *a: None)
    for how, ip in candidates():
        mac = mac_of(ip)
        if mac == ROBOT_MAC:
            say(f"{ip} answers and has the robot's MAC ({how})")
            return confirm(ip, how, say)
        say(f"  {ip} ({how}): {'not answering' if mac is None else 'answers but it is ' + mac + ', not the robot'}")
    nets = networks()
    if not nets:
        say("no network: is the laptop's Wi-Fi on?")
        return None
    for dev, net in nets:
        say(f"sweeping {net} on {dev} ({net.num_addresses - 2} addresses) ...")
        up = sweep(net)
        table = arp()
        hit = next((ip for ip, mac in table.items() if mac == ROBOT_MAC and ipaddress.ip_address(ip) in net), None)
        if hit:
            say(f"{hit} has the robot's MAC")
            return confirm(hit, f"sweep of {net}", say)
        pis = sorted((ip, mac) for ip, mac in table.items() if mac[:8] in PI_OUIS and ipaddress.ip_address(ip) in net)
        say(f"  {len(up)} addresses answered, the robot's MAC {ROBOT_MAC} is not among them"
            + (f"; other Raspberry Pis: {', '.join(f'{ip} ({mac})' for ip, mac in pis)}" if pis else ""))
    return None


def confirm(ip, how, say):
    bat = rest_battery(ip)
    ssh = port_open(ip, SSH_PORT)
    res = {"ip": ip, "mac": ROBOT_MAC, "how": how, "battery": bat, "ssh": ssh, "rest": bat is not None, "t": time.time()}
    try:
        with open(REMEMBER, "w") as f:
            f.write(ip + "\n")
    except OSError:
        pass
    return res


def realvnc_viewer():
    """The RealVNC viewer's executable on this laptop, or None."""
    return next((v for v in REALVNC_VIEWERS if os.access(v, os.X_OK)), None)


def vnc_open(ip, say=print):
    """The robot's screen in RealVNC Viewer: starts the viewer on ip:5900 (it asks for the robot's user and password
    the first time and can remember them). Returns True when the viewer was started."""
    viewer = realvnc_viewer()
    if viewer is None:
        say("RealVNC Viewer is not installed on this laptop: download 'DEB x64' from "
            "https://www.realvnc.com/en/connect/download/viewer/linux/ and run  sudo apt install ~/Downloads/RealVNC-*.deb")
        return False
    if not port_open(ip, VNC_PORT):
        say(f"the robot's VNC server is not answering on port {VNC_PORT} (it starts with the robot; wait a minute)")
        return False
    subprocess.Popen([viewer, f"{ip}:{VNC_PORT}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    say(f"RealVNC Viewer opened on {ip}:{VNC_PORT}: log in as {ROBOT_USER} with the robot's password (it can remember them)")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", action="store_true", help="print one JSON line instead of text")
    ap.add_argument("--wait", action="store_true", help="wait for Enter before exiting (a launcher's window stays open)")
    ap.add_argument("--vnc", action="store_true", help="then open the robot's screen in RealVNC Viewer")
    args = ap.parse_args()
    res = find(verbose=not args.json)
    if args.json:
        print(json.dumps(res))
    elif res is None:
        print("\nROBOT NOT FOUND on this network. Check: is it switched on (chest light), is it on the same Wi-Fi as this "
              "laptop (the Yanshee app shows its network and IP), did it finish booting (about 3 minutes)?")
    else:
        b = res["battery"]
        print(f"\nYANSHEE ROBOT: {res['ip']}   (MAC {res['mac']}, found by {res['how']})")
        print("  REST server (port 9090): " + (f"OK, battery {b['percent']}%, {'charging' if b.get('charging') else 'not charging'}, "
                                             f"{b.get('voltage', '?')} mV" if b else "NOT answering (still booting? wait a minute and run again)"))
        print(f"  ssh (port 22): {'OK' if res['ssh'] else 'NOT answering'}")
        print(f"\n  GUI with the robot:  python3 scripts/web_gui.py --viewer --ip {res['ip']} --mode robot")
        if args.vnc:
            print()
            vnc_open(res["ip"])
    if args.wait:
        try:
            input("\nPress Enter to close.")
        except EOFError:
            pass
    return 0 if res else 1


if __name__ == "__main__":
    sys.exit(main())
