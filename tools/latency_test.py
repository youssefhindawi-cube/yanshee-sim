#!/usr/bin/env python3
"""Latency test: how fast the laptop reaches the robot, and how often the link stalls. Run it before and after each
change (the robot's Wi-Fi power saving, a travel router, another Wi-Fi adapter) and compare.

    python3 tools/latency_test.py --ip 192.168.89.74                   # 60 s; the robot is not moved
    python3 tools/latency_test.py --ip IP --seconds 300 --label "travel router"
    python3 tools/latency_test.py --ip IP --power-save off             # first turn the robot's Wi-Fi power saving off
    python3 tools/latency_test.py --ip IP --relay                      # also: REST through the robot link
    python3 tools/latency_test.py --ip IP --move                       # also: command -> servo (moves the left arm)
    python3 tools/latency_test.py --compare robot_dump/latency_test_*.json  # results side by side

For --seconds, all at once:
- ping: ICMP round trip, the network alone, 5 per second;
- link: round trip over the robot link's transport (ssh to a Python 2 echo on the robot, a frame-sized line, the way
  robot_stream.py's frames, readings and clock pings go), 5 per second;
- rest: GET /v1/devices/battery from the robot's REST server (port 9090: AppWalk's walk and turn commands go there),
  once a second over a new connection (as rest.py does) and over one kept-open connection;
- wifi: the robot's and the laptop's Wi-Fi (signal, bitrate, frequency, power saving) and the robot's load, before
  and after.
--power-save on|off sets the robot's Wi-Fi power saving first (sudo iw; it lasts until the robot restarts).
--relay (GUI disconnected: one robot link at a time): the same REST call through the robot link (the way the GUI
sends AppWalk's commands while the link is up), and the time the REST server on the robot took for it.
--move (GUI disconnected, robot standing still): through the robot link, N single frames moving LeftShoulderFlex
15 deg and back; from the robot's own log (robot clock, so no network in it): frame -> first encoder reading that
moved -> encoders settled. The encoders are read about every 40 ms, so these are +- one reading.
Results are printed and saved to robot_dump/latency_test_<stamp>.json. (tools/robot_checks.py latency times
several REST calls on their own: on 29 Oct the battery call took a median 64 ms, against a 7 ms link round trip.)
"""
import argparse
import bisect
import http.client
import json
import os
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import robot_stream  # noqa: E402
from yanshee_sim.rest import split_host  # noqa: E402
from yanshee_sim.servos import LIMITS, NAMES  # noqa: E402

STALL_MS = 100  # a reply slower than this counts as a stall
ECHO = "import sys\nfor l in iter(sys.stdin.readline, ''):\n    sys.stdout.write(l)\n    sys.stdout.flush()\n"
PAYLOAD = "x" * 110  # about one of the link's servo frames
WIFI_CMD = ("export PATH=$PATH:/sbin:/usr/sbin; iw dev wlan0 link; iw dev wlan0 get power_save; iw dev wlan0 info; "
            "iwconfig wlan0; echo load $(cat /proc/loadavg)")
WIFI = (("ssid", r"SSID:\s*(\S+)"), ("signal_dbm", r"signal:\s*(-?\d+)|Signal level=(-?\d+)"),
        ("tx_mbps", r"tx bitrate:\s*([\d.]+)|Bit Rate=([\d.]+)"), ("freq_mhz", r"freq:\s*(\d+)"),
        ("channel", r"channel (\d+)"), ("power_save", r"Power save:\s*(\w+)|Power Management:(\w+)"),
        ("load", r"^load ([\d.]+ [\d.]+ [\d.]+)"))
MOVE_JOINT, MOVE_DEG = "LeftShoulderFlex", 15.0


def stats(ms, tried=None):
    """Round trips in ms -> n, lost (tried - n, if tried is given), min, median, p95, p99, max, stalls (over STALL_MS)."""
    ms = sorted(ms)
    d = {"n": len(ms)}
    if tried is not None:
        d["lost"] = max(0, tried - len(ms))
    if ms:
        def q(p):
            return ms[min(len(ms) - 1, int(p * len(ms)))]
        d.update(min=ms[0], median=statistics.median(ms), p95=q(0.95), p99=q(0.99), max=ms[-1],
                 stalls=sum(1 for x in ms if x > STALL_MS))
    return {k: round(v, 1) if isinstance(v, float) else v for k, v in d.items()}


def ping_test(host, seconds, interval=0.2):
    out = subprocess.run(["ping", "-n", "-i", str(interval), "-w", str(max(1, int(seconds))), host],
                         capture_output=True, text=True).stdout
    tried = re.search(r"(\d+) packets transmitted", out)
    ms = [float(x) for x in re.findall(r"time=([\d.]+) ms", out)]
    return stats(ms, int(tried.group(1)) if tried else len(ms))


def link_test(cmd, seconds, interval=0.2, env=None):
    """Round trips (ms) of frame-sized lines through cmd, a process echoing them back (ssh to the robot)."""
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                         bufsize=1, env=env)
    sent, got, ready = {}, [], threading.Event()

    def read():
        for line in p.stdout:
            back = time.monotonic()
            seq = line.split(" ", 1)[0].strip()
            if seq == "0":
                ready.set()
            elif seq.isdigit() and int(seq) in sent:
                got.append((back - sent[int(seq)]) * 1000)
    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    p.stdin.write("0\n")  # the first echo: logged in (ssh), not timed
    p.stdin.flush()
    if not ready.wait(20):
        p.kill()
        raise RuntimeError("no echo (ssh to the robot failed?)")
    seq, end = 0, time.monotonic() + seconds
    while time.monotonic() < end:
        seq += 1
        sent[seq] = time.monotonic()
        p.stdin.write(f"{seq} {PAYLOAD}\n")
        p.stdin.flush()
        time.sleep(max(0.0, sent[seq] + interval - time.monotonic()))
    time.sleep(3.0)  # the last replies
    p.stdin.close()
    try:
        p.wait(5)
    except subprocess.TimeoutExpired:
        p.kill()
    reader.join(2)
    return stats(got, seq)


def rest_test(host, seconds, path="/v1/devices/battery"):
    """Round trips (ms) of a read-only REST call, once a second over a new connection and over one kept open."""
    name, port = split_host(host)
    new, kept, conn, n, keeps = [], [], None, 0, True
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        n += 1
        t0 = time.monotonic()
        try:
            c = http.client.HTTPConnection(name, port, timeout=3)
            c.request("GET", path)
            c.getresponse().read()
            c.close()
            new.append((time.monotonic() - t0) * 1000)
        except (OSError, http.client.HTTPException):
            pass
        t1 = time.monotonic()
        try:
            conn = conn or http.client.HTTPConnection(name, port, timeout=3)
            conn.request("GET", path)
            r = conn.getresponse()
            r.read()
            kept.append((time.monotonic() - t1) * 1000)
            if r.will_close:  # the server closes it: the next call connects again
                keeps, conn = False, conn.close()
        except (OSError, http.client.HTTPException):
            conn = conn.close() if conn else None
        time.sleep(max(0.0, t0 + 1.0 - time.monotonic()))
    if conn:
        conn.close()
    return {"new_connection": stats(new, n), "kept_open": stats(kept, n), "server_keeps_connection": keeps}


def parse_wifi(text):
    d = {}
    for key, pat in WIFI:
        m = re.search(pat, text, re.M)
        if m:
            v = next(g for g in m.groups() if g is not None)
            d[key] = float(v) if re.fullmatch(r"-?[\d.]+", v) else v
    return d


def robot_wifi(ssh, env):
    try:
        r = subprocess.run(ssh + [WIFI_CMD], env=env, capture_output=True, text=True, timeout=20)
        return parse_wifi(r.stdout) or {"error": (r.stderr.strip() or "no Wi-Fi details")[:200]}
    except subprocess.TimeoutExpired:
        return {"error": "ssh timed out"}


def laptop_wifi(host):
    m = re.search(r"dev (\S+)", subprocess.run(["ip", "route", "get", host], capture_output=True, text=True).stdout)
    if not m:
        return {}
    dev = m.group(1)
    if not os.path.isdir(f"/sys/class/net/{dev}/wireless"):
        return {"dev": dev, "wired": True}
    text = ""
    if shutil.which("iw"):
        text = "".join(subprocess.run(["iw", "dev", dev, *a], capture_output=True, text=True).stdout
                       for a in (["link"], ["get", "power_save"], ["info"]))
    else:  # no iw: the signal from the kernel, the rest from NetworkManager
        try:
            row = next(x for x in open("/proc/net/wireless") if x.strip().startswith(dev + ":"))
            text += f"signal: {float(row.split()[3]):.0f} dBm\n"
        except (OSError, StopIteration, IndexError, ValueError):
            pass
        if shutil.which("nmcli"):
            out = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,CHAN,FREQ,RATE", "dev", "wifi", "list", "ifname", dev,
                                  "--rescan", "no"], capture_output=True, text=True).stdout
            p = next((x.split(":") for x in out.splitlines() if x.startswith("yes:")), [])
            if len(p) >= 4:
                text += f"channel {p[1]}\nfreq: {p[2].split()[0]}\ntx bitrate: {p[3].split()[0]}\n"
    return dict(parse_wifi(text), dev=dev, wired=False)


def node_tests(host, user, password, relay_n, move_n, dest):
    """Through one robot link: relay_test, then move_test and move_stats on the robot's log of it."""
    st = robot_stream.RobotStream(host, user=user, password=password, enc_hz=25.0 if move_n else 5.0,
                                  session="latency_" + time.strftime("%Y%m%d_%H%M%S"))
    r = {}
    try:
        if relay_n:
            r["relay"] = guarded(relay_test, st, relay_n)
        if move_n:
            r["move"] = guarded(move_test, st, move_n)
    finally:
        st.close()
    if move_n and "error" not in r["move"]:
        r["move"] = guarded(lambda: move_stats(st.fetch_log(dest)))
    return r


def relay_test(st, n, path="devices/battery"):
    """n REST calls through the link, 5 per second: the round trip and the REST server's own time on the robot."""
    total, server = [], []
    for _ in range(n):
        t0 = time.monotonic()
        try:
            ms = st.rest("GET", path, timeout=3)[3]
            total.append((time.monotonic() - t0) * 1000)
            server.append(float(ms))
        except RuntimeError:
            pass
        time.sleep(max(0.0, t0 + 0.2 - time.monotonic()))
    return {"through_link": stats(total, n), "rest_server": stats(server)}


def move_test(st, n):
    """N moves of MOVE_JOINT by MOVE_DEG and back, one frame each, through the robot link."""
    end = time.time() + 5
    while st.fresh_encoders() is None and time.time() < end:
        time.sleep(0.1)
    enc = st.fresh_encoders()
    if enc is None:
        raise RuntimeError("no encoder readings from the robot")
    base, (lo, hi) = enc[MOVE_JOINT], LIMITS[MOVE_JOINT]
    target = base + MOVE_DEG if base + MOVE_DEG <= hi else base - MOVE_DEG
    for _ in range(n):
        for deg in (target, base):
            st.send({MOVE_JOINT: deg}, ms=20)
            time.sleep(1.2)
    return {}


def move_stats(path, joint=MOVE_JOINT, moved=3.0, near=2.0):
    """From a robot log: for each of our frames that moves joint, ms from the frame to the first encoder reading
    moved by `moved` deg, and to the first within `near` deg of where it settled (the last reading before the next
    frame); and how far the settled reading is from the command."""
    rows = []
    for line in open(path):
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    i = NAMES.index(joint)
    frames = [r for r in rows if r.get("k") == "frame" and r.get("src") == "ours"]
    encs = sorted((r for r in rows if r.get("k") == "enc" and len(r.get("s") or []) > i), key=lambda r: r["t"])
    et = [e["t"] for e in encs]

    def when(e):  # the middle of the reading
        return e["t"] + e.get("ms", 0) / 2000.0
    first, settle, err = [], [], []
    for k, f in enumerate(frames):
        cmd = f["c"][i] * 360.0 / 4096
        j = bisect.bisect_left(et, f["t"])
        stop = frames[k + 1]["t"] if k + 1 < len(frames) else f["t"] + 1.5
        after = [e for e in encs[j:] if e["t"] < stop]
        if j == 0 or not after or abs(cmd - encs[j - 1]["s"][i]) < 2 * moved:
            continue
        was, end = encs[j - 1]["s"][i], after[-1]["s"][i]
        m = next((e for e in after if abs(e["s"][i] - was) >= moved), None)
        a = next((e for e in after if abs(e["s"][i] - end) <= near), None)
        if m:
            first.append((when(m) - f["t"]) * 1000)
        if a:
            settle.append((when(a) - f["t"]) * 1000)
        err.append(abs(end - cmd))
    gaps = [b - a for a, b in zip(et, et[1:])]
    return {"moves": len(err), "frame_to_first_motion": stats(first), "frame_to_settled": stats(settle),
            "settled_off_command_deg": round(statistics.median(err), 1) if err else None,
            "encoder_every_ms": round(statistics.median(gaps) * 1000) if gaps else None}


def fmt(s):
    if not s or "error" in s:
        return (s or {}).get("error", "not run")
    if not s.get("n"):
        return f"no replies ({s.get('lost', 0)} lost)"
    lost = f", lost {s['lost']} of {s['n'] + s['lost']}" if "lost" in s else ""
    return (f"median {s['median']:.0f} ms, 95% {s['p95']:.0f}, 99% {s['p99']:.0f}, max {s['max']:.0f}; "
            f"{s['stalls']} over {STALL_MS} ms{lost}")


def wifi_line(w):
    if not w:
        return "?"
    if "error" in w:
        return w["error"]
    if w.get("wired"):
        return f"wired ({w['dev']})"
    parts = [f"{w['signal_dbm']:.0f} dBm" if "signal_dbm" in w else None, f"{w['tx_mbps']:g} Mb/s" if "tx_mbps" in w else None,
             f"{w['freq_mhz'] / 1000:g} GHz" if "freq_mhz" in w else None,
             f"channel {w['channel']:.0f}" if "channel" in w else None,
             f"power saving {w['power_save']}" if "power_save" in w else None, f"load {w['load']}" if "load" in w else None]
    return ", ".join(p for p in parts if p)


def show(r):
    print(f"latency test {r['stamp']}{' (' + r['label'] + ')' if r.get('label') else ''}: robot {r['ip']}, {r['seconds']} s")
    print(f"  ping (network)              {fmt(r.get('ping'))}")
    print(f"  link (ssh, as robot_stream) {fmt(r.get('link'))}")
    rest = r.get("rest") or {}
    if "error" in rest:
        print(f"  REST                        {rest['error']}")
    elif rest:
        print(f"  REST, new connection        {fmt(rest['new_connection'])}")
        print(f"  REST, kept open             {fmt(rest['kept_open'])}"
              + ("" if rest["server_keeps_connection"] else " (the server closes it: reconnects each time)"))
    print(f"  robot Wi-Fi                 before: {wifi_line(r.get('robot_wifi'))}; after: {wifi_line(r.get('robot_wifi_after'))}")
    print(f"  laptop                      {wifi_line(r.get('laptop_wifi'))}")
    rl = r.get("relay")
    if rl:
        if "error" in rl:
            print(f"  REST through the link       {rl['error']}")
        else:
            print(f"  REST through the link       {fmt(rl['through_link'])}")
            print(f"    of which the REST server  {fmt(rl['rest_server'])}")
    mv = r.get("move")
    if mv:
        if "error" in mv:
            print(f"  command -> servo            {mv['error']}")
        else:
            print(f"  command -> servo moves      {fmt(mv['frame_to_first_motion'])} ({mv['moves']} moves, "
                  f"a reading every {mv['encoder_every_ms']} ms)")
            print(f"  command -> servo settled    {fmt(mv['frame_to_settled'])}; settled {mv['settled_off_command_deg']} deg "
                  "off the command")


def compare(paths):
    print(f"{'test':34} {'ping med/95%/max':>17} {'lost':>4} {'link med/95%/max':>17} {'REST new/kept':>13} {'robot Wi-Fi':>24}")
    for p in paths:
        r = json.load(open(p))
        if "stamp" not in r:
            print(f"{os.path.basename(p)}: not a latency_test result (robot_checks.py latency?)")
            continue

        def three(s):
            return f"{s['median']:.0f}/{s['p95']:.0f}/{s['max']:.0f}" if s and s.get("n") else "-"

        def med(s):
            return f"{s['median']:.0f}" if s and s.get("n") else "-"
        rest, w = r.get("rest") or {}, r.get("robot_wifi") or {}
        wifi = " ".join(x for x in (f"{w['signal_dbm']:.0f}dBm" if "signal_dbm" in w else "",
                                    f"ps {w['power_save']}" if "power_save" in w else "") if x)
        print(f"{(r['stamp'] + ' ' + r.get('label', ''))[:34]:34} {three(r.get('ping')):>17} {(r.get('ping') or {}).get('lost', '-'):>4} "
              f"{three(r.get('link')):>17} {med(rest.get('new_connection')) + '/' + med(rest.get('kept_open')):>13} {wifi:>24}")


def guarded(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except Exception as e:  # noqa: BLE001 - one test failing must not lose the others
        return {"error": f"{type(e).__name__}: {e}"[:200]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--label", default="")
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberry")
    ap.add_argument("--power-save", choices=("on", "off"), help="set the robot's Wi-Fi power saving first")
    ap.add_argument("--relay", type=int, nargs="?", const=25, default=0, metavar="N",
                    help="also time N REST calls through the robot link (default 25; GUI disconnected)")
    ap.add_argument("--move", type=int, nargs="?", const=5, default=0, metavar="N",
                    help="also time command -> servo with N arm moves (default 5; GUI disconnected, robot standing)")
    ap.add_argument("--compare", nargs="+", metavar="JSON")
    args = ap.parse_args()
    if args.compare:
        return compare(args.compare)
    if not args.ip:
        ap.error("--ip is required")
    host, env = split_host(args.ip)[0], dict(os.environ, SSHPASS=args.password)
    ssh = ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8", f"{args.user}@{host}"]
    r = {"stamp": time.strftime("%Y%m%d_%H%M%S"), "label": args.label, "ip": args.ip, "seconds": args.seconds}
    if args.power_save:
        subprocess.run(ssh + [f"export PATH=$PATH:/sbin:/usr/sbin; sudo iw dev wlan0 set power_save {args.power_save} "
                              f"|| sudo iwconfig wlan0 power {args.power_save}"], env=env, timeout=20, capture_output=True)
    r["robot_wifi"], r["laptop_wifi"] = robot_wifi(ssh, env), laptop_wifi(host)
    print(f"testing for {args.seconds:g} s (ping, link, REST at once) ...", flush=True)
    jobs = {"ping": lambda: ping_test(host, args.seconds),
            "link": lambda: link_test(ssh + ["python2 -u -c " + shlex.quote(ECHO)], args.seconds, env=env),
            "rest": lambda: rest_test(args.ip, args.seconds)}
    threads = [threading.Thread(target=lambda k=k, f=f: r.__setitem__(k, guarded(f))) for k, f in jobs.items()]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    r["robot_wifi_after"] = robot_wifi(ssh, env)
    if args.relay or args.move:
        print("through the robot link:" + (f" {args.relay} REST calls" if args.relay else "")
              + (f" moving {MOVE_JOINT} {MOVE_DEG:g} deg and back {args.move} times" if args.move else "") + " ...", flush=True)
        res = guarded(node_tests, host, args.user, args.password, args.relay, args.move,
                      os.path.join(ROOT, "robot_dump", "robot_logs"))
        if "error" in res:  # no link: say so in each part asked for
            res = {k: dict(res) for k, n in (("relay", args.relay), ("move", args.move)) if n}
        r.update(res)
    path = os.path.join(ROOT, "robot_dump", f"latency_test_{r['stamp']}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(r, f, indent=1)
    show(r)
    print(f"saved {os.path.relpath(path, ROOT)}")


if __name__ == "__main__":
    main()
