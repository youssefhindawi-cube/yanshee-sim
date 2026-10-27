#!/usr/bin/env python3
"""Web GUI for the simulator and the real Yanshee (docs/ARCHITECTURE.md section 3).

    python3 scripts/web_gui.py                                  # simulator only, http://localhost:8080/
    python3 scripts/web_gui.py --ip 192.168.3.13                # connect the robot; switch modes in the GUI
    python3 scripts/web_gui.py --mode both --ip 192.168.3.13    # start relaying to the sim and the robot
    python3 scripts/web_gui.py --host 127.0.0.1 --port 9000 --fixed   # this machine only, torso in the air
    python3 scripts/web_gui.py --mode robot --ip 127.0.0.1:9191 # against tests/fake_robot.py --port 9191
    python3 scripts/web_gui.py --motions-dir ~/shows           # keep the GUI's motion files elsewhere

Open the printed URL on this laptop, or the LAN one on a tablet in the same network. --host 0.0.0.0 (the
default) lets anyone on the network drive the robot; use --host 127.0.0.1 to keep it to this machine.
Ctrl+C stops everything (activity, robot motions, renderer, simulator) and exits.
Exit code 2: the port is in use, or --mode robot/both could not connect to --ip (with --mode sim a failed
--ip only warns and the GUI starts in sim mode).
"""
import argparse
import errno
import fcntl
import os
import socket
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def lan_ips():
    """IPv4 addresses of this machine's network interfaces (no traffic sent; loopback left out)."""
    ips = []
    try:
        names = [n for _, n in socket.if_nameindex()]
    except OSError:
        names = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for name in names:
            try:  # SIOCGIFADDR
                raw = fcntl.ioctl(s.fileno(), 0x8915, struct.pack("256s", name[:15].encode()))
            except OSError:
                continue
            ip = socket.inet_ntoa(raw[20:24])
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    return ips


def port_free(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
        except OSError as e:
            return e.errno != errno.EADDRINUSE and e.errno != errno.EACCES
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default 0.0.0.0: the whole LAN)")
    ap.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    ap.add_argument("--mode", choices=("sim", "robot", "both"), default="sim", help="starting relay mode (default sim)")
    ap.add_argument("--ip", help="robot IP (or IP:PORT) to connect at start; needed for --mode robot/both")
    ap.add_argument("--fixed", action="store_true", help="hold the simulated torso in the air")
    ap.add_argument("--viewer", action="store_true", help="also open the MuJoCo viewer window (needs a display)")
    ap.add_argument("--motions-dir", help="folder for the GUI's motion files (default: motions/)")
    ap.add_argument("--log-level", default="info", choices=("critical", "error", "warning", "info", "debug"),
                    help="server log level (default info)")
    args = ap.parse_args()
    if args.mode != "sim" and not args.ip:
        ap.error(f"--mode {args.mode} needs --ip (the robot's address)")
    if not 0 < args.port < 65536:
        ap.error("--port must be 1-65535")
    if not port_free(args.host, args.port):
        print(f"error: port {args.port} on {args.host} is in use (another web_gui?); try --port {args.port + 1}",
              file=sys.stderr)
        return 2
    viewer = args.viewer and os.environ.get("YANSHEE_SIM_HEADLESS") != "1"

    import logging
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format="%(levelname)s %(name)s: %(message)s")
    from yanshee_sim.controller import Controller, ModeError
    from yanshee_sim.web.server import create_app, run

    print("starting the simulator ...", flush=True)
    try:
        ctl = Controller(start_mode=args.mode, sim_viewer=viewer, fixed_base=args.fixed, ip=args.ip,
                         motions_dir=args.motions_dir)
    except ModeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    try:
        if args.ip and ctl.robot is None:
            print(f"warning: could not connect to the robot at {args.ip} (see the GUI log); running in sim mode",
                  file=sys.stderr)
        app = create_app(ctl, close_controller=True)
        urls = [f"http://localhost:{args.port}/"] if args.host in ("0.0.0.0", "127.0.0.1", "localhost") else []
        if args.host == "0.0.0.0":
            urls += [f"http://{ip}:{args.port}/   (LAN)" for ip in lan_ips()]
        elif args.host not in ("127.0.0.1", "localhost"):
            urls.append(f"http://{args.host}:{args.port}/")
        print(f"Yanshee web GUI ({ctl.mode} mode) at:")
        for u in urls:
            print("  " + u)
        print("Ctrl+C to stop.", flush=True)
        run(app, host=args.host, port=args.port, log_level=args.log_level)
    finally:
        ctl.close()
    print("stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
