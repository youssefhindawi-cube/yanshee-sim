#!/usr/bin/env python3
"""Mirror the simulator and the real robot live.

    python3 scripts/mirror.py from-robot --ip 192.168.3.13            # robot moves -> simulator copies it
    python3 scripts/mirror.py from-robot --ip 192.168.3.13 --limp-arms   # arms go limp: pose them by hand
    python3 scripts/mirror.py to-robot                                  # dry run: prints what would be sent
    python3 scripts/mirror.py to-robot --ip 192.168.3.13               # drag sliders -> robot follows

to-robot: open the "Control" panel on the right of the viewer and drag a servo slider (radians).
Each change is checked (limits, self-collision, speed cap) and sent a few times per second.
Start with "from-robot" on new hardware: it confirms every joint turns the same way in both.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import NAMES, RealRobot, SimRobot  # noqa: E402
from yanshee_sim.mirror import DryRunRobot, RobotToSim, SimToRobot  # noqa: E402

ARMS_AND_HEAD = ["RightShoulderRoll", "RightShoulderFlex", "RightElbowFlex",
                 "LeftShoulderRoll", "LeftShoulderFlex", "LeftElbowFlex", "NeckLR"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["to-robot", "from-robot"])
    ap.add_argument("--ip", help="robot IP; to-robot without --ip is a dry run")
    ap.add_argument("--rate", type=float, default=4.0, help="updates per second (default 4)")
    ap.add_argument("--max-speed", type=float, default=90.0, help="to-robot: joint speed cap, deg/s (default 90)")
    ap.add_argument("--limp-arms", action="store_true",
                    help="from-robot: put arm and neck servos in program (limp) mode so you can move them by hand")
    ap.add_argument("--no-reset", action="store_true", help="to-robot: do not return the robot to reset at the end")
    ap.add_argument("--yes", action="store_true", help="skip the safety confirmation")
    args = ap.parse_args()

    if args.mode == "from-robot" and not args.ip:
        sys.exit("from-robot needs --ip")
    if args.ip and not args.yes:
        input("Robot on a flat surface, clear of edges, emergency stop within reach? Press Enter (Ctrl+C aborts) ")
    robot = RealRobot(args.ip) if args.ip else DryRunRobot()
    if args.ip:
        print(f"connected to {args.ip}, battery {robot.battery}%")

    sim = SimRobot(viewer=True, fixed_base=True, manual=(args.mode == "to-robot"))
    period = 1.0 / args.rate
    limp = False
    try:
        if args.mode == "to-robot":
            link = SimToRobot(sim, robot, max_speed_deg_s=args.max_speed)
            link.start()
            print("simulator now matches the robot; drag sliders in the viewer's Control panel")
            while sim.is_running():
                status, info = link.step()
                if status == "sent":
                    print(f"sent {info[0]} over {info[1]} ms")
                elif status == "blocked":
                    print(f"NOT sent (self-collision): {info}")
                time.sleep(period)
        else:
            if args.limp_arms:
                robot.api.set_servos_mode("program", ARMS_AND_HEAD)
                limp = True
                print("arm and neck servos are limp: move them by hand and watch the simulator")
            link = RobotToSim(sim, robot, period_s=period)
            while sim.is_running():
                angles = link.step()
                print("  ".join(f"{n}={angles[n]}" for n in NAMES), end="\r")
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if limp:
            robot.api.set_servos_mode("work", ARMS_AND_HEAD)
            print("servos back in work mode")
        if args.mode == "to-robot" and args.ip and not args.no_reset:
            print("returning robot to reset")
            robot.reset()
        sim.close()


if __name__ == "__main__":
    main()
