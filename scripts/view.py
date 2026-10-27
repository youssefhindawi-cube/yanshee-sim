#!/usr/bin/env python3
"""Open the Yanshee model in the full MuJoCo viewer (physics running, reset pose).

    python3 scripts/view.py            # free-standing robot on the floor
    python3 scripts/view.py --fixed    # torso fixed in the air, for exploring joints safely

In the right-hand panel, open "Control" to drag each servo (values are radians; degrees = rad * 57.3).
Ctrl+right-drag applies forces to the robot; Backspace resets.
"""
import argparse
import os
import sys

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import SimRobot  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixed", action="store_true")
    args = ap.parse_args()
    bot = SimRobot(viewer=False, realtime=False, fixed_base=args.fixed)  # loads the model in the reset pose
    mujoco.viewer.launch(bot.model, bot.data)


if __name__ == "__main__":
    main()
