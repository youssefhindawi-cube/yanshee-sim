#!/usr/bin/env python3
"""Open the Yanshee model in the full MuJoCo viewer (physics running, reset pose).

    python3 scripts/view.py            # free-standing robot on the floor
    python3 scripts/view.py --fixed    # torso fixed in the air, for exploring joints safely

In the right-hand panel, open "Control" to drag each servo (values are radians; degrees = rad * 57.3).
Ctrl+right-drag applies forces to the robot. Backspace (or the Simulation panel's Reset button)
returns the robot to the standing reset pose and holds it there.
"""
import argparse
import os
import sys

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yanshee_sim import SimRobot, sim_extras  # noqa: E402


def reset_hold_callback(bot, key_name="reset"):
    """A mjcb_control callback that turns the viewer's reset into a reset-keyframe reset.

    The viewer's Backspace/Reset calls mj_resetData: time goes back to 0 and every servo target to 0 rad,
    which would fold the robot to its lower joint limits. qpos0 already equals the reset keyframe's
    pose, so restoring the keyframe's servo targets on the next step is enough.
    """
    model = bot.model
    key_ctrl = model.key_ctrl[model.key(key_name).id].copy()
    last = {"t": -1.0}

    def cb(m, d):
        if d.time < last["t"] and not d.ctrl.any():
            d.ctrl[:] = key_ctrl
        last["t"] = d.time
        bot.paint_leds()  # the chest and eye LEDs animate as in the web GUI
    return cb


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixed", action="store_true")
    args = ap.parse_args()
    bot = SimRobot(viewer=False, realtime=False, fixed_base=args.fixed,
                   spec_hook=sim_extras.add_leds)  # loads the model in the reset pose, with the LEDs
    mujoco.set_mjcb_control(reset_hold_callback(bot))
    try:
        mujoco.viewer.launch(bot.model, bot.data)
    finally:
        mujoco.set_mjcb_control(None)


if __name__ == "__main__":
    main()
